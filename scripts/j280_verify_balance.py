#!/usr/bin/env python3
"""Verify balance v4 offline, optionally build and archive. No board access."""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

from j280_balance_protocol import FrameStream, crc16_ccitt
FRAME_STREAM = FrameStream
from j280_verify_measurement import _run, check_timing_report
from j280_verify_state import check_reset_timing

ROOT = Path(__file__).resolve().parents[1]
# Firmware identity; scripts/j280_verify_swing.py replaces these on a private copy.
NAME = 'j280_balance'
RTL = ROOT / 'rtl/j280_balance'
SIM = ROOT / 'sim/j280_balance'
TEST_PATTERNS = ('test_j280_balance*.py',)
EVIDENCE_ENV = 'J280_BALANCE_EVIDENCE_DIR'
ARCHIVE = ROOT / 'docs/data/balance-firmware-2026-10-18'
TEST_TIMEOUT_S = 2400
SHARED = [ROOT / 'rtl/j280_measure' / name for name in
          ('j280_adc_capture.v', 'j280_uart_tx.v')]
SHARED += [ROOT / 'rtl/j280_state' / name for name in
           ('j280_signed_divider.v', 'j280_raw_estimator.v', 'j280_state_packet_tx.v')]
SHARED += [ROOT / 'rtl/j280_motor_test' / name for name in ('j280_uart_rx.v', 'j280_local_inputs.v')]
# Firmware-defined configurations (v6 built-in frames) that a scenario may load
# without a matching CONFIG in commands.hex: config_id -> CRC.
EXTRA_CONFIGS = {}
SCENARIOS = {'stop': ('SAFE', 'stop_key', {'CAPTURE', 'BALANCE'}),
             'angle': ('FAULT', 'angle_limit', {'CAPTURE', 'BALANCE'})}
EVIDENCE_FILES = ('commands.hex', 'wire.bin', 'pins.csv', 'snapshots.csv', 'events.csv',
                  'result.json', 'compile.log', 'simulation.log')
PYTHON = ('j280_measurement.py', 'j280_motor_test.py', 'j280_balance_design.py',
          'j280_balance_math.py', 'j280_balance_protocol.py', 'j280_balance.py',
          'j280_verify_balance.py', 'j280_verify_measurement.py', 'j280_verify_state.py')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_files():
    return sorted(set(SHARED + list(RTL.glob('*')) + list(SIM.glob('*.v'))
                      + [ROOT / 'tests' / 'test_j280_motor_test_rtl.py']
                      + [t for pattern in TEST_PATTERNS for t in (ROOT / 'tests').glob(pattern)]
                      + [ROOT / 'scripts' / name for name in PYTHON]))


def fingerprints(paths):
    return {p.relative_to(ROOT).as_posix(): sha256(p) for p in paths if p.is_file()}


def check_evidence(directory):
    """Re-decode the simulated UART pin bytes; a passing test count is not evidence."""
    evidence = {}
    try:
        for name, (final_state, final_reason, required_states) in SCENARIOS.items():
            folder = directory / name
            for file in EVIDENCE_FILES:
                path = folder / file
                if not path.is_file():
                    raise ValueError(f'missing {name} integration evidence: {file}')
                if file in ('wire.bin', 'pins.csv', 'snapshots.csv') and not path.stat().st_size:
                    raise ValueError(f'empty {name} integration evidence: {file}')
            result = json.loads((folder / 'result.json').read_text(encoding='utf-8'))
            wire = (folder / 'wire.bin').read_bytes()
            if result.get('wire_sha256') != hashlib.sha256(wire).hexdigest():
                raise ValueError(f'{name} result does not describe its UART bytes')
            stream = FRAME_STREAM()
            rows = stream.feed(wire)
            if (not rows or len(rows) != result['frames'] or stream.buffered_bytes
                    or any(stream.stats[key] for key in
                           ('frames_invalid', 'discarded_bytes', 'packet_seq_missed', 'duplicates',
                            'out_of_order', 'resets', 'reset_or_reorder', 'continuity_errors'))):
                raise ValueError(f'unclean {name} UART evidence: {stream.stats}')
            if [r['packet_seq'] for r in rows] != list(range(len(rows))):
                raise ValueError('packet sequence is not continuous from reset')
            if [r['tick_us'] for r in rows] != [2000 + 10000 * i for i in range(len(rows))]:
                raise ValueError('telemetry timing is not 100 Hz')
            commands = [bytes.fromhex(line)[::-1] for line in
                        (folder / 'commands.hex').read_text(encoding='ascii').splitlines() if line]
            for c in commands:
                if len(c) != 192 or crc16_ccitt(c[2:190]) != int.from_bytes(c[190:], 'little'):
                    raise ValueError('integration command fixture CRC is invalid')
            expected = {**EXTRA_CONFIGS, **{int.from_bytes(c[4:6], 'little'): int.from_bytes(c[190:], 'little')
                                             for c in commands if c[3] == 1}}
            for row in rows:
                if row['config_loaded'] and expected.get(row['config_id']) != row['config_crc']:
                    raise ValueError('configuration echo differs from submitted command')
            states = {r['state'] for r in rows}
            if not required_states <= states or not any(r['output_enabled'] for r in rows):
                raise ValueError(f'{name} scenario did not engage the controller')
            last = rows[-1]
            if ((last['state'], last['stop_reason']) != (final_state, final_reason)
                    or last['output_enabled'] or last['config_loaded']):
                raise ValueError(f'{name} scenario ended {last["state"]}/{last["stop_reason"]}')
            evidence[name] = {'frames': len(rows), 'stats': stream.stats,
                              'states': sorted(states), 'final': [final_state, final_reason],
                              'evidence_sha256': {f: sha256(folder / f) for f in EVIDENCE_FILES}}
    except (OSError, KeyError, IndexError, json.JSONDecodeError) as error:
        raise ValueError(f'missing/incomplete integration evidence: {error}') from error
    return evidence


def stage_firmware(destination):
    destination.mkdir()
    paths = SHARED + [p for p in RTL.iterdir() if p.suffix in ('.v', '.cst', '.sdc', '.tcl', '.gprj')]
    for p in paths:
        shutil.copy2(p, destination / p.name)
    # Flat ASCII build directory: point the GUI project at the same flat copies as run.tcl.
    project = destination / f'{NAME}.gprj'
    text = project.read_text(encoding='utf-8')
    for folder in ('../j280_measure/', '../j280_state/', '../j280_motor_test/', '../j280_balance/'):
        text = text.replace(folder, '')
    project.write_text(text, encoding='utf-8')


def run_gowin(gowin, build, timeout=3600, idle_s=90):
    """Run gw_sh; returns (console text, terminated_after_outputs).

    On this PC gw_sh has three times stayed alive after writing every output.
    It is ended only once the place/route log has reported the final timing
    file and nothing in impl/pnr has changed for idle_s seconds; the caller
    still checks the bitstream and timing report as for a normal exit.
    """
    pnr = build / 'impl/pnr'
    log = build / 'build-console.log'
    marker = f'{NAME}.tr.html" completed'
    start = time.monotonic()
    stuck = False
    with log.open('wb') as sink:
        proc = subprocess.Popen([str(gowin), 'run.tcl'], cwd=build, stdout=sink, stderr=subprocess.STDOUT)
        while proc.poll() is None:
            time.sleep(2)
            if time.monotonic() - start > timeout:
                proc.kill()
                raise subprocess.TimeoutExpired(str(gowin), timeout)
            pnr_log = pnr / f'{NAME}.log'
            if pnr_log.is_file() and marker in pnr_log.read_text(encoding='utf-8', errors='replace'):
                if time.time() - max(p.stat().st_mtime for p in pnr.iterdir()) > idle_s:
                    proc.kill()
                    proc.wait()
                    stuck = True
    text = log.read_text(encoding='utf-8', errors='replace')
    if not stuck and proc.returncode:
        raise RuntimeError(f'exit {proc.returncode}: {gowin}\n{text[-6000:]}')
    pnr_text = (pnr / f'{NAME}.log').read_text(encoding='utf-8', errors='replace')
    if re.search(r'\bERROR\b', text + pnr_text):
        raise RuntimeError('Gowin reported ERROR')
    return text, stuck


def archive_verified_run(run, destination):
    """Seal an already verified run without repeating unchanged tests."""
    try:
        result = json.loads((run / 'verification.json').read_text(encoding='utf-8'))
        if (not result.get('build_performed') or not result.get('python_tests_passed')
                or result.get('board_accessed') is not False
                or result['source_sha256'] != fingerprints(source_files())):
            raise ValueError('verified inputs differ from current source or build is incomplete')
        for path, digest in result['source_sha256'].items():
            if sha256(run / 'source' / path) != digest:
                raise ValueError(f'verified source snapshot differs: {path}')
        build = run / 'gowin'
        for name, digest in result['staged_firmware_sha256'].items():
            if sha256(build / name) != digest:
                raise ValueError(f'compiled source differs: {name}')
        fs = build / f'impl/pnr/{NAME}.fs'
        if sha256(fs) != result['bitstream_sha256']:
            raise ValueError('verified bitstream differs')
        if check_evidence(run / 'integration') != result['integration']:
            raise ValueError('integration evidence differs from the verified manifest')
        timing = build / f'impl/pnr/{NAME}_tr_content.html'
        text = timing.read_text(encoding='utf-8')
        check_timing_report(text)
        check_reset_timing(text)
        if destination.exists():
            raise ValueError('archive already exists')
        destination.mkdir(parents=True)
        shutil.copytree(run / 'source', destination / 'source')
        shutil.copytree(run / 'integration', destination / 'integration')
        for src, name in ((fs, f'{NAME}.fs'), (timing, 'timing-report.html'),
                          (build / f'impl/pnr/{NAME}.rpt.txt', 'pnr-report.txt'),
                          (build / 'build-console.log', 'build-console.log'),
                          (run / 'python-tests.log', 'python-tests.log'),
                          (run / 'verification.json', 'verification.json')):
            shutil.copy2(src, destination / name)
        if sha256(destination / f'{NAME}.fs') != result['bitstream_sha256']:
            raise ValueError('archived bitstream differs')
        for path, digest in result['source_sha256'].items():
            if sha256(destination / 'source' / path) != digest:
                raise ValueError(f'archived source differs: {path}')
        if check_evidence(destination / 'integration') != result['integration']:
            raise ValueError('archived integration evidence differs')
        return result
    except (OSError, KeyError, json.JSONDecodeError) as error:
        raise ValueError(f'incomplete verified run: {error}') from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    local = Path(os.environ.get('LOCALAPPDATA', str(Path.home())))
    parser.add_argument('--build', action='store_true')
    parser.add_argument('--archive', action='store_true')
    parser.add_argument('--archive-dir', type=Path, default=ARCHIVE)
    parser.add_argument('--build-dir', type=Path, default=local / f'J280Build/{NAME[5:]}-verification')
    parser.add_argument('--iverilog-bin', type=Path,
                        default=local / 'J280Tools/iverilog-msys2/ucrt64/bin')
    parser.add_argument('--gowin', type=Path,
                        default=Path('C:/Gowin/Gowin_V1.9.12.03_x64/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe'))
    args = parser.parse_args(argv)
    try:
        if args.archive and not args.build:
            raise ValueError('--archive requires --build')
        if args.archive and args.archive_dir.exists():
            raise ValueError('archive directory already exists; choose a new --archive-dir')
        if not str(args.build_dir.resolve()).isascii():
            raise ValueError('build path must be ASCII')
        for name in ('iverilog.exe', 'vvp.exe'):
            if not (args.iverilog_bin / name).is_file():
                raise ValueError(f'missing simulator: {name}')
        args.build_dir.mkdir(parents=True, exist_ok=True)
        run = Path(tempfile.mkdtemp(prefix='run-', dir=args.build_dir))
        inputs = source_files()
        before = fingerprints(inputs)
        for path in inputs:
            target = run / 'source' / path.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
        env = dict(os.environ, PATH=str(args.iverilog_bin.resolve()) + os.pathsep + os.environ.get('PATH', ''),
                   **{EVIDENCE_ENV: str(run / 'integration')})
        expected_tools = Path(env.get('LOCALAPPDATA', '')) / 'J280Tools/iverilog-msys2/ucrt64/bin'
        if expected_tools.resolve() != args.iverilog_bin.resolve():
            raise ValueError('set LOCALAPPDATA to the tool root used by RTL harnesses')
        print('RUN complete Python/HDL regression; evidence:', run, flush=True)
        unit = _run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_j280_*.py'],
                    ROOT, env=env, log=run / 'python-tests.log', timeout=TEST_TIMEOUT_S)
        match = re.search(r'Ran (\d+) tests?', unit)
        if not match or not re.search(r'(?m)^OK\s*$', unit):
            raise ValueError('test runner supplied no successful test result')
        evidence = check_evidence(run / 'integration')
        print('PASS tests:', match.group(1), '; UART bytes re-decoded for every top scenario', flush=True)
        result = {'checked_at': datetime.now(timezone(timedelta(hours=8))).isoformat(),
                  'run_directory': str(run.resolve()), 'python_tests_passed': int(match.group(1)),
                  'integration': evidence, 'source_sha256': before,
                  'build_performed': args.build, 'board_accessed': False}
        if args.build:
            build = run / 'gowin'
            stage_firmware(build)
            result['staged_firmware_sha256'] = {p.name: sha256(p) for p in sorted(build.iterdir())}
            print('RUN Gowin synthesis, place/route and timing', flush=True)
            console, stuck = run_gowin(args.gowin.resolve(), build)
            result['gowin_terminated_after_outputs'] = stuck
            fs = build / f'impl/pnr/{NAME}.fs'
            if not fs.is_file() or not fs.stat().st_size:
                raise ValueError('Gowin produced no bitstream')
            timing_text = (build / f'impl/pnr/{NAME}_tr_content.html').read_text(encoding='utf-8')
            result['timing'] = check_timing_report(timing_text)
            result['timing']['minimum_reset_slack_ns'] = check_reset_timing(timing_text)
            result['build_warnings'] = [s.strip() for s in console.splitlines() if re.search(r'\bWARN\b', s)]
            result['bitstream'] = str(fs.resolve())
            result['bitstream_sha256'] = sha256(fs)
            print('PASS build and Setup/Hold/Recovery/Removal; warnings:', len(result['build_warnings']), flush=True)
        if fingerprints(source_files()) != before:
            raise ValueError('source changed during verification; re-run on final files')
        (run / 'verification.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        if args.archive:
            archive_verified_run(run, args.archive_dir)
            print('PASS archived:', args.archive_dir, flush=True)
        print('PASS verification manifest:', run / 'verification.json', flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
