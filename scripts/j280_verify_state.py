#!/usr/bin/env python3
"""Offline verification and optional Gowin build of the passive state firmware.
Never accesses a board or serial port."""
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'tests'))
from j280_verify_measurement import check_timing_report, _report_table, _run  # noqa: E402
from j280_state_telemetry import FrameStream  # noqa: E402

SHARED = ('j280_adc_capture.v', 'j280_encoder.v', 'j280_uart_tx.v')
FIRMWARE_SUFFIXES = ('.v', '.cst', '.sdc', '.tcl')
ARCHIVE = ROOT/'docs/data/fpga-state-2026-10-16'


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_reset_timing(text):
    result = {}
    for kind in ('Recovery', 'Removal'):
        slacks = [float(row[1]) for row in _report_table(text, kind + '_Slack_Table') if len(row) >= 2]
        if not slacks or min(slacks) < 0:
            raise ValueError(f'missing or negative {kind} slack')
        result[kind] = min(slacks)
    return result


def check_packet_wire(wire, frame):
    if wire != frame:
        raise ValueError('packet TX UART bytes differ from the reference frame')
    rows = FrameStream().feed(wire)
    if len(rows) != 1:
        raise ValueError('packet TX UART bytes do not decode to exactly one frame')


def check_top_wire(wire):
    parser = FrameStream()
    rows = parser.feed(wire)
    s = parser.stats
    if len(rows) != 9 or parser.buffered_bytes or any(
            s[k] for k in ('frames_invalid', 'discarded_bytes', 'packet_seq_missed',
                           'continuity_errors', 'reset_or_reorder')):
        raise ValueError(f'top UART stream not clean: rows={len(rows)} {s}')
    if [r['packet_seq'] for r in rows] != list(range(9)):
        raise ValueError('top packet_seq mismatch')
    if [r['tick_us_unwrapped'] for r in rows] != [2000 + 10000*i for i in range(9)]:
        raise ValueError('top telemetry tick mismatch (expected 100 Hz)')
    if any(r['controller_ready'] for r in rows):
        raise ValueError('passive firmware reports controller_ready')
    if any(r['reason'] == 'input_overrun' for r in rows):
        raise ValueError('estimator overrun at top level')
    if (rows[0]['status'], rows[0]['reason']) != ('warming', 'segment_start'):
        raise ValueError('top first packet is not a warming segment start')
    for r in rows[3:6]:
        if ((r['status'], r['segment_id'], r['encoder_velocity_count_s'],
             r['adc_velocity_code_s']) != ('ready', 1, 2000, 1000)):
            raise ValueError(f'top steady velocity mismatch at packet {r["packet_seq"]}')
    r = rows[6]
    if ((r['status'], r['reason'], r['fault_bits']) != ('invalid', 'invalid_flags', 1)
            or r['encoder_velocity_count_s'] is not None):
        raise ValueError('top ADC over-range packet not fail-closed')
    if (rows[7]['segment_id'], rows[7]['status']) != (2, 'warming'):
        raise ValueError('top segment did not restart after fault')
    r = rows[8]
    if ((r['status'], r['encoder_velocity_count_s'], r['adc_velocity_code_s'],
         r['enc_count']) != ('ready', 0, 0, 80)):
        raise ValueError('top final static packet mismatch')
    return s


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    local = Path(os.environ.get('LOCALAPPDATA', str(Path.home())))
    cli.add_argument('--iverilog-bin', type=Path,
                     default=local/'J280Tools/iverilog-msys2/ucrt64/bin')
    cli.add_argument('--build-dir', type=Path, default=local/'J280Build/state-verification')
    cli.add_argument('--build', action='store_true', help='also build with Gowin; no programming')
    cli.add_argument('--archive', action='store_true',
                     help=f'copy evidence and bitstream to {ARCHIVE.relative_to(ROOT)}')
    cli.add_argument('--gowin', type=Path,
                     default=Path('C:/Gowin/Gowin_V1.9.12.03_x64/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe'))
    args = cli.parse_args(argv)
    args.iverilog_bin = args.iverilog_bin.resolve()
    args.build_dir = args.build_dir.resolve()
    args.gowin = args.gowin.resolve()
    try:
        if args.archive and not args.build:
            raise ValueError('--archive requires --build')
        if not str(args.build_dir).isascii():
            raise ValueError('build directory must use an ASCII path for Gowin')
        for executable in ('iverilog.exe', 'vvp.exe'):
            if not (args.iverilog_bin/executable).is_file():
                raise ValueError(f'missing simulator: {args.iverilog_bin/executable}')
        args.build_dir.mkdir(parents=True, exist_ok=True)
        run_root = Path(tempfile.mkdtemp(prefix='run-', dir=args.build_dir))
        firmware = run_root/'source'
        firmware.mkdir()
        for name in SHARED:
            shutil.copy2(ROOT/'rtl/j280_measure'/name, firmware/name)
        for source in (ROOT/'rtl/j280_state').iterdir():
            if source.suffix in FIRMWARE_SUFFIXES:
                shutil.copy2(source, firmware/source.name)
        sim = run_root/'sim'
        sim.mkdir()
        env = dict(os.environ, PATH=str(args.iverilog_bin) + os.pathsep + os.environ.get('PATH', ''))

        unit_text = _run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests',
                          '-p', 'test_j280_*.py'], ROOT, log=sim/'python-tests.log', timeout=900)
        unit_count = re.search(r'Ran (\d+) tests?', unit_text)
        if not unit_count or int(unit_count.group(1)) == 0:
            raise ValueError('no Python tests discovered')
        print('PASS Python tests:', unit_count.group(1))

        sources = sorted(firmware.glob('*.v'))
        for source in sources:
            shutil.copy2(source, sim/source.name)
        for name in ('tb_adc_capture.v', 'tb_encoder.v', 'tb_uart_tx.v'):
            shutil.copy2(ROOT/'sim/j280_measure'/name, sim/name)
        for source in (ROOT/'sim/j280_state').glob('*.v'):
            shutil.copy2(source, sim/source.name)
        from test_j280_state_telemetry import fixture
        frame = fixture()
        (sim/'payload.hex').write_text(frame[2:-2][::-1].hex() + '\n', encoding='ascii')

        source_names = [source.name for source in sources]
        jobs = ['tb_adc_capture', 'tb_encoder', 'tb_uart_tx',
                'tb_state_packet_tx', 'tb_state_top']
        for index, top in enumerate(jobs):
            target = f'job-{index}.vvp'
            _run([args.iverilog_bin/'iverilog.exe', '-g2001', '-Wall', '-s', top,
                  '-o', target, top+'.v', *source_names], sim, env,
                 sim/f'job-{index}-compile.log')
            text = _run([args.iverilog_bin/'vvp.exe', target], sim, env,
                        sim/f'job-{index}-run.log', timeout=300)
            if 'PASS ' + top not in text:
                raise ValueError(f'no explicit PASS: {top}')
            if (sim/'wire.bin').is_file():
                (sim/'wire.bin').replace(sim/f'{top}-uart.bin')
            print('PASS', top)
        check_packet_wire((sim/'tb_state_packet_tx-uart.bin').read_bytes(), frame)
        stats = check_top_wire((sim/'tb_state_top-uart.bin').read_bytes())
        print('PASS UART pin -> Python v2 decoder: packet TX 1 frame, top 9 frames')

        result = {'checked_at': datetime.now(timezone(timedelta(hours=8))).isoformat(),
                  'run_directory': str(run_root),
                  'python_tests_passed': int(unit_count.group(1)),
                  'hdl_jobs_passed': jobs, 'top_wire_stats': stats,
                  'top_wire_sha256': _sha256(sim/'tb_state_top-uart.bin'),
                  'build_performed': args.build, 'source_sha256': {
                      source.name: _sha256(source) for source in sorted(firmware.iterdir())}}
        if args.build:
            build = run_root/'gowin'
            build.mkdir()
            for source in firmware.iterdir():
                shutil.copy2(source, build/source.name)
            _run([args.gowin, 'run.tcl'], build, log=build/'build-console.log', timeout=1800)
            fs = build/'impl/pnr/j280_state.fs'
            if not fs.is_file() or fs.stat().st_size == 0:
                raise ValueError('Gowin produced no bitstream')
            timing = build/'impl/pnr/j280_state_tr_content.html'
            timing_text = timing.read_text(encoding='utf-8')
            result['timing'] = check_timing_report(timing_text)
            result['timing']['minimum_reset_slack_ns'] = check_reset_timing(timing_text)
            result['bitstream'] = str(fs)
            result['bitstream_sha256'] = _sha256(fs)
            print('PASS Gowin build and both clocks Setup/Hold timing (conditional ADC input budget)')
        result_text = json.dumps(result, indent=2)
        (run_root/'verification.json').write_text(result_text, encoding='utf-8')
        pending = args.build_dir/(run_root.name + '.json')
        pending.write_text(result_text, encoding='utf-8')
        pending.replace(args.build_dir/'verification.json')
        if args.archive:
            ARCHIVE.mkdir(parents=True, exist_ok=True)
            pnr = build/'impl/pnr'
            copies = {pnr/'j280_state.fs': 'j280_state.fs',
                      pnr/'j280_state.rpt.txt': 'pnr-report.txt',
                      timing: 'timing-report.html',
                      build/'build-console.log': 'build-console.log',
                      sim/'python-tests.log': 'python-tests.log',
                      sim/'tb_state_top-uart.bin': 'simulation-top-uart.bin',
                      sim/'tb_state_packet_tx-uart.bin': 'simulation-packet-uart.bin'}
            for index in range(len(jobs)):
                copies[sim/f'job-{index}-run.log'] = f'job-{index}-run.log'
            for src, name in copies.items():
                if src.is_file():
                    shutil.copy2(src, ARCHIVE/name)
            (ARCHIVE/'verification.json').write_text(result_text, encoding='utf-8')
            if _sha256(ARCHIVE/'j280_state.fs') != result['bitstream_sha256']:
                raise ValueError('archived bitstream fingerprint mismatch')
            print('PASS archived to', ARCHIVE.relative_to(ROOT))
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
