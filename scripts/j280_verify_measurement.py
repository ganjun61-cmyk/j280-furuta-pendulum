#!/usr/bin/env python3
"""Offline verification and optional Gowin build. Never accesses a board/serial port."""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import html as html_module
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def _report_table(text, anchor):
    match = re.search(r'name="' + re.escape(anchor) +
                      r'".*?</h[23]>.*?<table[^>]*>(.*?)</table>', text, re.S)
    if not match:
        raise ValueError(f"missing timing table: {anchor}")
    result = []
    for row in re.findall(r'<tr[^>]*>(.*?)</tr>', match.group(1), re.S):
        cells = [html_module.unescape(re.sub(r'<[^>]*>', '', cell)).strip()
                 for cell in re.findall(r'<td[^>]*>(.*?)</td>', row, re.S)]
        if cells:
            result.append(cells)
    return result


def check_timing_report(text):
    """Reject missing clocks or failed timing, even if gw_sh returned zero."""
    required = {'clk_50m': 50.0, 'adc_5m': 5.0}
    frequencies = {}
    for row in _report_table(text, 'Max_Frequency_Report'):
        if len(row) >= 4:
            frequencies[row[1]] = (float(row[2].split('(')[0]),
                                   float(row[3].split('(')[0]))
    for name, constraint in required.items():
        if name not in frequencies:
            raise ValueError(f"missing clock: {name}")
        actual_constraint, actual_fmax = frequencies[name]
        if abs(actual_constraint - constraint) > 0.001 or actual_fmax < constraint:
            raise ValueError(f"clock frequency requirement not met: {name}")
    tns = {}
    for row in _report_table(text, 'Total_Negative_Slack_Report'):
        if len(row) >= 4:
            tns[row[0] + '/' + row[1]] = float(row[2])
    for name in required:
        for kind in ('Setup', 'Hold'):
            key = name + '/' + kind
            if key not in tns or tns[key] < 0:
                raise ValueError(f"missing or negative slack: {key}")
    summary = dict(row[:2] for row in _report_table(text, 'STA_Tool_Run_Summary') if len(row)>=2)
    violations = {}
    for kind in ('Setup', 'Hold'):
        label = f'Numbers of {kind} Violated Endpoints'
        if label not in summary or int(summary[label]) != 0:
            raise ValueError(f'missing or nonzero global violations: {kind}')
        violations[kind] = int(summary[label])
    minimum_slacks = {}
    for kind in ('Setup', 'Hold'):
        detail = _report_table(text, kind + '_Slack_Table')
        slacks = [float(row[1]) for row in detail if len(row)>=2]
        if not slacks or min(slacks) < 0:
            raise ValueError(f'missing or negative detailed slack: {kind}')
        minimum_slacks[kind] = min(slacks)
    return {'constraints_mhz': required,
            'reported_fmax_mhz': {name: frequencies[name][1] for name in required},
            'tns_ns': tns, 'global_violated_endpoints': violations,
            'minimum_reported_slack_ns': minimum_slacks}


def _run(command, cwd, env=None, log=None, timeout=120):
    result = subprocess.run([str(arg) for arg in command], cwd=cwd, env=env,
                            capture_output=True, timeout=timeout)
    text = (result.stdout + result.stderr).decode('utf-8', errors='replace')
    if log:
        Path(log).write_text(text, encoding='utf-8')
    if result.returncode:
        raise RuntimeError(f"exit {result.returncode}: {command[0]}\n{text[-6000:]}")
    return text


def _check_wire_fixture(path):
    from j280_measurement import FrameStream
    parser = FrameStream()
    rows = parser.feed(path.read_bytes())
    expected = {
        'seq': list(range(5)),
        'tick_us': [2000, 4000, 6000, 8000, 10000],
        'adc_raw10': [513, 513, 1023, 1023, 17],
        'adc_valid': [True, True, False, False, True],
        'enc_count': [4] * 5,
    }
    if len(rows) != 5 or parser.stats['frames_invalid'] or parser.buffered_bytes:
        raise ValueError(f"invalid top UART fixture: {parser.stats}")
    for field, values in expected.items():
        if [row[field] for row in rows] != values:
            raise ValueError(f"top UART mismatch: {field}")
    for row in rows:
        if any(row[name] for name in ('output_enabled', 'pwm_count', 'pwm_period_count', 'motor_mode')):
            raise ValueError("top UART reports active motor output")
        if not row['adc_new'] or not 1 <= row['adc_age_us'] <= 2:
            raise ValueError("top ADC freshness mismatch")
    return parser.stats


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    local = Path(os.environ.get('LOCALAPPDATA', str(Path.home())))
    cli.add_argument('--iverilog-bin', type=Path,
                     default=local/'J280Tools/iverilog-msys2/ucrt64/bin')
    cli.add_argument('--build-dir', type=Path, default=local/'J280Build/passive-verification')
    cli.add_argument('--build', action='store_true', help='also build with Gowin; no programming')
    cli.add_argument('--gowin', type=Path,
                     default=Path('C:/Gowin/Gowin_V1.9.12.03_x64/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe'))
    args = cli.parse_args(argv)
    args.iverilog_bin = args.iverilog_bin.resolve()
    args.build_dir = args.build_dir.resolve()
    args.gowin = args.gowin.resolve()
    try:
        if not str(args.build_dir.resolve()).isascii():
            raise ValueError('build directory must use an ASCII path for Gowin')
        for executable in ('iverilog.exe', 'vvp.exe'):
            if not (args.iverilog_bin/executable).is_file():
                raise ValueError(f'missing simulator: {args.iverilog_bin/executable}')
        args.build_dir.mkdir(parents=True, exist_ok=True)
        run_root = Path(tempfile.mkdtemp(prefix='run-', dir=args.build_dir))
        firmware = run_root/'source'
        firmware.mkdir()
        for source in (ROOT/'rtl/j280_measure').iterdir():
            if source.suffix in ('.v', '.cst', '.sdc', '.tcl'):
                shutil.copy2(source, firmware/source.name)
        sim = run_root/'sim'
        sim.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, PATH=str(args.iverilog_bin) + os.pathsep + os.environ.get('PATH', ''))
        unit_text = _run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_j280_*.py'],
                         ROOT, log=sim/'python-tests.log')
        unit_count = re.search(r'Ran (\d+) tests?', unit_text)
        if not unit_count or int(unit_count.group(1)) == 0:
            raise ValueError('no Python tests discovered')
        sources = sorted(firmware.glob('*.v'))
        for source in sources + sorted((ROOT/'sim/j280_measure').glob('*.v')):
            shutil.copy2(source, sim/source.name)
        source_names = [source.name for source in sources]
        jobs = [('tb_adc_capture', []), ('tb_encoder', []), ('tb_frame_tx', []),
                ('tb_uart_tx', []), ('tb_uart_tx', ['-Ptb_uart_tx.CLOCK_HZ=1000',
                 '-Ptb_uart_tx.BAUD=60', '-Ptb_uart_tx.DIVISOR=17']),
                ('tb_uart_tx', ['-Ptb_uart_tx.CLOCK_HZ=1000', '-Ptb_uart_tx.BAUD=1000',
                 '-Ptb_uart_tx.DIVISOR=1']), ('tb_measure_top', []), ('tb_output_age', [])]
        for index, (top, overrides) in enumerate(jobs):
            target = f'job-{index}.vvp'
            _run([args.iverilog_bin/'iverilog.exe', '-g2001', '-Wall', '-s', top,
                  *overrides, '-o', target, top+'.v', *source_names], sim, env,
                 sim/f'job-{index}-compile.log')
            plusarg = ['+OUT=top-uart.bin'] if top == 'tb_measure_top' else []
            text = _run([args.iverilog_bin/'vvp.exe', target, *plusarg], sim, env,
                        sim/f'job-{index}-run.log')
            if 'PASS ' + top not in text:
                raise ValueError(f'no explicit PASS: {top}')
            print('PASS', top, ' '.join(overrides))
        stats = _check_wire_fixture(sim/'top-uart.bin')
        print('PASS UART pin -> Python decoder: 5 frames')
        result = {'checked_at': datetime.now(timezone(timedelta(hours=8))).isoformat(),
                  'run_directory': str(run_root),
                  'python_tests_passed': int(unit_count.group(1)),
                  'hdl_jobs_passed': len(jobs), 'wire_fixture_stats': stats,
                  'build_performed': args.build, 'source_sha256': {
                      source.name: hashlib.sha256(source.read_bytes()).hexdigest()
                      for source in firmware.iterdir()
                      if source.suffix in ('.v', '.cst', '.sdc', '.tcl')}}
        if args.build:
            build = run_root/'gowin'
            build.mkdir(exist_ok=True)
            for source in firmware.iterdir():
                if source.is_file():
                    shutil.copy2(source, build/source.name)
            _run([args.gowin, 'run.tcl'], build, log=build/'build-console.log')
            fs = build/'impl/pnr/j280_measure.fs'
            if not fs.is_file() or fs.stat().st_size == 0:
                raise ValueError('Gowin produced no bitstream')
            timing = build/'impl/pnr/j280_measure_tr_content.html'
            result['timing'] = check_timing_report(timing.read_text(encoding='utf-8'))
            result['bitstream'] = str(fs)
            result['bitstream_sha256'] = hashlib.sha256(fs.read_bytes()).hexdigest()
            print('PASS Gowin build and both clocks Setup/Hold timing (conditional ADC input budget)')
        result_text = json.dumps(result, indent=2)
        (run_root/'verification.json').write_text(result_text, encoding='utf-8')
        pending = args.build_dir/(run_root.name + '.json')
        pending.write_text(result_text, encoding='utf-8')
        pending.replace(args.build_dir/'verification.json')
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
