#!/usr/bin/env python3
"""Run a duty ladder of pre-written powered configs, one board-started pulse each.

For every step: load the config with j280_motor_test.py run, beep twice once the
board echoes it (the operator then arms with DIP 1 and presses SW1), then audit
the wire and summarize the pulse. Stops at the first breakaway (motion late in the pulse or large travel),
on any fault or operator stop, or after one failed repeat of an incomplete record.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import time

import j280_audit_motor_capture as audit
import j280_pulse_response as pulse

SCRIPTS = Path(__file__).resolve().parent
MAX_OFFSET_COUNTS = 120


def decide(result, wire_ok, response, reference_enc, retried):
    """Return ('next'|'repeat'|'stop', why) for one completed run."""
    if result is None:
        return 'stop', 'tool error'
    if not result.get('started'):
        return 'stop', 'not started (' + str(result.get('phase')) + ')'
    if result.get('end_reason') != 'complete':
        return 'stop', 'ended by ' + str(result.get('end_reason'))
    if response.get('breakaway'):
        return 'stop', 'breakaway'
    if abs(response['enc_end'] - reference_enc) > MAX_OFFSET_COUNTS:
        return 'stop', 'arm offset from reference too large'
    if not wire_ok:
        return ('stop', 'record incomplete twice') if retried else ('repeat', 'record incomplete')
    return 'next', 'no motion'


def beep(pattern):
    try:
        import winsound
        for frequency, ms in pattern:
            winsound.Beep(frequency, ms)
            time.sleep(0.08)
    except (ImportError, RuntimeError):
        print('\a', end='', flush=True)


def _last_row(path):
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except OSError:
        return None
    if len(lines) < 3:
        return None
    return next(csv.DictReader([lines[0], lines[-2]]), None)


def run_step(port, config_path, stem, wait_start_s):
    config = json.loads(config_path.read_text(encoding='utf-8'))
    out = {k: stem.with_name(stem.name + k) for k in ('.csv', '.bin', '-diagnostics.json', '-stderr.log')}
    command = [sys.executable, str(SCRIPTS / 'j280_motor_test.py'), 'run', '--port', port,
               '--config', str(config_path), '--confirm-powered', '--wait-start-s', str(wait_start_s),
               '--observe-s', '3', '--output', str(out['.csv']), '--raw-output', str(out['.bin'])]
    with out['-diagnostics.json'].open('x') as stdout, out['-stderr.log'].open('x') as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr, cwd=SCRIPTS)
        echoed = False
        while process.poll() is None:
            row = _last_row(out['.csv'])
            if (not echoed and row and row.get('config_loaded') == 'True'
                    and int(row['config_id']) == config['values']['config_id']):
                echoed = True
                print(f"  loaded config {config['values']['config_id']} "
                      f"({config['values']['direction']}, {config['values']['duty_count']/25:g}%): "
                      'DIP ON -> SW1 -> DIP OFF', flush=True)
                beep([(1200, 180), (1200, 180)])
            time.sleep(0.2)
    text = out['-diagnostics.json'].read_text(encoding='utf-8').strip()
    summary = json.loads(text) if text else None
    result = None if summary is None else summary.get('result')
    report = audit.audit(out['.bin'].read_bytes())
    return config, result, report, out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--port', required=True)
    parser.add_argument('--config-dir', type=Path, required=True)
    parser.add_argument('--direction', choices=('in1_high', 'in2_high'), required=True)
    parser.add_argument('--start-step', type=int, default=1)
    parser.add_argument('--last-step', type=int, default=6)
    parser.add_argument('--reference-enc', type=int, default=0)
    parser.add_argument('--tag', required=True, help='suffix keeping output names unique')
    parser.add_argument('--wait-start-s', type=float, default=300)
    args = parser.parse_args(argv)
    summary_path = args.config_dir / f'ladder-{args.direction}{args.tag}-summary.json'
    if summary_path.exists():
        parser.error(f'{summary_path.name} exists; choose a new --tag')
    steps = []
    step, retried = args.start_step, False
    while step <= args.last_step:
        config_path = args.config_dir / f'ladder-{args.direction}-{step:02d}-config.json'
        attempt = 'r' if retried else ''
        stem = args.config_dir / f'ladder-{args.direction}-{step:02d}{args.tag}{attempt}'
        print(f'step {step}: waiting for board echo', flush=True)
        config, result, report, out = run_step(args.port, config_path, stem, args.wait_start_s)
        response = pulse.analyze(pulse.load_rows(out['.csv']), args.reference_enc)
        action, why = decide(result, report['passed'], response, args.reference_enc, retried)
        record = dict(step=step, files=str(stem.name), result=result, wire_ok=report['passed'],
                      wire_issues=report['issues'], response=response, action=action, why=why)
        steps.append(record)
        print(f"  -> {why}; total={response.get('total_counts')} late={response.get('late_drive_counts')} "
              f"coast={response.get('coast_counts')} peak={response.get('peak_speed_count_s')} "
              f"wire_ok={report['passed']}", flush=True)
        if action == 'stop':
            break
        if action == 'repeat':
            retried = True
            continue
        step, retried = step + 1, False
    beep([(600, 600)])
    summary_path.write_text(json.dumps(steps, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(dict(summary=summary_path.name, last=steps[-1] if steps else None),
                     ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
