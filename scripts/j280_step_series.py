#!/usr/bin/env python3
"""Open-loop step responses on the arm: several duties, both directions.

Every run drives toward the arm center (count 0 at firmware load), so the arm
stays within the travel limit plus coast of the center. The first run at each
duty and direction is a short probe; later pulses are lengthened so the drive
travel approaches a target below the travel limit. Before loading a config the
script waits for the pendulum to settle; the operator then arms with DIP 1 and
presses SW1 after the double beep. Stops on any end other than complete.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

import j280_audit_motor_capture as audit
from j280_ladder import beep, run_step
import j280_pulse_response as pulse
import j280_step_fit as stepfit

SCRIPTS = Path(__file__).resolve().parent
TRAVEL_LIMIT = 130
SPEED_LIMIT = 5000
PROBE_MS = 100
MIN_MS, MAX_MS = 60, 600
MAX_START = 120
SETTLE_ADC_P2P = 7
SETTLE_TRIES = 20


def target_travel(start):
    """Cross the center by a margin, staying well inside the travel limit."""
    return max(50, min(TRAVEL_LIMIT - 45, abs(start) + 45))


def toward_center(position):
    return 'in1_high' if position > 0 else 'in2_high'  # in1_high lowers the count


def plan_pulse(previous, target):
    """previous: None or (pulse_ms, |drive counts|, |end speed count/s|)."""
    if previous is None:
        return PROBE_MS
    ms, travel, speed = previous
    proposed = ms + (target - travel) * 1000 / max(speed, 50.0)
    return int(max(MIN_MS, min(MAX_MS, 2 * ms, proposed)))


def write_config(path, run_id, duty, direction, pulse_ms, target, start):
    config = {
        'schema': 'j280-motor-test-config-v1',
        'purpose': ('12V ON open-loop step response for motor identification; drive toward the '
                    'arm center from the current position. Base clamped, >=90 deg cable clearance '
                    'each side of count 0 confirmed by user 2026-10-17.'),
        'power': 'powered',
        'values': {'config_id': run_id, 'direction': direction, 'duty_count': duty * 25,
                   'pulse_ms': pulse_ms, 'travel_limit_count': TRAVEL_LIMIT,
                   'speed_limit_count_s': SPEED_LIMIT, 'keepalive_timeout_ms': 200},
        'sources': {
            'config_id': f'Step-series run id {run_id}',
            'direction': (f'Toward center from count {start}; BIN1 lowers the count, BIN2 raises it '
                          '(2026-10-17 ladder; BIN2 seen clockwise from above)'),
            'duty_count': (f'{duty}% slow-decay PWM, above the 2026-10-17 breakaway (BIN1 11-14%, '
                           'BIN2 14-17%) and within the user-approved 40% ceiling; no current sensing'),
            'pulse_ms': (f'{pulse_ms} ms planned for ~{target} count drive travel from the previous '
                         f'pulse at this duty/direction (probe {PROBE_MS} ms otherwise)'),
            'travel_limit_count': (f'{TRAVEL_LIMIT} count (~45 deg) from RUN entry; start within '
                                   f'{MAX_START} count of center on the opposite side, so the arm '
                                   'stays inside the >=260 count clearance'),
            'speed_limit_count_s': f'{SPEED_LIMIT} count/s, about half the datasheet no-load speed',
            'keepalive_timeout_ms': '200 ms board watchdog with 50 ms host keepalive'}}
    path.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
    return config


def wait_settled(port, folder, tag):
    """1 s read-only captures until the pendulum and arm are still; returns the last row."""
    folder.mkdir(exist_ok=True)
    last = None
    for attempt in range(SETTLE_TRIES):
        out = folder / f'{tag}-{attempt:02d}.csv'
        subprocess.run([sys.executable, str(SCRIPTS / 'j280_motor_test.py'), 'capture', '--port', port,
                        '--seconds', '1', '--output', str(out)], capture_output=True, cwd=SCRIPTS)
        with out.open(newline='', encoding='utf-8') as source:
            rows = list(csv.DictReader(source))
        if len(rows) < 300:
            continue
        last = rows[-1]
        if last['state'] == 'FAULT' or last['fault_bits'] != '0':
            return last, False
        adc = [int(r['adc_raw10']) for r in rows]
        if max(adc) - min(adc) <= SETTLE_ADC_P2P and len({r['enc_count'] for r in rows}) == 1:
            return last, True
    return last, False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--port', required=True)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--duties', type=int, nargs='+', required=True)
    parser.add_argument('--repeats', type=int, default=2, help='full runs per duty and direction')
    parser.add_argument('--first-id', type=int, required=True)
    args = parser.parse_args(argv)
    summary_path = args.folder / f'steps-{args.tag}-summary.json'
    if summary_path.exists():
        parser.error(f'{summary_path.name} exists; choose a new --tag')
    records, run_id, stop_why = [], args.first_id, None
    for duty in args.duties:
        if stop_why:
            break
        full = {'in1_high': 0, 'in2_high': 0}
        previous = {}
        wire_failures = 0
        for _ in range(4 * (args.repeats + 1)):
            if min(full.values()) >= args.repeats:
                break
            last, settled = wait_settled(args.port, args.folder / 'settle', f'{args.tag}-{run_id}')
            if last is None or last['state'] != 'SAFE' or last['fault_bits'] != '0':
                stop_why = f'board not ready: {None if last is None else (last["state"], last["fault_bits"])}'
                break
            start = int(last['enc_count'])
            if abs(start) > MAX_START:
                stop_why = f'arm at {start}, beyond {MAX_START} from center'
                break
            direction = toward_center(start)
            target = target_travel(start)
            pulse_ms = plan_pulse(previous.get(direction), target)
            config_path = args.folder / f'step-{args.tag}-{run_id}-config.json'
            write_config(config_path, run_id, duty, direction, pulse_ms, target, start)
            print(f'run {run_id}: {direction} {duty}% {pulse_ms} ms from {start} '
                  f'(target {target}, settled={settled})', flush=True)
            stem = args.folder / f'step-{args.tag}-{run_id}'
            _, result, report, out = run_step(args.port, config_path, stem, 300)
            record = dict(run_id=run_id, duty_percent=duty, direction=direction, pulse_ms=pulse_ms,
                          start=start, target=target, pendulum_settled=settled, result=result,
                          wire_ok=report['passed'], wire_issues=report['issues'])
            records.append(record)
            run_id += 1
            if result is None or not result.get('started') or result.get('end_reason') != 'complete':
                stop_why = 'run ended: ' + (str(result.get('end_reason') or result.get('phase'))
                                            if result else 'tool error')
                break
            rows = pulse.load_rows(out['.csv'])
            samples = stepfit.drive_samples(rows)
            record['response'] = pulse.analyze(rows)
            record['fit'] = stepfit.fit(samples)
            record['end_speed_count_s'] = round(stepfit.end_speed(samples), 1)
            print(f"  drive={record['response']['drive_counts']} coast={record['response']['coast_counts']} "
                  f"v_end={record['end_speed_count_s']} fit={record['fit']} wire_ok={report['passed']}",
                  flush=True)
            if not report['passed']:
                wire_failures += 1
                if wire_failures > 2:
                    stop_why = 'records incomplete three times'
                    break
                continue
            if direction in previous:
                full[direction] += 1
            previous[direction] = (pulse_ms, abs(record['response']['drive_counts']),
                                   abs(record['end_speed_count_s']))
    beep([(600, 600)])
    summary_path.write_text(json.dumps(dict(stop=stop_why, runs=records), indent=2,
                                       ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(dict(summary=summary_path.name, stop=stop_why, runs=len(records))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
