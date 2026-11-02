#!/usr/bin/env python3
"""Summarize one bounded motor-test v3 pulse from a decoded run CSV.

Reports the commanded output, encoder motion during the drive and after the
output turned off (coast), first-motion delay and peak speed. Offline only;
wire completeness is checked separately by j280_audit_motor_capture.py.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

MOTION_COUNTS = 3
SPEED_WINDOW = 5  # frames, 10 ms at 500 Hz
# Gear backlash alone moved the motor-side encoder ~4 counts and then stalled
# (2026-10-17, 8%); breakaway needs motion late in the pulse or a larger travel.
LATE_WINDOW_US = 100000
SUSTAINED_LATE_COUNTS = 2
BREAKAWAY_COUNTS = 15


def _bool(value):
    return value in (True, 'True', 'true', '1', 1)


def load_rows(path):
    with Path(path).open(newline='', encoding='utf-8') as source:
        rows = list(csv.DictReader(source))
    return [dict(t=int(r['tick_us_unwrapped']), enc=int(r['enc_count']), adc=int(r['adc_raw10']),
                 on=_bool(r['output_enabled']), pwm=int(r['pwm_count']), in1=_bool(r['in1']),
                 in2=_bool(r['in2']), state=r['state'], reason=r['stop_reason'])
            for r in rows]


def analyze(rows, reference_enc=None):
    if not rows:
        raise ValueError('no rows')
    on = next((i for i, r in enumerate(rows) if r['on']), None)
    result = dict(frames=len(rows), output_seen=on is not None, enc_start=rows[0]['enc'],
                  enc_end=rows[-1]['enc'], final_state=rows[-1]['state'])
    if reference_enc is not None:
        result['offset_from_reference'] = rows[-1]['enc'] - reference_enc
    if on is None:
        result.update(moved=rows[-1]['enc'] != rows[0]['enc'])
        return result
    off = next((i for i in range(on, len(rows)) if not rows[i]['on']), None)
    if off is None:
        raise ValueError('output still enabled at end of record')
    drive = rows[on:off]
    if len({(r['pwm'], r['in1'], r['in2']) for r in drive}) != 1:
        raise ValueError('drive settings changed during the pulse')
    t_on, t_off, enc_on, enc_off = rows[on]['t'], rows[off]['t'], rows[on]['enc'], rows[off]['enc']
    first = next((r for r in rows[on:] if r['enc'] != enc_on), None)
    last_change = max((i for i in range(off, len(rows)) if rows[i]['enc'] != rows[i - 1]['enc']),
                      default=None)
    window = rows[on:]
    speeds = [abs(b['enc'] - a['enc']) * 1e6 / (b['t'] - a['t'])
              for a, b in zip(window, window[SPEED_WINDOW:])]
    after_on = rows[on:]
    total = rows[-1]['enc'] - enc_on
    late_start = next(r for r in rows[on:off + 1] if r['t'] >= t_off - LATE_WINDOW_US)
    late = enc_off - late_start['enc']
    result.update(
        direction='in1_high' if drive[0]['in1'] else 'in2_high', pwm_count=drive[0]['pwm'],
        duty_percent=round(drive[0]['pwm'] / 25, 2), output_ms=(t_off - t_on) / 1000,
        stop_reason=next((r['reason'] for r in rows[off:] if r['reason'] != 'none'), 'none'),
        enc_at_on=enc_on, drive_counts=enc_off - enc_on, coast_counts=rows[-1]['enc'] - enc_off,
        total_counts=total, moved=abs(total) >= MOTION_COUNTS,
        late_drive_counts=late,
        breakaway=abs(late) >= SUSTAINED_LATE_COUNTS or abs(total) >= BREAKAWAY_COUNTS,
        first_motion_ms=None if first is None else (first['t'] - t_on) / 1000,
        settle_ms_after_off=None if last_change is None else (rows[last_change]['t'] - t_off) / 1000,
        peak_speed_count_s=round(max(speeds), 1) if speeds else 0.0,
        max_excursion_counts=max(abs(r['enc'] - enc_on) for r in after_on),
        adc_range=[min(r['adc'] for r in after_on), max(r['adc'] for r in after_on)])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('csv', type=Path)
    parser.add_argument('--reference-enc', type=int,
                        help='encoder count of the session reference position')
    args = parser.parse_args(argv)
    try:
        print(json.dumps(analyze(load_rows(args.csv), args.reference_enc), ensure_ascii=False))
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps(dict(error=str(error))), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
