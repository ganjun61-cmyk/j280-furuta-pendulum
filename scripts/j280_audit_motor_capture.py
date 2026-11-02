#!/usr/bin/env python3
"""Offline v3 wire-quality audit. Never opens a serial port or sends commands.

Exit 0 means captured bytes pass the selected wire profile and minimum frame
count. It does not establish experiment completion, sensor calibration or
permission to apply motor power. Diagnostic125 is explicitly a separate profile.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from j280_motor_test import FrameStream, FIELDS, PAYLOAD, crc16_ccitt

PROFILES = {'production500': 1, 'diagnostic125': 4}
MASK32 = 0xffffffff


def audit(data, profile='production500', min_frames=1, expect_safe=False):
    if profile not in PROFILES:
        raise ValueError('unknown wire profile')
    if type(min_frames) is not int or min_frames < 1:
        raise ValueError('min_frames must be a positive integer')
    stride = PROFILES[profile]
    stream = FrameStream()
    frames = missed = intentional = bad_phase = bad_stride = unsafe = 0
    prefix = None
    previous = None
    for start in range(0, len(data), 65536):
        for row in stream.feed(data[start:start+65536]):
            frames += 1
            if prefix is None:
                payload = PAYLOAD.pack(*(row[key] for key in FIELDS))
                first = b'\xa5\x5a'+payload+crc16_ccitt(payload).to_bytes(2,'little')
                prefix = data.find(first)
            if stride > 1 and (row['seq']+1) % stride:
                bad_phase += 1
            if previous is not None:
                delta = (row['seq']-previous['seq']) & MASK32
                if 0 < delta < 0x80000000:
                    if delta % stride:
                        bad_stride += 1
                    else:
                        periods = delta // stride
                        missed += periods-1
                        intentional += periods*(stride-1)
            if (row['state'] != 'SAFE' or row['output_enabled'] or row['config_loaded']):
                unsafe += 1
            previous = row
    stats = dict(stream.stats)
    prefix = len(data) if prefix is None else prefix
    interior = max(0, stats['discarded_bytes']-prefix)
    issues = []
    if frames < min_frames:
        issues.append(f'only {frames} valid frames; require {min_frames}')
    for key in ('frames_invalid','seq_duplicates','seq_out_of_order','resets',
                'uncertain_rollbacks','continuity_errors'):
        if stats[key]:
            issues.append(f'{key}={stats[key]}')
    if prefix > 39:
        issues.append(f'prefix exceeds one partial frame: {prefix} bytes')
    if interior:
        issues.append(f'discarded bytes after first valid frame={interior}')
    if bad_phase or bad_stride:
        issues.append(f'wire profile mismatch: phase={bad_phase}, stride={bad_stride}')
    if missed:
        issues.append(f'actual packets missed={missed}')
    if expect_safe and unsafe:
        issues.append(f'rows not SAFE/unconfigured/output-off={unsafe}')
    final = None if previous is None else {key: previous[key] for key in
        ('seq','tick_us','state','stop_reason','output_enabled','config_loaded','fault_bits')}
    return dict(passed=not issues, scope='captured wire quality only', profile=profile,
                diagnostic_only=(stride != 1), min_frames=min_frames,
                expect_safe=expect_safe, frames=frames, bytes=len(data),
                sha256=hashlib.sha256(data).hexdigest(), prefix_bytes=prefix,
                incomplete_tail_bytes=stream.buffered_bytes,
                interior_discarded_bytes=interior, actual_packets_missed=missed,
                intentional_skipped_snapshots=intentional, stats=stats,
                issues=issues, final=final)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--profile', choices=PROFILES, default='production500')
    parser.add_argument('--min-frames', type=int, default=1)
    parser.add_argument('--expect-safe', action='store_true',
                        help='also require every decoded row SAFE, unconfigured and output off')
    args = parser.parse_args(argv)
    try:
        if args.report is not None:
            if args.report.resolve() == args.input.resolve():
                raise ValueError('report aliases input')
            if args.report.exists() or args.report.is_symlink():
                raise FileExistsError('report already exists')
        result = audit(args.input.read_bytes(), args.profile, args.min_frames, args.expect_safe)
        result['source'] = str(args.input.resolve())
        if args.report is not None:
            with args.report.open('x',encoding='utf-8') as target:
                json.dump(result,target,indent=2,ensure_ascii=False)
                target.write('\n')
        print(json.dumps(result,ensure_ascii=False))
        return 0 if result['passed'] else 2
    except (OSError, ValueError) as error:
        print(json.dumps(dict(passed=False,error=str(error))),file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
