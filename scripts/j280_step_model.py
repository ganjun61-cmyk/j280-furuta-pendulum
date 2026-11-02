#!/usr/bin/env python3
"""Fit one first-order motor model per direction to a series of step records.

Per direction: v_inf(duty) = k * (duty - d0), shared tau, over every run's
drive segment (positions relative to the enable frame). For each tau on a grid
the two linear parameters come from least squares. Coast after the output is
cut is summarized separately (high-impedance Stop, no electrical braking).
"""
import argparse
import json
import math
from pathlib import Path
import sys

from j280_pulse_response import load_rows
from j280_step_fit import drive_samples

TAU_GRID_MS = [4 * 1.03 ** i for i in range(150)]


def _shape(t, tau):
    return (t - tau * (1 - math.exp(-t / tau))) / 1000


def fit_direction(runs):
    """runs: [(duty_percent, [(t_ms, x)])] for one direction."""
    best = None
    for tau in TAU_GRID_MS:
        saa = sab = sbb = sax = sbx = 0.0
        for duty, samples in runs:
            for t, x in samples:
                f = _shape(t, tau)
                a, b = duty * f, f
                saa += a * a; sab += a * b; sbb += b * b; sax += a * x; sbx += b * x
        det = saa * sbb - sab * sab
        if det <= 0:
            continue
        k = (sax * sbb - sbx * sab) / det
        c = (saa * sbx - sab * sax) / det
        sse = sum((x - (k * duty + c) * _shape(t, tau)) ** 2 for duty, s in runs for t, x in s)
        if best is None or sse < best[0]:
            best = (sse, tau, k, c)
    sse, tau, k, c = best
    count = sum(len(s) for _, s in runs)
    per_run = [math.sqrt(sum((x - (k * d + c) * _shape(t, tau)) ** 2 for t, x in s) / len(s))
               for d, s in runs]
    return dict(k_count_s_per_percent=round(k, 2), deadband_percent=round(-c / k, 2),
                tau_ms=round(tau, 1), rms_counts=round(math.sqrt(sse / count), 3),
                worst_run_rms_counts=round(max(per_run), 3), runs=len(runs), samples=count,
                tau_at_grid_edge=tau in (TAU_GRID_MS[0], TAU_GRID_MS[-1]))


def coast(rows):
    off = next(i for i, r in enumerate(rows) if i and rows[i - 1]['on'] and not r['on'])
    before = next(r for r in rows[:off] if r['t'] >= rows[off]['t'] - 20000)
    v_cut = (rows[off]['enc'] - before['enc']) * 1e6 / (rows[off]['t'] - before['t'])
    distance = rows[-1]['enc'] - rows[off]['enc']
    return v_cut, distance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('summary', type=Path, help='steps-*-summary.json from j280_step_series.py')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output is not None and args.output.exists():
            raise FileExistsError(f'{args.output} exists')
        summary = json.loads(args.summary.read_text(encoding='utf-8'))
        folder = args.summary.parent
        groups, coasts = {}, {}
        for run in summary['runs']:
            if not run.get('wire_ok') or (run.get('result') or {}).get('end_reason') != 'complete':
                continue
            rows = load_rows(folder / f"step-{args.summary.stem.split('-')[1]}-{run['run_id']}.csv")
            groups.setdefault(run['direction'], []).append((run['duty_percent'], drive_samples(rows)))
            coasts.setdefault(run['direction'], []).append(coast(rows))
        result = dict(source=args.summary.name, model='v_inf = k*(duty-d0), first-order tau, td=0',
                      directions={})
        for direction, runs in sorted(groups.items()):
            fit = fit_direction(runs)
            pairs = coasts[direction]
            # Pure viscous coast: distance = v_cut * tau_coast.
            ratio = [d / v for v, d in pairs if abs(v) >= 200]
            fit['coast_tau_ms_estimate'] = (round(1000 * sum(ratio) / len(ratio), 1) if ratio else None)
            fit['coast_pairs_v_cut_count_s_distance'] = [[round(v), d] for v, d in pairs]
            result['directions'][direction] = fit
        text = json.dumps(result, indent=2)
        if args.output is not None:
            args.output.write_text(text + '\n', encoding='utf-8')
        print(text)
    except (OSError, ValueError, KeyError, StopIteration) as error:
        print(json.dumps(dict(error=str(error) or type(error).__name__)), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
