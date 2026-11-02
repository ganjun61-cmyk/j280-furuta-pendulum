#!/usr/bin/env python3
"""Fit first-order-plus-dead-time velocity to one pulse's encoder trace.

Model while the output is enabled, t from the enable frame:
    x(t) = v_inf * ((t-td) - tau*(1-exp(-(t-td)/tau))) for t >= td, else 0.
Positions are counts relative to the enable frame. Pure-Python grid search for
td and tau, closed-form least squares for v_inf. Offline only.
"""
import argparse
import json
import math
from pathlib import Path
import sys

from j280_pulse_response import load_rows

TD_GRID_MS = [2 * i for i in range(31)]
TAU_GRID_MS = [4 * 1.07 ** i for i in range(70)]
END_SPEED_MS = 20


def drive_samples(rows):
    on = next(i for i, r in enumerate(rows) if r['on'])
    off = next(i for i in range(on, len(rows)) if not rows[i]['on'])
    t0, x0 = rows[on]['t'], rows[on]['enc']
    return [((r['t'] - t0) / 1000, r['enc'] - x0) for r in rows[on:off + 1]]


def fit(samples):
    """samples: [(t_ms, x_counts)] from output enable; returns the best fit."""
    best = None
    for td in TD_GRID_MS:
        for tau in TAU_GRID_MS:
            f = []
            for t, _ in samples:
                s = t - td
                f.append(0.0 if s <= 0 else (s - tau * (1 - math.exp(-s / tau))) / 1000)
            ff = sum(v * v for v in f)
            if ff == 0:
                continue
            v_inf = sum(fi * x for fi, (_, x) in zip(f, samples)) / ff
            sse = sum((x - v_inf * fi) ** 2 for fi, (_, x) in zip(f, samples))
            if best is None or sse < best[0]:
                best = (sse, td, tau, v_inf)
    if best is None:
        raise ValueError('pulse too short to fit')
    sse, td, tau, v_inf = best
    duration = samples[-1][0]
    reached = 1 - math.exp(-max(0.0, duration - td) / tau)
    return dict(v_inf_count_s=round(v_inf, 1), tau_ms=round(tau, 1), dead_time_ms=td,
                rms_counts=round(math.sqrt(sse / len(samples)), 3), samples=len(samples),
                drive_ms=duration, fraction_of_v_inf_reached=round(reached, 3),
                tau_at_grid_edge=tau in (TAU_GRID_MS[0], TAU_GRID_MS[-1]))


def end_speed(samples):
    """Mean speed over the last END_SPEED_MS of the drive, count/s."""
    t_end, x_end = samples[-1]
    start = next(s for s in samples if s[0] >= t_end - END_SPEED_MS)
    return 0.0 if t_end == start[0] else (x_end - start[1]) * 1000 / (t_end - start[0])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('csv', type=Path)
    args = parser.parse_args(argv)
    try:
        samples = drive_samples(load_rows(args.csv))
        result = fit(samples)
        result['end_speed_count_s'] = round(end_speed(samples), 1)
        print(json.dumps(result))
    except (OSError, ValueError, StopIteration, KeyError) as error:
        print(json.dumps(dict(error=str(error) or type(error).__name__)), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
