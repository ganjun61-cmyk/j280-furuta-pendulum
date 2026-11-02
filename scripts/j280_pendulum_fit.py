#!/usr/bin/env python3
"""Fit hanging-pendulum dynamics driven by the measured arm motion.

phi = pendulum angle from hanging (rad, ADC via the provisional phone slope),
theta = arm angle (rad, nominal 1040 count/rev):
    phi'' = beta cos(phi) theta'' - w0^2 sin(phi) - c phi' - mu sgn(phi')
            + sin(phi) cos(phi) theta'^2
The simulation never differentiates theta twice: with w = phi' - beta cos(phi) theta',
    phi += w dt + beta cos(phi) dtheta.
Parameters beta (signed; ADC and encoder signs are absorbed), w0, c, mu are fitted
by Nelder-Mead over every run's window from 0.3 s before the pulse to 3 s after.
"""
import argparse
import json
import math
from pathlib import Path
import sys

from j280_pulse_response import load_rows

COUNT_RAD = 2 * math.pi / 1040
ADC_CODE_PER_RAD = 2.8331868545 * 180 / math.pi
SAMPLE_PERIOD_US = 2000
DT = SAMPLE_PERIOD_US / 1e6
SUBSTEPS = 2
PRE_FRAMES, POST_FRAMES, SPEED_HALF = 150, 1500, 5
NAMES = ('beta', 'w0', 'c', 'mu')


def prepare(rows):
    """Validate all t timestamps (unwrapped us), then return one run's window.

    Requires finite numeric timestamps with exactly 2000 us between rows.
    Returns theta (rad), smoothed theta' (rad/s), and measured phi (rad).
    """
    previous = None
    for i, row in enumerate(rows):
        timestamp = row.get('t')
        if (isinstance(timestamp, bool) or not isinstance(timestamp, (int, float))
                or not math.isfinite(timestamp)):
            raise ValueError(f'row {i}: timestamp t must be finite numeric microseconds')
        if previous is not None:
            delta = timestamp - previous
            if delta <= 0:
                raise ValueError(f'row {i}: timestamps t must be strictly increasing')
            if delta != SAMPLE_PERIOD_US:
                raise ValueError(f'row {i}: expected {SAMPLE_PERIOD_US} us timestamp cadence; '
                                 f'got {delta} us')
        previous = timestamp
    on = next(i for i, r in enumerate(rows) if r['on'])
    off = next(i for i in range(on, len(rows)) if not rows[i]['on'])
    start, end = max(0, on - PRE_FRAMES), min(len(rows), off + POST_FRAMES)
    rest = rows[max(0, on - 100):on]
    adc0 = sum(r['adc'] for r in rest) / len(rest)
    theta = [r['enc'] * COUNT_RAD for r in rows[start:end]]
    n = len(theta)
    speed = []
    for k in range(n):
        a, b = max(0, k - SPEED_HALF), min(n - 1, k + SPEED_HALF)
        speed.append(0.0 if a == b else (theta[b] - theta[a]) / ((b - a) * DT))
    phi = [(r['adc'] - adc0) / ADC_CODE_PER_RAD for r in rows[start:end]]
    return dict(theta=theta, speed=speed, phi=phi)


def simulate(run, params):
    beta, w0, c, mu = params
    w0sq = w0 * w0
    theta, speed = run['theta'], run['speed']
    phi, w = run['phi'][0], 0.0
    out = [phi]
    h = DT / SUBSTEPS
    for k in range(len(theta) - 1):
        dtheta = (theta[k + 1] - theta[k]) / SUBSTEPS
        thd = speed[k]
        for _ in range(SUBSTEPS):
            s, co = math.sin(phi), math.cos(phi)
            phid = w + beta * co * thd
            acc = (-w0sq * s - c * phid - mu * math.tanh(phid / 0.02)
                   + s * co * thd * thd + beta * s * phid * thd)
            w += h * acc
            phi += h * w + beta * co * dtheta
        out.append(phi)
    return out


def cost(runs, params):
    return sum((a - b) ** 2 for run in runs for a, b in zip(simulate(run, params), run['phi']))


def rms(runs, params):
    n = sum(len(r['phi']) for r in runs)
    return math.sqrt(cost(runs, params) / n)


def nelder_mead(f, x0, steps, iterations=300, tolerance=1e-10):
    points = [list(x0)] + [[x + (s if i == j else 0) for j, x in enumerate(x0)] for i, s in enumerate(steps)]
    values = [f(p) for p in points]
    for _ in range(iterations):
        order = sorted(range(len(points)), key=values.__getitem__)
        points, values = [points[i] for i in order], [values[i] for i in order]
        if values[-1] - values[0] <= tolerance * (abs(values[0]) + 1e-30):
            break
        centroid = [sum(p[j] for p in points[:-1]) / (len(points) - 1) for j in range(len(x0))]
        worst = points[-1]
        reflect = [c + (c - w) for c, w in zip(centroid, worst)]
        fr = f(reflect)
        if fr < values[0]:
            expand = [c + 2 * (c - w) for c, w in zip(centroid, worst)]
            fe = f(expand)
            points[-1], values[-1] = (expand, fe) if fe < fr else (reflect, fr)
        elif fr < values[-2]:
            points[-1], values[-1] = reflect, fr
        else:
            contract = [c + 0.5 * (w - c) for c, w in zip(centroid, worst)]
            fc = f(contract)
            if fc < values[-1]:
                points[-1], values[-1] = contract, fc
            else:
                best = points[0]
                points = [best] + [[b + 0.5 * (p - b) for b, p in zip(best, q)] for q in points[1:]]
                values = [values[0]] + [f(p) for p in points[1:]]
    i = min(range(len(points)), key=values.__getitem__)
    return points[i], values[i]


def fit(runs, initial=(1.5, 9.7, 0.3, 0.5)):
    if not runs:
        raise ValueError('fit runs must not be empty')

    def objective(p):
        beta, w0, c, mu = p
        if w0 <= 0 or c < 0 or mu < 0:
            return float('inf')
        return cost(runs, p)
    params, _ = nelder_mead(objective, initial, (0.3, 0.5, 0.3, 0.3))
    return params


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--fit-runs', type=int, nargs='+', required=True)
    parser.add_argument('--check-runs', type=int, nargs='*', default=[])
    parser.add_argument('--tag', default='a')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        for name, ids in (('fit', args.fit_runs), ('check', args.check_runs)):
            if any(not 1 <= run_id <= 65535 for run_id in ids):
                raise ValueError(f'{name} run IDs must be integers in 1..65535')
            if len(ids) != len(set(ids)):
                raise ValueError(f'{name} run IDs contain duplicates')
        overlap = sorted(set(args.fit_runs) & set(args.check_runs))
        if overlap:
            raise ValueError(f'fit/check run IDs overlap: {overlap}')
        if args.output is not None and args.output.exists():
            raise FileExistsError(f'{args.output} exists')
        load = lambda ids: [prepare(load_rows(args.folder / f'step-{args.tag}-{i}.csv')) for i in ids]
        fit_runs, check_runs = load(args.fit_runs), load(args.check_runs)
        params = fit(fit_runs)
        to_code = ADC_CODE_PER_RAD
        result = dict(model=__doc__.split('\n')[2:7], fit_runs=args.fit_runs, check_runs=args.check_runs,
                      params=dict(zip(NAMES, [round(p, 4) for p in params])),
                      fit_rms_adc_code=round(rms(fit_runs, params) * to_code, 3),
                      check_rms_adc_code=(round(rms(check_runs, params) * to_code, 3) if check_runs else None),
                      check_per_run_rms_adc_code=[round(rms([r], params) * to_code, 3) for r in check_runs],
                      no_coupling_check_rms_adc_code=(round(rms(check_runs, (0.0,) + tuple(params[1:])) * to_code, 3)
                                                      if check_runs else None),
                      adc_code_per_rad=round(to_code, 3))
        text = json.dumps(result, indent=2)
        if args.output is not None:
            args.output.write_text(text + '\n', encoding='utf-8')
        print(text)
    except (OSError, ValueError, StopIteration, ZeroDivisionError) as error:
        print(json.dumps(dict(error=str(error) or type(error).__name__)), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
