"""Summarise v6 balance captures for the alpha-rate filter A/B.

Metrics are taken from BALANCE frames after 10 s that are settled and not
moving. u_linear is Q16.16 compare units (2500 = 100% duty); alpha is ADC
codes from the configured upright zero. Run:

    python scripts/j280_filter_ab.py docs/data/filter-ab-2026-10-20/*.csv
"""
import argparse
import csv
import math
import statistics as st
from pathlib import Path

SATURATE_COMPARE = 1000   # max_compare in the filter A/B profiles


def load(path):
    rows = [r for r in csv.DictReader(open(path, encoding="utf-8")) if r["state"] == "BALANCE"]
    if not rows:
        raise SystemExit(f"{path}: no BALANCE frames")
    t0 = int(rows[0]["tick_us_unwrapped"])
    out = []
    for r in rows:
        t = (int(r["tick_us_unwrapped"]) - t0) / 1e6
        if t <= 10:
            continue
        zero = int(r["upright_zero_q16"]) >> 16
        out.append(dict(t=t, alpha=int(r["adc_raw10"]) - zero,
                        arm=int(r["enc_count"]) - int(r["theta_ref"]),
                        u=int(r["u_linear_q16"]) / 65536,
                        applied=abs(int(r["applied_compare"])),
                        moving=r["traj_moving"] == "True",
                        settled=r["traj_settled"] == "True"))
    if not out:
        raise SystemExit(f"{path}: no frames after 10 s")
    return out


def moving_average(values, width):
    total, out = 0.0, []
    for i, v in enumerate(values):
        total += v
        if i >= width:
            total -= values[i - width]
        out.append(total / min(i + 1, width))
    return out


def goertzel_amp(values, frequency, dt=0.01):
    mean = st.mean(values)
    w = 2 * math.pi * frequency * dt
    c = 2 * math.cos(w)
    s1 = s2 = 0.0
    for v in values:
        s0 = (v - mean) + c * s1 - s2
        s2, s1 = s1, s0
    return 2 * math.sqrt(max(0.0, s1 * s1 + s2 * s2 - c * s1 * s2)) / len(values)


def longest_run(mask, dt=0.01):
    best = current = 0
    for flag in mask:
        current = current + 1 if flag else 0
        best = max(best, current)
    return best * dt


def summarise(path):
    rows = load(path)
    still = [r for r in rows if r["settled"] and not r["moving"]]
    if len(still) < 100:
        raise SystemExit(f"{path}: only {len(still)} still frames")
    alpha = [r["alpha"] for r in still]
    arm = [r["arm"] for r in still]
    u = [r["u"] for r in still]
    applied = [r["applied"] for r in still]
    slow = lambda x: st.pstdev(moving_average(x, 100))
    peak = max(((f, goertzel_amp(u, f)) for f in
                (4 + 0.5 * i for i in range(23))), key=lambda item: item[1])
    return dict(
        name=Path(path).stem, frames=len(still), seconds=still[-1]["t"],
        alpha_std=st.pstdev(alpha), alpha_slow=slow(alpha),
        u8=goertzel_amp(u, 8.0), peak=peak,
        u_abs=st.mean(applied) / 25.0, sat=100 * sum(x >= SATURATE_COMPARE
                                                     for x in applied) / len(applied),
        sat_run=longest_run([x >= SATURATE_COMPARE for x in applied]),
        arm_std=st.pstdev(arm), arm_slow=slow(arm))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+")
    args = parser.parse_args()
    print(f"{'name':28s} {'frames':>6s} {'dur':>5s} {'alpha_std':>9s} {'alpha_slow':>10s} "
          f"{'u8':>6s} {'peak':>9s} {'|u|%':>5s} {'sat%':>5s} {'sat_run':>7s} "
          f"{'arm_std':>7s} {'arm_slow':>8s}")
    for path in args.csv:
        s = summarise(path)
        print(f"{s['name']:28s} {s['frames']:6d} {s['seconds']:5.0f} "
              f"{s['alpha_std']:9.2f} {s['alpha_slow']:10.2f} {s['u8']:6.0f} "
              f"{s['peak'][0]:4.1f}Hz{s['peak'][1]:5.0f} {s['u_abs']:5.1f} "
              f"{s['sat']:5.1f} {s['sat_run']*1000:6.0f}ms {s['arm_std']:7.1f} "
              f"{s['arm_slow']:8.1f}")


if __name__ == "__main__":
    main()
