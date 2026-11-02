"""Integral demonstration: shift the upright zero on purpose, compare ki=0 vs ki>0.

The resting arm offset of this machine equals about -13.5 x (settled ADC - zero),
so moving the configured zero by D codes moves the no-integral resting point by
about -13.5 D counts. Both runs share the shifted zero, so the only difference
between them is the arm integral.
"""
import argparse
import csv
import json
import statistics as st
from pathlib import Path

COUNT_DEG = 360.0 / 1040.0


def shifted_config(base_path, out_path, config_id, zero_code, ki, i_window):
    data = json.loads(Path(base_path).read_text(encoding="utf-8-sig"))
    values = data["values"]
    values["config_id"] = config_id
    values["upright_zero_q16"] = zero_code << 16
    data["traj"]["ki"] = ki
    data["traj"]["i_window"] = i_window
    if "swing" in data:
        data["swing"]["config_id"] = config_id
    data["purpose"] = (f"integral demo {config_id}: upright zero {zero_code}, "
                       f"arm integral ki={ki} window={i_window}")
    data["sources"]["upright_zero_q16"] = f"{zero_code} code: deliberately shifted for the integral demo"
    data["sources"]["config_id"] = f"integral demo {config_id}"
    data["traj_source"] = f"integral demo: ki={ki}, i_window={i_window}"
    Path(out_path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")


def balance_rows(path):
    rows = [r for r in csv.DictReader(open(path, encoding="utf-8")) if r["state"] == "BALANCE"]
    if not rows:
        raise SystemExit(f"{path}: no BALANCE frames")
    t0 = int(rows[0]["tick_us_unwrapped"])
    out = []
    for r in rows:
        out.append(dict(
            t=(int(r["tick_us_unwrapped"]) - t0) / 1e6,
            err=int(r["enc_count"]) - int(r["theta_ref"]),
            adc=int(r["adc_raw10"]),
            u_ff=int(r["u_ff_q16"]) / 65536.0,
            moving=r["traj_moving"] == "True",
            settled=r["traj_settled"] == "True",
            ref=int(r["theta_ref"]),
        ))
    return out


def still(rows):
    return [r for r in rows if not r["moving"] and r["settled"]]


def convergence(rows):
    """First time the mean over the following 10 s of still frames is within 8 counts."""
    s = still(rows)
    n = len(s)
    if n < 300:
        return None
    pref = [0]
    for r in s:
        pref.append(pref[-1] + r["err"])
    for i in range(n - 300):
        j = min(n, i + 1000)
        if abs((pref[j] - pref[i]) / (j - i)) <= 8:
            return s[i]["t"]
    return None


def summary(name, rows):
    s = still(rows)
    late = [r for r in s if r["t"] > 12]
    if not late:
        late = s
    print(f"{name}: BALANCE {rows[-1]['t']:.1f} s, still frames {len(s)}")
    print(f"   after 12 s: arm error mean {st.mean(r['err'] for r in late):+7.1f} count "
          f"({st.mean(r['err'] for r in late) * COUNT_DEG:+.1f} deg), "
          f"std {st.pstdev([r['err'] for r in late]):.1f}")
    print(f"               ADC mean {st.mean(r['adc'] for r in late):.2f}, "
          f"u_ff mean {st.mean(r['u_ff'] for r in late):+7.2f} compare")
    c = convergence(rows)
    print(f"   convergence to |mean|<=8 count: {c:.1f} s" if c is not None
          else "   convergence to |mean|<=8 count: not reached")
    print("   per 5 s mean arm error:")
    buckets = {}
    for r in s:
        buckets.setdefault(int(r["t"] // 5) * 5, []).append(r["err"])
    print("     " + "  ".join(f"{b}-{b + 5}s:{st.mean(v):+.0f}" for b, v in sorted(buckets.items())))


def svg(path, series, title):
    W, H = 960, 360
    L, R, T, B = 60, 20, 40, 40
    allpts = [p for pts, _, _ in series for p in pts]
    tmax = max(p[0] for p in allpts)
    ymax = max(40, max(abs(p[1]) for p in allpts))
    ymax = int(ymax / 20 + 1) * 20

    def px(t):
        return L + (W - L - R) * t / tmax

    def py(y):
        return T + (H - T - B) * (ymax - y) / (2 * ymax)

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'font-family="sans-serif" font-size="12">',
           f'<rect width="{W}" height="{H}" fill="white"/>',
           f'<text x="{L}" y="22" font-size="15">{title}</text>']
    for y in range(-ymax, ymax + 1, ymax // 4):
        out.append(f'<line x1="{L}" y1="{py(y):.1f}" x2="{W - R}" y2="{py(y):.1f}" '
                   f'stroke="#ddd"/>')
        out.append(f'<text x="{L - 8}" y="{py(y) + 4:.1f}" text-anchor="end">{y}</text>')
    for t in range(0, int(tmax) + 1, max(5, int(tmax // 8))):
        out.append(f'<line x1="{px(t):.1f}" y1="{T}" x2="{px(t):.1f}" y2="{H - B}" stroke="#eee"/>')
        out.append(f'<text x="{px(t):.1f}" y="{H - B + 16}" text-anchor="middle">{t}</text>')
    out.append(f'<text x="{(L + W - R) / 2}" y="{H - 6}" text-anchor="middle">time after takeover (s)</text>')
    out.append(f'<text x="14" y="{(T + H - B) / 2}" text-anchor="middle" '
               f'transform="rotate(-90 14 {(T + H - B) / 2})">arm error (encoder count, 1 count = 0.35 deg)</text>')
    out.append(f'<line x1="{L}" y1="{py(0):.1f}" x2="{W - R}" y2="{py(0):.1f}" stroke="#555"/>')
    for pts, color, label in series:
        poly = " ".join(f"{px(t):.1f},{py(y):.1f}" for t, y in pts)
        out.append(f'<polyline points="{poly}" fill="none" stroke="{color}" stroke-width="1.6"/>')
    lx = L + 20
    for _, color, label in series:
        out.append(f'<line x1="{lx}" y1="{T + 12}" x2="{lx + 24}" y2="{T + 12}" stroke="{color}" stroke-width="3"/>')
        out.append(f'<text x="{lx + 30}" y="{T + 16}">{label}</text>')
        lx += 40 + 8 * len(label)
    out.append("</svg>")
    Path(path).write_text("\n".join(out) + "\n", encoding="utf-8")


def thin(rows, step=0.1):
    pts = []
    last = -1
    for r in rows:
        if r["t"] - last >= step:
            pts.append((r["t"], r["err"]))
            last = r["t"]
    return pts


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("config", help="write a demo config from a base profile")
    c.add_argument("--base", required=True)
    c.add_argument("--out", required=True)
    c.add_argument("--config-id", type=int, required=True)
    c.add_argument("--zero-code", type=int, required=True)
    c.add_argument("--ki", type=int, default=0)
    c.add_argument("--i-window", type=int, default=120)
    a = sub.add_parser("analyze", help="compare an A/B pair")
    a.add_argument("--a", required=True)
    a.add_argument("--b", required=True)
    a.add_argument("--svg")
    a.add_argument("--title", default="arm integral: resting offset")
    args = ap.parse_args()
    if args.cmd == "config":
        shifted_config(args.base, args.out, args.config_id, args.zero_code, args.ki, args.i_window)
        print(f"wrote {args.out}")
        return
    series = []
    colors = ["#c0392b", "#1f77b4"]
    for i, path in enumerate((args.a, args.b)):
        rows = balance_rows(path)
        summary(Path(path).stem, rows)
        series.append((thin(rows), colors[i], Path(path).stem))
    if args.svg:
        svg(args.svg, series, args.title)
        print(f"wrote {args.svg}")


if __name__ == "__main__":
    main()
