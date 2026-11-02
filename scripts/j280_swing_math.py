#!/usr/bin/env python3
"""Bit-exact v5 swing-up arithmetic: the specification of rtl/j280_swing/j280_swing_core.v.

Units: pendulum angle is a binary angle (int32, 2**32 = one turn, 0 upright,
ADC-increasing positive, hanging = -2**31); its rate is binary angle per 2 ms
sample. Arm position is whole encoder counts relative to the swing start.
Every right shift is an arithmetic (floor) shift; products are exact.
The observer preset uses the balance core's native Q16 units.
"""
import json
import math
from pathlib import Path

from j280_measurement import crc16_ccitt

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP = ROOT / 'docs/data/angle-calibration-2026-10-18/angle-map-v2.json'
MASK32 = (1 << 32) - 1
QUARTER = 1 << 30
AD_LIMIT = 1 << 30
V_LIMIT = 1 << 20
TH_LIMIT = 4095
RATE_LIMIT = 300
ONE = 1 << 16
T = 0.002

# SWING command fields: (name, struct code, lower, upper). Order is the wire order.
FIELDS = (
    ('config_id', 'H', 1, 65535), ('swing_ms', 'H', 500, 10000),
    ('travel_count', 'H', 1, 260), ('blind_samples', 'H', 1, 100),
    ('step_code', 'H', 1, 64), ('edge_low', 'H', 0, 200), ('edge_high', 'H', 800, 1023),
    ('hang_code', 'H', 0, 1023), ('hang_window', 'H', 0, 60),
    ('kick_samples', 'H', 0, 250), ('kick_compare', 'h', -1000, 1000),
    ('lead_samples', 'H', 0, 100), ('table_crc', 'H', 0, 65535),
    ('catch_angle', 'i', 1, 357913941), ('catch_rate', 'i', 1, 1 << 28),
    ('gravity', 'i', 0, 1 << 20), ('gravity_lead', 'i', 0, 1 << 27),
    ('kv', 'i', 0, 1 << 26), ('kdrive', 'i', 0, 1 << 30),
    ('margin', 'i', -(1 << 20), 1 << 20), ('kscale', 'i', 0, 1 << 20),
    ('amp', 'i', 0, 1 << 25), ('kp', 'i', 0, 1 << 24), ('kd', 'i', 0, 1 << 27),
)


def wrap32(x):
    return ((x + (1 << 31)) & MASK32) - (1 << 31)


def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def load_knots(path=None):
    data = json.loads(Path(path or DEFAULT_MAP).read_text(encoding='utf-8'))
    return [(k['code'], k['deg']) for k in data['knots']]


def angle_table(path=None):
    """1024 uint16 binary angles (2**16 = one turn) from upright for each ADC code."""
    knots = load_knots(path)
    out = []
    for code in range(1024):
        for (c0, d0), (c1, d1) in zip(knots, knots[1:]):
            if c0 <= code <= c1:
                deg = d0 + (d1 - d0) * (code - c0) / (c1 - c0)
                break
        else:
            raise ValueError(f'map does not cover code {code}')
        out.append(round((deg - 180.0) / 360.0 * 65536) & 0xFFFF)
    return out


def sine_table():
    """1024 signed Q16 samples of sin over one turn (+-65536 representable in 18 bits)."""
    return [round(math.sin(2 * math.pi * i / 1024) * ONE) for i in range(1024)]


def table_crc(angles, sines):
    data = b''.join(a.to_bytes(2, 'little') for a in angles)
    data += b''.join((s & 0x3FFFF).to_bytes(3, 'little') for s in sines)
    return crc16_ccitt(data)


ANGLES = angle_table()
SINES = sine_table()
TABLE_CRC = table_crc(ANGLES, SINES)


def sin_q16(a):
    return SINES[((a + (1 << 21)) & MASK32) >> 22]


def cos_q16(a):
    return sin_q16(a + QUARTER)


def angle_of(code):
    return wrap32(ANGLES[code] << 16)


def compile_params(p, *, config_id, max_compare, w0, count_rad, hang_code=221,
                   swing_ms=6000, travel_count=220, blind_samples=100, step_code=24,
                   edge_low=40, edge_high=960, hang_window=15):
    """Floating swing-up parameters (j280_swingup_sim 'ref' strategy) -> SWING integers."""
    g = T * T * w0 * w0 * (1 << 32) / (2 * math.pi)
    lead = round(p['lead_s'] / T)
    v = dict(
        config_id=config_id, swing_ms=swing_ms, travel_count=travel_count,
        blind_samples=blind_samples, step_code=step_code, edge_low=edge_low, edge_high=edge_high,
        hang_code=hang_code, hang_window=hang_window,
        kick_samples=round(p.get('kick_ms', 120.0) / 2), kick_compare=max_compare,
        lead_samples=lead, table_crc=TABLE_CRC,
        catch_angle=round(p['catch_deg'] / 360 * (1 << 32)),
        catch_rate=round(p['catch_rate'] * T * (1 << 32) / (2 * math.pi)),
        gravity=round(g), gravity_lead=round(lead * g),
        kv=round(2 * math.pi * ONE / (T * w0)),
        kdrive=round(2 * math.pi * ONE / (T * p['v_sat'])),
        margin=round(p['energy_margin'] / (w0 * w0) * ONE),
        kscale=round(w0 * w0 / p['e_scale'] * ONE),
        amp=round(math.radians(p['amp_deg']) / count_rad * ONE),
        kp=round(p['kp'] * 25 * count_rad * ONE),
        kd=round(p['kd'] * 25 * count_rad / (5 * T) * ONE))
    return validate(v)


def validate(values):
    if not isinstance(values, dict) or set(values) != {f[0] for f in FIELDS}:
        raise ValueError('swing values must be exactly ' + ', '.join(f[0] for f in FIELDS))
    for name, _, lo, hi in FIELDS:
        value = values[name]
        if type(value) is not int or not lo <= value <= hi:
            raise ValueError(f'{name} must be an integer in {lo}..{hi}')
    if values['edge_low'] >= values['edge_high']:
        raise ValueError('edge_low must be below edge_high')
    if not values['edge_low'] < values['hang_code'] - values['hang_window'] or \
            not values['hang_code'] + values['hang_window'] < values['edge_high']:
        raise ValueError('hang window must lie outside the dead-zone edge bands')
    if values['table_crc'] != TABLE_CRC:
        raise ValueError('table_crc does not match the compiled angle/sine tables')
    return dict(values)


TABLES_V = ROOT / 'rtl/j280_swing/j280_swing_tables.v'


def tables_verilog():
    """Synchronous-read ROMs for the swing core, generated from ANGLES/SINES."""
    lines = ['`timescale 1ns/1ps', '// GENERATED by scripts/j280_swing_math.py --write-tables from',
             f'// {DEFAULT_MAP.relative_to(ROOT).as_posix()}. Do not edit.', '',
             'module j280_swing_angle_rom(input wire clk, input wire [9:0] addr, output reg [15:0] q);',
             '    always @(posedge clk) case (addr)']
    lines += [f"        10'd{i}: q <= 16'h{v:04x};" for i, v in enumerate(ANGLES)]
    lines += ["        default: q <= 16'h0000;", '    endcase', 'endmodule', '',
              'module j280_swing_sine_rom(input wire clk, input wire [9:0] addr, output reg signed [17:0] q);',
              '    always @(posedge clk) case (addr)']
    lines += [f"        10'd{i}: q <= 18'h{v & 0x3FFFF:05x};" for i, v in enumerate(SINES)]
    lines += ["        default: q <= 18'h00000;", '    endcase', 'endmodule', '',
              'module j280_swing_table_id(output wire [15:0] crc);',
              f"    assign crc = 16'h{TABLE_CRC:04x};", 'endmodule', '']
    return '\n'.join(lines)


def initial_state(code, adc_valid):
    a = angle_of(code)
    return dict(est_a=a, est_ad=0, hist=[(adc_valid, a, code)] * 4, last_good=adc_valid,
                last_code=code, theta=[0] * 5, n=0, blind=0)


def step(s, code, adc_valid, theta, p, *, max_compare, zero_q16):
    """One 2 ms sample. Returns (new_state, outputs); s is not modified."""
    in_edge = lambda c: c <= p['edge_low'] or c >= p['edge_high']
    jump = abs(code - s['last_code'])           # last_code: the last good code
    if adc_valid and s['last_good']:
        good = jump <= p['step_code']
    elif adc_valid:
        # After a rejected sample: continuity with a widened step, or a dead-zone edge.
        good = jump <= p['step_code'] * (1 + min(s['blind'], 3)) or in_edge(code)
    else:
        good = False
    # An invalid ADC with the last good angle mid-range is a sensor failure, not the dead zone.
    blind_mid = not adc_valid and not in_edge(s['last_code'])
    blind = 0 if good else min(s['blind'] + 1, 0xFFFF)
    h = s['hist']
    m = angle_of(code)
    run4 = good and all(x[0] for x in h)
    if run4:
        ad, a = (wrap32(m - h[3][1])) >> 2, m
    elif good and h[0][0] and h[1][0]:
        ad, a = (wrap32(m - h[1][1])) >> 1, m
    else:
        ad = clip(s['est_ad'] + ((p['gravity'] * sin_q16(s['est_a'])) >> 16), -AD_LIMIT, AD_LIMIT)
        a = m if good else wrap32(s['est_a'] + ad)
    th = clip(theta, -TH_LIMIT, TH_LIMIT)
    rate5 = th - s['theta'][4]
    sa, ca = sin_q16(a), cos_q16(a)
    v = clip((ad * p['kv']) >> 32, -V_LIMIT, V_LIMIT)
    energy = ((v * v) >> 17) + ca - ONE
    a_pred = wrap32(a + p['lead_samples'] * ad)
    ad_pred = ad + ((p['gravity_lead'] * sa) >> 16)
    t = (ad_pred * cos_q16(a_pred)) >> 16
    drive = clip(-((t * p['kdrive']) >> 32), -ONE, ONE)
    scale = clip(((p['margin'] - energy) * p['kscale']) >> 16, 0, ONE)
    ref = -((((p['amp'] * scale) >> 16) * drive) >> 16)
    u = ((p['kp'] * (ref - (th << 16))) >> 16) - p['kd'] * rate5
    command = clip(u >> 16, -max_compare, max_compare)
    if s['n'] < p['kick_samples']:
        command = clip(p['kick_compare'], -max_compare, max_compare)
    catch = run4 and abs(a) < p['catch_angle'] and abs(ad) < p['catch_rate']
    theta_speed = clip(rate5, -RATE_LIMIT, RATE_LIMIT) * 100 * ONE    # count/s, Q16
    preset = [th << 16, (code << 16) - zero_q16, theta_speed,
              (code - h[3][2]) * 125 * ONE] if run4 else [0, 0, 0, 0]
    new = dict(est_a=a, est_ad=ad, hist=[(good, m, code)] + h[:3],
               last_good=good, last_code=code if good else s['last_code'],
               theta=[th] + s['theta'][:4], n=min(s['n'] + 1, 0xFFFF), blind=blind)
    return new, dict(command=command, good=good, blind=blind, blind_mid=blind_mid, catch=catch,
                     est_a=a, est_ad=ad, energy=energy, ref=ref, theta_speed=theta_speed, preset=preset)


if __name__ == '__main__':
    import sys
    if sys.argv[1:] == ['--write-tables']:
        TABLES_V.parent.mkdir(parents=True, exist_ok=True)
        TABLES_V.write_text(tables_verilog(), encoding='ascii', newline='\n')
        print(TABLES_V, f'crc=0x{TABLE_CRC:04x}')
    else:
        raise SystemExit('usage: j280_swing_math.py --write-tables')
