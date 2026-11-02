"""Swing core RTL against the exact integer reference (scripts/j280_swing_math.py)."""
import json
import random
import struct
import sys
import unittest
from pathlib import Path

from test_j280_motor_test_rtl import simulate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_swing_math as sm
import j280_swingup_sim as ss
import j280_swing_protocol as wire

SOURCES = [ROOT / 'rtl/j280_swing/j280_swing_tables.v', ROOT / 'rtl/j280_swing/j280_swing_core.v',
           ROOT / 'sim/j280_swing/tb_swing_core.v']
ZERO = 737 << 16


def default_values(**change):
    base = json.loads((ROOT / 'docs/data/balance-design-2026-10-17/swingup-fixed-01.json')
                      .read_text(encoding='utf-8'))['swing_values']
    return dict(base, **change)


def cfg_bits(values):
    payload = bytes((5, 4)) + struct.pack(wire.SWING_FORMAT, *(values[f[0]] for f in sm.FIELDS))
    return int.from_bytes(payload, 'little')


class Vectors:
    def __init__(self, values, max_compare=1000, zero_q16=ZERO):
        self.p, self.max, self.zero = values, max_compare, zero_q16
        self.rows = [f'{cfg_bits(values):0144x} {max_compare} {zero_q16 & 0xffffffff:08x}']
        self.s = None

    def start(self, code, valid=True):
        self.s = sm.initial_state(code, valid)
        self.rows.append(f'0 {code} {int(valid)} 00000000 ' + ' '.join(['0000', '0', '0', '0', '0'] + ['00000000'] * 9))

    def step(self, code, valid, theta):
        self.s, o = sm.step(self.s, code, valid, theta, self.p, max_compare=self.max, zero_q16=self.zero)
        words = [o['est_a'], o['est_ad'], o['energy'], o['ref'], o['theta_speed'], *o['preset']]
        self.rows.append(' '.join([f'1 {code} {int(valid)} {theta & 0xffffffff:08x} {o["command"] & 0xffff:04x}',
                                   str(int(o['good'])), str(int(o['blind_mid'])), str(int(o['catch'])),
                                   f'{o["blind"]:04x}'] + [f'{w & 0xffffffff:08x}' for w in words]))
        return o


def calibrated_values():
    # Swing law calibrated to the hardware (swing-605); catches under the coupled plant model.
    return json.loads((ROOT / 'docs/data/swing-board-2026-10-18/swing-605-config.json')
                      .read_text(encoding='utf-8'))['swing']


def closed_loop(vectors, case, seed, swing=None, seconds=4.0):
    inputs = []
    ss.simulate_fixed(swing or vectors.p, case, seconds, seed, inputs=inputs)
    vectors.start(inputs[0])
    return [vectors.step(*x) for x in inputs[1:]]


class SwingCoreRTLTests(unittest.TestCase):
    def run_rtl(self, vectors):
        text, _ = simulate('tb_swing_core', SOURCES, {'vectors.txt': '\n'.join(vectors.rows) + '\n'}, timeout=600)
        self.assertIn(f'PASS tb_swing_core rows={len(vectors.rows) - 1}', text)

    def test_generated_tables_match_reference(self):
        self.assertEqual(sm.TABLES_V.read_text(encoding='ascii'), sm.tables_verilog())

    def test_closed_loop_swing_ups_including_dead_zone_and_spurious_samples(self):
        v = Vectors(calibrated_values())
        outs = []
        for case, seed in (({}, 1), (dict(spurious=1.0), 2), (dict(warp_deg=-4.0), 3),
                           (dict(alpha0_deg=-175.0, gain=0.9, rho=0.25), 4)):
            outs += closed_loop(v, case, seed)
        self.assertTrue(any(o['catch'] for o in outs))
        self.assertTrue(any(o['blind'] for o in outs))
        self.assertTrue(any(not o['good'] and o['blind'] == 1 for o in outs))
        self.run_rtl(v)

    def test_long_swing_without_catch_wraps_through_the_dead_zone(self):
        v = Vectors(default_values(catch_angle=1, margin=1 << 15))
        outs = closed_loop(v, dict(spurious=0.5), 5, seconds=6.0)
        self.assertGreaterEqual(sum(1 for o in outs if o['blind'] == 1), 3)
        self.run_rtl(v)

    def test_random_inputs_and_extreme_configurations(self):
        rng = random.Random(0x5714)
        values = {name: rng.choice((lo, hi, rng.randint(lo, hi))) for name, _, lo, hi in sm.FIELDS}
        values.update(edge_low=rng.randint(0, 200), edge_high=rng.randint(800, 1023))
        v = Vectors(values, max_compare=rng.randint(1, 1000), zero_q16=rng.randint(0, 1023 << 16))
        for trial in range(12):
            v.start(rng.randint(0, 1023), rng.random() < 0.8)
            code, theta = rng.randint(0, 1023), 0
            for _ in range(300):
                r = rng.random()
                if r < 0.6:
                    code = min(1023, max(0, code + rng.randint(-8, 8)))
                elif r < 0.8:
                    code = rng.randint(0, 1023)
                theta = rng.choice((theta + rng.randint(-20, 20), rng.randint(-5000, 5000),
                                    rng.choice((-(1 << 31), (1 << 31) - 1))))
                theta = max(-(1 << 31), min((1 << 31) - 1, theta))
                v.step(code, rng.random() < 0.85, theta)
        self.run_rtl(v)

    def test_random_configuration_corners(self):
        rng = random.Random(0x5715)
        for trial in range(4):
            values = {name: (lo, hi)[(trial >> (i % 2)) & 1] for i, (name, _, lo, hi) in enumerate(sm.FIELDS)}
            values.update(edge_low=100, edge_high=900)
            v = Vectors(values, max_compare=(1000, 1, 517, 1000)[trial], zero_q16=(0, 1023 << 16, ZERO, 1)[trial])
            v.start(rng.randint(0, 1023), True)
            for _ in range(400):
                v.step(rng.randint(0, 1023), rng.random() < 0.9, rng.randint(-6000, 6000))
            self.run_rtl(v)


if __name__ == '__main__':
    unittest.main()
