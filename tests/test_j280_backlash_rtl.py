"""Backlash tracker and gap boost RTL against scripts/j280_backlash.py, bit for bit."""
import random
import sys
import unittest
from pathlib import Path

from test_j280_motor_test_rtl import simulate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_backlash as bl

SOURCES = [ROOT / 'rtl/j280_swing/j280_backlash.v', ROOT / 'sim/j280_swing/tb_backlash.v']


class Rows:
    def __init__(self):
        self.rows, self.a = [], 0

    def track(self, pos, balancing, half):
        self.a, offset = bl.track(self.a, pos, half, balancing)
        self.rows.append(f'T {pos & 0xffffffff:08x} {int(balancing)} {half} {self.a & 0xffffffff:08x} {offset}')
        return offset

    def boost(self, command, offset, half, gap_boost, threshold, max_compare):
        out, on = bl.boost(command, offset, half, gap_boost, threshold, max_compare)
        self.rows.append(f'B {command & 0xffff:04x} {offset} {half} {gap_boost} {threshold} {max_compare} {out & 0xffff:04x} {int(on)}')
        return out, on


class BacklashRTLTests(unittest.TestCase):
    def run_rtl(self, rows):
        text, _ = simulate('tb_backlash', SOURCES, {'vectors.txt': '\n'.join(rows.rows) + '\n'}, timeout=600)
        self.assertIn(f'PASS tb_backlash rows={len(rows.rows)}', text)

    def test_tracker_follows_gap_edges_and_resets_outside_balance(self):
        r = Rows()
        r.track(100, False, 5)
        for pos in (100, 103, 105, 106, 110, 108, 104, 100, 99, 95, 96, 101, 101):
            r.track(pos, True, 5)
        r.track(50, False, 5)
        self.assertEqual(r.a, 50)
        for pos in (51, 70, 69, 55, 54):
            r.track(pos, True, 0)
            self.assertEqual(r.a, pos)
        for half in range(16):
            r.track(1000, False, half)
            for pos in (1000 + half + 3, 1000 - half - 2, 1000):
                r.track(pos, True, half)
        self.run_rtl(r)

    def test_tracker_int32_extremes(self):
        r = Rows()
        for pos in (bl.I32_MAX, bl.I32_MAX - 3, bl.I32_MIN, bl.I32_MIN + 2, -1, 0):
            r.track(pos, False, 15)
            for d in (15, -15, 7, 0):
                p = max(bl.I32_MIN, min(bl.I32_MAX, pos + d))
                r.track(p, True, 15)
        self.run_rtl(r)

    def test_boost_rules_threshold_clip_and_disable(self):
        r = Rows()
        for command in (0, 1, -1, 74, 75, -75, 400, -400, 999, -999, 1000, -1000):
            for offset in (-5, -4, 0, 4, 5):
                for half, boost, thr in ((5, 100, 75), (5, 0, 75), (0, 100, 75), (5, 250, 0), (5, 1, 250)):
                    r.boost(command, offset, half, boost, thr, 1000)
        for mc in (1, 200, 517):
            r.boost(mc, -3, 5, 250, 0, mc)
            r.boost(-mc, 3, 5, 250, 0, mc)
        self.run_rtl(r)

    def test_random_mixed_streams(self):
        rng = random.Random(0xB1A5)
        r = Rows()
        pos = 0
        for _ in range(1500):
            half = rng.choice((0, 3, 5, 7, 15, rng.randint(0, 15)))
            if rng.random() < 0.5:
                pos = max(bl.I32_MIN, min(bl.I32_MAX, pos + rng.randint(-20, 20)))
                r.track(pos, rng.random() < 0.9, half)
            else:
                mc = rng.randint(1, 1000)
                r.boost(rng.randint(-mc, mc), rng.randint(-half, half) if half else 0, half,
                        rng.randint(0, 250), rng.randint(0, 250), mc)
        self.run_rtl(r)


if __name__ == '__main__':
    unittest.main()
