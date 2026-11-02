"""j280_traj RTL against scripts/j280_traj.py, bit for bit (file-driven tb_traj)."""
import random
import sys
import unittest
from pathlib import Path

from test_j280_motor_test_rtl import simulate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_traj as tj  # noqa: E402

SOURCES = [ROOT / 'rtl/j280_swing/j280_traj.v', ROOT / 'sim/j280_swing/tb_traj.v']
M48, M32, M16 = (1 << 48) - 1, (1 << 32) - 1, (1 << 16) - 1


class Rows:
    def __init__(self):
        self.rows, self.t, self.p, self.max_compare = [], None, None, 1000

    def params(self, p, max_compare=1000):
        self.p, self.max_compare = tj.validate(p), max_compare
        q = self.p
        self.rows.append('P ' + ' '.join(str(v) for v in (
            q['step_counts'], q['move_speed'], q['rotate_speed'], q['accel'], q['kv'], q['ka'], q['ki'],
            q['lean_arm'], q['lean_pend'], q['i_window'], max_compare)))

    def reset(self, anchor):
        self.t = tj.Trajectory(self.p, anchor, max_compare=self.max_compare)
        self.rows.append(f'R {anchor & M32:08x}')

    def sample(self, ccw=False, cw=False, rotate=False, settled=True, arm=None):
        if arm is None:
            arm = self.t.r >> 16
        o = self.t.step(ccw=ccw, cw=cw, rotate=rotate, settled=settled, arm=arm)
        self.rows.append(
            f'S {int(ccw)} {int(cw)} {int(rotate)} {int(settled)} {arm & M32:08x} '
            f'{o["r"] & M48:012x} {o["theta_ref"] & M48:012x} {o["alpha_ref"] & M32:08x} {o["u_ff"] & M32:08x} '
            f'{o["v"] & M32:08x} {o["target"] & M32:08x} {o["index"] & M16:04x} {int(o["moving"])} {int(o["rotating"])}')
        return o


class TrajRTLTests(unittest.TestCase):
    def run_rtl(self, rows):
        n = sum(r.startswith('S') for r in rows.rows)
        text, _ = simulate('tb_traj', SOURCES, {'vectors.txt': '\n'.join(rows.rows) + '\n'}, timeout=1200)
        self.assertIn(f'PASS tb_traj rows={n}', text, text[-2000:])

    def test_steps_reversal_and_queue(self):
        r = Rows()
        r.params(tj.default_params())
        r.reset(123)
        for k in range(1600):
            r.sample(cw=k in (5, 6), ccw=k == 700, settled=k >= 3)
        for k in range(12):
            r.sample(cw=True)
        for k in range(200):
            r.sample(ccw=k < 3)
        self.run_rtl(r)

    def test_rotation_start_exit_and_direction(self):
        r = Rows()
        r.params(tj.default_params(rotate_dps=120, accel_dps2=400))
        r.reset(-7000)
        r.sample(ccw=True)
        for k in range(500):
            r.sample(rotate=k > 300)
        for k in range(900):
            r.sample(rotate=k < 600)
        self.run_rtl(r)

    def test_integral_window_clamp_and_clip(self):
        r = Rows()
        r.params(tj.default_params(ki=65535, i_window=1040), max_compare=50)
        r.reset(0)
        for k in range(300):
            r.sample(arm=1040 if k < 200 else -900)
        r.params(tj.default_params(ki=1000, i_window=30))
        r.reset(5)
        rng = random.Random(3)
        for k in range(400):
            r.sample(arm=5 + rng.randint(-60, 60), cw=k == 100, settled=k > 20)
        self.run_rtl(r)

    def test_disabled_and_random_parameter_streams(self):
        r = Rows()
        r.params(dict(tj.default_params(), step_counts=0, ki=200, i_window=100))
        r.reset(40)
        for k in range(60):
            r.sample(cw=True, rotate=True, arm=40 + k % 7)
        rng = random.Random(11)
        for trial in range(6):
            accel = rng.randint(1, 4095)
            move = rng.randint(accel, 65535)
            rot = rng.choice((0, rng.randint(accel, 65535)))
            p = dict(step_counts=rng.randint(1, 1040), move_speed=move, rotate_speed=rot, accel=accel,
                     kv=rng.randint(0, 65535), ka=rng.randint(0, 65535), ki=rng.randint(0, 65535),
                     lean_arm=rng.randint(0, 16383), lean_pend=rng.randint(0, 16383), i_window=rng.randint(0, 1040))
            r.params(p, max_compare=rng.randint(1, 1000))
            r.reset(rng.randint(-(1 << 30), 1 << 30))
            rotating = False
            for k in range(500):
                if rng.random() < 0.02:
                    rotating = not rotating
                r.sample(ccw=rng.random() < 0.03, cw=rng.random() < 0.03, rotate=rotating,
                         settled=rng.random() < 0.95, arm=(r.t.r >> 16) + rng.randint(-1200, 1200))
        self.run_rtl(r)


if __name__ == '__main__':
    unittest.main()
