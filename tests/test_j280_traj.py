"""v6 trajectory spec: landing, grid, rotation exit, key queue, integral, feedforward, validation."""
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import j280_traj as tj  # noqa: E402

S = tj.S


def run(params, n, events=None, anchor=0, arm=None, settled_from=0):
    t = tj.Trajectory(params, anchor)
    out = []
    events = events or {}
    for k in range(n):
        ev = events.get(k, {})
        a = arm(k, t) if arm else (t.r >> 16)
        out.append(t.step(settled=k >= settled_from, arm=a, **ev))
    return t, out


class TrajectorySpecTests(unittest.TestCase):
    def test_step_lands_exactly_without_overshoot_within_speed(self):
        p = tj.default_params()
        t, out = run(p, 3000, {10: dict(cw=True)})
        self.assertEqual(out[-1]['r'], 260 * S)
        self.assertLessEqual(max(o['r'] for o in out), 260 * S)
        self.assertTrue(all(abs(o['v']) <= p['move_speed'] for o in out))
        self.assertEqual(t.m, 0)
        moving = [o for o in out if o['moving']]
        self.assertGreater(len(moving), 900)
        self.assertEqual({o['acc'] for o in out}, {-p['accel'], 0, p['accel']})

    def test_keys_ignored_before_settle_and_queue_is_bounded(self):
        p = tj.default_params()
        _, out = run(p, 50, {5: dict(cw=True)}, settled_from=10)
        self.assertEqual(out[-1]['target'], 0)
        ev = {k: dict(cw=True) for k in range(1, 12)}
        _, out = run(p, 30, ev)
        self.assertEqual(out[-1]['target'], 4 * 260)
        _, out = run(p, 5, {2: dict(cw=True, ccw=True)})
        self.assertEqual(out[-1]['target'], 0)

    def test_reversal_mid_move_decelerates_then_returns(self):
        p = tj.default_params()
        _, out = run(p, 4000, {10: dict(cw=True), 400: dict(ccw=True)})
        self.assertEqual(out[-1]['r'], 0)
        peak = max(o['r'] for o in out)
        self.assertGreater(peak, 0)
        self.assertLess(peak, 260 * S)

    def test_rotation_turns_and_exit_stops_on_the_grid(self):
        p = tj.default_params()
        ev = {k: dict(rotate=True) for k in range(5, 6000)}
        t, out = run(p, 9000, ev)
        self.assertTrue(any(o['rotating'] for o in out))
        self.assertEqual(max(abs(o['v']) for o in out), p['rotate_speed'] // p['accel'] * p['accel'])
        end = out[-1]
        self.assertFalse(end['moving'] or end['rotating'])
        self.assertEqual(end['r'] % (260 * S), 0)
        self.assertGreater(end['r'], 4 * 260 * S)
        self.assertEqual(end['target'], end['index'] * 260)
        self.assertTrue(all(o['r'] <= end['r'] for o in out))

    def test_rotation_direction_follows_last_key(self):
        p = tj.default_params()
        ev = {5: dict(ccw=True)}
        ev.update({k: dict(rotate=True) for k in range(2000, 4000)})
        _, out = run(p, 7000, ev)
        self.assertLess(out[-1]['r'], -260 * S)
        self.assertEqual(out[-1]['r'] % (260 * S), 0)

    def test_lean_references_follow_the_acceleration_sign(self):
        p = tj.default_params()
        _, out = run(p, 3000, {10: dict(cw=True)})
        for o in out:
            s = (o['acc'] > 0) - (o['acc'] < 0)
            self.assertEqual(o['alpha_ref'], s * (p['lean_pend'] << 8))
            self.assertEqual(o['theta_ref'], o['r'] - s * (p['lean_arm'] << 8))

    def test_feedforward_formula_and_clip(self):
        p = tj.default_params()
        _, out = run(p, 3000, {10: dict(cw=True)})
        for o in out:
            self.assertEqual(o['u_ff'], tj.trunc0(o['v'] * p['kv'], 2) + o['acc'] * p['ka'])
        big = dict(p, kv=65535, ka=65535)
        t = tj.Trajectory(big, 0, max_compare=10)
        o = None
        for k in range(400):
            o = t.step(settled=True, cw=(k == 1))
        self.assertLessEqual(abs(o['u_ff']), 10 * S)

    def test_integral_sign_window_freeze_and_clamp(self):
        p = tj.default_params(ki=1000, i_window=80)
        t = tj.Trajectory(p, 0)
        o = t.step(settled=True, arm=10)
        self.assertEqual(t.i_acc, 10)
        self.assertEqual(o['u_ff'], tj.trunc0(10 * 1000, 4))
        t.step(settled=True, arm=81)
        self.assertEqual(t.i_acc, 10)
        t.step(settled=False, arm=10)
        self.assertEqual(t.i_acc, 10)
        t.step(settled=True, arm=-5, cw=True)
        self.assertEqual(t.i_acc, 5)
        t.step(settled=True, arm=50)
        self.assertEqual(t.i_acc, 5)
        t = tj.Trajectory(dict(p, i_window=1040), 0)
        for _ in range(2000):
            o = t.step(settled=True, arm=1040)
        self.assertEqual(t.i_acc, tj.I_ACC_MAX)
        self.assertEqual(o['u_ff'], tj.U_I_MAX)

    def test_disabled_and_validation(self):
        p = dict(tj.default_params(), step_counts=0)
        _, out = run(p, 200, {10: dict(cw=True), 20: dict(rotate=True)})
        self.assertTrue(all(o['r'] == 0 and o['u_ff'] == 0 for o in out))
        for bad in (dict(step_counts=1041), dict(accel=0), dict(move_speed=10, accel=11),
                    dict(rotate_speed=5, accel=11, move_speed=20), dict(lean_arm=16384), dict(kv=-1),
                    dict(i_window=1041), dict(bogus=1)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                tj.validate(dict(tj.default_params(), **bad))

    def test_random_streams_never_overshoot_a_resting_target(self):
        rng = random.Random(6)
        for trial in range(20):
            p = tj.default_params(move_dps=rng.choice((20, 45, 90)), rotate_dps=rng.choice((0, 60, 120)),
                                  accel_dps2=rng.choice((50, 150, 400)))
            t = tj.Trajectory(p, rng.randint(-5000, 5000))
            rot = False
            for k in range(6000):
                if rng.random() < 0.002:
                    rot = not rot
                ev = dict(cw=rng.random() < 0.004, ccw=rng.random() < 0.004, rotate=rot)
                o = t.step(settled=True, arm=t.r >> 16, **ev)
                self.assertLessEqual(abs(o['v']), max(p['move_speed'],
                                                       p['rotate_speed'] + (p['rotate_speed'] >> 1)))
                self.assertEqual(t.speed, t.m * p['accel'])
                self.assertEqual(t.stop, p['accel'] * t.m * (t.m - 1) // 2)
                if not t.rotating:
                    self.assertLessEqual(abs(t.target * S - t.r), (tj.MAX_QUEUE_STEPS + 2) * 260 * S)

    def test_rotation_gear_keys_and_reset(self):
        p = tj.default_params(rotate_dps=60, accel_dps2=150)
        t = tj.Trajectory(p, 0)
        base = p['rotate_speed']
        t.step(settled=True, rotate=True)
        for _ in range(400):
            o = t.step(settled=True, rotate=True)
        self.assertEqual(t.rot_gear, 1)
        self.assertLessEqual(abs(o['v']), base)
        self.assertGreater(abs(o['v']), base - p['accel'])
        t.step(settled=True, rotate=True, rot_gear_up=True)   # long SW3 -> 1.5x
        for _ in range(400):
            o = t.step(settled=True, rotate=True)
        self.assertEqual(t.rot_gear, 2)
        rt15 = base + (base >> 1)
        self.assertLessEqual(abs(o['v']), rt15)
        self.assertGreater(abs(o['v']), rt15 - p['accel'])
        t.step(settled=True, rotate=True, rot_gear_down=True) # long SW4 twice -> 0.5x
        t.step(settled=True, rotate=True, rot_gear_down=True)
        for _ in range(400):
            o = t.step(settled=True, rotate=True)
        self.assertEqual(t.rot_gear, 0)
        rt05 = base >> 1
        self.assertLessEqual(abs(o['v']), rt05)
        self.assertGreater(abs(o['v']), rt05 - p['accel'])
        for _ in range(3000):                                # exit and ramp to rest
            t.step(settled=True, rotate=False)
            if t.m == 0 and t.r == t.target * S:
                break
        t.step(settled=True, rotate=True)                    # re-enter resets to 1.0x
        self.assertEqual(t.rot_gear, 1)


if __name__ == '__main__':
    unittest.main()
