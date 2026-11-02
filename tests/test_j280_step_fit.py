"""Step-response fit and step-series planning on synthetic data."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import j280_step_fit as stepfit  # noqa: E402
import j280_step_series as series  # noqa: E402
import j280_step_model as model  # noqa: E402


def trace(v_inf, tau, td, duration_ms, x0=0):
    out = []
    for i in range(duration_ms // 2 + 1):
        t = 2 * i
        s = t - td
        x = 0.0 if s <= 0 else v_inf * (s - tau * (1 - math.exp(-s / tau))) / 1000
        out.append((t, x0 + math.floor(x + 0.5)))
    return out


class StepFitTests(unittest.TestCase):
    def test_recovers_quantized_first_order_step(self):
        for v_inf, tau, td in ((800, 40, 10), (-1500, 25, 4), (400, 80, 20)):
            with self.subTest(v_inf=v_inf):
                r = stepfit.fit(trace(v_inf, tau, td, 300))
                self.assertAlmostEqual(r['v_inf_count_s'], v_inf, delta=abs(v_inf) * 0.08)
                self.assertAlmostEqual(r['tau_ms'], tau, delta=tau * 0.3 + 4)
                self.assertLessEqual(abs(r['dead_time_ms'] - td), 8)
                self.assertLess(r['rms_counts'], 0.6)

    def test_end_speed_and_drive_samples(self):
        samples = [(2 * i, 3 * i) for i in range(51)]
        self.assertEqual(stepfit.end_speed(samples), 1500.0)
        rows = [dict(t=2000 * i, enc=5 + i, on=1 <= i <= 4) for i in range(7)]
        self.assertEqual(stepfit.drive_samples(rows), [(0, 0), (2, 1), (4, 2), (6, 3), (8, 4)])


class StepSeriesPlanningTests(unittest.TestCase):
    def test_direction_returns_toward_center(self):
        self.assertEqual(series.toward_center(35), 'in1_high')
        self.assertEqual(series.toward_center(-20), 'in2_high')
        self.assertEqual(series.toward_center(0), 'in2_high')

    def test_target_crosses_center_inside_limit(self):
        self.assertEqual(series.target_travel(0), 50)
        self.assertEqual(series.target_travel(30), 75)
        self.assertEqual(series.target_travel(100), series.TRAVEL_LIMIT - 45)

    def test_pulse_plan_is_bounded(self):
        self.assertEqual(series.plan_pulse(None, 80), series.PROBE_MS)
        self.assertEqual(series.plan_pulse((100, 20, 400), 80), 200)   # doubling cap
        self.assertEqual(series.plan_pulse((200, 60, 1000), 80), 220)
        self.assertEqual(series.plan_pulse((100, 0, 0), 80), 200)
        self.assertEqual(series.plan_pulse((100, 120, 1000), 80), series.MIN_MS)
        self.assertEqual(series.plan_pulse((500, 10, 60), 80), series.MAX_MS)


class StepModelTests(unittest.TestCase):
    def test_shared_model_recovers_gain_deadband_and_tau(self):
        runs = [(d, trace(-30 * (d - 7), 60, 0, ms)) for d, ms in ((20, 300), (24, 200), (28, 150))]
        r = model.fit_direction(runs)
        self.assertAlmostEqual(r['k_count_s_per_percent'], -30, delta=1.5)
        self.assertAlmostEqual(r['deadband_percent'], 7, delta=0.6)
        self.assertAlmostEqual(r['tau_ms'], 60, delta=6)
        self.assertLess(r['rms_counts'], 0.6)


if __name__ == '__main__':
    unittest.main()
