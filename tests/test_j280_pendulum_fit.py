"""Pendulum coupling fit on synthetic arm motion and quantized ADC."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import j280_pendulum_fit as pf  # noqa: E402


def synthetic_rows(true_params, travel, pulse_frames=100):
    """Arm moves `travel` counts with a smooth profile; ADC follows the model."""
    n_pre, n_post = 200, 1500
    enc = [0] * n_pre
    for k in range(pulse_frames + 40):
        s = min(1.0, k / (pulse_frames + 20))
        enc.append(round(travel * (s - math.sin(2 * math.pi * s) / (2 * math.pi))))
    enc += [enc[-1]] * n_post
    on = [n_pre <= i < n_pre + pulse_frames for i in range(len(enc))]
    run = dict(theta=[e * pf.COUNT_RAD for e in enc], phi=[0.0] * len(enc))
    n = len(enc)
    run['speed'] = [(run['theta'][min(n - 1, k + 5)] - run['theta'][max(0, k - 5)])
                    / ((min(n - 1, k + 5) - max(0, k - 5)) * pf.DT) for k in range(n)]
    phi = pf.simulate(run, true_params)
    return [dict(t=2000 * i, enc=enc[i], adc=220 + round(phi[i] * pf.ADC_CODE_PER_RAD), on=on[i])
            for i in range(n)]


class PendulumFitTests(unittest.TestCase):
    def test_recovers_coupling_and_frequency(self):
        truth = (-1.4, 9.8, 0.4, 0.6)
        runs = [pf.prepare(synthetic_rows(truth, t)) for t in (90, -80, 60)]
        params = pf.fit(runs, initial=(1.0, 9.0, 0.2, 0.3))
        self.assertAlmostEqual(params[0], truth[0], delta=0.1)
        self.assertAlmostEqual(params[1], truth[1], delta=0.1)
        self.assertLess(pf.rms(runs, params) * pf.ADC_CODE_PER_RAD, 0.6)

    def test_no_arm_motion_gives_no_swing(self):
        run = pf.prepare([dict(t=2000 * i, enc=0, adc=220, on=200 <= i < 250) for i in range(600)])
        self.assertTrue(all(abs(p) < 1e-12 for p in pf.simulate(run, (1.5, 9.7, 0.3, 0.5))))

    def test_rejects_empty_fit_input(self):
        with self.assertRaisesRegex(ValueError, 'fit.*empty'):
            pf.fit([])


class PendulumTimestampTests(unittest.TestCase):
    def setUp(self):
        self.rows = [dict(t=2000 * i, enc=0, adc=220, on=200 <= i < 250)
                     for i in range(1800)]

    def test_accepts_unwrapped_microseconds_with_large_offset(self):
        expected = pf.prepare(self.rows)
        for row in self.rows:
            row['t'] += 2 ** 40
        self.assertEqual(pf.prepare(self.rows), expected)

    def test_rejects_missing_timestamp(self):
        for index in (0, 220, len(self.rows) - 1):
            with self.subTest(index=index):
                rows = [dict(r) for r in self.rows]
                del rows[index]['t']
                with self.assertRaisesRegex(ValueError, 'timestamp'):
                    pf.prepare(rows)

    def test_rejects_nonfinite_timestamp(self):
        for value in (math.nan, math.inf, -math.inf):
            for index in (0, 220):
                with self.subTest(value=value, index=index):
                    rows = [dict(r) for r in self.rows]
                    rows[index]['t'] = value
                    with self.assertRaisesRegex(ValueError, 'timestamp.*finite'):
                        pf.prepare(rows)

    def test_rejects_nonnumeric_timestamp(self):
        for value in (None, '440000', True):
            with self.subTest(value=value):
                rows = [dict(r) for r in self.rows]
                rows[220]['t'] = value
                with self.assertRaisesRegex(ValueError, 'timestamp'):
                    pf.prepare(rows)

    def test_rejects_nonmonotonic_timestamp(self):
        for delta in (0, -2000):
            with self.subTest(delta=delta):
                rows = [dict(r) for r in self.rows]
                rows[220]['t'] = rows[219]['t'] + delta
                with self.assertRaisesRegex(ValueError, 'strictly increasing'):
                    pf.prepare(rows)

    def test_rejects_different_sample_period(self):
        for period_us in (1000, 4000, 2000.5):
            with self.subTest(period_us=period_us):
                rows = [dict(r, t=period_us * i) for i, r in enumerate(self.rows)]
                with self.assertRaisesRegex(ValueError, '2000 us'):
                    pf.prepare(rows)

    def test_rejects_dropped_sample(self):
        del self.rows[220]
        with self.assertRaisesRegex(ValueError, '2000 us'):
            pf.prepare(self.rows)

    def test_checks_timestamps_outside_fit_window(self):
        for index in (1, len(self.rows) - 1):
            with self.subTest(index=index):
                rows = [dict(r) for r in self.rows]
                rows[index]['t'] += 1
                with self.assertRaisesRegex(ValueError, '2000 us'):
                    pf.prepare(rows)


class PendulumCliValidationTests(unittest.TestCase):
    def assert_invalid_ids(self, fit_ids, check_ids, message):
        stdout, stderr = io.StringIO(), io.StringIO()
        args = ['--folder', str(Path(__file__).parent / 'no-pendulum-input'),
                '--fit-runs', *map(str, fit_ids), '--check-runs', *map(str, check_ids)]
        with redirect_stdout(stdout), redirect_stderr(stderr), \
                patch.object(pf, 'load_rows', wraps=pf.load_rows) as load, \
                patch.object(pf, 'fit', wraps=pf.fit) as fit:
            status = pf.main(args)
        self.assertEqual(status, 2)
        self.assertEqual(stdout.getvalue(), '')
        self.assertRegex(json.loads(stderr.getvalue())['error'], message)
        load.assert_not_called()
        fit.assert_not_called()

    def test_rejects_intersecting_fit_and_check_ids(self):
        self.assert_invalid_ids([301, 302], [303, 302], 'overlap.*302')

    def test_rejects_duplicate_fit_ids(self):
        self.assert_invalid_ids([301, 301], [302], 'fit.*duplicate')

    def test_rejects_duplicate_check_ids(self):
        self.assert_invalid_ids([301], [302, 302], 'check.*duplicate')

    def test_rejects_invalid_fit_ids(self):
        for run_id in (0, -1, 65536):
            with self.subTest(run_id=run_id):
                self.assert_invalid_ids([run_id], [], r'fit.*1\.\.65535')

    def test_rejects_invalid_check_ids(self):
        for run_id in (0, -1, 65536):
            with self.subTest(run_id=run_id):
                self.assert_invalid_ids([301], [run_id], r'check.*1\.\.65535')

    def test_rejects_empty_cli_fit_group(self):
        stderr = io.StringIO()
        with redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
            pf.main(['--folder', '.', '--fit-runs', '--check-runs', '302'])
        self.assertEqual(error.exception.code, 2)
        self.assertIn('--fit-runs', stderr.getvalue())

    def test_rejects_noninteger_cli_id(self):
        stderr = io.StringIO()
        with redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
            pf.main(['--folder', '.', '--fit-runs', '301.5'])
        self.assertEqual(error.exception.code, 2)
        self.assertIn('invalid int value', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
