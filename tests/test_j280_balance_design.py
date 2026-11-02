"""Balance design numerics: matrix exponential, Riccati, observer, simulation."""
import contextlib
import inspect
import io
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import j280_balance_design as bd  # noqa: E402


def residual(A, B, Q, R, X):
    BtX = bd.mm(bd.tr(B), X)
    S = bd.add(R, bd.mm(BtX, B))
    rhs = bd.add(bd.add(bd.mm(bd.mm(bd.tr(A), X), A), Q),
                 bd.mm(bd.mm(bd.tr(bd.mm(BtX, A)), bd.inv(S)), bd.mm(BtX, A)), -1)
    return max(abs(x - y) for rx, rr in zip(X, rhs) for x, y in zip(rx, rr)) / max(abs(x) for r in X for x in r)


class BalanceDesignTests(unittest.TestCase):
    def run_cli(self, *argv):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            status = bd.main(list(argv))
        return status, json.loads(output.getvalue())

    def test_compensation_inverse_covers_blend_boundary_and_both_signs(self):
        self.assertTrue(callable(getattr(bd, 'inverse_compensation', None)))
        for applied, expected in ((0, 0), (4, .5), (8, 1), (40, 33),
                                  (-4, -.5), (-8, -1), (-40, -33)):
            with self.subTest(applied=applied):
                self.assertAlmostEqual(bd.inverse_compensation(applied, 7), expected)
        self.assertAlmostEqual(bd.inverse_compensation(4, 6, blend=2), 1)
        self.assertEqual(bd.inverse_compensation(-3, 0), -3)

    def test_applied_duty_is_saturated_quantized_and_invertible(self):
        self.assertTrue(callable(getattr(bd, 'applied_duty', None)))
        for value in (-60, -1, -.5, 0, .5, 1, 60):
            with self.subTest(value=value):
                applied = bd.applied_duty(value, comp=7, u_max=40)
                expected = max(-40, min(40, value + 7 * max(-1, min(1, value))))
                self.assertAlmostEqual(applied, expected)
                virtual = bd.inverse_compensation(applied, 7)
                self.assertAlmostEqual(bd.applied_duty(virtual, comp=7, u_max=40), applied)
        self.assertEqual(bd.applied_duty(60, comp=7, u_max=40, pwm_step=3), 39)
        self.assertEqual(bd.applied_duty(-60, comp=7, u_max=40, pwm_step=3), -39)
        self.assertEqual(bd.applied_duty(.0625, comp=7, pwm_step=1), 1)
        self.assertEqual(bd.applied_duty(-.0625, comp=7, pwm_step=1), -1)

    def test_predictor_uses_inverse_of_previous_actual_duty(self):
        self.assertTrue(callable(getattr(bd, 'inverse_compensation', None)))
        for pwm_step, expected in ((0, 33), (3, 32)):
            inputs = []

            class InputProbe:
                def __mul__(self, value):
                    inputs.append(value)
                    return 0.0

            bd.simulate([0, -1000, 0, 0], [[0, 0], [0, 1], [0, 0], [0, 0]],
                        bd.eye(4), [[InputProbe()], [0], [0], [0]],
                        dict(adc_noise=0, pwm_step=pwm_step), seconds=2 * bd.T)
            self.assertEqual(inputs, [0, expected])

    def test_firmware_guards_have_strict_measured_limits_and_zero_reference(self):
        self.assertTrue(callable(getattr(bd, 'firmware_guard', None)))
        for enc in (-200, 0, 200):
            for angle in (-20, 0, 20):
                self.assertIsNone(bd.firmware_guard(enc, math.radians(angle)))
        self.assertEqual(bd.firmware_guard(201, 0), 'arm_count_limit')
        self.assertEqual(bd.firmware_guard(-201, 0), 'arm_count_limit')
        self.assertEqual(bd.firmware_guard(0, math.radians(20.001)), 'alpha_limit')
        self.assertEqual(bd.firmware_guard(0, math.radians(-20.001)), 'alpha_limit')

    def test_measured_guard_trips_before_physical_fall(self):
        self.assertIn('mode', inspect.signature(bd.simulate).parameters)
        d = bd.design((30, 200, 20, 20), .05)
        trip = bd.simulate(d['K'], d['L'], d['Ad'], d['Bd'],
                           dict(alpha0_deg=11, adc_scale=2, adc_noise=0),
                           seconds=.02, mode='firmware')
        self.assertTrue(trip['protection_trip'])
        self.assertFalse(trip['fell'])
        self.assertFalse(trip['accepted'])
        self.assertEqual(trip['termination'], 'protection_trip')
        self.assertEqual(trip['reason'], 'alpha_limit')
        self.assertEqual(trip['at_s'], 0)

    def test_b_negative_four_tap_recovers_in_plant_but_trips_firmware(self):
        self.assertIn('mode', inspect.signature(bd.simulate).parameters)
        d = bd.design((100, 200, 5, 50), .2)
        args = (d['K'], d['L'], d['Ad'], d['Bd'], dict(comp=9, tap=-4))
        plant = bd.simulate(*args, seconds=4, seed=1)
        guarded = bd.simulate(*args, seconds=4, seed=1, mode='firmware')
        self.assertEqual(plant['mode'], 'plant')
        self.assertTrue(plant['accepted'])
        self.assertGreater(plant['max_arm_deg'], 200 * 360 / 1040)
        self.assertTrue(guarded['protection_trip'])
        self.assertFalse(guarded['fell'])
        self.assertEqual(guarded['reason'], 'arm_count_limit')
        self.assertLess(guarded['at_s'], 3)

    def test_simulation_substeps_and_short_duration_are_explicit(self):
        self.assertIn('substeps', inspect.signature(bd.simulate).parameters)
        d = bd.design((30, 200, 20, 20), .05)
        args = (d['K'], d['L'], d['Ad'], d['Bd'], {})
        for substeps in (1, 8, 32):
            result = bd.simulate(*args, seconds=bd.T, substeps=substeps)
            self.assertTrue(math.isfinite(result['tail_alpha_rms_deg']))
            self.assertEqual(result['substeps'], substeps)
        for kwargs in (dict(substeps=0), dict(substeps=1.5), dict(seconds=0),
                       dict(seconds=float('nan')), dict(mode='unknown')):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                bd.simulate(*args, **kwargs)

    def test_aggregation_keeps_all_seeds_and_independent_metric_extrema(self):
        self.assertTrue(callable(getattr(bd, 'aggregate_runs', None)))
        runs = [dict(seed=1, accepted=True, fell=False, protection_trip=False,
                     max_arm_deg=4.8, tail_alpha_rms_deg=1.238, final_arm_deg=-3),
                dict(seed=2, accepted=True, fell=False, protection_trip=False,
                     max_arm_deg=16.3, tail_alpha_rms_deg=.9, final_arm_deg=2),
                dict(seed=3, accepted=False, fell=False, protection_trip=True,
                     max_arm_deg=10, final_arm_deg=-10)]
        summary = bd.aggregate_runs(runs)
        self.assertEqual(summary['runs'], runs)
        self.assertEqual(summary['failures'], 1)
        self.assertEqual(summary['protection_trips'], 1)
        self.assertEqual(summary['falls'], 0)
        arm = summary['metric_extrema']['max_arm_deg']
        self.assertEqual(arm['max'], dict(value=16.3, seed=2))
        rms = summary['metric_extrema']['tail_alpha_rms_deg']
        self.assertEqual(rms['max'], dict(value=1.238, seed=1))
        self.assertEqual(rms['runs_with_metric'], 2)
        self.assertEqual(summary['metric_extrema']['final_arm_deg']['max_abs'],
                         dict(value=-10, seed=3))

    def test_integration_comparison_pairs_seeds_and_reports_missing_metrics(self):
        self.assertTrue(callable(getattr(bd, 'compare_integration', None)))
        coarse = [dict(seed=1, accepted=True, max_arm_deg=10, tail_alpha_rms_deg=1),
                  dict(seed=2, accepted=True, max_arm_deg=8, tail_alpha_rms_deg=.5)]
        refined = [dict(seed=2, accepted=False, max_arm_deg=9),
                   dict(seed=1, accepted=True, max_arm_deg=12, tail_alpha_rms_deg=1.2)]
        report = bd.compare_integration(coarse, refined)
        self.assertFalse(report['qualified_bound'])
        self.assertEqual(report['acceptance_changed_seeds'], [2])
        self.assertAlmostEqual(report['max_abs_metric_delta']['tail_alpha_rms_deg']['value'], .2)
        self.assertEqual(report['max_abs_metric_delta']['max_arm_deg'], dict(value=2, seed=1))
        self.assertEqual(report['pairs'][1]['missing_metrics'], ['tail_alpha_rms_deg'])

    def test_refined_pendulum_integration_approaches_independent_rk4(self):
        # No feedback/arm motion: integrate the direct pendulum ODE independently.
        angle, speed, h = math.radians(3), 0.0, .2 / 4000

        def rhs(a, v):
            return v, 9.724 ** 2 * math.sin(a) - .085 * v - .84 * math.tanh(v / .02)

        for _ in range(4000):
            k1 = rhs(angle, speed)
            k2 = rhs(angle + h * k1[0] / 2, speed + h * k1[1] / 2)
            k3 = rhs(angle + h * k2[0] / 2, speed + h * k2[1] / 2)
            k4 = rhs(angle + h * k3[0], speed + h * k3[1])
            angle += h * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6
            speed += h * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6
        d = bd.design((30, 200, 20, 20), .05)
        args = ([0] * 4, d['L'], d['Ad'], d['Bd'], dict(comp=0, adc_noise=0))
        coarse = bd.simulate(*args, seconds=.2, substeps=1)
        refined = bd.simulate(*args, seconds=.2, substeps=32)
        reference = math.degrees(angle)
        self.assertGreater(abs(coarse['max_alpha_deg'] - reference), .08)
        self.assertLess(abs(refined['max_alpha_deg'] - reference), .01)

    def test_closed_loop_grid_report_measures_real_sensitivity(self):
        d = bd.design((100, 200, 5, 50), .2)
        case = dict(comp=9, gain=.7, adc_scale=.8, adc_zero_deg=1,
                    backlash_counts=8, friction=1.3)
        args = (d['K'], d['L'], d['Ad'], d['Bd'], case)
        coarse = bd.simulate(*args, seconds=20, seed=1, mode='firmware', substeps=8)
        refined = bd.simulate(*args, seconds=20, seed=1, mode='firmware', substeps=32)
        report = bd.compare_integration([coarse], [refined])
        self.assertTrue(coarse['accepted'])
        self.assertTrue(refined['accepted'])
        self.assertGreater(report['max_abs_metric_delta']['tail_alpha_rms_deg']['value'], 0)
        self.assertFalse(report['qualified_bound'])

    def test_cli_reports_firmware_acceptance_and_all_runs(self):
        status, out = self.run_cli('--seconds', '.02', '--seeds', '2')
        self.assertEqual(status, 0)
        self.assertEqual(out.get('validation_mode'), 'firmware')
        self.assertTrue(out['validation_passed'])
        self.assertEqual(out['required_runs'], 50)
        self.assertEqual(out['required_failures'], 0)
        self.assertEqual(out['guards']['arm_reference_count'], 0)
        self.assertEqual(out['guards']['alpha_limit_deg'], 20)
        self.assertEqual(out['guards']['arm_limit_counts'], 200)
        self.assertFalse(out['integration']['qualified_bound'])
        self.assertEqual(len(out['sweep']['nominal']['runs']), 2)
        self.assertEqual(out['closed_loop_rho_kind'], 'finite_matrix_power_estimate')
        self.assertEqual(out['closed_loop_rho'], out['closed_loop_matrix_power_radius_estimate'])

    def test_cli_required_guard_trips_fail_exit_status(self):
        status, out = self.run_cli('--r', '500', '--seconds', '3', '--seeds', '1')
        self.assertNotEqual(status, 0)
        self.assertFalse(out['validation_passed'])
        self.assertGreater(out['required_failures'], 0)
        self.assertGreater(out['required_protection_trips'], 0)

    def test_cli_reports_paired_refinement_without_claiming_a_bound(self):
        self.assertIn('substeps', inspect.signature(bd.simulate).parameters)
        status, out = self.run_cli('--seconds', '.02', '--seeds', '1',
                                   '--substeps', '4', '--refine-substeps', '16', '--mode', 'plant')
        self.assertEqual(status, 0)
        self.assertEqual(out['validation_mode'], 'plant')
        self.assertEqual(out['integration']['coarse_substeps'], 4)
        self.assertEqual(out['integration']['refined_substeps'], 16)
        self.assertFalse(out['integration']['qualified_bound'])
        self.assertIn('tail_alpha_rms_deg', out['integration']['max_abs_metric_delta'])
        nominal = out['sweep']['nominal']
        self.assertEqual(nominal['refined']['runs'][0]['substeps'], 16)
        self.assertEqual(nominal['integration_comparison']['pairs'][0]['seed'], 1)

    def test_cli_rejects_invalid_numerical_arguments_before_simulation(self):
        self.assertIn('substeps', inspect.signature(bd.simulate).parameters)
        invalid = [('--r', 'nan'), ('--r', '0'), ('--q', '1', '-1', '1', '1'),
                   ('--qw', 'inf', '1'), ('--comp', '-1'), ('--seconds', '0'),
                   ('--seconds', 'nan'), ('--seeds', '0'), ('--substeps', '0'),
                   ('--refine-substeps', '8'), ('--blend', '0'), ('--pwm-step', '-1'),
                   ('--u-max', '101')]
        for argv in invalid:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as exc:
                    bd.main(list(argv))
                self.assertEqual(exc.exception.code, 2)

    def test_external_controller_owns_state_and_applied_duty(self):
        self.assertIn('controller', inspect.signature(bd.simulate).parameters)
        samples = []

        def controller(y_si, case_cfg):
            samples.append((y_si, dict(case_cfg)))
            return dict(xhat_si=[0, y_si[1], 0, 0], u_linear_percent=0,
                        u_applied_percent=20, u_model_percent=13, fault=None)

        result = bd.simulate(None, None, None, None, dict(adc_noise=0), seconds=.02,
                             mode='firmware', controller=controller)
        self.assertTrue(result['accepted'])
        self.assertEqual(len(samples), 10)
        self.assertAlmostEqual(samples[0][0][1], round(bd.ADC_PER_RAD * math.radians(3)) / bd.ADC_PER_RAD)
        self.assertEqual(samples[0][1]['comp'], 7)
        self.assertEqual(result['tail_u_rms'], 20)

    def test_external_controller_fault_stops_without_reporting_physical_fall(self):
        self.assertIn('controller', inspect.signature(bd.simulate).parameters)
        result = bd.simulate(None, None, None, None, {}, seconds=.02,
                             controller=lambda y, cfg: dict(fault='fixed_overflow'))
        self.assertFalse(result['accepted'])
        self.assertFalse(result['fell'])
        self.assertEqual(result['termination'], 'controller_fault')
        self.assertEqual(result['reason'], 'fixed_overflow')
        self.assertEqual(result['at_s'], 0)

    def test_external_duty_outside_case_limit_is_rejected(self):
        self.assertIn('controller', inspect.signature(bd.simulate).parameters)
        def controller(y, cfg):
            return dict(xhat_si=[0] * 4, u_linear_percent=0,
                        u_applied_percent=41, u_model_percent=34, fault=None)
        with self.assertRaises(ValueError):
            bd.simulate(None, None, None, None, {}, controller=controller)

    def test_stopped_run_saturation_fraction_uses_applied_intervals(self):
        def controller(y, cfg):
            return dict(xhat_si=[0] * 4, u_linear_percent=1000,
                        u_applied_percent=40, u_model_percent=33, fault=None)
        result = bd.simulate(None, None, None, None, dict(adc_noise=0),
                             seconds=1, mode='firmware', controller=controller)
        self.assertTrue(result['protection_trip'])
        self.assertEqual(result['saturated_fraction'], 1)

    def test_discretize_double_integrator(self):
        Ad, Bd = bd.discretize([[0, 1], [0, 0]], [[0], [1]], 0.1)
        for got, want in zip([x for r in Ad for x in r] + [r[0] for r in Bd], [1, 0.1, 0, 1, 0.005, 0.1]):
            self.assertAlmostEqual(got, want, places=12)

    def test_riccati_solution_satisfies_equation_for_pendulum_model(self):
        A, B = bd.continuous()
        Ad, Bd = bd.discretize(A, B)
        Q = [[30, 0, 0, 0], [0, 200, 0, 0], [0, 0, 20, 0], [0, 0, 0, 20]]
        X = bd.dare(Ad, Bd, Q, [[0.05]])
        self.assertLess(residual(Ad, Bd, Q, [[0.05]], X), 1e-9)

    def test_designs_are_stable_with_upright_sign(self):
        for q, r in (((30, 200, 20, 20), 0.05), ((100, 200, 5, 50), 0.2)):
            d = bd.design(q, r)
            self.assertLess(d['rho'], 1.0)
            self.assertLess(d['K'][1], 0)  # pendulum leaning ADC-up needs u > 0 (arm count-up)
            observer = bd.add(d['Ad'], bd.mm(bd.mm(d['L'], [[1, 0, 0, 0], [0, 1, 0, 0]]), d['Ad']), -1)
            self.assertLess(bd.spectral_radius(observer), 1.0)

    def test_nominal_release_balances_and_no_feedback_falls(self):
        d = bd.design((30, 200, 20, 20), 0.05)
        ok = bd.simulate(d['K'], d['L'], d['Ad'], d['Bd'], {}, 8.0)
        self.assertFalse(ok['fell'])
        self.assertLess(ok['tail_alpha_rms_deg'], 1.5)
        fell = bd.simulate([0, 0, 0, 0], d['L'], d['Ad'], d['Bd'], dict(comp=0.0), 8.0)
        self.assertTrue(fell['fell'])
        self.assertLess(fell['at_s'], 2.0)


if __name__ == '__main__':
    unittest.main()
