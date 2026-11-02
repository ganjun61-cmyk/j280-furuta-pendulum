"""Offline, file-driven v4 core contracts against the exact integer oracle.

Only temporary ASCII simulator directories are written by these tests. No
hardware, serial port, vendor tool, or repository operation is involved.
"""
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import j280_balance_math as fx

Q, S = fx.Q, fx.S
DEFAULT = dict(max_compare=1000, comp_compare=175, blend_compare=25,
               comp_gain_q20=7 * Q)
ZERO_COMP = dict(max_compare=1000, comp_compare=0, blend_compare=25,
                 comp_gain_q20=0)


def identity_observer():
    return ([Q if i == j else 0 for i in range(4) for j in range(4)]
            + [0] * 4 + [Q, 0, 0, Q, 0, 0, 0, 0] + [-Q, 0, 0, 0])


class Timeline:
    """Sparse input edges; the bench asserts stable outputs on EVERY other edge.

    Each line is self-contained: gap, controls, all inputs, and full expected
    externally visible state. The Python oracle never reads simulator internals.
    """
    def __init__(self, latency=256):
        self.latency = latency
        self.clock = 0
        self.rows = []
        self.pending = None
        self.state = [0] * 4
        self.previous = 0
        self.u = [0] * 4
        self.fault = self.overrun = False
        self.reported_latency = 0
        self.ys = [0, 0]
        self.ds = [0, 0]

    def edge(self, gap=1, *, sample=False, clear=False, reset=False,
             y=(0, 0), co=None, config=None, reject=False, preset=None, ff=0, mode=0):
        assert gap >= 1
        co = identity_observer() if co is None else list(co)
        config = DEFAULT if config is None else config
        at = self.clock + gap
        if self.pending:
            assert at <= self.pending[0], "unobserved commit in vector generator"
        valid = False
        # Preset acts only while idle (busy before this edge) and fault-free.
        if preset and self.pending is None and not self.fault and not clear and not reset:
            self.state, self.previous = list(preset), 0
            self.ys = [preset[3], preset[3]]
            self.ds = [0, preset[2]]
        if clear or reset:
            self.state, self.previous, self.u = [0] * 4, 0, [0] * 4
            self.pending = None
            self.fault = self.overrun = False
            self.reported_latency = 0
            self.ys = [0, 0]
            self.ds = [0, 0]
        elif not self.fault:
            normal_commit = (self.pending is not None and at == self.pending[0]
                             and not self.pending[2]["fault"])
            if sample and self.pending and not normal_commit:
                self.reported_latency = at - self.pending[1]
                self.pending = None
                self.fault = self.overrun = valid = True
                self.u = [0] * 4
            else:
                if self.pending and at == self.pending[0]:
                    _, accepted, result = self.pending
                    self.pending = None
                    self.reported_latency = at - accepted
                    valid = True
                    self.fault = result["fault"]
                    self.state = result["state"]
                    self.u = [result[k] for k in
                              ("u_linear_q16", "u_compensated_q16",
                               "u_model_q16", "command")]
                    if not self.fault:
                        self.previous = result["model_input_q16"]
                    self.ys = list(result.get("filter_state", self.ys))
                if sample and not self.fault:
                    if reject:
                        result = dict(fault=True, state=list(self.state),
                                      u_linear_q16=0, u_compensated_q16=0,
                                      u_model_q16=0, command=0, model_input_q16=0)
                    else:
                        if mode in (8, 9, 10, 11):
                            filter_mode = 1
                            deadband = {8: 4, 9: 6, 10: 8, 11: 10}[mode]
                            dob_alpha = dob_gain = 0
                        elif mode in (12, 13, 14):
                            filter_mode = 1
                            deadband = 0
                            dob_alpha = {12: 3991, 13: 7739, 14: 7739}[mode]
                            dob_gain = {12: 65536, 13: 65536, 14: 98304}[mode]
                        else:
                            filter_mode, deadband = mode, 0
                            dob_alpha = dob_gain = 0
                        result = fx.step_q16(self.state, y, self.previous, co, **config,
                                             feedforward_q16=ff, filter_mode=filter_mode,
                                             filter_state=tuple(self.ys), pos_deadband=deadband,
                                             dob_alpha=dob_alpha, dob_gain_q16=dob_gain,
                                             dob_state=tuple(self.ds))
                        self.ys = list(result.get("filter_state", self.ys))
                        self.ds = list(result.get("dob_state", self.ds))
                    self.pending = (at + self.latency, at, result)
        self.clock = at
        ready = (not self.fault and not clear and not reset and
                 (self.pending is None or
                  (self.pending[0] == at + 1 and not self.pending[2]["fault"])))
        status = (bool(self.pending) | (ready << 1) | (valid << 2)
                  | (self.fault << 3) | (self.overrun << 4))
        controls = sample | (clear << 1) | (reset << 2) | (bool(preset) << 3)
        packed = sum((c & 0xffffffff) << (32 * i) for i, c in enumerate(co))
        values = [str(gap), str(controls), f"{y[0] & 0xffffffff:08x}",
                  f"{y[1] & 0xffffffff:08x}", f"{packed:0256x}",
                  str(config["max_compare"]), str(config["comp_compare"]),
                  str(config["blend_compare"]),
                  f'{config["comp_gain_q20"] & 0xffffffff:08x}', f"{status:02x}"]
        values += [f"{v & 0xffffffff:08x}" for v in self.state + self.u[:3]]
        values += [f"{self.u[3] & 0xffff:04x}", str(self.reported_latency)]
        values += [f"{v & 0xffffffff:08x}" for v in (preset or (0, 0, 0, 0))]
        values.append(f"{ff & 0xffffffff:08x}")
        values.append(str(mode))
        self.rows.append(" ".join(values))

    def finish(self, **inputs):
        assert self.pending
        self.edge(self.pending[0] - self.clock, **inputs)
        return dict(state=list(self.state), u=list(self.u), fault=self.fault)

    def run(self, y=(0, 0), co=None, config=None, mutate=False, reject=False, mode=0):
        self.edge(sample=True, y=y, co=co, config=config, reject=reject, mode=mode)
        if mutate:
            # Change every input class, including to invalid divider settings.
            self.edge(7, y=(fx.I32_MIN, fx.I32_MAX), co=[fx.COEFF_LIMIT + 1] * 32,
                      config=dict(max_compare=0, comp_compare=65535,
                                  blend_compare=0, comp_gain_q20=-Q))
        return self.finish()


class BalanceCoreRTLTests(unittest.TestCase):
    def simulate(self, timeline):
        sources = [ROOT / "rtl/j280_balance/j280_balance_core.v",
                   ROOT / "rtl/j280_state/j280_signed_divider.v",
                   ROOT / "sim/j280_balance/tb_balance_core.v"]
        for source in sources:
            self.assertTrue(source.is_file(), f"missing core contract source (RED): {source}")
        local = Path(os.environ.get("LOCALAPPDATA", str(Path.home())))
        candidates = [Path(os.environ[k]) for k in ("J280_IVERILOG_BIN", "IVERILOG_BIN")
                      if k in os.environ]
        candidates.append(local / "J280Tools/iverilog-msys2/ucrt64/bin")
        found = shutil.which("iverilog")
        if found:
            candidates.append(Path(found).parent)
        binary = next((p for p in candidates if (p / "iverilog.exe").is_file()), None)
        self.assertIsNotNone(binary, f"missing existing Icarus installation: {candidates}")
        build = Path(tempfile.gettempdir()) / "J280Build/balance-core-unit"
        build.mkdir(parents=True, exist_ok=True)
        run = Path(tempfile.mkdtemp(prefix="run-", dir=build))
        self.assertTrue(str(run).isascii(), run)
        for source in sources:
            shutil.copy2(source, run / source.name)
        (run / "vectors.txt").write_text("\n".join(timeline.rows) + "\n", encoding="ascii")
        env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ.get("PATH", ""))
        compiled = subprocess.run(
            [str(binary / "iverilog.exe"), "-g2001", "-Wall", "-s", "tb_balance_core",
             f"-Ptb_balance_core.RESULT_LATENCY={timeline.latency}", "-o", "core.vvp",
             *[s.name for s in sources]], cwd=run, env=env, capture_output=True,
            text=True, timeout=60)
        log = compiled.stdout + compiled.stderr
        (run / "compile.log").write_text(log, encoding="utf-8")
        self.assertEqual(compiled.returncode, 0, f"{run}\n{log}")
        self.assertEqual(log.strip(), "", f"compiler diagnostics: {run}\n{log}")
        result = subprocess.run([str(binary / "vvp.exe"), "core.vvp"], cwd=run,
                                env=env, capture_output=True, text=True, timeout=60)
        log = result.stdout + result.stderr
        (run / "simulation.log").write_text(log, encoding="utf-8")
        self.assertEqual(result.returncode, 0, f"{run}\n{log}")
        self.assertNotIn("FAIL", log, f"{run}\n{log}")
        self.assertIn(f"PASS tb_balance_core rows={len(timeline.rows)}", log, f"{run}\n{log}")

    def test_alpha_rate_filter_presets_bypass_and_saturation(self):
        # Drive x3 = alpha measurement: pred1 = pred3 = 0, correction copies e1
        # into x1 and x3, feedback uses only x3.
        co = [0] * 32
        co[20:28] = [Q, 0, 0, Q, 0, 0, 0, Q]
        co[28:32] = [0, 0, 0, -Q]
        t = Timeline()
        for mode in (0, 1, 2, 3, 4, 5, 6, 7):
            t.edge(clear=True)
            for v in (40 * S, 60 * S, 100 * S, 0, -40 * S, 1000 * S, 3 * S,
                      fx.I32_MAX, fx.I32_MIN):
                t.run((0, v), co=co, config=ZERO_COMP, mode=mode)
        # A two-section section rises slower but decays slower after the step.
        out = {}
        for mode in (3, 6):
            y1 = y2 = 0
            trace = []
            for k in range(60):
                used, y1, y2 = fx.filter_x3(1000 * S if k < 30 else 0, mode, y1, y2)
                trace.append(used)
            out[mode] = trace
        self.assertLess(out[3][29], out[6][29])
        self.assertGreater(out[3][59], out[6][59])
        self.assertGreater(out[3][59], 0)
        self.simulate(t)

    def test_position_deadband_presets_match_the_oracle(self):
        # candidate0 = theta measurement; feedback only the arm-error term, so
        # the four dead-zone widths are visible in u_linear.
        co = [0] * 32
        co[20] = Q
        co[28] = -Q
        t = Timeline()
        for mode in (8, 9, 10, 11):
            t.edge(clear=True)
            for n in (-20, -12, -10, -8, -6, -4, -3, -1, 0, 1, 3, 4, 6, 8, 10, 12, 20):
                t.run((n * S, 0), co=co, config=ZERO_COMP, mode=mode)
        self.simulate(t)

    def test_disturbance_observer_presets_match_the_oracle(self):
        co = identity_observer()
        co[28:32] = [-Q, 0, -Q, 0]
        t = Timeline()
        for mode in (12, 13, 14):
            t.edge(clear=True)
            for k in range(40):
                t.run(((k * 37 - 200) * S, (k * 11) % 17 - 8), co=co,
                      config=ZERO_COMP, mode=mode)
        self.simulate(t)

    def test_hand_cases_signed_compensation_quantization_and_saturated_inverse(self):
        t = Timeline()
        for value in (0, 1, -1, S // 3, -S // 3, S // 2, -S // 2,
                      5 * S, -5 * S, 25 * S - 1, 25 * S, 25 * S + 1,
                      -25 * S + 1, -25 * S, -25 * S - 1,
                      1000 * S, -1000 * S, fx.I32_MAX):
            t.run((value, 0))
        t.edge(clear=True)
        for value in (33 * S + 12345, -33 * S - 12345, S - 1, -S + 1):
            t.run((value, 0), config=dict(max_compare=40, comp_compare=7,
                                        blend_compare=25, comp_gain_q20=round(7 * Q / 25)))
        self.simulate(t)

    def test_round_once_negative_prediction_and_combined_correction_bias(self):
        t = Timeline()
        co = identity_observer()
        t.run((-1, -3), config=ZERO_COMP)
        co[20] = Q // 2
        out = t.run((0, -3), co=co, config=ZERO_COMP)
        self.assertEqual(out["state"][0], 0)
        # A dot product of four fractional terms distinguishes round-once
        # from both per-product rounding and arithmetic-right-shift rounding.
        co = [0] * 32
        co[20:28] = [Q // 2, Q // 2, Q // 2, -Q // 2,
                      -Q // 2, Q // 2, -Q // 2, -Q // 2]
        t.run((-3, -5), co=co, config=ZERO_COMP)
        co[:16] = [Q // 3, -Q // 7, Q // 5, -Q // 11] * 4
        co[20:28] = [0] * 8
        t.run((0, 0), co=co, config=ZERO_COMP)
        self.simulate(t)

    def test_previous_model_input_is_inverse_of_integer_pwm(self):
        t = Timeline()
        for first in (1000 * S, -1000 * S, S // 3, -S // 3):
            t.edge(clear=True)
            initial = t.run((first, 0))
            co = [0] * 32
            co[16:20] = [Q, -Q, Q // 3, -Q // 5]
            co[28] = -Q
            out = t.run(co=co)
            self.assertEqual(out["state"][0], initial["u"][2])
        self.simulate(t)

    def test_legal_parameter_limits_and_inverse_branch_boundary(self):
        t = Timeline()
        configs = [ZERO_COMP | dict(max_compare=1, blend_compare=250),
                   dict(max_compare=1, comp_compare=1, blend_compare=25,
                        comp_gain_q20=(Q + 12) // 25),
                   dict(max_compare=1000, comp_compare=250, blend_compare=25,
                        comp_gain_q20=10 * Q),
                   dict(max_compare=250, comp_compare=250, blend_compare=250,
                        comp_gain_q20=Q)]
        for config in configs:
            for value in (0, 1, -1, S // 3, -S // 3,
                          25 * S - 1, 25 * S, 25 * S + 1,
                          -25 * S + 1, -25 * S, -25 * S - 1,
                          1000 * S, -1000 * S):
                t.edge(clear=True)
                out = t.run((value, 0), config=config)
                self.assertFalse(out["fault"], (config, value))
        self.simulate(t)

    def test_wide_feedback_clips_before_narrowing_and_int32_endpoints_are_legal(self):
        t = Timeline()
        co = identity_observer()
        co[20:28] = [Q, 0] * 4
        co[28:32] = [-fx.COEFF_LIMIT] * 4
        for sign in (1, -1):
            t.edge(clear=True)
            t.run((sign * 1000 * S, 0), co=co, config=ZERO_COMP)
        for endpoint in (fx.I32_MIN, fx.I32_MAX):
            t.edge(clear=True)
            co = identity_observer()
            t.run((endpoint, 0), co=co, config=ZERO_COMP)
            co[20:28] = [0] * 8
            t.run((endpoint, 0), co=co, config=ZERO_COMP)
        self.simulate(t)

    def test_large_signed_products_cancel_before_single_rounding(self):
        t = Timeline()
        for endpoint in (fx.I32_MIN, fx.I32_MAX):
            t.edge(clear=True)
            seed = identity_observer()
            seed[20:28] = [Q, 0] * 4
            seed[28:32] = [-fx.COEFF_LIMIT] * 4
            t.run((endpoint, endpoint), co=seed, config=ZERO_COMP)
            co = ([fx.COEFF_LIMIT, -fx.COEFF_LIMIT, Q // 2, Q // 2] * 4
                  + [0] * 12 + [fx.COEFF_LIMIT, -fx.COEFF_LIMIT] * 2)
            out = t.run((endpoint, endpoint), co=co, config=ZERO_COMP)
            self.assertEqual(out["state"], [endpoint] * 4)
            self.assertEqual(out["u"], [0] * 4)
            co = identity_observer()
            co[20:28] = [fx.COEFF_LIMIT, -fx.COEFF_LIMIT] * 4
            co[28:32] = [fx.COEFF_LIMIT, -fx.COEFF_LIMIT] * 2
            measurement = -1 if endpoint == fx.I32_MIN else 0
            out = t.run((measurement, measurement), co=co, config=ZERO_COMP)
            self.assertEqual(out["state"], [endpoint] * 4)
            self.assertFalse(out["fault"])
        self.simulate(t)

    def test_numeric_prediction_error_and_correction_overflow_preserve_committed_state(self):
        t = Timeline()
        for kind in ("prediction+", "prediction-", "error+", "error-", "correction+", "correction-"):
            t.edge(clear=True)
            sign = -1 if kind.endswith("-") else 1
            endpoint = fx.I32_MAX if sign > 0 else fx.I32_MIN
            t.run((endpoint, 19), config=ZERO_COMP)
            co = identity_observer()
            y = (endpoint, 19)
            if kind.startswith("prediction"):
                co[0] = 2 * Q
            elif kind.startswith("error"):
                y = (fx.I32_MIN if sign > 0 else fx.I32_MAX, 19)
            else:
                co[22] = 2 * Q
                y = (endpoint - sign * 3 * S, 19)
                co[23] = 0
                co[4:8] = [Q, 0, 0, 0]  # pred1 is the endpoint too
                co[22] = -Q  # pred1 - e0 crosses the endpoint
            out = t.run(y, co=co, config=ZERO_COMP)
            self.assertTrue(out["fault"], kind)
            t.edge(sample=True, y=(1, 2))
            t.edge(300)
        self.simulate(t)

    def test_latch_all_inputs_and_allow_same_edge_commit_and_next_accept(self):
        t = Timeline()
        t.run((3 * S + 7, -2 * S), mutate=True)
        co = identity_observer()
        co[16:20] = [Q // 2, -Q // 3, Q // 7, -Q // 11]
        t.edge(sample=True, y=(S // 3, -S), co=co)
        for i in range(8):
            t.edge(t.pending[0] - t.clock - 1)
            # sample_ready must be high before the normal commit edge.
            co[16] = (i + 1) * Q // 13
            co[20] = Q // (i + 1)
            config = dict(max_compare=100 + i, comp_compare=i * 3,
                          blend_compare=30 + i,
                          comp_gain_q20=(i * 3 * Q + (30 + i) // 2) // (30 + i))
            t.edge(sample=True, y=((i - 4) * S + i, (3 - i) * S),
                   co=co, config=config)
        t.finish()
        self.simulate(t)

    def test_clear_and_reset_cancel_at_mac_divider_and_commit_edges(self):
        t = Timeline()
        for reset in (False, True):
            for offset in (1, 12, 65, 110, 145, 180, 255, 256):
                t.run((4 * S, -S))
                t.edge(sample=True, y=(-S // 3, 17))
                t.edge(offset, sample=True, clear=not reset, reset=reset)
                t.edge(300)
                # Nonzero Bd observes whether clear/reset really erased u_prev.
                co = [0] * 32
                co[16:20] = [Q, -Q, Q // 3, -Q // 5]
                t.run(co=co)
                t.run((S // 3, 1), mutate=True)
        self.simulate(t)

    def test_overrun_immediate_sticky_cancels_pending_and_clear_recovers(self):
        t = Timeline()
        for offset in (1, 17, 100, 150, 200, 255):
            t.edge(clear=True)
            t.run((2 * S, -S))
            t.edge(sample=True, y=(9 * S, 4 * S))
            t.edge(offset, sample=True, y=(-33 * S, 0))
            t.edge(sample=True)
            t.edge(300)
        t.edge(clear=True)
        t.run((-S // 3, 0))
        self.simulate(t)

    def test_reject_invalid_limits_slopes_and_coefficients_without_bad_division(self):
        t = Timeline()
        bad = [dict(max_compare=0), dict(max_compare=1126), dict(comp_compare=251),
               dict(max_compare=100, comp_compare=101), dict(blend_compare=0),
               dict(blend_compare=24), dict(blend_compare=251),
               dict(comp_gain_q20=-1), dict(comp_gain_q20=10 * Q + 1),
               dict(comp_gain_q20=6 * Q)]
        for change in bad:
            t.edge(clear=True)
            t.run((S, 0), config=DEFAULT | change, reject=True)
            t.edge(300, sample=True)
        for index in range(32):
            t.edge(clear=True)
            co = identity_observer()
            co[index] = (fx.COEFF_LIMIT + 1) * (-1 if index & 1 else 1)
            t.run((S, 0), co=co, reject=True)
        self.simulate(t)

    def test_long_random_sequences_all_states_and_all_control_outputs(self):
        rng = random.Random(0xB4A1A)
        t = Timeline()
        # Arbitrary stable dense matrices exercise every coefficient and state.
        for sequence in range(6):
            t.edge(clear=True)
            co = [rng.randint(-Q // 12, Q // 12) for _ in range(16)]
            co += [rng.randint(-Q // 200, Q // 200) for _ in range(4)]
            co += [rng.randint(-Q // 3, Q // 3) for _ in range(8)]
            co += [rng.randint(-8 * Q, 8 * Q) for _ in range(4)]
            blend = rng.randint(25, 250)
            comp = rng.randint(0, 250)
            config = dict(max_compare=rng.randint(max(1, comp), 1000),
                          comp_compare=comp, blend_compare=blend,
                          comp_gain_q20=(comp * Q + blend // 2) // blend)
            for i in range(200):
                result = t.run(tuple(rng.randint(-700 * S, 700 * S) for _ in range(2)),
                               co=co, config=config, mutate=i % 37 == 0)
                self.assertFalse(result["fault"], (sequence, i))
        # Also cover the exact provisional design over a long coherent stream.
        co = fx.design_coefficients()
        t.edge(clear=True)
        for i in range(500):
            result = t.run((rng.randint(-8 * S, 8 * S), rng.randint(-2 * S, 2 * S)), co=co)
            self.assertFalse(result["fault"], i)
        self.simulate(t)

    def test_preset_loads_state_zeroes_model_input_only_while_idle(self):
        t = Timeline()
        co = fx.design_coefficients()
        t.run((5 * S, -2 * S), co=co)
        t.run((7 * S, -S), co=co)
        t.edge(3, preset=(123 * S + 5, -40 * S, 900 * S, -1500 * S))
        out = t.run((120 * S, -38 * S), co=co)
        self.assertNotEqual(out["state"], [0] * 4)
        # Same edge as a sample start: the new prediction reads the preset.
        t.edge(5, sample=True, y=(119 * S, -37 * S), co=co, preset=(-S, 2 * S, -3 * S, 4 * S))
        t.finish()
        # Ignored while busy, and while faulted.
        t.edge(sample=True, y=(S, S), co=co)
        t.edge(40, preset=(9 * S, 9 * S, 9 * S, 9 * S))
        t.finish()
        t.run((S, 0), config=DEFAULT | dict(max_compare=0), reject=True)
        t.edge(4, preset=(9 * S, 9 * S, 9 * S, 9 * S))
        t.edge(clear=True)
        t.edge(2, preset=(-S // 3, S // 7, -S // 11, S // 13))
        t.run((0, 0), co=co)
        self.simulate(t)

    def test_latency_500_and_too_short_deadline_fault(self):
        t = Timeline(500)
        t.run((-S // 3, 0))
        self.simulate(t)
        t = Timeline(8)
        t.run((S, 0), reject=True)
        t.edge(20, sample=True)
        self.simulate(t)

    def test_feedforward_seeds_feedback_and_leaves_the_model_input(self):
        co = fx.design_coefficients((300., 200., 20., 5.), 0.5, (10., 6.))
        cfg = dict(max_compare=1000, comp_compare=175, blend_compare=62,
                   comp_gain_q20=fx.compensation_gain_q20(175, 62))
        rng = random.Random(66)
        t = Timeline()
        for _ in range(150):
            ff = rng.choice((0, rng.randint(-1000 * S, 1000 * S), rng.randint(-60 * S, 60 * S),
                             1000 * S, -1000 * S))
            y = (rng.randint(-50, 50) * S + rng.randint(0, S - 1), rng.randint(-40, 40) * S)
            t.edge(sample=True, y=y, co=co, config=cfg, ff=ff)
            t.finish()
        # |ff| beyond 1000<<16 is rejected like an invalid limit.
        t.edge(sample=True, y=(0, 0), co=co, config=cfg, ff=1125 * S + 1, reject=True)
        t.finish()
        self.simulate(t)


if __name__ == "__main__":
    unittest.main()
