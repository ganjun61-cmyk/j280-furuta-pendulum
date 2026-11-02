"""Offline signed continuous PWM contract simulations; no board or serial access."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
LOCAL = Path(os.environ.get("LOCALAPPDATA", str(Path.home())))


def _iverilog_bin():
    candidates = [Path(os.environ[k]) for k in ("J280_IVERILOG_BIN", "IVERILOG_BIN")
                  if k in os.environ]
    candidates.append(LOCAL / "J280Tools/iverilog-msys2/ucrt64/bin")
    found = shutil.which("iverilog")
    if found:
        candidates.append(Path(found).parent)
    return next((p for p in candidates if (p / "iverilog.exe").is_file()), candidates[0])


BIN = _iverilog_bin()
BUILD = Path(tempfile.gettempdir()) / "J280Build/balance-pwm-unit"
SOURCES = (ROOT / "rtl/j280_balance/j280_balance_pwm.v",
           ROOT / "sim/j280_balance/tb_balance_pwm.v")


class BalancePWMRTLTests(unittest.TestCase):
    def simulate(self, case, period=2500, max_compare=1000):
        for source in SOURCES:
            self.assertTrue(source.is_file(), f"missing PWM contract source (RED): {source}")
        self.assertTrue((BIN / "iverilog.exe").is_file(), f"missing Icarus: {BIN}")
        BUILD.mkdir(parents=True, exist_ok=True)
        run = Path(tempfile.mkdtemp(prefix=f"case{case}-", dir=BUILD))
        self.assertTrue(str(run).isascii(), run)
        for source in SOURCES:
            shutil.copy2(source, run / source.name)
        env = dict(os.environ, PATH=str(BIN) + os.pathsep + os.environ.get("PATH", ""))
        command = [str(BIN / "iverilog.exe"), "-g2001", "-Wall", "-s", "tb_balance_pwm",
                   f"-Ptb_balance_pwm.PERIOD={period}",
                   f"-Ptb_balance_pwm.MAX_COMPARE={max_compare}",
                   "-o", "sim.vvp", *[source.name for source in SOURCES]]
        compiled = subprocess.run(command, cwd=run, env=env, capture_output=True,
                                  text=True, timeout=60)
        compile_log = compiled.stdout + compiled.stderr
        (run / "compile.log").write_text(compile_log, encoding="utf-8")
        self.assertEqual(compiled.returncode, 0, f"{run}\n{compile_log}")
        self.assertEqual(compile_log.strip(), "", f"compiler diagnostics: {run}\n{compile_log}")
        result = subprocess.run([str(BIN / "vvp.exe"), "sim.vvp", f"+CASE={case}"],
                                cwd=run, env=env, capture_output=True, text=True, timeout=60)
        log = result.stdout + result.stderr
        (run / "simulation.log").write_text(log, encoding="utf-8")
        self.assertEqual(result.returncode, 0, f"{run}\n{log}")
        self.assertNotIn("FAIL", log, f"{run}\n{log}")
        self.assertIn(f"PASS tb_balance_pwm CASE={case} PERIOD={period} MAX_COMPARE={max_compare}",
                      log, f"{run}\n{log}")

    def test_default_50mhz_carrier_signed_widths_and_zero_brake(self):
        self.simulate(1)

    def test_small_odd_and_even_carriers_full_period_widths(self):
        for period, maximum in ((17, 6), (16, 15), (9, 4)):
            with self.subTest(period=period, maximum=maximum):
                self.simulate(1, period, maximum)

    def test_latest_pending_and_coincident_boundary_update_wins(self):
        self.simulate(2, 17, 6)

    def test_reversal_blanks_exactly_one_period_even_with_new_updates(self):
        self.simulate(3, 17, 6)

    def test_immediate_stop_reset_flush_and_fresh_update_after_rearm(self):
        self.simulate(4, 17, 6)

    def test_illegal_signed_magnitudes_sticky_reject_and_rearm(self):
        for period, maximum in ((2500, 1000), (2500, 1125), (17, 6)):
            with self.subTest(period=period, maximum=maximum):
                self.simulate(5, period, maximum)


if __name__ == "__main__":
    unittest.main()
