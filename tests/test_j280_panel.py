"""Board panel logic against the scripted v5 fake board; the window itself is only smoke-tested."""
from contextlib import nullcontext
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'tests'))
import j280_balance as profiles  # noqa: E402
import j280_panel as panel  # noqa: E402
from test_j280_board_session import Clock  # noqa: E402
from test_j280_swing import FakeBoard5, SAFE, ARMED, SWING, BALANCE  # noqa: E402
from test_j280_v6 import FakeBoard6  # noqa: E402


class PanelLogicTests(unittest.TestCase):
    def paths(self, tmp):
        return dict(csv=Path(tmp) / 's.csv', raw=Path(tmp) / 's.bin', summary=Path(tmp) / 's-summary.json')

    def test_every_preset_loads(self):
        for name, rel in panel.PRESETS:
            cfg = profiles.load_config(ROOT / rel)
            self.assertTrue(cfg['purpose'], name)

    def test_swing_gains2_backlash_session_writes_files_and_stats(self):
        clock = Clock()
        cfg = profiles.load_config(ROOT / 'docs/data/swing-board-2026-10-18/swing-610-config.json')
        fake = FakeBoard5(clock, [(SAFE, 5, None), (ARMED, 5, None), (SWING, 60, None), (BALANCE, 300, None),
                                  (SAFE, 1, 2)])
        with tempfile.TemporaryDirectory() as tmp:
            paths = self.paths(tmp)
            seen = []
            summary = panel.run_profile(lambda: nullcontext(fake), cfg, paths, clock=clock,
                                        on_recorder=seen.append)
            self.assertTrue(seen and paths['csv'].stat().st_size and paths['raw'].stat().st_size)
            saved = json.loads(paths['summary'].read_text(encoding='utf-8'))
        self.assertEqual(saved['result']['end_reason'], 'stop_key')
        self.assertEqual([c for _, c, _ in fake.writes if c not in (2, 3)][:3], [1, 4, 5])
        stats = summary['balance_stats']
        self.assertEqual(stats['balance_frames'], 300)
        self.assertIn('pend_std_code', stats)
        text = panel.describe(summary)
        self.assertIn('按了 SW2', text)
        self.assertIn('摆杆晃动', text)

    def test_cancel_aborts_and_errors_are_reported_not_raised(self):
        clock = Clock()
        cfg = profiles.load_config(ROOT / 'docs/data/balance-board-2026-10-18b/balance-745-config.json')
        fake = FakeBoard5(clock, [(SAFE, 10 ** 6, None)])
        with tempfile.TemporaryDirectory() as tmp:
            summary = panel.run_profile(lambda: nullcontext(fake), cfg, self.paths(tmp), clock=clock,
                                        cancel=lambda: clock.now > 1.0)
        self.assertEqual(summary['result']['phase'], 'cancelled')
        self.assertIn('电脑中止', panel.describe(summary))
        clock = Clock()
        with tempfile.TemporaryDirectory() as tmp:
            summary = panel.run_profile(lambda: nullcontext(FakeBoard5(clock, start_state=255)), cfg,
                                        self.paths(tmp), clock=clock)
        self.assertIn('not SAFE', summary['error'])
        self.assertIn('出错', panel.describe(summary))

    def test_v6_profile_session_and_motion_hints(self):
        clock = Clock()
        cfg = profiles.load_config(ROOT / 'docs/data/v6-board-2026-10-19/v6-811-config.json')
        fake = FakeBoard6(clock, [(SAFE, 5, None), (ARMED, 5, None), (SWING, 60, None), (BALANCE, 400, None),
                                  (SAFE, 1, 2)])
        with tempfile.TemporaryDirectory() as tmp:
            summary = panel.run_profile(lambda: nullcontext(fake), cfg, self.paths(tmp), clock=clock)
        self.assertEqual(summary['result']['end_reason'], 'stop_key')
        self.assertTrue(all(d[2] == 6 for _, _, d in fake.writes))
        row = dict(state='BALANCE', config_loaded=True, arm_on=True, state_elapsed_ms=4000,
                   traj_settled=True, traj_rotating=False)
        self.assertIn('SW3/SW4', panel.hint(row, cfg))
        self.assertIn('拨码 2 拨到 OFF', panel.hint(dict(row, traj_rotating=True), cfg))
        self.assertNotIn('SW3', panel.hint(dict(row, traj_settled=False), cfg))

    def test_check_link_reads_only_and_explains(self):
        clock = Clock()
        fake = FakeBoard6(clock, [(SAFE, 10 ** 6, None)])
        text = panel.check_link(lambda: nullcontext(fake), clock=clock)
        self.assertIn('板子在线', text)
        self.assertEqual(fake.writes, [])
        def broken():
            raise OSError('port busy')
        self.assertIn('连不上', panel.check_link(broken, clock=clock))

    def test_program_firmware_reports_programmer_result(self):
        class Done:
            returncode, stdout, stderr = 0, ' User Code is: 0x00004EC1\n Finished.\n', ''
        calls = []
        ok, text = panel.program_firmware(fs=__file__, run=lambda cmd, **kw: calls.append(cmd) or Done())
        self.assertTrue(ok)
        self.assertIn('0x00004EC1', text)
        self.assertIn('-Fs', calls[0])
        ok, text = panel.program_firmware(fs=ROOT / 'missing.fs')
        self.assertFalse(ok)

    def test_hints_follow_the_board_state(self):
        swing = profiles.load_config(ROOT / 'docs/data/swing-board-2026-10-18/swing-605-config.json')
        plain = profiles.load_config(ROOT / 'docs/data/balance-board-2026-10-18b/balance-741-config.json')
        row = dict(state='SAFE', config_loaded=False, arm_on=True, state_elapsed_ms=0)
        self.assertIn('OFF', panel.hint(row, swing))
        self.assertIn('载入', panel.hint(row, plain))
        self.assertIn('ON', panel.hint(dict(row, config_loaded=True), swing))
        self.assertIn('起摆', panel.hint(dict(row, state='ARMED', config_loaded=True), swing))
        self.assertIn('扶到竖直', panel.hint(dict(row, state='ARMED', config_loaded=True), plain))
        self.assertIn('SW5', panel.hint(dict(row, state='FAULT'), plain))
        self.assertIn('2.5 s', panel.hint(dict(row, state='BALANCE', state_elapsed_ms=2500), plain))
        self.assertAlmostEqual(panel.pendulum_deg(737), 0.0, places=3)
        self.assertAlmostEqual(abs(panel.pendulum_deg(221)), 180.0, places=3)

    def test_window_builds_hidden(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as error:             # no display: nothing to smoke-test
            self.skipTest(f'no Tk: {error}')
        try:
            root.withdraw()
            p = panel.Panel(root)
            root.update()
            self.assertIn('自动起摆', p.choice.get())
        finally:
            root.destroy()


if __name__ == '__main__':
    unittest.main()
