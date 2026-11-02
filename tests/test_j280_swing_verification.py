"""The v5 verifier must cover every input and reject incomplete evidence."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SCRIPT = ROOT / 'scripts/j280_verify_swing.py'


def module():
    spec = importlib.util.spec_from_file_location('swing_verify', SCRIPT)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


class SwingVerificationTests(unittest.TestCase):
    def test_v4_verifier_keeps_its_own_identity(self):
        module()
        spec = importlib.util.spec_from_file_location('balance_verify_check', ROOT / 'scripts/j280_verify_balance.py')
        v4 = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(v4)
        self.assertEqual((v4.NAME, v4.EVIDENCE_ENV), ('j280_balance', 'J280_BALANCE_EVIDENCE_DIR'))
        self.assertEqual(set(v4.SCENARIOS), {'stop', 'angle'})

    def test_evidence_must_exist_and_match_its_wire_bytes(self):
        verify = module()
        with tempfile.TemporaryDirectory() as folder, self.assertRaises(ValueError):
            verify.check_evidence(Path(folder))
        with tempfile.TemporaryDirectory() as folder:
            for name in verify.SCENARIOS:
                path = Path(folder) / name
                path.mkdir()
                for file in verify.EVIDENCE_FILES:
                    (path / file).write_bytes(b'x')
                (path / 'result.json').write_text(json.dumps({'frames': 1, 'wire_sha256': '0'}),
                                                  encoding='utf-8')
            with self.assertRaises(ValueError):
                verify.check_evidence(Path(folder))

    def test_scenarios_match_the_integration_test(self):
        sys.path.insert(0, str(ROOT / 'tests'))
        import test_j280_swing_integration as integration
        expected = {name: (state, reason) for name, state, reason in integration.SCENARIOS.values()}
        self.assertEqual({k: v[:2] for k, v in module().SCENARIOS.items()}, expected)

    def test_source_list_covers_every_firmware_input(self):
        names = {p.relative_to(ROOT).as_posix() for p in module().source_files()}
        for required in ('rtl/j280_swing/j280_swing_top.v', 'rtl/j280_swing/run.tcl',
                         'rtl/j280_swing/j280_swing.cst', 'rtl/j280_swing/j280_swing.sdc',
                         'rtl/j280_balance/j280_balance_core.v', 'rtl/j280_motor_test/j280_uart_rx.v',
                         'sim/j280_swing/tb_swing_top.v', 'sim/j280_swing/tb_swing_core.v',
                         'scripts/j280_swing_math.py', 'scripts/j280_swing_protocol.py',
                         'scripts/j280_balance_protocol.py', 'tests/test_j280_swing_integration.py',
                         'tests/test_j280_balance_guard_rtl.py', 'tests/test_j280_board_session.py',
                         'tests/test_j280_balance_protocol.py'):
            self.assertIn(required, names)
        for p in (ROOT / 'rtl/j280_swing').glob('*'):
            self.assertIn(p.relative_to(ROOT).as_posix(), names)

    def test_run_tcl_and_project_list_every_rtl_module(self):
        # The v5 run.tcl is a sealed archive; the shipped firmware builds
        # j280_swing_hyb_top from scripts/j280_build_gear.py FILES.
        import j280_build_gear as gear
        staged = {n for n in gear.FILES if n.endswith('.v')}
        text = (ROOT / 'rtl/j280_swing/run_hyb.tcl').read_text(encoding='ascii')
        listed = {line.split()[1] for line in text.splitlines() if line.startswith('add_file')}
        self.assertEqual({n for n in listed if n.endswith('.v')}, staged)
        project = (ROOT / 'rtl/j280_swing/j280_swing_hyb.gprj').read_text(encoding='utf-8')
        self.assertEqual({Path(x.split('"')[0]).name for x in project.split('File path="')[1:]
                          if x.split('"')[0].endswith('.v')}, staged)


if __name__ == '__main__':
    unittest.main()
