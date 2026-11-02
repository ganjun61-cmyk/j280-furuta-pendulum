"""The balance verifier must reject incomplete or inconsistent evidence."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SCRIPT = ROOT / 'scripts/j280_verify_balance.py'


def module():
    spec = importlib.util.spec_from_file_location('balance_verify', SCRIPT)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


class BalanceVerificationTests(unittest.TestCase):
    def test_missing_evidence_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder, self.assertRaises(ValueError):
            module().check_evidence(Path(folder))

    def test_result_that_does_not_match_its_wire_bytes_is_rejected(self):
        verify = module()
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

    def test_archive_requires_verified_manifest(self):
        verify = module()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaises(ValueError):
                verify.archive_verified_run(root, root / 'archive')
            (root / 'verification.json').write_text(json.dumps({
                'build_performed': True, 'source_sha256': {}, 'python_tests_passed': 1,
                'board_accessed': False}), encoding='utf-8')
            with self.assertRaises(ValueError):
                verify.archive_verified_run(root, root / 'archive')
            self.assertFalse((root / 'archive').exists())

    def test_source_list_covers_every_firmware_and_balance_input(self):
        names = {p.relative_to(ROOT).as_posix() for p in module().source_files()}
        for required in ('rtl/j280_balance/j280_balance_top.v', 'rtl/j280_balance/run.tcl',
                         'rtl/j280_balance/j280_balance.cst', 'rtl/j280_balance/j280_balance.sdc',
                         'rtl/j280_motor_test/j280_uart_rx.v', 'rtl/j280_motor_test/j280_local_inputs.v',
                         'sim/j280_balance/tb_balance_top.v', 'scripts/j280_balance_math.py',
                         'scripts/j280_balance_protocol.py', 'tests/test_j280_balance_integration.py'):
            self.assertIn(required, names)
        for p in (ROOT / 'rtl/j280_balance').glob('*'):
            self.assertIn(p.relative_to(ROOT).as_posix(), names)

    def test_run_tcl_lists_every_rtl_module_used_by_the_top(self):
        text = (ROOT / 'rtl/j280_balance/run.tcl').read_text(encoding='ascii')
        staged = {p.name for p in module().SHARED} | {
            p.name for p in (ROOT / 'rtl/j280_balance').glob('*.v')}
        listed = set(line.split()[1] for line in text.splitlines() if line.startswith('add_file'))
        self.assertEqual({n for n in listed if n.endswith('.v')}, staged)


if __name__ == '__main__':
    unittest.main()
