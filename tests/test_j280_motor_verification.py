"""The motor verifier must reject incomplete integration evidence."""
import importlib.util
import binascii
import json
from pathlib import Path
import sys
import struct
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SCRIPT = ROOT / 'scripts/j280_verify_motor_test.py'


class MotorVerificationTests(unittest.TestCase):
    def module(self):
        self.assertTrue(SCRIPT.exists(), 'implement motor verification entry point')
        spec = importlib.util.spec_from_file_location('motor_verify', SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_missing_top_evidence_is_rejected(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                module.check_evidence(Path(folder))

    def test_clean_looking_counts_without_uart_bytes_are_rejected(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as folder:
            for name in ('default', 'stops'):
                path = Path(folder) / name
                path.mkdir()
                (path / 'result.json').write_text(json.dumps({'frames': 10}), encoding='utf-8')
                (path / 'wire.bin').write_bytes(b'')
            with self.assertRaises(ValueError):
                module.check_evidence(Path(folder))

    def test_archive_requires_verified_manifest(self):
        module = self.module()
        self.assertTrue(hasattr(module, 'archive_verified_run'), 'add verified-run sealing')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaises(ValueError):
                module.archive_verified_run(root, root / 'archive')
            self.assertFalse((root / 'archive').exists())

    def test_archive_rejects_changed_sources_before_creating_directory(self):
        module = self.module()
        self.assertTrue(hasattr(module, 'archive_verified_run'), 'add verified-run sealing')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'verification.json').write_text(json.dumps({
                'build_performed': True, 'source_sha256': {}, 'python_tests_passed': 1,
                'board_accessed': False}), encoding='utf-8')
            with self.assertRaises(ValueError):
                module.archive_verified_run(root, root / 'archive')
            self.assertFalse((root / 'archive').exists())

    def make_verified_run(self, module, root):
        repo, run = root / 'repo', root / 'run'
        repo.mkdir()
        (repo / 'fixture.v').write_text('module fixture; endmodule\n', encoding='ascii')
        self.addCleanup(patch.stopall)
        patch.object(module, 'ROOT', repo).start()
        patch.object(module, 'source_files', return_value=[repo / 'fixture.v']).start()
        patch.object(module, 'check_timing_report', return_value={}).start()
        patch.object(module, 'check_reset_timing', return_value={}).start()
        source, build = run / 'source', run / 'gowin'
        source.mkdir(parents=True)
        (source / 'fixture.v').write_bytes((repo / 'fixture.v').read_bytes())
        build.mkdir()
        (build / 'fixture.v').write_bytes((repo / 'fixture.v').read_bytes())
        pnr = build / 'impl/pnr'
        pnr.mkdir(parents=True)
        fs = pnr / 'j280_motor_test.fs'
        fs.write_bytes(b'fixture bitstream\n')
        (pnr / 'j280_motor_test_tr_content.html').write_text('timing fixture', encoding='utf-8')
        (pnr / 'j280_motor_test.rpt.txt').write_text('pnr fixture', encoding='ascii')
        (build / 'build-console.log').write_text('build fixture', encoding='ascii')
        (run / 'python-tests.log').write_text('Ran 1 test\nOK\n', encoding='ascii')
        payload = struct.pack('<BBHBBHHHHH', 3, 1, 1, 1, 0, 125, 4, 60, 3000, 500)
        crc = binascii.crc_hqx(payload, 0xffff)
        command = b'\xa5\x5a' + payload + struct.pack('<H', crc)

        def frame(seq, reason, drive=False):
            state = 2 if drive else 255 if reason == 16 else 0
            flags = 0x77 if drive else 0x63
            body = struct.pack('<BBIIHHiHHBBHHBBBBHH',
                               3, flags, seq, (seq + 1) * 2000, 220, 2, 0,
                               125 if drive else 0, 2500, 1 if drive else 4,
                               state, 0, 0, reason, 15, 0x11, 7,
                               1 if drive else 0, crc if drive else 0)
            return b'\xa5\x5a' + body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))

        for name, reasons in (('default', (0, 1)), ('stops', (0, 1, 2, 3, 4, 16))):
            folder = run / 'integration' / name
            folder.mkdir(parents=True)
            wire = b''.join(frame(i, reason, drive=(i == 0)) for i, reason in enumerate(reasons))
            (folder / 'wire.bin').write_bytes(wire)
            (folder / 'result.json').write_text(json.dumps({'frames': len(reasons)}), encoding='utf-8')
            (folder / 'commands.hex').write_text(command[::-1].hex() + '\n', encoding='ascii')
            (folder / 'scenario.txt').write_text('W 10000\nE\n', encoding='ascii')
            (folder / 'pins.txt').write_text('0 1 0 0\n', encoding='ascii')
            (folder / 'snapshots.csv').write_text('0,2000,220,0,125,1,2,1,0\n', encoding='ascii')
            (folder / 'compile.log').write_text('', encoding='ascii')
            (folder / 'simulation.log').write_text('PASS tb_motor_test_top\n', encoding='ascii')
        result = dict(build_performed=True, python_tests_passed=1, board_accessed=False,
                      source_sha256=module.fingerprints(module.source_files()),
                      staged_firmware_sha256={'fixture.v': module.sha256(build / 'fixture.v')},
                      bitstream_sha256=module.sha256(fs), integration=module.check_evidence(run / 'integration'))
        (run / 'verification.json').write_text(json.dumps(result), encoding='utf-8')
        return run

    def test_archive_accepts_complete_unchanged_verified_run(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            run = self.make_verified_run(module, root)
            module.archive_verified_run(run, root / 'archive')
            self.assertEqual((root / 'archive/integration/default/wire.bin').read_bytes(),
                             (run / 'integration/default/wire.bin').read_bytes())

    def test_archive_rejects_uart_mutation_even_with_repaired_crc(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            run = self.make_verified_run(module, root)
            wire = run / 'integration/default/wire.bin'
            data = bytearray(wire.read_bytes())
            data[12] += 1
            data[38:40] = struct.pack('<H', binascii.crc_hqx(data[2:38], 0xffff))
            wire.write_bytes(data)
            with self.assertRaises(ValueError):
                module.archive_verified_run(run, root / 'archive')
            self.assertFalse((root / 'archive').exists())

    def test_archive_rejects_missing_or_changed_pin_snapshot_evidence(self):
        for name in ('pins.txt', 'snapshots.csv'):
            for missing in (False, True):
                with self.subTest(name=name, missing=missing), tempfile.TemporaryDirectory() as folder:
                    module = self.module()
                    root = Path(folder)
                    run = self.make_verified_run(module, root)
                    target = run / 'integration/default' / name
                    if missing:
                        target.unlink()
                    else:
                        target.write_text('changed evidence\n', encoding='ascii')
                    with self.assertRaises(ValueError):
                        module.archive_verified_run(run, root / 'archive')
                    self.assertFalse((root / 'archive').exists())


if __name__ == '__main__':
    unittest.main()
