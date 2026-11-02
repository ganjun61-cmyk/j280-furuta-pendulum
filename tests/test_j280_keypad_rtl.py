"""4x4 keypad entry RTL check: digits, direction, speed and submit."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [ROOT / 'rtl/j280_swing/j280_keypad.v', ROOT / 'sim/j280_swing/tb_keypad.v']


def _iverilog_bin():
    local = Path(os.environ.get('LOCALAPPDATA', str(Path.home())))
    candidates = [Path(os.environ[k]) for k in ('J280_IVERILOG_BIN', 'IVERILOG_BIN')
                  if k in os.environ]
    candidates.append(local / 'J280Tools/iverilog-msys2/ucrt64/bin')
    found = shutil.which('iverilog')
    if found:
        candidates.append(Path(found).parent)
    for cand in candidates:
        if (cand / 'iverilog.exe').is_file():
            return cand
    raise unittest.SkipTest('iverilog not installed')


class KeypadRTLTests(unittest.TestCase):
    def test_entry_120_cw_90_and_submit(self):
        binary = _iverilog_bin()
        run = Path(tempfile.mkdtemp(prefix='keypad-'))
        for source in SOURCES:
            shutil.copy2(source, run / source.name)
        env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ.get('PATH', ''))
        compiled = subprocess.run([str(binary / 'iverilog.exe'), '-g2005', '-o', 'sim.vvp',
                                   *(s.name for s in SOURCES)], cwd=run, env=env,
                                  capture_output=True, text=True, timeout=60)
        self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
        result = subprocess.run([str(binary / 'vvp.exe'), 'sim.vvp'], cwd=run, env=env,
                                capture_output=True, text=True, timeout=120)
        log = result.stdout + result.stderr
        self.assertIn('PASS', log, log)
        self.assertNotIn('FAIL', log, log)


if __name__ == '__main__':
    unittest.main()
