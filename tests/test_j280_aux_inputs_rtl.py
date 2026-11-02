"""j280_aux_inputs (SW3/SW4 press events, DIP 2) against its debounce contract.

The module resets to "pressed"/"on" (like the SW1 start input): a key or switch
held through reset never produces an event or counts as on until it has been
released / seen off once. Every test therefore starts with a release settle.
"""
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_j280_motor_test_rtl import BIN, BUILD

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [ROOT / 'rtl/j280_swing/j280_aux_inputs.v', ROOT / 'sim/j280_swing/tb_aux_inputs.v']
SETTLE = ['I sw3_n 1', 'I sw4_n 1', 'W 8']


class AuxInputsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.run_dir = Path(tempfile.mkdtemp(prefix='tb_aux_inputs-', dir=BUILD))
        cls.env = dict(os.environ, PATH=str(BIN) + os.pathsep + os.environ['PATH'])
        for source in SOURCES:
            (cls.run_dir / source.name).write_bytes(source.read_bytes())
        result = subprocess.run([str(BIN / 'iverilog.exe'), '-g2001', '-Wall', '-s', 'tb_aux_inputs',
                                 '-Ptb_aux_inputs.DEBOUNCE_CLOCKS=5', '-o', 'aux.vvp',
                                 *[s.name for s in SOURCES]],
                                cwd=cls.run_dir, env=cls.env, capture_output=True, text=True, timeout=120)
        log = result.stdout + result.stderr
        if result.returncode or log.strip():
            raise AssertionError(f'{cls.run_dir}\n{log}')

    def simulate(self, lines):
        """Runs the vector lines; returns the list of EVENTS counts in order."""
        path = self.run_dir / 'vectors.txt'
        path.write_text('\n'.join(lines + ['Q']) + '\n', encoding='ascii')
        result = subprocess.run([str(BIN / 'vvp.exe'), 'aux.vvp', '+VECTORS=vectors.txt'],
                                cwd=self.run_dir, env=self.env, capture_output=True, text=True, timeout=120)
        log = result.stdout + result.stderr
        self.assertEqual(result.returncode, 0, log)
        self.assertNotIn('FAIL', log, log)
        self.assertIn('PASS tb_aux_inputs', log, log)
        return [int(x) for x in re.findall(r'EVENTS (-?\d+)', log)]

    def test_press_fires_once_and_release_does_not(self):
        counts = self.simulate(SETTLE + ['I sw3_n 0', 'C 12', 'C 20', 'I sw3_n 1', 'C 12',
                                         'I sw4_n 0', 'D 12', 'I sw4_n 1', 'D 12'])
        self.assertEqual(counts, [1, 0, 0, 1, 0])

    def test_bounce_shorter_than_the_debounce_time_is_ignored(self):
        counts = self.simulate(SETTLE + ['I sw3_n 0', 'C 3', 'I sw3_n 1', 'C 3', 'I sw3_n 0', 'C 3',
                                         'I sw3_n 1', 'C 15'])
        self.assertEqual(counts, [0, 0, 0, 0])

    def test_key_held_through_reset_fires_only_after_a_release(self):
        counts = self.simulate(SETTLE + ['I sw3_n 0', 'C 12', 'R', 'C 20', 'I sw3_n 1', 'C 10',
                                         'I sw3_n 0', 'C 12'])
        self.assertEqual(counts, [1, 0, 0, 1])

    def test_dip2_needs_a_debounced_on_after_an_off(self):
        # Off drops at once once synchronized (one clock for sync, one for dip2_on);
        # on needs a debounced on after an off was seen.
        self.simulate(SETTLE + ['E dip2_on 0', 'I dip2 0', 'W 8', 'E dip2_on 1',
                                'I dip2 1', 'W 2', 'E dip2_on 0', 'I dip2 0', 'W 8', 'E dip2_on 1'])
        # A reset with DIP 2 already on never counts as on until it is seen off.
        self.simulate(['I dip2 0', 'W 8', 'R', 'E dip2_on 0', 'W 8', 'E dip2_on 0',
                       'I dip2 1', 'W 8', 'I dip2 0', 'W 8', 'E dip2_on 1'])

    def test_levels_are_the_synchronized_pins(self):
        self.simulate(SETTLE + ['E levels 7', 'I sw3_n 0', 'W 3', 'E levels 6',
                                'I sw4_n 0', 'I dip2 0', 'W 3', 'E levels 0'])

    def test_one_event_per_press_over_a_long_stream(self):
        lines = list(SETTLE)
        for _ in range(6):
            lines += ['I sw3_n 0', 'C 10', 'I sw3_n 1', 'C 10']
        lines += ['I sw4_n 0', 'D 10', 'I sw4_n 1', 'D 10']
        self.assertEqual(self.simulate(lines), [1, 0] * 6 + [1, 0])


if __name__ == '__main__':
    unittest.main()
