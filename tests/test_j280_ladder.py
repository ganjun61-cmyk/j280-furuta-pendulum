"""Ladder stopping rules."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from j280_ladder import decide  # noqa: E402

DONE = dict(started=True, end_reason='complete', phase='run')
STILL = dict(breakaway=False, enc_end=0)


class LadderDecisionTests(unittest.TestCase):
    def test_clean_no_motion_advances(self):
        self.assertEqual(decide(DONE, True, STILL, 0, False)[0], 'next')

    def test_motion_fault_operator_stop_and_timeout_stop(self):
        self.assertEqual(decide(DONE, True, dict(breakaway=True, enc_end=-8), 0, False), ('stop', 'breakaway'))
        for result in (dict(DONE, end_reason='travel_limit'), dict(DONE, end_reason='stop_key'),
                       dict(DONE, end_reason='arm_off'), dict(started=False, phase='start_timeout'), None):
            with self.subTest(result=result):
                self.assertEqual(decide(result, True, STILL, 0, False)[0], 'stop')

    def test_motion_wins_over_incomplete_record(self):
        self.assertEqual(decide(DONE, False, dict(breakaway=True, enc_end=5), 0, False)[0], 'stop')

    def test_incomplete_record_repeats_once(self):
        self.assertEqual(decide(DONE, False, STILL, 0, False)[0], 'repeat')
        self.assertEqual(decide(DONE, False, STILL, 0, True)[0], 'stop')

    def test_large_offset_stops(self):
        self.assertEqual(decide(DONE, True, dict(breakaway=False, enc_end=121), 0, False)[0], 'stop')


if __name__ == '__main__':
    unittest.main()
