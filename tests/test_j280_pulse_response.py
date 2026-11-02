"""Pulse-response summary on synthetic rows."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import j280_pulse_response as pulse  # noqa: E402


def row(i, enc, on=False, pwm=0, in1=False, in2=False, state='SAFE', reason='none', adc=220):
    return dict(t=2000*i, enc=enc, adc=adc, on=on, pwm=pwm, in1=in1, in2=in2,
                state=state, reason=reason)


class PulseResponseTests(unittest.TestCase):
    def test_motion_drive_coast_and_delay(self):
        rows = [row(i, 10) for i in range(5)]
        encs = [10, 10, 10, 9, 7, 5, 3, 1]
        rows += [row(5+i, e, on=True, pwm=250, in2=True, state='RUN') for i, e in enumerate(encs)]
        rows += [row(13, 0, reason='complete'), row(14, -1, reason='complete'),
                 row(15, -2, reason='complete')] + [row(16+i, -2, reason='complete') for i in range(5)]
        r = pulse.analyze(rows, reference_enc=0)
        self.assertEqual((r['direction'], r['pwm_count'], r['duty_percent']), ('in2_high', 250, 10.0))
        self.assertEqual(r['output_ms'], 16.0)
        self.assertEqual((r['drive_counts'], r['coast_counts'], r['total_counts']), (-10, -2, -12))
        self.assertEqual(r['first_motion_ms'], 6.0)
        self.assertEqual(r['settle_ms_after_off'], 4.0)
        self.assertEqual(r['stop_reason'], 'complete')
        self.assertTrue(r['moved'] and r['breakaway'])
        self.assertEqual(r['offset_from_reference'], -2)
        self.assertEqual(r['peak_speed_count_s'], 900.0)  # 10 -> 1 over 10 ms

    def test_backlash_then_stall_is_not_breakaway(self):
        # 2026-10-17 8% pulse: -4 counts within 42 ms, then still for the rest of 200 ms.
        drive = [0]*5 + [-1]*5 + [-2]*5 + [-3]*6 + [-4]*79
        rows = [row(0, 0)] + [row(1+i, e, on=True, pwm=200, in1=True) for i, e in enumerate(drive)]
        rows += [row(101, -4, reason='complete'), row(102, -4, reason='complete')]
        r = pulse.analyze(rows)
        self.assertTrue(r['moved'])
        self.assertEqual(r['late_drive_counts'], 0)
        self.assertFalse(r['breakaway'])

    def test_slow_sustained_creep_is_breakaway(self):
        drive = [-(i // 10) for i in range(100)]
        rows = [row(0, 0)] + [row(1+i, e, on=True, pwm=275, in1=True) for i, e in enumerate(drive)]
        rows += [row(101, -10, reason='complete')]
        r = pulse.analyze(rows)
        self.assertLessEqual(r['late_drive_counts'], -4)
        self.assertTrue(r['breakaway'])

    def test_no_motion_below_threshold(self):
        rows = [row(0, 0)] + [row(1+i, 0 if i < 3 else 1, on=True, pwm=125, in1=True)
                              for i in range(5)] + [row(6, 1)]
        r = pulse.analyze(rows)
        self.assertFalse(r['moved'])
        self.assertEqual(r['direction'], 'in1_high')

    def test_no_output_and_unfinished_output(self):
        self.assertFalse(pulse.analyze([row(0, 0), row(1, 0)])['output_seen'])
        with self.assertRaises(ValueError):
            pulse.analyze([row(0, 0), row(1, 0, on=True, pwm=125, in1=True)])


if __name__ == '__main__':
    unittest.main()
