"""Wire-quality acceptance, using independently encoded v3 fixtures."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_j280_motor_test import sample
import j280_audit_motor_capture as audit


class MotorCaptureAuditTests(unittest.TestCase):
    def test_clean_capture_can_start_and_end_with_partial_frames(self):
        data=bytes(17)+sample()+sample(seq=1,tick_us=4000)+sample(seq=2,tick_us=6000)[:11]
        result=audit.audit(data,min_frames=2,expect_safe=True)
        self.assertTrue(result['passed'])
        self.assertEqual((result['prefix_bytes'],result['incomplete_tail_bytes']),(17,11))
        self.assertEqual(result['actual_packets_missed'],0)

    def test_empty_and_too_short_capture_are_not_success(self):
        for data in (b'',sample()):
            with self.subTest(data=data):
                self.assertFalse(audit.audit(data,min_frames=2)['passed'])

    def test_corrupted_frame_is_rejected_even_after_resynchronization(self):
        broken=bytearray(sample());broken[16]^=1
        result=audit.audit(bytes(broken)+sample(seq=1,tick_us=4000))
        self.assertFalse(result['passed'])
        self.assertEqual(result['stats']['crc_errors'],1)

    def test_missing_whole_frame_is_rejected_without_crc_failure(self):
        result=audit.audit(sample()+sample(seq=2,tick_us=6000))
        self.assertFalse(result['passed'])
        self.assertEqual(result['actual_packets_missed'],1)
        self.assertEqual(result['stats']['crc_errors'],0)

    def test_duplicate_reorder_reset_and_wrong_tick_are_rejected(self):
        for data in (
            sample()+sample(),
            sample(seq=5,tick_us=12000)+sample(seq=4,tick_us=10000),
            sample(seq=100,tick_us=202000)+sample(),
            sample()+sample(seq=1,tick_us=4001),
        ):
            with self.subTest(data=data):self.assertFalse(audit.audit(data)['passed'])

    def test_counter_wrap_is_valid(self):
        data=sample(seq=0xffffffff,tick_us=0xfffff830)+sample(seq=0,tick_us=0)
        self.assertTrue(audit.audit(data)['passed'])

    def test_interior_garbage_is_not_excused_as_capture_boundary(self):
        result=audit.audit(sample()+b'junk'+sample(seq=1,tick_us=4000))
        self.assertFalse(result['passed'])
        self.assertEqual(result['interior_discarded_bytes'],4)

    def test_diagnostic_stride_does_not_look_like_lost_packets(self):
        data=sample(seq=3,tick_us=8000)+sample(seq=7,tick_us=16000)
        normal=audit.audit(data)
        diagnostic=audit.audit(data,profile='diagnostic125')
        self.assertFalse(normal['passed'])
        self.assertTrue(diagnostic['passed'])
        self.assertEqual(diagnostic['actual_packets_missed'],0)
        self.assertEqual(diagnostic['intentional_skipped_snapshots'],3)
        self.assertTrue(diagnostic['diagnostic_only'])

    def test_actual_diagnostic_packet_loss_is_still_rejected(self):
        data=sample(seq=3,tick_us=8000)+sample(seq=11,tick_us=24000)
        result=audit.audit(data,profile='diagnostic125')
        self.assertFalse(result['passed'])
        self.assertEqual(result['actual_packets_missed'],1)
        wrong=sample(seq=3,tick_us=8000)+sample(seq=5,tick_us=12000)
        self.assertFalse(audit.audit(wrong,profile='diagnostic125')['passed'])

    def test_safe_expectation_checks_configuration_and_output(self):
        loaded=sample(flags=0x53,config_id=1,config_crc=2)
        run=sample(flags=0x77,config_id=1,pwm_count=125,motor_mode=1,state_code=2)
        for data in (loaded,run):
            self.assertTrue(audit.audit(data)['passed'])
            self.assertFalse(audit.audit(data,expect_safe=True)['passed'])

    def test_invalid_arguments_are_rejected(self):
        for options in (dict(profile='anything'),dict(min_frames=0),dict(min_frames=True)):
            with self.subTest(options=options),self.assertRaises(ValueError):
                audit.audit(sample(),**options)

    def test_cli_failure_exit_and_report_preserve_input(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'wire.bin';source.write_bytes(sample()+sample(seq=2,tick_us=6000))
            report=Path(directory)/'audit.json'
            with contextlib.redirect_stdout(io.StringIO()):
                rc=audit.main(['--input',str(source),'--report',str(report)])
            self.assertEqual(rc,2)
            self.assertFalse(json.loads(report.read_text())['passed'])
            self.assertEqual(source.read_bytes(),sample()+sample(seq=2,tick_us=6000))
            with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(audit.main(['--input',str(source),'--report',str(source)]),2)
                self.assertEqual(audit.main(['--input',str(source),'--report',str(report)]),2)


if __name__=='__main__':unittest.main()
