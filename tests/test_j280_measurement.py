"""Offline protocol and CLI tests; these never open a real serial port."""

import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "j280_measurement.py"
if SCRIPT.exists():
    spec = importlib.util.spec_from_file_location("j280_measurement", SCRIPT)
    measurement = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(measurement)
else:
    measurement = None


def reference_crc(data):
    value = 0xFFFF
    for octet in data:
        for shift in range(7, -1, -1):
            feedback = ((value >> 15) ^ (octet >> shift)) & 1
            value = (value << 1) & 0xFFFF
            if feedback:
                value ^= 0x1021
    return value


def frame(seq=0, tick=0, raw=0, enc=0, flags=0, mode=0, version=1,
          adc_age=0, pwm_count=0, pwm_period=0, state=0, fault=0, pwm_age=0):
    payload = struct.pack("<BBIIHHiHHBBHH", version, flags, seq, tick,
                          raw, adc_age, enc, pwm_count, pwm_period,
                          mode, state, fault, pwm_age)
    return b"\xa5\x5a" + payload + struct.pack("<H", reference_crc(payload))


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(measurement, "Implement scripts/j280_measurement.py")

    def test_crc_known_answer(self):
        self.assertEqual(measurement.crc16_ccitt(b"123456789"), 0x29B1)

    def test_independent_golden_frame_and_extrema(self):
        golden = bytes.fromhex(
            "a55a010f78563412f0debc9aff03ffff00000080ffff000003feefbe34120932")
        rows = measurement.FrameStream().feed(golden)
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["seq"], 0x12345678)
        self.assertEqual(row["tick_us"], 0x9ABCDEF0)
        self.assertEqual(row["adc_raw10"], 1023)
        self.assertEqual(row["enc_count"], -(1 << 31))
        self.assertEqual(row["adc_age_us"], 65535)
        self.assertEqual(row["pwm_count"], 65535)
        self.assertEqual(row["pwm_period_count"], 0)
        self.assertEqual(row["motor_mode"], 3)
        self.assertEqual(row["state"], 254)
        self.assertEqual(row["fault_bits"], 0xBEEF)
        self.assertEqual(row["pwm_age_us"], 0x1234)
        for key in ("adc_valid", "adc_new", "output_enabled", "buffer_overflow",
                    "frame_valid", "crc_ok"):
            self.assertTrue(row[key], key)
        self.assertNotIn("angle_deg", row)

    def test_signed_encoder_and_unsigned_extrema(self):
        parser = measurement.FrameStream()
        row = parser.feed(frame(seq=0xFFFFFFFF, tick=0xFFFFFFFF,
                                enc=(1 << 31) - 1))[0]
        self.assertEqual(row["enc_count"], (1 << 31) - 1)
        self.assertEqual(row["tick_us"], 0xFFFFFFFF)

    def test_each_fragmentation_boundary(self):
        data = frame(enc=-123456)
        for split in range(33):
            with self.subTest(split=split):
                parser = measurement.FrameStream()
                rows = parser.feed(data[:split]) + parser.feed(data[split:])
                self.assertEqual([r["enc_count"] for r in rows], [-123456])
                self.assertEqual(parser.stats["frames_valid"], 1)

    def test_leading_noise_and_split_sync(self):
        parser = measurement.FrameStream()
        self.assertEqual(parser.feed(b"noise\xa5"), [])
        self.assertEqual(len(parser.feed(frame()[1:])), 1)
        self.assertEqual(parser.stats["discarded_bytes"], 5)

    def test_bad_crc_then_good_frame(self):
        broken = bytearray(frame(seq=10))
        broken[16] ^= 0x80
        parser = measurement.FrameStream()
        rows = parser.feed(broken + frame(seq=11))
        self.assertEqual([r["seq"] for r in rows], [11])
        self.assertEqual(parser.stats["crc_errors"], 1)
        self.assertEqual(parser.stats["frames_invalid"], 1)

    def test_embedded_sync_is_payload_in_valid_frame(self):
        rows = measurement.FrameStream().feed(frame(enc=0x5AA5, seq=7))
        self.assertEqual([r["enc_count"] for r in rows], [0x5AA5])

    def test_embedded_sync_allows_recovery_after_dropped_byte(self):
        damaged = frame(seq=5, enc=0x5AA5)
        parser = measurement.FrameStream()
        rows = parser.feed(damaged[:10] + damaged[11:] + frame(seq=6))
        self.assertEqual([r["seq"] for r in rows], [6])
        self.assertGreaterEqual(parser.stats["crc_errors"], 1)

    def test_invalid_version_reserved_bits_and_raw_are_diagnosed(self):
        cases = [(frame(version=2), "version_errors"),
                 (frame(flags=16), "reserved_bits_errors"),
                 (frame(mode=4), "reserved_bits_errors"),
                 (frame(raw=1024), "domain_errors")]
        for data, diagnostic in cases:
            with self.subTest(diagnostic=diagnostic, data=data.hex()):
                parser = measurement.FrameStream()
                self.assertEqual(parser.feed(data), [])
                self.assertEqual(parser.stats[diagnostic], 1)
                self.assertEqual(parser.stats["frames_invalid"], 1)

    def test_invalid_content_needs_good_crc_before_semantic_diagnosis(self):
        data = bytearray(frame(version=2))
        data[-1] ^= 1
        parser = measurement.FrameStream()
        self.assertEqual(parser.feed(data), [])
        self.assertEqual(parser.stats["crc_errors"], 1)
        self.assertEqual(parser.stats["version_errors"], 0)

    def test_noise_retains_bounded_tail(self):
        parser = measurement.FrameStream()
        self.assertEqual(parser.feed(b"\x00" * 1_000_000 + b"\xa5"), [])
        self.assertLessEqual(parser.buffered_bytes, 31)
        self.assertEqual(parser.stats["incomplete_tail_bytes"], 1)
        self.assertEqual(parser.stats["bytes_received"], 1_000_001)

    def test_incomplete_frame_tail_is_reported(self):
        parser = measurement.FrameStream()
        parser.feed(frame() + frame(seq=1)[:17])
        self.assertEqual(parser.stats["incomplete_tail_bytes"], 17)
        self.assertEqual(parser.stats["frames_valid"], 1)

    def test_sequence_gap_and_duplicate(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(10, 100) + frame(13, 130) + frame(13, 130))
        self.assertEqual([r["seq_gap"] for r in rows], [0, 2, 0])
        self.assertTrue(rows[2]["duplicate"])
        self.assertEqual(parser.stats["seq_missed"], 2)
        self.assertEqual(parser.stats["seq_duplicates"], 1)

    def test_sequence_and_timestamp_wrap(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(0xFFFFFFFF, 0xFFFFFFF0) + frame(0, 16))
        self.assertEqual(rows[1]["tick_us_unwrapped"], (1 << 32) + 16)
        self.assertFalse(rows[1]["reset"])
        self.assertEqual(rows[1]["epoch"], 0)
        self.assertEqual(parser.stats["tick_wraps"], 1)
        self.assertEqual(parser.stats["seq_wraps"], 1)

    def test_known_reset_starts_new_epoch(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(100, 100000) + frame(0, 0) + frame(1, 1000))
        self.assertEqual([r["epoch"] for r in rows], [0, 1, 1])
        self.assertEqual(rows[1]["tick_us_unwrapped"], 0)
        self.assertTrue(rows[1]["reset"])
        self.assertEqual(parser.stats["seq_missed"], 0)
        self.assertEqual(parser.stats["resets"], 1)

    def test_sequence_rollback_reset_at_time_wrap_does_not_continue_old_epoch(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(100, 0xFFFFFFF0) + frame(0, 0))
        self.assertTrue(rows[1]["reset"])
        self.assertEqual(rows[1]["epoch"], 1)
        self.assertEqual(rows[1]["tick_us_unwrapped"], 0)
        self.assertEqual(parser.stats["tick_wraps"], 0)

    def test_sequence_wrap_with_forward_time_and_missing_frames(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(0xFFFFFFFE, 100) + frame(1, 400))
        self.assertEqual(rows[1]["seq_gap"], 2)
        self.assertFalse(rows[1]["reset"])
        self.assertEqual(parser.stats["seq_wraps"], 1)

    def test_bytewise_feed_handles_all_sync_prefixes(self):
        parser = measurement.FrameStream()
        data = b"\xa5\xa5\x00\x5a" + frame(42, 123, enc=-2)
        rows = []
        for value in data:
            rows.extend(parser.feed(bytes([value])))
            self.assertLessEqual(parser.buffered_bytes, 31)
        self.assertEqual([r["seq"] for r in rows], [42])

    def test_ambiguous_nonzero_rollback_preserves_anchor(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(100, 100000) + frame(3, 3000) + frame(101, 101000))
        self.assertTrue(rows[1]["reset_or_reorder"])
        self.assertTrue(rows[1]["out_of_order"])
        self.assertEqual(rows[1]["tick_us_unwrapped"], 3000)
        self.assertEqual(rows[2]["seq_gap"], 0)
        self.assertEqual(rows[2]["tick_us_unwrapped"], 101000)
        self.assertEqual(parser.stats["seq_missed"], 0)
        self.assertEqual(parser.stats["uncertain_rollbacks"], 1)

    def test_nonzero_rollback_near_tick_wrap_remains_explicitly_ambiguous(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(100, 0xFFFFFFF0) + frame(3, 3000)
                           + frame(101, 0xFFFFFFF8))
        self.assertTrue(rows[1]["out_of_order"])
        self.assertTrue(rows[1]["reset_or_reorder"])
        self.assertFalse(rows[1]["reset"])
        self.assertEqual(rows[1]["epoch"], 0)
        self.assertEqual(rows[1]["tick_us_unwrapped"], 3000)
        self.assertEqual(rows[2]["seq_gap"], 0)
        self.assertEqual(rows[2]["tick_us_unwrapped"], 0xFFFFFFF8)
        self.assertEqual(parser.stats["seq_missed"], 0)
        self.assertEqual(parser.stats["uncertain_rollbacks"], 1)

    def test_isolated_out_of_order_does_not_advance_anchor(self):
        parser = measurement.FrameStream()
        rows = parser.feed(frame(10, 1000) + frame(9, 900) + frame(11, 1100))
        self.assertTrue(rows[1]["out_of_order"])
        self.assertFalse(rows[1]["reset"])
        self.assertEqual(rows[2]["seq_gap"], 0)
        self.assertEqual(parser.stats["seq_out_of_order"], 1)

    def test_time_reset_with_forward_sequence_starts_epoch(self):
        rows = measurement.FrameStream().feed(frame(100, 100000) + frame(101, 10))
        self.assertTrue(rows[1]["reset"])
        self.assertEqual(rows[1]["epoch"], 1)
        self.assertEqual(rows[1]["tick_us_unwrapped"], 10)


class CliTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(measurement, "Implement scripts/j280_measurement.py")
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)

    def invoke(self, arguments):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            result = measurement.main(arguments)
        return result, stdout.getvalue(), stderr.getvalue()

    def test_decode_creates_template_compatible_csv_and_json_stats(self):
        binary = self.directory / "input.bin"
        output = self.directory / "output.csv"
        binary.write_bytes(frame(1, 10, enc=-99) + frame(2, 20)[:7])
        result, stdout, stderr = self.invoke(
            ["decode", "--input", str(binary), "--output", str(output)])
        self.assertEqual((result, stderr), (0, ""))
        stats = json.loads(stdout)
        self.assertEqual(stats["frames_valid"], 1)
        self.assertEqual(stats["incomplete_tail_bytes"], 7)
        with output.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            row = next(reader)
            self.assertEqual(row["enc_count"], "-99")
            template = SCRIPT.parents[1] / "docs" / "templates" / "j280-dynamic.csv"
            for name in template.read_text(encoding="utf-8-sig").strip().split(","):
                self.assertIn(name, reader.fieldnames)

    def test_decode_does_not_overwrite_existing_csv(self):
        binary = self.directory / "input.bin"
        output = self.directory / "output.csv"
        binary.write_bytes(frame())
        output.write_text("keep me", encoding="utf-8")
        result, _, stderr = self.invoke(
            ["decode", "--input", str(binary), "--output", str(output)])
        self.assertEqual(result, 2)
        self.assertEqual(output.read_text(encoding="utf-8"), "keep me")
        self.assertIn("error", json.loads(stderr))

    def test_capture_rejects_nonpositive_or_nonfinite_duration_before_open(self):
        for duration in ("0", "-1", "nan", "inf"):
            with self.subTest(duration=duration):
                with mock.patch.dict(sys.modules, {"serial": None}):
                    result, _, stderr = self.invoke([
                        "capture", "--port", "never-open", "--seconds", duration,
                        "--output", str(self.directory / (duration + ".csv"))])
                self.assertEqual(result, 2)
                self.assertIn("seconds", stderr)

    def test_capture_default_baud_closes_port_and_preserves_raw(self):
        output, raw = self.directory / "output.csv", self.directory / "raw.bin"
        payload = frame(2, 50)
        port = mock.Mock()
        port.read.return_value = payload
        serial_module = types.SimpleNamespace(Serial=mock.Mock(return_value=port))
        with mock.patch.dict(sys.modules, {"serial": serial_module}), \
                mock.patch.object(measurement.time, "monotonic", side_effect=[0, 0, 2]):
            result, stdout, stderr = self.invoke([
                "capture", "--port", "offline-fake", "--seconds", "1",
                "--output", str(output), "--raw-output", str(raw)])
        self.assertEqual((result, stderr), (0, ""))
        self.assertEqual(json.loads(stdout)["frames_valid"], 1)
        self.assertEqual(raw.read_bytes(), payload)
        self.assertEqual(serial_module.Serial.call_args.kwargs["baudrate"], 230400)
        port.close.assert_called_once()
        port.write.assert_not_called()
        port.setDTR.assert_not_called()
        port.setRTS.assert_not_called()

    def test_capture_closes_port_if_read_fails(self):
        port = mock.Mock()
        port.read.side_effect = OSError("offline read failure")
        serial_module = types.SimpleNamespace(Serial=mock.Mock(return_value=port))
        with mock.patch.dict(sys.modules, {"serial": serial_module}), \
                mock.patch.object(measurement.time, "monotonic", return_value=0):
            result, stdout, stderr = self.invoke([
                "capture", "--port", "offline-fake", "--seconds", "1",
                "--output", str(self.directory / "output.csv")])
        self.assertEqual(result, 2)
        port.close.assert_called_once()
        self.assertIn("error", json.loads(stderr))
        self.assertEqual(json.loads(stdout)["frames_valid"], 0)

    def test_capture_interruption_closes_port_and_emits_partial_stats(self):
        port = mock.Mock()
        port.read.side_effect = KeyboardInterrupt
        serial_module = types.SimpleNamespace(Serial=mock.Mock(return_value=port))
        with mock.patch.dict(sys.modules, {"serial": serial_module}), \
                mock.patch.object(measurement.time, "monotonic", return_value=0):
            result, stdout, stderr = self.invoke([
                "capture", "--port", "offline-fake", "--seconds", "1",
                "--output", str(self.directory / "output.csv")])
        self.assertEqual(result, 130)
        port.close.assert_called_once()
        self.assertEqual(json.loads(stdout)["frames_valid"], 0)
        self.assertIn("interrupted", json.loads(stderr)["error"])

    def test_capture_does_not_overwrite_existing_raw_or_create_csv(self):
        raw, output = self.directory / "raw.bin", self.directory / "output.csv"
        raw.write_bytes(b"original")
        result, _, _ = self.invoke([
            "capture", "--port", "never-open", "--seconds", "1",
            "--output", str(output), "--raw-output", str(raw)])
        self.assertEqual(result, 2)
        self.assertEqual(raw.read_bytes(), b"original")
        self.assertFalse(output.exists())

    def test_capture_rejects_same_csv_and_raw_path(self):
        output = self.directory / "output.csv"
        result, _, _ = self.invoke([
            "capture", "--port", "never-open", "--seconds", "1",
            "--output", str(output), "--raw-output", str(output)])
        self.assertEqual(result, 2)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
