"""Offline v4 fixtures: absolute wire offsets and an independent bitwise CRC."""

import importlib.util
from pathlib import Path
import struct
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SCRIPT = ROOT / "scripts" / "j280_balance_protocol.py"
protocol = None
if SCRIPT.exists():
    spec = importlib.util.spec_from_file_location("j280_balance_protocol", SCRIPT)
    protocol = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(protocol)


def reference_crc(data):
    value = 0xFFFF
    for octet in data:
        for shift in range(7, -1, -1):
            feedback = ((value >> 15) ^ (octet >> shift)) & 1
            value = (value << 1) & 0xFFFF
            if feedback:
                value ^= 0x1021
    return value


def seal(data):
    data = bytearray(data)
    data[-2:] = reference_crc(data[2:-2]).to_bytes(2, "little")
    return bytes(data)


def valid_values(**changes):
    values = dict(config_id=0x1234, algorithm_id=1, run_ms=3000,
                  keepalive_ms=500, travel_limit_count=200,
                  speed_limit_count_s=10000, adc_speed_limit_code_s=30000,
                  max_compare=1000, hold_ms=200, wait_ms=10000,
                  upright_zero_q16=(512 << 16) + 32768, enter_code=9,
                  exit_code=57, comp_compare=175, blend_compare=100,
                  comp_gain_q20=1835008, coefficients=[0] * 32,
                  sample_period_us=2000)
    values.update(changes)
    return values


def reference_command(values=None, opcode=1):
    data = bytearray(192)
    data[0:4] = bytes((0xA5, 0x5A, 4, opcode))
    if opcode == 1:
        values = valid_values() if values is None else values
        offsets = {4: "config_id", 6: "algorithm_id", 8: "run_ms",
                   10: "keepalive_ms", 12: "travel_limit_count",
                   14: "speed_limit_count_s", 16: "adc_speed_limit_code_s",
                   18: "max_compare", 20: "hold_ms", 22: "wait_ms",
                   28: "enter_code", 30: "exit_code", 32: "comp_compare",
                   34: "blend_compare", 168: "sample_period_us"}
        for offset, key in offsets.items():
            data[offset:offset + 2] = values[key].to_bytes(2, "little")
        for offset, key in ((24, "upright_zero_q16"), (36, "comp_gain_q20")):
            data[offset:offset + 4] = values[key].to_bytes(4, "little", signed=True)
        for index, coefficient in enumerate(values["coefficients"]):
            offset = 40 + 4 * index
            data[offset:offset + 4] = coefficient.to_bytes(4, "little", signed=True)
    return seal(data)


def telemetry_fixture(packet_seq=20, tick_us=None, raw_seq=None, **changes):
    values = dict(version=4, flags=0x43, packet_seq=packet_seq,
                  tick_us=1_000_000 + packet_seq * 10000 if tick_us is None else tick_us,
                  raw_seq=packet_seq * 5 if raw_seq is None else raw_seq,
                  adc_raw10=513, adc_age_us=17, enc_count=-123456,
                  theta_ref=12345, upright_zero_q16=(512 << 16) + 32768,
                  config_id=0, config_crc=0, state_code=0, stop_reason_code=0,
                  input_levels=5, motor_mode=4, applied_compare=0,
                  pwm_period=2500, u_linear_q16=-1234567,
                  u_compensated_q16=2345678, xhat=[-1, 2, -(1 << 31), (1 << 31) - 1],
                  controller_age_us=321, fault_bits=0, raw_estimator_quality=7,
                  rx_status=0xA1, controller_latency_cycles=499,
                  sample_period_us=2000, telemetry_period_us=10000,
                  state_elapsed_ms=1234, safety_position_delta=-199,
                  u_model_q16=-7654321)
    values.update(changes)
    data = bytearray(128)
    data[0:2] = b"\xa5\x5a"
    # Independently listed absolute offsets; never import production field maps.
    for offset, key in ((2, "version"), (3, "flags"), (40, "state_code"),
                        (41, "stop_reason_code"), (42, "input_levels"),
                        (43, "motor_mode"), (78, "raw_estimator_quality"), (79, "rx_status")):
        data[offset] = values[key]
    for offset, key in ((20, "adc_raw10"), (22, "adc_age_us"), (36, "config_id"),
                        (38, "config_crc"), (44, "applied_compare"), (46, "pwm_period"),
                        (76, "fault_bits")):
        data[offset:offset + 2] = values[key].to_bytes(2, "little")
    for offset, key in ((4, "packet_seq"), (16, "raw_seq"), (72, "controller_age_us"),
                        (80, "controller_latency_cycles"), (84, "sample_period_us"),
                        (88, "telemetry_period_us"), (92, "state_elapsed_ms")):
        data[offset:offset + 4] = values[key].to_bytes(4, "little")
    data[8:16] = values["tick_us"].to_bytes(8, "little")
    for offset, key in ((24, "enc_count"), (28, "theta_ref"), (32, "upright_zero_q16"),
                        (48, "u_linear_q16"), (52, "u_compensated_q16"),
                        (96, "safety_position_delta"), (100, "u_model_q16")):
        data[offset:offset + 4] = values[key].to_bytes(4, "little", signed=True)
    for index, value in enumerate(values["xhat"]):
        offset = 56 + index * 4
        data[offset:offset + 4] = value.to_bytes(4, "little", signed=True)
    return seal(data)


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(protocol, "Implement scripts/j280_balance_protocol.py")

    def test_reference_crc_known_answer(self):
        self.assertEqual(reference_crc(b"123456789"), 0x29B1)
        self.assertEqual(protocol.crc16_ccitt(b"123456789"), 0x29B1)

    def test_config_matches_independent_absolute_offsets_and_crc(self):
        coefficients = [(-1 if i % 2 else 1) * (i + 1) * 123456 for i in range(32)]
        coefficients[0], coefficients[-1] = -(128 << 20), 128 << 20
        values = valid_values(coefficients=coefficients)
        command = protocol.build_command("config", values)
        self.assertEqual(command, reference_command(values))
        self.assertEqual(len(command), 192)
        self.assertEqual(command[170:190], bytes(20))
        self.assertEqual(struct.unpack_from("<i", command, 40)[0], -(128 << 20))
        self.assertEqual(struct.unpack_from("<i", command, 164)[0], 128 << 20)
        self.assertEqual(int.from_bytes(command[190:192], "little"), reference_crc(command[2:190]))

    def test_keepalive_abort_have_entire_zero_body(self):
        for name, opcode in (("keepalive", 2), ("abort", 3)):
            with self.subTest(name=name):
                command = protocol.build_command(name)
                self.assertEqual(command, reference_command(opcode=opcode))
                self.assertEqual(command[4:190], bytes(186))
                self.assertEqual(command[-2:], b"\xb0\xb0" if opcode == 2 else b"\x9a\x97")
                for values in ({}, valid_values()):
                    with self.assertRaises(ValueError):
                        protocol.build_command(name, values)

    def test_no_start_unknown_or_nonstr_command(self):
        for name in ("start", "START", "run", "", 1, True, None, [], {}):
            with self.subTest(name=name), self.assertRaises(ValueError):
                protocol.build_command(name)

    def test_config_requires_complete_values(self):
        with self.assertRaises(ValueError):
            protocol.build_command("config")

    def test_synthetic_zero_coefficients_are_valid_and_defensively_copied(self):
        original = valid_values(coefficients=(0,) * 32)
        validated = protocol.validate_values(original)
        self.assertEqual(validated, valid_values())
        validated["coefficients"][0] = 1
        validated["max_compare"] = 1
        self.assertEqual(original["coefficients"], (0,) * 32)
        self.assertEqual(original["max_compare"], 1000)

    def test_values_require_exact_keys_and_dictionary(self):
        missing = valid_values()
        del missing["sample_period_us"]
        for value in (None, [], (), "values", missing, valid_values(extra=1)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                protocol.validate_values(value)

    def test_every_scalar_rejects_bool_float_nonfinite_and_noninteger(self):
        for key in valid_values():
            if key == "coefficients":
                continue
            for bad in (True, False, 1.0, float("nan"), float("inf"), "1", None):
                with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                    protocol.validate_values(valid_values(**{key: bad}))

    def test_scalar_boundaries_and_fixed_ids_period(self):
        ranges = {"config_id": (1, 65535), "algorithm_id": (1, 1),
                  "run_ms": (1, 30000), "keepalive_ms": (1, 500),
                  "travel_limit_count": (1, 300), "speed_limit_count_s": (1, 10000),
                  "adc_speed_limit_code_s": (1, 30000), "max_compare": (1, 1125),
                  "hold_ms": (200, 1000), "wait_ms": (202, 10000),
                  "upright_zero_q16": (0, 1023 << 16), "enter_code": (0, 9),
                  "exit_code": (1, 57), "comp_compare": (0, 250),
                  "blend_compare": (25, 250), "comp_gain_q20": (0, 10 << 20),
                  "sample_period_us": (2000, 2000)}
        for key, (low, high) in ranges.items():
            for bad in (low - 1, high + 1):
                with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                    protocol.validate_values(valid_values(**{key: bad}))
        for key in ("config_id", "run_ms", "keepalive_ms", "travel_limit_count",
                    "speed_limit_count_s", "adc_speed_limit_code_s", "hold_ms",
                    "upright_zero_q16", "enter_code", "blend_compare"):
            for edge in ranges[key]:
                changes = {key: edge}
                if key == "blend_compare":
                    changes.update(comp_compare=0, comp_gain_q20=0)
                self.assertEqual(protocol.validate_values(valid_values(**changes))[key], edge)
        for changes in (dict(max_compare=1, comp_compare=0, comp_gain_q20=0),
                        dict(exit_code=1, enter_code=0), dict(wait_ms=202),
                        dict(comp_compare=250, blend_compare=25, comp_gain_q20=10 << 20)):
            protocol.validate_values(valid_values(**changes))

    def test_threshold_wait_and_compensation_coherence(self):
        for changes in (dict(enter_code=9, exit_code=9), dict(hold_ms=200, wait_ms=201),
                        dict(hold_ms=1000, wait_ms=1001), dict(max_compare=174),
                        dict(comp_gain_q20=1835009), dict(comp_gain_q20=1835007)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                protocol.validate_values(valid_values(**changes))
        # 1/27 in Q20: nearest integer has error 4, within ceil(27/2).
        protocol.validate_values(valid_values(comp_compare=1, blend_compare=27, comp_gain_q20=38836))
        # Both signs of the allowed ceil(27/2) == 14 boundary are inclusive.
        protocol.validate_values(valid_values(comp_compare=10, blend_compare=27, comp_gain_q20=388362))
        protocol.validate_values(valid_values(comp_compare=17, blend_compare=27, comp_gain_q20=660214))
        protocol.validate_values(valid_values(hold_ms=1000, wait_ms=1002))

    def test_coefficients_require_exactly_32_bounded_integers(self):
        for bad in (None, "0" * 32, bytes(32), [0] * 31, [0] * 33, {i: 0 for i in range(32)}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                protocol.validate_values(valid_values(coefficients=bad))
        for bad in (True, False, 0.0, float("nan"), float("inf"), None, "0",
                    (128 << 20) + 1, -(128 << 20) - 1, 1 << 31):
            for index in (0, 15, 16, 19, 20, 27, 28, 31):
                coefficients = [0] * 32
                coefficients[index] = bad
                with self.subTest(index=index, bad=bad), self.assertRaises(ValueError):
                    protocol.validate_values(valid_values(coefficients=coefficients))


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(hasattr(protocol, "FrameStream"), "Implement v4 FrameStream")

    def test_literal_fixture_crc_and_signed_model_input(self):
        data = bytes.fromhex(
            "a55a044314000000804f1200000000006400000001021100c01dfeff39300000"
            "0080000200000000000005040000c4097929edffceca2300ffffffff02000000"
            "00000080ffffff7f41010000000007a1f3010000d007000010270000d2040000"
            "39ffffff4f348bff000000000000000000000000000000000000000000009103")
        self.assertEqual(len(data), 128)
        row = protocol.FrameStream().feed(data)[0]
        self.assertEqual((row["packet_seq"], row["tick_us"], row["raw_seq"]), (20, 1200000, 100))
        self.assertEqual(row["u_model_q16"], -7654321)
        self.assertEqual(row["xhat"], [-1, 2, -(1 << 31), (1 << 31) - 1])

    def test_absolute_offsets_signed_values_and_64_bit_time(self):
        data = telemetry_fixture(packet_seq=0x89ABCDEF, raw_seq=0xFEDCBA98,
                                 tick_us=0x123456789ABCDEF0, flags=0x7F, fault_bits=4,
                                 config_id=0x1234, config_crc=0xBEEF, state_code=3,
                                 motor_mode=2, applied_compare=1000)
        rows = protocol.FrameStream().feed(data)
        self.assertEqual(len(rows), 1)
        row = rows[0]
        expected = dict(version=4, flags=0x7F, packet_seq=0x89ABCDEF, raw_seq=0xFEDCBA98,
                        tick_us=0x123456789ABCDEF0, adc_raw10=513, adc_age_us=17,
                        enc_count=-123456, theta_ref=12345, upright_zero_q16=33587200,
                        config_id=0x1234, config_crc=0xBEEF, state_code=3,
                        state="BALANCE", stop_reason_code=0, stop_reason="none",
                        input_levels=5, motor_mode=2, applied_compare=1000, pwm_period=2500,
                        u_linear_q16=-1234567, u_compensated_q16=2345678,
                        xhat=[-1, 2, -(1 << 31), (1 << 31) - 1], controller_age_us=321,
                        fault_bits=4, raw_estimator_quality=7, rx_status=0xA1,
                        controller_latency_cycles=499, sample_period_us=2000,
                        telemetry_period_us=10000, state_elapsed_ms=1234,
                        safety_position_delta=-199, u_model_q16=-7654321)
        for key, value in expected.items():
            self.assertEqual(row[key], value, key)
        for key in ("adc_valid", "adc_new", "output_enabled", "tx_overflow",
                    "config_loaded", "arm_on", "core_ok", "frame_valid", "crc_ok"):
            self.assertTrue(row[key], key)
        self.assertFalse(row["fault_latched"])

    def test_u_model_is_signed_int32_not_reserved_or_unsaturated_request(self):
        for value in (-(1125 << 16), -1, 0, 1, 1000 << 16):
            with self.subTest(value=value):
                row = protocol.FrameStream().feed(telemetry_fixture(u_model_q16=value))[0]
                self.assertEqual(row["u_model_q16"], value)

    def test_every_reserved_byte_104_through_125_is_checked_with_valid_crc(self):
        for offset in range(104, 126):
            data = bytearray(telemetry_fixture())
            data[offset] = 1
            parser = protocol.FrameStream()
            with self.subTest(offset=offset):
                self.assertEqual(parser.feed(seal(data)), [])
                self.assertEqual(parser.stats["reserved_bits_errors"], 1)
                self.assertEqual(parser.stats["frames_invalid"], 1)

    def test_state_output_mode_and_brake_coherence(self):
        for state in (0, 1, 2, 3, 255):
            loaded = state in (1, 2, 3)
            flags = 0xC3 if state == 255 else 0x53 if loaded else 0x43
            row = protocol.FrameStream().feed(telemetry_fixture(
                state_code=state, flags=flags, config_id=int(loaded)))[0]
            self.assertFalse(row["output_enabled"])
            self.assertEqual(row["motor_mode"], 4)
        for mode in (1, 2):
            for compare in (0, 1, 1000):
                with self.subTest(mode=mode, compare=compare):
                    row = protocol.FrameStream().feed(telemetry_fixture(
                        state_code=3, flags=0x57, config_id=1,
                        motor_mode=mode, applied_compare=compare))[0]
                    self.assertTrue(row["output_enabled"])
                    self.assertEqual(row["applied_compare"], compare)
        invalid = [dict(state_code=s, flags=0xC7 if s == 255 else 0x47,
                        motor_mode=1) for s in (0, 1, 2, 255)]
        invalid += [dict(state_code=3, flags=0x57, config_id=1, motor_mode=m)
                    for m in (0, 3, 4, 5, 6, 7, 8, 255)]
        invalid += [dict(motor_mode=m) for m in (0, 1, 2, 3, 5, 6, 7, 255)]
        invalid += [dict(applied_compare=1),
                    dict(state_code=3, flags=0x53, config_id=1, applied_compare=1),
                    dict(state_code=3, flags=0x57, config_id=1,
                         motor_mode=2, applied_compare=1126)]
        for changes in invalid:
            with self.subTest(changes=changes):
                parser = protocol.FrameStream()
                self.assertEqual(parser.feed(telemetry_fixture(**changes)), [])
                self.assertEqual(parser.stats["domain_errors"], 1)

    def test_flags_match_sensor_overflow_and_fault_state(self):
        invalid = (dict(flags=0x42), dict(fault_bits=1), dict(flags=0x4B),
                   dict(fault_bits=4), dict(flags=0xC3), dict(state_code=255))
        for changes in invalid:
            with self.subTest(changes=changes):
                parser = protocol.FrameStream()
                self.assertEqual(parser.feed(telemetry_fixture(**changes)), [])
                self.assertEqual(parser.stats["domain_errors"], 1)
        for changes in (dict(flags=0x42, fault_bits=1, raw_estimator_quality=0),
                        dict(flags=0x4B, fault_bits=4), dict(flags=0xC3, state_code=255)):
            self.assertEqual(len(protocol.FrameStream().feed(telemetry_fixture(**changes))), 1)

    def test_fixed_domains_and_raw_quality(self):
        for quality in (0, 1, 3, 5, 7):
            self.assertEqual(len(protocol.FrameStream().feed(
                telemetry_fixture(raw_estimator_quality=quality))), 1)
        invalid = [dict(state_code=s) for s in (4, 127, 254)]
        invalid += [dict(raw_estimator_quality=q) for q in (2, 4, 6, 8, 255)]
        invalid += [dict(adc_raw10=1024), dict(pwm_period=2499),
                    dict(sample_period_us=1000), dict(telemetry_period_us=2000),
                    dict(stop_reason_code=7), dict(stop_reason_code=255),
                    dict(upright_zero_q16=-1), dict(upright_zero_q16=(1023 << 16) + 1)]
        for changes in invalid:
            with self.subTest(changes=changes):
                parser = protocol.FrameStream()
                self.assertEqual(parser.feed(telemetry_fixture(**changes)), [])
                self.assertEqual(parser.stats["domain_errors"], 1)

    def assert_domain_rejected(self, **changes):
        parser = protocol.FrameStream()
        self.assertEqual(len(parser.feed(telemetry_fixture(**changes))), 0, changes)
        self.assertEqual(parser.stats["frames_invalid"], 1)
        self.assertEqual(parser.stats["domain_errors"], 1)
        self.assertEqual(parser.stats["crc_errors"], 0)

    def test_input_levels_upper_five_bits_are_zero(self):
        for value in range(8):
            self.assertEqual(len(protocol.FrameStream().feed(
                telemetry_fixture(input_levels=value))), 1)
        for bit in range(3, 8):
            self.assert_domain_rejected(input_levels=5 | (1 << bit))

    def test_fault_bits_upper_thirteen_bits_are_zero(self):
        for fault in range(8):
            # Retain the independently required ADC-valid and overflow mirrors.
            flags = (0x43 & ~9) | int(not (fault & 1)) | (int(bool(fault & 4)) << 3)
            self.assertEqual(len(protocol.FrameStream().feed(
                telemetry_fixture(flags=flags, fault_bits=fault))), 1)
        for bit in range(3, 16):
            self.assert_domain_rejected(fault_bits=1 << bit)

    def test_rx_status_low_nibble_is_only_zero_through_seven(self):
        for count in range(16):
            for code in range(8):
                self.assertEqual(len(protocol.FrameStream().feed(
                    telemetry_fixture(rx_status=(count << 4) | code))), 1)
        for count in range(16):
            for code in range(8, 16):
                self.assert_domain_rejected(rx_status=(count << 4) | code)

    def test_config_loaded_matches_id_and_unloaded_crc_is_zero(self):
        for changes in (dict(flags=0x43, config_id=0, config_crc=0),
                        dict(flags=0x53, config_id=1, config_crc=0),
                        dict(flags=0x53, config_id=65535, config_crc=65535)):
            # SAFE is permitted to hold a configuration; its CRC can itself be zero.
            self.assertEqual(len(protocol.FrameStream().feed(telemetry_fixture(**changes))), 1)
        for changes in (dict(flags=0x53, config_id=0), dict(flags=0x43, config_id=1),
                        dict(flags=0x43, config_crc=1), dict(flags=0x43, config_crc=65535),
                        dict(flags=0x53, config_id=0, config_crc=1234)):
            self.assert_domain_rejected(**changes)

    def test_active_guard_states_require_loaded_configuration(self):
        for state in (1, 2, 3):
            self.assertEqual(len(protocol.FrameStream().feed(telemetry_fixture(
                state_code=state, flags=0x53, config_id=1, config_crc=0x1234))), 1)
            self.assert_domain_rejected(state_code=state)

    def test_fault_requires_consumed_configuration(self):
        row = protocol.FrameStream().feed(telemetry_fixture(
            state_code=255, flags=0xC3, stop_reason_code=19))[0]
        self.assertFalse(row["config_loaded"])
        self.assertEqual((row["config_id"], row["config_crc"]), (0, 0))
        self.assert_domain_rejected(state_code=255, flags=0xD3,
                                    config_id=1, config_crc=0x1234)

    def test_all_control_inputs_respect_signed_compiled_q16_limit(self):
        limit = 1125 << 16
        for key in ("u_linear_q16", "u_compensated_q16", "u_model_q16"):
            for value in (-limit, -1, 0, 1, limit):
                row = protocol.FrameStream().feed(telemetry_fixture(**{key: value}))[0]
                self.assertEqual(row[key], value)
            for value in (-limit - 1, limit + 1, -(1 << 31), (1 << 31) - 1):
                self.assert_domain_rejected(**{key: value})

    def test_large_signed_xhat_diagnostics_survive_overspeed_fault(self):
        for value in (-(1 << 31), (1 << 31) - 1):
            row = protocol.FrameStream().feed(telemetry_fixture(
                state_code=255, flags=0xC3, stop_reason_code=19, xhat=[value] * 4))[0]
            self.assertEqual(row["xhat"], [value] * 4)
            self.assertEqual(row["stop_reason"], "speed_limit")

    def test_new_domain_rejections_cannot_advance_continuity_anchor(self):
        invalid = (dict(input_levels=8), dict(fault_bits=8), dict(rx_status=0xF8),
                   dict(flags=0x53), dict(config_id=1), dict(config_crc=1),
                   dict(state_code=1), dict(state_code=2), dict(state_code=3),
                   dict(state_code=255, flags=0xD3, config_id=1),
                   dict(u_linear_q16=(1125 << 16) + 1),
                   dict(u_compensated_q16=-(1125 << 16) - 1),
                   dict(u_model_q16=(1125 << 16) + 1))
        for changes in invalid:
            parser = protocol.FrameStream()
            rows = parser.feed(telemetry_fixture()
                               + telemetry_fixture(packet_seq=21, **changes)
                               + telemetry_fixture(packet_seq=21))
            self.assertEqual([row["packet_seq"] for row in rows], [20, 21], changes)
            self.assertTrue(rows[1]["continuity_ok"])
            self.assertEqual(rows[1]["packet_seq_gap"], 0)
            self.assertEqual(parser.stats["packet_seq_missed"], 0)
            self.assertEqual(parser.stats["domain_errors"], 1)

    def test_crc_precedes_new_domain_checks(self):
        data = bytearray(telemetry_fixture(input_levels=8))
        data[126] ^= 1
        parser = protocol.FrameStream()
        self.assertEqual(parser.feed(data), [])
        self.assertEqual(parser.stats["crc_errors"], 1)
        self.assertEqual(parser.stats["domain_errors"], 0)
        self.assertEqual(len(parser.feed(seal(data))), 0)
        self.assertEqual(parser.stats["domain_errors"], 1)

    def test_all_documented_stop_reasons(self):
        reasons = {0: "none", 1: "complete", 2: "stop_key", 3: "arm_off", 4: "host_abort",
                   5: "arm_timeout", 6: "capture_timeout", 16: "sensor_invalid",
                   17: "sensor_stale", 18: "travel_limit", 19: "speed_limit",
                   20: "keepalive_timeout", 21: "angle_limit", 22: "numeric_range",
                   23: "control_overrun", 24: "pwm_reject"}
        for code, name in reasons.items():
            row = protocol.FrameStream().feed(telemetry_fixture(stop_reason_code=code))[0]
            self.assertEqual(row["stop_reason"], name)

    def test_crc_precedes_version_and_semantic_diagnostics(self):
        data = bytearray(telemetry_fixture(version=3, adc_raw10=1024))
        data[126] ^= 1
        parser = protocol.FrameStream()
        self.assertEqual(parser.feed(data), [])
        self.assertEqual(parser.stats["crc_errors"], 1)
        self.assertEqual(parser.stats["version_errors"], 0)
        self.assertEqual(parser.stats["domain_errors"], 0)
        self.assertEqual(parser.feed(seal(data)), [])
        self.assertEqual(parser.stats["version_errors"], 1)

    def test_every_fragmentation_boundary_and_bytewise_feed(self):
        data = telemetry_fixture()
        for split in range(129):
            with self.subTest(split=split):
                parser = protocol.FrameStream()
                rows = parser.feed(data[:split]) + parser.feed(data[split:])
                self.assertEqual([r["packet_seq"] for r in rows], [20])
                self.assertEqual(parser.buffered_bytes, 0)
        parser = protocol.FrameStream()
        rows = []
        for octet in data:
            rows += parser.feed(bytes((octet,)))
            self.assertLessEqual(parser.buffered_bytes, 127)
        self.assertEqual(len(rows), 1)

    def test_noise_corruption_dropped_bytes_and_embedded_sync_recover(self):
        data = telemetry_fixture(enc_count=0x5AA5)
        broken = bytearray(data)
        broken[51] ^= 0x80
        for prefix in (b"noise\xa5", bytes(broken), data[:21] + data[22:]):
            parser = protocol.FrameStream()
            rows = parser.feed(prefix + data)
            self.assertEqual([r["enc_count"] for r in rows], [0x5AA5])
            self.assertGreaterEqual(parser.stats["discarded_bytes"], len(prefix))
        parser = protocol.FrameStream()
        self.assertEqual(parser.feed(b"noise\xa5"), [])
        self.assertEqual(len(parser.feed(data[1:])), 1)
        self.assertEqual(parser.stats["discarded_bytes"], 5)

    def test_noise_tail_and_stats_are_bounded(self):
        parser = protocol.FrameStream()
        self.assertEqual(parser.feed(b"\x00" * 1_000_000 + b"\xa5"), [])
        self.assertEqual(parser.buffered_bytes, 1)
        self.assertEqual(parser.stats["bytes_received"], 1_000_001)
        self.assertEqual(parser.stats["discarded_bytes"], 1_000_000)
        self.assertEqual(parser.feed(b""), [])
        parser.feed(telemetry_fixture()[1:] + telemetry_fixture(packet_seq=21)[:97])
        self.assertEqual(parser.stats["frames_valid"], 1)
        self.assertEqual(parser.stats["incomplete_tail_bytes"], 97)
        self.assertEqual(parser.buffered_bytes, 97)


class ContinuityTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(hasattr(protocol, "FrameStream"), "Implement v4 FrameStream")

    def test_100_hz_packet_and_500_hz_raw_stride_five(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=21))
        self.assertIn("sample_seq_continuity_error", rows[1], "Implement v4 continuity")
        self.assertEqual(rows[1]["packet_seq_gap"], 0)
        self.assertFalse(rows[1]["sample_seq_continuity_error"])
        self.assertFalse(rows[1]["time_continuity_error"])
        self.assertTrue(rows[1]["continuity_ok"])
        self.assertEqual(parser.stats["continuity_errors"], 0)

    def test_gaps_scale_both_tick_and_raw_stride(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=23))
        self.assertIn("packet_seq_gap", rows[1], "Implement v4 continuity")
        self.assertEqual(rows[1]["packet_seq_gap"], 2)
        self.assertEqual(parser.stats["packet_seq_missed"], 2)
        self.assertTrue(rows[1]["continuity_ok"])

    def test_v3_2ms_timing_or_raw_stride_one_is_flagged(self):
        for changes, diagnostic in ((dict(tick_us=1_202_000), "time_continuity_error"),
                                    (dict(raw_seq=101), "sample_seq_continuity_error")):
            parser = protocol.FrameStream()
            rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=21, **changes))
            self.assertIn(diagnostic, rows[1], "Implement v4 continuity")
            self.assertTrue(rows[1][diagnostic])
            self.assertFalse(rows[1]["continuity_ok"])
            self.assertEqual(parser.stats["continuity_errors"], 1)
            self.assertEqual(parser.stats["frames_valid"], 2)
            self.assertEqual(parser.stats["frames_invalid"], 0)

    def test_bad_forward_time_or_raw_stride_does_not_poison_anchor(self):
        for changes in (dict(raw_seq=300), dict(tick_us=1_215_000), dict(tick_us=1_200_000)):
            parser = protocol.FrameStream()
            rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=21, **changes)
                               + telemetry_fixture(packet_seq=21))
            self.assertIn("continuity_ok", rows[1], "Implement v4 continuity")
            self.assertFalse(rows[1]["continuity_ok"])
            self.assertTrue(rows[2]["continuity_ok"])
            self.assertEqual(rows[2]["packet_seq_gap"], 0)
            self.assertEqual(parser.stats["packet_seq_missed"], 0)

    def test_duplicates_and_counter_conflicts_preserve_anchor(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture() + telemetry_fixture()
                           + telemetry_fixture(raw_seq=101) + telemetry_fixture(packet_seq=21))
        self.assertIn("duplicate", rows[1], "Implement v4 continuity")
        self.assertTrue(rows[1]["duplicate"])
        self.assertFalse(rows[1]["continuity_ok"])
        self.assertFalse(rows[2]["duplicate"])
        self.assertTrue(rows[2]["out_of_order"])
        self.assertTrue(rows[2]["sample_seq_continuity_error"])
        self.assertEqual(parser.stats["duplicates"], 1)
        self.assertEqual(rows[3]["packet_seq_gap"], 0)

    def test_tick_above_32_bits_does_not_use_low_word(self):
        rows = protocol.FrameStream().feed(
            telemetry_fixture(tick_us=(1 << 32) - 5000)
            + telemetry_fixture(packet_seq=21, tick_us=(1 << 32) + 5000))
        self.assertIn("tick_us_unwrapped", rows[1], "Implement v4 continuity")
        self.assertEqual(rows[1]["tick_us_unwrapped"], (1 << 32) + 5000)
        self.assertTrue(rows[1]["continuity_ok"])
        self.assertFalse(rows[1]["reset"])

    def test_packet_raw_and_64_bit_tick_wraps(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture(packet_seq=0xFFFFFFFF, raw_seq=0xFFFFFFFD,
                                            tick_us=(1 << 64) - 5000)
                           + telemetry_fixture(packet_seq=0, raw_seq=2, tick_us=5000))
        self.assertIn("tick_us_unwrapped", rows[1], "Implement v4 continuity")
        self.assertEqual(rows[1]["tick_us_unwrapped"], (1 << 64) + 5000)
        self.assertFalse(rows[1]["reset"])
        self.assertTrue(rows[1]["continuity_ok"])
        for name in ("tick_wraps", "packet_seq_wraps", "raw_seq_wraps"):
            self.assertEqual(parser.stats[name], 1, name)

    def test_wrap_with_gap_uses_modular_raw_stride(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture(packet_seq=0xFFFFFFFE, raw_seq=0xFFFFFFFC,
                                            tick_us=500000)
                           + telemetry_fixture(packet_seq=1, raw_seq=11, tick_us=530000))
        self.assertIn("packet_seq_gap", rows[1], "Implement v4 continuity")
        self.assertEqual(rows[1]["packet_seq_gap"], 2)
        self.assertTrue(rows[1]["continuity_ok"])
        self.assertEqual(parser.stats["packet_seq_wraps"], 1)

    def test_reset_starts_epoch_without_inventing_wraps_or_missed_packets(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=0, raw_seq=0, tick_us=0)
                           + telemetry_fixture(packet_seq=1, raw_seq=5, tick_us=10000))
        self.assertIn("epoch", rows[1], "Implement v4 continuity")
        self.assertEqual([r["epoch"] for r in rows], [0, 1, 1])
        self.assertTrue(rows[1]["reset"])
        self.assertTrue(rows[1]["reset_or_reorder"])
        self.assertFalse(rows[1]["continuity_ok"])
        self.assertEqual(rows[1]["tick_us_unwrapped"], 0)
        self.assertEqual(parser.stats["packet_seq_missed"], 0)
        self.assertEqual(parser.stats["resets"], 1)
        self.assertEqual(parser.stats["tick_wraps"], 0)

    def test_forward_packet_with_backward_tick_is_suspected_reset(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=21, tick_us=10))
        self.assertIn("reset", rows[1], "Implement v4 continuity")
        self.assertTrue(rows[1]["reset"])
        self.assertTrue(rows[1]["reset_or_reorder"])
        self.assertEqual(rows[1]["epoch"], 1)
        self.assertEqual(parser.stats["tick_wraps"], 0)

    def test_nonzero_rollback_remains_ambiguous_without_advancing_anchor(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=3, raw_seq=15, tick_us=30000)
                           + telemetry_fixture(packet_seq=21))
        self.assertIn("out_of_order", rows[1], "Implement v4 continuity")
        self.assertTrue(rows[1]["out_of_order"])
        self.assertTrue(rows[1]["reset_or_reorder"])
        self.assertFalse(rows[1]["reset"])
        self.assertEqual(rows[1]["epoch"], 0)
        self.assertEqual(rows[1]["tick_us_unwrapped"], 30000)
        self.assertEqual(rows[2]["packet_seq_gap"], 0)
        self.assertEqual(parser.stats["packet_seq_missed"], 0)

    def test_reordered_pre_wrap_frame_cannot_manufacture_wrap(self):
        parser = protocol.FrameStream()
        rows = parser.feed(telemetry_fixture(packet_seq=1, raw_seq=5, tick_us=15000)
                           + telemetry_fixture(packet_seq=0xFFFFFFFF, raw_seq=0xFFFFFFFB,
                                               tick_us=(1 << 64) - 5000)
                           + telemetry_fixture(packet_seq=2, raw_seq=10, tick_us=25000))
        self.assertIn("out_of_order", rows[1], "Implement v4 continuity")
        self.assertTrue(rows[1]["out_of_order"])
        self.assertTrue(rows[1]["reset_or_reorder"])
        self.assertTrue(rows[2]["continuity_ok"])
        self.assertEqual(parser.stats["tick_wraps"], 0)
        self.assertEqual(parser.stats["packet_seq_missed"], 0)

    def test_half_range_and_incoherent_forward_gap_never_add_huge_loss(self):
        for packet in (20 + (1 << 31), 1_000_000):
            parser = protocol.FrameStream()
            rows = parser.feed(telemetry_fixture() + telemetry_fixture(packet_seq=packet,
                                                                       raw_seq=105, tick_us=1210000)
                               + telemetry_fixture(packet_seq=21))
            self.assertIn("continuity_ok", rows[1], "Implement v4 continuity")
            self.assertFalse(rows[1]["continuity_ok"])
            self.assertTrue(rows[2]["continuity_ok"])
            self.assertEqual(parser.stats["packet_seq_missed"], 0)

    def test_crc_rejected_frame_does_not_affect_continuity_anchor(self):
        parser = protocol.FrameStream()
        damaged = bytearray(telemetry_fixture(packet_seq=900))
        damaged[90] ^= 1
        rows = parser.feed(telemetry_fixture() + damaged + telemetry_fixture(packet_seq=21))
        self.assertIn("continuity_ok", rows[1], "Implement v4 continuity")
        self.assertTrue(rows[1]["continuity_ok"])
        self.assertEqual(rows[1]["packet_seq_gap"], 0)
        self.assertEqual(parser.stats["crc_errors"], 1)


if __name__ == "__main__":
    unittest.main()
