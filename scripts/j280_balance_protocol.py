"""Offline J280 balance v4 wire protocol, independent of the arithmetic model.

CONFIG, KEEPALIVE and ABORT are the only host commands. This module performs
no serial I/O. Configuration validation checks the wire and safety contract;
it does not establish that a set of controller coefficients is stable.
"""

import struct

from j280_measurement import crc16_ccitt


VERSION = 4
SYNC = b"\xa5\x5a"
COMMAND_SIZE = 192
FRAME_SIZE = 128
SAMPLE_PERIOD_US = 2000
TELEMETRY_PERIOD_US = 10000
PWM_PERIOD = 2500
MASK32 = (1 << 32) - 1
MASK64 = (1 << 64) - 1
COMMANDS = {"config": 1, "keepalive": 2, "abort": 3}
STATES = {0: "SAFE", 1: "ARMED", 2: "CAPTURE", 3: "BALANCE", 255: "FAULT"}
STOP_REASONS = {
    0: "none", 1: "complete", 2: "stop_key", 3: "arm_off", 4: "host_abort",
    5: "arm_timeout", 6: "capture_timeout", 16: "sensor_invalid",
    17: "sensor_stale", 18: "travel_limit", 19: "speed_limit",
    20: "keepalive_timeout", 21: "angle_limit", 22: "numeric_range",
    23: "control_overrun", 24: "pwm_reject",
}
VALUE_FIELDS = (
    "config_id", "algorithm_id", "run_ms", "keepalive_ms",
    "travel_limit_count", "speed_limit_count_s", "adc_speed_limit_code_s",
    "max_compare", "hold_ms", "wait_ms", "upright_zero_q16", "enter_code",
    "exit_code", "comp_compare", "blend_compare", "comp_gain_q20",
    "coefficients", "sample_period_us",
)
VALUE_RANGES = {
    "config_id": (1, 65535), "algorithm_id": (1, 1),
    "run_ms": (1, 30000), "keepalive_ms": (1, 500),
    "travel_limit_count": (1, 300), "speed_limit_count_s": (1, 10000),
    "adc_speed_limit_code_s": (1, 30000), "max_compare": (1, 1125),
    "hold_ms": (200, 1000), "wait_ms": (202, 10000),
    "upright_zero_q16": (0, 1023 << 16), "enter_code": (0, 9),
    "exit_code": (1, 57), "comp_compare": (0, 250),
    "blend_compare": (25, 250), "comp_gain_q20": (0, 10 << 20),
    "sample_period_us": (SAMPLE_PERIOD_US, SAMPLE_PERIOD_US),
}


def validate_values(values, ranges=None) -> dict:
    """Return a defensive copy; all wire quantities must be integers, not bools."""
    if not isinstance(values, dict) or set(values) != set(VALUE_FIELDS):
        raise ValueError(f"values must be exactly {VALUE_FIELDS}")
    for key, (lower, upper) in (ranges or VALUE_RANGES).items():
        value = values[key]
        if type(value) is not int or not lower <= value <= upper:
            raise ValueError(f"{key} must be an integer in {lower}..{upper}")
    coefficients = values["coefficients"]
    if (not isinstance(coefficients, (list, tuple)) or len(coefficients) != 32
            or any(type(c) is not int or abs(c) > (128 << 20) for c in coefficients)):
        raise ValueError("coefficients must be 32 int Q20 values with abs <= 128 << 20")
    if values["exit_code"] <= values["enter_code"]:
        raise ValueError("exit_code must be greater than enter_code")
    if values["wait_ms"] < values["hold_ms"] + 2:
        raise ValueError("wait_ms must be at least hold_ms + 2")
    if values["comp_compare"] > values["max_compare"]:
        raise ValueError("comp_compare must not exceed min(250, max_compare)")
    blend = values["blend_compare"]
    error = values["comp_gain_q20"] * blend - (values["comp_compare"] << 20)
    if abs(error) > (blend + 1) // 2:
        raise ValueError("comp_gain_q20 is inconsistent with comp_compare / blend_compare")
    return dict(values, coefficients=list(coefficients))


def build_command(name, values=None, *, version=VERSION, ranges=None) -> bytes:
    """Build a fixed 192-byte command with a little-endian CCITT-FALSE CRC."""
    if not isinstance(name, str) or name not in COMMANDS:
        raise ValueError("command must be config, keepalive or abort")
    data = bytearray(COMMAND_SIZE)
    data[:4] = SYNC + bytes((version, COMMANDS[name]))
    if name == "config":
        v = validate_values(values, ranges)
        struct.pack_into("<10Hi4Hi", data, 4,
                         *(v[k] for k in VALUE_FIELDS[:10]),
                         v["upright_zero_q16"], v["enter_code"], v["exit_code"],
                         v["comp_compare"], v["blend_compare"], v["comp_gain_q20"])
        struct.pack_into("<32iH", data, 40, *v["coefficients"], v["sample_period_us"])
    elif values is not None:
        raise ValueError(f"{name} takes no values")
    struct.pack_into("<H", data, 190, crc16_ccitt(data[2:190]))
    return bytes(data)


def _decode(data, *, version=VERSION, states=STATES, reasons=STOP_REASONS,
            active_states=(1, 2, 3), drive_states=(3,), reserved=slice(104, 126), constants=True):
    """constants=False (v6) skips the PWM/sample/telemetry period checks: those
    words carry the trajectory status, reference speed and feedforward instead."""
    if data[2] != version:
        raise ValueError("version_errors")
    if any(data[reserved]):
        raise ValueError("reserved_bits_errors")
    row = dict(zip((
        "version", "flags", "packet_seq", "tick_us", "raw_seq", "adc_raw10",
        "adc_age_us", "enc_count", "theta_ref", "upright_zero_q16", "config_id",
        "config_crc", "state_code", "stop_reason_code", "input_levels",
        "motor_mode", "applied_compare", "pwm_period", "u_linear_q16",
        "u_compensated_q16",
    ), struct.unpack_from("<BBIQIHHiiiHHBBBBHHii", data, 2)))
    row["xhat"] = list(struct.unpack_from("<4i", data, 56))
    row.update(zip((
        "controller_age_us", "fault_bits", "raw_estimator_quality", "rx_status",
        "controller_latency_cycles", "sample_period_us", "telemetry_period_us",
        "state_elapsed_ms", "safety_position_delta", "u_model_q16",
    ), struct.unpack_from("<IHBB4Iii", data, 72)))
    for bit, name in enumerate(("adc_valid", "adc_new", "output_enabled", "tx_overflow",
                                "config_loaded", "arm_on", "core_ok", "fault_latched")):
        row[name] = bool(row["flags"] & (1 << bit))
    row["state"] = states.get(row["state_code"])
    row["stop_reason"] = reasons.get(row["stop_reason_code"])
    if (row["state"] is None or row["stop_reason"] is None
            or row["input_levels"] & 0xF8
            or row["fault_bits"] & 0xFFF8
            or (row["rx_status"] & 0x0F) > 7
            or row["adc_raw10"] > 1023
            or not 0 <= row["upright_zero_q16"] <= (1023 << 16)
            or (constants and (row["pwm_period"] != PWM_PERIOD
                               or row["sample_period_us"] != SAMPLE_PERIOD_US
                               or row["telemetry_period_us"] != TELEMETRY_PERIOD_US))
            or row["raw_estimator_quality"] not in (0, 1, 3, 5, 7)
            or row["adc_valid"] == bool(row["fault_bits"] & 1)
            or row["tx_overflow"] != bool(row["fault_bits"] & 4)
            or row["fault_latched"] != (row["state_code"] == 255)):
        raise ValueError("domain_errors")
    if (row["config_loaded"] != bool(row["config_id"])
            or (not row["config_loaded"] and row["config_crc"] != 0)
            or (row["state_code"] in active_states and not row["config_loaded"])
            or (row["state_code"] == 255 and row["config_loaded"])):
        raise ValueError("domain_errors")
    compiled_limit_q16 = VALUE_RANGES["max_compare"][1] << 16
    if any(abs(row[key]) > compiled_limit_q16
           for key in ("u_linear_q16", "u_compensated_q16", "u_model_q16")):
        raise ValueError("domain_errors")
    # xhat remains full signed int32: large estimates can diagnose a speed fault.
    if row["output_enabled"]:
        drive_ok = (row["state_code"] in drive_states and row["motor_mode"] in (1, 2)
                    and row["applied_compare"] <= VALUE_RANGES["max_compare"][1])
    else:
        drive_ok = row["motor_mode"] == 4 and row["applied_compare"] == 0
    if not drive_ok:
        raise ValueError("domain_errors")
    # Keep established host-tool names alongside the explicit v4 wire names.
    row.update(seq=row["raw_seq"], active_B_motor_mode=row["motor_mode"],
               pwm_count=row["applied_compare"], pwm_period_count=row["pwm_period"],
               buffer_overflow=row["tx_overflow"], estimator_quality=row["raw_estimator_quality"],
               in1=bool(row["motor_mode"] & 1), in2=bool(row["motor_mode"] & 2),
               pwm_idle_high=bool(row["motor_mode"] & 4), frame_valid=True, crc_ok=True)
    return row


class FrameStream:
    """CRC-checked 128-byte frames; at most 127 bytes retained after each feed.

    Invalid candidates increment frames_invalid and one reason counter. Noise
    and rejected bytes count towards discarded_bytes. incomplete_tail_bytes is
    a current length, not an accumulated counter.

    frame_valid/CRC validity does not imply continuity_ok. packet_seq_gap (also
    seq_gap) and packet_seq_missed count gaps between coherent anchors, whose
    tick advances by 10000 us and raw_seq by five per packet. Duplicates,
    reorders and incoherent forward frames do not advance the anchor.

    There is no boot identifier. A backward clock with a forward packet counter,
    or with packet_seq/tick zero, starts a *suspected* reset epoch and sets both
    reset and reset_or_reorder. Other rollbacks remain ambiguous and preserve
    the anchor. Consumers must treat either reset flag as loss of continuity.
    Tick wraps require the exact 100 Hz modular time difference. Unwrapped time
    is local to an epoch; questionable rows retain their wire tick value.
    """

    def __init__(self, decode=None):
        self._decode = decode or _decode
        self._buffer = bytearray()
        self._anchor = None
        self._epoch = 0
        self.stats = dict.fromkeys((
            "bytes_received", "frames_valid", "frames_invalid", "crc_errors",
            "version_errors", "reserved_bits_errors", "domain_errors",
            "discarded_bytes", "incomplete_tail_bytes",
            "packet_seq_missed", "duplicates", "out_of_order", "reset_or_reorder",
            "resets", "tick_wraps", "packet_seq_wraps", "raw_seq_wraps",
            "continuity_errors", "time_continuity_errors", "sample_seq_continuity_errors",
        ), 0)

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    def feed(self, chunk) -> list[dict]:
        self.stats["bytes_received"] += len(chunk)
        self._buffer.extend(chunk)
        rows = []
        cursor = 0
        length = len(self._buffer)
        while cursor < length:
            start = self._buffer.find(SYNC, cursor)
            if start < 0:
                keep = int(self._buffer[-1] == SYNC[0])
                self.stats["discarded_bytes"] += length - cursor - keep
                cursor = length - keep
                break
            self.stats["discarded_bytes"] += start - cursor
            cursor = start
            if length - start < FRAME_SIZE:
                break
            candidate = self._buffer[start:start + FRAME_SIZE]
            try:
                if crc16_ccitt(candidate[2:126]) != struct.unpack_from("<H", candidate, 126)[0]:
                    raise ValueError("crc_errors")
                row = self._decode(candidate)
            except ValueError as error:
                self.stats["frames_invalid"] += 1
                self.stats[str(error)] += 1
                self.stats["discarded_bytes"] += 1
                cursor += 1
                continue
            self._continuity(row)
            rows.append(row)
            self.stats["frames_valid"] += 1
            cursor += FRAME_SIZE
        del self._buffer[:cursor]
        self.stats["incomplete_tail_bytes"] = len(self._buffer)
        return rows

    def _continuity(self, row):
        packet, tick, raw = row["packet_seq"], row["tick_us"], row["raw_seq"]
        row.update(packet_seq_gap=0, seq_gap=0, sample_seq_continuity_error=False,
                   time_continuity_error=False, duplicate=False, out_of_order=False,
                   reset=False, reset_or_reorder=False, epoch=self._epoch,
                   tick_us_unwrapped=tick, continuity_ok=True)
        if self._anchor is None:
            self._anchor = (packet, tick, raw, tick)
            return
        previous_packet, previous_tick, previous_raw, previous_unwrapped = self._anchor
        packet_delta = (packet - previous_packet) & MASK32
        tick_delta = (tick - previous_tick) & MASK64
        raw_delta = (raw - previous_raw) & MASK32
        if packet_delta == tick_delta == raw_delta == 0:
            row.update(duplicate=True, continuity_ok=False, tick_us_unwrapped=previous_unwrapped)
            self.stats["duplicates"] += 1
            return
        forward = 0 < packet_delta < (1 << 31)
        expected_tick = packet_delta * TELEMETRY_PERIOD_US
        time_coherent = forward and tick_delta == expected_tick
        if (tick < previous_tick and not time_coherent
                and (forward or packet == 0 or tick == 0)):
            self._epoch += 1
            row.update(reset=True, reset_or_reorder=True, epoch=self._epoch, continuity_ok=False)
            self.stats["resets"] += 1
            self.stats["reset_or_reorder"] += 1
            self._anchor = (packet, tick, raw, tick)
            return
        if not forward:
            row.update(out_of_order=True, continuity_ok=False)
            row["reset_or_reorder"] = packet_delta >= (1 << 31) or tick_delta >= (1 << 63)
            self.stats["out_of_order"] += 1
            self.stats["reset_or_reorder"] += int(row["reset_or_reorder"])
            if packet_delta != 0:
                return
        stride = TELEMETRY_PERIOD_US // SAMPLE_PERIOD_US
        row["time_continuity_error"] = tick_delta != expected_tick
        row["sample_seq_continuity_error"] = raw_delta != ((stride * packet_delta) & MASK32)
        if row["time_continuity_error"] or row["sample_seq_continuity_error"]:
            row["continuity_ok"] = False
            self.stats["continuity_errors"] += 1
            self.stats["time_continuity_errors"] += int(row["time_continuity_error"])
            self.stats["sample_seq_continuity_errors"] += int(row["sample_seq_continuity_error"])
            return
        row["packet_seq_gap"] = row["seq_gap"] = packet_delta - 1
        row["tick_us_unwrapped"] = previous_unwrapped + tick_delta
        self.stats["packet_seq_missed"] += packet_delta - 1
        self.stats["packet_seq_wraps"] += int(packet < previous_packet)
        self.stats["tick_wraps"] += int(tick < previous_tick)
        self.stats["raw_seq_wraps"] += (previous_raw + stride * packet_delta) >> 32
        self._anchor = (packet, tick, raw, row["tick_us_unwrapped"])
