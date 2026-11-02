#!/usr/bin/env python3
"""Decode J280 v1 passive measurement frames, offline or during bounded capture.

The public API is ``crc16_ccitt(data)`` and ``FrameStream.feed(chunk)``. ``feed``
returns dictionaries of CRC-checked, semantically valid frames. ``stats`` is a
dictionary of integer counters; ``incomplete_tail_bytes`` is the current retained
tail, rather than an accumulated counter. At most 31 bytes remain after a feed.
Rejected sync candidates increment ``frames_invalid`` and one reason counter;
``discarded_bytes`` also counts noise and bytes discarded during resynchronization.

Sequence/timestamp continuity is heuristic because v1 has no boot identifier.
Modulo-forward sequence and tick differences below 2**31 support normal wraps.
A backwards tick with a forward sequence, or a backwards sequence with new seq==0
or tick==0, starts an epoch. Other backwards sequences accompanied by a backwards
tick (numeric or modular) are marked ``reset_or_reorder`` and ``out_of_order``
without moving the anchor or adding a new tick wrap. A
duplicate sequence also leaves the anchor unchanged. ``tick_us_unwrapped`` is
local to its epoch; on reordered/ambiguous frames it uses the signed modular
distance to the anchor, never a spurious full-wrap elapsed time. Do not concatenate
epochs as elapsed time. Gaps >=2**31 us (about 35.8 minutes) are ambiguous too.

CSV keeps the j280-dynamic template columns and appends explicit diagnostics.
Only ADC counts and encoder counts are reported; no angle calibration is inferred.
The host receive timestamp identifies a read chunk, not the sensor sample time.
Capture imports optional pyserial only in capture mode, reads only, and closes the
port in finally. Opening a port can itself affect hardware-controlled DTR/RTS;
this tool issues no DTR/RTS assignments or control commands.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import sys
import time
from contextlib import ExitStack


FRAME_SIZE = 32
SYNC = b"\xa5\x5a"
MASK32 = (1 << 32) - 1
HALF32 = 1 << 31
CSV_FIELDS = [
    "session_id", "source", "host_time_iso", "seq", "tick_us", "adc_raw10",
    "adc_valid", "adc_new", "adc_age_us", "enc_count", "pwm_count",
    "pwm_period_count", "motor_mode", "state", "fault_bits", "pwm_age_us",
    "buffer_overflow", "frame_valid", "crc_ok", "notes", "output_enabled",
    "tick_us_unwrapped", "epoch", "seq_gap", "duplicate", "out_of_order",
    "reset", "reset_or_reorder",
]


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection/xorout."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class FrameStream:
    """Incremental fixed-size decoder. Public stats counters are described above."""

    def __init__(self):
        self.stats = dict.fromkeys((
            "bytes_received", "frames_valid", "frames_invalid", "crc_errors",
            "version_errors", "reserved_bits_errors", "domain_errors",
            "discarded_bytes", "incomplete_tail_bytes", "seq_missed",
            "seq_duplicates", "seq_out_of_order", "resets", "tick_wraps",
            "seq_wraps", "uncertain_rollbacks",
        ), 0)
        self._buffer = bytearray()
        self._anchor = None
        self._epoch = 0

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    def feed(self, chunk: bytes) -> list[dict]:
        """Consume bytes, resynchronize after errors, return valid measurements."""
        self.stats["bytes_received"] += len(chunk)
        self._buffer.extend(chunk)
        rows = []
        cursor = 0
        length = len(self._buffer)
        while cursor < length:
            start = self._buffer.find(SYNC, cursor)
            if start < 0:
                # Keep a final A5, which may be half of the next sync word.
                keep = int(self._buffer[-1] == SYNC[0])
                self.stats["discarded_bytes"] += length - cursor - keep
                cursor = length - keep
                break
            self.stats["discarded_bytes"] += start - cursor
            cursor = start
            if length - start < FRAME_SIZE:
                break
            candidate = self._buffer[start:start + FRAME_SIZE]
            reason = self._invalid_reason(candidate)
            if reason:
                self.stats["frames_invalid"] += 1
                self.stats[reason] += 1
                self.stats["discarded_bytes"] += 1
                cursor += 1
                continue
            row = self._decode(candidate)
            self._continuity(row)
            rows.append(row)
            self.stats["frames_valid"] += 1
            cursor += FRAME_SIZE
        del self._buffer[:cursor]
        self.stats["incomplete_tail_bytes"] = len(self._buffer)
        return rows

    @staticmethod
    def _invalid_reason(data):
        if crc16_ccitt(data[2:30]) != struct.unpack_from("<H", data, 30)[0]:
            return "crc_errors"
        if data[2] != 1:
            return "version_errors"
        if data[3] & 0xF0 or data[24] & 0xFC:
            return "reserved_bits_errors"
        if struct.unpack_from("<H", data, 12)[0] > 1023:
            return "domain_errors"
        return None

    @staticmethod
    def _decode(data):
        (_, flags, seq, tick, raw, adc_age, encoder, pwm_count, pwm_period,
         mode, state, fault, pwm_age) = struct.unpack_from("<BBIIHHiHHBBHH", data, 2)
        return {
            "seq": seq, "tick_us": tick, "adc_raw10": raw,
            "adc_valid": bool(flags & 1), "adc_new": bool(flags & 2),
            "output_enabled": bool(flags & 4), "buffer_overflow": bool(flags & 8),
            "adc_age_us": adc_age, "enc_count": encoder, "pwm_count": pwm_count,
            "pwm_period_count": pwm_period, "motor_mode": mode, "state": state,
            "fault_bits": fault, "pwm_age_us": pwm_age,
            "frame_valid": True, "crc_ok": True,
        }

    def _continuity(self, row):
        seq, tick = row["seq"], row["tick_us"]
        row.update(seq_gap=0, duplicate=False, out_of_order=False, reset=False,
                   reset_or_reorder=False, epoch=self._epoch, tick_us_unwrapped=tick)
        if self._anchor is None:
            self._anchor = seq, tick, tick
            return
        previous_seq, previous_tick, previous_unwrapped = self._anchor
        seq_delta = (seq - previous_seq) & MASK32
        tick_delta = (tick - previous_tick) & MASK32
        seq_forward = 0 < seq_delta < HALF32
        tick_forward = tick_delta < HALF32
        signed_tick_delta = tick_delta if tick_forward else tick_delta - (1 << 32)
        row["tick_us_unwrapped"] = previous_unwrapped + signed_tick_delta
        if seq_delta == 0:
            row["duplicate"] = True
            self.stats["seq_duplicates"] += 1
            return
        if seq_forward and tick_forward:
            row["seq_gap"] = seq_delta - 1
            self.stats["seq_missed"] += seq_delta - 1
            self.stats["seq_wraps"] += int(seq < previous_seq)
            self.stats["tick_wraps"] += int(tick < previous_tick)
        elif ((not tick_forward and seq_forward)
              or (not seq_forward and (seq == 0 or tick == 0))):
            self._epoch += 1
            self.stats["resets"] += 1
            row.update(epoch=self._epoch, reset=True, tick_us_unwrapped=tick)
        else:
            row["out_of_order"] = True
            self.stats["seq_out_of_order"] += 1
            if not seq_forward and (not tick_forward or tick < previous_tick):
                row["reset_or_reorder"] = True
                row["tick_us_unwrapped"] = previous_unwrapped - previous_tick + tick
                self.stats["uncertain_rollbacks"] += 1
            return
        self._anchor = seq, tick, row["tick_us_unwrapped"]


def _writer(handle):
    writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
    writer.writeheader()
    return writer


def _write_rows(writer, rows, session_id, source, host_time=""):
    for row in rows:
        row.update(session_id=session_id, source=source, host_time_iso=host_time)
        row["notes"] = ";".join(name for name in (
            "duplicate", "out_of_order", "reset", "reset_or_reorder", "buffer_overflow"
        ) if row[name])
        writer.writerow(row)


def _validate_outputs(output, raw_output=None):
    paths = [output] + ([raw_output] if raw_output is not None else [])
    if len({path.resolve() for path in paths}) != len(paths):
        raise ValueError("CSV output and raw output must be different paths")
    for path in paths:
        if path.exists():
            raise FileExistsError(f"output already exists: {path}")


def _decode_file(args, parser):
    _validate_outputs(args.output)
    with args.input.open("rb") as source, args.output.open("x", newline="", encoding="utf-8") as target:
        writer = _writer(target)
        while chunk := source.read(65536):
            _write_rows(writer, parser.feed(chunk), "decode", str(args.input))


def _capture(args, parser):
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    if args.baud <= 0:
        raise ValueError("baud must be positive")
    _validate_outputs(args.output, args.raw_output)
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("capture requires optional pyserial; install with: python -m pip install pyserial") from exc

    with ExitStack() as stack:
        target = stack.enter_context(args.output.open("x", newline="", encoding="utf-8"))
        raw = stack.enter_context(args.raw_output.open("xb")) if args.raw_output else None
        writer = _writer(target)
        # No serial writes, DTR/RTS assignments, or output-control commands.
        port = serial.Serial(port=args.port, baudrate=args.baud, timeout=min(0.1, args.seconds))
        try:
            deadline = time.monotonic() + args.seconds
            while (remaining := deadline - time.monotonic()) > 0:
                port.timeout = min(0.1, remaining)
                chunk = port.read(4096)
                if not chunk:
                    continue
                if raw:
                    raw.write(chunk)
                host_time = datetime.now(timezone.utc).isoformat(timespec="microseconds")
                _write_rows(writer, parser.feed(chunk), "capture", args.port, host_time)
        finally:
            port.close()


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    commands = cli.add_subparsers(dest="command", required=True)
    decode = commands.add_parser("decode", help="decode an existing binary file without serial access")
    decode.add_argument("--input", type=Path, required=True)
    decode.add_argument("--output", type=Path, required=True)
    capture = commands.add_parser("capture", help="read frames for a finite duration; never send commands")
    capture.add_argument("--port", required=True)
    capture.add_argument("--seconds", type=float, required=True)
    capture.add_argument("--baud", type=int, default=230400)
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--raw-output", type=Path)
    args = cli.parse_args(argv)
    parser = FrameStream()
    code = 0
    try:
        if args.command == "decode":
            _decode_file(args, parser)
        else:
            _capture(args, parser)
    except KeyboardInterrupt:
        print(json.dumps({"error": "interrupted; partial output retained"}), file=sys.stderr)
        code = 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=True), file=sys.stderr)
        code = 2
    finally:
        print(json.dumps(parser.stats, sort_keys=True))
    return code


if __name__ == "__main__":
    sys.exit(main())
