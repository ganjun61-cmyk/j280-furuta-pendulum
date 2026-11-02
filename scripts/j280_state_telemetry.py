#!/usr/bin/env python3
"""Read-only J280 state telemetry v2 (128 bytes, 100 Hz, 230400 baud).

FPGA samples at 500 Hz. Raw sample seq therefore advances by five per packet;
packet_seq is the transport sequence. No commands or motor operations exist.
"""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import sys
import time

from j280_measurement import FrameStream as RawStream, CSV_FIELDS as RAW_FIELDS, crc16_ccitt

SIZE = 128
MASK = 2**32-1
MAX_TIME = 2**52-1
EXTRA_STRUCT = struct.Struct('<IQqiiIIQQQIIBBHHHHHHIH')
EXTRA_FIELDS = (
    'segment_id', 'tick_us_unwrapped', 'wire_encoder_position',
    'wire_encoder_velocity_q16', 'wire_adc_velocity_q16',
    'wire_encoder_span_us', 'wire_adc_span_us', 'wire_adc_position_time_us',
    'wire_encoder_midpoint_x2_us', 'wire_adc_midpoint_x2_us',
    'wire_encoder_age_x2_us', 'wire_adc_age_x2_us', 'quality', 'reason_code',
    'estimator_version', 'window_us', 'sample_period_us', 'max_adc_age_us',
    'max_adc_step', 'max_encoder_step', 'packet_seq', 'telemetry_period_us')
REASONS = ('ready', 'window_incomplete', 'invalid_flags', 'invalid_fault',
           'invalid_adc_age', 'boundary_seq', 'boundary_tick', 'invalid_encoder_step',
           'invalid_adc_step', 'invalid_adc_source_time', 'invalid_numeric_range',
           'input_overrun', 'segment_start')
DERIVED_FIELDS = (
    'controller_ready', 'status', 'reason', 'position_valid',
    'encoder_velocity_valid', 'adc_velocity_valid', 'encoder_position_count',
    'adc_position_code', 'encoder_position_time_us', 'adc_position_time_us',
    'encoder_velocity_count_s', 'adc_velocity_code_s',
    'encoder_velocity_midpoint_us', 'adc_velocity_midpoint_us',
    'encoder_velocity_age_us', 'adc_velocity_age_us', 'encoder_velocity_span_us',
    'adc_velocity_span_us', 'packet_seq_gap', 'sample_seq_continuity_error',
    'time_continuity_error')
CSV_FIELDS = list(dict.fromkeys(list(RAW_FIELDS)+list(EXTRA_FIELDS)+list(DERIVED_FIELDS)))


def _decode(frame):
    if frame[2] != 2:
        raise ValueError('version_errors')
    if frame[3] & 0xf0 or any(frame[118:126]):
        raise ValueError('reserved_bits_errors')
    r = RawStream._decode(frame)
    r.update(zip(EXTRA_FIELDS, EXTRA_STRUCT.unpack_from(frame, 30)))
    q, reason, tick = r['quality'], r['reason_code'], r['tick_us_unwrapped']
    if (q not in (0, 1, 3, 5, 7) or reason >= len(REASONS)
            or r['estimator_version'] != 1 or r['state'] != 2
            or tick > MAX_TIME or tick & MASK != r['tick_us']
            or not 0 <= r['adc_raw10'] <= 1023
            or any(r[k] for k in ('output_enabled', 'motor_mode', 'pwm_count', 'pwm_period_count'))
            or not 0 < r['sample_period_us'] <= r['window_us']
            or r['window_us'] % r['sample_period_us']
            or r['max_adc_age_us'] >= r['sample_period_us']
            or r['max_adc_step'] > 1023
            or not r['telemetry_period_us']
            or r['telemetry_period_us'] % r['sample_period_us']):
        raise ValueError('domain_errors')
    if ((q == 7 and reason != 0) or (q != 7 and reason == 0)
            or (q in (1, 3, 5) and reason not in (1, 5, 6, 12))
            or (q in (3, 5) and reason != 1)
            or (q == 0 and reason not in (2, 3, 4, 7, 8, 9, 10, 11))):
        raise ValueError('domain_errors')
    pos_valid = bool(q & 1)
    if pos_valid:
        if (not r['adc_valid'] or not r['adc_new'] or r['fault_bits']
                or r['buffer_overflow'] or r['adc_age_us'] > r['max_adc_age_us']
                or tick < r['adc_age_us']
                or r['wire_encoder_position'] & MASK != r['enc_count'] & MASK
                or r['wire_adc_position_time_us'] != tick-r['adc_age_us']):
            raise ValueError('domain_errors')
    elif r['wire_encoder_position'] or r['wire_adc_position_time_us']:
        raise ValueError('domain_errors')
    r.update(controller_ready=False, status='ready' if q == 7 else 'warming' if q else 'invalid',
             reason=REASONS[reason], position_valid=pos_valid,
             encoder_position_count=r['wire_encoder_position'] if pos_valid else None,
             adc_position_code=r['adc_raw10'] if pos_valid else None,
             encoder_position_time_us=tick if pos_valid else None,
             adc_position_time_us=r['wire_adc_position_time_us'] if pos_valid else None)
    for channel, bit, unit in (('encoder', 2, 'count_s'), ('adc', 4, 'code_s')):
        valid = bool(q & bit)
        wire = f'wire_{channel}_'
        fields = ('velocity_q16', 'span_us', 'midpoint_x2_us', 'age_x2_us')
        if valid:
            end = tick if channel == 'encoder' else tick-r['adc_age_us']
            span = r[wire+'span_us']
            mid = r[wire+'midpoint_x2_us']
            if (span < r['window_us'] or span > end or mid != 2*end-span
                    or r[wire+'age_x2_us'] != 2*tick-mid):
                raise ValueError('domain_errors')
        elif any(r[wire+field] for field in fields):
            raise ValueError('domain_errors')
        r[channel+'_velocity_valid'] = valid
        r[channel+'_velocity_'+unit] = r[wire+'velocity_q16']/65536 if valid else None
        r[channel+'_velocity_midpoint_us'] = r[wire+'midpoint_x2_us']/2 if valid else None
        r[channel+'_velocity_age_us'] = r[wire+'age_x2_us']/2 if valid else None
        r[channel+'_velocity_span_us'] = r[wire+'span_us'] if valid else None
    return r


class FrameStream:
    """CRC resynchronization plus packet continuity; reboot has no unique boot ID."""
    def __init__(self):
        self._buffer = bytearray()
        self._anchor = None
        self._epoch = 0
        self.stats = dict.fromkeys(('bytes_received', 'frames_valid', 'frames_invalid',
                                   'crc_errors', 'version_errors', 'reserved_bits_errors',
                                   'domain_errors', 'discarded_bytes', 'incomplete_tail_bytes',
                                   'packet_seq_missed', 'duplicates', 'out_of_order',
                                   'reset_or_reorder', 'continuity_errors'), 0)

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    def feed(self, data):
        self.stats['bytes_received'] += len(data)
        self._buffer.extend(data)
        rows = []
        cursor = 0
        while cursor < len(self._buffer):
            start = self._buffer.find(b'\xa5\x5a', cursor)
            if start < 0:
                keep = int(self._buffer[-1] == 0xa5)
                self.stats['discarded_bytes'] += len(self._buffer)-cursor-keep
                cursor = len(self._buffer)-keep
                break
            self.stats['discarded_bytes'] += start-cursor
            cursor = start
            if len(self._buffer)-start < SIZE:
                break
            candidate = self._buffer[start:start+SIZE]
            try:
                if crc16_ccitt(candidate[2:126]) != struct.unpack_from('<H',candidate,126)[0]:
                    raise ValueError('crc_errors')
                row = _decode(candidate)
            except ValueError as error:
                self.stats['frames_invalid'] += 1
                self.stats[str(error)] += 1
                self.stats['discarded_bytes'] += 1
                cursor += 1
                continue
            self._continuity(row)
            rows.append(row)
            self.stats['frames_valid'] += 1
            cursor += SIZE
        del self._buffer[:cursor]
        self.stats['incomplete_tail_bytes'] = len(self._buffer)
        return rows

    def _continuity(self, r):
        r.update(packet_seq_gap=0, seq_gap=0, sample_seq_continuity_error=False,
                 time_continuity_error=False, duplicate=False, out_of_order=False,
                 reset=False, reset_or_reorder=False, epoch=self._epoch)
        current = {k:r[k] for k in ('packet_seq','seq','tick_us_unwrapped')}
        previous = self._anchor
        if previous is None:
            self._anchor = current
            return
        delta = (r['packet_seq']-previous['packet_seq']) & MASK
        tick_delta = r['tick_us_unwrapped']-previous['tick_us_unwrapped']
        if delta == 0 and tick_delta == 0:
            r['duplicate'] = True
            self.stats['duplicates'] += 1
            return
        if (tick_delta < 0 and (r['packet_seq'] == 0 or 0 < delta < 2**31)):
            r.update(reset=True, reset_or_reorder=True)
            self._epoch += 1
            r['epoch'] = self._epoch
            self.stats['reset_or_reorder'] += 1
            self._anchor = current
            return
        if not 0 < delta < 2**31 or tick_delta < 0:
            r['out_of_order'] = True
            r['reset_or_reorder'] = tick_delta < 0
            self.stats['out_of_order'] += 1
            self.stats['reset_or_reorder'] += int(r['reset_or_reorder'])
            return
        r['packet_seq_gap'] = r['seq_gap'] = delta-1
        self.stats['packet_seq_missed'] += delta-1
        stride = r['telemetry_period_us']//r['sample_period_us']
        r['sample_seq_continuity_error'] = ((r['seq']-previous['seq']) & MASK) != ((stride*delta) & MASK)
        r['time_continuity_error'] = tick_delta != r['telemetry_period_us']*delta
        self.stats['continuity_errors'] += int(r['sample_seq_continuity_error'] or r['time_continuity_error'])
        self._anchor = current


def _write(writer, rows, source, host_time=''):
    for row in rows:
        row.update(session_id='state-v2', source=str(source), host_time_iso=host_time)
        row['notes'] = ';'.join(k for k in ('duplicate','out_of_order','reset_or_reorder',
                         'sample_seq_continuity_error','time_continuity_error') if row[k])
        writer.writerow(row)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    commands = parser.add_subparsers(dest='command',required=True)
    decode = commands.add_parser('decode')
    decode.add_argument('--input',type=Path,required=True)
    capture = commands.add_parser('capture')
    capture.add_argument('--port',required=True)
    capture.add_argument('--seconds',type=float,required=True)
    capture.add_argument('--baud',type=int,default=230400)
    for command in (decode,capture):
        command.add_argument('--output',type=Path,required=True)
        command.add_argument('--raw-output',type=Path)
    args = parser.parse_args(argv)
    stream = FrameStream()
    try:
        paths = [args.output]+([args.raw_output] if args.raw_output else [])
        if len({p.resolve() for p in paths}) != len(paths):
            raise ValueError('outputs must be different paths')
        if any(p.exists() or p.is_symlink() for p in paths):
            raise FileExistsError('output already exists')
        if args.command == 'decode':
            if any(args.input.resolve() == p.resolve() for p in paths):
                raise ValueError('output aliases input')
        elif not math.isfinite(args.seconds) or args.seconds <= 0 or args.baud <= 0:
            raise ValueError('capture duration and baud must be positive and finite')
        with ExitStack() as stack:
            target = stack.enter_context(args.output.open('x',newline='',encoding='utf-8'))
            writer = csv.DictWriter(target,fieldnames=CSV_FIELDS)
            writer.writeheader()
            raw = stack.enter_context(args.raw_output.open('xb')) if args.raw_output else None
            if args.command == 'decode':
                source = stack.enter_context(args.input.open('rb'))
                while chunk := source.read(65536):
                    if raw:raw.write(chunk)
                    _write(writer,stream.feed(chunk),args.input)
            else:
                import serial
                port = stack.enter_context(serial.Serial(port=args.port,baudrate=args.baud,
                                                         timeout=min(.1,args.seconds)))
                deadline = time.monotonic()+args.seconds
                while (remaining := deadline-time.monotonic()) > 0:
                    port.timeout = min(.1,remaining)
                    chunk = port.read(4096)
                    if not chunk:continue
                    if raw:raw.write(chunk)
                    _write(writer,stream.feed(chunk),args.port,
                           datetime.now(timezone.utc).isoformat(timespec='microseconds'))
        print(json.dumps(stream.stats,allow_nan=False))
    except (OSError, ValueError, ImportError) as error:
        print(json.dumps(dict(error=str(error)),ensure_ascii=True),file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
