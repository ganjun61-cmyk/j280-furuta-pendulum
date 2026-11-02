#!/usr/bin/env python3
"""J280 bounded motor test v3: telemetry decoder, command frames and runner.

The FPGA sends a 40-byte frame per 2 ms snapshot (500 Hz, 230400 8N1). The PC
sends 20-byte commands. Motion can only start from SW1 on the board after the
FPGA has accepted a CONFIG frame and been armed with DIP switch 1; the PC can
load, keep alive and abort, never start.
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

from j280_measurement import FrameStream as V1Stream, crc16_ccitt

VERSION = 3
FRAME_SIZE = 40
COMMAND_SIZE = 20
SAMPLE_PERIOD_US = 2000
PWM_PERIOD = 2500
TELEMETRY_TIMEOUT_S = 0.5
RUN_GRACE_S = 0.5
LIMITS = {'duty_count': 1000, 'pulse_ms': 2000, 'travel_limit_count': 200,
          'speed_limit_count_s': 10000, 'keepalive_timeout_ms': 500}
VALUE_FIELDS = ('config_id', 'direction', 'duty_count', 'pulse_ms', 'travel_limit_count',
                'speed_limit_count_s', 'keepalive_timeout_ms')
STATES = {0: 'SAFE', 1: 'ARMED', 2: 'RUN', 255: 'FAULT'}
STOP_REASONS = {0: 'none', 1: 'complete', 2: 'stop_key', 3: 'arm_off', 4: 'host_abort',
                5: 'arm_timeout', 16: 'sensor_invalid', 17: 'sensor_stale',
                18: 'travel_limit', 19: 'speed_limit', 20: 'keepalive_timeout',
                21: 'output_reject', 22: 'run_overtime'}
RX_CODES = {0: 'none', 1: 'config_accepted', 2: 'keepalive', 3: 'abort', 4: 'reject_state',
            5: 'reject_range', 6: 'reject_command', 7: 'frame_error'}
COMMANDS = {'config': 1, 'keepalive': 2, 'abort': 3}
DIRECTIONS = {'in1_high': 1, 'in2_high': 2}
PAYLOAD = struct.Struct('<BBIIHHiHHBBHHBBBBHH')
FIELDS = ('version', 'flags', 'seq', 'tick_us', 'adc_raw10', 'adc_age_us', 'enc_count',
          'pwm_count', 'pwm_period_count', 'motor_mode', 'state_code', 'fault_bits',
          'pwm_age_us', 'stop_reason_code', 'estimator_byte', 'rx_status', 'input_levels',
          'config_id', 'config_crc')
DERIVED = ('state', 'stop_reason', 'adc_valid', 'adc_new', 'output_enabled', 'buffer_overflow',
           'config_loaded', 'arm_on', 'estimator_ok', 'in1', 'in2', 'pwm_idle_high',
           'estimator_quality', 'estimator_fresh', 'rx_code', 'rx_frame_count',
           'sw1_level', 'sw2_level', 'dip1_level')
CONTINUITY = ('tick_us_unwrapped', 'epoch', 'seq_gap', 'duplicate', 'out_of_order', 'reset',
              'reset_or_reorder', 'time_continuity_error')
CSV_FIELDS = ['host_time_iso', *FIELDS, *DERIVED, *CONTINUITY, 'notes']


def command_frame(payload):
    payload = bytes(payload)
    if len(payload) != 16:
        raise ValueError('command payload must be 16 bytes')
    return b'\xa5\x5a' + payload + struct.pack('<H', crc16_ccitt(payload))


def build_command(command, values=None):
    """CONFIG takes the validated values dictionary; other commands are all-zero."""
    if not isinstance(command, str) or command not in COMMANDS:
        raise ValueError('command must be config, keepalive or abort')
    payload = bytearray(16)
    payload[0], payload[1] = VERSION, COMMANDS[command]
    if command == 'config':
        v = validate_values(values)
        struct.pack_into('<HBBHHHHH', payload, 2, v['config_id'], DIRECTIONS[v['direction']], 0,
                         v['duty_count'], v['pulse_ms'], v['travel_limit_count'],
                         v['speed_limit_count_s'], v['keepalive_timeout_ms'])
    elif values is not None:
        raise ValueError(f'{command} takes no values')
    return command_frame(payload)


def validate_values(values):
    if not isinstance(values, dict) or set(values) != set(VALUE_FIELDS):
        raise ValueError(f'values must be exactly {VALUE_FIELDS}')
    if not isinstance(values['direction'], str) or values['direction'] not in DIRECTIONS:
        raise ValueError(f'direction must be one of {sorted(DIRECTIONS)}')
    numeric = dict(LIMITS, config_id=65535)
    for key, upper in numeric.items():
        value = values[key]
        if type(value) is not int or not 1 <= value <= upper:
            raise ValueError(f'{key} must be an integer in 1..{upper}')
    return values


def load_config(path):
    """A test configuration needs a purpose, power state and a source per value."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('configuration must be a JSON object')
    if data.get('schema') != 'j280-motor-test-config-v1':
        raise ValueError('schema must be j280-motor-test-config-v1')
    if data.get('power') not in ('isolated', 'powered'):
        raise ValueError('power must be "isolated" or "powered"')
    if not isinstance(data.get('purpose'), str) or not data['purpose'].strip():
        raise ValueError('purpose is required')
    validate_values(data.get('values'))
    sources = data.get('sources')
    if not isinstance(sources, dict):
        raise ValueError('sources must be a JSON object with a source for every value')
    missing = [k for k in VALUE_FIELDS if not isinstance(sources.get(k), str) or not sources[k].strip()]
    if missing or set(sources) != set(VALUE_FIELDS):
        raise ValueError(f'every value needs a non-empty source: {missing or sorted(set(sources)-set(VALUE_FIELDS))}')
    return data


def _decode(frame):
    values = dict(zip(FIELDS, PAYLOAD.unpack_from(frame, 2)))
    v = values
    if v['version'] != VERSION:
        raise ValueError('version_errors')
    if (v['flags'] & 0x80 or v['motor_mode'] & 0xf8 or v['estimator_byte'] & 0xf0
            or v['input_levels'] & 0xf8):
        raise ValueError('reserved_bits_errors')
    flags, mode, est, rx = v['flags'], v['motor_mode'], v['estimator_byte'], v['rx_status']
    v.update(state=STATES.get(v['state_code']), stop_reason=STOP_REASONS.get(v['stop_reason_code']),
             adc_valid=bool(flags & 1), adc_new=bool(flags & 2), output_enabled=bool(flags & 4),
             buffer_overflow=bool(flags & 8), config_loaded=bool(flags & 16),
             arm_on=bool(flags & 32), estimator_ok=bool(flags & 64),
             in1=bool(mode & 1), in2=bool(mode & 2), pwm_idle_high=bool(mode & 4),
             estimator_quality=est & 7, estimator_fresh=bool(est & 8),
             rx_code=RX_CODES.get(rx & 15), rx_frame_count=rx >> 4,
             sw1_level=v['input_levels'] & 1, sw2_level=(v['input_levels'] >> 1) & 1,
             dip1_level=(v['input_levels'] >> 2) & 1)
    if (v['state'] is None or v['stop_reason'] is None or v['rx_code'] is None
            or v['adc_raw10'] > 1023 or v['pwm_period_count'] != PWM_PERIOD
            or v['estimator_quality'] not in (0, 1, 3, 5, 7)
            or v['estimator_ok'] != (v['estimator_quality'] == 7 and v['estimator_fresh'])
            or bool(v['fault_bits'] & 0xfff8)
            or bool(v['fault_bits'] & 1) == v['adc_valid']
            or bool(v['fault_bits'] & 4) != v['buffer_overflow']):
        raise ValueError('domain_errors')
    if v['output_enabled']:
        drive_ok = (v['in1'] != v['in2'] and not v['pwm_idle_high']
                    and 1 <= v['pwm_count'] <= LIMITS['duty_count'])
    else:
        drive_ok = (not v['in1'] and not v['in2'] and v['pwm_idle_high'] and v['pwm_count'] == 0)
    if not drive_ok:
        raise ValueError('domain_errors')
    if v['config_loaded'] != bool(v['config_id']) or (not v['config_loaded'] and v['config_crc']):
        raise ValueError('domain_errors')
    return v


class FrameStream(V1Stream):
    """40-byte v3 frames; v1 sequence/tick continuity plus a 2000 us tick check."""

    def __init__(self):
        super().__init__()
        self.stats['continuity_errors'] = 0

    def feed(self, chunk):
        self.stats['bytes_received'] += len(chunk)
        self._buffer.extend(chunk)
        rows = []
        cursor = 0
        while cursor < len(self._buffer):
            start = self._buffer.find(b'\xa5\x5a', cursor)
            if start < 0:
                keep = int(self._buffer[-1] == 0xa5)
                self.stats['discarded_bytes'] += len(self._buffer) - cursor - keep
                cursor = len(self._buffer) - keep
                break
            self.stats['discarded_bytes'] += start - cursor
            cursor = start
            if len(self._buffer) - start < FRAME_SIZE:
                break
            frame = bytes(self._buffer[start:start + FRAME_SIZE])
            try:
                if crc16_ccitt(frame[2:38]) != struct.unpack_from('<H', frame, 38)[0]:
                    raise ValueError('crc_errors')
                row = _decode(frame)
            except ValueError as error:
                self.stats['frames_invalid'] += 1
                self.stats[str(error)] += 1
                self.stats['discarded_bytes'] += 1
                cursor += 1
                continue
            previous = self._anchor
            self._continuity(row)
            row['time_continuity_error'] = False
            if previous is not None and not (row['reset'] or row['out_of_order'] or row['duplicate']):
                expected = (row['seq_gap'] + 1) * SAMPLE_PERIOD_US
                if row['tick_us_unwrapped'] - previous[2] != expected:
                    row['time_continuity_error'] = True
                    self.stats['continuity_errors'] += 1
            rows.append(row)
            self.stats['frames_valid'] += 1
            cursor += FRAME_SIZE
        del self._buffer[:cursor]
        self.stats['incomplete_tail_bytes'] = len(self._buffer)
        return rows


def _stopped(row):
    return (row['state'] in ('SAFE', 'FAULT') and not row['output_enabled']
            and not row['config_loaded'])


class _RunSession:
    """Gate live decisions without changing offline FrameStream diagnostics."""

    def __init__(self, clock):
        self.clock = clock
        self.accepted_frames = 0
        self.latest = None
        self.error = None
        self.stopped = None
        self.stopped_at = None

    def accept(self, row):
        if row['reset'] or row['reset_or_reorder']:
            if self.error is None:
                reason = 'reset' if row['reset'] else 'reset_or_reorder'
                self.error = (f"session continuity lost: {reason} at seq={row['seq']} "
                              f"tick_us={row['tick_us']}")
            self.stopped = None
        if self.error or any(row[k] for k in ('duplicate', 'out_of_order', 'time_continuity_error')):
            return False
        if self.latest is not None:
            seq_delta = (row['seq'] - self.latest['seq']) & 0xffffffff
            tick_delta = (row['tick_us'] - self.latest['tick_us']) & 0xffffffff
            # FrameStream's diagnostic anchor can advance on a timing error.
            # Only accepted rows may establish the live session's timebase.
            if not 0 < seq_delta < (1 << 31) or tick_delta != seq_delta * SAMPLE_PERIOD_US:
                return False
        self.accepted_frames += 1
        self.latest = row
        self.stopped = row if _stopped(row) else None
        self.stopped_at = self.clock() if self.stopped is not None else None
        return True

    def check(self):
        if self.error:
            raise RuntimeError(self.error)

    def fresh_stop(self):
        if (not self.error and self.stopped is not None
                and self.clock() - self.stopped_at <= TELEMETRY_TIMEOUT_S):
            return self.stopped
        return None


class Recorder:
    """Writes every decoded frame to CSV and raw bytes, keeping state-change events."""

    def __init__(self, writer, raw):
        self.writer, self.raw = writer, raw
        self.stream = FrameStream()
        self.latest = None
        self.events = []
        self.rows = 0
        self.enc_range = None
        self.output_frames = 0
        self._session = None

    def feed(self, chunk, host_time=''):
        if self.raw:
            self.raw.write(chunk)
        rows = self.stream.feed(chunk)
        accepted = []
        for row in rows:
            row['host_time_iso'] = host_time
            row['notes'] = ';'.join(k for k in ('duplicate', 'out_of_order', 'reset_or_reorder',
                                                'time_continuity_error') if row[k])
            self.writer.writerow(row)
            self.rows += 1
            if row['output_enabled']:
                self.output_frames += 1
            e = row['enc_count']
            self.enc_range = (e, e) if self.enc_range is None else (min(self.enc_range[0], e), max(self.enc_range[1], e))
            if self._session is not None and not self._session.accept(row):
                continue
            accepted.append(row)
            key = (row['state'], row['stop_reason'], row['output_enabled'], row['config_loaded'])
            if self.latest is None or key != (self.latest['state'], self.latest['stop_reason'],
                                              self.latest['output_enabled'], self.latest['config_loaded']):
                self.events.append({k: row[k] for k in (
                    'seq', 'tick_us_unwrapped', 'state', 'stop_reason', 'output_enabled',
                    'config_loaded', 'enc_count', 'adc_raw10', 'pwm_count', 'in1', 'in2')})
            self.latest = row
        if self._session is not None:
            self._session.check()
        return accepted


class _DiagnosticExitStack(ExitStack):
    """Close every resource without replacing an existing operation error."""

    def __init__(self):
        super().__init__()
        self.close_errors = []

    def enter_context(self, resource):
        value = resource.__enter__()

        def close_resource(exc_type, error, traceback):
            try:
                return resource.__exit__(exc_type, error, traceback)
            except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as close_error:
                self.close_errors.append(f'{type(resource).__name__} close: '
                                         f'{type(close_error).__name__}: {close_error}')
                if error is not None:
                    return False
                raise

        self.push(close_resource)
        return value


def _open_outputs(stack, output, raw_output):
    paths = [output] + ([raw_output] if raw_output else [])
    if len({p.resolve() for p in paths}) != len(paths):
        raise ValueError('outputs must be different paths')
    if any(p.exists() or p.is_symlink() for p in paths):
        raise FileExistsError('output already exists')
    target = stack.enter_context(output.open('x', newline='', encoding='utf-8'))
    writer = csv.DictWriter(target, fieldnames=CSV_FIELDS)
    writer.writeheader()
    raw = stack.enter_context(raw_output.open('xb')) if raw_output else None
    return writer, raw


def _now():
    return datetime.now(timezone.utc).isoformat(timespec='microseconds')


def _read_serial_chunk(port):
    # CH340 on this host can wait for all requested bytes despite a short
    # timeout. At 20 kB/s, read(4096) then stalls the keepalive for ~205 ms.
    # Drain already queued bytes, or wait for just one byte when the queue is
    # empty, so serial traffic does not set the host's scheduling period.
    queued = getattr(port, 'in_waiting', 0)
    return port.read(max(1, min(4096, queued)))


def _read_for(port, recorder, seconds, until=None, clock=None, on_rows=None):
    clock = time.monotonic if clock is None else clock
    deadline = clock() + seconds
    while (remaining := deadline - clock()) > 0:
        port.timeout = min(0.01, remaining)
        chunk = _read_serial_chunk(port)
        if chunk:
            rows = recorder.feed(chunk, _now())
            if on_rows:
                on_rows(rows)
            if until and any(until(row) for row in rows):
                return True
    return False


def _wait_stopped(port, recorder, seconds, clock=None, prior_stop=None):
    """Require a new usable shutdown frame, or the still-current prior evidence."""
    confirmed = _read_for(port, recorder, seconds, clock=clock,
                          until=lambda row: row is recorder.latest and _stopped(row))
    session = recorder._session
    session.check()
    if not confirmed and not (prior_stop is not None and session.stopped is prior_stop):
        last = recorder.latest
        detail = ('no usable telemetry' if last is None else
                  f"state={last['state']}, output_enabled={last['output_enabled']}, "
                  f"config_loaded={last['config_loaded']}")
        raise RuntimeError(f'ABORT stop not confirmed within {seconds:g} s ({detail})')
    return recorder.latest


def run_test(port, recorder, config, wait_start_s, observe_s, keepalive_period_s,
             stop_keepalive_after_run_ms=None, clock=None):
    """Board protocol; returns a result dictionary. The caller sends a final ABORT."""
    clock = time.monotonic if clock is None else clock
    values = config['values']
    command = build_command('config', values)
    recorder._session = session = _RunSession(clock)
    expected_crc = struct.unpack_from('<H', command, 18)[0]
    result = {'config_crc': expected_crc, 'phase': 'sync'}
    if not _read_for(port, recorder, 2.0, until=lambda r: session.accepted_frames >= 5, clock=clock):
        raise RuntimeError('no valid telemetry within 2 s')
    if recorder.latest['state'] != 'SAFE':
        raise RuntimeError(f"board not SAFE: {recorder.latest['state']} "
                           f"({recorder.latest['stop_reason']}); press SW5 to reset if FAULT")
    if recorder.latest['config_loaded']:
        port.write(build_command('abort'))
        _wait_stopped(port, recorder, 1.0, clock=clock)
        if recorder.latest['state'] != 'SAFE':
            raise RuntimeError(f"board not SAFE after stale configuration ABORT: {recorder.latest['state']}")
    result['phase'] = 'config'
    # A CONFIG may be delivered even when its write subsequently raises.
    # Earlier SAFE telemetry can no longer confirm this session's shutdown.
    session.stopped = None
    port.write(command)

    def echo_matches(row):
        return (row['config_loaded'] and row['config_id'] == values['config_id']
                and row['config_crc'] == expected_crc)

    echo_rows = []
    if not _read_for(port, recorder, 1.0, until=echo_matches, clock=clock, on_rows=echo_rows.extend):
        raise RuntimeError(f"configuration not echoed (last rx={recorder.latest['rx_code']})")
    # One read can contain CONFIG acceptance, RUN and even the terminal frame.
    # Retain every row from acceptance onwards for the state loop below.
    pending_rows = echo_rows[next(i for i, row in enumerate(echo_rows) if echo_matches(row)):]
    result['phase'] = 'wait_start'
    loaded_at = clock()
    last_keepalive = loaded_at
    last_telemetry = loaded_at
    run_seen_at = None
    ended = None
    while ended is None:
        now = clock()
        if now - last_telemetry >= TELEMETRY_TIMEOUT_S:
            raise RuntimeError(f"telemetry lost for {TELEMETRY_TIMEOUT_S:g} s "
                               f"during {result['phase']} (last state={recorder.latest['state']})")
        if run_seen_at is not None and now - run_seen_at >= values['pulse_ms'] / 1000 + RUN_GRACE_S:
            raise RuntimeError('run duration exceeded configured pulse plus grace deadline')
        if pending_rows:
            rows, pending_rows = pending_rows, []
        else:
            suppress = (stop_keepalive_after_run_ms is not None and run_seen_at is not None
                        and now - run_seen_at >= stop_keepalive_after_run_ms / 1000)
            if now - last_keepalive >= keepalive_period_s and not suppress:
                port.write(build_command('keepalive'))
                last_keepalive = now
            port.timeout = 0.005
            chunk = _read_serial_chunk(port)
            rows = recorder.feed(chunk, _now()) if chunk else []
        for row in rows:
            last_telemetry = clock()
            if row['state'] == 'RUN' and run_seen_at is None:
                run_seen_at = clock()
                result['phase'] = 'run'
            ended = row if _stopped(row) else None
        if ended is None and run_seen_at is None and clock() - loaded_at > wait_start_s:
            result['phase'] = 'start_timeout'
            port.write(build_command('abort'))
            ended = _wait_stopped(port, recorder, 0.5, clock=clock)
    result.update(end_state=ended['state'], end_reason=ended['stop_reason'],
                  end_tick_us=ended['tick_us_unwrapped'], started=run_seen_at is not None)
    _read_for(port, recorder, observe_s, clock=clock)
    return result


def summarize(recorder, extra=None):
    runs = [e for e in recorder.events if e['state'] == 'RUN']
    output_on = [e for e in recorder.events if e['output_enabled']]
    output_off = [e for e in recorder.events if not e['output_enabled']
                  and output_on and e['tick_us_unwrapped'] > output_on[0]['tick_us_unwrapped']]
    summary = {'stats': recorder.stream.stats, 'frames': recorder.rows,
               'events': recorder.events, 'output_frames': recorder.output_frames,
               'enc_range': recorder.enc_range}
    if output_on:
        summary['output_on_tick_us'] = output_on[0]['tick_us_unwrapped']
        summary['output_on_enc'] = output_on[0]['enc_count']
    if output_off:
        summary['output_off_tick_us'] = output_off[0]['tick_us_unwrapped']
        summary['output_frames_duration_us'] = output_off[0]['tick_us_unwrapped'] - output_on[0]['tick_us_unwrapped']
    if runs:
        summary['run_entry_tick_us'] = runs[0]['tick_us_unwrapped']
        summary['run_entry_enc'] = runs[0]['enc_count']
    if recorder.latest:
        summary['final'] = {k: recorder.latest[k] for k in ('state', 'stop_reason', 'enc_count', 'adc_raw10')}
    if recorder._session is not None:
        summary['stop_confirmed'] = recorder._session.fresh_stop() is not None
        if recorder._session.error:
            summary['session_error'] = recorder._session.error
    summary.update(extra or {})
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    commands = parser.add_subparsers(dest='command', required=True)
    decode = commands.add_parser('decode')
    decode.add_argument('--input', type=Path, required=True)
    capture = commands.add_parser('capture', help='read only; never writes to the port')
    capture.add_argument('--seconds', type=float, required=True)
    run = commands.add_parser('run', help='load one configuration and wait for SW1 on the board')
    run.add_argument('--config', type=Path, required=True)
    run.add_argument('--wait-start-s', type=float, default=90.0)
    run.add_argument('--observe-s', type=float, default=3.0)
    run.add_argument('--keepalive-period-ms', type=float, default=50.0)
    run.add_argument('--stop-keepalive-after-run-ms', type=float)
    run.add_argument('--confirm-powered', action='store_true',
                     help='required for a configuration marked power=powered')
    for command in (capture, run):
        command.add_argument('--port', required=True)
        command.add_argument('--baud', type=int, default=230400)
    for command in (decode, capture, run):
        command.add_argument('--output', type=Path, required=True)
        command.add_argument('--raw-output', type=Path)
    args = parser.parse_args(argv)
    summary = {}
    recorder = None
    stack = None
    try:
        config = None
        if args.command == 'run':
            config = load_config(args.config)
            if config['power'] == 'powered' and not args.confirm_powered:
                raise ValueError('configuration is marked powered; add --confirm-powered')
            if not 0 < args.keepalive_period_ms < config['values']['keepalive_timeout_ms'] / 2:
                raise ValueError('keepalive period must be below half the board timeout')
            if not (math.isfinite(args.wait_start_s) and args.wait_start_s > 0
                    and math.isfinite(args.observe_s) and args.observe_s >= 0):
                raise ValueError('wait and observe durations must be finite')
            if (args.stop_keepalive_after_run_ms is not None
                    and not (math.isfinite(args.stop_keepalive_after_run_ms)
                             and args.stop_keepalive_after_run_ms >= 0)):
                raise ValueError('stop keepalive delay must be finite and non-negative')
        if args.command == 'capture' and not (math.isfinite(args.seconds) and args.seconds > 0):
            raise ValueError('seconds must be finite and positive')
        with _DiagnosticExitStack() as stack:
            writer, raw = _open_outputs(stack, args.output, args.raw_output)
            recorder = Recorder(writer, raw)
            if args.command == 'decode':
                if any(args.input.resolve() == p.resolve() for p in [args.output] + ([args.raw_output] if args.raw_output else [])):
                    raise ValueError('output aliases input')
                source = stack.enter_context(args.input.open('rb'))
                while chunk := source.read(65536):
                    recorder.feed(chunk)
                summary = summarize(recorder)
            else:
                import serial
                port = stack.enter_context(serial.Serial(port=args.port, baudrate=args.baud,
                                                         timeout=0.01, write_timeout=0.2))
                if args.command == 'capture':
                    _read_for(port, recorder, args.seconds)
                    summary = summarize(recorder)
                else:
                    result = {}
                    try:
                        result = run_test(port, recorder, config, args.wait_start_s, args.observe_s,
                                          args.keepalive_period_ms / 1000,
                                          args.stop_keepalive_after_run_ms)
                    finally:
                        primary_error = sys.exc_info()[1]
                        cleanup_errors = []
                        prior_stop = recorder._session.fresh_stop() if recorder._session else None
                        stop_confirmed = False
                        try:
                            port.write(build_command('abort'))
                        except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as error:
                            cleanup_errors.append(f'ABORT: {type(error).__name__}: {error}')
                        try:
                            _wait_stopped(port, recorder, 0.2, prior_stop=prior_stop)
                            stop_confirmed = True
                        except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as error:
                            cleanup_errors.append(f'ABORT stop confirmation: {type(error).__name__}: {error}')
                        summary = summarize(recorder, {'config': config, 'result': result})
                        summary['stop_confirmed'] = stop_confirmed
                        if cleanup_errors:
                            summary['cleanup_errors'] = cleanup_errors
                            if primary_error is None:
                                raise RuntimeError('exit cleanup failed: ' + '; '.join(cleanup_errors))
        print(json.dumps(summary, allow_nan=False))
    except (OSError, ValueError, RuntimeError, ImportError, KeyboardInterrupt) as error:
        if recorder is not None and not summary:
            summary = summarize(recorder)
        if stack is not None and stack.close_errors:
            summary.setdefault('cleanup_errors', []).extend(stack.close_errors)
        interrupted = isinstance(error, KeyboardInterrupt)
        message = 'interrupted by user' if interrupted else str(error)
        print(json.dumps(dict(error=message, error_type=type(error).__name__, partial=summary or None),
                         ensure_ascii=True, default=str), file=sys.stderr)
        return 130 if interrupted else 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
