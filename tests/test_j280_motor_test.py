"""Independent v3 PC protocol tests: stdlib only, no hardware or RTL tools.

Wire fixtures use explicit specification offsets and a bit-at-a-time reference
CRC, never the production packer/CRC to manufacture their expected answers.
The serial double advances a virtual clock; a guard turns unbounded loops into
an immediate test failure instead of hanging the suite.
"""
import contextlib
import csv
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_motor_test as motor

HARD_LIMITS = dict(config_id=65535, duty_count=1000, pulse_ms=2000,
                   travel_limit_count=200, speed_limit_count_s=10000,
                   keepalive_timeout_ms=500)


def reference_crc(data):
    crc = 0xffff
    for octet in data:
        for bit in range(7, -1, -1):
            feedback = ((crc >> 15) ^ (octet >> bit)) & 1
            crc = (crc << 1) & 0xffff
            if feedback:
                crc ^= 0x1021
    return crc


def sample(**changes):
    fields = dict(version=3, flags=0x43, seq=0, tick_us=2000,
                  adc_raw10=513, adc_age_us=12, enc_count=-123456,
                  pwm_count=0, pwm_period_count=2500, motor_mode=4,
                  state_code=0, fault_bits=0, pwm_age_us=65535,
                  stop_reason_code=0, estimator_byte=15, rx_status=0,
                  input_levels=7, config_id=0, config_crc=0)
    fields.update(changes)
    wire = bytearray(40)
    wire[:2] = b'\xa5\x5a'
    # Independent byte offsets from the v3 specification.
    for offset, fmt, name in (
            (2, 'B', 'version'), (3, 'B', 'flags'), (4, 'I', 'seq'),
            (8, 'I', 'tick_us'), (12, 'H', 'adc_raw10'), (14, 'H', 'adc_age_us'),
            (16, 'i', 'enc_count'), (20, 'H', 'pwm_count'),
            (22, 'H', 'pwm_period_count'), (24, 'B', 'motor_mode'),
            (25, 'B', 'state_code'), (26, 'H', 'fault_bits'), (28, 'H', 'pwm_age_us'),
            (30, 'B', 'stop_reason_code'), (31, 'B', 'estimator_byte'),
            (32, 'B', 'rx_status'), (33, 'B', 'input_levels'),
            (34, 'H', 'config_id'), (36, 'H', 'config_crc')):
        struct.pack_into('<' + fmt, wire, offset, fields[name])
    struct.pack_into('<H', wire, 38, reference_crc(wire[2:38]))
    return bytes(wire)


def changed_sample(wire, **changes):
    """Edit selected spec offsets, preserving an independent, valid CRC."""
    offsets = dict(seq=(4, 'I'), tick_us=(8, 'I'), flags=(3, 'B'),
                   state_code=(25, 'B'), stop_reason_code=(30, 'B'),
                   config_id=(34, 'H'), config_crc=(36, 'H'),
                   pwm_count=(20, 'H'), motor_mode=(24, 'B'))
    wire = bytearray(wire)
    for name, value in changes.items():
        offset, fmt = offsets[name]
        struct.pack_into('<' + fmt, wire, offset, value)
    wire[38:] = reference_crc(wire[2:38]).to_bytes(2, 'little')
    return bytes(wire)


def abort_reply(port, reply):
    """Replace only the board's ABORT response (None models a silent link)."""
    respond = port.on_write

    def on_write(port, command):
        if command[3] != 3:
            respond(port, command)
        elif reply is not None:
            port.chunks.append(reply)
    port.on_write = on_write
    return port


def configuration():
    values = dict(config_id=0x1234, direction='in1_high', duty_count=125,
                  pulse_ms=100, travel_limit_count=60, speed_limit_count_s=3000,
                  keepalive_timeout_ms=500)
    return dict(schema='j280-motor-test-config-v1', power='isolated',
                purpose='simulation only', values=values,
                sources={k: 'independent unittest fixture' for k in values})


class Clock:
    def __init__(self, limit=5.0):
        self.now = 0.0
        self.limit = limit

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds
        if self.now > self.limit:
            raise AssertionError('runner exceeded virtual deadline (unbounded loop)')


class SerialDouble:
    """Blocking read model, with command-triggered board responses."""
    def __init__(self, clock, chunks=(), on_write=None):
        self.clock, self.chunks, self.on_write = clock, list(chunks), on_write
        self.timeout = 0.01
        self.writes = []
        self.closed = False
        self.read_count = 0

    def read(self, count):
        self.read_count += 1
        self.clock.advance(self.timeout)
        chunk = self.chunks.pop(0) if self.chunks else b''
        if isinstance(chunk, BaseException):
            raise chunk
        return chunk

    def write(self, data):
        data = bytes(data)
        if len(data) != 20 or data[:3] != b'\xa5\x5a\x03':
            raise AssertionError('invalid host command layout')
        if reference_crc(data[2:18]) != int.from_bytes(data[18:], 'little'):
            raise AssertionError('invalid host CRC')
        if data[3] not in (1, 2, 3):
            raise AssertionError('PC attempted a command beyond CONFIG/KEEPALIVE/ABORT')
        self.writes.append((self.clock(), data))
        if self.on_write:
            self.on_write(self, data)
        return len(data)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True


class ContinuousSerialDouble(SerialDouble):
    """500 Hz wire with the observed driver waiting for requested byte count.

    A 4096-byte request costs about 205 ms, starving a 50 ms keepalive. Queued
    data drains immediately; an empty queue receives one 40-byte frame / 2 ms.
    """
    def __init__(self, clock):
        super().__init__(clock)
        self.buffer = bytearray()
        self.seq = 0
        self.loaded_at = self.last_keepalive = None
        self.config_id = self.config_crc = 0
        self.terminal = None

    @property
    def in_waiting(self):
        return len(self.buffer)

    def write(self, data):
        result = super().write(data)
        if data[3] == 1:
            self.loaded_at = self.last_keepalive = self.clock()
            self.config_id = int.from_bytes(data[4:6], 'little')
            self.config_crc = int.from_bytes(data[18:20], 'little')
        elif data[3] == 2:
            self.last_keepalive = self.clock()
        elif data[3] == 3:
            self.terminal = (0, 4)
        return result

    def read(self, count):
        self.read_count += 1
        while len(self.buffer) < count:
            next_time = (self.seq + 1) * 0.002
            self.clock.advance(max(0, next_time - self.clock()))
            state, reason, loaded, output = 0, 0, self.loaded_at is not None, False
            if loaded and self.terminal is None:
                elapsed = self.clock() - self.loaded_at
                if self.clock() - self.last_keepalive >= 0.200:
                    self.terminal = (255, 20)
                elif elapsed >= 0.350:
                    self.terminal = (0, 1)
                elif elapsed >= 0.250:
                    state, output = 2, True
                else:
                    state = 1
            if self.terminal is not None:
                state, reason = self.terminal
                loaded, output = False, False
            self.buffer.extend(sample(seq=self.seq, tick_us=(self.seq + 1) * 2000,
                flags=0x43 | (16 if loaded else 0) | (4 if output else 0),
                state_code=state, stop_reason_code=reason,
                config_id=self.config_id if loaded else 0,
                config_crc=self.config_crc if loaded else 0,
                pwm_count=125 if output else 0, motor_mode=1 if output else 4))
            self.seq += 1
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data


def recorder():
    target, raw = io.StringIO(), io.BytesIO()
    writer = csv.DictWriter(target, motor.CSV_FIELDS)
    writer.writeheader()
    return motor.Recorder(writer, raw), target, raw


def board(clock, after_config=(), stale=False, echo_id=None, echo_crc=None):
    """Starts SAFE; the test controls all subsequent physical board states."""
    seq = 0

    def packet(**kw):
        nonlocal seq
        result = sample(seq=seq, tick_us=(seq + 1) * 2000, **kw)
        seq += 1
        return result

    initial = b''.join(packet(flags=0x53 if stale else 0x43,
                              config_id=99 if stale else 0,
                              config_crc=123 if stale else 0) for _ in range(5))

    def respond(port, command):
        if command[3] == 3:
            port.chunks.append(packet(stop_reason_code=4, rx_status=0x33))
        if command[3] == 1:
            config_id = int.from_bytes(command[4:6], 'little')
            crc = int.from_bytes(command[18:20], 'little')
            port.chunks.append(packet(flags=0x53, config_id=config_id if echo_id is None else echo_id,
                                      config_crc=crc if echo_crc is None else echo_crc,
                                      rx_status=0x11))
            for entry in after_config:
                if isinstance(entry, dict):
                    fields = dict(flags=0x73, state_code=1, config_id=config_id, config_crc=crc)
                    fields.update(entry)
                    port.chunks.append(packet(**fields))
                else:
                    port.chunks.append(entry)
    return SerialDouble(clock, [initial], respond)


class WireTests(unittest.TestCase):
    def test_crc_known_answer_and_command_layout(self):
        self.assertEqual(motor.crc16_ccitt(b'123456789'), 0x29b1)
        values = configuration()['values']
        payload = bytes.fromhex('0301341201007d0064003c00b80bf401')
        expected = b'\xa5\x5a' + payload + reference_crc(payload).to_bytes(2, 'little')
        self.assertEqual(motor.build_command('config', values), expected)
        self.assertEqual(len(expected), 20)
        for name, code in (('keepalive', 2), ('abort', 3)):
            payload = bytes([3, code]) + bytes(14)
            self.assertEqual(motor.build_command(name), b'\xa5\x5a' + payload
                             + reference_crc(payload).to_bytes(2, 'little'))
        values['direction'] = 'in2_high'
        self.assertEqual(motor.build_command('config', values)[6:8], b'\x02\x00')

    def test_command_rejects_unknown_start_and_invalid_values(self):
        for command in ('start', 'CONFIG', '', None):
            with self.subTest(command=command), self.assertRaises(ValueError):
                motor.build_command(command)
        for values in (None, {}, dict(configuration()['values'], duty_count=True),
                       dict(configuration()['values'], duty_count=1001)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                motor.build_command('config', values)
        for name in ('keepalive', 'abort'):
            with self.assertRaises(ValueError):
                motor.build_command(name, {})
        with self.assertRaises(ValueError):
            motor.command_frame(bytes(15))

    def test_independent_40_byte_sample_and_derived_layout(self):
        wire = sample(seq=0x12345678, tick_us=0x9abcdef0, flags=0x77,
                      pwm_count=1000, motor_mode=2, state_code=2, rx_status=0xa2,
                      config_id=0x1234, config_crc=0xbeef, input_levels=5)
        self.assertEqual(len(wire), 40)
        self.assertEqual(wire[4:12], bytes.fromhex('78563412f0debc9a'))
        row, = motor.FrameStream().feed(wire)
        for key, value in dict(version=3, seq=0x12345678, tick_us=0x9abcdef0,
                               adc_raw10=513, adc_age_us=12, enc_count=-123456,
                               pwm_count=1000, pwm_period_count=2500, pwm_age_us=65535,
                               state='RUN', config_id=0x1234, config_crc=0xbeef,
                               rx_code='keepalive', rx_frame_count=10, estimator_quality=7,
                               sw1_level=1, sw2_level=0, dip1_level=1).items():
            self.assertEqual(row[key], value, key)
        for key in ('adc_valid', 'adc_new', 'output_enabled', 'config_loaded',
                    'arm_on', 'estimator_ok', 'estimator_fresh', 'in2'):
            self.assertTrue(row[key], key)
        self.assertFalse(row['in1'])
        self.assertFalse(row['pwm_idle_high'])
        self.assertEqual(motor.PWM_PERIOD, 2500)
        self.assertEqual(motor.SAMPLE_PERIOD_US, 2000)

    def test_every_fragment_boundary_noise_and_partial_sync(self):
        wire = sample()
        for split in range(41):
            with self.subTest(split=split):
                stream = motor.FrameStream()
                rows = stream.feed(wire[:split]) + stream.feed(wire[split:])
                self.assertEqual(len(rows), 1)
                self.assertEqual(stream.buffered_bytes, 0)
        stream = motor.FrameStream()
        self.assertEqual(stream.feed(b'noise\xa5'), [])
        rows = []
        for octet in wire[1:]:
            rows += stream.feed(bytes([octet]))
        self.assertEqual(len(rows), 1)
        self.assertEqual(stream.stats['discarded_bytes'], 5)
        self.assertEqual(stream.feed(b''), [])
        self.assertEqual(stream.feed(wire[:39]), [])
        self.assertEqual(stream.stats['incomplete_tail_bytes'], 39)

    def test_crc_version_reserved_bits_and_resynchronization(self):
        bad_crc = bytearray(sample())
        bad_crc[16] ^= 1
        invalid = [(bytes(bad_crc), 'crc_errors'),
                   (sample(version=2), 'version_errors')]
        invalid += [(sample(**fields), 'reserved_bits_errors') for fields in (
            dict(flags=0xc3), dict(motor_mode=12), dict(estimator_byte=31), dict(input_levels=15))]
        for wire, counter in invalid:
            with self.subTest(counter=counter, wire=wire):
                stream = motor.FrameStream()
                rows = stream.feed(wire + sample(seq=1, tick_us=4000))
                self.assertEqual([r['seq'] for r in rows], [1])
                self.assertEqual(stream.stats[counter], 1)
                self.assertEqual(stream.stats['frames_invalid'], 1)
                self.assertEqual(stream.buffered_bytes, 0)

    def test_domain_checks(self):
        cases = [dict(state_code=3), dict(stop_reason_code=6), dict(rx_status=8),
                 dict(adc_raw10=1024), dict(pwm_period_count=2499),
                 dict(estimator_byte=10), dict(estimator_byte=7),
                 dict(flags=3), dict(fault_bits=8), dict(fault_bits=1),
                 dict(flags=0x4b), dict(fault_bits=4),
                 dict(pwm_count=1), dict(motor_mode=0), dict(motor_mode=5),
                 dict(config_id=1), dict(config_crc=1), dict(flags=0x53),
                 dict(flags=0x77, state_code=2, config_id=1, motor_mode=3, pwm_count=1),
                 dict(flags=0x77, state_code=2, config_id=1, motor_mode=5, pwm_count=1),
                 dict(flags=0x77, state_code=2, config_id=1, motor_mode=1, pwm_count=0),
                 dict(flags=0x77, state_code=2, config_id=1, motor_mode=1, pwm_count=1001)]
        for fields in cases:
            with self.subTest(fields=fields):
                stream = motor.FrameStream()
                self.assertEqual(stream.feed(sample(**fields)), [])
                self.assertEqual(stream.stats['domain_errors'], 1)

    def test_valid_faults_extrema_and_estimator_states(self):
        for quality in (0, 1, 3, 5, 7):
            for fresh in (0, 8):
                flags = 3 | (64 if quality == 7 and fresh else 0)
                self.assertEqual(len(motor.FrameStream().feed(sample(
                    estimator_byte=quality | fresh, flags=flags))), 1)
        for enc in (-(1 << 31), (1 << 31) - 1):
            row, = motor.FrameStream().feed(sample(enc_count=enc, adc_raw10=1023,
                                                  flags=8, fault_bits=7,
                                                  estimator_byte=0, state_code=255,
                                                  stop_reason_code=16))
            self.assertEqual(row['enc_count'], enc)
            self.assertEqual(row['stop_reason'], 'sensor_invalid')

    def test_sequence_gap_duplicate_reorder_and_time_error(self):
        stream = motor.FrameStream()
        rows = stream.feed(b''.join(sample(seq=s, tick_us=t) for s, t in (
            (10, 20000), (12, 24000), (12, 24000), (11, 22000), (13, 26000), (14, 28001))))
        self.assertEqual(rows[1]['seq_gap'], 1)
        self.assertTrue(rows[2]['duplicate'])
        self.assertTrue(rows[3]['out_of_order'])
        self.assertTrue(rows[3]['reset_or_reorder'])
        self.assertFalse(rows[4]['time_continuity_error'])
        self.assertTrue(rows[5]['time_continuity_error'])
        for key in ('seq_missed', 'seq_duplicates', 'seq_out_of_order', 'continuity_errors'):
            self.assertEqual(stream.stats[key], 1)

    def test_sequence_and_tick_wrap_and_reset_epochs(self):
        stream = motor.FrameStream()
        rows = stream.feed(sample(seq=0xffffffff, tick_us=0xfffffc18)
                           + sample(seq=0, tick_us=1000) + sample(seq=1, tick_us=3000)
                           + sample(seq=0, tick_us=0) + sample(seq=1, tick_us=2000))
        self.assertEqual(rows[1]['tick_us_unwrapped'], (1 << 32) + 1000)
        self.assertTrue(rows[3]['reset'])
        self.assertEqual(rows[3]['epoch'], 1)
        self.assertEqual(stream.stats['seq_wraps'], 1)
        self.assertEqual(stream.stats['tick_wraps'], 1)
        self.assertEqual(stream.stats['resets'], 1)
        self.assertEqual(stream.stats['continuity_errors'], 0)


class ConfigTests(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            return motor.load_config(path)

    def test_valid_isolated_powered_and_all_numeric_boundaries(self):
        for power in ('isolated', 'powered'):
            config = configuration()
            config['power'] = power
            self.assertEqual(self.load(config), config)
        for key, upper in HARD_LIMITS.items():
            for value in (1, upper):
                config = configuration()
                config['values'][key] = value
                self.assertEqual(self.load(config), config)

    def test_numeric_fields_are_exact_integers_within_limits(self):
        for key, upper in HARD_LIMITS.items():
            for value in (0, -1, upper + 1, True, False, 1.0, '1', None):
                config = configuration()
                config['values'][key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.load(config)

    def test_schema_power_purpose_fields_and_sources_are_strict(self):
        cases = []
        for key, invalids in (
                ('schema', [None, 1, 'v1']), ('power', [None, True, 'POWERED', '']),
                ('purpose', [None, 1, '', ' \t']), ('values', [None, [], [1], 'bad']),
                ('sources', [None, [], [1], 'bad'])):
            for value in invalids:
                config = configuration()
                config[key] = value
                cases.append(config)
        for section in ('values', 'sources'):
            config = configuration()
            del config[section]['config_id']
            cases.append(config)
            config = configuration()
            config[section]['unexpected'] = 'typo'
            cases.append(config)
        for value in (None, True, [], {}, 1, '', 'forward'):
            config = configuration()
            config['values']['direction'] = value
            cases.append(config)
        for field in configuration()['values']:
            for value in (None, True, 1, '', ' \n'):
                config = configuration()
                config['sources'][field] = value
                cases.append(config)
        for data in [None, [], [1], 'bad', 1, *cases]:
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.load(data)


class RunnerTests(unittest.TestCase):
    def test_continuous_serial_reads_do_not_starve_keepalive(self):
        clock = Clock()
        port = ContinuousSerialDouble(clock)
        config = configuration()
        config['values']['keepalive_timeout_ms'] = 200
        rec, _, _ = recorder()
        result = motor.run_test(port, rec, config, wait_start_s=1, observe_s=0,
                                keepalive_period_s=0.05, clock=clock)
        self.assertTrue(result['started'])
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'complete'))
        keepalive_times = [t for t, command in port.writes if command[3] in (1, 2)]
        self.assertGreaterEqual(len(keepalive_times), 5)
        self.assertLess(max(b - a for a, b in zip(keepalive_times, keepalive_times[1:])), 0.070)

    def execute(self, port, clock, **kwargs):
        rec, _, _ = recorder()
        options = dict(wait_start_s=0.2, observe_s=0, keepalive_period_s=0.02, clock=clock)
        options.update(kwargs)
        return motor.run_test(port, rec, configuration(), **options), rec

    def test_handshake_echo_keepalive_then_local_run_and_completion(self):
        clock = Clock()
        port = board(clock, [{}] * 12 + [dict(state_code=2)] * 12
                     + [dict(state_code=0, flags=0x43, config_id=0, config_crc=0,
                             stop_reason_code=1)])
        result, rec = self.execute(port, clock)
        self.assertTrue(result['started'])
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'complete'))
        self.assertEqual(port.writes[0][1][3], 1)
        self.assertTrue(any(c[3] == 2 for _, c in port.writes))
        self.assertEqual([e['state'] for e in rec.events], ['SAFE', 'SAFE', 'ARMED', 'RUN', 'SAFE'])
        times = [t for t, c in port.writes if c[3] == 2]
        self.assertTrue(all(0.02 <= round(b - a, 9) <= 0.025 for a, b in zip(times, times[1:])))

    def test_stale_config_is_aborted_before_new_config(self):
        clock = Clock()
        port = board(clock, [dict(state_code=255, flags=0x43, config_id=0,
                                  config_crc=0, stop_reason_code=20)], stale=True)
        result, _ = self.execute(port, clock)
        self.assertEqual([c[3] for _, c in port.writes[:2]], [3, 1])
        self.assertEqual((result['end_state'], result['end_reason']), ('FAULT', 'keepalive_timeout'))

    def test_fault_after_run_is_terminal_and_observation_uses_clock(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), dict(state_code=255, flags=0x43,
                     config_id=0, config_crc=0, stop_reason_code=18)])
        result, _ = self.execute(port, clock, observe_s=0.03)
        self.assertTrue(result['started'])
        self.assertEqual(result['end_reason'], 'travel_limit')
        self.assertGreaterEqual(clock(), 0.06)
        self.assertLess(clock(), 0.08)

    def test_no_start_has_bounded_abort(self):
        clock = Clock()
        port = board(clock)
        result, _ = self.execute(port, clock, wait_start_s=0.06)
        self.assertEqual(result['phase'], 'start_timeout')
        self.assertFalse(result['started'])
        self.assertEqual(result['end_reason'], 'host_abort')
        self.assertEqual(port.writes[-1][1][3], 3)
        self.assertLess(clock(), 0.2)

    def test_silent_sync_times_out_using_injected_clock(self):
        clock = Clock(limit=2.1)
        port = SerialDouble(clock)
        with self.assertRaisesRegex(RuntimeError, 'no valid telemetry'):
            self.execute(port, clock)
        self.assertLessEqual(clock(), 2.01)
        self.assertEqual(port.writes, [])

    def test_non_safe_initial_state_refuses_config(self):
        for state in (1, 2, 255):
            clock = Clock()
            port = SerialDouble(clock, [b''.join(sample(seq=i, tick_us=2000*(i+1),
                                                     state_code=state) for i in range(5))])
            with self.subTest(state=state), self.assertRaisesRegex(RuntimeError, 'not SAFE'):
                self.execute(port, clock)
            self.assertEqual(port.writes, [])

    def test_config_echo_requires_both_id_and_crc(self):
        for options in (dict(echo_id=99), dict(echo_crc=0)):
            clock = Clock(limit=1.2)
            port = board(clock, **options)
            with self.subTest(options=options), self.assertRaisesRegex(RuntimeError, 'not echoed'):
                self.execute(port, clock)
            self.assertLess(clock(), 1.1)
            self.assertEqual([c[3] for _, c in port.writes], [1])

    def test_run_followed_by_silence_is_bounded(self):
        clock = Clock(limit=2)
        port = board(clock, [dict(state_code=2)])
        with self.assertRaisesRegex(RuntimeError, 'telemetry.*(lost|timeout)'):
            self.execute(port, clock)
        self.assertLess(clock(), 1)

    def test_corrupt_telemetry_does_not_extend_liveness(self):
        clock = Clock(limit=2)
        corrupt = bytearray(sample())
        corrupt[38] ^= 1
        port = board(clock, [dict(state_code=2)] + [bytes(corrupt)] * 300)
        with self.assertRaisesRegex(RuntimeError, 'telemetry.*(lost|timeout)'):
            self.execute(port, clock)
        self.assertLess(clock(), 1)

    def test_continuous_run_cannot_exceed_configured_pulse_and_grace(self):
        clock = Clock(limit=2)
        port = board(clock, [dict(state_code=2)] * 300)
        with self.assertRaisesRegex(RuntimeError, 'run.*(deadline|timeout|duration)'):
            self.execute(port, clock)
        self.assertLess(clock(), 1)

    def test_stop_keepalive_option_suppresses_only_after_run_delay(self):
        clock = Clock()
        port = board(clock, [{}] * 6 + [dict(state_code=2)] * 20
                     + [dict(state_code=255, flags=0x43, config_id=0, config_crc=0,
                             stop_reason_code=20)])
        result, _ = self.execute(port, clock, stop_keepalive_after_run_ms=30)
        times = [t for t, c in port.writes if c[3] == 2]
        self.assertTrue(result['started'])
        self.assertGreaterEqual(len(times), 2)
        # RUN arrives at .055 s; suppression starts at .085 s.
        self.assertLess(times[-1], 0.085)

    def test_coalesced_config_echo_run_and_completion_are_not_lost(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), dict(state_code=0, flags=0x43,
                     config_id=0, config_crc=0, stop_reason_code=1)])
        respond = port.on_write

        def coalesce(port, command):
            respond(port, command)
            if command[3] == 1:
                port.chunks[:] = [b''.join(port.chunks)]
        port.on_write = coalesce
        result, _ = self.execute(port, clock)
        self.assertTrue(result['started'])
        self.assertEqual(result['end_reason'], 'complete')
        self.assertLess(clock(), 0.1)
        self.assertEqual([c[3] for _, c in port.writes], [1])

    def test_run_already_in_config_echo_is_seen(self):
        clock = Clock()
        port = board(clock, [dict(state_code=0, flags=0x43, config_id=0,
                                  config_crc=0, stop_reason_code=1)])
        respond = port.on_write

        def echo_running(port, command):
            respond(port, command)
            if command[3] == 1:
                echo = bytearray(port.chunks[0])
                echo[25] = 2
                echo[38:] = reference_crc(echo[2:38]).to_bytes(2, 'little')
                port.chunks[0] = bytes(echo)
        port.on_write = echo_running
        result, _ = self.execute(port, clock)
        self.assertTrue(result['started'])
        self.assertEqual(result['end_reason'], 'complete')

    def test_stale_config_clear_cannot_authorize_config_in_fault(self):
        clock = Clock()
        port = board(clock, stale=True)
        respond = port.on_write

        def fault_on_abort(port, command):
            if command[3] == 3:
                port.chunks.append(sample(seq=5, tick_us=12000,
                                          state_code=255, stop_reason_code=20))
            else:
                respond(port, command)
        port.on_write = fault_on_abort
        with self.assertRaisesRegex(RuntimeError, '(not SAFE|FAULT)'):
            self.execute(port, clock)
        self.assertEqual([c[3] for _, c in port.writes], [3])

    def test_replayed_run_samples_do_not_extend_liveness(self):
        clock = Clock(limit=2)
        port = board(clock, [dict(state_code=2)])
        respond = port.on_write

        def replay(port, command):
            respond(port, command)
            if command[3] == 1:
                port.chunks.extend([port.chunks[-1]] * 300)
        port.on_write = replay
        with self.assertRaisesRegex(RuntimeError, 'telemetry.*(lost|timeout)'):
            self.execute(port, clock)

    def test_communication_loss_before_run_stops_long_start_wait(self):
        clock = Clock(limit=2)
        port = board(clock)
        with self.assertRaisesRegex(RuntimeError, 'telemetry.*(lost|timeout)'):
            self.execute(port, clock, wait_start_s=90)
        self.assertLess(clock(), 1)

    def test_start_timeout_requires_abort_stop_confirmation(self):
        clock = Clock(limit=1)
        port = abort_reply(board(clock, [dict(state_code=1)]), None)
        with self.assertRaisesRegex(RuntimeError, '(?i)stop.*not confirmed'):
            self.execute(port, clock, wait_start_s=0.03)
        self.assertEqual(port.writes[-1][1][3], 3)
        self.assertLess(clock(), 0.7)

    def test_timeout_abort_cannot_use_old_duplicate_or_bad_time_stop(self):
        replies = dict(old=sample(seq=3, tick_us=8000, stop_reason_code=4),
                       duplicate=sample(seq=6, tick_us=14000, stop_reason_code=4),
                       bad_time=sample(seq=7, tick_us=16001, stop_reason_code=4),
                       reset=sample(seq=0, tick_us=0, stop_reason_code=4))
        for name, reply in replies.items():
            with self.subTest(name=name):
                clock = Clock(limit=1)
                port = abort_reply(board(clock, [dict(state_code=1)]), reply)
                with self.assertRaisesRegex(RuntimeError, '(?i)(stop.*not confirmed|session.*(reset|continuity))'):
                    self.execute(port, clock, wait_start_s=0.03)

    def test_timeout_abort_requires_terminal_state_and_outputs_off(self):
        replies = [sample(seq=7, tick_us=16000, state_code=1),
                   sample(seq=7, tick_us=16000, state_code=0, flags=0x47,
                          pwm_count=125, motor_mode=1, stop_reason_code=4),
                   sample(seq=7, tick_us=16000, state_code=255, flags=0x53,
                          config_id=0x1234, config_crc=123, stop_reason_code=20)]
        for reply in replies:
            with self.subTest(reply=reply):
                clock = Clock(limit=1)
                port = abort_reply(board(clock, [dict(state_code=1)]), reply)
                with self.assertRaisesRegex(RuntimeError, '(?i)stop.*not confirmed'):
                    self.execute(port, clock, wait_start_s=0.03)

    def test_terminal_pipeline_frame_waits_for_actual_outputs_off(self):
        for state, reason in ((0, 1), (255, 18)):
            with self.subTest(state=state):
                clock = Clock()
                terminal = dict(state_code=state, flags=0x47, config_id=0, config_crc=0,
                                pwm_count=125, motor_mode=1, stop_reason_code=reason)
                off = dict(terminal, flags=0x43, pwm_count=0, motor_mode=4)
                port = board(clock, [dict(state_code=2), terminal, off])
                result, rec = self.execute(port, clock)
                self.assertEqual(result['end_tick_us'], 18000)
                self.assertFalse(rec.latest['output_enabled'])
                self.assertEqual(rec.rows, 9)

    def test_duplicate_or_bad_time_terminal_cannot_finish_run(self):
        for name, stop in (
                ('duplicate', sample(seq=6, tick_us=14000, stop_reason_code=1)),
                ('out_of_order', sample(seq=3, tick_us=16000, stop_reason_code=1)),
                ('bad_time', sample(seq=7, tick_us=16001, stop_reason_code=1))):
            with self.subTest(name=name):
                clock = Clock(limit=1)
                port = board(clock, [dict(state_code=2), stop])
                with self.assertRaisesRegex(RuntimeError, 'telemetry.*lost'):
                    self.execute(port, clock)

    def test_bad_time_chain_cannot_establish_a_new_session_timebase(self):
        clock = Clock(limit=1)
        # The decoder advances its diagnostic anchor on a timing error. A later
        # frame matching that wrong anchor still cannot confirm this session.
        port = board(clock, [dict(state_code=2), sample(seq=7, tick_us=16001,
                                                      flags=0x73, config_id=0x1234,
                                                      state_code=2),
                             sample(seq=8, tick_us=18001, stop_reason_code=1)])
        with self.assertRaisesRegex(RuntimeError, '(telemetry.*lost|session.*continuity)'):
            self.execute(port, clock)

    def test_reset_failure_preserves_raw_csv_and_only_trusted_state_events(self):
        clock = Clock()
        old = sample(seq=3, tick_us=8000, stop_reason_code=1)
        port = board(clock, [dict(state_code=2), old])
        rec, csv_target, raw = recorder()
        with self.assertRaisesRegex(RuntimeError, 'session.*continuity'):
            motor.run_test(port, rec, configuration(), 0.2, 0, 0.02, clock=clock)
        self.assertEqual(rec.rows, 8)
        self.assertEqual(len(raw.getvalue()), 8 * 40)
        rows = list(csv.DictReader(io.StringIO(csv_target.getvalue())))
        self.assertEqual(len(rows), 8)
        self.assertIn('reset_or_reorder', rows[-1]['notes'])
        self.assertEqual(rec.latest['state'], 'RUN')
        self.assertEqual(rec.events[-1]['state'], 'RUN')

    def test_unusable_run_does_not_mark_session_started(self):
        for seq, tick in ((5, 12000), (6, 14001)):
            with self.subTest(seq=seq, tick=tick):
                clock = Clock()
                port = board(clock)
                respond = port.on_write

                def bad_run(port, command):
                    if command[3] == 1:
                        respond(port, command)
                        port.chunks.append(changed_sample(port.chunks[0],
                                                         seq=seq, tick_us=tick, state_code=2))
                        port.chunks.append(changed_sample(port.chunks[0], seq=seq + 1,
                                                         tick_us=(seq + 2) * 2000, state_code=1))
                        port.chunks.append(changed_sample(port.chunks[0], seq=seq + 2,
                                                         tick_us=(seq + 3) * 2000, state_code=1))
                    elif command[3] == 3:
                        port.chunks.append(sample(seq=seq + 3, tick_us=(seq + 4) * 2000,
                                                  stop_reason_code=4))
                port.on_write = bad_run
                result, _ = self.execute(port, clock, wait_start_s=0.03)
                self.assertFalse(result['started'])
                self.assertEqual(result['phase'], 'start_timeout')

    def test_reset_or_ambiguous_rollback_terminates_session(self):
        for seq, tick in ((0, 0), (3, 8000)):
            with self.subTest(seq=seq, tick=tick):
                clock = Clock()
                port = board(clock, [dict(state_code=2), sample(seq=seq, tick_us=tick,
                                                              stop_reason_code=1)])
                with self.assertRaisesRegex(RuntimeError, '(?i)session.*(reset|continuity)'):
                    self.execute(port, clock)

    def test_reset_in_observation_also_invalidates_success(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), dict(state_code=0, flags=0x43,
                     config_id=0, config_crc=0, stop_reason_code=1), sample(seq=0, tick_us=0)])
        with self.assertRaisesRegex(RuntimeError, '(?i)session.*(reset|continuity)'):
            self.execute(port, clock, observe_s=0.03)

    def test_sequence_gap_does_not_block_a_new_confirmed_stop(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), sample(seq=9, tick_us=20000,
                                                      stop_reason_code=1)])
        result, rec = self.execute(port, clock)
        self.assertTrue(result['started'])
        self.assertEqual(result['end_tick_us'], 20000)
        self.assertEqual(rec.stream.stats['seq_missed'], 2)

    def test_run_and_stop_accept_normal_sequence_and_tick_wrap(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), dict(state_code=0, flags=0x43,
                     config_id=0, config_crc=0, stop_reason_code=1)])

        def wrap(chunk):
            frames = []
            for start in range(0, len(chunk), 40):
                frame = chunk[start:start + 40]
                seq, tick = struct.unpack_from('<II', frame, 4)
                frames.append(changed_sample(frame, seq=(seq + 0xfffffff9) & 0xffffffff,
                                             tick_us=(tick - 15000) & 0xffffffff))
            return b''.join(frames)
        port.chunks[:] = [wrap(chunk) for chunk in port.chunks]
        respond = port.on_write

        def wrap_responses(port, command):
            first = len(port.chunks)
            respond(port, command)
            port.chunks[first:] = [wrap(chunk) for chunk in port.chunks[first:]]
        port.on_write = wrap_responses
        result, rec = self.execute(port, clock)
        self.assertTrue(result['started'])
        self.assertEqual(result['end_tick_us'], (1 << 32) + 1000)
        self.assertEqual(rec.stream.stats['seq_wraps'], 1)
        self.assertEqual(rec.stream.stats['tick_wraps'], 1)


class CliTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.config_path = self.root / 'config.json'
        self.config_path.write_text(json.dumps(configuration()), encoding='utf-8')

    def cli(self, args, port=None, clock=None):
        stdout, stderr = io.StringIO(), io.StringIO()
        factory = (mock.Mock(return_value=port) if port is not None
                   else mock.Mock(side_effect=AssertionError('unexpected serial access')))
        serial = types.SimpleNamespace(Serial=factory)
        with contextlib.ExitStack() as stack:
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            stack.enter_context(mock.patch.dict(sys.modules, serial=serial))
            if clock:
                stack.enter_context(mock.patch.object(motor.time, 'monotonic', clock))
            try:
                status = motor.main(args)
            except KeyboardInterrupt:
                self.fail('CLI leaked KeyboardInterrupt instead of reporting partial data')
        return status, stdout.getvalue(), stderr.getvalue(), serial.Serial

    def run_args(self, *extra):
        return ['run', '--config', str(self.config_path), '--port', 'FAKE_ONLY',
                '--output', str(self.root / 'out.csv'), '--observe-s', '0',
                '--wait-start-s', '.06', *extra]

    def test_serial_close_error_preserves_primary_read_error(self):
        clock = Clock()
        class CloseFailure(SerialDouble):
            def __exit__(self, *args):
                self.closed = True
                raise OSError('serial close failed')
        port = CloseFailure(clock, [OSError('primary read disconnected')])
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        report = json.loads(stderr)
        self.assertEqual((status, stdout), (2, ''))
        self.assertEqual(report['error'], 'primary read disconnected')
        self.assertTrue(any('serial close failed' in e for e in report['partial']['cleanup_errors']))
        self.assertTrue(port.closed)
        self.assertEqual(port.writes[-1][1][3], 3)

    def test_serial_close_error_preserves_keyboard_interrupt_exit_code(self):
        clock = Clock()
        class CloseFailure(SerialDouble):
            def __exit__(self, *args):
                self.closed = True
                raise OSError('serial close failed')
        port = CloseFailure(clock, [KeyboardInterrupt()])
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (130, ''))
        report = json.loads(stderr)
        self.assertEqual(report['error_type'], 'KeyboardInterrupt')
        self.assertTrue(any('serial close failed' in e for e in report['partial']['cleanup_errors']))

    def test_output_close_error_preserves_primary_read_error(self):
        clock = Clock()
        port = SerialDouble(clock, [OSError('primary read disconnected')])
        original_open = Path.open
        class CloseFailure:
            def __init__(self, file):
                self.file = file
            def __enter__(self):
                return self.file.__enter__()
            def __exit__(self, *args):
                self.file.__exit__(*args)
                raise OSError('CSV close failed')
        def open_file(path, *args, **kwargs):
            file = original_open(path, *args, **kwargs)
            return CloseFailure(file) if path.name == 'out.csv' else file
        with mock.patch.object(Path, 'open', open_file):
            status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        report = json.loads(stderr)
        self.assertEqual((status, stdout), (2, ''))
        self.assertEqual(report['error'], 'primary read disconnected')
        self.assertTrue(any('CSV close failed' in e for e in report['partial']['cleanup_errors']))

    def test_close_failure_after_success_is_failure_with_cleanup_diagnostic(self):
        clock = Clock()
        port = board(clock)
        def fail_close(*args):
            port.closed = True
            raise OSError('serial close failed')
        # __exit__ is resolved on the class, as with a real context manager.
        with mock.patch.object(SerialDouble, '__exit__', lambda self, *args: fail_close(*args)):
            status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertIn('serial close failed', report['error'])
        self.assertTrue(report['partial'].get('cleanup_errors'))

    def test_outputs_refuse_existing_csv_raw_and_same_path(self):
        for raw_exists in (False, True):
            output, raw = self.root / 'out.csv', self.root / 'raw.bin'
            existing = raw if raw_exists else output
            existing.write_bytes(b'preserve me')
            with contextlib.ExitStack() as stack, self.assertRaises(FileExistsError):
                motor._open_outputs(stack, output, raw)
            self.assertEqual(existing.read_bytes(), b'preserve me')
            self.assertFalse((output if raw_exists else raw).exists())
            existing.unlink()
        with contextlib.ExitStack() as stack, self.assertRaises(ValueError):
            motor._open_outputs(stack, output, output)
        self.assertFalse(output.exists())

    def test_decode_offline_records_raw_csv_and_summary(self):
        source, output, raw = (self.root / p for p in ('in.bin', 'out.csv', 'raw.bin'))
        wire = sample() + sample(seq=1, tick_us=4000)
        source.write_bytes(wire)
        status, stdout, stderr, serial = self.cli(['decode', '--input', str(source),
                '--output', str(output), '--raw-output', str(raw)])
        self.assertEqual((status, stderr), (0, ''))
        self.assertEqual(json.loads(stdout)['frames'], 2)
        self.assertEqual(raw.read_bytes(), wire)
        self.assertEqual(len(list(csv.DictReader(io.StringIO(output.read_text())))), 2)
        serial.assert_not_called()

    def test_capture_is_read_only_and_closes_port(self):
        clock = Clock()
        wire = sample()
        port = SerialDouble(clock, [wire])
        raw, output = self.root / 'raw.bin', self.root / 'capture.csv'
        status, stdout, stderr, serial = self.cli(['capture', '--port', 'FAKE_ONLY',
                '--seconds', '.03', '--output', str(output), '--raw-output', str(raw)], port, clock)
        self.assertEqual((status, stderr), (0, ''))
        self.assertEqual(json.loads(stdout)['frames'], 1)
        self.assertEqual(port.writes, [])
        self.assertTrue(port.closed)
        self.assertEqual(raw.read_bytes(), wire)
        self.assertEqual(serial.call_args.kwargs['baudrate'], 230400)

    def test_powered_requires_explicit_confirmation(self):
        config = configuration()
        config['power'] = 'powered'
        self.config_path.write_text(json.dumps(config))
        status, stdout, stderr, serial = self.cli(self.run_args())
        self.assertEqual(status, 2)
        self.assertIn('--confirm-powered', json.loads(stderr)['error'])
        self.assertEqual(stdout, '')
        self.assertFalse((self.root / 'out.csv').exists())
        serial.assert_not_called()
        clock = Clock()
        port = board(clock)
        status, _, stderr, _ = self.cli(self.run_args('--confirm-powered'), port, clock)
        self.assertEqual((status, stderr), (0, ''))
        self.assertEqual(port.writes[-1][1][3], 3)

    def test_cli_rejects_nonfinite_or_negative_stop_keepalive_delay(self):
        for value in ('nan', 'inf', '-1'):
            with self.subTest(value=value):
                output = self.root / f'{value}.csv'
                status, _, stderr, serial = self.cli(self.run_args('--stop-keepalive-after-run-ms', value,
                                                                  '--output', str(output)))
                self.assertEqual(status, 2)
                self.assertIn('stop', json.loads(stderr)['error'])
                serial.assert_not_called()
                self.assertFalse(output.exists())

    def test_cli_exits_with_abort_on_success(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), dict(state_code=0, flags=0x43,
                     config_id=0, config_crc=0, stop_reason_code=1)])
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stderr), (0, ''))
        self.assertTrue(json.loads(stdout)['result']['started'])
        self.assertEqual(port.writes[-1][1][3], 3)
        self.assertTrue(port.closed)

    def test_cli_preserves_primary_read_error_if_abort_also_fails(self):
        clock = Clock()
        port = board(clock, [OSError('primary read disconnected')])
        respond = port.on_write

        def fail_abort(port, command):
            if command[3] == 3:
                raise OSError('cleanup abort disconnected')
            respond(port, command)
        port.on_write = fail_abort
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertIn('primary read disconnected', report['error'])
        self.assertIsNotNone(report['partial'])
        self.assertGreaterEqual(report['partial']['frames'], 6)
        self.assertIn('cleanup abort disconnected', json.dumps(report))
        self.assertTrue(port.closed)
        self.assertEqual(port.writes[-1][1][3], 3)

    def test_cli_ctrl_c_attempts_abort_and_reports_partial_data(self):
        clock = Clock()
        port = board(clock, [KeyboardInterrupt()])
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (130, ''))
        report = json.loads(stderr)
        self.assertIn('interrupt', report['error'].lower())
        self.assertGreaterEqual(report['partial']['frames'], 6)
        self.assertTrue(port.closed)
        self.assertEqual(port.writes[-1][1][3], 3)

    def test_cli_serial_writes_have_finite_timeout(self):
        clock = Clock()
        port = board(clock)
        status, _, stderr, serial = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stderr), (0, ''))
        timeout = serial.call_args.kwargs.get('write_timeout')
        self.assertIsNotNone(timeout, 'blocking serial writes defeat bounded exits')
        self.assertGreater(timeout, 0)
        self.assertLessEqual(timeout, 0.5)

    def test_cli_capture_read_error_keeps_partial_data_without_abort(self):
        clock = Clock()
        port = SerialDouble(clock, [sample(), OSError('capture disconnected')])
        status, stdout, stderr, _ = self.cli(['capture', '--port', 'FAKE_ONLY', '--seconds', '.1',
                '--output', str(self.root / 'capture.csv')], port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertIn('capture disconnected', report['error'])
        self.assertIsNotNone(report['partial'])
        self.assertEqual(report['partial']['frames'], 1)
        self.assertEqual(port.writes, [])
        self.assertTrue(port.closed)

    def test_cli_run_fault_and_start_timeout_both_finish_with_abort(self):
        for name, states, expected in (
                ('fault', [dict(state_code=2), dict(state_code=255, flags=0x43,
                               config_id=0, config_crc=0, stop_reason_code=19)], 'speed_limit'),
                ('start_timeout', [], 'host_abort')):
            with self.subTest(name=name):
                clock = Clock()
                port = board(clock, states)
                status, stdout, stderr, _ = self.cli(self.run_args('--output',
                        str(self.root / f'{name}.csv')), port, clock)
                self.assertEqual((status, stderr), (0, ''))
                self.assertEqual(json.loads(stdout)['result']['end_reason'], expected)
                self.assertEqual(port.writes[-1][1][3], 3)
                self.assertTrue(port.closed)
                self.assertLess(clock(), 0.5)

    def test_cli_run_communication_loss_has_partial_data_and_final_abort(self):
        clock = Clock(limit=1)
        port = board(clock, [dict(state_code=2)])
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertIn('telemetry lost', report['error'])
        self.assertGreaterEqual(report['partial']['frames'], 7)
        self.assertEqual(port.writes[-1][1][3], 3)
        self.assertTrue(port.closed)
        self.assertLess(clock(), 0.8)

    def test_cli_sync_and_echo_failures_still_attempt_final_abort(self):
        for name in ('sync', 'echo'):
            with self.subTest(name=name):
                clock = Clock(limit=3)
                port = SerialDouble(clock) if name == 'sync' else board(clock, echo_id=99)
                status, stdout, stderr, _ = self.cli(self.run_args('--output',
                        str(self.root / f'{name}.csv')), port, clock)
                self.assertEqual((status, stdout), (2, ''))
                self.assertIsNotNone(json.loads(stderr)['partial'])
                self.assertEqual(port.writes[-1][1][3], 3)
                self.assertTrue(port.closed)
                self.assertLess(clock(), 2.3)

    def test_cli_cleanup_failure_after_completion_is_not_reported_as_success(self):
        clock = Clock()
        port = board(clock, [dict(state_code=2), dict(state_code=0, flags=0x43,
                     config_id=0, config_crc=0, stop_reason_code=1)])
        respond = port.on_write

        def fail_abort(port, command):
            if command[3] == 3:
                raise OSError('final ABORT write timed out')
            respond(port, command)
        port.on_write = fail_abort
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertIn('final ABORT write timed out', report['error'])
        self.assertEqual(report['partial']['result']['end_reason'], 'complete')
        self.assertTrue(report['partial']['cleanup_errors'])
        self.assertTrue(port.closed)

    def test_cli_invalid_config_type_produces_json_diagnostic(self):
        self.config_path.write_text('[]')
        status, stdout, stderr, serial = self.cli(self.run_args())
        self.assertEqual((status, stdout), (2, ''))
        self.assertIn('JSON object', json.loads(stderr)['error'])
        serial.assert_not_called()

    def test_cli_timing_validation_precedes_files_and_serial_access(self):
        for option, value in (('--wait-start-s', 'nan'), ('--wait-start-s', '0'),
                              ('--observe-s', 'inf'), ('--observe-s', '-1'),
                              ('--keepalive-period-ms', 'nan'), ('--keepalive-period-ms', '0'),
                              ('--keepalive-period-ms', '250')):
            with self.subTest(option=option, value=value):
                status, stdout, stderr, serial = self.cli(self.run_args(option, value))
                self.assertEqual((status, stdout), (2, ''))
                self.assertTrue(json.loads(stderr)['error'])
                serial.assert_not_called()
                self.assertFalse((self.root / 'out.csv').exists())

    def test_cli_unconfirmed_start_timeout_is_nonzero_with_partial_and_cleanup(self):
        clock = Clock(limit=1)
        port = abort_reply(board(clock, [dict(state_code=1)]), None)
        status, stdout, stderr, _ = self.cli(self.run_args('--wait-start-s', '.03'), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertRegex(report['error'], '(?i)stop.*not confirmed')
        self.assertEqual(report['partial']['final']['state'], 'ARMED')
        self.assertFalse(report['partial']['stop_confirmed'])
        self.assertTrue(report['partial']['cleanup_errors'])
        self.assertGreaterEqual(sum(c[3] == 3 for _, c in port.writes), 2)
        self.assertTrue(port.closed)
        self.assertLess(clock(), 0.9)

    def test_cli_old_stop_cannot_complete_run_or_confirm_final_abort(self):
        clock = Clock(limit=1)
        old = sample(seq=3, tick_us=8000, stop_reason_code=1)
        port = abort_reply(board(clock, [dict(state_code=2), old]), old)
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertRegex(report['error'], '(?i)session.*(reset|continuity)')
        self.assertFalse(report['partial']['stop_confirmed'])
        self.assertTrue(report['partial']['cleanup_errors'])
        self.assertEqual(port.writes[-1][1][3], 3)
        self.assertTrue(port.closed)

    def test_cli_final_abort_rejects_duplicate_bad_time_and_reset_stop(self):
        replies = [sample(seq=6, tick_us=14000, stop_reason_code=4),
                   sample(seq=3, tick_us=16000, stop_reason_code=4),
                   sample(seq=7, tick_us=16001, stop_reason_code=4),
                   sample(seq=0, tick_us=0, stop_reason_code=4)]
        for index, reply in enumerate(replies):
            with self.subTest(index=index):
                clock = Clock()
                port = abort_reply(board(clock, [dict(state_code=2), OSError('primary disconnect')]), reply)
                status, stdout, stderr, _ = self.cli(self.run_args('--output',
                        str(self.root / f'final-{index}.csv')), port, clock)
                self.assertEqual((status, stdout), (2, ''))
                report = json.loads(stderr)
                self.assertIn('primary disconnect', report['error'])
                self.assertFalse(report['partial']['stop_confirmed'])
                self.assertTrue(report['partial']['cleanup_errors'])

    def test_cli_transient_terminal_without_off_evidence_is_failure(self):
        clock = Clock(limit=1)
        port = abort_reply(board(clock, [dict(state_code=2),
                           dict(state_code=255, flags=0x47, config_id=0, config_crc=0,
                                pwm_count=125, motor_mode=1, stop_reason_code=18)]), None)
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertFalse(report['partial']['stop_confirmed'])
        self.assertTrue(report['partial']['cleanup_errors'])

    def test_cli_redundant_abort_can_reuse_recent_confirmed_shutdown(self):
        clock = Clock()
        port = abort_reply(board(clock, [dict(state_code=2),
                           dict(state_code=0, flags=0x43, config_id=0,
                                config_crc=0, stop_reason_code=1)]), None)
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stderr), (0, ''))
        report = json.loads(stdout)
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(report['result']['end_reason'], 'complete')
        self.assertEqual(port.writes[-1][1][3], 3)

    def test_cli_reset_during_final_abort_invalidates_prior_shutdown(self):
        clock = Clock()
        port = abort_reply(board(clock, [dict(state_code=2),
                           dict(state_code=0, flags=0x43, config_id=0,
                                config_crc=0, stop_reason_code=1)]), sample(seq=0, tick_us=0))
        status, stdout, stderr, _ = self.cli(self.run_args(), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertFalse(report['partial']['stop_confirmed'])
        self.assertTrue(report['partial']['cleanup_errors'])

    def test_cli_stale_shutdown_cannot_replace_final_confirmation(self):
        clock = Clock(limit=1)
        port = abort_reply(board(clock, [dict(state_code=2),
                           dict(state_code=0, flags=0x43, config_id=0,
                                config_crc=0, stop_reason_code=1)]), None)
        status, stdout, stderr, _ = self.cli(self.run_args('--observe-s', '.51'), port, clock)
        self.assertEqual((status, stdout), (2, ''))
        report = json.loads(stderr)
        self.assertFalse(report['partial']['stop_confirmed'])
        self.assertTrue(report['partial']['cleanup_errors'])


if __name__ == '__main__':
    unittest.main()
