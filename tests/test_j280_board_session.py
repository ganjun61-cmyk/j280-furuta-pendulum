"""Balance v4 board-session tool against a scripted fake board (no serial port)."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'tests'))
import j280_balance as profiles  # noqa: E402
import j280_balance_board as board  # noqa: E402
from test_j280_balance_protocol import telemetry_fixture  # noqa: E402

SAFE, ARMED, CAPTURE, BALANCE, FAULT = 0, 1, 2, 3, 255


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeBoard:
    """One 100 Hz telemetry frame per read; reacts to CONFIG/KEEPALIVE/ABORT."""

    def __init__(self, clock, script=(), start_state=SAFE, echo_crc_error=False,
                 silent_after=None, reset_after=None):
        self.clock, self.script = clock, list(script)
        self.state, self.reason = start_state, 0
        self.loaded_id = self.loaded_crc = 0
        self.seq = 0
        self.writes = []
        self.timeout = 0.01
        self.echo_crc_error = echo_crc_error
        self.silent_after, self.reset_after = silent_after, reset_after
        self.step = None

    # serial.Serial-like interface --------------------------------------
    @property
    def in_waiting(self):
        return 0 if self.silent() else 128

    def silent(self):
        return self.silent_after is not None and self.seq >= self.silent_after

    def write(self, data):
        self.writes.append((self.clock.now, data[3], bytes(data)))
        if data[3] == 1 and self.state == SAFE:
            self.loaded_id = int.from_bytes(data[4:6], 'little')
            self.loaded_crc = int.from_bytes(data[190:192], 'little') ^ int(self.echo_crc_error)
            self.step = 0
        elif data[3] == 3 and self.state in (SAFE, ARMED, CAPTURE, BALANCE) and self.loaded_id:
            self.stop(SAFE, 4)
        return len(data)

    def read(self, size):
        if self.silent():
            self.clock.now += self.timeout
            return b''
        if self.reset_after is not None and self.seq == self.reset_after:
            self.seq, self.reset_after = 0, None
            self.state, self.reason, self.loaded_id, self.loaded_crc = SAFE, 0, 0, 0
        if self.step is not None and self.loaded_id and self.script:
            state, frames, reason = self.script[0]
            self.state = state
            self.step += 1
            if self.step >= frames:
                self.script.pop(0)
                self.step = 0
                if state in (SAFE, FAULT) and reason is not None:
                    self.stop(state, reason)
        frame = self.frame()
        self.seq += 1
        self.clock.now += 0.01
        return frame

    # board model -------------------------------------------------------
    def stop(self, state, reason):
        self.state, self.reason = state, reason
        self.loaded_id = self.loaded_crc = 0
        self.step = None
        self.script = []

    def frame(self):
        driving = self.state == BALANCE
        flags = 0x43 | (0x10 if self.loaded_id else 0) | (0x04 if driving else 0) | (
            0x80 if self.state == FAULT else 0)
        return telemetry_fixture(packet_seq=self.seq, tick_us=2000 + 10000 * self.seq,
                                 raw_seq=5 * self.seq, flags=flags, state_code=self.state,
                                 stop_reason_code=self.reason, config_id=self.loaded_id,
                                 config_crc=self.loaded_crc, motor_mode=2 if driving else 4,
                                 applied_compare=150 if driving else 0, enc_count=7,
                                 u_linear_q16=0, u_compensated_q16=0, u_model_q16=0)


def isolated_config():
    return profiles.build_isolated_config(734.0, config_id=0x1234)


def recorder(clock):
    sink = io.StringIO()
    writer = board.csv.DictWriter(sink, fieldnames=board.CSV_FIELDS)
    return board.Recorder(writer, io.BytesIO(), clock=clock)


NORMAL = [(SAFE, 5, None), (ARMED, 20, None), (CAPTURE, 30, None), (BALANCE, 40, None),
          (SAFE, 1, 2)]


class BoardSessionTests(unittest.TestCase):
    def run_session(self, fake, clock, **kw):
        rec = recorder(clock)
        args = dict(wait_start_s=kw.pop('wait_start_s', 5.0), observe_s=0.1, keepalive_period_s=0.05)
        args.update(kw)
        return rec, board.run_session(fake, rec, isolated_config(), clock=clock, **args)

    def test_normal_takeover_and_local_stop(self):
        clock = Clock()
        fake = FakeBoard(clock, NORMAL)
        rec, result = self.run_session(fake, clock)
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'stop_key'))
        self.assertTrue(result['balance_seen'])
        self.assertEqual(rec.balance_frames, 40)
        commands = [c for _, c, _ in fake.writes]
        self.assertEqual(commands[0], 1)
        self.assertNotIn(3, commands)
        keepalive_times = [t for t, c, _ in fake.writes if c == 2]
        self.assertGreater(len(keepalive_times), 10)
        self.assertLessEqual(max(b - a for a, b in zip(keepalive_times, keepalive_times[1:])), 0.07)
        self.assertEqual([e['state'] for e in rec.events][-4:], ['ARMED', 'CAPTURE', 'BALANCE', 'SAFE'])

    def test_echo_must_carry_the_sent_crc(self):
        clock = Clock()
        fake = FakeBoard(clock, NORMAL, echo_crc_error=True)
        with self.assertRaisesRegex(RuntimeError, 'not echoed'):
            self.run_session(fake, clock)

    def test_board_must_start_safe(self):
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'not SAFE'):
            self.run_session(FakeBoard(clock, start_state=FAULT), clock)

    def test_telemetry_loss_and_reset_end_the_session(self):
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'telemetry lost'):
            self.run_session(FakeBoard(clock, NORMAL, silent_after=40), clock)
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'continuity lost'):
            self.run_session(FakeBoard(clock, NORMAL, reset_after=40), clock)

    def test_start_timeout_aborts_and_confirms_stop(self):
        clock = Clock()
        fake = FakeBoard(clock, [(SAFE, 10**6, None)])
        _, result = self.run_session(fake, clock, wait_start_s=1.0)
        self.assertEqual(result['phase'], 'start_timeout')
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'host_abort'))
        self.assertIn(3, [c for _, c, _ in fake.writes])

    def test_keepalive_suppression_after_takeover(self):
        clock = Clock()
        script = [(ARMED, 5, None), (CAPTURE, 5, None), (BALANCE, 60, None), (FAULT, 1, 20)]
        fake = FakeBoard(clock, script)
        _, result = self.run_session(fake, clock, stop_keepalive_after_balance_ms=100)
        balance_start = min(t for t, c, _ in fake.writes if c == 2 and t > 0.1)
        late = [t for t, c, _ in fake.writes if c == 2 and t > clock.now - 0.4]
        self.assertEqual(late, [])
        self.assertEqual((result['end_state'], result['end_reason']), ('FAULT', 'keepalive_timeout'))
        self.assertGreater(balance_start, 0)

    def test_balance_past_run_deadline_is_an_error(self):
        clock = Clock()
        cfg_run = isolated_config()['values']['run_ms'] / 1000
        frames = int((cfg_run + board.RUN_GRACE_S) / 0.01) + 20
        fake = FakeBoard(clock, [(ARMED, 5, None), (CAPTURE, 5, None), (BALANCE, frames, None)])
        with self.assertRaisesRegex(RuntimeError, 'exceeded run_ms'):
            self.run_session(fake, clock)

    def test_capture_cue_tells_direction(self):
        row = dict(state='CAPTURE', adc_raw10=740, upright_zero_q16=734 << 16)
        self.assertEqual(board.capture_cue(row, 8), (board.CUE_IN, 80))
        self.assertEqual(board.capture_cue(dict(row, adc_raw10=757), 8)[0], board.CUE_PAST)
        self.assertEqual(board.capture_cue(dict(row, adc_raw10=700), 8)[0], board.CUE_SHORT)
        self.assertEqual(board.capture_cue(dict(row, adc_raw10=757), 8, lift_sign=-1)[0], board.CUE_SHORT)
        self.assertIsNone(board.capture_cue(dict(row, state='BALANCE'), 8))
        clock = Clock()
        tones = []
        self.run_session(FakeBoard(clock, NORMAL), clock, cue=lambda f, d: tones.append(f))
        self.assertTrue(tones)
        self.assertLessEqual(len(tones), 0.3 / board.CUE_PERIOD_S + 2)

    def test_nonblocking_cue_returns_immediately_and_skips_overlap(self):
        import threading, time as _time
        release, calls = threading.Event(), []

        def slow(frequency, duration):
            calls.append(frequency)
            release.wait(2)
        play = board.nonblocking(slow)
        start = _time.monotonic()
        play(1000, 60)
        play(2000, 60)
        self.assertLess(_time.monotonic() - start, 0.1)
        release.set()
        _time.sleep(0.05)
        self.assertEqual(calls, [1000])

    def test_wire_issues_flags_missing_packets(self):
        clean = b''.join(telemetry_fixture(packet_seq=i, tick_us=2000 + 10000 * i, raw_seq=5 * i)
                         for i in range(3))
        gap = clean + telemetry_fixture(packet_seq=5, tick_us=52000, raw_seq=25)
        for data, ok in ((clean, True), (gap, False)):
            stream = board.protocol.FrameStream()
            stream.feed(data)
            self.assertEqual(not board.wire_issues(stream.stats), ok)

    def test_main_sends_abort_even_when_the_session_fails(self):
        clock = Clock()
        fake = FakeBoard(clock, NORMAL, echo_crc_error=True)
        module = types.SimpleNamespace(Serial=lambda **kw: contextlib.nullcontext(fake))
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            config = folder / 'c.json'
            config.write_text(json.dumps(isolated_config()), encoding='utf-8')
            out, err = io.StringIO(), io.StringIO()
            with mock.patch.dict(sys.modules, serial=module), \
                    contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                    rc = board.main(['run', '--port', 'X', '--config', str(config), '--no-cue',
                                 '--output', str(folder / 'o.csv'), '--raw-output', str(folder / 'o.bin')])
            self.assertEqual(rc, 2)
            self.assertEqual(fake.writes[-1][1], 3)
            self.assertIn('not echoed', json.loads(err.getvalue())['error'])
            self.assertTrue((folder / 'o.bin').stat().st_size > 0)

    def test_powered_profile_requires_confirmation_and_make_config(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            path = folder / 'p.json'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(board.main(['make-config', '--upright-code', '734',
                                             '--output', str(path)]), 0)
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(board.main(['make-config', '--upright-code', '734',
                                                 '--output', str(path)]), 2)
            data = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(data['power'], 'isolated')
            self.assertEqual(data['values']['upright_zero_q16'], 734 << 16)
            data['power'] = 'powered'
            path.write_text(json.dumps(data), encoding='utf-8')
            with contextlib.redirect_stderr(io.StringIO()) as err:
                rc = board.main(['run', '--port', 'X', '--config', str(path),
                                 '--output', str(folder / 'o.csv')])
            self.assertEqual(rc, 2)
            self.assertIn('confirm-powered', err.getvalue())


if __name__ == '__main__':
    unittest.main()
