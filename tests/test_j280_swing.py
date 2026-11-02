"""Swing-up v5 host side: integer reference semantics, wire protocol, profiles, board session."""
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'tests'))
import j280_balance as profiles  # noqa: E402
import j280_balance_board as board  # noqa: E402
import j280_balance_protocol as v4  # noqa: E402
import j280_swing_math as sm  # noqa: E402
import j280_swing_protocol as wire  # noqa: E402
from j280_measurement import crc16_ccitt  # noqa: E402
from test_j280_balance_protocol import telemetry_fixture  # noqa: E402
from test_j280_board_session import Clock, FakeBoard  # noqa: E402

DATA = ROOT / 'docs/data'
FIXED = json.loads((DATA / 'balance-design-2026-10-17/swingup-fixed-01.json').read_text(encoding='utf-8'))
BASE = DATA / 'balance-board-2026-10-17/balance-511-config.json'
SAFE, ARMED, BALANCE, SWING, FAULT = 0, 1, 3, 4, 255


def values(**change):
    return dict(FIXED['swing_values'], **change)


def frame5(swing=(0, 0, 0, 0, 0, 0, 1), **changes):
    data = bytearray(telemetry_fixture(**changes))
    data[2] = 5
    struct.pack_into('<iiiihHB', data, 104, *swing)
    data[126:128] = crc16_ccitt(data[2:126]).to_bytes(2, 'little')
    return bytes(data)


class SwingMathTests(unittest.TestCase):
    def step(self, s, code, valid=True, theta=0, **p):
        return sm.step(s, code, valid, theta, values(**p), max_compare=1000, zero_q16=737 << 16)

    def test_compiled_profile_matches_saved_values_and_bounds(self):
        v = sm.compile_params(FIXED['float_params'], config_id=511, max_compare=1000,
                              w0=9.724, count_rad=0.006041524333826525)
        self.assertEqual(v, FIXED['swing_values'])
        for name, change in (('edge', dict(edge_low=960)), ('hang', dict(hang_code=90, hang_window=60)),
                             ('crc', dict(table_crc=sm.TABLE_CRC ^ 1)), ('range', dict(kp=1 << 25))):
            with self.assertRaises(ValueError, msg=name):
                sm.validate(values(**change))
        with self.assertRaises(ValueError):
            sm.validate(dict(values(), extra=1))

    def test_tables_anchor_hanging_upright_and_quadrants(self):
        self.assertEqual(sm.angle_of(221), -(1 << 31))
        self.assertEqual(sm.angle_of(737), 0)
        self.assertEqual(sm.sin_q16(1 << 30), 65536)
        self.assertEqual(sm.cos_q16(0), 65536)
        self.assertEqual(sm.cos_q16(-(1 << 31)), -65536)

    def test_kick_then_law_and_clip(self):
        s = sm.initial_state(221, True)
        s, o = self.step(s, 221, kick_samples=2, kick_compare=-1000)
        self.assertEqual(o['command'], -1000)
        s, o = sm.step(s, 221, True, 0, values(kick_samples=2, kick_compare=-1000),
                       max_compare=400, zero_q16=737 << 16)
        self.assertEqual(o['command'], -400)
        s, o = self.step(s, 221, kick_samples=2)
        self.assertNotIn(abs(o['command']), (1000,))

    def test_blind_entry_and_edge_reacquisition(self):
        s = sm.initial_state(500, True)
        _, o = self.step(s, 0, valid=False)
        self.assertTrue(o['blind_mid'])
        _, o = self.step(s, 530)                       # a valid 30-code jump: blind, not a fault
        self.assertEqual((o['good'], o['blind_mid'], o['blind']), (False, False, 1))
        s = sm.initial_state(30, True)
        s, o = self.step(s, 0, valid=False)
        self.assertFalse(o['blind_mid'])
        self.assertEqual(o['blind'], 1)
        s, o = self.step(s, 500)                       # mid-range after blind: still blind
        self.assertEqual((o['good'], o['blind']), (False, 2))
        s, o = self.step(s, 1010)                      # edge band: re-acquired
        self.assertEqual((o['good'], o['blind']), (True, 0))

    def test_valid_spike_is_skipped_and_reacquired_with_a_widened_step(self):
        s = sm.initial_state(500, True)
        s, o = self.step(s, 530)
        s, o = self.step(s, 545)                       # 45 <= 24*2 from the last good 500
        self.assertEqual((o['good'], o['blind']), (True, 0))
        s = sm.initial_state(500, True)
        for code, good, blind in ((600, False, 1), (560, False, 2), (571, True, 0)):
            s, o = self.step(s, code)                  # 60 > 48, then 71 <= 72
            self.assertEqual((o['good'], o['blind'], o['blind_mid']), (good, blind, False))
        s = sm.initial_state(500, True)
        s, o = self.step(s, 900)
        s, o = self.step(s, 0, valid=False)           # invalid after a mid-range last good code
        self.assertTrue(o['blind_mid'])

    def test_catch_requires_four_good_samples_and_low_rate(self):
        # edge_low above upright lets 737 re-acquire after a blind sample.
        s = sm.initial_state(737, False)
        for _ in range(4):
            s, o = self.step(s, 737, edge_low=740)
            self.assertTrue(o['good'])
            self.assertFalse(o['catch'])
        s, o = self.step(s, 737, edge_low=740)
        self.assertTrue(o['catch'])
        self.assertEqual(o['preset'], [0, 0, 0, 0])
        s, o = self.step(s, 757, edge_low=740)
        self.assertEqual(o['preset'][3], 20 * 125 * 65536)
        s, o = self.step(s, 757, edge_low=740, catch_rate=1)
        self.assertFalse(o['catch'])


class SwingProtocolTests(unittest.TestCase):
    def test_swing_command_layout_crc_and_validation(self):
        data = wire.build_command('swing', values())
        self.assertEqual((len(data), data[:4]), (192, b'\xa5\x5a\x05\x04'))
        unpacked = struct.unpack_from(wire.SWING_FORMAT, data, 4)
        self.assertEqual(unpacked, tuple(values()[f[0]] for f in sm.FIELDS))
        self.assertFalse(any(data[74:190]))
        self.assertEqual(int.from_bytes(data[190:], 'little'), crc16_ccitt(data[2:190]))
        with self.assertRaises(ValueError):
            wire.build_command('swing', values(blind_samples=101))

    def test_v5_config_exit_range_and_version(self):
        base = profiles.load_config(BASE)['values']
        self.assertEqual(wire.build_command('config', dict(base, exit_code=90))[2], 5)
        with self.assertRaises(ValueError):
            wire.build_command('config', dict(base, exit_code=91))
        with self.assertRaises(ValueError):
            v4.build_command('config', dict(base, exit_code=58))
        self.assertEqual(wire.build_command('keepalive')[2:4], b'\x05\x02')

    def test_decode_swing_fields_and_domain(self):
        rows = wire.FrameStream().feed(frame5((-5, 6, -7, 8, -1000, 3, 0b1101), flags=0x57, state_code=4,
                                              config_id=9, config_crc=1, motor_mode=1, applied_compare=1000))
        r = rows[0]
        self.assertEqual((r['state'], r['swing_est_a'], r['swing_command'], r['swing_blind']), ('SWING', -5, -1000, 3))
        self.assertTrue(r['swing_loaded'] and r['swing_catch'] and r['swing_blind_mid'] and not r['swing_good'])
        self.assertEqual(r['lash_offset'], 0)
        for bad in (dict(swing=(0, 0, 0, 0, 0, 0, 0), flags=0x57, state_code=4, config_id=9, config_crc=1,
                         motor_mode=1, applied_compare=10),
                    dict(swing=(0, 0, 0, 0, 0, 0, 0x81)),
                    dict(swing=(0, 0, 0, 0, 1126, 0, 1)),
                    dict(swing=(0, 0, 0, 0, 0, 0, 0x41)),                      # gains2 active, not loaded
                    dict(swing=(0, 0, 0, 0, 0, 0, 0x61))):                     # active outside BALANCE
            stream = wire.FrameStream()
            self.assertEqual(stream.feed(frame5(**bad)), [])
            self.assertEqual(stream.stats['domain_errors'], 1)
        data = bytearray(frame5((0, 0, 0, 0, 0, 0, 0x71), state_code=3, flags=0x57, config_id=9, config_crc=1,
                                motor_mode=2, applied_compare=100))
        data[125] = (-5) & 0xFF
        data[126:128] = crc16_ccitt(data[2:126]).to_bytes(2, 'little')
        r = wire.FrameStream().feed(bytes(data))[0]
        self.assertEqual((r['lash_offset'], r['gap_boosted'], r['gains2_loaded'], r['gains2_active']), (-5, True, True, True))
        data[125] = 16
        data[126:128] = crc16_ccitt(data[2:126]).to_bytes(2, 'little')
        stream = wire.FrameStream()
        self.assertEqual(stream.feed(bytes(data)), [])
        self.assertEqual(stream.stats['domain_errors'], 1)
        self.assertEqual(v4.FrameStream().feed(frame5()), [])

    def test_config_backlash_fields_and_gains2_layout(self):
        base = profiles.load_config(BASE)['values']
        plain = wire.build_command('config', base)
        data = wire.build_command('config', base, backlash=dict(lash_half_count=5, gap_boost_compare=100,
                                                               gap_threshold_compare=75))
        self.assertEqual(data[:170], plain[:170])
        self.assertEqual(struct.unpack_from('<3H', data, 170), (5, 100, 75))
        self.assertFalse(any(data[176:190]))
        self.assertEqual(int.from_bytes(data[190:], 'little'), crc16_ccitt(data[2:190]))
        for bad in (dict(lash_half_count=16), dict(gap_boost_compare=251), dict(gap_threshold_compare=-1), dict(x=1)):
            with self.assertRaises(ValueError):
                wire.build_command('config', base, backlash=bad)
        co = list(range(-6, 6))
        g = wire.build_command('gains2', dict(config_id=601, switch_ms=2000, coefficients=co))
        self.assertEqual(g[:4], bytes((0xA5, 0x5A, 5, 5)))
        self.assertEqual(struct.unpack_from('<HH12i', g, 4), (601, 2000, *co))
        self.assertFalse(any(g[56:190]))
        for bad in (dict(switch_ms=0), dict(switch_ms=30001), dict(coefficients=co[:11]),
                    dict(coefficients=list(range(-16, 16))),
                    dict(coefficients=[(128 << 20) + 1] + co[1:]), dict(config_id=0)):
            with self.assertRaises(ValueError):
                wire.build_command('gains2', dict(dict(config_id=601, switch_ms=2000, coefficients=co), **bad))


class FakeBoard5(FakeBoard):
    """v5: SWING command, SWING state with drive, swing_loaded flag."""

    def __init__(self, clock, script=(), reject_swing=False, dip_on_frames=0, reject_gains2=False, **kw):
        super().__init__(clock, script, **kw)
        self.swing_loaded = self.gains2_loaded = False
        self.reject_swing, self.reject_gains2 = reject_swing, reject_gains2
        self.dip_on_frames = dip_on_frames

    def write(self, data):
        if data[2] != 5:
            raise AssertionError('v4 command sent to a v5 board')
        if data[3] == 4 and self.state == SAFE and self.loaded_id and not self.reject_swing:
            self.swing_loaded = True
            self.writes.append((self.clock.now, 4, bytes(data)))
            return len(data)
        if data[3] == 5:
            self.writes.append((self.clock.now, 5, bytes(data)))
            if self.state == SAFE and self.loaded_id and not self.reject_gains2:
                self.gains2_loaded = True
            return len(data)
        if data[3] == 1:
            self.swing_loaded = self.gains2_loaded = False
        return super().write(data)

    def stop(self, state, reason):
        super().stop(state, reason)
        self.swing_loaded = self.gains2_loaded = False

    def frame(self):
        driving = self.state in (BALANCE, SWING)
        flags = 0x43 | (0x10 if self.loaded_id else 0) | (0x04 if driving else 0) | (
            0x80 if self.state == FAULT else 0) | (0x20 if self.seq < self.dip_on_frames else 0)
        sflags = int(self.swing_loaded) | (0x20 if self.gains2_loaded else 0) | (
            0x40 if self.gains2_loaded and self.state == BALANCE else 0)
        return frame5(swing=(0, 0, 0, 0, 1000 if self.state == SWING else 0, 0, sflags),
                      packet_seq=self.seq, tick_us=2000 + 10000 * self.seq, raw_seq=5 * self.seq,
                      flags=flags, state_code=self.state, stop_reason_code=self.reason,
                      config_id=self.loaded_id, config_crc=self.loaded_crc,
                      motor_mode=2 if driving else 4, applied_compare=150 if driving else 0,
                      enc_count=7, u_linear_q16=0, u_compensated_q16=0, u_model_q16=0)


class SwingBoardSessionTests(unittest.TestCase):
    def profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'swing.json'
            code = board.main(['make-swing-config', '--base', str(BASE), '--params',
                               json.dumps(FIXED['float_params']), '--config-id', '601',
                               '--output', str(path)])
            self.assertEqual(code, 0)
            return profiles.load_config(path)

    def run_session(self, fake, clock, **kw):
        sink = io.StringIO()
        rec = board.Recorder(board.csv.DictWriter(sink, fieldnames=board.CSV_FIELDS5), io.BytesIO(),
                             clock=clock, wire=wire)
        args = dict(wait_start_s=5.0, observe_s=0.1, keepalive_period_s=0.05)
        args.update(kw)
        return rec, board.run_session(fake, rec, self.profile(), clock=clock, **args)

    def test_profile_carries_validated_swing_block(self):
        p = self.profile()
        self.assertEqual(p['values']['config_id'], p['swing']['config_id'])
        self.assertEqual(p['values']['exit_code'], 80)
        self.assertEqual({k: v for k, v in p['swing'].items() if k != 'config_id'},
                         {k: v for k, v in FIXED['swing_values'].items() if k != 'config_id'})

    def test_swing_catch_balance_and_stop(self):
        clock = Clock()
        script = [(SAFE, 5, None), (ARMED, 10, None), (SWING, 150, None), (BALANCE, 30, None), (SAFE, 1, 2)]
        fake = FakeBoard5(clock, script)
        rec, result = self.run_session(fake, clock)
        self.assertEqual([c for _, c, _ in fake.writes][:2], [1, 4])
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'stop_key'))
        self.assertAlmostEqual(result['catch_s'], 1.5, delta=0.05)
        self.assertEqual((rec.swing_frames, rec.balance_frames), (150, 30))

    def test_rejected_swing_block_is_an_error_and_swing_timeout_guard(self):
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'swing block not accepted'):
            self.run_session(FakeBoard5(clock, [(SAFE, 10**6, None)], reject_swing=True), clock)
        clock = Clock()
        frames = int((6.0 + board.RUN_GRACE_S) / 0.01) + 20
        with self.assertRaisesRegex(RuntimeError, 'SWING exceeded'):
            self.run_session(FakeBoard5(clock, [(SAFE, 5, None), (ARMED, 5, None), (SWING, frames, None)]), clock)

    def test_plain_balance_profile_on_v5_firmware_sends_no_swing_block(self):
        clock = Clock()
        script = [(SAFE, 5, None), (ARMED, 10, None), (2, 20, None), (BALANCE, 30, None), (SAFE, 1, 2)]
        fake = FakeBoard5(clock, script)
        rec = board.Recorder(board.csv.DictWriter(io.StringIO(), fieldnames=board.CSV_FIELDS5), io.BytesIO(),
                             clock=clock, wire=wire)
        result = board.run_session(fake, rec, profiles.load_config(BASE), 5.0, 0.1, 0.05, clock=clock, wire=wire)
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'stop_key'))
        self.assertNotIn(4, [c for _, c, _ in fake.writes])
        self.assertTrue(result['balance_seen'])

    def gains2_profile(self, swing=True, backlash=True):
        p = self.profile() if swing else profiles.load_config(BASE)
        p = json.loads(json.dumps(p))
        p['gains2'] = dict(switch_ms=2000, coefficients=list(range(-6, 6)), source='test')
        if backlash:
            p['backlash'] = dict(lash_half_count=5, gap_boost_compare=100, gap_threshold_compare=75)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'g2.json'
            path.write_text(json.dumps(p), encoding='utf-8')
            return profiles.load_config(path)

    def test_gains2_and_backlash_are_sent_in_order_and_awaited(self):
        clock = Clock()
        script = [(SAFE, 5, None), (ARMED, 5, None), (SWING, 50, None), (BALANCE, 30, None), (SAFE, 1, 2)]
        fake = FakeBoard5(clock, script)
        cfg = self.gains2_profile()
        rec = board.Recorder(board.csv.DictWriter(io.StringIO(), fieldnames=board.CSV_FIELDS5), io.BytesIO(),
                             clock=clock, wire=wire)
        result = board.run_session(fake, rec, cfg, 5.0, 0.1, 0.05, clock=clock)
        commands = [c for _, c, _ in fake.writes if c != 2]
        self.assertEqual(commands[:3], [1, 4, 5])
        sent = next(d for _, c, d in fake.writes if c == 1)
        self.assertEqual(struct.unpack_from('<3H', sent, 170), (5, 100, 75))
        g = next(d for _, c, d in fake.writes if c == 5)
        self.assertEqual(struct.unpack_from('<HH', g, 4), (cfg['values']['config_id'], 2000))
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'stop_key'))
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'gains2 block not accepted'):
            board.run_session(FakeBoard5(clock, script, reject_gains2=True), board.Recorder(
                board.csv.DictWriter(io.StringIO(), fieldnames=board.CSV_FIELDS5), io.BytesIO(), clock=clock,
                wire=wire), cfg, 5.0, 0.1, 0.05, clock=clock)

    def test_plain_balance_with_gains2_uses_v5_and_cancel_aborts(self):
        clock = Clock()
        cfg = self.gains2_profile(swing=False, backlash=False)
        self.assertIs(board.wire_of(cfg), wire)
        fake = FakeBoard5(clock, [(SAFE, 10 ** 6, None)])
        rec = board.Recorder(board.csv.DictWriter(io.StringIO(), fieldnames=board.CSV_FIELDS5), io.BytesIO(),
                             clock=clock, wire=wire)
        stop_at = []
        result = board.run_session(fake, rec, cfg, 50.0, 0.1, 0.05, clock=clock,
                                   cancel=lambda: stop_at.append(clock.now) or clock.now > 1.0)
        self.assertEqual(result['phase'], 'cancelled')
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'host_abort'))
        self.assertEqual([c for _, c, _ in fake.writes if c not in (2,)][:2], [1, 5])
        with self.assertRaises(ValueError):
            board.run_session(fake, rec, cfg, 1.0, 0.1, 0.05, clock=clock, wire=v4)

    def test_swing_profile_loads_only_after_dip_reads_off(self):
        clock = Clock()
        script = [(SAFE, 5, None), (ARMED, 10, None), (SWING, 50, None), (BALANCE, 30, None), (SAFE, 1, 2)]
        fake = FakeBoard5(clock, script, dip_on_frames=60)
        _, result = self.run_session(fake, clock)
        first = min(t for t, c, _ in fake.writes if c == 1)
        self.assertGreaterEqual(first, 0.55)
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'stop_key'))
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'DIP 1 stayed ON'):
            self.run_session(FakeBoard5(clock, [(SAFE, 10**6, None)], dip_on_frames=10**6), clock, wait_start_s=1.0)

    def test_start_timeout_does_not_abort_a_running_swing(self):
        clock = Clock()
        fake = FakeBoard5(clock, [(SAFE, 5, None), (ARMED, 5, None), (SWING, 120, None), (SAFE, 1, 7)])
        _, result = self.run_session(fake, clock, wait_start_s=0.5)
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'swing_timeout'))
        self.assertNotIn(3, [c for _, c, _ in fake.writes])


if __name__ == '__main__':
    unittest.main()
