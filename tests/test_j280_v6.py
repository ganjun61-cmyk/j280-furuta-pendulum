"""v6 host side: wire protocol, trajectory profiles, board sessions, built-in frames."""
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
import j280_builtin as builtin  # noqa: E402
import j280_swing_protocol as protocol5  # noqa: E402
import j280_traj as tj  # noqa: E402
import j280_v6_protocol as wire  # noqa: E402
from j280_measurement import crc16_ccitt  # noqa: E402
from test_j280_balance_protocol import telemetry_fixture  # noqa: E402
from test_j280_board_session import Clock  # noqa: E402
from test_j280_swing import FakeBoard5, SAFE, ARMED, BALANCE, SWING  # noqa: E402

SWING_PROFILE = ROOT / 'docs/data/swing-board-2026-10-18/swing-605-config.json'


def frame6(status=0, velocity=0, u_ff=0, swing=(0, 0, 0, 0, 0, 0, 1), last=0, **changes):
    data = bytearray(telemetry_fixture(**changes))
    data[2] = 6
    struct.pack_into('<H', data, 46, status)
    struct.pack_into('<ii', data, 84, velocity, u_ff)
    struct.pack_into('<iiiihHB', data, 104, *swing)
    data[125] = last
    data[126:128] = crc16_ccitt(data[2:126]).to_bytes(2, 'little')
    return bytes(data)


class FakeBoard6(FakeBoard5):
    """v6: version-6 commands and telemetry; the reference moves while BALANCE lasts."""

    def write(self, data):
        if data[2] != 6:
            raise AssertionError('non-v6 command sent to a v6 board')
        patched = bytearray(data)
        patched[2] = 5
        n = super().write(bytes(patched))
        self.writes[-1] = (self.writes[-1][0], self.writes[-1][1], bytes(data))
        return n

    def frame(self):
        base = bytearray(super().frame())
        balancing = self.state == BALANCE
        status = (0x09 if balancing else 0) | (2 << 8 if balancing else 0)
        velocity = 300 if balancing else 0
        sw = struct.unpack_from('<iiiihHB', base, 104)
        return frame6(status, velocity, 1000 if balancing else 0, swing=sw[:6] + (sw[6] & 0x6F,),
                      **{k: v for k, v in decode_fixture_fields(base).items()})


def decode_fixture_fields(base):
    row = protocol5._decode(bytes(base))
    keys = ('packet_seq', 'tick_us', 'raw_seq', 'flags', 'state_code', 'stop_reason_code', 'config_id',
            'config_crc', 'motor_mode', 'applied_compare', 'enc_count')
    return {k: row[k] for k in keys}


class V6ProtocolTests(unittest.TestCase):
    def test_decode_trajectory_words_and_domains(self):
        row = wire.FrameStream().feed(frame6(0x0F | (0xFE << 8), -1234, -(5 << 16), state_code=3,
                                             flags=0x53 | 0x04, config_id=7, config_crc=9,
                                             motor_mode=2, applied_compare=100))[0]
        self.assertEqual((row['traj_moving'], row['traj_rotating'], row['standalone'], row['traj_settled']),
                         (True, True, True, True))
        self.assertEqual((row['traj_index'], row['ref_velocity_q16'], row['u_ff_q16']), (-2, -1234, -(5 << 16)))
        self.assertEqual((row['pwm_period'], row['sample_period_us'], row['telemetry_period_us']), (2500, 2000, 10000))
        bad = (dict(status=0x80), dict(status=0x01), dict(u_ff=1126 << 16), dict(velocity=65536),
               dict(swing=(0, 0, 0, 0, 0, 0, 0x11)), dict(last=1))
        for change in bad:
            s = wire.FrameStream()
            with self.subTest(change=change):
                self.assertEqual(s.feed(frame6(**change)), [])
                self.assertEqual(s.stats['frames_invalid'], 1)
        # Standalone may show while the built-in CONFIG is still being checked in SAFE.
        self.assertTrue(wire.FrameStream().feed(frame6(0x04, state_code=0))[0]['standalone'])
        # Bits 4..6 are SW3 pressed, SW4 pressed and DIP 2 on.
        row = wire.FrameStream().feed(frame6(0x70, state_code=0))[0]
        self.assertEqual((row['sw3_pressed'], row['sw4_pressed'], row['dip2_on']), (True, True, True))
        row = wire.FrameStream().feed(frame6(0x20, state_code=0))[0]
        self.assertEqual((row['sw3_pressed'], row['sw4_pressed'], row['dip2_on']), (False, True, False))

    def test_config_layout_run_zero_and_versions(self):
        values = json.loads(SWING_PROFILE.read_text(encoding='utf-8'))['values']
        t = tj.default_params(ki=1000, i_window=80)
        data = wire.build_command('config', dict(values, run_ms=0), traj=t)
        self.assertEqual((data[2], data[3]), (6, 1))
        self.assertEqual(struct.unpack_from('<10H', data, 170), tuple(t[f] for f in tj.FIELDS))
        self.assertEqual(struct.unpack_from('<H', data, 190)[0], crc16_ccitt(data[2:190]))
        self.assertEqual(struct.unpack_from('<H', data, 8)[0], 0)
        with self.assertRaises(ValueError):
            protocol5.build_command('config', dict(values, run_ms=0))
        with self.assertRaises(ValueError):
            wire.build_command('config', values, traj=dict(t, accel=0))
        with self.assertRaises(ValueError):
            wire.build_command('keepalive', traj=t)
        self.assertEqual(wire.build_command('abort')[2:4], bytes((6, 3)))
        swing = json.loads(SWING_PROFILE.read_text(encoding='utf-8'))['swing']
        self.assertEqual(wire.build_command('swing', swing)[2:4], bytes((6, 4)))


class V6ProfileAndSessionTests(unittest.TestCase):
    def profile(self, **extra):
        src = json.loads(SWING_PROFILE.read_text(encoding='utf-8'))
        src.update(extra)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'p.json'
            path.write_text(json.dumps(src), encoding='utf-8')
            return profiles.load_config(path)

    def test_traj_profiles_validate_and_select_the_v6_wire(self):
        p = self.profile(traj=tj.default_params(), values=dict(
            json.loads(SWING_PROFILE.read_text(encoding='utf-8'))['values'], run_ms=0))
        self.assertIs(board.wire_of(p), wire)
        self.assertEqual(p['values']['run_ms'], 0)
        with self.assertRaises(ValueError):
            self.profile(values=dict(json.loads(SWING_PROFILE.read_text(encoding='utf-8'))['values'], run_ms=0))
        with self.assertRaises(ValueError):
            self.profile(traj=tj.default_params(), backlash=dict(lash_half_count=1))
        with self.assertRaises(ValueError):
            self.profile(traj=dict(tj.default_params(), lean_arm=1 << 14))

    def test_session_sends_v6_frames_and_unlimited_run_waits_for_the_board(self):
        clock = Clock()
        p = self.profile(traj=tj.default_params(), values=dict(
            json.loads(SWING_PROFILE.read_text(encoding='utf-8'))['values'], run_ms=0))
        script = [(SAFE, 5, None), (ARMED, 10, None), (SWING, 150, None), (BALANCE, 6000, None), (SAFE, 1, 2)]
        fake = FakeBoard6(clock, script)
        sink = io.StringIO()
        rec = board.Recorder(board.csv.DictWriter(sink, fieldnames=board.CSV_FIELDS6), io.BytesIO(),
                             clock=clock, wire=wire, keep_rows=True)
        result = board.run_session(fake, rec, p, wait_start_s=5.0, observe_s=0.1, keepalive_period_s=0.05,
                                   clock=clock)
        self.assertEqual([c for _, c, _ in fake.writes if c not in (2, 3)][:2], [1, 4])
        self.assertTrue(all(d[2] == 6 for _, _, d in fake.writes))
        self.assertEqual((result['end_state'], result['end_reason']), ('SAFE', 'stop_key'))
        self.assertGreater(result['balance_s'], 59)
        moving = [r for r in rec.kept if r['state'] == 'BALANCE']
        self.assertTrue(moving and all(r['traj_moving'] and r['ref_velocity_q16'] == 300 for r in moving))
        self.assertIn('u_ff_q16', rec.fields)
        with self.assertRaisesRegex(ValueError, 'v6 protocol'):
            board.run_session(fake, rec, p, wait_start_s=1, observe_s=0, keepalive_period_s=0.05,
                              clock=clock, wire=protocol5)


class BuiltinFramesTests(unittest.TestCase):
    def test_generated_verilog_matches_and_frames_are_the_605_profile(self):
        self.assertEqual(builtin.VERILOG.read_text(encoding='ascii'), builtin.verilog())
        config, swing, crc = builtin.frames()
        self.assertEqual((config[0], config[1], swing[0], swing[1]), (6, 1, 6, 4))
        self.assertEqual(struct.unpack_from('<HH', config, 2), (6001, 1))
        self.assertEqual(struct.unpack_from('<H', config, 6)[0], 0)
        self.assertEqual(struct.unpack_from('<H', swing, 2)[0], 6001)
        frame = bytearray(192)
        frame[:2], frame[2:190] = b'\xa5\x5a', config
        self.assertEqual(crc, crc16_ccitt(frame[2:190]))
        p = builtin.profile()
        src = json.loads(SWING_PROFILE.read_text(encoding='utf-8'))
        skip = ('config_id', 'run_ms', 'keepalive_ms')
        self.assertEqual({k: v for k, v in p['values'].items() if k not in skip},
                         {k: v for k, v in src['values'].items() if k not in skip})
        self.assertEqual(p['values']['keepalive_ms'], 20)
        self.assertEqual(struct.unpack_from('<10H', config, 168), tuple(p['traj'][f] for f in tj.FIELDS))


if __name__ == '__main__':
    unittest.main()
