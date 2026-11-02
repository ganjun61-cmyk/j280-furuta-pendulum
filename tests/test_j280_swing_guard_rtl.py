"""v5/v6 guard: the complete v4 guard suite (VERSION=4, EXIT_MAX=57), v5 swing-up vectors, v6 trajectory/standalone."""
import json
import struct
import sys
import unittest
from pathlib import Path

import test_j280_balance_guard_rtl as v4t

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_builtin as builtin
import j280_swing_math as sm
import j280_swing_protocol as wire

SOURCES = (ROOT / 'rtl/j280_swing/j280_swing_guard.v', ROOT / 'sim/j280_swing/tb_swing_guard.v')
SAFE, ARMED, CAPTURE, BALANCE, SWING, FAULT = 0, 1, 2, 3, 4, 255
DEFAULT = dict(json.loads((ROOT / 'docs/data/balance-design-2026-10-17/swingup-fixed-01.json')
                          .read_text(encoding='utf-8'))['swing_values'], config_id=0x1234)


def swing_block(version=5, **changes):
    values = dict(DEFAULT, **changes)
    data = bytearray(188)
    data[:2] = bytes((version, 4))
    struct.pack_into(wire.SWING_FORMAT, data, 2, *(values[f[0]] for f in sm.FIELDS))
    return data


def lash_config(lash=0, boost=0, threshold=0, dirty=None, **fields):
    data = v4t.config(**dict(dict(version=5), **fields))
    struct.pack_into('<3H', data, 168, lash, boost, threshold)
    if dirty is not None:
        data[dirty] = 1
    return data


def gains2_block(config_id=0x1234, switch_ms=5, coefficients=None, version=5, dirty=None):
    data = bytearray(188)
    data[:2] = bytes((version, 5))
    struct.pack_into('<HH12i', data, 2, config_id, switch_ms,
                     *(coefficients if coefficients is not None else [(i * 977) - 15000 for i in range(12)]))
    if dirty is not None:
        data[dirty] = 1
    return data


class SV(v4t.Vectors):
    def __init__(self):
        super().__init__(version=5)
        self.set('table', sm.TABLE_CRC)

    def swing(self, block=None, **changes):
        return self.command(block if block is not None else swing_block(**changes)).wait(3)

    def swing_armed(self, cfg=None, **changes):
        self.wait(45).load(**(cfg or {})).swing(**changes).expect(rx=1).add('G', 1)
        return self.set('raw', 1).set('stable', 1).wait(1).expect(ARMED)

    def swinging(self, cfg=None, **changes):
        self.swing_armed(cfg, **changes).set('adc', 221)
        return self.set('start', 1).wait(1).set('start', 0).expect(SWING).add('A', 1)

    def keepalive(self, clocks):
        for _ in range(clocks // 2000):
            self.wait(2000).control(2)
        return self.wait(clocks % 2000)


class SwingGuardV4RegressionTests(v4t.BalanceGuardRTLTests):
    TB = 'tb_swing_guard'
    SOURCES = SOURCES
    PARAMS = dict(VERSION=4, EXIT_MAX=57)


class SwingGuardTests(v4t.GuardHarness):
    TB = 'tb_swing_guard'
    SOURCES = SOURCES
    PARAMS = dict(VERSION=5, EXIT_MAX=90)

    def test_backlash_fields_bounds_latch_and_reserved_tail(self):
        v = SV().wait(45)
        for lash, boost, thr in ((0, 0, 0), (5, 100, 75), (15, 250, 250), (1, 1, 0)):
            v.command(lash_config(lash, boost, thr)).wait(34).expect(rx=1).add('L', lash, boost, thr)
        for lash, boost, thr in ((16, 0, 0), (0, 251, 0), (0, 0, 251), (0x100, 0, 0), (0, 0x8000, 0)):
            v.command(lash_config(lash, boost, thr)).wait(34).expect(loaded=0, rx=5)
        for offset in (174, 180, 187):
            v.command(lash_config(5, 100, 75, dirty=offset)).wait(34).expect(loaded=0, rx=5)
        v.command(lash_config(3, 50, 25)).wait(34).expect(rx=1).add('L', 3, 50, 25)
        self.simulate(v)

    def test_gains2_acceptance_bounds_and_clearing(self):
        v = SV().wait(45)
        v.command(gains2_block()).wait(34).expect(loaded=0, rx=4).add('X', 0, 0)
        v.load()
        for change in (dict(config_id=0x1235), dict(switch_ms=0), dict(switch_ms=30001), dict(version=4),
                       dict(dirty=54), dict(dirty=187)):
            v.command(gains2_block(**change)).wait(34).expect(rx=5).add('X', 0, 0)
        for index in (0, 5, 11):
            for bad in ((128 << 20) + 1, -(128 << 20) - 1):
                coeffs = [0] * 12
                coeffs[index] = bad
                v.command(gains2_block(coefficients=coeffs)).wait(34).expect(rx=5).add('X', 0, 0)
        edge = [(128 << 20) if i % 2 else -(128 << 20) for i in range(12)]
        v.command(gains2_block(switch_ms=30000, coefficients=edge)).wait(34).expect(rx=1).add('X', 1, 0)
        v.add('Y', gains2_block(switch_ms=30000, coefficients=edge)[6:54][::-1].hex())
        v.command(gains2_block()).wait(34).expect(rx=1).add('X', 1, 0)
        v.add('Y', gains2_block()[6:54][::-1].hex())
        # A second command while one is pending is refused; new CONFIG and ABORT clear the block.
        v.command(gains2_block()).command(gains2_block()).expect(rx=4).wait(34).expect(rx=1).add('X', 1, 0)
        v.load().add('X', 0, 0).command(gains2_block()).wait(34).add('X', 1, 0)
        v.control(3).expect(loaded=0, rx=3).add('X', 0, 0)
        v.load().command(gains2_block()).wait(3).command(swing_block()).expect(rx=4).wait(34).add('X', 1, 0)
        v.command(swing_block()).command(gains2_block()).expect(rx=4).wait(3).add('G', 1)
        v.set('raw', 1).set('stable', 1).wait(1).expect(ARMED).command(gains2_block()).expect(ARMED, rx=4)
        self.simulate(v)

    def test_arm_waits_for_pending_gains2(self):
        v = SV().wait(45).load().set('raw', 1).set('stable', 1).command(gains2_block()).expect(SAFE)
        for _ in range(12):
            v.wait(1).expect(SAFE)
        v.wait(3).expect(ARMED).add('X', 1, 0)
        self.simulate(v)

    def test_gains2_switches_after_switch_ms_in_balance_and_clears_on_stop(self):
        v = SV().wait(45).load(hold_ms=2).command(gains2_block(switch_ms=5)).wait(34).expect(rx=1)
        v.set('raw', 1).set('stable', 1).wait(1).expect(ARMED).set('start', 1).wait(1).set('start', 0)
        v.expect(CAPTURE).wait(90).expect(BALANCE).add('X', 1, 0)
        v.wait(200).expect(BALANCE).add('X', 1, 1)
        v.wait(200).add('X', 1, 1).set('stop', 1).wait(1).set('stop', 0).expect(SAFE, 2, 0).add('X', 0, 0)
        self.simulate(v)
        # Without a GAINS2 block the core keeps the CONFIG set; SWING never switches.
        v = SV().balance().wait(400).add('X', 0, 0)
        self.simulate(v)
        v = SV().wait(45).load().swing().command(gains2_block(switch_ms=1)).wait(34).expect(rx=1)
        v.set('raw', 1).set('stable', 1).wait(1).set('adc', 221).set('start', 1).wait(1).set('start', 0)
        v.expect(SWING).wait(100).add('X', 1, 0).set('catch', 1).set('adc', 500).add('J', 'swing').expect(BALANCE)
        v.set('catch', 0).wait(10).add('X', 1, 0).wait(30).add('X', 1, 1)
        self.simulate(v)

    def test_swing_block_requires_safe_loaded_matching_config_and_tables(self):
        v = SV().wait(45).swing().expect(loaded=0, rx=4).add('G', 0)
        v.load().swing(config_id=0x1235).expect(rx=5).add('G', 0)
        v.swing(table_crc=sm.TABLE_CRC ^ 1).expect(rx=5).add('G', 0)
        v.swing(version=4).expect(rx=5).add('G', 0)
        v.swing().expect(rx=1).add('G', 1).add('Z', swing_block()[:72][::-1].hex())
        v.load().expect(rx=1).add('G', 0).swing().add('G', 1)
        v.control(3).expect(loaded=0, rx=3).add('G', 0)
        v.load().command(v4t.config(version=5, config_id=7)).swing().expect(loaded=0, rx=4).add('G', 0)
        v.wait(34).expect(rx=1).add('G', 0)
        v.swing().expect(rx=5).add('G', 0).swing(config_id=7).expect(rx=1).add('G', 1)
        v.set('raw', 1).set('stable', 1).wait(1).expect(ARMED).swing().expect(ARMED, rx=4).add('G', 1)
        self.simulate(v)

    def test_every_swing_field_bound_and_cross_constraint(self):
        v = SV().wait(45).load()
        free = [f for f in sm.FIELDS if f[0] not in ('config_id', 'table_crc', 'hang_code', 'hang_window',
                                                      'edge_low', 'edge_high')]
        for name, code, lo, hi in free:
            for good in (lo, hi):
                v.swing(**{name: good}).expect(rx=1).add('G', 1)
            width = {'H': 16, 'h': 16, 'i': 32}[code]
            signed = code in 'hi'
            for bad in (lo - 1, hi + 1):
                if signed and not -(1 << width - 1) <= bad < (1 << width - 1):
                    continue
                if not signed and not 0 <= bad < (1 << width):
                    continue
                v.swing(**{name: bad}).expect(rx=5).add('G', 0)
        for change in (dict(edge_low=201), dict(edge_high=799), dict(edge_high=1024),
                       dict(hang_window=61), dict(edge_low=200, hang_window=21),
                       dict(edge_high=800, hang_code=785, hang_window=15),
                       dict(hang_code=15), dict(hang_code=10, edge_low=0),
                       dict(hang_code=1023, hang_window=0, edge_high=1023)):
            v.swing(**change).expect(rx=5).add('G', 0)
        for change in (dict(edge_low=200, hang_window=20), dict(edge_high=800, hang_code=785, hang_window=14),
                       dict(hang_code=16, edge_low=0), dict(hang_window=0)):
            v.swing(**change).expect(rx=1).add('G', 1)
        for offset in (74, 120, 187):
            block = swing_block()
            block[offset] = 1
            v.swing(block).expect(rx=5).add('G', 0)
        self.simulate(v)

    def test_start_requires_hang_window_and_sets_arm_reference(self):
        v = SV().set('position', 40).swing_armed().set('adc', 500)
        v.set('start', 1).wait(1).set('start', 0).expect(ARMED)
        v.set('adc', 205).set('start', 1).wait(1).set('start', 0).expect(ARMED)
        v.set('adc', 237).set('start', 1).wait(1).set('start', 0).expect(ARMED)
        v.set('adc', 206).set('position', 70).set('start', 1).wait(1).set('start', 0).expect(SWING)
        self.simulate(v.add('T', 70, 30).add('A', 1))
        v = SV().swing_armed().set('adc', 236).set('faults', 1).set('start', 1).wait(1)
        self.simulate(v.expect(FAULT, 16, 0))

    def test_manual_capture_without_swing_block(self):
        v = SV().wait(45).load().set('raw', 1).set('stable', 1).wait(1).expect(ARMED)
        self.simulate(v.set('start', 1).wait(1).set('start', 0).expect(CAPTURE))

    def test_swing_tolerates_adc_invalid_and_quality_only(self):
        v = SV().swinging().set('faults', 1).set('quality', 3).wait(200).expect(SWING)
        self.simulate(v.set('faults', 3).wait(1).expect(FAULT, 16, 0))
        for name, value in (('faults', 2), ('faults', 4), ('age', 101)):
            self.simulate(SV().swinging().set(name, value).wait(1).expect(FAULT, 16, 0))

    def test_blind_count_limit_entry_and_priority(self):
        v = SV().swinging(blind_samples=50).set('blind', 50).add('J', 'swing').expect(SWING)
        self.simulate(v.set('blind', 51).add('J', 'swing').expect(FAULT, 25, 0))
        v = SV().swinging(blind_samples=1).set('blind', 2).set('swing_on', 0).wait(50).expect(SWING)
        self.simulate(v.add('J', 'swing').expect(FAULT, 25, 0))
        self.simulate(SV().swinging().set('mid', 1).add('J', 'swing').expect(FAULT, 26, 0))
        v = SV().swinging().set('mid', 1).set('blind', 101).set('faults', 2)
        self.simulate(v.add('J', 'swing').expect(FAULT, 16, 0))
        v = SV().swinging().set('mid', 1).set('blind', 101).set('bad_tick', 1)
        self.simulate(v.add('J', 'swing').expect(FAULT, 26, 0))
        v = SV().swinging().set('blind', 101).set('bad_tick', 1)
        self.simulate(v.add('J', 'swing').expect(FAULT, 25, 0))

    def test_arm_stall_counts_consecutive_large_commands_without_motion(self):
        v = SV().swinging().set('swing_on', 0).set('command', 499)
        for _ in range(80):
            v.add('J', 'swing')
        v.expect(SWING).set('command', -500)
        for _ in range(75):
            v.add('J', 'swing')
        v.set('sv', 1).add('J', 'swing').set('sv', 0)
        for _ in range(76):
            v.add('J', 'swing')
        v.expect(SWING).wait(1).expect(FAULT, 27, 0)
        self.simulate(v)
        v = SV().swinging().set('swing_on', 0).set('command', -32768)
        for _ in range(76):
            v.add('J', 'swing')
        self.simulate(v.wait(1).expect(FAULT, 27, 0))
        # Only counted in SWING: a stalled command left over does not trip BALANCE.
        v = SV().swinging().set('command', 1000).set('catch', 1).set('adc', 500).add('J', 'swing')
        self.simulate(v.expect(BALANCE).set('catch', 0).wait(200).expect(BALANCE))

    def test_start_requires_arm_near_the_config_position(self):
        v = SV().set('position', 100).swing_armed().set('adc', 221).set('position', 133).wait(2)
        v.set('start', 1).wait(1).set('start', 0).expect(ARMED)
        v.set('position', 67).wait(2).set('start', 1).wait(1).set('start', 0).expect(ARMED)
        v.set('position', 68).wait(2).set('start', 1).wait(1).set('start', 0).expect(SWING)
        self.simulate(v)

    def test_failed_second_swing_block_restores_config_travel(self):
        v = SV().set('position', 1000).wait(45).load().swing().expect(rx=1).add('G', 1)
        v.set('position', 1150).wait(1).expect(loaded=1).set('position', 1000)
        v.swing(blind_samples=0).expect(rx=5).add('G', 0)
        v.set('raw', 1).set('stable', 1).wait(1).expect(ARMED)
        v.set('position', 1100).wait(1).expect(ARMED).set('position', 1101).wait(1)
        self.simulate(v.expect(FAULT, 18, 0))

    def test_swing_results_must_stay_fresh(self):
        v = SV().swinging().wait(300).expect(SWING).set('swing_on', 0).wait(60).expect(SWING)
        self.simulate(v.wait(110).expect(FAULT, 17, 0))

    def test_swing_arm_speed_both_signs_and_int_min(self):
        for sign in (-1, 1):
            v = SV().swinging().set('sv', sign * (5000 << 16)).add('J', 'swing').expect(SWING)
            self.simulate(v.set('sv', sign * ((5000 << 16) + 1)).add('J', 'swing').wait(1)
                          .expect(FAULT, 19, 0))
        self.simulate(SV().swinging().set('sv', -(1 << 31)).add('J', 'swing').wait(1)
                      .expect(FAULT, 19, 0))

    def test_swing_overrun_faults_only_in_swing(self):
        self.simulate(SV().swinging().set('swing_overrun', 1).wait(1).expect(FAULT, 23, 0))
        v = SV().swinging().set('catch', 1).set('adc', 500).add('J', 'swing').expect(BALANCE)
        self.simulate(v.set('swing_overrun', 1).wait(80).expect(BALANCE))

    def test_swing_timeout_returns_safe_and_consumes_both_blocks(self):
        v = SV().swinging(swing_ms=500).keepalive(9990).expect(SWING)
        self.simulate(v.wait(20).expect(SAFE, 7, 0).add('G', 0).add('A', 0))

    def test_catch_requires_quality_valid_adc_and_exit_window(self):
        v = SV().set('position', 10).swinging().set('position', 25).set('catch', 1).set('adc', 600)
        v.add('J', 'swing').expect(SWING).set('adc', 442).add('J', 'swing').expect(SWING)
        v.set('adc', 557).set('quality', 3).add('J', 'all').expect(SWING).set('quality', 7).add('J', 'est')
        v.set('quality', 7).set('faults', 1).add('J', 'swing').expect(SWING)
        v.set('faults', 0).set('catch', 0).add('J', 'swing').expect(SWING)
        v.set('catch', 1).add('J', 'swing').expect(BALANCE).add('T', 10, 15).add('A', 1)
        v.set('swing_on', 0).wait(80).expect(BALANCE).set('adc', 558).wait(1)
        self.simulate(v.expect(FAULT, 21, 0))
        v = SV().swinging(cfg=dict(exit=90)).set('catch', 1).set('adc', 590).add('J', 'swing')
        self.simulate(v.expect(BALANCE).set('core_on', 0).wait(101).expect(FAULT, 17, 0))

    def test_swing_travel_relatched_about_config_reference(self):
        v = SV().set('position', 1000).swinging(travel_count=260).add('T', 1000, 0)
        v.set('position', 1260).wait(1).expect(SWING).set('position', 740).wait(1).expect(SWING)
        self.simulate(v.set('position', 739).wait(1).expect(FAULT, 18, 0))
        v = SV().set('position', 1000).swing_armed(travel_count=1).set('position', 1001).wait(1)
        self.simulate(v.expect(ARMED).set('position', 1002).wait(1).expect(FAULT, 18, 0))
        v = SV().set('position', 1000).swinging(travel_count=130).set('catch', 1).set('adc', 500)
        v.set('position', 1130).add('J', 'swing').expect(BALANCE).wait(1)
        self.simulate(v.set('position', 1131).wait(1).expect(FAULT, 18, 0))

    def test_normal_stops_in_swing_and_after_catch(self):
        for caught in (False, True):
            for kind, reason in (('stop', 2), ('raw', 3), ('abort', 4)):
                v = SV().swinging()
                if caught:
                    v.set('catch', 1).set('adc', 500).add('J', 'swing').expect(BALANCE)
                if kind == 'abort':
                    v.control(3)
                else:
                    v.set(kind, 0 if kind == 'raw' else 1).add('A', 1).wait(1)
                self.simulate(v.expect(SAFE, reason, 0).add('G', 0).add('A', 0))

    def test_arm_waits_for_swing_commit_and_new_config_cancels_pending_swing(self):
        v = SV().wait(45).load().set('raw', 1).set('stable', 1).command(swing_block()).expect(SAFE)
        v.wait(1).expect(SAFE).wait(1).expect(SAFE).add('G', 1).wait(1).expect(ARMED)
        self.simulate(v)
        v = SV().wait(45).load().command(swing_block()).command(v4t.config(version=5))
        self.simulate(v.wait(40).expect(rx=1).add('G', 0))


TRAJ = dict(step_counts=260, move_speed=16384, rotate_speed=21845, accel=114, kv=1475, ka=14750, ki=1000,
            lean_arm=1173, lean_pend=1661, i_window=80)
TRAJ_ORDER = ('step_counts', 'move_speed', 'rotate_speed', 'accel', 'kv', 'ka', 'ki', 'lean_arm', 'lean_pend',
              'i_window')


def v6_config(traj=None, dirty=None, **fields):
    data = v4t.config(**dict(dict(version=6), **fields))
    t = dict(TRAJ, **(traj or {}))
    struct.pack_into('<10H', data, 168, *(t[k] for k in TRAJ_ORDER))
    if dirty is not None:
        data[dirty] = 1
    return data


def traj_hex(config_bytes):
    return bytes(config_bytes[168:188])[::-1].hex()


class SV6(SV):
    def __init__(self):
        v4t.Vectors.__init__(self, version=6)
        self.set('table', sm.TABLE_CRC)

    def load(self, **fields):
        traj = fields.pop('traj', None)
        return self.command(v6_config(traj, **fields)).wait(34)

    def builtin(self, cfg=None, swing=None, crc=0xABCD):
        cfg = cfg if cfg is not None else v6_config(config_id=0x6001, run_ms=0, keepalive_ms=10)
        swing = swing if swing is not None else swing_block(version=6, config_id=0x6001)
        return self.add('B', bytes(cfg)[::-1].hex(), bytes(swing)[::-1].hex(), f'{crc:04x}')

    def dip(self, on):
        return self.set('raw', int(on)).set('stable', int(on))


class SwingGuardV6Tests(v4t.GuardHarness):
    TB = 'tb_swing_guard'
    SOURCES = SOURCES
    PARAMS = dict(VERSION=6, EXIT_MAX=90)

    def test_traj_fields_bounds_latch_and_run_zero(self):
        v = SV6().wait(45)
        v.command(v6_config()).wait(34).expect(rx=1).add('V', traj_hex(v6_config()), 0)
        edge = dict(step_counts=1040, move_speed=4095, rotate_speed=4095, accel=4095, kv=65535, ka=65535,
                    ki=65535, lean_arm=16383, lean_pend=16383, i_window=1040)
        v.command(v6_config(edge)).wait(34).expect(rx=1).add('V', traj_hex(v6_config(edge)), 0)
        off = dict(step_counts=0, move_speed=0, rotate_speed=0, accel=0)
        v.command(v6_config(off, run_ms=0)).wait(34).expect(rx=1).add('V', traj_hex(v6_config(off)), 0)
        for bad in (dict(step_counts=1041), dict(accel=4096, move_speed=5000), dict(accel=0),
                    dict(move_speed=113), dict(rotate_speed=113), dict(lean_arm=16384), dict(lean_pend=16384),
                    dict(i_window=1041), dict(step_counts=0, accel=4096)):
            v.command(v6_config(bad)).wait(34).expect(loaded=0, rx=5)
        v.command(v6_config(run_ms=30001)).wait(34).expect(loaded=0, rx=5)
        for preset in (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14):
            v.command(v6_config(algorithm_id=preset)).wait(34).expect(rx=1)
        for bad in (0, 15):
            v.command(v6_config(algorithm_id=bad)).wait(34).expect(loaded=0, rx=5)
        self.simulate(v)

    def test_run_zero_never_completes_but_a_run_limit_still_does(self):
        v = SV6().balance(run_ms=0).keepalive(60000).expect(BALANCE)
        self.simulate(v.set('stop', 1).wait(1).expect(SAFE, 2, 0))
        self.simulate(SV6().balance(run_ms=5).keepalive(4000).expect(SAFE, 1, 0))

    def test_balance_travel_follows_the_reference_delta(self):
        v = SV6().set('position', 1000).balance(travel=100)
        v.set('position', 9000).set('track', 100).wait(5).expect(BALANCE)
        v.set('track', -100).wait(5).expect(BALANCE)
        self.simulate(v.set('track', 101).wait(1).expect(FAULT, 18, 0))
        self.simulate(SV6().balance(travel=100).set('track', -101).wait(1).expect(FAULT, 18, 0))
        # Outside BALANCE the absolute CONFIG-time bound still applies and the delta is ignored.
        v = SV6().set('position', 1000).armed(travel=100).set('track', 5000).wait(3).expect(ARMED)
        self.simulate(v.set('position', 1101).wait(1).expect(FAULT, 18, 0))

    def test_standalone_loads_builtin_and_skips_keepalive(self):
        cfg = v6_config(config_id=0x6001, run_ms=0, keepalive_ms=10)
        v = SV6().builtin(cfg).wait(45).dip(1).wait(60).expect(ARMED, rx=1).add('G', 1)
        v.add('V', traj_hex(cfg), 1).add('P', 0x6001, 0xABCD, 1000, 175, 25, 7 << 20, 500 << 16)
        v.wait(2000).expect(ARMED)
        v.set('adc', 221).set('start', 1).wait(1).set('start', 0).expect(SWING)
        v.set('catch', 1).set('adc', 500).add('J', 'swing').expect(BALANCE).set('catch', 0)
        v.wait(4000).expect(BALANCE).add('V', traj_hex(cfg), 1)
        v.set('stop', 1).wait(1).set('stop', 0).expect(SAFE, 2, 0).add('V', traj_hex(cfg), 0)
        # DIP still on: no reload until it has been switched off.
        v.wait(100).expect(SAFE, 2, 0)
        v.dip(0).wait(3).dip(1).wait(60).expect(ARMED, 2, rx=1).add('V', traj_hex(cfg), 1)
        self.simulate(v)

    def test_bad_builtin_is_tried_once_per_dip_on(self):
        v = SV6().builtin(v6_config(config_id=0)).wait(45).dip(1).wait(60).expect(SAFE, loaded=0, rx=5)
        v.add('V', '0' * 40, 0).builtin().wait(200).expect(SAFE, loaded=0, rx=5)
        v.dip(0).wait(3).dip(1).wait(60).expect(ARMED, rx=1).add('V', traj_hex(v6_config(config_id=0x6001,
                                                                                    run_ms=0, keepalive_ms=10)), 1)
        self.simulate(v)
        # A builtin SWING block that fails its checks drops the builtin CONFIG too.
        bad_swing = swing_block(version=6, config_id=0x6001, table_crc=sm.TABLE_CRC ^ 1)
        v = SV6().builtin(swing=bad_swing).wait(45).dip(1).wait(60).expect(SAFE, loaded=0, rx=5).add('G', 0)
        self.simulate(v.add('V', traj_hex(v6_config(config_id=0x6001, run_ms=0, keepalive_ms=10)), 0))

    def test_host_config_wins_over_a_pending_builtin_and_abort_stops_standalone(self):
        v = SV6().builtin().wait(45).dip(1).wait(5).command(v6_config(config_id=0x2222)).wait(34)
        v.expect(ARMED, rx=1).add('G', 0).add('P', 0x2222, 0xBEEF, 1000, 175, 25, 7 << 20, 500 << 16)
        self.simulate(v.add('V', traj_hex(v6_config()), 0))
        v = SV6().builtin().wait(45).dip(1).wait(60).expect(ARMED, rx=1).control(3)
        self.simulate(v.expect(SAFE, 4, 0, rx=3).add('V', traj_hex(v6_config(config_id=0x6001, run_ms=0,
                                                                              keepalive_ms=10)), 0))

    def test_real_builtin_frames_pass_the_rtl_checks(self):
        cfg, swing, crc = builtin.frames()
        v = SV6().builtin(cfg, swing, crc).wait(45).dip(1).wait(60).expect(ARMED, rx=1).add('G', 1)
        v.add('V', traj_hex(cfg), 1).add('P', builtin.CONFIG_ID, crc, 1000, 175, 62, 2959690, 48300032)
        self.simulate(v.wait(3000).expect(ARMED))

    def test_lost_builtin_frame_times_out_and_waits_for_dip_off(self):
        cfg = v6_config(config_id=0x6001, run_ms=0, keepalive_ms=10)
        v = SV6().builtin(cfg).set('inject', 0).wait(45).dip(1).wait(60).expect(SAFE, loaded=0)
        v.add('V', '0' * 40, 1).wait(2000).expect(SAFE, loaded=0).add('V', '0' * 40, 0)
        v.set('inject', 1).wait(200).expect(SAFE, loaded=0).add('V', '0' * 40, 0)
        v.dip(0).wait(3).dip(1).wait(60).expect(ARMED, rx=1).add('G', 1).add('V', traj_hex(cfg), 1)
        self.simulate(v)

    def test_host_session_still_needs_keepalive(self):
        v = SV6().builtin().balance(run_ms=0)
        self.simulate(v.wait(12000).expect(FAULT, 20, 0))


if __name__ == '__main__':
    unittest.main()
