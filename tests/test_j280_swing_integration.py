"""True UART pins and B PWM pins for the v6 swing top, with source provenance."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import unittest

from test_j280_motor_test_rtl import simulate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_balance_math as bm
import j280_builtin as builtin
import j280_v6_protocol as wire
from j280_measurement import crc16_ccitt

# Fast trajectory for the bench: 8-count steps at one count per sample.
TRAJ = dict(step_counts=8, move_speed=65535, rotate_speed=65535, accel=4095, kv=1475, ka=14750,
            lean_arm=1177, lean_pend=1667)
GAINS2 = dict(config_id=0x1234, switch_ms=4,
              coefficients=bm.design_coefficients((300., 200., 20., 5.), 0.5, (5., 3.))[20:32])

SOURCES = [ROOT / 'rtl/j280_measure' / n for n in ('j280_adc_capture.v', 'j280_uart_tx.v')]
SOURCES += [ROOT / 'rtl/j280_state' / n for n in ('j280_raw_estimator.v', 'j280_signed_divider.v',
                                                   'j280_state_packet_tx.v')]
SOURCES += [ROOT / 'rtl/j280_motor_test' / n for n in ('j280_uart_rx.v', 'j280_local_inputs.v')]
SOURCES += [ROOT / 'rtl/j280_balance' / n for n in ('j280_encoder_filtered.v', 'j280_balance_cmd_rx.v',
                                                     'j280_balance_core.v', 'j280_balance_pwm.v')]
SOURCES += [ROOT / 'rtl/j280_swing' / n for n in ('j280_swing_tables.v', 'j280_swing_core.v', 'j280_traj.v',
                                                   'j280_aux_inputs.v', 'j280_builtin.v',
                                                   'j280_swing_guard.v', 'j280_swing_top.v')]
SOURCES += [ROOT / 'sim/j280_swing/tb_swing_top.v']
SCENARIOS = {0: ('stop', 'SAFE', 'stop_key'), 1: ('dead_zone', 'FAULT', 'angle_limit'),
             2: ('blind_entry', 'FAULT', 'blind_entry'), 3: ('standalone', 'SAFE', 'stop_key')}
EVIDENCE_FILES = ('commands.hex', 'wire.bin', 'pins.csv', 'snapshots.csv', 'events.csv',
                  'result.json', 'compile.log', 'simulation.log')


def fixture_commands():
    data = ROOT / 'docs/data'
    values = json.loads((data / 'balance-board-2026-10-17/balance-511-config.json')
                        .read_text(encoding='utf-8'))['values']
    values.update(config_id=0x1234, exit_code=80)
    good = wire.build_command('config', values, traj=TRAJ)
    bad = bytearray(good)
    bad[164:168] = (129 << 20).to_bytes(4, 'little', signed=True)
    bad[190:] = crc16_ccitt(bad[2:190]).to_bytes(2, 'little')
    swing = json.loads((data / 'balance-design-2026-10-17/swingup-fixed-01.json')
                       .read_text(encoding='utf-8'))['swing_values']
    return [bytes(bad), good, wire.build_command('swing', dict(swing, config_id=0x1234)),
            wire.build_command('gains2', GAINS2), wire.build_command('keepalive')]


class SwingIntegrationTests(unittest.TestCase):
    def scenario(self, case):
        name, final_state, final_reason = SCENARIOS[case]
        packets = fixture_commands()
        commands = '\n'.join(p[::-1].hex() for p in packets) + '\n'
        _, run = simulate('tb_swing_top', SOURCES, {'commands.hex': commands},
                          plusargs=(f'+CASE={case}',), timeout=3600)
        data = (run / 'wire.bin').read_bytes()
        stream = wire.FrameStream()
        rows = stream.feed(data)
        self.assertTrue(rows)
        for key in ('frames_invalid', 'discarded_bytes', 'incomplete_tail_bytes', 'packet_seq_missed',
                    'duplicates', 'out_of_order', 'resets', 'reset_or_reorder', 'continuity_errors'):
            self.assertEqual(stream.stats[key], 0, (key, stream.stats))
        self.assertEqual([r['packet_seq'] for r in rows], list(range(len(rows))))
        self.assertEqual([r['tick_us'] for r in rows], [2000 + 10000 * i for i in range(len(rows))])
        states = [r['state'] for r in rows]
        self.assertIn('SWING', states)
        self.assertNotIn('CAPTURE', states)
        _, _, builtin_crc = builtin.frames()
        expected_id = (builtin.CONFIG_ID, builtin_crc) if case == 3 else (0x1234, int.from_bytes(packets[1][190:], 'little'))
        if case != 3:
            self.assertTrue(any(r['rx_status'] & 15 == 5 and not r['config_loaded'] for r in rows))
            self.assertTrue(any(r['gains2_loaded'] for r in rows if r['state'] == 'SWING'))
        for r in rows:
            if r['config_loaded']:
                self.assertEqual((r['config_id'], r['config_crc']), expected_id)
                self.assertEqual(r['standalone'], case == 3)
            if r['state'] == 'SWING':
                self.assertTrue(r['swing_loaded'])
            if r['state'] != 'BALANCE':
                self.assertFalse(r['traj_moving'] or r['traj_rotating'] or r['u_ff_q16'] or r['ref_velocity_q16'])
        kick = [r for r in rows if r['state'] == 'SWING' and r['swing_command'] == 1000]
        self.assertTrue(kick and all(r['motor_mode'] == 2 for r in kick[1:]), kick[:3])
        if case == 1:
            self.assertTrue(any(r['state'] == 'SWING' and not r['adc_valid'] for r in rows))
            self.assertTrue(any(r['state'] == 'SWING' and r['swing_blind'] > 0 for r in rows))
        if case in (0, 1, 3):
            self.assertIn('BALANCE', states)
            self.assertLess(states.index('SWING'), states.index('BALANCE'))
        balance = [r for r in rows if r['state'] == 'BALANCE']
        if case in (0, 1):
            self.assertTrue(any(r['gains2_active'] for r in balance))
        if case == 0:
            # The bench drives DIP 2 on while turning: the telemetry shows it.
            self.assertTrue(any(r['dip2_on'] for r in balance))
            anchor = balance[0]['theta_ref']
            self.assertTrue(any(r['traj_moving'] for r in balance))
            self.assertTrue(any(r['traj_rotating'] for r in balance))
            self.assertTrue(any(r['u_ff_q16'] for r in balance))
            self.assertTrue(any(r['theta_ref'] == anchor + 8 for r in balance))
            self.assertTrue(any(r['theta_ref'] == anchor - 8 for r in balance))
            self.assertTrue(min(r['theta_ref'] for r in balance) < anchor - 16)
            settled = [r for r in balance if r['traj_settled']]
            self.assertTrue(settled and not any(r['traj_moving'] for r in balance if not r['traj_settled']))
            last = balance[-1]
            self.assertFalse(last['traj_moving'] or last['traj_rotating'])
            self.assertEqual((last['theta_ref'] - anchor) % 8, 0)
        if case == 3:
            self.assertGreater(max(r['state_elapsed_ms'] for r in balance), 40)
            self.assertTrue(all(r['standalone'] for r in balance))
        events = (run / 'events.csv').read_text(encoding='ascii')
        self.assertEqual(',catch,' in events, case in (0, 1, 3))
        last = rows[-1]
        self.assertEqual((last['state'], last['stop_reason']), (final_state, final_reason))
        self.assertFalse(last['output_enabled'] or last['config_loaded'] or last['swing_loaded'])
        with (run / 'pins.csv').open(encoding='ascii', newline='') as source:
            pinrows = [tuple(map(int, x)) for x in csv.reader(source)]
        self.assertTrue(any(in2 for _, pwm, in1, in2 in pinrows))
        with (run / 'snapshots.csv').open(encoding='ascii', newline='') as source:
            snapshotrows = [tuple(map(int, x)) for x in csv.reader(source)]
        self.assertEqual([x[2] for x in snapshotrows], list(range(len(snapshotrows))))
        self.assertTrue(all(b[1] - a[1] == 2000 for a, b in zip(snapshotrows, snapshotrows[1:])))
        result = dict(case=case, scenario=name, frames=len(rows), stats=stream.stats,
                      final=[final_state, final_reason], wire_sha256=hashlib.sha256(data).hexdigest(),
                      source_sha256={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in SOURCES})
        (run / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        evidence = os.environ.get('J280_SWING_EVIDENCE_DIR')
        if evidence:
            target = Path(evidence) / name
            target.mkdir(parents=True, exist_ok=True)
            for file in EVIDENCE_FILES:
                shutil.copy2(run / file, target / file)

    def test_swing_up_catch_balance_steps_turn_and_local_stop(self):
        self.scenario(0)

    def test_dead_zone_crossing_then_catch_then_angle_fault(self):
        self.scenario(1)

    def test_mid_range_adc_loss_is_a_blind_entry_fault(self):
        self.scenario(2)

    def test_standalone_builtin_swing_balance_without_host(self):
        self.scenario(3)


if __name__ == '__main__':
    unittest.main()
