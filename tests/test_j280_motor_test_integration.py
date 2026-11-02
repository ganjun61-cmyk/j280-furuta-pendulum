"""Motor-test v3 through physical UART and motor pins; never opens hardware."""
import csv
import json
import os
from pathlib import Path
import shutil
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import j280_motor_test as protocol
from test_j280_motor_test_rtl import simulate

SOURCES = ([ROOT / 'rtl/j280_measure' / n for n in
            ('j280_adc_capture.v', 'j280_encoder.v', 'j280_uart_tx.v')]
           + [ROOT / 'rtl/j280_state' / n for n in
              ('j280_signed_divider.v', 'j280_raw_estimator.v', 'j280_state_packet_tx.v')]
           + sorted((ROOT / 'rtl/j280_motor_test').glob('*.v'))
           + [ROOT / 'sim/j280_motor_test/tb_motor_test_top.v'])


def scenario(name):
    values = dict(config_id=0x1234, direction='in1_high', duty_count=125,
                  pulse_ms=4, travel_limit_count=60, speed_limit_count_s=3000,
                  keepalive_timeout_ms=500)
    long_values = dict(values, config_id=0x1235, direction='in2_high', pulse_ms=30)
    commands = [protocol.build_command('config', values),
                protocol.build_command('config', long_values),
                protocol.build_command('keepalive'), protocol.build_command('abort')]
    if name == 'default':
        # Real 20 ms debounce, 20 kHz PWM, 230400 baud, 500 Hz acquisition.
        script = 'W 25000000\nC 0\nW 3000000\nD 0\nW 23000000\nS1 0\nW 21000000\nS1 1\nW 10000000\nE\n'
        params = {}
    else:
        # Only debounce is reduced to 100 us; UART, ADC, PWM and guard clocks
        # remain their physical rates. Exercise reuse, local/host stops, fault.
        script = ('W 15000000\nC 0\nW 2500000\nD 0\nW 2500000\n'
                  'S1 0\nW 7000000\nS1 1\nW 500000\nD 1\nW 500000\n'
                  'C 1\nD 0\nW 2500000\nS1 0\nW 3500000\nS2 0\nW 10000\n'
                  'S1 1\nS2 1\nD 1\nW 2500000\n'
                  'C 1\nD 0\nW 2500000\nS1 0\nW 3500000\nD 1\n'
                  'S1 1\nW 2500000\n'
                  'C 1\nD 0\nW 2500000\nS1 0\nW 3500000\nC 3\n'
                  'S1 1\nD 1\nW 2500000\n'
                  'C 1\nD 0\nW 2500000\nS1 0\nW 3500000\nO 1\n'
                  'W 5000000\nO 0\nS1 1\nW 15000000\nE\n')
        params = {'DEBOUNCE_CLOCKS': 5000}
    return commands, script, params


def run_scenario(name):
    commands, script, params = scenario(name)
    for source in SOURCES:
        if not source.is_file():
            raise AssertionError(f'missing integration source: {source.relative_to(ROOT)}')
    _, run = simulate('tb_motor_test_top', SOURCES,
                      {'commands.hex': '\n'.join(c[::-1].hex() for c in commands) + '\n',
                       'scenario.txt': script}, params, timeout=300)
    stream = protocol.FrameStream()
    wire = (run / 'wire.bin').read_bytes()
    rows = stream.feed(wire)
    if not rows or stream.buffered_bytes or any(stream.stats[k] for k in
            ('frames_invalid', 'discarded_bytes', 'seq_missed', 'seq_duplicates',
             'seq_out_of_order', 'resets', 'continuity_errors')):
        raise AssertionError(f'unclean UART: {stream.stats}')
    snapshots = list(csv.reader((run / 'snapshots.csv').read_text().splitlines()))
    expected_fields = ('seq', 'tick_us', 'adc_raw10', 'enc_count', 'pwm_count',
                       'motor_mode', 'state_code', 'config_id', 'config_crc')
    if len(rows) != len(snapshots):
        raise AssertionError('snapshot/UART frame count differs')
    for row, expected in zip(rows, snapshots):
        actual = [int(row[key]) for key in expected_fields]
        if actual != [int(v) for v in expected]:
            raise AssertionError(f'non-atomic snapshot seq {row["seq"]}: {actual} != {expected}')
    trace = [[int(v) for v in line.split()] for line in (run / 'pins.txt').read_text().splitlines()]
    result = {'scenario': name, 'frames': len(rows), 'wire_stats': stream.stats,
              'states': list(dict.fromkeys(r['state'] for r in rows)),
              'stop_reasons': list(dict.fromkeys(r['stop_reason'] for r in rows))}
    evidence = os.environ.get('J280_MOTOR_TEST_EVIDENCE_DIR')
    if evidence:
        dest = Path(evidence) / name
        dest.mkdir(parents=True, exist_ok=True)
        for file in ('wire.bin', 'commands.hex', 'scenario.txt', 'snapshots.csv',
                     'pins.txt', 'compile.log', 'simulation.log'):
            shutil.copy2(run / file, dest / file)
        (dest / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return rows, trace


class MotorTopIntegrationTests(unittest.TestCase):
    def test_default_timing_local_start_and_exact_pulse(self):
        rows, trace = run_scenario('default')
        self.assertEqual([r['seq'] for r in rows], list(range(len(rows))))
        self.assertEqual([r['tick_us'] for r in rows], [2000 * (i + 1) for i in range(len(rows))])
        self.assertTrue(any(r['state'] == 'ARMED' for r in rows))
        self.assertTrue(any(r['output_enabled'] and r['in1'] and not r['in2'] for r in rows))
        self.assertEqual((rows[-1]['state'], rows[-1]['stop_reason'], rows[-1]['config_loaded']),
                         ('SAFE', 'complete', False))
        # Trace columns: cycle, PWM, IN1, IN2. Verify actual output pins.
        starts = [r[0] for prev, r in zip(trace, trace[1:]) if not prev[2] and r[2]]
        ends = [r[0] for prev, r in zip(trace, trace[1:]) if prev[2] and not r[2]]
        self.assertEqual(len(starts), 1)
        self.assertEqual(ends[0] - starts[0], 4 * 20 * 2500)
        highs = []
        start = None
        for cycle, pwm, in1, in2 in trace:
            high = pwm and in1
            if high and start is None:
                start = cycle
            elif not high and start is not None:
                highs.append(cycle - start)
                start = None
        self.assertEqual(highs, [125] * 80)
        self.assertTrue(all(not in2 for _, _, _, in2 in trace))

    def test_stops_reconfiguration_and_adc_fault_latch(self):
        rows, _ = run_scenario('stops')
        reasons = {r['stop_reason'] for r in rows}
        self.assertTrue({'complete', 'stop_key', 'arm_off', 'host_abort', 'sensor_invalid'} <= reasons)
        self.assertTrue(any(r['output_enabled'] and r['in2'] for r in rows))
        faults = [i for i, r in enumerate(rows) if r['state'] == 'FAULT']
        self.assertTrue(faults)
        self.assertTrue(all(r['state'] == 'FAULT' and not r['output_enabled'] for r in rows[faults[0]:]))
        self.assertTrue(any(r['adc_valid'] for r in rows[faults[0] + 3:]))
        self.assertEqual(rows[-1]['stop_reason'], 'sensor_invalid')


if __name__ == '__main__':
    unittest.main()
