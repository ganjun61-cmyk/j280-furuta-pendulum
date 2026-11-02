"""Motor-test RTL unit simulations (Icarus); no board or serial access."""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import j280_motor_test as tool  # noqa: E402

RTL = ROOT/'rtl/j280_motor_test'
SIM = ROOT/'sim/j280_motor_test'
LOCAL = Path(os.environ.get('LOCALAPPDATA', str(Path.home())))


def _iverilog_bin():
    candidates = [Path(os.environ[k]) for k in ('J280_IVERILOG_BIN', 'IVERILOG_BIN')
                  if k in os.environ]
    candidates.append(LOCAL/'J280Tools/iverilog-msys2/ucrt64/bin')
    found = shutil.which('iverilog')
    if found:
        candidates.append(Path(found).parent)
    return next((p for p in candidates if (p/'iverilog.exe').is_file()), candidates[0])


BIN = _iverilog_bin()
BUILD = LOCAL/'J280Build/motor-test-unit'
CONFIG = dict(config_id=0x1234, direction='in2_high', duty_count=250, pulse_ms=300,
              travel_limit_count=60, speed_limit_count_s=3000, keepalive_timeout_ms=200)


def simulate(top, sources, files=None, params=None, plusargs=(), timeout=300):
    """Compile and run one testbench in a fresh ASCII directory; return (stdout, dir)."""
    BUILD.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix=top+'-', dir=BUILD))
    if not str(run).isascii():
        raise AssertionError(run)
    for source in sources:
        shutil.copy2(source, run/source.name)
    for name, text in (files or {}).items():
        (run/name).write_text(text, encoding='ascii')
    env = dict(os.environ, PATH=str(BIN)+os.pathsep+os.environ.get('PATH', ''))
    command = [str(BIN/'iverilog.exe'), '-g2001', '-Wall', '-s', top,
               *[f'-P{top}.{k}={v}' for k, v in (params or {}).items()],
               '-o', 'sim.vvp', *[s.name for s in sources]]
    compiled = subprocess.run(command, cwd=run, env=env, capture_output=True, text=True, timeout=60)
    (run/'compile.log').write_text(compiled.stdout+compiled.stderr, encoding='utf-8')
    if compiled.returncode:
        raise AssertionError(compiled.stdout+compiled.stderr)
    result = subprocess.run([str(BIN/'vvp.exe'), 'sim.vvp', *plusargs], cwd=run, env=env,
                            capture_output=True, text=True, timeout=timeout)
    text = result.stdout+result.stderr
    (run/'simulation.log').write_text(text, encoding='utf-8')
    if result.returncode or 'FAIL' in text or 'PASS '+top not in text:
        raise AssertionError(f'{run}\n{text[-4000:]}')
    return text, run


def rx_script(*items):
    """items: ('B'|'F', bytes, idle_ns) or ('G', low_ns, idle_ns)."""
    lines = []
    for item in items:
        if item[0] in 'BF':
            for n, value in enumerate(item[1]):
                lines.append(f'{item[0]} {value:02x} {item[2] if n == 0 else 0}')
        else:
            lines.append(f'G {item[1]} {item[2]}')
    return '\n'.join(lines+['E'])+'\n'


class CommandReceiveTests(unittest.TestCase):
    sources = [RTL/'j280_uart_rx.v', RTL/'j280_cmd_rx.v', SIM/'tb_cmd_rx.v']

    def run_rx(self, script, **params):
        _, run = simulate('tb_cmd_rx', self.sources, {'rx_script.txt': script}, params)
        log = (run/'rx_log.txt').read_text().split('\n')
        frames = [(bytes.fromhex(line.split()[1])[::-1], int(line.split()[2], 16))
                  for line in log if line.startswith('V ')]
        return dict(frames=frames, frame_errors=log.count('X'), byte_errors=log.count('R'),
                    bytes=bytes(int(line[2:], 16) for line in log if line.startswith('D ')))

    def test_python_frames_decode_exactly_back_to_back(self):
        config = tool.build_command('config', CONFIG)
        stream = config + tool.build_command('keepalive') + tool.build_command('abort')
        out = self.run_rx(rx_script(('B', stream, 1000)))
        self.assertEqual(out['bytes'], stream)
        self.assertEqual(out['frame_errors'], 0)
        self.assertEqual([f[0] for f in out['frames']],
                         [stream[2:18], stream[22:38], stream[42:58]])
        self.assertEqual(out['frames'][0][1], struct.unpack('<H', config[18:])[0])

    def test_bad_crc_version_and_noise_then_recovery(self):
        good = tool.build_command('keepalive')
        bad_crc = bytearray(good); bad_crc[19] ^= 1
        wrong_version = tool.command_frame(bytes([2, 2])+bytes(14))
        stream = b'\x00\xa5\x11' + bytes(bad_crc) + wrong_version + b'\xa5' + good
        out = self.run_rx(rx_script(('B', stream, 1000)))
        self.assertEqual(out['frames'], [(good[2:18], struct.unpack('<H', good[18:])[0])])
        self.assertEqual(out['frame_errors'], 2)

    def test_interbyte_gap_and_bad_stop_bit_drop_frame(self):
        good = tool.build_command('abort')
        out = self.run_rx(rx_script(('B', good[:9], 1000), ('B', good[9:], 1_100_000),
                                    ('B', good[:5], 50000), ('F', b'\x00', 0), ('B', good[5:], 0),
                                    ('B', good, 50000)))
        self.assertEqual(len(out['frames']), 1)
        self.assertEqual(out['byte_errors'], 1)
        self.assertGreaterEqual(out['frame_errors'], 2)

    def test_short_glitch_ignored_and_baud_tolerance(self):
        good = tool.build_command('config', CONFIG)
        out = self.run_rx(rx_script(('G', 1500, 1000), ('B', good, 20000)))
        self.assertEqual(out['bytes'], good)
        for bit_ns in (4253, 4427):
            with self.subTest(bit_ns=bit_ns):
                out = self.run_rx(rx_script(('B', good, 1000)), BIT_NS=bit_ns)
                self.assertEqual(len(out['frames']), 1)
                self.assertEqual(out['frame_errors'], 0)


CORE_SOURCES = [RTL/'j280_local_inputs.v', RTL/'j280_motor_guard.v', RTL/'j280_pwm_pulse.v',
                SIM/'tb_motor_core.v']
CORE_FIELDS = ('cycle', 'state', 'reason', 'rx', 'loaded', 'id', 'crc', 'pwm', 'in1', 'in2',
               'driving', 'cmp', 'allow', 'fresh', 'ok', 'arm_stable')
MS = 200          # tb_motor_core CLOCKS_PER_MS
PERIOD = 20       # tb_motor_core PWM_PERIOD
SAFE, ARMED, RUN, FAULT = 0, 1, 2, 255


class Core:
    """Script builder for tb_motor_core (DIP off = 1, keys released = 1)."""

    def __init__(self):
        self.lines = []

    def add(self, *items):
        self.lines.append(' '.join(str(i) for i in items))
        return self

    def wait(self, clocks, keepalive=1000):
        while clocks > 0:
            step = clocks if keepalive is None else min(clocks, keepalive)
            self.add('W', step)
            clocks -= step
            if keepalive is not None and clocks > 0:
                self.add('M', 2)
        return self

    def config(self, cid=0x1234, direction=2, reserved=0, duty=5, pulse=3, travel=60,
               speed=3000, keep=20, crc=0xBEEF):
        return self.add('C', cid, direction, reserved, duty, pulse, travel, speed, keep, crc)

    def armed(self, **config):
        return self.wait(500).config(**config).add('D', 0).wait(60)

    def press(self, key, clocks=40):
        return self.add(key, 0).wait(clocks).add(key, 1).wait(10)


def run_core(core, **params):
    script = '\n'.join(core.lines + ['E']) + '\n'
    _, run = simulate('tb_motor_core', CORE_SOURCES, {'core_script.txt': script}, params)
    rows, inputs = [], []
    for line in (run/'core_log.txt').read_text().splitlines():
        f = line.split()
        if f[0] == 'I':
            inputs.append((int(f[1]), f[2], int(f[3])))
        else:
            rows.append(dict(zip(CORE_FIELDS, map(int, f))))
    return rows, inputs


def transitions(rows):
    out = []
    for r in rows:
        key = (r['state'], r['reason'])
        if not out or out[-1] != key:
            out.append(key)
    return out


def first(rows, after=0, **match):
    return next(r for r in rows if r['cycle'] >= after and all(r[k] == v for k, v in match.items()))


def high_intervals(rows):
    """(start, width) of PWM-high intervals while driving, from the change log."""
    out, start = [], None
    for r in rows:
        high = r['driving'] and r['pwm']
        if high and start is None:
            start = r['cycle']
        elif not high and start is not None:
            out.append((start, r['cycle'] - start))
            start = None
    return out


class GuardCoreTests(unittest.TestCase):
    def test_nonzero_keepalive_and_abort_fields_are_rejected(self):
        for command in (2, 3):
            with self.subTest(command=command):
                payload = int.from_bytes(bytes([3, command, 1]) + bytes(13), 'little')
                rows, _ = run_core(Core().armed().add('F', f'{payload:032x}', '1234').wait(20))
                self.assertEqual(rows[-1]['state'], ARMED)
                self.assertEqual(rows[-1]['rx'] & 15, 5)
                self.assertEqual(rows[-1]['id'], 0x1234)

    def test_invalid_config_on_arm_edge_never_arms_empty_configuration(self):
        for delay in range(18, 26):
            with self.subTest(delay=delay):
                core = Core().wait(500).config().add('D', 0).wait(delay).config(cid=0).wait(60)
                rows, _ = run_core(core)
                self.assertFalse(any(r['state'] == ARMED and not r['loaded'] for r in rows))

    def test_complete_pulse_exact_shape_and_single_use(self):
        rows, _ = run_core(Core().armed().press('S1').wait(5 * MS))
        self.assertEqual(transitions(rows)[-4:], [(SAFE, 0), (ARMED, 0), (RUN, 0), (SAFE, 1)])
        run_row = first(rows, state=RUN)
        on = first(rows, driving=1)
        off = first(rows, after=on['cycle'], driving=0)
        self.assertLessEqual(on['cycle'] - run_row['cycle'], PERIOD + 2)
        self.assertEqual(off['cycle'] - on['cycle'], 3 * 10 * PERIOD)
        highs = high_intervals(rows)
        self.assertEqual(len(highs), 30)
        self.assertTrue(all(w == 5 for _, w in highs))
        self.assertTrue(all(b[0] - a[0] == PERIOD for a, b in zip(highs, highs[1:])))
        driving = [r for r in rows if r['driving']]
        self.assertTrue(all(r['in2'] and not r['in1'] and r['cmp'] == 5 for r in driving))
        end = rows[-1]
        self.assertEqual((end['state'], end['loaded'], end['id'], end['crc']), (SAFE, 0, 0, 0))

    def test_direction_one_drives_in1(self):
        rows, _ = run_core(Core().armed(direction=1, duty=1, pulse=1).press('S1').wait(3 * MS))
        driving = [r for r in rows if r['driving']]
        self.assertTrue(driving and all(r['in1'] and not r['in2'] for r in driving))
        self.assertEqual(len(high_intervals(rows)), 10)
        self.assertTrue(all(w == 1 for _, w in high_intervals(rows)))

    def test_local_stops_cut_output_within_four_clocks(self):
        for name, op, level, reason in (('stop_key', 'S2', 0, 2), ('arm_off', 'D', 1, 3)):
            with self.subTest(name=name):
                core = Core().armed(pulse=10).press('S1').wait(MS).add(op, level).wait(MS)
                rows, inputs = run_core(core)
                event = [i for i in inputs if i[1] == op and i[2] == level][-1]
                off = first(rows, after=event[0], driving=0)
                self.assertLessEqual(off['cycle'] - event[0], 4)
                self.assertEqual(transitions(rows)[-1], (SAFE, reason))
                self.assertEqual(rows[-1]['loaded'], 0)

    def test_stop_key_and_arm_off_together_report_stop_key(self):
        rows, _ = run_core(Core().armed(pulse=10).press('S1').wait(MS).add('S2', 0).add('D', 1).wait(MS))
        self.assertEqual(transitions(rows)[-1], (SAFE, 2))

    def test_host_abort_in_run_armed_and_safe(self):
        rows, _ = run_core(Core().armed(pulse=10).press('S1').wait(MS).add('M', 3).wait(MS))
        self.assertEqual(transitions(rows)[-1], (SAFE, 4))
        rows, _ = run_core(Core().armed().add('M', 3).wait(MS))
        self.assertEqual(transitions(rows)[-2:], [(ARMED, 0), (SAFE, 4)])
        rows, _ = run_core(Core().wait(500).config().wait(100).add('M', 3).wait(100))
        self.assertEqual(rows[-1]['loaded'], 0)
        self.assertNotIn(ARMED, [r['state'] for r in rows])

    def test_arm_timeout_returns_safe(self):
        rows, _ = run_core(Core().armed().wait(52 * MS))
        self.assertEqual(transitions(rows)[-2:], [(ARMED, 0), (SAFE, 5)])
        armed, safe = first(rows, state=ARMED), first(rows, state=SAFE, reason=5)
        self.assertAlmostEqual(safe['cycle'] - armed['cycle'], 50 * MS, delta=MS)

    def test_sensor_fault_latches_until_reset(self):
        core = Core().armed(pulse=10).press('S1').wait(MS).add('Q', 1).wait(3 * MS).add('Q', 7)
        core.wait(5 * MS).config().wait(MS)
        rows, _ = run_core(core)
        self.assertEqual(transitions(rows)[-1], (FAULT, 16))
        fault = first(rows, state=FAULT)
        self.assertEqual(first(rows, after=fault['cycle'], driving=0)['cycle'] - fault['cycle'], 1)
        self.assertEqual(rows[-1]['rx'] & 15, 4)
        rows, _ = run_core(core.add('R', 0).wait(5).add('R', 1).wait(10))
        self.assertEqual((rows[-1]['state'], rows[-1]['reason'], rows[-1]['loaded']), (SAFE, 0, 0))

    def test_stale_estimator_faults_in_armed(self):
        rows, inputs = run_core(Core().armed().add('P', 0).wait(10 * MS))
        self.assertEqual(transitions(rows)[-1], (FAULT, 17))
        stop = [i for i in inputs if i[1] == 'P'][0][0]
        self.assertLessEqual(first(rows, state=FAULT)['cycle'] - stop, 1000 + 400 + 2)

    def test_travel_limit_inclusive_and_both_signs(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                core = Core().add('N', 1000).armed(pulse=10).press('S1').wait(MS)
                core.add('N', 1000 + sign * 60).wait(MS)
                rows, _ = run_core(core)
                self.assertEqual(rows[-1]['state'], RUN)
                rows, inputs = run_core(core.add('N', 1000 + sign * 61).wait(MS))
                self.assertEqual(transitions(rows)[-1], (FAULT, 18))
                fault = first(rows, state=FAULT)
                self.assertEqual(fault['cycle'] - inputs[-1][0], 1)
                self.assertEqual(first(rows, after=fault['cycle'], driving=0)['cycle'] - inputs[-1][0], 2)

    def test_speed_limit_strict_and_both_signs(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                core = Core().armed(pulse=10).press('S1').add('V', sign * 3000).wait(2 * MS)
                rows, _ = run_core(core)
                self.assertEqual(rows[-1]['state'], RUN)
                rows, _ = run_core(core.add('V', sign * 3001).wait(2 * MS))
                self.assertEqual(transitions(rows)[-1], (FAULT, 19))

    def test_keepalive_timeout_in_armed_and_run(self):
        rows, _ = run_core(Core().armed(keep=5).wait(10 * MS, keepalive=None))
        self.assertEqual(transitions(rows)[-1], (FAULT, 20))
        rows, _ = run_core(Core().armed(keep=5, pulse=15).press('S1').wait(10 * MS, keepalive=None))
        self.assertEqual(transitions(rows)[-2:], [(RUN, 0), (FAULT, 20)])

    def test_output_module_rejects_parameters_beyond_its_own_limit(self):
        rows, _ = run_core(Core().armed(duty=5).press('S1').wait(MS), PWM_MAX_COMPARE=4)
        self.assertEqual(transitions(rows)[-1], (FAULT, 21))
        self.assertFalse(any(r['driving'] for r in rows))

    def test_run_overtime_faults(self):
        rows, _ = run_core(Core().armed(pulse=12).press('S1').wait(30 * MS), PWM_PERIOD=40)
        self.assertEqual(transitions(rows)[-1], (FAULT, 22))
        run_row = first(rows, state=RUN)
        self.assertAlmostEqual(first(rows, state=FAULT)['cycle'] - run_row['cycle'], 23 * MS, delta=MS)

    def test_config_field_and_state_rejections(self):
        bad = [dict(cid=0), dict(direction=0), dict(direction=3), dict(reserved=1), dict(duty=0),
               dict(duty=9), dict(pulse=0), dict(pulse=21), dict(travel=0), dict(travel=201),
               dict(speed=0), dict(speed=10001), dict(keep=0), dict(keep=501)]
        core = Core().wait(500)
        for b in bad:
            core.config().wait(20).config(**b).wait(20)
        core.config().add('D', 0).wait(60).config(cid=7).wait(20)
        rows, _ = run_core(core)
        rejected = [r for r in rows if r['rx'] & 15 == 5]
        entries = [r for a, r in zip(rows, rows[1:]) if r['rx'] & 15 == 5 and a['rx'] & 15 != 5]
        self.assertEqual(len(entries), len(bad))
        self.assertTrue(all(r['loaded'] == 0 for r in rejected))
        self.assertEqual((rows[-1]['rx'] & 15, rows[-1]['id'], rows[-1]['state']), (4, 0x1234, ARMED))
        rows, _ = run_core(core.add('M', 9).wait(20))
        self.assertEqual(rows[-1]['rx'] & 15, 6)
        rows, _ = run_core(core.add('X').wait(20))
        self.assertEqual((rows[-1]['rx'] & 15, rows[-1]['state']), (7, ARMED))

    def test_arming_preconditions(self):
        cases = {
            'dip_on_through_reset': Core().add('D', 0).wait(500).config().wait(200),
            'start_key_held': Core().add('S1', 0).armed().wait(200),
            'stop_key_held': Core().add('S2', 0).armed().wait(200),
            'estimator_not_ready': Core().add('Q', 3).armed().wait(200),
            'keepalive_stale': Core().wait(500).config(keep=2).wait(3 * MS, keepalive=None)
                                     .add('D', 0).wait(200, keepalive=None),
            'short_dip_on': Core().wait(500).config().add('D', 0).wait(10).add('D', 1).wait(200),
        }
        for name, core in cases.items():
            with self.subTest(name=name):
                rows, _ = run_core(core)
                self.assertNotIn(ARMED, [r['state'] for r in rows])
        core = Core().add('D', 0).wait(500).config().wait(200).add('D', 1).wait(60).add('D', 0).wait(60)
        rows, _ = run_core(core)
        self.assertEqual(rows[-1]['state'], ARMED)

    def test_rearm_needs_new_config_and_dip_toggle(self):
        core = Core().armed(pulse=1).press('S1').wait(2 * MS).config(cid=9, pulse=1).wait(200).press('S1').wait(MS)
        rows, _ = run_core(core)
        complete = first(rows, state=SAFE, reason=1)
        self.assertFalse(any(r['state'] != SAFE for r in rows if r['cycle'] > complete['cycle']))
        self.assertEqual(rows[-1]['id'], 9)
        rows, _ = run_core(core.add('D', 1).wait(60).add('D', 0).wait(60).press('S1').wait(2 * MS))
        self.assertEqual(transitions(rows)[-3:], [(ARMED, 1), (RUN, 1), (SAFE, 1)])

    def test_short_press_and_held_start_key(self):
        rows, _ = run_core(Core().armed().press('S1', clocks=10).wait(MS))
        self.assertEqual(rows[-1]['state'], ARMED)
        core = Core().wait(500).config().add('S1', 0).add('D', 0).wait(200).add('S1', 1).wait(200)
        rows, _ = run_core(core)
        self.assertNotIn(RUN, [r['state'] for r in rows])
        self.assertEqual(rows[-1]['state'], ARMED)

    def test_reset_during_run_idles_pins_immediately(self):
        core = Core().armed(pulse=10).press('S1').wait(MS).add('R', 0).wait(3).add('R', 1).wait(MS)
        rows, inputs = run_core(core)
        reset = [i for i in inputs if i[1] == 'R' and i[2] == 0][0][0]
        # Asynchronous reset; the monitor samples this negedge before the script applies it.
        self.assertLessEqual(first(rows, after=reset, driving=0)['cycle'] - reset, 1)
        self.assertEqual((rows[-1]['state'], rows[-1]['loaded']), (SAFE, 0))


if __name__ == '__main__':
    unittest.main()
