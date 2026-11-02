"""Bit-exact file-driven tests for the bounded raw estimator, no board access.

Every simulator run uses a fresh retained ASCII directory beneath
LOCALAPPDATA/J280Build/state-core-tests; paths are passed as plusargs.
"""
import csv
import os
from pathlib import Path
import random
import shutil
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MASK32 = (1 << 32) - 1
MAX_TIME = (1 << 52) - 1
DEFAULT = dict(WINDOW_US=10000, PERIOD_US=2000, MAX_ADC_AGE_US=100,
               MAX_ADC_STEP=64, MAX_ENCODER_STEP=64, HISTORY_DEPTH=8)
FIELDS = ('tick', 'segment', 'position', 'ev', 'av', 'espan', 'aspan',
          'atime', 'emid', 'amid', 'eage', 'aage', 'quality', 'reason')


def sample(seq=0, tick=2000, enc=0, adc=500, age=2, flags=3, fault=0,
           raw_tick=None, version=2, state=2, **extra):
    data = bytearray(28)
    struct.pack_into('<BBIIHHiHHBBHH', data, 0, version, flags, seq & MASK32,
                     (tick if raw_tick is None else raw_tick) & MASK32,
                     adc, age, ((enc + (1 << 31)) & MASK32) - (1 << 31),
                     0, 0, 0, state, fault, 65535)
    return dict(tick=tick, payload=int.from_bytes(data, 'little'), **extra)


class IntegerOracle:
    """Rational integer oracle: choose endpoints by time, never nominal span.

    Independent from the RTL ring implementation and floating-point velocities
    in RawWindowEstimator. Invalid arithmetic candidates drop the entire row.
    """
    def __init__(self, **params):
        self.p = DEFAULT | params
        self.segment = 0
        self.history = []

    def update(self, row, overrun=False):
        p, tick, payload = self.p, row['tick'], row['payload']
        b = payload.to_bytes(28, 'little')
        version, flags, seq, low, adc, age, enc, _, _, mode, state, fault, _ = struct.unpack('<BBIIHHiHHBBHH', b)
        out = dict.fromkeys(FIELDS, 0) | dict(tick=tick, segment=self.segment)
        reason = 0
        config_ok = (p['PERIOD_US'] > 0 and p['WINDOW_US'] > 0 and
                     p['WINDOW_US'] % p['PERIOD_US'] == 0 and
                     p['HISTORY_DEPTH'] >= p['WINDOW_US'] // p['PERIOD_US'] + 2 and
                     0 <= p['MAX_ADC_AGE_US'] <= 65535 and
                     p['MAX_ADC_AGE_US'] < p['PERIOD_US'] and
                     0 <= p['MAX_ADC_STEP'] <= 1023 and
                     0 <= p['MAX_ENCODER_STEP'] <= (1 << 31) - 1)
        if overrun:
            reason = 11
        elif not config_ok:
            reason = 10
        elif flags != 3 or version != 2 or state != 2:
            reason = 2
        elif fault:
            reason = 3
        elif age > p['MAX_ADC_AGE_US']:
            reason = 4
        elif adc > 1023 or tick > MAX_TIME or low != tick & MASK32:
            reason = 10
        elif tick < age:
            reason = 9
        if reason:
            self.history = []
            return out | dict(reason=reason)
        atime, boundary = tick - age, 0
        if self.history:
            prev = self.history[-1]
            if seq != (prev['seq'] + 1) & MASK32:
                boundary = 5
            elif tick != prev['tick'] + p['PERIOD_US']:
                boundary = 6
            if boundary:
                self.history = []
            else:
                delta = ((enc - prev['enc'] + (1 << 31)) & MASK32) - (1 << 31)
                if abs(delta) > p['MAX_ENCODER_STEP']:
                    reason = 7
                elif atime <= prev['atime']:
                    reason = 9
                elif abs(adc - prev['adc']) > p['MAX_ADC_STEP']:
                    reason = 8
                if reason:
                    self.history = []
                    return out | dict(reason=reason)
        if not self.history:
            self.segment += 1
            pos = enc
            boundary = boundary or 12
        else:
            pos = self.history[-1]['position'] + delta
        current = dict(seq=seq, tick=tick, enc=enc, adc=adc, atime=atime, position=pos)
        out.update(segment=self.segment, position=pos, atime=atime, quality=1)
        for key, value, stem, bit in [('tick', 'position', 'e', 2), ('atime', 'adc', 'a', 4)]:
            eligible = [past for past in self.history if current[key] - past[key] >= p['WINDOW_US']]
            if not eligible:
                continue
            start = eligible[-1]
            span = current[key] - start[key]
            mid = current[key] + start[key]
            age2 = 2 * tick - mid
            diff = current[value] - start[value]
            q = abs(diff) * 1000000 * 65536 // span
            q = -q if diff < 0 else q
            if not (0 < span <= MASK32 and 0 <= age2 <= MASK32 and
                    -(1 << 31) <= q < (1 << 31)):
                self.history = []
                return dict.fromkeys(FIELDS, 0) | dict(tick=tick, segment=self.segment, reason=10)
            out.update({stem+'v': q, stem+'span': span, stem+'mid': mid, stem+'age': age2})
            out['quality'] |= bit
        self.history.append(current)
        out['reason'] = boundary or (0 if out['quality'] == 7 else 1)
        return out


class RawEstimatorRTLTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from test_j280_motor_test_rtl import BIN as _iverilog_bin
        cls.bin = _iverilog_bin
        cls.build = Path(os.environ.get('LOCALAPPDATA', tempfile.gettempdir())) / 'J280Build/state-core-tests'
        cls.build.mkdir(parents=True, exist_ok=True)
        cls.env = dict(os.environ, PATH=str(cls.bin) + os.pathsep + os.environ.get('PATH', ''))

    def run_vectors(self, rows, **params):
        run = Path(tempfile.mkdtemp(prefix='run-', dir=self.build))
        self.assertTrue(str(run).isascii(), run)
        core = ROOT / 'rtl/j280_state/j280_raw_estimator.v'
        self.assertTrue(core.is_file(), 'missing estimator module (RED): ' + str(core))
        sources = [core]
        helper = core.with_name('j280_signed_divider.v')
        if helper.exists():
            sources.append(helper)
        sources.append(ROOT / 'sim/j280_state/tb_raw_estimator.v')
        for src in sources:
            shutil.copy2(src, run / src.name)
        vector_text = ''.join(f"{r.get('op', 0)} {r.get('gap', 2)} {r.get('tick', 0):016x} {r.get('payload', 0):056x}\n" for r in rows)
        (run / 'vectors.txt').write_text(vector_text, encoding='ascii')
        command = [str(self.bin / 'iverilog.exe'), '-g2001', '-Wall', '-s', 'tb_raw_estimator',
                   *[f'-Ptb_raw_estimator.{k}={v}' for k, v in params.items()], '-o', 'core.vvp',
                   *[s.name for s in sources]]
        result = subprocess.run(command, cwd=run, env=self.env, capture_output=True, text=True, timeout=60)
        (run / 'compile.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        result = subprocess.run([str(self.bin / 'vvp.exe'), 'core.vvp', '+VECTORS=vectors.txt', '+RESULTS=results.txt'],
                                cwd=run, env=self.env, capture_output=True, text=True, timeout=60)
        (run / 'simulation.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn('FAIL', result.stdout, str(run) + '\n' + result.stdout)
        self.assertIn('PASS tb_raw_estimator', result.stdout)
        outputs = []
        for line in (run / 'results.txt').read_text().splitlines():
            fields = line.split()
            outputs.append(dict(zip(FIELDS, map(int, fields[2:]))) | dict(payload=int(fields[1], 16)))
        return outputs

    def check_rows(self, rows, **params):
        oracle = IntegerOracle(**params)
        expected = []
        for r in rows:
            if r.get('op') == 2:
                oracle = IntegerOracle(**params)
            elif r.get('op') == 3:
                pass
            elif r.get('op') == 1:
                # Unless marked as completing, a busy submission is canceled
                # before it produces a result.
                if r.get('completes'):
                    expected.append(oracle.update(r, r.get('overrun', False)) | dict(payload=r['payload']))
            else:
                expected.append(oracle.update(r, r.get('overrun', False)) | dict(payload=r['payload']))
        actual = self.run_vectors(rows, **params)
        self.assertEqual(len(actual), len(expected))
        for i, (a, e) in enumerate(zip(actual, expected)):
            self.assertEqual(a, e, f'row {i}: actual={a}, expected={e}')
        return actual

    def test_signed_and_constant_velocities_independent_source_windows(self):
        for step in (3, -3, 0):
            with self.subTest(step=step):
                rows = [sample(i, 2000+i*2000, 500+i*step, 500+i*step,
                               age=[0, 99, 1, 100, 2, 1, 99][i % 7]) for i in range(35)]
                out = self.check_rows(rows)
                self.assertTrue(any(r['espan'] != r['aspan'] and r['quality'] == 7 for r in out))

    def test_modular_encoder_both_directions_sequence_and_tick_wraps(self):
        for step, start in [(10, (1 << 31)-21), (-10, -(1 << 31)+21)]:
            with self.subTest(step=step):
                self.check_rows([sample(MASK32-3+i, (1 << 32)-5000+i*2000,
                                        start+i*step, 500+i, age=i % 101) for i in range(20)])

    def test_rejections_clear_both_histories_and_rewarm(self):
        bads = [dict(flags=0), dict(flags=1), dict(flags=2), dict(flags=7), dict(flags=11),
                dict(flags=19), dict(flags=131), dict(fault=1), dict(age=101),
                dict(adc=1024), dict(raw_tick=123), dict(enc=1000), dict(adc=900),
                dict(tick=MAX_TIME+1), dict(tick=0, age=2), dict(version=1), dict(state=1)]
        rows = []
        for n, bad in enumerate(bads):
            base = 100000 + n * 100000
            rows += [sample(i, base+i*2000, i, 500+i) for i in range(8)]
            rows.append(sample(**(dict(seq=8, tick=base+16000, enc=8, adc=508) | bad)))
            rows += [sample(i, base+i*2000, i, 500+i) for i in range(9, 17)]
        self.check_rows(rows)

    def test_gaps_seed_valid_boundary_rows(self):
        rows = [sample(i, 100000+i*2000, i, 500+i) for i in range(8)]
        rows += [sample(100+i, 200000+i*2000, -100+i, 600+i) for i in range(9)]
        rows += [sample(109+i, 230000+i*2000, 10+i, 300+i) for i in range(9)]
        out = self.check_rows(rows)
        self.assertEqual((out[8]['reason'], out[8]['quality']), (5, 1))
        self.assertEqual((out[17]['reason'], out[17]['quality']), (6, 1))

    def test_reset_cancels_scan_and_division_without_stale_results(self):
        for delay in (1, 12, 55, 110, 170):
            with self.subTest(delay=delay):
                rows = [sample(i, 100000+i*2000, i, 500+i) for i in range(8)]
                rows += [sample(8, 116000, 8, 508, op=1), dict(op=2, gap=delay), dict(op=3)]
                rows += [sample(i, 300000+i*2000, -i, 500-i) for i in range(10)]
                self.check_rows(rows)

    def test_busy_overrun_cancels_old_context_and_divider(self):
        for delay in (1, 12, 55, 110, 170):
            with self.subTest(delay=delay):
                rows = [sample(i, 100000+i*2000, i, 500+i) for i in range(8)]
                rows += [sample(8, 116000, 8, 508, op=1),
                         sample(9, 118000, 9, 509, gap=delay, overrun=True), dict(op=3)]
                rows += [sample(10+i, 120000+i*2000, 10+i, 510+i) for i in range(9)]
                # op=0 normally asserts ready. The overrun row deliberately bypasses it.
                out = self.check_rows(rows)
                self.assertEqual(out[8]['reason'], 11)

    def test_submission_on_commit_edge_is_not_overrun(self):
        # Sample 9 is captured gap+1 cycles after sample 8, whose result commits
        # 256 cycles after capture; a capture on the commit edge keeps the result.
        for delay, kept in ((254, False), (255, True), (256, True)):
            with self.subTest(delay=delay):
                rows = [sample(i, 100000+i*2000, i, 500+i) for i in range(8)]
                rows += [sample(8, 116000, 8, 508, op=1, completes=kept),
                         sample(9, 118000, 9, 509, op=1, gap=delay, completes=True,
                                overrun=not kept), dict(op=3, gap=600)]
                rows += [sample(10+i, 120000+i*2000, 10+i, 510+i) for i in range(9)]
                out = self.check_rows(rows)
                self.assertEqual(out[8]['reason'], 0 if kept else 11)

    def test_numeric_velocity_overflow_and_rewarming(self):
        rows = [sample(i, 10000+i*2, i, 500+i, age=0) for i in range(6)]
        rows += [sample(6+i, 10012+i*2, 6, 506, age=0) for i in range(8)]
        self.check_rows(rows, WINDOW_US=10, PERIOD_US=2, MAX_ADC_AGE_US=1)

    def test_timestamp_limit_zero_source_and_half_microsecond_midpoints(self):
        self.check_rows([sample(i, MAX_TIME-30000+i*2000, -i, 500-i, age=i % 2) for i in range(16)])
        self.check_rows([sample(i, i*2000, i, 500+i, age=0 if i == 0 else 1) for i in range(10)])

    def test_invalid_candidate_parameters_fail_closed(self):
        for params in [dict(HISTORY_DEPTH=6), dict(WINDOW_US=9999), dict(MAX_ADC_AGE_US=2000),
                       dict(PERIOD_US=0), dict(MAX_ADC_STEP=1024), dict(MAX_ENCODER_STEP=-1)]:
            with self.subTest(params=params):
                self.check_rows([sample(i, 10000+i*2000) for i in range(8)], **params)

    def test_deterministic_long_ring_and_error_replay(self):
        rng = random.Random(280)
        seq, tick, enc, adc = 0, 100000, 0, 500
        rows = []
        for i in range(600):
            seq = (seq + (2 if i % 79 == 0 else 1)) & MASK32
            tick += 4000 if i % 93 == 0 else 2000
            enc += rng.randint(-64, 64)
            adc = max(0, min(1023, adc + rng.randint(-60, 60)))
            rows.append(sample(seq, tick, enc, adc, age=rng.randint(0, 100),
                               fault=1 if i % 101 == 100 else 0))
        self.check_rows(rows)

    def test_real_static_capture_replay_against_rational_oracle(self):
        path = ROOT / 'docs/data/adc-static-recheck-2026-10-16-01.csv'
        rows = []
        with path.open(newline='', encoding='utf-8-sig') as f:
            for r in list(csv.DictReader(f))[:500]:
                flags = int(r['adc_valid'] == 'True') | (int(r['adc_new'] == 'True') << 1)
                flags |= int(r['buffer_overflow'] == 'True') << 3
                rows.append(sample(int(r['seq']), int(r['tick_us_unwrapped']), int(r['enc_count']),
                                   int(r['adc_raw10']), int(r['adc_age_us']), flags, int(r['fault_bits'])))
        self.assertGreater(len(rows), 20)
        self.check_rows(rows)


if __name__ == '__main__':
    unittest.main()
