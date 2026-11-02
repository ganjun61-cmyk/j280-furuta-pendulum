"""Raw-domain offline estimator contracts; synthetic data only, no hardware."""

import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'j280_state_estimation.py'
if SCRIPT.exists():
    SPEC = importlib.util.spec_from_file_location('j280_state_estimation', SCRIPT)
    estimation = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(estimation)
else:
    estimation = None

CONFIG = dict(window_us=2000, expected_period_us=1000, max_adc_age_us=500,
              max_adc_step=100, max_encoder_step=100)
MOD = 2**32


def row(i=0, **changes):
    tick = 10000 + i * 1000
    value = dict(session_id='test', seq=i % MOD, tick_us=tick % MOD,
                 tick_us_unwrapped=tick, epoch=0, adc_raw10=100+i*10,
                 adc_valid=True, adc_new=True, adc_age_us=100, enc_count=i*2,
                 frame_valid=True, crc_ok=True, buffer_overflow=False,
                 duplicate=False, out_of_order=False, reset=False,
                 reset_or_reorder=False, fault_bits=0, seq_gap=0)
    value.update(changes)
    return value


class EstimatorTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(estimation, 'Implement scripts/j280_state_estimation.py')

    def estimator(self, **changes):
        return estimation.RawWindowEstimator(**(CONFIG | changes))

    def warm(self, estimator):
        return [estimator.update(row(i)) for i in range(3)][-1]

    def assert_blank(self, result):
        self.assertEqual(result['status'], 'invalid')
        self.assertFalse(result['position_valid'])
        for key in estimation.ESTIMATE_FIELDS:
            self.assertIsNone(result[key], key)
        self.assertTrue(result['reason'])

    def test_ramp_source_times_midpoints_and_warmup(self):
        estimator = self.estimator()
        for i in range(2):
            result = estimator.update(row(i))
            self.assertEqual(result['status'], 'warming')
            self.assertTrue(result['position_valid'])
            self.assertIsNone(result['encoder_velocity_count_s'])
        result = estimator.update(row(2))
        self.assertEqual(result['status'], 'ready')
        self.assertFalse(result['controller_ready'])
        self.assertTrue(result['estimator_version'])
        self.assertEqual(result['encoder_position_count'], 4)
        self.assertEqual(result['adc_position_code'], 120)
        self.assertEqual(result['encoder_position_time_us'], 12000)
        self.assertEqual(result['adc_position_time_us'], 11900)
        self.assertEqual(result['encoder_velocity_count_s'], 2000)
        self.assertEqual(result['adc_velocity_code_s'], 10000)
        for channel, start, end in [('encoder', 10000, 12000), ('adc', 9900, 11900)]:
            self.assertTrue(result[f'{channel}_velocity_valid'])
            self.assertEqual(result[f'{channel}_velocity_start_us'], start)
            self.assertEqual(result[f'{channel}_velocity_end_us'], end)
            self.assertEqual(result[f'{channel}_velocity_midpoint_us'], (start+end)/2)
            self.assertEqual(result[f'{channel}_velocity_span_us'], 2000)
            self.assertEqual(result[f'{channel}_velocity_age_us'], 12000-(start+end)/2)

    def test_changing_adc_age_uses_actual_time_and_independent_windows(self):
        estimator = self.estimator(window_us=1500)
        estimator.update(row(0, adc_age_us=0, adc_raw10=100))
        estimator.update(row(1, adc_age_us=200, adc_raw10=108))
        result = estimator.update(row(2, adc_age_us=500, adc_raw10=115))
        self.assertEqual(result['adc_velocity_code_s'], 10000)
        self.assertEqual(result['adc_velocity_span_us'], 1500)
        self.assertEqual(result['encoder_velocity_span_us'], 2000)
        result = estimator.update(row(3, adc_age_us=0, adc_raw10=130))
        self.assertEqual(result['adc_velocity_start_us'], 11500)
        self.assertEqual(result['adc_velocity_span_us'], 1500)
        estimator = self.estimator()
        estimator.update(row(0, adc_age_us=0))
        estimator.update(row(1, adc_age_us=0))
        result = estimator.update(row(2, adc_age_us=500))
        self.assertTrue(result['encoder_velocity_valid'])
        self.assertFalse(result['adc_velocity_valid'])
        self.assertIsNone(result['adc_velocity_code_s'])
        self.assertEqual(result['status'], 'warming')

    def test_constant_and_reversal(self):
        estimator = self.estimator()
        for i in range(3):
            result = estimator.update(row(i, adc_raw10=100, enc_count=5))
        self.assertEqual(result['adc_velocity_code_s'], 0)
        self.assertEqual(result['encoder_velocity_count_s'], 0)
        for i in range(3, 6):
            result = estimator.update(row(i, adc_raw10=100-(i-2)*10, enc_count=5-(i-2)*2))
        self.assertEqual(result['adc_velocity_code_s'], -10000)
        self.assertEqual(result['encoder_velocity_count_s'], -2000)

    def test_all_bad_flags_clear_both_histories(self):
        bad = {k: False for k in ('adc_valid', 'adc_new', 'frame_valid', 'crc_ok')}
        bad.update({k: True for k in ('buffer_overflow', 'duplicate', 'out_of_order',
                                     'reset', 'reset_or_reorder')})
        bad.update(fault_bits=1, seq_gap=1)
        for key, value in bad.items():
            with self.subTest(key=key):
                estimator = self.estimator()
                previous = self.warm(estimator)
                self.assert_blank(estimator.update(row(3, **{key: value})))
                result = estimator.update(row(4))
                self.assertEqual(result['status'], 'warming')
                self.assertGreater(result['segment_id'], previous['segment_id'])
                self.assertIsNone(result['encoder_velocity_count_s'])
                self.assertIsNone(result['adc_velocity_code_s'])

    def test_continuity_boundaries_seed_without_bridging(self):
        cases = [dict(session_id='other'), dict(epoch=1), dict(seq=4), dict(seq=2),
                 dict(tick_us=14000, tick_us_unwrapped=14000),
                 dict(tick_us=11000, tick_us_unwrapped=11000)]
        for changes in cases:
            with self.subTest(changes=changes):
                estimator = self.estimator()
                old = self.warm(estimator)
                result = estimator.update(row(3, **changes))
                self.assertEqual(result['status'], 'warming')
                self.assertTrue(result['position_valid'])
                self.assertGreater(result['segment_id'], old['segment_id'])
                self.assertNotEqual(result['reason'], 'window_incomplete')
                self.assertIsNone(result['adc_velocity_code_s'])
                self.assertIsNone(result['encoder_velocity_count_s'])

    def test_normal_seq_and_tick_wrap(self):
        estimator = self.estimator()
        for i in range(3):
            tick = MOD-1000+i*1000
            result = estimator.update(row(i, seq=(MOD-2+i) % MOD,
                                          tick_us=tick % MOD, tick_us_unwrapped=tick))
        self.assertEqual(result['status'], 'ready')
        self.assertEqual(result['encoder_velocity_count_s'], 2000)
        self.assertEqual(result['segment_id'], 1)

    def test_both_signed_encoder_wrap_directions(self):
        for values, expected in [([2**31-2, 2**31-1, -2**31], 2**31),
                                 ([-2**31+1, -2**31, 2**31-1], -2**31-1)]:
            with self.subTest(values=values):
                estimator = self.estimator()
                for i, value in enumerate(values):
                    result = estimator.update(row(i, enc_count=value))
                self.assertEqual(result['encoder_position_count'], expected)
                self.assertEqual(result['encoder_velocity_count_s'], 1000 if expected > 0 else -1000)

    def test_bounds_jumps_and_negative_source_reject(self):
        cases = [dict(adc_raw10=-1), dict(adc_raw10=1024), dict(adc_age_us=-1),
                 dict(adc_age_us=65536), dict(adc_age_us=501), dict(enc_count=2**31),
                 dict(enc_count=-2**31-1), dict(seq=-1), dict(seq=MOD),
                 dict(tick_us=-1), dict(tick_us=MOD), dict(epoch=-1),
                 dict(tick_us_unwrapped=-1), dict(tick_us=13001),
                 dict(adc_raw10=400), dict(enc_count=400),
                 dict(enc_count=-2**31+4),
                 dict(tick_us=0, tick_us_unwrapped=0, adc_age_us=1)]
        for changes in cases:
            with self.subTest(changes=changes):
                estimator = self.estimator()
                self.warm(estimator)
                self.assert_blank(estimator.update(row(3, **changes)))
                self.assertEqual(estimator.update(row(4))['status'], 'warming')

    def test_threshold_equality_and_adc_does_not_wrap(self):
        estimator = self.estimator(max_adc_step=10, max_encoder_step=2, max_adc_age_us=100)
        self.assertEqual(self.warm(estimator)['status'], 'ready')
        estimator = self.estimator()
        estimator.update(row(0, adc_raw10=1023))
        self.assert_blank(estimator.update(row(1, adc_raw10=0)))

    def test_adc_source_must_strictly_increase(self):
        for age in (1000, 1001):
            with self.subTest(age=age):
                estimator = self.estimator(max_adc_age_us=2000)
                estimator.update(row(0, adc_age_us=0))
                self.assert_blank(estimator.update(row(1, adc_age_us=age)))

    def test_missing_fields_malformed_numbers_and_strict_booleans(self):
        for key in row():
            with self.subTest(missing=key):
                estimator = self.estimator()
                broken = row()
                del broken[key]
                self.assert_blank(estimator.update(broken))
        for key in ('seq', 'tick_us', 'tick_us_unwrapped', 'epoch', 'adc_raw10',
                    'adc_age_us', 'enc_count', 'fault_bits', 'seq_gap'):
            for value in (True, False, 1.0, 1.5, '1.0', 'nan', '', None, ' 1', '1_0'):
                with self.subTest(key=key, value=value):
                    self.assert_blank(self.estimator().update(row(**{key: value})))
        for key in ('adc_valid', 'adc_new', 'frame_valid', 'crc_ok', 'buffer_overflow',
                    'duplicate', 'out_of_order', 'reset', 'reset_or_reorder'):
            for value in (1, 0, 'true', 'false', '1', '0', None, ''):
                with self.subTest(key=key, value=value):
                    self.assert_blank(self.estimator().update(row(**{key: value})))
        for value in ('', None, 1):
            self.assert_blank(self.estimator().update(row(session_id=value)))
        estimator = self.estimator()
        for i in range(3):
            result = estimator.update({key: str(value) for key, value in row(i).items()})
        self.assertEqual(result['status'], 'ready')
        self.assert_blank(estimator.update(None))

    def test_integer_config_validation(self):
        for key in CONFIG:
            for value in (True, 1.0, '1000', None, -1):
                with self.subTest(key=key, value=value):
                    with self.assertRaises(ValueError):
                        self.estimator(**{key: value})
        for key in ('window_us', 'expected_period_us'):
            with self.assertRaises(ValueError):
                self.estimator(**{key: 0})
        for key, value in [('max_adc_age_us', 65536), ('max_adc_step', 1024),
                           ('max_encoder_step', 2**31), ('expected_period_us', 2**31)]:
            with self.assertRaises(ValueError):
                self.estimator(**{key: value})
        estimator = self.estimator(max_adc_age_us=0, max_adc_step=0, max_encoder_step=0)
        for i in range(3):
            result = estimator.update(row(i, adc_age_us=0, adc_raw10=100, enc_count=0))
        self.assertEqual(result['status'], 'ready')

    def test_window_uses_newest_eligible_endpoint_and_bounds_history(self):
        estimator = self.estimator(window_us=1501)
        for i in range(2000):
            result = estimator.update(row(i, adc_raw10=100, enc_count=0))
        self.assertEqual(result['encoder_velocity_span_us'], 2000)
        self.assertEqual(result['adc_velocity_span_us'], 2000)
        self.assertLessEqual(len(estimator._encoder_history), 4)
        self.assertLessEqual(len(estimator._adc_history), 4)

    def test_odd_span_midpoint_and_adc_can_warm_first(self):
        estimator = self.estimator(window_us=1999)
        estimator.update(row(0, adc_age_us=100))
        estimator.update(row(1, adc_age_us=100))
        result = estimator.update(row(2, adc_age_us=101))
        self.assertEqual(result['adc_velocity_span_us'], 1999)
        self.assertEqual(result['adc_velocity_midpoint_us'], 10899.5)
        self.assertEqual(result['adc_velocity_age_us'], 1100.5)
        estimator = self.estimator(window_us=2500)
        estimator.update(row(0, adc_age_us=500))
        estimator.update(row(1, adc_age_us=0))
        result = estimator.update(row(2, adc_age_us=0))
        self.assertTrue(result['adc_velocity_valid'])
        self.assertFalse(result['encoder_velocity_valid'])
        self.assertIsNone(result['encoder_velocity_count_s'])

    def test_origin_bounds_and_update_does_not_mutate_input(self):
        original = row(0, tick_us=0, tick_us_unwrapped=0, adc_age_us=0,
                       adc_raw10=0, enc_count=-2**31)
        before = original.copy()
        result = self.estimator().update(original)
        self.assertTrue(result['position_valid'])
        self.assertEqual(result['adc_position_time_us'], 0)
        self.assertEqual(original, before)
        result = self.estimator(max_adc_age_us=65535).update(
            row(tick_us=65535, tick_us_unwrapped=65535, adc_age_us=65535,
                adc_raw10=1023, enc_count=2**31-1))
        self.assertTrue(result['position_valid'])

    def test_timestamp_ceiling_rejects_and_clears_both_histories(self):
        for tick in (2**52, 10**400):
            for value in (tick, str(tick)):
                with self.subTest(tick=value):
                    estimator = self.estimator()
                    previous = self.warm(estimator)
                    result = estimator.update(row(3, tick_us=tick % MOD,
                                                  tick_us_unwrapped=value))
                    self.assert_blank(result)
                    self.assertEqual(result['reason'], 'invalid:tick_us_unwrapped')
                    self.assertEqual(len(estimator._encoder_history), 0)
                    self.assertEqual(len(estimator._adc_history), 0)
                    for i in range(4, 7):
                        result = estimator.update(row(i))
                        self.assertGreater(result['segment_id'], previous['segment_id'])
                        if i < 6:
                            self.assertEqual(result['status'], 'warming')
                            self.assertIsNone(result['encoder_velocity_count_s'])
                            self.assertIsNone(result['adc_velocity_code_s'])
                    self.assertEqual(result['status'], 'ready')

    def test_timestamp_ceiling_preserves_half_us_midpoints_and_wraps(self):
        ceiling = 2**52-1
        # Exercise the exact ceiling and a nearby uint32 tick wrap; seq wraps
        # in both cases. Odd spans require exact half-microsecond midpoints.
        for end in (ceiling, ceiling-(MOD-1)+1001):
            with self.subTest(end=end):
                estimator = self.estimator(window_us=1001, expected_period_us=1001)
                for i in range(3):
                    tick = end-(2-i)*1001
                    result = estimator.update(row(i, seq=(MOD-2+i) % MOD,
                                                  tick_us=tick % MOD,
                                                  tick_us_unwrapped=tick))
                self.assertEqual(result['status'], 'ready')
                self.assertEqual(result['segment_id'], 1)
                for channel, age in (('encoder', 0), ('adc', 100)):
                    self.assertEqual(result[f'{channel}_velocity_midpoint_us'], end-age-500.5)
                    self.assertEqual(result[f'{channel}_velocity_age_us'], age+500.5)
                    self.assertEqual(result[f'{channel}_velocity_span_us'], 1001)


class ReplayTests(unittest.TestCase):
    setUp = EstimatorTests.setUp
    def write_csv(self, path, rows, fields=None):
        with path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields or list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    def test_file_preserves_original_columns_and_summarizes(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'raw.csv', Path(directory)/'estimated.csv'
            rows = [row(i, note=f'original,{i}') for i in range(4)]
            self.write_csv(source, rows)
            before = source.read_bytes()
            summary = estimation.replay_file(source, output, CONFIG)
            self.assertEqual(summary['rows'], 4)
            self.assertEqual(summary['counts'], dict(warming=2, ready=2))
            self.assertEqual(summary['config'], CONFIG)
            self.assertFalse(summary['controller_ready'])
            self.assertTrue(summary['limitations'])
            json.dumps(summary, allow_nan=False)
            with output.open(newline='', encoding='utf-8') as handle:
                records = list(csv.DictReader(handle))
            self.assertEqual(source.read_bytes(), before)
            for original, record in zip(rows, records):
                for key, value in original.items():
                    self.assertEqual(record[key], str(value))
            self.assertEqual(records[0]['encoder_velocity_count_s'], '')
            self.assertEqual(records[-1]['encoder_velocity_count_s'], '2000.0')

    def test_refuses_existing_alias_and_malformed_headers_without_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'raw.csv', Path(directory)/'estimated.csv'
            self.write_csv(source, [row()])
            original = source.read_bytes()
            with self.assertRaises(FileExistsError):
                estimation.replay_file(source, source, CONFIG)
            output.write_text('keep', encoding='utf-8')
            with self.assertRaises(FileExistsError):
                estimation.replay_file(source, output, CONFIG)
            self.assertEqual(output.read_text(), 'keep')
            output.unlink()
            for fields in ([], ['seq'], list(row())+['seq']):
                source.write_text(','.join(fields)+'\n', encoding='utf-8')
                with self.assertRaises(ValueError):
                    estimation.replay_file(source, output, CONFIG)
                self.assertFalse(output.exists())
            for field in estimation.DERIVED_FIELDS:
                self.write_csv(source, [row(**{field: 'reserved'})])
                with self.assertRaises(ValueError):
                    estimation.replay_file(source, output, CONFIG)
                self.assertFalse(output.exists())
            source.write_bytes(original)
            with self.assertRaises(ValueError):
                estimation.replay_file(source, output, CONFIG | dict(window_us=1.5))
            self.assertFalse(output.exists())

    def test_bad_rows_are_blank_and_recovery_warms(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'raw.csv', Path(directory)/'estimated.csv'
            rows = [row(i) for i in range(7)]
            rows[3]['crc_ok'] = False
            self.write_csv(source, rows)
            summary = estimation.replay_file(source, output, CONFIG)
            self.assertEqual(summary['counts'], dict(warming=4, ready=2, invalid=1))
            with output.open(newline='', encoding='utf-8') as handle:
                records = list(csv.DictReader(handle))
            for key in estimation.ESTIMATE_FIELDS:
                self.assertEqual(records[3][key], '')
            self.assertNotEqual(records[4]['segment_id'], records[2]['segment_id'])

    def test_short_row_invalidates_and_extra_cells_leave_no_partial_file(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'raw.csv', Path(directory)/'estimated.csv'
            self.write_csv(source, [row()])
            original = source.read_text(encoding='utf-8')
            source.write_text(original + 'test,1\n', encoding='utf-8')
            summary = estimation.replay_file(source, output, CONFIG)
            self.assertEqual(summary['counts'], dict(warming=1, invalid=1))
            output.unlink()
            lines = original.splitlines()
            source.write_text(original + lines[-1] + ',extra\n', encoding='utf-8')
            before = source.read_bytes()
            with self.assertRaises(ValueError):
                estimation.replay_file(source, output, CONFIG)
            self.assertFalse(output.exists())
            self.assertEqual(source.read_bytes(), before)

    def test_cli_all_arguments_explicit_and_json_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'raw.csv', Path(directory)/'estimated.csv'
            self.write_csv(source, [row(i) for i in range(3)])
            args = [sys.executable, str(SCRIPT), '--input', str(source), '--output', str(output)]
            options = [part for key, value in CONFIG.items()
                       for part in ('--'+key.replace('_', '-'), str(value))]
            completed = subprocess.run(args+options, text=True, capture_output=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(json.loads(completed.stdout)['counts'], dict(warming=2, ready=1))
            completed = subprocess.run(args+options, text=True, capture_output=True)
            self.assertEqual(completed.returncode, 2)
            self.assertIn('error', json.loads(completed.stderr))
            for index in range(0, len(options), 2):
                completed = subprocess.run(args+options[:index]+options[index+2:],
                                           text=True, capture_output=True)
                self.assertEqual(completed.returncode, 2)

    def test_cli_rejects_out_of_domain_timestamps_with_json_and_no_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'raw.csv', Path(directory)/'estimated.csv'
            rows = []
            for i in range(3):
                tick = 10**400+i*1000
                rows.append(row(i, tick_us=tick % MOD, tick_us_unwrapped=tick))
            rows.append(row(3, tick_us=0, tick_us_unwrapped=2**52))
            self.write_csv(source, rows)
            args = [sys.executable, str(SCRIPT), '--input', str(source), '--output', str(output)]
            options = [part for key, value in CONFIG.items()
                       for part in ('--'+key.replace('_', '-'), str(value))]
            completed = subprocess.run(args+options, text=True, capture_output=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(completed.stderr, '')
            summary = json.loads(completed.stdout)
            self.assertEqual(summary['counts'], dict(invalid=4))
            self.assertEqual(summary['reason_counts'], {'invalid:tick_us_unwrapped': 4})
            with output.open(newline='', encoding='utf-8') as handle:
                records = list(csv.DictReader(handle))
            self.assertEqual(len(records), 4)
            for record in records:
                self.assertEqual(record['status'], 'invalid')
                for key in estimation.ESTIMATE_FIELDS:
                    self.assertEqual(record[key], '')


if __name__ == '__main__':
    unittest.main()
