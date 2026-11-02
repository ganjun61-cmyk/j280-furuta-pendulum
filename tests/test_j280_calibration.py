"""Offline calibration contracts; never access hardware."""

import csv
import importlib.util
import io
import math
from pathlib import Path
import json
import struct
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location('j280_calibration', SCRIPTS / 'j280_calibration.py')
if (SCRIPTS / 'j280_calibration.py').exists():
    calibration = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(calibration)
else:
    calibration = None


def points(prefix='train', offset=0):
    return [dict(id=f'{prefix}-{x}', reference_deg=x, mean_raw=100 + 2*x + offset,
                 raw_sha256=f'{prefix}-raw-{x}', sample_raw_min=99+2*x+offset,
                 sample_raw_max=101+2*x+offset, frames=1001)
            for x in (0, 45, 90, 135, 180)]


def row(raw=280, **changes):
    value = dict(adc_raw10=str(raw), adc_valid='True', adc_new='True',
                 frame_valid='True', crc_ok='True', fault_bits='0',
                 buffer_overflow='False', seq_gap='0', duplicate='False',
                 out_of_order='False', reset='False', reset_or_reorder='False')
    value.update(changes)
    return value


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(calibration, 'Implement scripts/j280_calibration.py')

    def test_linear_known_answer_and_equal_segment_weight(self):
        data = points()
        data[0]['frames'] = 100000
        fit = calibration.fit_linear(data)
        self.assertAlmostEqual(fit['raw_per_deg'], 2)
        self.assertAlmostEqual(fit['raw_intercept'], 100)

    def test_validation_is_held_out_and_residual_is_phone_relative(self):
        profile = calibration.build_profile(points(), points('check', 10), 1)
        self.assertAlmostEqual(profile['model']['raw_per_deg'], 2)
        self.assertAlmostEqual(profile['model']['raw_intercept'], 100)
        self.assertAlmostEqual(profile['validation']['max_abs_phone_residual_deg'], 5)
        self.assertFalse(profile['controller_ready'])
        self.assertEqual(profile['status'], 'provisional_offline_only')
        self.assertEqual(profile['reference']['reported_resolution_deg'], 1)
        self.assertIsNone(profile['reference']['accuracy_bound_deg'])

    def test_rejects_leaked_segment_ids_or_raw_capture(self):
        for attribute in ('id', 'raw_sha256'):
            with self.subTest(attribute=attribute):
                check = points('check')
                check[0][attribute] = points()[0][attribute]
                with self.assertRaisesRegex(ValueError, 'overlap'):
                    calibration.build_profile(points(), check, 1)

    def test_rejects_bad_fit_inputs(self):
        cases = [[], points()[:2], [dict(p, reference_deg=0) for p in points()],
                 [dict(p, mean_raw=math.nan) for p in points()],
                 [dict(p, reference_deg=math.inf) for p in points()],
                 [dict(p, mean_raw=100) for p in points()]]
        for value in cases:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    calibration.fit_linear(value)

    def test_upright_coordinate_is_not_downward_phone_zero(self):
        profile = calibration.build_profile(points(), points('check'), 1)
        self.assertAlmostEqual(calibration.map_raw(profile, 460)['alpha_deg'], 0)
        self.assertAlmostEqual(calibration.map_raw(profile, 100)['alpha_deg'], -180)
        self.assertAlmostEqual(calibration.map_raw(profile, 280)['phone_deg'], 90)

    def test_does_not_extrapolate_or_silently_clamp(self):
        profile = calibration.build_profile(points(), points('check'), 1)
        for raw in (99, 461, -1, 1024, math.nan):
            with self.subTest(raw=raw):
                result = calibration.map_raw(profile, raw)
                self.assertIsNone(result['alpha_deg'])
                self.assertNotEqual(result['status'], 'ok_provisional')

    def test_bad_telemetry_is_not_converted_to_a_valid_angle(self):
        profile = calibration.build_profile(points(), points('check'), 1)
        cases = [dict(adc_valid='False'), dict(adc_new='False'), dict(crc_ok='False'),
                 dict(frame_valid='False'), dict(fault_bits='1'), dict(seq_gap='2'),
                 dict(reset='True'), dict(out_of_order='True'), dict(duplicate='True'),
                 dict(reset_or_reorder='True'), dict(buffer_overflow='True')]
        for changes in cases:
            with self.subTest(changes=changes):
                result = calibration.derive_row(profile, row(**changes))
                self.assertIsNone(result['alpha_deg'])

    def test_rejects_bad_validation_reference_or_duplicate_capture(self):
        for changes in (dict(reference_deg=-1), dict(reference_deg=math.nan),
                        dict(mean_raw=math.inf)):
            check = points('check')
            check[0].update(changes)
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    calibration.build_profile(points(), check, 1)
        check = points('check')
        check[1]['raw_sha256'] = check[0]['raw_sha256']
        with self.assertRaisesRegex(ValueError, 'overlap'):
            calibration.build_profile(points(), check, 1)

    def test_fractional_raw_is_not_truncated(self):
        profile = calibration.build_profile(points(), points('check'), 1)
        self.assertIsNone(calibration.derive_row(profile, row(adc_raw10=280.5))['alpha_deg'])

    def test_malformed_profile_is_rejected_before_output_creation(self):
        profile = calibration.build_profile(points(), points('check'), 1)
        profile['model']['phone_deg_domain'] = [-360, 360]
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory)/'source.csv', Path(directory)/'target.csv'
            with source.open('w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(row()))
                writer.writeheader()
                writer.writerow(row())
            with self.assertRaises(ValueError):
                calibration.derive_file(profile, source, target)
            self.assertFalse(target.exists())

    def test_derive_keeps_raw_fields_and_refuses_overwrite(self):
        profile = calibration.build_profile(points(), points('check'), 1)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'source.csv'
            target = Path(directory) / 'angles.csv'
            with source.open('w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(row()))
                writer.writeheader()
                writer.writerow(row())
            original = source.read_bytes()
            calibration.derive_file(profile, source, target)
            with target.open(newline='', encoding='utf-8') as handle:
                result = next(csv.DictReader(handle))
            self.assertEqual(result['adc_raw10'], '280')
            self.assertAlmostEqual(float(result['alpha_deg_provisional']), -90)
            self.assertEqual(source.read_bytes(), original)
            with self.assertRaises(FileExistsError):
                calibration.derive_file(profile, source, target)

    def _capture_fixture(self, directory):
        import j280_measurement as measurement
        root = Path(directory)
        raw = bytearray()
        for seq in range(1001):
            payload = struct.pack('<BBIIHHiHHBBHH', 1, 3, seq, (seq+1)*2000,
                                  280, 2, 0, 0, 0, 0, 1, 0, 65535)
            raw.extend(b'\xa5\x5a' + payload + struct.pack('<H', measurement.crc16_ccitt(payload)))
        (root/'fixture.bin').write_bytes(raw)
        parser = measurement.FrameStream()
        rows = parser.feed(raw)
        with (root/'fixture.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=measurement.CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        (root/'fixture-diagnostics.json').write_text(json.dumps(parser.stats), encoding='utf-8')
        session = dict(session_id='fixture', angle_reference=dict(phone_display_deg_reported=90),
                       raw_files=['fixture.csv', 'fixture.bin', 'fixture-diagnostics.json'],
                       summary=dict(frames=1001, adc_raw_mean=280))
        (root/'fixture-session.json').write_text(json.dumps(session), encoding='utf-8')
        manifest = dict(points=[dict(id='fixture', session_file='fixture-session.json',
                                     frames=1001, phone_deg=90, adc_mean=280)])
        path = root/'round.json'
        path.write_text(json.dumps(manifest), encoding='utf-8')
        return path

    def test_load_redecodes_raw_and_checks_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._capture_fixture(directory)
            loaded = calibration.load_round(path)
            self.assertEqual(loaded[0]['mean_raw'], 280)
            csv_path = Path(directory)/'fixture.csv'
            text = csv_path.read_text(encoding='utf-8')
            csv_path.write_text(text.replace(',280,', ',281,', 1), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'CSV|raw'):
                calibration.load_round(path)

    def test_rejects_unmatched_raw_or_forged_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._capture_fixture(directory)
            session_path = Path(directory)/'fixture-session.json'
            session = json.loads(session_path.read_text(encoding='utf-8'))
            session['summary']['adc_raw_mean'] = 300
            session_path.write_text(json.dumps(session), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'mean|summary'):
                calibration.load_round(path)


if __name__ == '__main__':
    unittest.main()
