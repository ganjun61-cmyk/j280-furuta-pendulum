#!/usr/bin/env python3
"""J280 provisional OFFLINE calibration; no serial or programming operations.

Train on segment means from round 1, hold out all new captures in round 2.
Phone angle is zero downward; the indicated control coordinate is phone-180.
This file never enables a controller. Unknown physical direction, phone accuracy,
unmeasured paths and fixture errors remain experiment limitations.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

from j280_measurement import FrameStream


TRUE_FIELDS = ('adc_valid', 'adc_new', 'frame_valid', 'crc_ok')
FALSE_FIELDS = ('buffer_overflow', 'duplicate', 'out_of_order', 'reset', 'reset_or_reorder')
DIAGNOSTIC_ERRORS = ('frames_invalid', 'crc_errors', 'version_errors',
                     'reserved_bits_errors', 'domain_errors', 'seq_missed',
                     'seq_duplicates', 'seq_out_of_order', 'resets', 'uncertain_rollbacks')
DERIVED_FIELDS = ('phone_reference_deg_provisional', 'alpha_deg_provisional', 'angle_status')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def is_true(value):
    return value is True or value == 'True'


def is_false(value):
    return value is False or value == 'False'


def telemetry_ok(row):
    try:
        return (all(is_true(row.get(k)) for k in TRUE_FIELDS)
                and all(is_false(row.get(k)) for k in FALSE_FIELDS)
                and int(row['fault_bits']) == 0 and int(row['seq_gap']) == 0)
    except (ValueError, TypeError, KeyError):
        return False


def load_round(manifest_path):
    """Re-decode raw BIN and verify every fixed segment against its evidence."""
    manifest_path = Path(manifest_path).resolve()
    root = manifest_path.parent
    manifest = read_json(manifest_path)
    points = []
    seen = set()
    for entry in manifest['points']:
        session_path = root / entry['session_file']
        session = read_json(session_path)
        name = entry['id']
        if name in seen or session['session_id'] != name:
            raise ValueError(f'duplicate or mismatched segment id: {name}')
        seen.add(name)
        files = [root / item for item in session['raw_files']]
        csv_path, = [p for p in files if p.suffix == '.csv']
        raw_path, = [p for p in files if p.suffix == '.bin']
        diag_path, = [p for p in files if p.name.endswith('-diagnostics.json')]
        parser = FrameStream()
        raw_rows = parser.feed(raw_path.read_bytes())
        diagnostics = read_json(diag_path)
        if diagnostics != parser.stats:
            raise ValueError(f'raw/diagnostics mismatch: {name}')
        if not raw_rows or any(parser.stats[k] for k in DIAGNOSTIC_ERRORS):
            raise ValueError(f'raw capture has no usable continuous frames: {name}')
        with csv_path.open(newline='', encoding='utf-8-sig') as handle:
            rows = list(csv.DictReader(handle))
        count = len(raw_rows)
        if count != len(rows) or count != session['summary']['frames'] or count != entry['frames']:
            raise ValueError(f'CSV/raw/frame count mismatch: {name}')
        for i, (raw, row) in enumerate(zip(raw_rows, rows)):
            for key, value in raw.items():
                if isinstance(value, bool):
                    same = is_true(row.get(key)) if value else is_false(row.get(key))
                elif isinstance(value, int):
                    try:
                        same = int(row[key]) == value
                    except (KeyError, TypeError, ValueError):
                        same = False
                else:
                    continue
                if not same:
                    raise ValueError(f'CSV/raw mismatch: {name} row {i} {key}')
            if (not telemetry_ok(raw) or raw['output_enabled']
                    or any(raw[k] != 0 for k in ('pwm_count', 'pwm_period_count', 'motor_mode'))):
                raise ValueError(f'invalid passive telemetry: {name}')
            if i and raw['tick_us_unwrapped'] - raw_rows[i-1]['tick_us_unwrapped'] != 2000:
                raise ValueError(f'nonuniform sample interval: {name}')
        span = raw_rows[-1]['tick_us_unwrapped'] - raw_rows[0]['tick_us_unwrapped']
        if span < 2_000_000:
            raise ValueError(f'static segment shorter than 2 s: {name}')
        codes = [r['adc_raw10'] for r in raw_rows]
        mean = statistics.mean(codes)
        if (not math.isclose(mean, session['summary']['adc_raw_mean'], abs_tol=1e-9)
                or not math.isclose(mean, entry['adc_mean'], abs_tol=1e-9)):
            raise ValueError(f'raw mean/summary mismatch: {name}')
        reference = session['angle_reference']['phone_display_deg_reported']
        if reference != entry['phone_deg']:
            raise ValueError(f'phone reference mismatch: {name}')
        enc = [r['enc_count'] for r in raw_rows]
        all_files = [manifest_path, session_path, csv_path, raw_path, diag_path]
        points.append(dict(id=name, reference_deg=reference, mean_raw=mean,
                           std_raw=statistics.pstdev(codes), frames=count,
                           sample_raw_min=min(codes), sample_raw_max=max(codes),
                           raw_sha256=sha256(raw_path), span_s=span/1e6,
                           encoder_min=min(enc), encoder_max=max(enc),
                           approach=entry.get('approach'),
                           input_sha256={str(p.resolve()): sha256(p) for p in all_files}))
    if not points:
        raise ValueError('round contains no points')
    return points


def fit_linear(points):
    if len(points) < 5:
        raise ValueError('need at least five independently fixed segments')
    x = [float(p['reference_deg']) for p in points]
    y = [float(p['mean_raw']) for p in points]
    if not all(math.isfinite(v) for v in x+y) or len(set(x)) < 5:
        raise ValueError('need finite measurements at five distinct angles')
    if min(x) < 0 or max(x) > 180 or min(y) < 0 or max(y) > 1023:
        raise ValueError('reference or raw ADC domain invalid')
    mx, my = statistics.mean(x), statistics.mean(y)
    slope = sum((a-mx)*(b-my) for a, b in zip(x, y)) / sum((a-mx)**2 for a in x)
    if not math.isfinite(slope) or abs(slope) < 1e-12:
        raise ValueError('zero or invalid sensitivity')
    return dict(raw_per_deg=slope, raw_intercept=my-slope*mx)


def residuals(model, points):
    a, b = model['raw_per_deg'], model['raw_intercept']
    rows = [dict(id=p['id'], reference_deg=p['reference_deg'],
                 mean_raw=p['mean_raw'],
                 indicated_phone_deg=(p['mean_raw']-b)/a,
                 residual_phone_deg=(p['mean_raw']-b)/a-p['reference_deg']) for p in points]
    return dict(segment_count=len(rows), segments=rows,
                max_abs_phone_residual_deg=max(abs(r['residual_phone_deg']) for r in rows),
                rmse_phone_residual_deg=math.sqrt(statistics.mean(r['residual_phone_deg']**2 for r in rows)))


def build_profile(training, validation, resolution_deg=1):
    if not validation:
        raise ValueError('independent validation captures required')
    for group in (training, validation):
        if (len({p['id'] for p in group}) != len(group)
                or len({p['raw_sha256'] for p in group}) != len(group)):
            raise ValueError('overlap within capture group')
        for point in group:
            x, y = float(point['reference_deg']), float(point['mean_raw'])
            if not math.isfinite(x) or not math.isfinite(y) or not 0 <= x <= 180 or not 0 <= y <= 1023:
                raise ValueError('nonfinite or out-of-domain calibration point')
    if ({p['id'] for p in training} & {p['id'] for p in validation}
            or {p['raw_sha256'] for p in training} & {p['raw_sha256'] for p in validation}):
        raise ValueError('training/validation overlap')
    if not math.isfinite(resolution_deg) or resolution_deg <= 0:
        raise ValueError('reference resolution must be finite and positive')
    model = fit_linear(training)
    domain = [min(p['reference_deg'] for p in training), max(p['reference_deg'] for p in training)]
    if domain != [0, 180]:
        raise ValueError('need measured downward and upward endpoints')
    model.update(phone_deg_domain=domain, alpha_deg_domain=[-180, 0],
                 alpha_definition='phone_reference_deg - 180; measured path only',
                 fitted_upright_raw=model['raw_intercept']+180*model['raw_per_deg'])
    profile = dict(schema_version=1, status='provisional_offline_only', controller_ready=False,
                   swingup_path_validated=False,
                   reference=dict(instrument='smartphone_inclinometer', app=None,
                                  reported_resolution_deg=resolution_deg, accuracy_bound_deg=None,
                                  physical_view_and_direction=None),
                   model=model, weighting='equal_weight_per_fixed_segment_mean',
                   training=residuals(model, training), validation=residuals(model, validation),
                   points=dict(training=training, validation=validation),
                   limitations=[
                       'Phone accuracy and alignment uncertainty not independently calibrated.',
                       'Arm held by hand; encoder constancy does not prove a rigid pivot.',
                       'Discrete static points do not certify the continuous swing-up path.',
                       'Only phone reference 0..180 measured; no extrapolation or capture-margin claim.',
                       'Fitted upright raw is not a measured precise zero or controller setting.',
                       'No encoder angular scale, motor model, feedback gains or velocity estimator included.',
                       'Firmware identity was not read back for every static segment.'])
    profile['source_code_sha256'] = {Path(__file__).name: sha256(__file__),
                                    'j280_measurement.py': sha256(Path(__file__).with_name('j280_measurement.py'))}
    return profile


def validate_profile(profile):
    model = profile['model']
    a, b = model['raw_per_deg'], model['raw_intercept']
    if (profile.get('schema_version') != 1
            or profile.get('status') != 'provisional_offline_only'
            or profile.get('controller_ready') is not False
            or not all(isinstance(v, (float, int)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in (a, b))
            or abs(a) < 1e-12 or model.get('phone_deg_domain') != [0, 180]
            or model.get('alpha_deg_domain') != [-180, 0]):
        raise ValueError('invalid provisional calibration profile')


def map_raw(profile, raw):
    validate_profile(profile)
    failure = dict(phone_deg=None, alpha_deg=None, status='invalid_raw')
    if isinstance(raw, bool) or not isinstance(raw, (float, int)) or not math.isfinite(raw) or raw != int(raw) or not 0 <= raw <= 1023:
        return failure
    model = profile['model']
    a, b = model['raw_per_deg'], model['raw_intercept']
    angle = (raw-b)/a
    low, high = model['phone_deg_domain']
    if not low <= angle <= high:
        return dict(phone_deg=None, alpha_deg=None, status='outside_calibrated_segment')
    return dict(phone_deg=angle, alpha_deg=angle-180, status='ok_provisional')


def derive_row(profile, row):
    if not telemetry_ok(row):
        return dict(phone_deg=None, alpha_deg=None, status='invalid_telemetry')
    try:
        raw = row['adc_raw10']
        if isinstance(raw, str):
            raw = int(raw)
    except (KeyError, TypeError, ValueError):
        return dict(phone_deg=None, alpha_deg=None, status='invalid_raw')
    return map_raw(profile, raw)


def derive_file(profile, source, output):
    validate_profile(profile)
    source, output = Path(source), Path(output)
    if source.resolve() == output.resolve() or output.exists():
        raise FileExistsError(f'output already exists or aliases input: {output}')
    counts = {}
    with source.open(newline='', encoding='utf-8-sig') as src:
        reader = csv.DictReader(src)
        if not reader.fieldnames or any(k in reader.fieldnames for k in DERIVED_FIELDS):
            raise ValueError('missing CSV header or already derived input')
        with output.open('x', newline='', encoding='utf-8') as target:
            writer = csv.DictWriter(target, fieldnames=reader.fieldnames+list(DERIVED_FIELDS))
            writer.writeheader()
            for row in reader:
                result = derive_row(profile, row)
                counts[result['status']] = counts.get(result['status'], 0)+1
                row.update(phone_reference_deg_provisional=result['phone_deg'],
                           alpha_deg_provisional=result['alpha_deg'], angle_status=result['status'])
                writer.writerow(row)
    return counts


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    commands = cli.add_subparsers(dest='command', required=True)
    fit = commands.add_parser('fit')
    fit.add_argument('--training', type=Path, required=True)
    fit.add_argument('--validation', type=Path, required=True)
    fit.add_argument('--reference-resolution-deg', type=float, default=1)
    fit.add_argument('--output', type=Path, required=True)
    derive = commands.add_parser('derive')
    derive.add_argument('--profile', type=Path, required=True)
    derive.add_argument('--input', type=Path, required=True)
    derive.add_argument('--output', type=Path, required=True)
    args = cli.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(f'output already exists: {args.output}')
        if args.command == 'fit':
            profile = build_profile(load_round(args.training), load_round(args.validation),
                                    args.reference_resolution_deg)
            with args.output.open('x', encoding='utf-8') as handle:
                json.dump(profile, handle, ensure_ascii=False, indent=2, allow_nan=False)
                handle.write('\n')
            print(json.dumps(dict(status=profile['status'], model=profile['model'],
                                  validation_max_deg=profile['validation']['max_abs_phone_residual_deg'])))
        else:
            print(json.dumps(derive_file(read_json(args.profile), args.input, args.output)))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps(dict(error=str(exc)), ensure_ascii=True), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
