#!/usr/bin/env python3
"""Offline J280 raw-window estimates; no serial, motor or controller operations.

Positions use encoder tick and ADC (tick - adc_age_us), respectively. Negative
ADC source times at the clock origin are rejected. Velocities are secants over
reported endpoints, timestamped at their midpoint; age is relative to row tick.
Unwrapped ticks must be in 0..MAX_TIMESTAMP_US (2**52-1 microseconds), retaining
exact half-microsecond midpoints. Outside this domain rows are invalid and clear
both histories with reason invalid:tick_us_unwrapped before any estimation.
Channels warm independently, but every rejected row clears BOTH histories.
Valid continuity/session/epoch boundaries seed a new segment without bridging.
Encoder unwrapping is segment-local, anchored to its first signed int32 count.
Invalid numerical estimates are None (empty CSV cells), never invented zeros.
"""

import argparse
from collections import Counter, deque
from collections.abc import Mapping
import csv
import json
from pathlib import Path
import re
import sys


VERSION = 'j280-raw-window-v1'
MOD = 2**32
# Binary64 spacing remains at most 0.5 us throughout this supported time domain.
MAX_TIMESTAMP_US = 2**52-1
TRUE_FIELDS = ('adc_valid', 'adc_new', 'frame_valid', 'crc_ok')
FALSE_FIELDS = ('buffer_overflow', 'duplicate', 'out_of_order', 'reset', 'reset_or_reorder')
INTEGER_BOUNDS = {
    'seq': (0, MOD-1), 'tick_us': (0, MOD-1),
    'tick_us_unwrapped': (0, MAX_TIMESTAMP_US), 'epoch': (0, None),
    'adc_raw10': (0, 1023), 'adc_age_us': (0, 65535),
    'enc_count': (-2**31, 2**31-1), 'fault_bits': (0, 0), 'seq_gap': (0, 0),
}
REQUIRED_FIELDS = ('session_id',) + tuple(INTEGER_BOUNDS) + TRUE_FIELDS + FALSE_FIELDS
CONFIG_BOUNDS = {
    'window_us': (1, None), 'expected_period_us': (1, 2**31-1),
    'max_adc_age_us': (0, 65535), 'max_adc_step': (0, 1023),
    'max_encoder_step': (0, 2**31-1),
}
VELOCITY_TIME_FIELDS = ('start_us', 'end_us', 'midpoint_us', 'span_us', 'age_us')
ESTIMATE_FIELDS = (
    'encoder_position_count', 'adc_position_code',
    'encoder_position_time_us', 'adc_position_time_us',
    'encoder_velocity_count_s', 'adc_velocity_code_s',
) + tuple(f'{channel}_velocity_{field}' for channel in ('encoder', 'adc')
          for field in VELOCITY_TIME_FIELDS)
DERIVED_FIELDS = ('estimator_version', 'controller_ready', 'segment_id', 'status',
                  'reason', 'position_valid', 'encoder_velocity_valid',
                  'adc_velocity_valid') + ESTIMATE_FIELDS
LIMITATIONS = [
    'Raw counts/codes only; no angular calibration, filtering or controller readiness.',
    'Window secants refer to their own midpoints, not instantaneous or aligned velocities.',
    'Continuity and thresholds reject data; they do not establish physical sensor accuracy.',
    'Encoder shortest modular steps assume motion below half the int32 range per sample.',
    'Negative ADC source times are rejected; encoder origins restart at each segment.',
    f'Unwrapped ticks outside 0..{MAX_TIMESTAMP_US} us are rejected for midpoint precision.',
]


def _integer(value, name, low, high, strings=True):
    if strings and isinstance(value, str) and re.fullmatch(r'-?[0-9]+', value):
        value = int(value)
    if type(value) is not int or value < low or (high is not None and value > high):
        raise ValueError(f'invalid:{name}')
    return value


def _validated(row):
    if not isinstance(row, Mapping) or None in row:
        raise ValueError('invalid:row_schema')
    for name in REQUIRED_FIELDS:
        if name not in row:
            raise ValueError(f'missing:{name}')
    if not isinstance(row['session_id'], str) or not row['session_id'].strip():
        raise ValueError('invalid:session_id')
    result = {'session_id': row['session_id']}
    for name, (low, high) in INTEGER_BOUNDS.items():
        result[name] = _integer(row[name], name, low, high)
    for names, expected in ((TRUE_FIELDS, True), (FALSE_FIELDS, False)):
        for name in names:
            value = row[name]
            if not (value is expected or (type(value) is str and value == str(expected))):
                raise ValueError(f'invalid:{name}')
    tick = result['tick_us_unwrapped']
    if tick % MOD != result['tick_us']:
        raise ValueError('invalid:tick_modulo')
    result['adc_time'] = tick - result['adc_age_us']
    if result['adc_time'] < 0:
        raise ValueError('invalid:negative_adc_time')
    return result


class RawWindowEstimator:
    """Strict integer configuration; update accepts decoded mappings or CSV rows.

    status=ready iff both velocities have full windows; warming positions remain
    valid. Boundary reasons identify rows used as fresh seeds. All invalid rows
    discard their own samples. segment_id starts at 1 on the first valid seed.
    """

    def __init__(self, window_us, expected_period_us, max_adc_age_us,
                 max_adc_step, max_encoder_step):
        values = dict(window_us=window_us, expected_period_us=expected_period_us,
                      max_adc_age_us=max_adc_age_us, max_adc_step=max_adc_step,
                      max_encoder_step=max_encoder_step)
        self.config = {key: _integer(values[key], key, *bounds, strings=False)
                       for key, bounds in CONFIG_BOUNDS.items()}
        self._encoder_history = deque()
        self._adc_history = deque()
        self._previous = None
        self._encoder_count = None
        self.segment_id = 0

    def _clear(self):
        self._encoder_history.clear()
        self._adc_history.clear()
        self._previous = None
        self._encoder_count = None

    def _blank(self, reason):
        result = dict.fromkeys(ESTIMATE_FIELDS)
        result.update(estimator_version=VERSION, controller_ready=False,
                      segment_id=self.segment_id, status='invalid', reason=reason,
                      position_valid=False, encoder_velocity_valid=False,
                      adc_velocity_valid=False)
        return result

    def _velocity(self, history, tick, channel, units, result):
        cutoff = history[-1][0] - self.config['window_us']
        # Retain the newest eligible endpoint and all newer samples. This is
        # bounded by (window + max ADC age)/period, independent of file length.
        while len(history) > 1 and history[1][0] <= cutoff:
            history.popleft()
        start, first = history[0]
        end, last = history[-1]
        if start > cutoff:
            return
        span = end - start
        midpoint = start + span / 2
        prefix = f'{channel}_velocity_'
        result.update({prefix+'valid': True, prefix+units: (last-first)/span*1e6,
                       prefix+'start_us': start, prefix+'end_us': end,
                       prefix+'midpoint_us': midpoint, prefix+'span_us': span,
                       prefix+'age_us': tick-midpoint})

    def update(self, row):
        """Return a flat derived dictionary; never mutate the supplied row."""
        try:
            sample = _validated(row)
            if sample['adc_age_us'] > self.config['max_adc_age_us']:
                raise ValueError('invalid:adc_age')
            previous, reason = self._previous, ''
            delta = 0
            if previous is not None:
                if sample['session_id'] != previous['session_id']:
                    reason = 'boundary:session'
                elif sample['epoch'] != previous['epoch']:
                    reason = 'boundary:epoch'
                elif sample['seq'] != (previous['seq']+1) % MOD:
                    reason = 'boundary:seq'
                elif (sample['tick_us_unwrapped']-previous['tick_us_unwrapped']
                      != self.config['expected_period_us']):
                    reason = 'boundary:tick_step'
                if reason:
                    self._clear()
                else:
                    delta = (sample['enc_count']-previous['enc_count']+2**31) % MOD-2**31
                    if abs(delta) > self.config['max_encoder_step']:
                        raise ValueError('invalid:encoder_step')
                    if sample['adc_time'] <= previous['adc_time']:
                        raise ValueError('invalid:adc_source_time')
                    if abs(sample['adc_raw10']-previous['adc_raw10']) > self.config['max_adc_step']:
                        raise ValueError('invalid:adc_step')
            if self._previous is None:
                self.segment_id += 1
                self._encoder_count = sample['enc_count']
                reason = reason or 'segment_start'
            else:
                self._encoder_count += delta
            tick = sample['tick_us_unwrapped']
            self._encoder_history.append((tick, self._encoder_count))
            self._adc_history.append((sample['adc_time'], sample['adc_raw10']))
            self._previous = sample
            result = self._blank(reason)
            result.update(position_valid=True, encoder_position_count=self._encoder_count,
                          adc_position_code=sample['adc_raw10'], encoder_position_time_us=tick,
                          adc_position_time_us=sample['adc_time'])
            self._velocity(self._encoder_history, tick, 'encoder', 'count_s', result)
            self._velocity(self._adc_history, tick, 'adc', 'code_s', result)
            ready = result['encoder_velocity_valid'] and result['adc_velocity_valid']
            result['status'] = 'ready' if ready else 'warming'
            result['reason'] = reason or ('window_complete' if ready else 'window_incomplete')
            return result
        except ValueError as exc:
            self._clear()
            return self._blank(str(exc))


def replay_file(input_path, output_path, config):
    """Append derived columns to a new CSV; refuse collisions before creating it.

    Original column names, order and cell values are preserved (CSV quoting may
    change). Missing/bad cells invalidate rows; extra unnamed cells reject the
    file. Failed writes remove only the output exclusively created by this call.
    """
    estimator = RawWindowEstimator(**config)
    source, output = Path(input_path), Path(output_path)
    if source.resolve() == output.resolve() or output.exists() or output.is_symlink():
        raise FileExistsError(f'output exists or aliases input: {output}')
    counts, reasons = Counter(), Counter()
    with source.open(newline='', encoding='utf-8-sig') as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames
        if (not fields or any(not name.strip() for name in fields)
                or len(set(fields)) != len(fields)
                or not set(REQUIRED_FIELDS).issubset(fields)
                or set(DERIVED_FIELDS).intersection(fields)):
            raise ValueError('invalid, missing, duplicate or reserved CSV header')
        target = output.open('x', newline='', encoding='utf-8')
        try:
            with target:
                writer = csv.DictWriter(target, fieldnames=fields+list(DERIVED_FIELDS))
                writer.writeheader()
                for row in reader:
                    if None in row:
                        raise ValueError('CSV row has extra unnamed cells')
                    result = estimator.update(row)
                    counts[result['status']] += 1
                    reasons[result['reason']] += 1
                    writer.writerow(row | result)
        except BaseException:
            output.unlink()
            raise
    return dict(rows=sum(counts.values()), counts=dict(counts), reason_counts=dict(reasons),
                config=estimator.config.copy(), estimator_version=VERSION,
                controller_ready=False, limitations=LIMITATIONS.copy())


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    cli.add_argument('--input', type=Path, required=True)
    cli.add_argument('--output', type=Path, required=True)
    for key in CONFIG_BOUNDS:
        cli.add_argument('--'+key.replace('_', '-'), type=int, required=True)
    args = cli.parse_args(argv)
    config = {key: getattr(args, key) for key in CONFIG_BOUNDS}
    try:
        summary = replay_file(args.input, args.output, config)
        print(json.dumps(summary, allow_nan=False))
    except (ValueError, TypeError, OSError, csv.Error) as exc:
        print(json.dumps(dict(error=str(exc)), ensure_ascii=True, allow_nan=False), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
