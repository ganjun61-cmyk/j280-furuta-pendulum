#!/usr/bin/env python3
"""J280 balance v4 / swing-up v5 board session: decode, read-only capture, config/keepalive run.

There is no host START. `run` loads one CONFIG (and, for a v5 swing profile, the
SWING block), keeps it alive and waits for the person at the board: DIP ON, then
SW1 while holding the pendulum upright (v4) or while it hangs still (v5). Every
exit path sends ABORT and waits for a fresh stopped frame.
"""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import sys
import time

import j280_balance as profiles
import j280_balance_protocol as protocol
import j280_swing_protocol as protocol5
import j280_v6_protocol as protocol6

TELEMETRY_TIMEOUT_S = 0.5
CUE_PERIOD_S = 0.25
RUN_GRACE_S = 0.5
ROW_FIELDS = (
    'packet_seq', 'raw_seq', 'tick_us', 'tick_us_unwrapped', 'state', 'stop_reason',
    'output_enabled', 'config_loaded', 'config_id', 'config_crc', 'adc_raw10', 'adc_valid',
    'adc_age_us', 'enc_count', 'theta_ref', 'upright_zero_q16', 'applied_compare',
    'motor_mode', 'u_linear_q16', 'u_compensated_q16', 'u_model_q16',
    'xhat0', 'xhat1', 'xhat2', 'xhat3', 'controller_age_us', 'controller_latency_cycles',
    'fault_bits', 'raw_estimator_quality', 'rx_status', 'input_levels', 'arm_on', 'core_ok',
    'state_elapsed_ms', 'safety_position_delta', 'epoch', 'packet_seq_gap', 'duplicate',
    'out_of_order', 'reset', 'reset_or_reorder', 'time_continuity_error',
    'sample_seq_continuity_error')
CSV_FIELDS = ('host_time_iso',) + ROW_FIELDS
SWING_FIELDS = ('swing_est_a', 'swing_est_ad', 'swing_energy_q16', 'swing_ref_q16',
                'swing_command', 'swing_blind', 'swing_flags', 'lash_offset')
V5_BLOCKS = ('swing', 'backlash', 'gains2')
CSV_FIELDS5 = CSV_FIELDS + SWING_FIELDS
CSV_FIELDS6 = CSV_FIELDS + SWING_FIELDS[:-1] + protocol6.TRAJ_FIELDS


def wire_of(config):
    if config is not None and 'traj' in config:
        return protocol6
    return protocol5 if config is not None and any(k in config for k in V5_BLOCKS) else protocol


def fields_of(wire):
    return CSV_FIELDS6 if wire is protocol6 else CSV_FIELDS5 if wire is protocol5 else CSV_FIELDS


def stopped(row):
    return (row['state'] in ('SAFE', 'FAULT') and not row['output_enabled']
            and not row['config_loaded'])


def wire_issues(stats, min_frames=1):
    """Captured-wire quality only; a boundary partial frame at each end is allowed."""
    issues = []
    if stats['frames_valid'] < min_frames:
        issues.append(f"only {stats['frames_valid']} valid frames")
    for key in ('frames_invalid', 'packet_seq_missed', 'duplicates', 'out_of_order',
                'reset_or_reorder', 'resets', 'continuity_errors'):
        if stats[key]:
            issues.append(f'{key}={stats[key]}')
    if stats['discarded_bytes'] > protocol.FRAME_SIZE - 1:
        issues.append(f"discarded_bytes={stats['discarded_bytes']}")
    return issues


class Session:
    """Only continuous rows may drive live decisions; a reset ends the session."""

    def __init__(self, clock):
        self.clock = clock
        self.accepted = 0
        self.latest = None
        self.error = None
        self.stop_row = None
        self.stop_at = None

    def accept(self, row):
        if row['reset'] or row['reset_or_reorder']:
            self.error = self.error or (f"telemetry continuity lost (board reset?) at "
                                        f"packet_seq={row['packet_seq']}")
            self.stop_row = None
            return False
        if self.error or any(row[k] for k in ('duplicate', 'out_of_order', 'time_continuity_error',
                                              'sample_seq_continuity_error')):
            return False
        self.accepted += 1
        self.latest = row
        self.stop_row, self.stop_at = (row, self.clock()) if stopped(row) else (None, None)
        return True

    def fresh_stop(self):
        if (not self.error and self.stop_row is not None
                and self.clock() - self.stop_at <= TELEMETRY_TIMEOUT_S):
            return self.stop_row
        return None


class Recorder:
    def __init__(self, writer, raw, clock=time.monotonic, wire=protocol, keep_rows=False):
        self.writer, self.raw = writer, raw
        self.kept = [] if keep_rows else None
        self.fields = fields_of(wire)
        self.stream = wire.FrameStream()
        self.session = Session(clock)
        self.events = []
        self.rows = 0
        self.balance_frames = 0
        self.swing_frames = 0
        self.output_frames = 0
        self.enc_range = None
        self.adc_range = None
        self._key = None

    @property
    def latest(self):
        return self.session.latest

    def feed(self, chunk, host_time=''):
        if self.raw:
            self.raw.write(chunk)
        accepted = []
        for row in self.stream.feed(chunk):
            for i, value in enumerate(row['xhat']):
                row[f'xhat{i}'] = value
            row['host_time_iso'] = host_time
            self.writer.writerow({k: row.get(k) for k in self.fields})
            if self.kept is not None:
                self.kept.append({k: row.get(k) for k in self.fields})
            self.rows += 1
            self.output_frames += row['output_enabled']
            self.balance_frames += row['state'] == 'BALANCE'
            self.swing_frames += row['state'] == 'SWING'
            e, a = row['enc_count'], row['adc_raw10']
            self.enc_range = (e, e) if self.enc_range is None else (min(self.enc_range[0], e), max(self.enc_range[1], e))
            self.adc_range = (a, a) if self.adc_range is None else (min(self.adc_range[0], a), max(self.adc_range[1], a))
            if not self.session.accept(row):
                continue
            accepted.append(row)
            key = (row['state'], row['stop_reason'], row['output_enabled'], row['config_loaded'])
            if key != self._key:
                self.events.append({k: row[k] for k in (
                    'packet_seq', 'tick_us_unwrapped', 'state', 'stop_reason', 'output_enabled',
                    'config_loaded', 'enc_count', 'adc_raw10', 'applied_compare', 'theta_ref')})
                self._key = key
        if self.session.error:
            raise RuntimeError(self.session.error)
        return accepted


def _now():
    return datetime.now(timezone.utc).isoformat(timespec='microseconds')


def read_chunk(port):
    # CH340 here can block for the full request even with a short timeout;
    # drain queued bytes only, or one byte when empty (see motor-test host fix).
    return port.read(max(1, min(4096, getattr(port, 'in_waiting', 0))))


def read_for(port, recorder, seconds, until=None, on_rows=None, clock=time.monotonic):
    deadline = clock() + seconds
    while (remaining := deadline - clock()) > 0:
        port.timeout = min(0.01, remaining)
        chunk = read_chunk(port)
        if chunk:
            rows = recorder.feed(chunk, _now())
            if on_rows:
                on_rows(rows)
            if until and any(until(r) for r in rows):
                return True
    return False


def wait_stopped(port, recorder, seconds, clock=time.monotonic, prior=None):
    ok = read_for(port, recorder, seconds, clock=clock,
                  until=lambda r: r is recorder.latest and stopped(r))
    if not ok and not (prior is not None and recorder.session.stop_row is prior):
        last = recorder.latest
        detail = 'no usable telemetry' if last is None else (
            f"state={last['state']} output={last['output_enabled']} loaded={last['config_loaded']}")
        raise RuntimeError(f'ABORT stop not confirmed within {seconds:g} s ({detail})')
    return recorder.latest


_TONE_CACHE = {}


def tone(frequency_hz, milliseconds, volume=0.9):
    """Audible cue through the default output device.

    winsound.Beep needs a PC speaker; this machine succeeds silently instead
    (2026-10-19: the board operator heard nothing). A short generated WAV played
    asynchronously goes to the sound card. 0.9 volume and >=300 ms are what the
    operator could hear reliably.
    """
    import math
    import struct
    import tempfile
    import wave
    import winsound
    key = (int(frequency_hz), int(milliseconds))
    path = _TONE_CACHE.get(key)
    if path is None:
        rate, amp = 44100, int(32767 * volume)
        total = max(1, rate * key[1] // 1000)
        ramp = max(1, rate // 100)
        frames = bytearray()
        for i in range(total):
            gain = min(1.0, i / ramp, (total - i) / ramp)
            frames += struct.pack('<h', int(amp * gain * math.sin(2 * math.pi * key[0] * i / rate)))
        path = Path(tempfile.gettempdir()) / f'j280-tone-{key[0]}-{key[1]}.wav'
        with wave.open(str(path), 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(bytes(frames))
        _TONE_CACHE[key] = path
    winsound.PlaySound(str(path), winsound.SND_FILENAME | winsound.SND_ASYNC)


CUE_IN, CUE_SHORT, CUE_PAST = 1800, 1000, 450
CUE_SWING, CUE_CATCH = (1000, 150), (1800, 400)


def capture_cue(row, enter_code, lift_sign=1):
    """(frequency_hz, duration_ms) guiding the holder during CAPTURE; None otherwise.

    High tone: inside the entry window. Middle tone: not yet vertical (keep
    lifting). Low tone: past vertical (ease back). lift_sign is the ADC change
    direction while lifting from hanging to upright (+1 on this rig: 220 -> 734).
    Uses the board's own ADC and the loaded upright zero; advisory only.
    """
    if row['state'] != 'CAPTURE':
        return None
    offset = (row['adc_raw10'] * 65536 - row['upright_zero_q16']) / 65536 * lift_sign
    if abs(offset) <= enter_code:
        return CUE_IN, 80
    return (CUE_PAST if offset > 0 else CUE_SHORT), 60


def nonblocking(beep):
    """Run a blocking beep(frequency, ms) in a daemon thread; skip if one is still sounding."""
    import threading
    state = {'thread': None}

    def play(frequency, duration):
        thread = state['thread']
        if thread is not None and thread.is_alive():
            return
        state['thread'] = threading.Thread(target=beep, args=(frequency, duration), daemon=True)
        state['thread'].start()
    return play


def run_session(port, recorder, config, wait_start_s, observe_s, keepalive_period_s,
                stop_keepalive_after_balance_ms=None, clock=time.monotonic, cue=None, wire=None,
                cancel=None):
    """Load, keep alive, wait for the local takeover and the board's own stop.

    wire defaults to the profile's protocol; pass j280_swing_protocol to run a
    plain balance profile (manual CAPTURE, no SWING block) on v5 firmware.
    cancel, if given, is polled; when it returns True the session sends ABORT.
    """
    wire = wire or wire_of(config)
    if wire is protocol and any(k in config for k in V5_BLOCKS):
        raise ValueError('swing/backlash/gains2 blocks need the v5 protocol')
    if wire is not protocol6 and 'traj' in config:
        raise ValueError('trajectory profiles need the v6 protocol')
    if wire is protocol6 and 'backlash' in config:
        raise ValueError('the v6 protocol has no backlash fields')
    values = config['values']
    if wire is protocol6:
        extra = {'traj': config.get('traj')}
    else:
        extra = {'backlash': config['backlash']} if 'backlash' in config else {}
    command = wire.build_command('config', values, **extra)
    expected_crc = struct.unpack_from('<H', command, 190)[0]
    session = recorder.session
    result = dict(config_crc=expected_crc, phase='sync', balance_seen=False)
    if not read_for(port, recorder, 2.0, until=lambda r: session.accepted >= 5, clock=clock):
        raise RuntimeError('no valid telemetry within 2 s')
    if recorder.latest['state'] != 'SAFE':
        raise RuntimeError(f"board not SAFE: {recorder.latest['state']} "
                           f"({recorder.latest['stop_reason']}); press SW5 if FAULT")
    if recorder.latest['config_loaded']:
        port.write(wire.build_command('abort'))
        wait_stopped(port, recorder, 1.0, clock=clock)
    if ('swing' in config or 'gains2' in config) and recorder.latest['arm_on']:
        # With the DIP already ON (and OFF seen earlier) the board arms on CONFIG
        # alone, before the SWING/GAINS2 block arrives, and then rejects it (swing-673).
        result['phase'] = 'wait_dip_off'
        if not read_for(port, recorder, wait_start_s, clock=clock,
                        until=lambda r: r is recorder.latest and not r['arm_on']):
            raise RuntimeError('DIP 1 stayed ON; turn it OFF before loading a swing/gains2 profile')
    result['phase'] = 'config'
    session.stop_row = None
    port.write(command)

    def echo(r):
        return r['config_loaded'] and r['config_id'] == values['config_id'] and r['config_crc'] == expected_crc

    seen = []
    if not read_for(port, recorder, 1.0, until=echo, on_rows=seen.extend, clock=clock):
        raise RuntimeError(f"configuration not echoed (rx_status=0x{recorder.latest['rx_status']:02x})")
    pending = seen[next(i for i, r in enumerate(seen) if echo(r)):]
    if 'swing' in config:
        result['phase'] = 'swing_config'
        port.write(wire.build_command('swing', config['swing']))
        seen = []
        if not read_for(port, recorder, 1.0, until=lambda r: echo(r) and r['swing_loaded'],
                        on_rows=seen.extend, clock=clock):
            raise RuntimeError(f"swing block not accepted (rx_status=0x{recorder.latest['rx_status']:02x})")
        pending = seen[next(i for i, r in enumerate(seen) if r['swing_loaded']):]
    if 'gains2' in config:
        result['phase'] = 'gains2_config'
        port.write(wire.build_command('gains2', dict(config['gains2'], config_id=values['config_id'])))
        seen = []
        need_swing = 'swing' in config
        if not read_for(port, recorder, 1.0, on_rows=seen.extend, clock=clock,
                        until=lambda r: echo(r) and r['gains2_loaded'] and (r['swing_loaded'] or not need_swing)):
            raise RuntimeError(f"gains2 block not accepted (rx_status=0x{recorder.latest['rx_status']:02x})")
        pending = seen[next(i for i, r in enumerate(seen) if r['gains2_loaded']):]
    result['phase'] = 'wait_start'
    loaded_at = last_keepalive = last_telemetry = clock()
    balance_at = swing_at = None
    ended = None
    last_cue = -CUE_PERIOD_S
    # run_ms 0 (v6): no time limit; the board stops on SW2, DIP off or a fault.
    run_deadline = values['run_ms'] / 1000 + RUN_GRACE_S if values['run_ms'] else math.inf
    swing_deadline = config['swing']['swing_ms'] / 1000 + RUN_GRACE_S if 'swing' in config else None
    while ended is None:
        now = clock()
        if cancel is not None and cancel():
            result['phase'] = 'cancelled'
            port.write(wire.build_command('abort'))
            ended = wait_stopped(port, recorder, 0.5, clock=clock)
            break
        if now - last_telemetry >= TELEMETRY_TIMEOUT_S:
            raise RuntimeError(f"telemetry lost for {TELEMETRY_TIMEOUT_S:g} s during {result['phase']}")
        if balance_at is not None and now - balance_at >= run_deadline:
            raise RuntimeError('BALANCE exceeded run_ms plus grace without a board stop')
        if swing_at is not None and balance_at is None and now - swing_at >= swing_deadline:
            raise RuntimeError('SWING exceeded swing_ms plus grace without a board stop')
        if pending:
            rows, pending = pending, []
        else:
            suppress = (stop_keepalive_after_balance_ms is not None and balance_at is not None
                        and now - balance_at >= stop_keepalive_after_balance_ms / 1000)
            if now - last_keepalive >= keepalive_period_s and not suppress:
                port.write(wire.build_command('keepalive'))
                last_keepalive = now
            port.timeout = 0.005
            chunk = read_chunk(port)
            rows = recorder.feed(chunk, _now()) if chunk else []
        for r in rows:
            last_telemetry = clock()
            if r['state'] in ('ARMED', 'CAPTURE') and result['phase'] == 'wait_start':
                result['phase'] = r['state'].lower()
            if r['state'] == 'SWING' and swing_at is None:
                swing_at = clock()
                result.update(phase='swing', swing_start_enc=r['enc_count'], swing_start_adc=r['adc_raw10'])
                if cue is not None:
                    cue(*CUE_SWING)
            if r['state'] == 'BALANCE' and balance_at is None:
                balance_at = clock()
                result.update(phase='balance', balance_seen=True,
                              takeover_enc=r['enc_count'], takeover_adc=r['adc_raw10'])
                if swing_at is not None:
                    result['catch_s'] = round(balance_at - swing_at, 3)
                    if cue is not None:
                        cue(*CUE_CATCH)
            if stopped(r):
                ended = r
        if cue is not None and rows and clock() - last_cue >= CUE_PERIOD_S:
            tone = capture_cue(rows[-1], values['enter_code'])
            if tone is not None:
                cue(*tone)
                last_cue = clock()
        if ended is None and balance_at is None and swing_at is None and clock() - loaded_at > wait_start_s:
            result['phase'] = 'start_timeout'
            port.write(wire.build_command('abort'))
            ended = wait_stopped(port, recorder, 0.5, clock=clock)
    result.update(end_state=ended['state'], end_reason=ended['stop_reason'],
                  end_tick_us=ended['tick_us_unwrapped'],
                  balance_s=None if balance_at is None else round(clock() - balance_at, 3))
    read_for(port, recorder, observe_s, clock=clock)
    return result


def run_with_cleanup(port, recorder, config, wait_start_s, observe_s, keepalive_period_s,
                     stop_keepalive_after_balance_ms=None, cue=None, wire=None, cancel=None,
                     clock=time.monotonic):
    """run_session, then always ABORT and confirm a fresh stopped frame; returns the summary."""
    wire = wire or wire_of(config)
    result = {}
    try:
        result = run_session(port, recorder, config, wait_start_s, observe_s, keepalive_period_s,
                             stop_keepalive_after_balance_ms, clock=clock, cue=cue, wire=wire, cancel=cancel)
    finally:
        primary = sys.exc_info()[1]
        prior = recorder.session.fresh_stop()
        errors = []
        try:
            port.write(wire.build_command('abort'))
        except (OSError, ValueError, RuntimeError) as error:
            errors.append(f'ABORT write: {error}')
        confirmed = False
        try:
            wait_stopped(port, recorder, 0.3, clock=clock, prior=prior)
            confirmed = True
        except (OSError, ValueError, RuntimeError) as error:
            errors.append(f'ABORT confirm: {error}')
        summary = summarize(recorder, dict(config=config, result=result, stop_confirmed=confirmed))
        if errors:
            summary['cleanup_errors'] = errors
        recorder.last_summary = summary          # kept for error reports when run_session raised
        if errors and primary is None:
            raise RuntimeError('; '.join(errors))
    return summary


def balance_stats(rows, upright_code=737, settle_s=1.0):
    """Steady BALANCE quality from telemetry rows (dicts as written to the CSV).

    pend/arm std over BALANCE after settle_s; fast_pend_std is the deviation from a
    50 ms moving average (the audible 8-20 Hz backlash buzz lives there).
    """
    import statistics

    def num(r, k):
        v = r[k]
        return int(v) if not isinstance(v, bool) else v
    bal = [r for r in rows if r['state'] == 'BALANCE']
    out = dict(balance_frames=len(bal))
    if not bal:
        return out
    t0 = num(bal[0], 'tick_us_unwrapped')
    tail = [r for r in bal if num(r, 'tick_us_unwrapped') - t0 >= settle_s * 1e6]
    if len(tail) < 20:
        return out
    a = [num(r, 'adc_raw10') - upright_code for r in tail]
    e = [num(r, 'enc_count') for r in tail]
    duty = [(1 if num(r, 'motor_mode') == 2 else -1 if num(r, 'motor_mode') == 1 else 0)
            * num(r, 'applied_compare') / 25 for r in tail]
    smooth = [sum(a[max(0, i - 2):i + 3]) / len(a[max(0, i - 2):i + 3]) for i in range(len(a))]
    out.update(seconds=round((num(tail[-1], 'tick_us_unwrapped') - t0) / 1e6, 2),
               pend_std_code=round(statistics.pstdev(a), 2), pend_range_code=[min(a), max(a)],
               fast_pend_std_code=round(statistics.pstdev([x - y for x, y in zip(a, smooth)]), 2),
               arm_std_count=round(statistics.pstdev(e), 1), arm_range_count=[min(e), max(e)],
               duty_rms_percent=round((sum(x * x for x in duty) / len(duty)) ** 0.5, 1),
               saturated_fraction=round(sum(abs(x) >= 39.99 for x in duty) / len(duty), 3))
    return out


def summarize(recorder, extra=None):
    s = recorder.stream.stats
    summary = dict(stats=s, wire_issues=wire_issues(s), wire_ok=not wire_issues(s),
                   frames=recorder.rows, balance_frames=recorder.balance_frames,
                   swing_frames=recorder.swing_frames,
                   output_frames=recorder.output_frames, enc_range=recorder.enc_range,
                   adc_range=recorder.adc_range, events=recorder.events)
    if recorder.latest:
        summary['final'] = {k: recorder.latest[k] for k in
                            ('state', 'stop_reason', 'config_loaded', 'output_enabled', 'enc_count', 'adc_raw10')}
    if recorder.session.error:
        summary['session_error'] = recorder.session.error
    summary.update(extra or {})
    return summary


def open_outputs(stack, output, raw_output, fields=CSV_FIELDS):
    paths = [output] + ([raw_output] if raw_output else [])
    if len({p.resolve() for p in paths}) != len(paths):
        raise ValueError('outputs must be different paths')
    if any(p.exists() for p in paths):
        raise FileExistsError('output already exists')
    writer = csv.DictWriter(stack.enter_context(output.open('x', newline='', encoding='utf-8')),
                            fieldnames=fields)
    writer.writeheader()
    raw = stack.enter_context(raw_output.open('xb')) if raw_output else None
    return writer, raw


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    make = sub.add_parser('make-config', help='write an isolated logic profile')
    make.add_argument('--upright-code', type=float, required=True)
    make.add_argument('--config-id', type=int, default=1)
    make.add_argument('--output', type=Path, required=True)
    swing = sub.add_parser('make-swing-config', help='v5 profile: a v4 profile plus a compiled SWING block')
    swing.add_argument('--base', type=Path, required=True)
    swing.add_argument('--params', type=json.loads, required=True, help='float swing-up parameters (JSON)')
    swing.add_argument('--config-id', type=int, required=True)
    swing.add_argument('--exit-code', type=int, default=80)
    swing.add_argument('--options', type=json.loads, default={},
                       help='SWING limits for compile_params, e.g. {"swing_ms": 6000}')
    swing.add_argument('--output', type=Path, required=True)
    decode = sub.add_parser('decode')
    decode.add_argument('--input', type=Path, required=True)
    capture = sub.add_parser('capture', help='read only; never writes to the port')
    capture.add_argument('--seconds', type=float, required=True)
    run = sub.add_parser('run', help='load one CONFIG; takeover only by SW1 on the board')
    for p in (decode, capture, run):
        p.add_argument('--v5', action='store_true',
                       help='v5 firmware: v5 telemetry; for run, also v5 commands for a plain balance profile')
        p.add_argument('--v6', action='store_true',
                       help='v6 firmware: v6 telemetry; for run, also v6 commands for a plain balance profile')
    run.add_argument('--config', type=Path, required=True)
    run.add_argument('--wait-start-s', type=float, default=120.0)
    run.add_argument('--observe-s', type=float, default=2.0)
    run.add_argument('--keepalive-period-ms', type=float, default=50.0)
    run.add_argument('--stop-keepalive-after-balance-ms', type=float)
    run.add_argument('--confirm-powered', action='store_true')
    run.add_argument('--no-cue', action='store_true', help='disable CAPTURE guidance tones')
    for p in (capture, run):
        p.add_argument('--port', required=True)
        p.add_argument('--baud', type=int, default=230400)
    for p in (decode, capture, run):
        p.add_argument('--output', type=Path, required=True)
        p.add_argument('--raw-output', type=Path)
    args = parser.parse_args(argv)
    summary, recorder = {}, None
    try:
        if args.command == 'make-config':
            if args.output.exists():
                raise FileExistsError('output already exists')
            profile = profiles.build_isolated_config(args.upright_code, args.config_id)
            args.output.write_text(json.dumps(profile, indent=2) + '\n', encoding='utf-8')
            profiles.load_config(args.output)
            print(json.dumps(dict(written=str(args.output), config_id=args.config_id)))
            return 0
        if args.command == 'make-swing-config':
            if args.output.exists():
                raise FileExistsError('output already exists')
            profile = profiles.build_swing_config(profiles.load_config(args.base), args.params,
                                                  args.config_id, args.exit_code, **args.options)
            args.output.write_text(json.dumps(profile, indent=2) + chr(10), encoding='utf-8')
            profiles.load_config(args.output)
            print(json.dumps(dict(written=str(args.output), config_id=args.config_id)))
            return 0
        config = None
        if args.command == 'run':
            config = profiles.load_config(args.config)
            if config['power'] == 'powered' and not args.confirm_powered:
                raise ValueError('configuration is marked powered; add --confirm-powered')
            if not 0 < args.keepalive_period_ms < config['values']['keepalive_ms'] / 2:
                raise ValueError('keepalive period must be below half the board timeout')
            for name in ('wait_start_s', 'observe_s'):
                if not (math.isfinite(getattr(args, name)) and getattr(args, name) >= 0):
                    raise ValueError(f'{name} must be finite and non-negative')
        if args.command == 'capture' and not (math.isfinite(args.seconds) and args.seconds > 0):
            raise ValueError('seconds must be finite and positive')
        with ExitStack() as stack:
            wire = (protocol6 if getattr(args, 'v6', False) else
                    protocol5 if getattr(args, 'v5', False) else wire_of(config))
            writer, raw = open_outputs(stack, args.output, args.raw_output, fields_of(wire))
            recorder = Recorder(writer, raw, wire=wire)
            if args.command == 'decode':
                source = stack.enter_context(args.input.open('rb'))
                while chunk := source.read(65536):
                    recorder.feed(chunk)
                summary = summarize(recorder)
            else:
                import serial
                port = stack.enter_context(serial.Serial(port=args.port, baudrate=args.baud,
                                                         timeout=0.01, write_timeout=0.2))
                if args.command == 'capture':
                    read_for(port, recorder, args.seconds)
                    summary = summarize(recorder)
                else:
                    cue = None
                    if not args.no_cue:
                        try:
                            import winsound  # noqa: F401
                            cue = tone
                        except ImportError:
                            pass
                    summary = run_with_cleanup(port, recorder, config, args.wait_start_s, args.observe_s,
                                               args.keepalive_period_ms / 1000,
                                               args.stop_keepalive_after_balance_ms, cue=cue, wire=wire)
        print(json.dumps(summary, allow_nan=False, default=str))
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, KeyboardInterrupt) as error:
        if recorder is not None and not summary:
            summary = getattr(recorder, 'last_summary', None) or summarize(recorder)
        print(json.dumps(dict(error=str(error) or type(error).__name__, partial=summary or None),
                         ensure_ascii=True, default=str), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
