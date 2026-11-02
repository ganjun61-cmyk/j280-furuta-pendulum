"""Offline J280 v6 wire protocol: v5 (SWING, GAINS2) with the arm trajectory.

Commands keep the 192-byte layout with version 6. CONFIG carries the ten
trajectory fields of scripts/j280_traj.py at wire bytes 170..189 (payload bytes
168..187, where v5 had the backlash fields; all zero = no motion and no
integral) and run_ms may be 0 (no time limit). CONFIG algorithm_id selects the
alpha-rate feedback filter: 1 bypass, 2/3/4/5 two cascaded one-pole sections at
3/5/8/10 Hz, 6/7 one section at 5/8 Hz, and 8/9/10/11 the same controller with
a 4/6/8/10-count dead zone on the arm-error feedback, and 12/13/14 the
arm-side disturbance observer (5 Hz gain 1, 10 Hz gain 1, 10 Hz gain 1.5).
Telemetry keeps the v5 layout;
the v4 constant words now carry the trajectory: wire bytes 46..47 the status
(bit 0 moving, 1 rotating, 2 standalone, 3 settled, 4 SW3 pressed, 5 SW4
pressed, 6 DIP 2 on, bits 8..15 target index),
84..87 the reference speed (Q16 counts per sample) and 88..91 the feedforward
(Q16 compare). theta_ref is the integer part of the moving reference. Byte 125
(the v5 backlash offset) and swing flag bit 4 are reserved zero.

Modes 15/16 are the lean firmware, 17 the PIP-lite firmware, and 18/19 the
hybrid firmware (19 adds the 2 Hz arm-rate damping). On the keyboard firmware
(hybrid 18/19), while BALANCE the swing_est_a/ad/energy words carry the keypad
state instead of the swing estimator: entry[8:0], speed_sel[10:9], dir[11],
state[13:12], last_key[17:14], plus key_delta and key_speed in the next words.
The decoder exposes them as key_entry/key_speed_sel/key_dir/key_state/key_last/
key_delta/key_speed_q16.
"""
import struct

import j280_balance_protocol as v4
import j280_swing_math as sm
import j280_swing_protocol as v5
import j280_traj as traj_math
from j280_measurement import crc16_ccitt

VERSION = 6
COMMAND_SIZE = v4.COMMAND_SIZE
COMMANDS = v5.COMMANDS
STATES = v5.STATES
STOP_REASONS = v5.STOP_REASONS
VALUE_FIELDS = v4.VALUE_FIELDS
VALUE_RANGES = dict(v5.VALUE_RANGES, run_ms=(0, 30000), algorithm_id=(1, 19))
TRAJ_OFFSET = 170
SWING_FORMAT = v5.SWING_FORMAT
SWING_FLAGS = ('swing_loaded', 'swing_good', 'swing_catch', 'swing_blind_mid', None,
               'gains2_loaded', 'gains2_active')
TRAJ_STATUS = ('traj_moving', 'traj_rotating', 'standalone', 'traj_settled',
               'sw3_pressed', 'sw4_pressed', 'dip2_on')
TRAJ_FIELDS = ('traj_status', 'traj_index', 'traj_moving', 'traj_rotating', 'standalone',
               'traj_settled', 'sw3_pressed', 'sw4_pressed', 'dip2_on', 'ref_velocity_q16', 'u_ff_q16')


def validate_values(values):
    return v4.validate_values(values, VALUE_RANGES)


def _signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def build_command(name, values=None, *, traj=None):
    data = bytearray(COMMAND_SIZE)
    if name == 'swing':
        v = sm.validate(values)
        data[:4] = v4.SYNC + bytes((VERSION, COMMANDS['swing']))
        struct.pack_into(SWING_FORMAT, data, 4, *(v[f[0]] for f in sm.FIELDS))
    elif name == 'gains2':
        g = v5.validate_gains2(values, values.get('config_id'))
        if type(g['config_id']) is not int or not 1 <= g['config_id'] <= 65535:
            raise ValueError('gains2 config_id must be an integer in 1..65535')
        data[:4] = v4.SYNC + bytes((VERSION, COMMANDS['gains2']))
        struct.pack_into('<HH12i', data, 4, g['config_id'], g['switch_ms'], *g['coefficients'])
    else:
        data = bytearray(v4.build_command(name, values, version=VERSION, ranges=VALUE_RANGES))
        if name == 'config':
            t = traj_math.validate(traj or {})
            struct.pack_into('<10H', data, TRAJ_OFFSET, *(t[f] for f in traj_math.FIELDS))
        elif traj:
            raise ValueError('trajectory fields belong to CONFIG')
    struct.pack_into('<H', data, 190, crc16_ccitt(data[2:190]))
    return bytes(data)


def _decode(data):
    row = v4._decode(data, version=VERSION, states=STATES, reasons=STOP_REASONS,
                     active_states=(1, 2, 3, 4), drive_states=(3, 4), reserved=slice(125, 126),
                     constants=False)
    row.update(zip(('swing_est_a', 'swing_est_ad', 'swing_energy_q16', 'swing_ref_q16',
                    'swing_command', 'swing_blind', 'swing_flags'),
                   struct.unpack_from('<iiiihHB', data, 104)))
    flags = row['swing_flags']
    if flags & 0x90 or (row['state_code'] == 4 and not flags & 1):
        raise ValueError('domain_errors')
    for bit, name in enumerate(SWING_FLAGS):
        if name:
            row[name] = bool(flags & (1 << bit))
    status = row['pwm_period']
    row['traj_status'] = status
    row['traj_index'] = _signed(status >> 8, 8)
    for bit, name in enumerate(TRAJ_STATUS):
        row[name] = bool(status & (1 << bit))
    row['ref_velocity_q16'] = _signed(row['sample_period_us'], 32)
    row['u_ff_q16'] = _signed(row['telemetry_period_us'], 32)
    # In BALANCE the v6 swing_est_* words carry the keypad entry (mode 18/19
    # firmware): entry 9 bits, speed 2, dir 1, state 2, last key 4.
    if row['state_code'] == 3:
        a = row['swing_est_a']
        row['key_entry'] = a & 0x1ff
        row['key_speed_sel'] = (a >> 9) & 3
        row['key_dir'] = (a >> 11) & 1
        row['key_state'] = (a >> 12) & 3
        row['key_last'] = (a >> 14) & 0xf
        row['key_delta'] = _signed(row['swing_est_ad'] & 0xffff, 16)
        row['key_speed_q16'] = row['swing_energy_q16'] & 0xffff
    else:
        for name in ('key_entry', 'key_speed_sel', 'key_dir', 'key_state', 'key_last',
                     'key_delta', 'key_speed_q16'):
            row[name] = 0
    row['pwm_period'], row['pwm_period_count'] = v4.PWM_PERIOD, v4.PWM_PERIOD
    row['sample_period_us'], row['telemetry_period_us'] = v4.SAMPLE_PERIOD_US, v4.TELEMETRY_PERIOD_US
    limit = VALUE_RANGES['max_compare'][1]
    if (status & 0x80 or abs(row['swing_command']) > limit or abs(row['u_ff_q16']) > limit << 16
            or abs(row['ref_velocity_q16']) > 65535
            or (row['gains2_active'] and not (row['gains2_loaded'] and row['state_code'] == 3))
            or ((row['traj_moving'] or row['traj_rotating'] or row['traj_settled']) and row['state_code'] != 3)
            or (row['standalone'] and not row['config_loaded'] and row['state_code'] != 0)):
        raise ValueError('domain_errors')
    return row


class FrameStream(v4.FrameStream):
    def __init__(self):
        super().__init__(_decode)
