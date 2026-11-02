"""Offline J280 swing-up v5 wire protocol: the v4 balance protocol plus SWING and GAINS2.

Commands keep the 192-byte v4 layout with version 5. CONFIG is the v4 CONFIG
(exit_code may reach 90 codes, wide enough for a catch at 20 degrees) and
carries the backlash fields at wire bytes 170/172/174 (wire bytes 176..189 must
stay zero; all three zero = no compensation). SWING (command 4) and
GAINS2 (command 5: the 12 observer and feedback gains, coefficients 20..31,
switched in after switch_ms of BALANCE) are accepted only in SAFE after a CONFIG with the same
config_id; any later CONFIG or ABORT clears them. Telemetry keeps every v4
field at the same offset; bytes 104..124 carry the swing core state, byte 125
the backlash tracker offset.
"""
import struct

import j280_backlash as backlash_math
import j280_balance_protocol as v4
import j280_swing_math as sm
from j280_measurement import crc16_ccitt

VERSION = 5
COMMAND_SIZE = v4.COMMAND_SIZE
COMMANDS = dict(v4.COMMANDS, swing=4, gains2=5)
STATES = dict(v4.STATES)
STATES[4] = 'SWING'
STOP_REASONS = dict(v4.STOP_REASONS)
STOP_REASONS.update({7: 'swing_timeout', 25: 'blind_limit', 26: 'blind_entry', 27: 'arm_stall'})
VALUE_FIELDS = v4.VALUE_FIELDS
VALUE_RANGES = dict(v4.VALUE_RANGES, exit_code=(1, 90))
SWING_FORMAT = '<' + ''.join(f[1] for f in sm.FIELDS)
SWING_FLAGS = ('swing_loaded', 'swing_good', 'swing_catch', 'swing_blind_mid',
               'gap_boosted', 'gains2_loaded', 'gains2_active')
BACKLASH_OFFSET = 170          # wire byte of lash_half_count (payload bits 1344+); boost, threshold follow
GAINS2_SWITCH_MS = (1, 30000)
GAINS2_COUNT = 12               # observer L (coefficients 20..27) and feedback K (28..31)
COEFF_LIMIT = 128 << 20


def validate_values(values):
    return v4.validate_values(values, VALUE_RANGES)


def validate_gains2(block, config_id=None):
    if not isinstance(block, dict) or not {'switch_ms', 'coefficients'} <= set(block):
        raise ValueError('gains2 needs switch_ms and coefficients')
    ms, co = block['switch_ms'], block['coefficients']
    if type(ms) is not int or not GAINS2_SWITCH_MS[0] <= ms <= GAINS2_SWITCH_MS[1]:
        raise ValueError(f'gains2 switch_ms must be an integer in {GAINS2_SWITCH_MS[0]}..{GAINS2_SWITCH_MS[1]}')
    if (not isinstance(co, list) or len(co) != GAINS2_COUNT
            or any(type(c) is not int or abs(c) > COEFF_LIMIT for c in co)):
        raise ValueError('gains2 needs 12 integer coefficients (observer and feedback gains) within +-2^27')
    return dict(switch_ms=ms, coefficients=list(co), config_id=config_id)


def build_command(name, values=None, *, backlash=None):
    if name == 'swing':
        v = sm.validate(values)
        data = bytearray(COMMAND_SIZE)
        data[:4] = v4.SYNC + bytes((VERSION, COMMANDS['swing']))
        struct.pack_into(SWING_FORMAT, data, 4, *(v[f[0]] for f in sm.FIELDS))
    elif name == 'gains2':
        g = validate_gains2(values, values.get('config_id'))
        if type(g['config_id']) is not int or not 1 <= g['config_id'] <= 65535:
            raise ValueError('gains2 config_id must be an integer in 1..65535')
        data = bytearray(COMMAND_SIZE)
        data[:4] = v4.SYNC + bytes((VERSION, COMMANDS['gains2']))
        struct.pack_into('<HH12i', data, 4, g['config_id'], g['switch_ms'], *g['coefficients'])
    else:
        data = bytearray(v4.build_command(name, values, version=VERSION, ranges=VALUE_RANGES))
        if name == 'config' and backlash:
            b = backlash_math.validate(backlash)
            struct.pack_into('<3H', data, BACKLASH_OFFSET, *(b[f] for f in backlash_math.FIELDS))
        elif backlash:
            raise ValueError('backlash fields belong to CONFIG')
    struct.pack_into('<H', data, 190, crc16_ccitt(data[2:190]))
    return bytes(data)


def _decode(data):
    row = v4._decode(data, version=VERSION, states=STATES, reasons=STOP_REASONS,
                     active_states=(1, 2, 3, 4), drive_states=(3, 4), reserved=slice(126, 126))
    row.update(zip(('swing_est_a', 'swing_est_ad', 'swing_energy_q16', 'swing_ref_q16',
                    'swing_command', 'swing_blind', 'swing_flags', 'lash_offset'),
                   struct.unpack_from('<iiiihHBb', data, 104)))
    if row['swing_flags'] & 0x80 or (row['state_code'] == 4 and not row['swing_flags'] & 1):
        raise ValueError('domain_errors')
    for bit, name in enumerate(SWING_FLAGS):
        row[name] = bool(row['swing_flags'] & (1 << bit))
    if (abs(row['swing_command']) > VALUE_RANGES['max_compare'][1] or abs(row['lash_offset']) > 15
            or (row['gains2_active'] and not (row['gains2_loaded'] and row['state_code'] == 3))):
        raise ValueError('domain_errors')
    return row


class FrameStream(v4.FrameStream):
    def __init__(self):
        super().__init__(_decode)
