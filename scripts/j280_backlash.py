"""Bit-exact v5 backlash compensation: the specification of rtl/j280_swing/j280_backlash.v.

The encoder sits on the motor shaft, so inside the gearbox gap the motor turns
while the arm does not. The tracker keeps an arm-position estimate a (int32
counts) that only moves when the motor presses one side of the gap:
    a <- clip32(clamp(a, pos - half, pos + half))   while balancing
    a <- pos                                        otherwise (and when half = 0)
The balance core measures a instead of pos. offset = pos - a is where the motor
sits inside the gap (+half: pressing the + side).

The gap boost adds `gap_boost` compare counts in the command direction while
the motor has not reached that side yet (offset < half for +, > -half for -),
only for |command| >= max(threshold, 1), then clips to +-max_compare. The
balance core's command and model input exclude the boost: the boost is spent
crossing the gap, not accelerating the arm. half = 0 or boost = 0 disables it.
"""
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1
LIMITS = dict(lash_half_count=(0, 15), gap_boost_compare=(0, 250), gap_threshold_compare=(0, 250))
FIELDS = tuple(LIMITS)


def validate(values):
    if not isinstance(values, dict) or set(values) - set(FIELDS):
        raise ValueError('backlash values must be a subset of ' + ', '.join(FIELDS))
    out = {}
    for name, (lo, hi) in LIMITS.items():
        v = values.get(name, 0)
        if type(v) is not int or not lo <= v <= hi:
            raise ValueError(f'{name} must be an integer in {lo}..{hi}')
        out[name] = v
    return out


def track(a, pos, half, balancing):
    """(next arm estimate, offset = pos - estimate) after one 2 ms sample."""
    if not balancing or half == 0:
        return pos, 0
    a = max(I32_MIN, min(I32_MAX, min(max(a, pos - half), pos + half)))
    return a, pos - a


def boost(command, offset, half, gap_boost, threshold, max_compare):
    """(applied command, boost applied?) for one balance-core command."""
    if half == 0 or gap_boost == 0 or abs(command) < max(threshold, 1):
        return command, False
    if command > 0 and offset < half:
        return min(command + gap_boost, max_compare), True
    if command < 0 and offset > -half:
        return max(command - gap_boost, -max_compare), True
    return command, False


class CompensatedController:
    """j280_balance_math.FixedController behind the tracker and gap boost, for
    j280_balance_design.simulate(controller=...). Optionally switches the 12
    observer/feedback coefficients (20..31) after switch_s of balancing (v5 GAINS2)."""

    def __init__(self, coefficients, half=0, gap_boost=0, threshold=0, gains2=None, switch_s=None):
        import math
        import j280_balance_math as bm
        self.core = bm.FixedController(coefficients)
        self.gains2, self.switch_steps = gains2, None if switch_s is None else round(switch_s / 0.002)
        self.count_rad = 2 * math.pi / 1040
        self.half, self.gap_boost, self.threshold = half, gap_boost, threshold
        self.a = None
        self.steps = self.boosted = 0

    def __call__(self, y_si, case):
        if self.gains2 is not None and self.steps == self.switch_steps:
            self.core.coefficients = self.core.coefficients[:20] + list(self.gains2)
        self.steps += 1
        pos = round(y_si[0] / self.count_rad)
        self.a, offset = track(pos if self.a is None else self.a, pos, self.half, True)
        out = self.core((self.a * self.count_rad, y_si[1]), case)
        if out.get('fault'):
            return out
        command = round(out['u_applied_percent'] * 25)
        applied, on = boost(command, offset, self.half, self.gap_boost, self.threshold, round(case['u_max'] * 25))
        self.boosted += on
        out = dict(out)
        out['u_applied_percent'] = applied / 25
        return out
