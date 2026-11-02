"""v6 arm reference trajectory and feedforward: the bit-exact spec of rtl/j280_swing/j280_traj.v.

All values are integers. Positions are Q16 encoder counts, speeds Q16 counts per
2 ms sample, accelerations Q16 counts per sample squared, outputs Q16 PWM compare
(theta/alpha references in Q16 counts / Q16 ADC code).

The reference moves on a grid of `step_counts` around the balance anchor (the
takeover/swing-start arm position). Its speed is always m*A (m = 0, 1, 2, ...),
so the stopping distance D(m) = A*m*(m-1)/2 is kept incrementally and the
hardware needs no multiplier for the profile. Each sample chooses one of
accelerate / cruise / decelerate; below one acceleration step from the target
the reference snaps onto it.

Flat-output lean (linearized alpha'' = w0^2 alpha - beta theta''): while the
reference accelerates with sign s the pendulum is referenced at s*lean_pend and
the arm at r - s*lean_arm (both in 1/256 units), so the controller does not
fight the lean that the acceleration needs.

Feedforward (added before the friction compensation; the observer model input
excludes it): u_ff = trunc0(v*kv, 2) + acc*ka + clip(trunc0(i*ki, 4), +-U_I_MAX),
clipped to +-max_compare. The integral enters with a plus sign: with the balance
loop closed a constant input u_d settles the arm at -u_d/|K_theta|, so a positive
arm error needs a positive bias. The integral i sums e = arm - floor(r) in counts, only
after the settle time, with the reference resting on its target and |e| <=
i_window (integral separation), and is clamped to +-I_ACC_MAX (anti-windup).
"""

S = 1 << 16
LIMITS = dict(step_counts=(0, 1040), move_speed=(0, 65535), rotate_speed=(0, 65535),
              accel=(0, 4095), kv=(0, 65535), ka=(0, 65535), ki=(0, 65535),
              lean_arm=(0, 16383), lean_pend=(0, 16383), i_window=(0, 1040))
FIELDS = tuple(LIMITS)
CONFIG_OFFSET = 168          # CONFIG payload byte of the first field (v6)
MAX_QUEUE_STEPS = 4
I_ACC_MAX = 1 << 20
U_I_MAX = 250 * S
SETTLE_MS = 3000


def trunc0(value, shift):
    return -((-value) >> shift) if value < 0 else value >> shift


def validate(values):
    values = dict(values or {})
    unknown = set(values) - set(FIELDS)
    if unknown:
        raise ValueError(f'unknown trajectory fields: {sorted(unknown)}')
    out = {}
    for name in FIELDS:
        v = values.get(name, 0)
        lo, hi = LIMITS[name]
        if type(v) is not int or not lo <= v <= hi:
            raise ValueError(f'{name} must be an integer in [{lo}, {hi}]')
        out[name] = v
    if out['step_counts']:
        if not 1 <= out['accel'] <= out['move_speed']:
            raise ValueError('with step_counts set, need 1 <= accel <= move_speed')
        if out['rotate_speed'] and out['rotate_speed'] < out['accel']:
            raise ValueError('rotate_speed must be 0 or >= accel')
    return out


class Trajectory:
    def __init__(self, params, anchor_counts, max_compare=1000):
        self.p = validate(params)
        self.max_compare = max_compare
        self.r = anchor_counts * S
        self.m = self.speed = self.stop = 0
        self.dir = 1
        self.target = anchor_counts
        self.index = 0
        self.rotating = False
        self.last_dir = 1
        self.rot_gear = 1
        self.i_acc = 0

    def _accelerate(self, direction):
        self.dir = direction
        self.stop += self.speed
        self.speed += self.p['accel']
        self.m += 1
        return direction

    def _decelerate(self):
        self.m -= 1
        self.speed -= self.p['accel']
        self.stop -= self.speed
        return -self.dir

    def step(self, *, ccw=False, cw=False, rotate=False, settled=False, arm=0,
             rot_dir_cw=False, rot_dir_ccw=False, rot_gear_up=False, rot_gear_down=False):
        """One 2 ms sample; returns the references used by this sample's control."""
        p = self.p
        step, a = p['step_counts'], p['accel']
        if p['ki'] and settled and self.m == 0 and not self.rotating:
            e = arm - (self.r >> 16)
            if abs(e) <= p['i_window']:
                self.i_acc = max(-I_ACC_MAX, min(I_ACC_MAX, self.i_acc + e))
        enabled = step != 0 and settled
        if self.rotating and rotate:
            if rot_dir_cw:
                self.dir = 1; self.last_dir = 1
            if rot_dir_ccw:
                self.dir = -1; self.last_dir = -1
            if rot_gear_up and self.rot_gear < 2:
                self.rot_gear += 1
            if rot_gear_down and self.rot_gear > 0:
                self.rot_gear -= 1
        if self.rotating and not rotate:
            self.rotating = False
            g = self.target + self.dir * step
            if self.dir * (g * S - self.r) < self.stop + self.speed:
                g += self.dir * step
                self.index += self.dir
            self.target = g
            self.index += self.dir
        elif (enabled and rotate and not self.rotating and p['rotate_speed'] and self.m == 0
              and self.r == self.target * S):
            self.rotating = True
            self.dir = self.last_dir
            self.rot_gear = 1
        if enabled and not self.rotating:
            for pressed, d in ((ccw, -1), (cw, 1)):
                if pressed:
                    base = (self.r >> 16) if self.r == self.target * S else self.target
                    g = base + d * step
                    if abs(g * S - self.r) <= MAX_QUEUE_STEPS * step * S:
                        self.target = g
                        self.index += d
                        self.last_dir = d
        s = 0
        if self.rotating:
            rt = p['rotate_speed']
            if self.rot_gear == 0:
                rt >>= 1
            elif self.rot_gear == 2:
                rt += rt >> 1
            if self.speed + a <= rt:
                s = self._accelerate(self.dir)
            elif self.speed > rt:
                s = self._decelerate()
            nxt = self.target + self.dir * step
            if self.dir * (self.r + self.dir * self.speed - nxt * S) >= 0:
                self.target = nxt
                self.index += self.dir
        else:
            d = self.target * S - self.r
            sgn = (d > 0) - (d < 0)
            if self.m and sgn != self.dir:
                s = self._decelerate()
            elif d:
                ad = abs(d)
                if self.speed + a <= p['move_speed'] and ad >= self.stop + 2 * self.speed + a:
                    s = self._accelerate(sgn)
                elif self.m and ad >= self.stop + self.speed and self.speed <= p['move_speed']:
                    pass
                elif self.m:
                    s = self._decelerate()
                else:
                    self.r = self.target * S
        self.r += self.dir * self.speed
        v = self.dir * self.speed
        u_i = max(-U_I_MAX, min(U_I_MAX, trunc0(self.i_acc * p['ki'], 4)))
        bound = self.max_compare * S
        u_ff = max(-bound, min(bound, trunc0(v * p['kv'], 2) + s * a * p['ka'] + u_i))
        return dict(r=self.r, v=v, acc=s * a, u_ff=u_ff,
                    theta_ref=self.r - s * (p['lean_arm'] << 8), alpha_ref=s * (p['lean_pend'] << 8),
                    target=self.target, index=self.index, moving=self.m != 0, rotating=self.rotating)


def counts_per_sample_q16(deg_per_s, counts_per_rev=1040):
    return round(deg_per_s / 360 * counts_per_rev * 0.002 * S)


def accel_q16(deg_per_s2, counts_per_rev=1040):
    return round(deg_per_s2 / 360 * counts_per_rev * 0.002 * 0.002 * S)


def model_gains(accel, a=12.5, b=2.56, w0=9.724, beta=1.444, adc_per_rad=162.33, counts_per_rev=1040):
    """kv, ka, lean_arm, lean_pend from the identified design model and the accel field."""
    import math
    count_rad = 2 * math.pi / counts_per_rev
    rad_s_per_unit_v = count_rad / 0.002 / S
    rad_s2_per_unit_a = count_rad / 0.002 / 0.002 / S
    kv = a / b * rad_s_per_unit_v * 25 * S * 4
    ka = rad_s2_per_unit_a / b * 25 * S
    acc = accel * rad_s2_per_unit_a
    lean_arm = acc / w0 ** 2 / count_rad * 256
    lean_pend = beta * acc / w0 ** 2 * adc_per_rad * 256
    return round(kv), round(ka), round(lean_arm), round(lean_pend)


def default_params(move_dps=45, rotate_dps=60, accel_dps2=150, ki=0, i_window=0):
    accel = accel_q16(accel_dps2)
    kv, ka, lean_arm, lean_pend = model_gains(accel)
    return validate(dict(step_counts=260, move_speed=counts_per_sample_q16(move_dps),
                         rotate_speed=counts_per_sample_q16(rotate_dps), accel=accel, kv=kv, ka=ka, ki=ki,
                         lean_arm=lean_arm, lean_pend=lean_pend, i_window=i_window))
