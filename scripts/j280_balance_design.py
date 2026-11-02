#!/usr/bin/env python3
"""Upright balance design for J280: discrete LQR, observer, nonlinear simulation.

Units: theta, alpha in rad (alpha from upright, ADC-increasing positive; theta
count-increasing positive), u in percent duty (positive = BIN2 = count up).
Design model (from the 2026-10-17 identification):
    theta'' = -a theta' + b u
    alpha'' = w0^2 alpha - beta theta'' - c alpha'
The simulation adds what the design model leaves out: per-direction motor
gain/time constant, kinetic friction and static breakaway, motor-to-arm gear
backlash, pendulum Coulomb friction and centrifugal term, encoder and ADC
quantization, ADC noise, ADC scale/zero errors, saturation and control period.
"""
import argparse
import json
import math
from pathlib import Path
import random
import sys

COUNT_RAD = 2 * math.pi / 1040
ADC_PER_RAD = 162.33
T = 0.002
FIRMWARE_ALPHA_LIMIT_DEG = 20.0
FIRMWARE_ARM_LIMIT_COUNTS = 200

# Identified 2026-10-17 (docs/data/2026-10-17-motor-powered-check.md).
MOTOR = {'neg': dict(k=29.3 * COUNT_RAD, tau=0.0663, d0=6.67, d_break=12.5),
         'pos': dict(k=38.21 * COUNT_RAD, tau=0.0945, d0=7.62, d_break=15.5)}
PENDULUM = dict(beta=1.444, w0=9.724, c=0.085, mu=0.84)
DESIGN = dict(a=1 / 0.080, b=(MOTOR['neg']['k'] / MOTOR['neg']['tau']
                              + MOTOR['pos']['k'] / MOTOR['pos']['tau']) / 2)


# ---- small dense linear algebra -------------------------------------------
def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
            for i in range(len(a))]


def tr(a):
    return [list(r) for r in zip(*a)]


def add(a, b, s=1.0):
    return [[x + s * y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def eye(n):
    return [[float(i == j) for j in range(n)] for i in range(n)]


def inv(a):
    n = len(a)
    m = [list(r) + e for r, e in zip(a, eye(n))]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        p = m[col][col]
        m[col] = [x / p for x in m[col]]
        for r in range(n):
            if r != col and m[r][col]:
                f = m[r][col]
                m[r] = [x - f * y for x, y in zip(m[r], m[col])]
    return [r[n:] for r in m]


def expm(a):
    norm = max(sum(abs(x) for x in r) for r in a)
    squarings = max(0, int(math.ceil(math.log2(norm))) + 1) if norm > 0.5 else 0
    s = [[x / 2 ** squarings for x in r] for r in a]
    result, term = eye(len(a)), eye(len(a))
    for k in range(1, 20):
        term = [[x / k for x in r] for r in mm(term, s)]
        result = add(result, term)
    for _ in range(squarings):
        result = mm(result, result)
    return result


# ---- model, LQR, observer --------------------------------------------------
def continuous(a=None, b=None, beta=None, w0=None, c=None):
    a = DESIGN['a'] if a is None else a
    b = DESIGN['b'] if b is None else b
    beta = PENDULUM['beta'] if beta is None else beta
    w0 = PENDULUM['w0'] if w0 is None else w0
    c = PENDULUM['c'] if c is None else c
    A = [[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, -a, 0], [0, w0 * w0, beta * a, -c]]
    B = [[0], [0], [b], [-beta * b]]
    return A, B


def discretize(A, B, dt=T):
    n = len(A)
    big = [list(r) + list(rb) for r, rb in zip(A, B)] + [[0.0] * (n + 1)]
    e = expm([[x * dt for x in r] for r in big])
    return [r[:n] for r in e[:n]], [[r[n]] for r in e[:n]]


def dare(A, B, Q, R, iterations=60):
    """X = A'XA - A'XB (R + B'XB)^-1 B'XA + Q by the structured doubling algorithm.

    Doubling avoids the many value-iteration steps needed for slow modes."""
    n = len(A)
    Ak, Gk, Hk = A, mm(mm(B, inv(R)), tr(B)), Q
    for _ in range(iterations):
        W = inv(add(eye(n), mm(Gk, Hk)))
        AW = mm(Ak, W)
        A_next = mm(AW, Ak)
        G_next = add(Gk, mm(mm(AW, Gk), tr(Ak)))
        H_next = add(Hk, mm(mm(mm(tr(Ak), Hk), W), Ak))
        change = max(abs(x - y) for rn, ro in zip(H_next, Hk) for x, y in zip(rn, ro))
        Ak, Gk, Hk = A_next, G_next, H_next
        if change <= 1e-12 * max(abs(x) for r in Hk for x in r):
            return Hk
    raise ValueError('Riccati doubling did not converge')


def dlqr(Ad, Bd, Q, R):
    X = dare(Ad, Bd, Q, [[R]])
    BtX = mm(tr(Bd), X)
    return [x / (R + mm(BtX, Bd)[0][0]) for x in mm(BtX, Ad)[0]]


def kalman(Ad, C, Qw, Rv):
    """Steady-state predictor-corrector gain L (n x m): x += L (y - C x_pred)."""
    Pp = dare(tr(Ad), tr(C), Qw, Rv)
    return mm(mm(Pp, tr(C)), inv(add(mm(mm(C, Pp), tr(C)), Rv)))


def matrix_power_radius_estimate(M, steps=4000):
    """Finite-power growth estimate, including transients; not exact eigenvalues."""
    P, scale = eye(len(M)), 0.0
    for _ in range(steps):
        P = mm(P, M)
        norm = max(abs(x) for r in P for x in r)
        scale += math.log(norm)
        P = [[x / norm for x in r] for r in P]
    return math.exp(scale / steps)


def spectral_radius(M, steps=4000):
    """Compatibility alias for matrix_power_radius_estimate, not exact rho."""
    return matrix_power_radius_estimate(M, steps)


# ---- nonlinear simulation ---------------------------------------------------
def inverse_compensation(applied, comp=7.0, blend=1.0):
    """Virtual percent input implied by actual signed duty, after PWM/clipping.

    Inverts v + comp * clip(v / blend, -1, 1), including its linear blend.
    This is the actuator-map inverse, not an estimate of physical friction.
    """
    if not all(math.isfinite(x) for x in (applied, comp, blend)) or comp < 0 or blend <= 0:
        raise ValueError('duty/comp/blend must be finite, comp >= 0 and blend > 0')
    if abs(applied) <= blend + comp:
        return applied / (1 + comp / blend)
    return applied - math.copysign(comp, applied)


def applied_duty(value, comp=7.0, blend=1.0, u_max=40.0, pwm_step=0.0):
    """Compensate, saturate, then optionally quantize signed duty in percent.

    PWM rounds to nearest, ties away from zero, with a grid limit rounded down
    so quantization never exceeds u_max. A zero step disables quantization.
    """
    inverse_compensation(value, comp, blend)  # Validate the compensation domain.
    if not all(math.isfinite(x) for x in (u_max, pwm_step)) or not 0 < u_max <= 100 or pwm_step < 0:
        raise ValueError('u_max must be in (0, 100] and pwm_step finite and >= 0')
    duty = value + comp * max(-1.0, min(1.0, value / blend))
    duty = max(-u_max, min(u_max, duty))
    if pwm_step:
        ticks = min(math.floor(abs(duty) / pwm_step + 0.5), math.floor(u_max / pwm_step))
        duty = math.copysign(ticks * pwm_step, duty)
    return duty


def velocity_compensation(velocity, comp, v0, viscous=0.0):
    """Smooth Coulomb compensation in the direction of the measured arm velocity."""
    if not math.isfinite(velocity) or not math.isfinite(v0) or v0 <= 0:
        raise ValueError('velocity compensation needs finite velocity and v0 > 0')
    if not math.isfinite(comp) or comp < 0 or not math.isfinite(viscous) or viscous < 0:
        raise ValueError('comp must be >= 0 and viscous >= 0')
    return comp * math.tanh(velocity / v0) + viscous * velocity


def applied_duty_velocity(value, velocity, comp, v0, u_max=40.0, pwm_step=0.0, viscous=0.0):
    """Command plus a velocity-signed, smooth friction compensation.

    Returns (applied duty, compensation used). The compensation flips with the
    actual arm velocity instead of the command sign, and vanishes at rest.
    """
    if not math.isfinite(value) or not 0 < u_max <= 100 or pwm_step < 0:
        raise ValueError('u_max must be in (0, 100] and pwm_step finite and >= 0')
    comp_v = velocity_compensation(velocity, comp, v0, viscous)
    duty = max(-u_max, min(u_max, value + comp_v))
    if pwm_step:
        ticks = min(math.floor(abs(duty) / pwm_step + 0.5), math.floor(u_max / pwm_step))
        duty = math.copysign(ticks * pwm_step, duty)
    return duty, comp_v


def firmware_guard(enc, measured_alpha):
    """Snapshot protection reason, using measured rad and reference count zero."""
    if not math.isfinite(enc) or not math.isfinite(measured_alpha):
        return 'sensor_invalid'
    if abs(measured_alpha) > math.radians(FIRMWARE_ALPHA_LIMIT_DEG):
        return 'alpha_limit'
    if abs(enc) > FIRMWARE_ARM_LIMIT_COUNTS:
        return 'arm_count_limit'
    return None


def simulate(K, L, Ad, Bd, case, seconds=10.0, seed=1, *, mode='plant', substeps=8,
             controller=None, trace=None):
    """Release at rest; plant mode preserves legacy 25 deg/90 deg limits.

    Firmware mode also latches measured snapshot guards (20 deg/200 counts,
    reference zero) before control. Protection trips are not physical falls.
    `fell` retains the legacy physical-envelope meaning, including arm travel.
    `accepted` requires completion without either kind of stop. The model does
    not implement arming, watchdogs or active pendulum reaction on the arm.
    Metrics describe this integration grid, not a qualified continuous bound.

    Optional controller(y_si, case_cfg) owns its state and returns a dict with
    xhat_si, u_linear_percent, u_applied_percent, u_model_percent and fault.
    Measurements are (theta_rad, alpha_rad). A truthy fault stops immediately
    and may be returned alone. Applied duty is used directly, without a second
    compensation/PWM pass; the callback owns quantization and model inversion.
    K/L/Ad/Bd may be None in this mode. Create a fresh closure for every run.

    case['rho'] (default 0, the legacy model) adds the pendulum's reaction on the
    arm as in j280_swingup_sim.Plant; about 0.25 fits the 2026-10-18 swing-up runs.
    case['arm_limit_deg'] (default 90) is the plant arm envelope; the v6 arm turns freely.
    trace, if a list, receives (t, alpha_rad, arm_rad, applied_duty) every snapshot.
    """
    if mode not in ('plant', 'firmware'):
        raise ValueError('mode must be plant or firmware')
    if not math.isfinite(seconds) or seconds < T:
        raise ValueError(f'seconds must be finite and >= {T}')
    if not isinstance(substeps, int) or isinstance(substeps, bool) or substeps < 1:
        raise ValueError('substeps must be a positive integer')
    rng = random.Random(seed)
    g = dict(gain=1.0, tau=1.0, beta=1.0, w0=1.0, adc_scale=1.0, adc_zero_deg=0.0,
             backlash_counts=4, adc_noise=0.6, alpha0_deg=3.0, comp=7.0, u_max=40.0,
             tap=0.0, estimator='observer', friction=1.0, blend=1.0, pwm_step=0.0, rho=0.0,
             arm_limit_deg=90.0, comp_mode='command', comp_v0=0.1, comp_viscous=0.0,
             knock_v=0.05, knock_u=5.0, knock_amp=15.0, knock_ms=15.0,
             dither_amp=0.0, dither_hz=0.0, dob_bw=0.0, dob_gain=0.0)
    g.update(case)
    if g['estimator'] not in ('observer', 'window'):
        raise ValueError('estimator must be observer or window')
    if g['comp_mode'] not in ('command', 'velocity', 'knocker'):
        raise ValueError('comp_mode must be command, velocity or knocker')
    for name, value in g.items():
        if name not in ('estimator', 'comp_mode') and not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
    for name in ('gain', 'tau', 'beta', 'w0', 'adc_scale', 'blend', 'comp_v0', 'knock_v', 'knock_ms'):
        if g[name] <= 0:
            raise ValueError(f'{name} must be positive')
    for name in ('backlash_counts', 'adc_noise', 'comp', 'friction', 'pwm_step', 'rho',
                 'comp_viscous', 'knock_u', 'knock_amp', 'dither_amp', 'dither_hz',
                 'dob_bw', 'dob_gain'):
        if g[name] < 0:
            raise ValueError(f'{name} must be nonnegative')
    applied_duty(0, g['comp'], g['blend'], g['u_max'], g['pwm_step'])
    motor = {d: dict(m, k=m['k'] * g['gain'], tau=m['tau'] * g['tau'], d0=m['d0'] * g['friction'],
                     d_break=m['d_break'] * g['friction']) for d, m in MOTOR.items()}
    beta, w0 = PENDULUM['beta'] * g['beta'], PENDULUM['w0'] * g['w0']
    c, mu = PENDULUM['c'], PENDULUM['mu']
    half_lash = g['backlash_counts'] * COUNT_RAD / 2
    h = T / substeps
    rho = g['rho']
    th_m = th_a = th_dm = v_a = 0.0
    al = math.radians(g['alpha0_deg'])
    w = 0.0
    xh = [0.0, 0.0, 0.0, 0.0]
    hist = []
    u_model = u_cmd = 0.0
    max_arm, max_alpha = 0.0, abs(al)
    sat = control_steps = 0
    sq_alpha = sq_u = 0.0
    n_tail = 0
    steps = int(seconds / T)
    knock_until = -1.0
    knock_next = 0.0
    dob_d = dob_v_prev = 0.0
    dob_alpha = 1.0 - math.exp(-2.0 * math.pi * g['dob_bw'] * T) if g['dob_bw'] > 0 else 0.0

    def finish(termination, at_s, reason=None):
        result = dict(seed=seed, mode=mode, substeps=substeps, integration_step_s=h,
                      fell=termination == 'physical_fall',
                      protection_trip=termination == 'protection_trip',
                      accepted=termination == 'completed', termination=termination,
                      max_arm_deg=round(math.degrees(max_arm), 1),
                      max_alpha_deg=round(math.degrees(max_alpha), 2),
                      final_arm_deg=round(math.degrees(th_a), 1), tail_samples=n_tail,
                      control_steps=control_steps, saturated_fraction=round(sat / max(1, control_steps), 4))
        if termination != 'completed':
            result.update(at_s=round(at_s, 6), reason=reason)
        if n_tail:
            result.update(tail_alpha_rms_deg=round(math.degrees(math.sqrt(sq_alpha / n_tail)), 3),
                          tail_u_rms=round(math.sqrt(sq_u / n_tail), 2))
        return result

    for k in range(steps):
        if g['tap'] and k == int(2.0 / T):
            w += g['tap']
        # Measurement and control at the snapshot.
        enc = round(th_m / COUNT_RAD)
        adc = round(ADC_PER_RAD * g['adc_scale'] * al + rng.gauss(0, g['adc_noise']))
        y = [enc * COUNT_RAD, adc / ADC_PER_RAD + math.radians(g['adc_zero_deg'])]
        if mode == 'firmware':
            reason = firmware_guard(enc, y[1])
            if reason is not None:
                return finish('protection_trip', k * T, reason)
        dither = 0.0
        if controller is not None:
            output = controller(tuple(y), dict(g))
            if output.get('fault'):
                return finish('controller_fault', k * T, str(output['fault']))
            xh = list(output['xhat_si'])
            u_lin, u_cmd, u_model = (output[key] for key in
                                     ('u_linear_percent', 'u_applied_percent', 'u_model_percent'))
            if len(xh) != 4 or not all(math.isfinite(x) for x in xh + [u_lin, u_cmd, u_model]):
                raise ValueError('controller must return four finite states and finite duty values')
            if abs(u_cmd) > g['u_max']:
                raise ValueError('external applied duty exceeds case u_max')
        else:
            if g['estimator'] == 'observer':
                pred = [sum(Ad[i][j] * xh[j] for j in range(4)) + Bd[i][0] * u_model for i in range(4)]
                e = [y[0] - pred[0], y[1] - pred[1]]
                xh = [pred[i] + L[i][0] * e[0] + L[i][1] * e[1] for i in range(4)]
            else:
                hist.append(y)
                old = hist[max(0, len(hist) - 6)]
                span = (min(5, len(hist) - 1) or 1) * T
                xh = [y[0], y[1], (y[0] - old[0]) / span, (y[1] - old[1]) / span]
            if g['dob_bw'] > 0:
                acc = (xh[2] - dob_v_prev) / T
                model_acc = -DESIGN['a'] * xh[2] + DESIGN['b'] * u_model
                dob_d += dob_alpha * ((acc - model_acc) - dob_d)
                dob_v_prev = xh[2]
            u_lin = -sum(Ki * xi for Ki, xi in zip(K, xh))
            if g['dob_bw'] > 0:
                u_lin -= g['dob_gain'] * dob_d / DESIGN['b']
            if g['comp_mode'] == 'velocity':
                u_cmd, comp_v = applied_duty_velocity(u_lin, xh[2], g['comp'], g['comp_v0'],
                                                     g['u_max'], g['pwm_step'], g['comp_viscous'])
                u_model = u_cmd - comp_v
            elif g['comp_mode'] == 'knocker':
                now = k * T
                if abs(xh[2]) > g['knock_v']:
                    knock_until = -1.0
                elif knock_until < now and now >= knock_next and abs(u_lin) > g['knock_u']:
                    knock_until = now + g['knock_ms'] / 1000.0
                    knock_next = knock_until + g['knock_ms'] / 1000.0
                comp_v = g['knock_amp'] * (1.0 if u_lin >= 0 else -1.0) if now < knock_until else 0.0
                u_cmd = max(-g['u_max'], min(g['u_max'], u_lin + comp_v))
                if g['pwm_step']:
                    ticks = min(math.floor(abs(u_cmd) / g['pwm_step'] + 0.5),
                                math.floor(g['u_max'] / g['pwm_step']))
                    u_cmd = math.copysign(ticks * g['pwm_step'], u_cmd)
                u_model = u_cmd - comp_v
            else:
                u_cmd = applied_duty(u_lin, g['comp'], g['blend'], g['u_max'], g['pwm_step'])
                u_model = inverse_compensation(u_cmd, g['comp'], g['blend'])
            if g['dither_amp'] > 0 and g['dither_hz'] > 0:
                dither = g['dither_amp'] if math.sin(2 * math.pi * g['dither_hz'] * k * T) >= 0 \
                    else -g['dither_amp']
                u_cmd = max(-g['u_max'], min(g['u_max'], u_cmd + dither))
                u_model += dither
        if g['comp_mode'] in ('velocity', 'knocker') and controller is None:
            requested = u_lin + comp_v + dither
        else:
            requested = u_lin + g['comp'] * max(-1.0, min(1.0, u_lin / g['blend'])) + dither
        if abs(requested) > g['u_max']:
            sat += 1
        control_steps += 1
        # Plant between snapshots.
        for _ in range(substeps):
            m = motor['pos' if u_cmd > 0 else 'neg']
            drive = (m['k'] * u_cmd - th_dm) / m['tau']
            den = 1.0
            if rho:
                s_, co = math.sin(al), math.cos(al)
                ad = w - beta * co * v_a
                drive += (-2 * rho * s_ * co * ad * th_dm + rho * beta * s_ * ad * ad
                          - rho * beta * co * (w0 * w0 * s_ + s_ * co * th_dm * th_dm - c * ad
                                               - mu * math.tanh(ad / 0.02)))
                den = 1 + rho * s_ * s_ - rho * beta * beta * co * co
            if abs(th_dm) > 1e-4:
                f = motor['pos' if th_dm > 0 else 'neg']
                acc = (drive - f['k'] / f['tau'] * f['d0'] * math.copysign(1, th_dm)) / den
                new = th_dm + h * acc
                th_dm = 0.0 if new * th_dm < 0 else new
            else:
                f = motor['pos' if drive > 0 else 'neg']
                limit = f['k'] / f['tau'] * f['d_break']
                if abs(drive) > limit:
                    th_dm += h * (drive - f['k'] / f['tau'] * f['d0'] * math.copysign(1, drive)) / den
                else:
                    th_dm = 0.0
            th_m += h * th_dm
            new_a = min(max(th_a, th_m - half_lash), th_m + half_lash)
            d_a = new_a - th_a
            th_a = new_a
            v_a = d_a / h
            al_dot = w - beta * math.cos(al) * v_a
            wd = (w0 * w0 * math.sin(al) - c * al_dot - mu * math.tanh(al_dot / 0.02)
                  + math.sin(al) * math.cos(al) * v_a * v_a - beta * math.sin(al) * al_dot * v_a)
            w += h * wd
            al += h * w - beta * math.cos(al) * d_a
        if trace is not None:
            trace.append(((k + 1) * T, al, th_a, u_cmd))
        max_arm = max(max_arm, abs(th_a))
        max_alpha = max(max_alpha, abs(al))
        if abs(al) > math.radians(25) or abs(th_a) > math.radians(g['arm_limit_deg']):
            reason = 'plant_alpha_limit' if abs(al) > math.radians(25) else 'plant_arm_limit'
            return finish('physical_fall', (k + 1) * T, reason)
        if k >= steps // 2:
            sq_alpha += al * al
            sq_u += u_cmd * u_cmd
            n_tail += 1
    return finish('completed', steps * T)


def design(q, r, qw=(100.0, 60.0), rv_alpha_code=0.8):
    A, B = continuous()
    Ad, Bd = discretize(A, B)
    K = dlqr(Ad, Bd, [[q[i] if i == j else 0.0 for j in range(4)] for i in range(4)], r)
    Gw = [[0, 0], [0, 0], [T, 0], [0, T]]
    Qw = mm(mm(Gw, [[qw[0] ** 2, 0], [0, qw[1] ** 2]]), tr(Gw))
    Rv = [[COUNT_RAD ** 2 / 12, 0], [0, (rv_alpha_code / ADC_PER_RAD) ** 2]]
    L = kalman(Ad, [[1, 0, 0, 0], [0, 1, 0, 0]], Qw, Rv)
    closed = add(Ad, mm(Bd, [K]), -1)
    estimate = matrix_power_radius_estimate(closed)
    return dict(K=K, L=L, Ad=Ad, Bd=Bd, rho=estimate,
                matrix_power_radius_estimate=estimate, rho_kind='finite_matrix_power_estimate')


SWEEP = [
    ('nominal', {}),
    ('alpha0 -3 deg', dict(alpha0_deg=-3.0)),
    ('alpha0 +6 deg', dict(alpha0_deg=6.0)),
    ('motor gain -30%', dict(gain=0.7)),
    ('motor gain +30%', dict(gain=1.3)),
    ('motor tau +50%', dict(tau=1.5)),
    ('adc scale -20%', dict(adc_scale=0.8)),
    ('adc scale +20%', dict(adc_scale=1.2)),
    ('upright zero +1 deg', dict(adc_zero_deg=1.0)),
    ('upright zero -1 deg', dict(adc_zero_deg=-1.0)),
    ('beta -10%', dict(beta=0.9)),
    ('w0 +3%', dict(w0=1.03)),
    ('backlash 8 counts', dict(backlash_counts=8)),
    ('friction -30%', dict(friction=0.7)),
    ('friction +30%', dict(friction=1.3)),
    ('adc noise 1.2 code', dict(adc_noise=1.2)),
    ('no friction comp', dict(comp=0.0)),
    ('comp 10%', dict(comp=10.0)),
    ('tap 1 rad/s', dict(tap=1.0)),
    ('window estimator', dict(estimator='window')),
]
EXTRA = [
    ('release +8 deg', dict(alpha0_deg=8.0)),
    ('release -8 deg', dict(alpha0_deg=-8.0)),
    ('release +12 deg', dict(alpha0_deg=12.0)),
    ('tap 2 rad/s', dict(tap=2.0)),
    ('tap -3 rad/s', dict(tap=-3.0)),
    ('combined low', dict(gain=0.7, adc_scale=0.8, adc_zero_deg=1.0, backlash_counts=8, friction=1.3)),
    ('combined high', dict(gain=1.3, adc_scale=1.2, adc_zero_deg=-1.0, friction=0.7, tau=1.5)),
    ('u_max 30%', dict(u_max=30.0)),
]
INFORMATIVE = ('no friction comp', 'comp 10%', 'window estimator')
RUN_METRICS = ('max_arm_deg', 'max_alpha_deg', 'tail_alpha_rms_deg', 'tail_u_rms',
               'final_arm_deg', 'saturated_fraction', 'at_s')


def _metric_point(value, run):
    point = dict(value=value, seed=run['seed'])
    if 'case' in run:
        point['case'] = run['case']
    return point


def aggregate_runs(runs):
    """Retain every seed; compute each metric's extrema independently.

    `worst` is a compatibility representative (failure, then tail RMS), never
    the source of another metric's extrema. Missing tail metrics stay missing.
    """
    runs = list(runs)
    if not runs:
        raise ValueError('at least one run is required')
    extrema = {}
    for metric in RUN_METRICS:
        available = [r for r in runs if metric in r]
        if not available:
            continue
        points = {}
        for name, key in (('min', lambda r: r[metric]), ('max', lambda r: r[metric]),
                          ('max_abs', lambda r: abs(r[metric]))):
            run = (min if name == 'min' else max)(available, key=key)
            points[name] = _metric_point(run[metric], run)
        extrema[metric] = dict(points, runs_with_metric=len(available))
    return dict(runs=runs, failures=sum(not r['accepted'] for r in runs),
                falls=sum(r['fell'] for r in runs),
                protection_trips=sum(r['protection_trip'] for r in runs),
                metric_extrema=extrema,
                worst=max(runs, key=lambda r: (not r['accepted'], r.get('tail_alpha_rms_deg', 0))),
                worst_selection='failure_then_tail_alpha_rms_deg')


def compare_integration(coarse, refined):
    """Paired-seed grid sensitivity, not a proof of a qualified numeric bound."""
    coarse, refined = list(coarse), list(refined)
    finer = {r['seed']: r for r in refined}
    seeds = {r['seed'] for r in coarse}
    if not coarse or len(seeds) != len(coarse) or len(finer) != len(refined) or seeds != set(finer):
        raise ValueError('integration comparison requires the same unique seeds')
    pairs, maxima = [], {}
    for run in coarse:
        other = finer[run['seed']]
        shared = [m for m in RUN_METRICS if m in run and m in other]
        missing = [m for m in RUN_METRICS if (m in run) != (m in other)]
        deltas = {m: abs(other[m] - run[m]) for m in shared}
        pairs.append(dict(seed=run['seed'], coarse_accepted=run['accepted'],
                          refined_accepted=other['accepted'], missing_metrics=missing,
                          absolute_metric_delta=deltas))
        for metric, delta in deltas.items():
            if metric not in maxima or delta > maxima[metric]['value']:
                maxima[metric] = _metric_point(delta, run)
    return dict(qualified_bound=False, pairs=pairs, max_abs_metric_delta=maxima,
                acceptance_changed_seeds=[p['seed'] for p in pairs
                                          if p['coarse_accepted'] != p['refined_accepted']])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--q', type=float, nargs=4, default=(30.0, 200.0, 20.0, 20.0),
                        help='LQR weights for theta, alpha, theta_dot, alpha_dot')
    parser.add_argument('--r', type=float, default=0.05)
    parser.add_argument('--qw', type=float, nargs=2, default=(100.0, 60.0),
                        help='observer process noise, rad/s^2, arm and pendulum')
    parser.add_argument('--comp', type=float, default=7.0, help='friction compensation, percent')
    parser.add_argument('--blend', type=float, default=1.0, help='compensation blend width, percent')
    parser.add_argument('--u-max', type=float, default=40.0, help='applied duty limit, percent')
    parser.add_argument('--pwm-step', type=float, default=0.0,
                        help='PWM duty quantum in percent; 0 disables quantization')
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--mode', choices=('firmware', 'plant'), default='firmware',
                        help='firmware applies measured snapshot guards; plant uses legacy envelope only')
    parser.add_argument('--substeps', type=int, default=8)
    parser.add_argument('--refine-substeps', type=int,
                        help='optional larger substep count for paired-seed grid sensitivity')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    for name, values in (('q', args.q), ('r', [args.r]), ('qw', args.qw), ('blend', [args.blend])):
        if any(not math.isfinite(v) or v <= 0 for v in values):
            parser.error(f'{name} must contain finite positive values')
    for name, value in (('comp', args.comp), ('pwm-step', args.pwm_step)):
        if not math.isfinite(value) or value < 0:
            parser.error(f'{name} must be finite and nonnegative')
    if not math.isfinite(args.u_max) or not 0 < args.u_max <= 100:
        parser.error('u-max must be finite and in (0, 100]')
    if not math.isfinite(args.seconds) or args.seconds < T:
        parser.error(f'seconds must be finite and >= {T}')
    if args.seeds < 1 or args.substeps < 1:
        parser.error('seeds and substeps must be positive integers')
    if args.refine_substeps is not None and args.refine_substeps <= args.substeps:
        parser.error('refine-substeps must exceed substeps')
    if args.output is not None and args.output.exists():
        parser.error(f'{args.output} exists')
    d = design(args.q, args.r, qw=args.qw)
    results, required_runs, refined_required = {}, [], []
    integration = dict(coarse_substeps=args.substeps, refined_substeps=args.refine_substeps,
                       qualified_bound=False,
                       status='single_grid_unqualified' if args.refine_substeps is None
                       else 'paired_grid_sensitivity_unqualified',
                       acceptance_changed_runs=0, max_abs_metric_delta={})
    for name, case in SWEEP + EXTRA:
        cfg = dict(comp=args.comp, blend=args.blend, u_max=args.u_max, pwm_step=args.pwm_step)
        cfg.update(case)
        runs = [simulate(d['K'], d['L'], d['Ad'], d['Bd'], cfg, args.seconds, seed=seed,
                         mode=args.mode, substeps=args.substeps) for seed in range(1, args.seeds + 1)]
        summary = dict(aggregate_runs(runs), informative_only=name in INFORMATIVE)
        if name not in INFORMATIVE:
            required_runs.extend(dict(run, case=name) for run in runs)
        if args.refine_substeps is not None:
            refined = [simulate(d['K'], d['L'], d['Ad'], d['Bd'], cfg, args.seconds, seed=seed,
                                mode=args.mode, substeps=args.refine_substeps)
                       for seed in range(1, args.seeds + 1)]
            comparison = compare_integration(runs, refined)
            summary.update(refined=aggregate_runs(refined), integration_comparison=comparison)
            if name not in INFORMATIVE:
                refined_required.extend(dict(run, case=name) for run in refined)
                integration['acceptance_changed_runs'] += len(comparison['acceptance_changed_seeds'])
                for metric, point in comparison['max_abs_metric_delta'].items():
                    previous = integration['max_abs_metric_delta'].get(metric)
                    if previous is None or point['value'] > previous['value']:
                        integration['max_abs_metric_delta'][metric] = dict(point, case=name)
        results[name] = summary
    required = aggregate_runs(required_runs)
    refined_failures = sum(not r['accepted'] for r in refined_required)
    passed = required['failures'] == 0 and refined_failures == 0
    rms = required['metric_extrema'].get('tail_alpha_rms_deg')
    arm = required['metric_extrema']['max_arm_deg']
    out = dict(design_model=dict(DESIGN, **PENDULUM), motor_sim=MOTOR, q=args.q, r=args.r,
               qw=args.qw, comp=args.comp, seconds=args.seconds, seeds=args.seeds,
               blend=args.blend, u_max=args.u_max, pwm_step=args.pwm_step,
               validation_mode=args.mode, validation_passed=passed,
               guards=dict(enabled=args.mode == 'firmware', alpha_limit_deg=FIRMWARE_ALPHA_LIMIT_DEG,
                           arm_limit_counts=FIRMWARE_ARM_LIMIT_COUNTS, arm_reference_count=0),
               required_cases=sum(not s['informative_only'] for s in results.values()),
               required_runs=len(required_runs), required_falls=required['falls'],
               required_protection_trips=required['protection_trips'], required_failures=required['failures'],
               required_refined_failures=refined_failures,
               required_metric_extrema=required['metric_extrema'], integration=integration,
               worst_tail_alpha_rms_deg=rms['max']['value'] if rms else None,
               worst_max_arm_deg=arm['max']['value'],
               K_percent_per_si=[round(x, 4) for x in d['K']],
               K_percent_per_count_code=[round(d['K'][0] * COUNT_RAD, 5), round(d['K'][1] / ADC_PER_RAD, 5),
                                         round(d['K'][2] * COUNT_RAD, 6), round(d['K'][3] / ADC_PER_RAD, 6)],
               L=[[round(x, 6) for x in r] for r in d['L']], closed_loop_rho=round(d['rho'], 6),
               closed_loop_matrix_power_radius_estimate=round(d['matrix_power_radius_estimate'], 6),
               closed_loop_rho_kind=d['rho_kind'],
               sweep=results)
    text = json.dumps(out, indent=2, allow_nan=False)
    if args.output is not None:
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
