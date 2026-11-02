#!/usr/bin/env python3
"""J280 swing-up feasibility simulation: energy pumping with a bounded arm,
a blind stretch through the potentiometer dead zone, and hand-off to the
identified balance controller (design D + friction compensation).

Coordinates: alpha from upright (rad), ADC-increasing positive, hanging at +-pi;
theta arm angle (rad, count-increasing positive); u signed duty percent.
The plant equations match j280_balance_design.simulate.
"""
import argparse
import json
import math
import random

import j280_balance_design as bd

T = bd.T
COUNT_RAD = bd.COUNT_RAD
W0 = bd.PENDULUM['w0']


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_map(path=None):
    """Piecewise-linear (code, deg-from-hanging) knots; default: the 2026-10-18 provisional map."""
    from pathlib import Path
    path = path or Path(__file__).resolve().parents[1] / 'docs/data/angle-calibration-2026-10-18/angle-map-v2.json'
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    return [(k['code'], k['deg']) for k in data['knots']], tuple(data['dead_zone_deg_from_hanging'])


MAP, DEAD_FROM_HANGING = load_map()


def _interp(x, xs, ys):
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / (xs[i + 1] - xs[i])
    return None


def adc_reading(al, rng, g):
    """True sensor: the calibrated map, optionally scaled about hanging and offset (case errors)."""
    phi = (math.degrees(wrap(al)) + 180.0 + g['zero_deg']) % 360.0      # from hanging, [0, 360)
    if phi > 270.0:                                                     # dead zone / decreasing side
        phi -= 360.0
    if DEAD_FROM_HANGING[0] < phi < DEAD_FROM_HANGING[1]:
        return None
    # Map distortion that keeps the measured hanging and upright anchors: up to warp_deg in between.
    phi_true = phi + g['warp_deg'] * math.sin(math.radians(phi))
    code = _interp(phi_true, [d for _, d in MAP], [c for c, _ in MAP])
    if code is None:
        return None
    code = round(code + rng.gauss(0, g['adc_noise']))
    return code if 0 <= code <= 1023 else None


def code_to_alpha(code):
    """Controller's belief: the same calibrated map (alpha from upright, rad)."""
    phi = _interp(code, [c for c, _ in MAP], [d for _, d in MAP])
    return wrap(math.radians(phi - 180.0))


def simulate(params, case=None, seconds=20.0, seed=1, record=False):
    """Run one swing-up from rest; return metrics (and optionally a trace)."""
    rng = random.Random(seed)
    g = dict(gain=1.0, tau=1.0, beta=1.0, w0=1.0, friction=1.0, backlash_counts=4,
             adc_noise=0.8, warp_deg=0.0, zero_deg=0.0, alpha0_deg=179.0)
    g.update(case or {})
    p = dict(k_energy=0.4, k_theta=60.0, k_theta_dot=8.0, u_max=40.0, catch_deg=20.0,
             catch_rate=6.0, kick_ms=120.0, comp=7.0, blend=2.48,
             energy_margin=0.0, pump_cos=0.0, blind_coast=True,
             strategy='energy', amp_deg=40.0, v_sat=2.0, lead_s=0.05, kp=60.0, kd=4.0, e_scale=60.0)
    p.update(params)
    motor = {d: dict(m, k=m['k'] * g['gain'], tau=m['tau'] * g['tau'], d0=m['d0'] * g['friction'],
                     d_break=m['d_break'] * g['friction']) for d, m in bd.MOTOR.items()}
    beta, w0 = bd.PENDULUM['beta'] * g['beta'], W0 * g['w0']
    c, mu = bd.PENDULUM['c'], bd.PENDULUM['mu']
    half_lash = g['backlash_counts'] * COUNT_RAD / 2
    design = bd.design((300.0, 200.0, 20.0, 5.0), 0.5, qw=(10.0, 6.0))
    K, L, Ad, Bd = design['K'], design['L'], design['Ad'], design['Bd']

    th_m = th_a = th_dm = 0.0
    al = math.radians(g['alpha0_deg'])
    w = 0.0
    sub = 8
    h = T / sub
    mode = 'swing'
    hist = []                 # (alpha_meas or None, theta_meas)
    est_a, est_ad = al, 0.0
    blind = 0
    xh = None
    u_cmd = u_model = 0.0
    caught_at = None
    max_arm = 0.0
    max_blind = 0
    trace = []
    steps = int(seconds / T)

    def finish(termination, at_s):
        out = dict(termination=termination, success=termination == 'balanced', at_s=round(at_s, 3),
                   catch_s=None if caught_at is None else round(caught_at, 3),
                   max_arm_deg=round(math.degrees(max_arm), 1), max_blind_ms=max_blind * 2)
        if record:
            out['trace'] = trace
        return out

    for k in range(steps):
        enc = round(th_m / COUNT_RAD)
        theta = enc * COUNT_RAD
        code = adc_reading(al, rng, g)
        meas = None if code is None else code_to_alpha(code)
        hist.append((meas, theta))
        old = hist[max(0, len(hist) - 6)]
        theta_dot = (theta - old[1]) / (min(5, len(hist) - 1) * T or T)
        # Pendulum estimate: difference over the newest run of valid samples (up to
        # 10 ms); during the dead zone, gravity-only prediction (the pump coasts).
        run = 0
        while run < min(5, len(hist) - 1) and hist[-2 - run][0] is not None and meas is not None:
            run += 1
        if meas is not None and run >= 2:
            est_ad = wrap(meas - hist[-1 - run][0]) / (run * T)
            est_a = meas
            blind = 0
        else:
            est_ad += T * w0 * w0 * math.sin(est_a)
            est_a = wrap(est_a + T * est_ad)
            blind += meas is None
            if meas is not None:
                est_a = meas
        max_blind = max(max_blind, blind)
        if mode == 'swing':
            energy = 0.5 * est_ad ** 2 + w0 * w0 * (math.cos(est_a) - 1.0)   # 0 at upright rest
            if k * T * 1000 < p['kick_ms']:
                u = p['u_max']
            else:
                cs = math.cos(est_a)
                s = est_ad * cs
                pump = -p['k_energy'] * (p['energy_margin'] - energy) * (1 if s > 0 else -1 if s < 0 else 0)
                if -cs < p['pump_cos']:          # pump only near the bottom (cos ~ -1 when hanging)
                    pump = 0.0
                if blind and p['blind_coast']:
                    pump = 0.0
                u = pump - p['k_theta'] * theta - p['k_theta_dot'] * theta_dot
                if p['strategy'] == 'ref':
                    # Bounded arm reference in anti-phase with the (lead-predicted) pendulum
                    # velocity, scaled down as the energy approaches the target.
                    a_pred = est_a + p['lead_s'] * est_ad
                    ad_pred = est_ad + p['lead_s'] * w0 * w0 * math.sin(est_a)
                    drive = -ad_pred * math.cos(a_pred) / p['v_sat']
                    scale = max(0.0, min(1.0, (p['energy_margin'] - energy) / p['e_scale']))
                    ref = -math.radians(p['amp_deg']) * scale * max(-1.0, min(1.0, drive))
                    u = p['kp'] * (ref - theta) - p['kd'] * theta_dot
            u_cmd = max(-p['u_max'], min(p['u_max'], u))
            if (meas is not None and abs(est_a) < math.radians(p['catch_deg'])
                    and abs(est_ad) < p['catch_rate']):
                mode = 'balance'
                caught_at = k * T
                xh = [theta, est_a, theta_dot, est_ad]
                u_model = 0.0
        else:
            if meas is None:
                return finish('lost_sensor_in_balance', k * T)
            # The balance firmware converts linearly about the upright zero (162.33 code/rad).
            y = [theta, (code - 737.0) / bd.ADC_PER_RAD]
            pred = [sum(Ad[i][j] * xh[j] for j in range(4)) + Bd[i][0] * u_model for i in range(4)]
            e = [y[0] - pred[0], y[1] - pred[1]]
            xh = [pred[i] + L[i][0] * e[0] + L[i][1] * e[1] for i in range(4)]
            u_lin = -sum(Ki * xi for Ki, xi in zip(K, xh))
            u_cmd = bd.applied_duty(u_lin, p['comp'], p['blend'], 40.0)
            u_model = bd.inverse_compensation(u_cmd, p['comp'], p['blend'])
            if abs(wrap(al)) > math.radians(25):
                return finish('fell_after_catch', k * T)
        for _ in range(sub):
            m = motor['pos' if u_cmd > 0 else 'neg']
            drive = (m['k'] * u_cmd - th_dm) / m['tau']
            if abs(th_dm) > 1e-4:
                f = motor['pos' if th_dm > 0 else 'neg']
                new = th_dm + h * (drive - f['k'] / f['tau'] * f['d0'] * math.copysign(1, th_dm))
                th_dm = 0.0 if new * th_dm < 0 else new
            else:
                f = motor['pos' if drive > 0 else 'neg']
                if abs(drive) > f['k'] / f['tau'] * f['d_break']:
                    th_dm += h * (drive - f['k'] / f['tau'] * f['d0'] * math.copysign(1, drive))
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
        max_arm = max(max_arm, abs(th_a))
        if record and k % 5 == 0:
            trace.append((round(k * T, 3), mode, round(math.degrees(wrap(al)), 1), round(math.degrees(th_a), 1), round(u_cmd, 1)))
        if max_arm > math.radians(70):
            return finish('arm_limit', k * T)
        if mode == 'balance' and k * T - caught_at > 5.0:
            return finish('balanced', k * T)

    return finish('timeout', seconds)


class Plant:
    """The identified arm/pendulum model of simulate(), advanced one 2 ms sample at a time.

    rho = J_pendulum / J_arm adds the pendulum's reaction on the arm (Furuta arm
    equation; coupling rho*beta). The motor model was identified with the pendulum
    hanging, where this load is small; 0.2 fits swing-621 (2026-10-18) best.
    """

    def __init__(self, g):
        self.motor = {d: dict(m, k=m['k'] * g['gain'], tau=m['tau'] * g['tau'], d0=m['d0'] * g['friction'],
                              d_break=m['d_break'] * g['friction']) for d, m in bd.MOTOR.items()}
        self.beta, self.w0 = bd.PENDULUM['beta'] * g['beta'], W0 * g['w0']
        self.half_lash = g['backlash_counts'] * COUNT_RAD / 2
        self.rho = g.get('rho', 0.0)
        self.th_m = self.th_a = self.th_dm = 0.0
        self.al = math.radians(g['alpha0_deg'])
        self.w = 0.0
        self.v_a = 0.0

    def advance(self, u_cmd, sub=8):
        c, mu, h = bd.PENDULUM['c'], bd.PENDULUM['mu'], T / sub
        motor = self.motor
        for _ in range(sub):
            m = motor['pos' if u_cmd > 0 else 'neg']
            drive = (m['k'] * u_cmd - self.th_dm) / m['tau']
            den = 1.0
            if self.rho:
                al, rb = self.al, self.rho * self.beta
                s, co = math.sin(al), math.cos(al)
                ad = self.w - self.beta * co * self.v_a
                vd = self.th_dm
                drive += (-2 * self.rho * s * co * ad * vd + rb * s * ad * ad
                          - rb * co * (self.w0 ** 2 * s + s * co * vd * vd - c * ad - mu * math.tanh(ad / 0.02)))
                den = 1 + self.rho * s * s - rb * self.beta * co * co
            if abs(self.th_dm) > 1e-4:
                f = motor['pos' if self.th_dm > 0 else 'neg']
                new = self.th_dm + h * (drive - f['k'] / f['tau'] * f['d0'] * math.copysign(1, self.th_dm)) / den
                self.th_dm = 0.0 if new * self.th_dm < 0 else new
            else:
                f = motor['pos' if drive > 0 else 'neg']
                if abs(drive) > f['k'] / f['tau'] * f['d_break']:
                    self.th_dm += h * (drive - f['k'] / f['tau'] * f['d0'] * math.copysign(1, drive)) / den
                else:
                    self.th_dm = 0.0
            self.th_m += h * self.th_dm
            new_a = min(max(self.th_a, self.th_m - self.half_lash), self.th_m + self.half_lash)
            d_a = new_a - self.th_a
            self.th_a = new_a
            v_a = d_a / h
            self.v_a = v_a
            al, w, beta = self.al, self.w, self.beta
            al_dot = w - beta * math.cos(al) * v_a
            wd = (self.w0 ** 2 * math.sin(al) - c * al_dot - mu * math.tanh(al_dot / 0.02)
                  + math.sin(al) * math.cos(al) * v_a * v_a - beta * math.sin(al) * al_dot * v_a)
            self.w += h * wd
            self.al += h * self.w - beta * math.cos(al) * d_a


STALL_COMPARE, STALL_SAMPLES = 500, 75
RHO = 0.2          # pendulum/arm inertia ratio fitted to swing-621


def balance_profile(path=None):
    from pathlib import Path
    path = path or Path(__file__).resolve().parents[1] / 'docs/data/balance-board-2026-10-17/balance-511-config.json'
    return json.loads(Path(path).read_text(encoding='utf-8'))['values']


def simulate_fixed(swing, case=None, seconds=8.0, seed=1, record=False, profile=None, exit_code=80,
                   inputs=None):
    """Firmware-exact v5 run: j280_swing_math in SWING, the v4 integer observer after the catch.

    swing: SWING command values (j280_swing_math.compile_params). Failure criteria are the
    firmware's: travel from the start position, |code-zero| > exit_code in BALANCE, the
    guard's blind limit / mid-range blind entry, sensor loss in BALANCE, swing timeout.
    case['spurious'] is the probability that the first sample inside / outside the dead
    zone reads a random mid-range code (seen in free-swing-01).
    inputs: optional list; receives the start code, then (code, valid, encoder) per SWING sample.
    """
    import j280_balance_math as bm
    import j280_swing_math as sm
    rng = random.Random(seed)
    g = dict(gain=1.0, tau=1.0, beta=1.0, w0=1.0, friction=1.0, backlash_counts=4,
             adc_noise=0.8, warp_deg=0.0, zero_deg=0.0, alpha0_deg=179.0, spurious=0.0,
             encoder_dead_s=None, rho=RHO)
    g.update(case or {})
    prof = profile or balance_profile()
    zero_q16, max_compare = prof['upright_zero_q16'], prof['max_compare']
    limits = dict(max_compare=max_compare, comp_compare=prof['comp_compare'],
                  blend_compare=prof['blend_compare'], comp_gain_q20=prof['comp_gain_q20'])
    plant = Plant(g)
    code0 = adc_reading(plant.al, rng, g)
    s = sm.initial_state(code0, True)
    if inputs is not None:
        inputs.append(code0)
    mode, caught_at, xh, u_model, command = 'swing', None, None, 0, 0
    accepted_run, est_code, prev_valid = 6, code0, True
    max_arm = max_blind = stall = max_stall = 0
    frozen = None
    trace = []

    def finish(termination, k):
        out = dict(termination=termination, success=termination == 'balanced', at_s=round(k * T, 3),
                   catch_s=None if caught_at is None else round(caught_at, 3),
                   max_arm_deg=round(max_arm * math.degrees(COUNT_RAD), 1), max_blind_ms=max_blind * 2,
                   max_stall=max_stall)
        if record:
            out['trace'] = trace
        return out

    for k in range(int(seconds / T)):
        enc = round(plant.th_m / COUNT_RAD)
        if g['encoder_dead_s'] is not None and k * T >= g['encoder_dead_s']:
            frozen = enc if frozen is None else frozen
            enc = frozen
        code = adc_reading(plant.al, rng, g)
        valid = code is not None
        boundary, prev_valid = valid != prev_valid, valid
        if boundary and rng.random() < g['spurious']:
            code, valid = rng.randint(300, 700), True
        # Raw estimator acceptance (quality 7 after six accepted samples, 64-code step limit).
        ok = valid and (accepted_run == 0 or abs(code - est_code) <= 64)
        accepted_run = (accepted_run + 1 if accepted_run else 1) if ok else 0
        if ok:
            est_code = code
        max_arm = max(max_arm, abs(enc))
        if abs(enc) > swing['travel_count']:
            return finish('travel_limit', k)
        if mode == 'swing':
            if k * T * 1000 >= swing['swing_ms']:
                return finish('swing_timeout', k)
            if inputs is not None:
                inputs.append((code if valid else 0, valid, enc))
            s, out = sm.step(s, code if valid else 0, valid, enc, swing,
                             max_compare=max_compare, zero_q16=zero_q16)
            max_blind = max(max_blind, out['blind'])
            if out['blind_mid']:
                return finish('blind_mid_range', k)
            if out['blind'] > swing['blind_samples']:
                return finish('blind_limit', k)
            command = out['command']
            # Guard arm-stall check (j280_swing_guard STALL_COMPARE / STALL_SAMPLES).
            stall = stall + 1 if abs(command) >= STALL_COMPARE and out['theta_speed'] == 0 else 0
            max_stall = max(max_stall, stall)
            if stall > STALL_SAMPLES:
                return finish('arm_stall', k)
            if (out['catch'] and accepted_run >= 6
                    and abs((code << 16) - zero_q16) <= exit_code << 16):
                mode, caught_at, xh, u_model = 'balance', k * T, out['preset'], 0
        else:
            if not valid or accepted_run < 6:
                return finish('sensor_in_balance', k)
            if abs((code << 16) - zero_q16) > exit_code << 16:
                return finish('angle_limit', k)
            r = bm.step_q16(xh, (enc << 16, (code << 16) - zero_q16), u_model, prof['coefficients'], **limits)
            if r['fault']:
                return finish('numeric', k)
            xh, u_model, command = r['state'], r['u_model_q16'], r['command']
            if k * T - caught_at > 5.0:
                return finish('balanced', k)
        if record and k % 5 == 0:
            trace.append((round(k * T, 3), mode, round(math.degrees(wrap(plant.al)), 1),
                          round(plant.th_a / COUNT_RAD), command))
        plant.advance(command / 25.0)
    return finish('timeout', int(seconds / T))


def sweep_fixed(swing, seeds=(1, 2, 3), seconds=8.0, cases=None, **kw):
    runs = []
    for name, case in cases or CASES_FIXED:
        for seed in seeds:
            r = simulate_fixed(swing, case, seconds, seed, **kw)
            r['case'] = name
            runs.append(r)
    ok = [r for r in runs if r['success']]
    return dict(runs=len(runs), success=len(ok),
                failures=sorted({(r['case'], r['termination']) for r in runs if not r['success']}),
                worst_catch_s=max((r['catch_s'] for r in ok), default=None),
                worst_arm_deg=max((r['max_arm_deg'] for r in runs), default=None),
                worst_blind_ms=max(r['max_blind_ms'] for r in runs))


CASES = [('nominal', {}), ('motor gain -30%', dict(gain=0.7)), ('motor gain +30%', dict(gain=1.3)),
         ('friction +30%', dict(friction=1.3)), ('friction -30%', dict(friction=0.7)),
         ('backlash 12', dict(backlash_counts=12)), ('map warp +4 deg', dict(warp_deg=4.0)),
         ('map warp -4 deg', dict(warp_deg=-4.0)), ('zero +1 deg', dict(zero_deg=1.0)),
         ('zero -1 deg', dict(zero_deg=-1.0)), ('beta -10%', dict(beta=0.9)), ('noise 1.5', dict(adc_noise=1.5)),
         ('start 170 deg', dict(alpha0_deg=170.0)), ('start -175 deg', dict(alpha0_deg=-175.0))]


CASES_FIXED = CASES + [('spurious edge 0.5', dict(spurious=0.5)), ('spurious edge 1.0', dict(spurious=1.0))]


def sweep(params, seeds=(1, 2), seconds=20.0):
    runs = []
    for name, case in CASES:
        for seed in seeds:
            r = simulate(params, case, seconds, seed)
            r['case'] = name
            runs.append(r)
    ok = [r for r in runs if r['success']]
    return dict(runs=len(runs), success=len(ok),
                failures=sorted({(r['case'], r['termination']) for r in runs if not r['success']}),
                worst_catch_s=max((r['catch_s'] for r in ok), default=None),
                worst_arm_deg=max((r['max_arm_deg'] for r in runs), default=None),
                worst_blind_ms=max(r['max_blind_ms'] for r in runs))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--params', type=json.loads, default={})
    parser.add_argument('--trace', action='store_true')
    parser.add_argument('--fixed', action='store_true', help='firmware-exact v5 model; --params are float ref params')
    args = parser.parse_args(argv)
    if args.fixed:
        import j280_swing_math as sm
        prof = balance_profile()
        swing = sm.compile_params(args.params, config_id=prof['config_id'], max_compare=prof['max_compare'],
                                  w0=W0, count_rad=COUNT_RAD)
        if args.trace:
            r = simulate_fixed(swing, record=True)
            for row in r.pop('trace')[::10]:
                print(row)
            print(json.dumps(r))
        else:
            print(json.dumps(sweep_fixed(swing)))
    elif args.trace:
        r = simulate(args.params, record=True)
        for row in r.pop('trace')[::10]:
            print(row)
        print(json.dumps(r))
    else:
        print(json.dumps(sweep(args.params)))


if __name__ == '__main__':
    main()
