#!/usr/bin/env python3
"""Bit-exact v4 arithmetic: Q16.16 native states and Q12.20 coefficients.

Coefficient order is Ad[16], Bd[4], L[8], K[4]. Every state dot product
rounds once toward zero. Runtime numeric faults discard all candidate state.
Positive signed PWM compares mean BIN2 / increasing encoder counts.
"""
import math

STATE_Q = 16
COEFF_Q = 20
S = 1 << STATE_Q
Q = 1 << COEFF_Q
I32_MIN, I32_MAX = -(1 << 31), (1 << 31)-1
COEFF_LIMIT = 128*Q


class NumericRange(ValueError):
    pass


def trunc0(value, shift):
    if type(value) is not int or type(shift) is not int or shift < 0:
        raise ValueError('integer value and nonnegative shift required')
    return -(abs(value) >> shift) if value < 0 else value >> shift


def i32(value):
    if type(value) is not int or not I32_MIN <= value <= I32_MAX:
        raise NumericRange('state/measurement is outside signed int32')
    return value


def quantize_q20(value):
    if isinstance(value,bool) or not math.isfinite(value):
        raise ValueError('coefficient must be finite')
    scaled=value*Q
    result=int(math.copysign(math.floor(abs(scaled)+0.5),scaled))
    if abs(result)>COEFF_LIMIT:
        raise ValueError('coefficient exceeds compiled limit')
    return result


def dot_q20(weights, values, bias_q16=0):
    if len(weights)!=len(values):
        raise ValueError('dot product dimensions differ')
    accumulator=i32(bias_q16)<<COEFF_Q
    for weight,value in zip(weights,values):
        accumulator+=i32(weight)*i32(value)
        if not -(1<<71)<=accumulator<(1<<71):
            raise NumericRange('72-bit accumulator overflow')
    return trunc0(accumulator,COEFF_Q)


def validate_compensation(max_compare,comp_compare,blend_compare,comp_gain_q20):
    if not all(type(x) is int for x in (max_compare,comp_compare,blend_compare,comp_gain_q20)):
        raise ValueError('compensation parameters must be exact integers')
    if not (1<=max_compare<=1125 and 0<=comp_compare<=min(250,max_compare)
            and 25<=blend_compare<=250 and 0<=comp_gain_q20<=10*Q):
        raise ValueError('compensation parameter outside compiled limit')
    if abs(comp_gain_q20*blend_compare-comp_compare*Q)>(blend_compare+1)//2:
        raise ValueError('compensation slope does not match comparison thresholds')


def model_input_q16(command,comp_compare,blend_compare,comp_gain_q20):
    """Inverse of the compensation map using the actually applied integer duty."""
    if abs(command)>=comp_compare+blend_compare:
        return (1 if command>=0 else -1)*(abs(command)-comp_compare)*S
    magnitude=(abs(command)<<(STATE_Q+COEFF_Q))//(Q+comp_gain_q20)
    return -magnitude if command<0 else magnitude


# v6 alpha-rate filter presets, fs = 500 Hz. Modes 2..5 run two cascaded
# one-pole sections; 6..7 run one; 0/1 bypass. The firmware uses this table.
FILTER_ALPHA={0:0,1:0,2:2425,3:3991,4:6268,5:7739,6:3991,7:6268}
FILTER_TWO={0:False,1:False,2:True,3:True,4:True,5:True,6:False,7:False}

# Arm-side disturbance observer constants in native units. The arm model is
# theta'' = -ARM_A*theta' + ARM_B*u + d with u in percent and theta'' in
# rad/s^2; the observer state x2 is Q16.16 counts/s and the model input is
# Q16.16 compare (2500 compare = 100 percent).
COUNT_RAD=2*math.pi/1040.0
ARM_A=12.5
ARM_B=(29.3*COUNT_RAD/0.0663+38.21*COUNT_RAD/0.0945)/2
MODEL_B_Q16=round(ARM_B*65536/(COUNT_RAD*1638400)*65536)
DOB_K_Q16=round((COUNT_RAD*25.0/ARM_B)*65536)
DOB_ALPHA={0:0,5:3991,10:7739}          # Hz -> one-pole Q16 coefficient
DOB_GAIN={1.0:65536,1.5:98304}

# Pendulum-side disturbance observer, same native units as the arm DOB.
# alpha'' = PEND_W2*alpha + PEND_BETA*ARM_A*theta' - PEND_C*alpha' - PEND_BETA*ARM_B*u + d2
# alpha and alpha' are Q16.16 ADC codes and codes/s; theta' is Q16.16 counts/s;
# u is Q16.16 compare. d2 is therefore Q16.16 codes/s^2.
PEND_W2=9.724**2
PEND_BETA=1.444
PEND_C=0.085
PEND_ADC_PER_RAD=162.33
PEND_W2_Q16=round(PEND_W2*65536)
PEND_CROSS_Q16=round(PEND_BETA*ARM_A*COUNT_RAD*PEND_ADC_PER_RAD*65536)
PEND_C_Q16=round(PEND_C*65536)
PEND_BU_Q16=round(PEND_BETA*ARM_B*PEND_ADC_PER_RAD/25.0*65536)
PEND_DOB_K_Q16=round(25.0/(PEND_BETA*ARM_B*PEND_ADC_PER_RAD)*65536)


def _clamp32(value):
    return max(I32_MIN,min(I32_MAX,value))


def filter_x3(value,mode,y1,y2):
    """One sample of the alpha-rate filter; returns (used,y1,y2).

    Differences and state adds saturate at int32, and each product rounds to
    nearest with ties toward +infinity, matching the RTL exactly.
    """
    if type(mode) is not int or mode not in FILTER_ALPHA:
        raise ValueError('filter mode must be in 0..7')
    alpha=FILTER_ALPHA[mode]
    if alpha==0:
        return value,y1,y2
    d1=_clamp32(value-y1)
    y1=max(I32_MIN,min(I32_MAX,y1+((d1*alpha+(1<<15))>>16)))
    if FILTER_TWO[mode]:
        d2=_clamp32(y1-y2)
        y2=max(I32_MIN,min(I32_MAX,y2+((d2*alpha+(1<<15))>>16)))
        return y2,y1,y2
    return y1,y1,y2


def step_q16(state,measurement,previous_model_input,coefficients,*,max_compare=1000,
             comp_compare=175,blend_compare=25,comp_gain_q20=7*Q,feedforward_q16=0,
             filter_mode=0,filter_state=(0,0),pos_deadband=0,dob_alpha=0,dob_gain_q16=0,
             dob_state=(0,0),dob2_alpha=0,dob2_gain_q16=0,dob2_state=(0,0),
             alpha_rate_ext_q16=0,alpha_rate_mix=False,arm_rate_ext_q16=0,arm_rate_mix=False):
    """One controller sample. feedforward_q16 (v6) is added to the feedback before
    the clip and friction compensation; model_input_q16 = u_model_q16 - feedforward
    is the observer input of the next sample (equal to u_model_q16 without it).
    filter_mode and filter_state select and carry the v6 alpha-rate filter.
    pos_deadband (whole encoder counts) subtracts a dead zone from the arm-error
    feedback only; the observer and committed state keep the raw error.
    dob_alpha (Q16) and dob_gain_q16 enable the arm-side disturbance observer;
    dob_state carries its (disturbance acceleration, previous arm velocity).
    dob2_alpha/dob2_gain_q16 and dob2_state add the pendulum-side disturbance
    observer; its state is (disturbance acceleration, previous alpha rate)."""
    if len(coefficients)!=32 or any(type(c) is not int or abs(c)>COEFF_LIMIT for c in coefficients):
        raise ValueError('require 32 bounded int32 Q20 coefficients')
    if len(state)!=4 or len(measurement)!=2:
        raise ValueError('require four states and two measurements')
    if type(alpha_rate_mix) is not bool or type(arm_rate_mix) is not bool:
        raise ValueError('mix flags must be bool')
    for ext in (alpha_rate_ext_q16,arm_rate_ext_q16):
        if type(ext) is not int or not I32_MIN<=ext<=I32_MAX:
            raise ValueError('rate ext must be int32')
    if (type(filter_mode) is not int or filter_mode not in FILTER_ALPHA or
            len(filter_state)!=2 or any(type(v) is not int or not I32_MIN<=v<=I32_MAX
                                        for v in filter_state)):
        raise ValueError('require a filter mode in 0..7 and two int32 states')
    if type(pos_deadband) is not int or not 0<=pos_deadband<=64:
        raise ValueError('pos_deadband must be an integer in 0..64 counts')
    if type(dob_alpha) is not int or not 0<=dob_alpha<=0xFFFF:
        raise ValueError('dob_alpha must be a Q16 integer in 0..65535')
    if type(dob_gain_q16) is not int or not 0<=dob_gain_q16<=2*S:
        raise ValueError('dob_gain_q16 must be a Q16 integer in 0..2*65536')
    if (len(dob_state)!=2 or any(type(v) is not int or not I32_MIN<=v<=I32_MAX for v in dob_state)):
        raise ValueError('require two int32 disturbance-observer states')
    if type(dob2_alpha) is not int or not 0<=dob2_alpha<=0xFFFF:
        raise ValueError('dob2_alpha must be a Q16 integer in 0..65535')
    if type(dob2_gain_q16) is not int or not 0<=dob2_gain_q16<=2*S:
        raise ValueError('dob2_gain_q16 must be a Q16 integer in 0..2*65536')
    if (len(dob2_state)!=2 or any(type(v) is not int or not I32_MIN<=v<=I32_MAX for v in dob2_state)):
        raise ValueError('require two int32 pendulum-observer states')
    coefficients=list(coefficients)
    validate_compensation(max_compare,comp_compare,blend_compare,comp_gain_q20)
    if type(feedforward_q16) is not int or abs(feedforward_q16)>1125*S:
        raise ValueError('feedforward outside the compiled +-1000<<16 limit')
    old=list(state)
    y1,y2=filter_state
    dob_d,dob_v=dob_state
    dob2_d,dob2_v=dob2_state
    try:
        x=[i32(v) for v in state]
        y=[i32(v) for v in measurement]
        previous=i32(previous_model_input)
        pred=[i32(dot_q20(coefficients[4*i:4*i+4]+[coefficients[16+i]],
                          x+[previous])) for i in range(4)]
        error=[i32(y[j]-pred[j]) for j in range(2)]
        candidate=[i32(dot_q20(coefficients[20+2*i:22+2*i],error,pred[i])) for i in range(4)]
        used,y1,y2=filter_x3(candidate[3],filter_mode,y1,y2)
        x0=candidate[0]
        if pos_deadband:
            band=pos_deadband*S
            x0=x0-band if x0>band else x0+band if x0<-band else 0
        # alg19: K2 sees (Kalman arm rate + 2 Hz LP)/2; K3 sees (Kalman alpha
        # rate + 20 Hz SVF)/2. Both mixes are arithmetic right shifts of a
        # 33-bit sum, matching the RTL.
        arm_used=candidate[2]
        rate_used=used
        if arm_rate_mix:
            arm_used=(candidate[2]+arm_rate_ext_q16)>>1
        if alpha_rate_mix:
            rate_used=(used+alpha_rate_ext_q16)>>1
        # Feedback may legitimately exceed int32; clip the wide result first.
        raw=-dot_q20(coefficients[28:32],[x0]+[candidate[1]]+[arm_used,rate_used],
                     -i32(feedforward_q16))
        if dob_alpha:
            acc=(candidate[2]-dob_v)*500
            model_acc=-((25*candidate[2])>>1)+((previous*MODEL_B_Q16)>>16)
            d_raw=_clamp32(acc-model_acc)
            diff=_clamp32(d_raw-dob_d)
            dob_d=_clamp32(dob_d+((diff*dob_alpha)>>16))
            dob_v=candidate[2]
            mid=_clamp32((dob_d*DOB_K_Q16)>>16)
            raw-=mid*dob_gain_q16>>16
        if dob2_alpha:
            acc_a=(candidate[3]-dob2_v)*500
            model_acc_a=(((PEND_W2_Q16*candidate[1])>>16)
                         +((PEND_CROSS_Q16*candidate[2])>>16)
                         -((PEND_C_Q16*candidate[3])>>16)
                         -((PEND_BU_Q16*previous)>>16))
            d2_raw=_clamp32(acc_a-model_acc_a)
            diff2=_clamp32(d2_raw-dob2_d)
            dob2_d=_clamp32(dob2_d+((diff2*dob2_alpha)>>16))
            dob2_v=candidate[3]
            mid2=_clamp32((dob2_d*PEND_DOB_K_Q16)>>16)
            raw+=mid2*dob2_gain_q16>>16
        bound=max_compare*S
        linear=max(-bound,min(bound,raw))
        if abs(linear)>=blend_compare*S:
            addition=comp_compare*S*(1 if linear>=0 else -1)
        else:
            addition=trunc0(linear*comp_gain_q20,COEFF_Q)
        compensated=max(-bound,min(bound,linear+addition))
        command=trunc0(compensated,STATE_Q)
        model=i32(model_input_q16(command,comp_compare,blend_compare,comp_gain_q20))
        return dict(fault=False,state=candidate,u_linear_q16=linear,
                    u_compensated_q16=compensated,u_model_q16=model,command=command,
                    model_input_q16=i32(model-feedforward_q16),filter_state=(y1,y2),
                    dob_state=(dob_d,dob_v),dob2_state=(dob2_d,dob2_v))
    except NumericRange as error:
        return dict(fault=True,state=old,u_linear_q16=0,u_compensated_q16=0,
                    u_model_q16=0,command=0,model_input_q16=0,error=str(error),
                    filter_state=(y1,y2),dob_state=(dob_d,dob_v),
                    dob2_state=(dob2_d,dob2_v))


def percent_to_compare(percent,name):
    """Exact 25-count-per-percent conversion; non-grid percentages are rejected."""
    compare=round(percent*25)
    if not math.isfinite(percent) or abs(percent*25-compare)>1e-9:
        raise ValueError(f'{name} must be a multiple of 0.04 percent')
    return compare


def compensation_gain_q20(comp_compare,blend_compare):
    """Nearest Q20 slope comp/blend, halves away from zero (both are nonnegative)."""
    return (2*comp_compare*Q+blend_compare)//(2*blend_compare)


class FixedController:
    """Firmware integer controller for j280_balance_design.simulate(controller=...).

    SI measurements are converted to native firmware units (whole encoder
    counts, ADC code relative to the upright zero, Q16.16), stepped through
    step_q16, and converted back. State and the virtual model input persist
    between calls, so create one instance per simulated run.
    """

    def __init__(self,coefficients,counts_per_rev=1040.,adc_per_rad=162.33,pos_deadband=0,
                 dob_alpha=0,dob_gain_q16=0,dob2_alpha=0,dob2_gain_q16=0,
                 filter_mode=0,alpha_svf=False,arm_lp=False):
        if len(coefficients)!=32 or any(type(c) is not int or abs(c)>COEFF_LIMIT for c in coefficients):
            raise ValueError('require 32 bounded int32 Q20 coefficients')
        if type(pos_deadband) is not int or not 0<=pos_deadband<=64:
            raise ValueError('pos_deadband must be an integer in 0..64 counts')
        self.coefficients=list(coefficients)
        self.count_rad=2*math.pi/counts_per_rev
        self.adc_per_rad=adc_per_rad
        self.pos_deadband=pos_deadband
        self.dob_alpha=dob_alpha
        self.dob_gain_q16=dob_gain_q16
        self.dob_state=(0,0)
        self.dob2_alpha=dob2_alpha
        self.dob2_gain_q16=dob2_gain_q16
        self.dob2_state=(0,0)
        self.filter_mode=filter_mode
        self.filter_state=(0,0)
        self.alpha_svf=alpha_svf
        self.arm_lp=arm_lp
        self.svf_x1=0; self.svf_x2=0; self.arm_lp_state=0
        self.state=[0]*4
        self.previous_model_q16=0
        self.steps=0

    def __call__(self,y_si,case):
        max_compare=percent_to_compare(case['u_max'],'u_max')
        comp_compare=percent_to_compare(case['comp'],'comp')
        blend_compare=percent_to_compare(case['blend'],'blend')
        limits=dict(max_compare=max_compare,comp_compare=comp_compare,blend_compare=blend_compare,
                    comp_gain_q20=compensation_gain_q20(comp_compare,blend_compare))
        theta_q16=round(y_si[0]/self.count_rad)*S
        alpha_q16=round(y_si[1]*self.adc_per_rad*S)
        out=step_q16(self.state,(theta_q16,alpha_q16),self.previous_model_q16,
                     self.coefficients,**limits,pos_deadband=self.pos_deadband,
                     filter_mode=self.filter_mode,filter_state=self.filter_state,
                     alpha_rate_ext_q16=self.svf_x2,alpha_rate_mix=self.alpha_svf,
                     arm_rate_ext_q16=self.arm_lp_state,arm_rate_mix=self.arm_lp,
                     dob_alpha=self.dob_alpha,dob_gain_q16=self.dob_gain_q16,
                     dob_state=self.dob_state,dob2_alpha=self.dob2_alpha,
                     dob2_gain_q16=self.dob2_gain_q16,dob2_state=self.dob2_state)
        self.dob_state=out.get('dob_state',self.dob_state)
        self.dob2_state=out.get('dob2_state',self.dob2_state)
        self.filter_state=out.get('filter_state',self.filter_state)
        if self.alpha_svf:
            d=_clamp32(alpha_q16-self.svf_x1)
            self.svf_x1=_clamp32(self.svf_x1+((1809*d+107*self.svf_x2)>>16))
            self.svf_x2=_clamp32((42255*self.svf_x2+1686407*d)>>16)
        if self.arm_lp:
            v=out['state'][2]
            self.arm_lp_state=_clamp32(self.arm_lp_state+((1627*(v-self.arm_lp_state))>>16))
        if out['fault']:
            return dict(fault=out['error'])
        self.state=out['state']
        self.previous_model_q16=out['u_model_q16']
        self.steps+=1
        x=self.state
        return dict(fault=False,
                    xhat_si=[x[0]/S*self.count_rad,x[1]/S/self.adc_per_rad,
                             x[2]/S*self.count_rad,x[3]/S/self.adc_per_rad],
                    u_linear_percent=out['u_linear_q16']/S/25,
                    u_applied_percent=out['command']/25,
                    u_model_percent=out['u_model_q16']/S/25)


def design_coefficients(q=(30.,200.,20.,20.),r=0.05,qw=(100.,60.),
                        counts_per_rev=1040.,adc_per_rad=162.33):
    """Compile provisional floating design into native count/code/PWM units.

    This produces arithmetic inputs; it does not qualify the physical model
    for powered use. Both observer measurement bases must be transformed.
    """
    import j280_balance_design as design
    if len(q)!=4 or len(qw)!=2:
        raise ValueError('require four state weights and two process-noise scales')
    values=[*q,*qw,r,counts_per_rev,adc_per_rad]
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in values):
        raise ValueError('weights, measurement and noise scales must be positive and finite')
    d=design.design(q,r,qw=qw)
    scales=[2*math.pi/counts_per_rev,1/adc_per_rad]*2
    ad=[d['Ad'][i][j]*scales[j]/scales[i] for i in range(4) for j in range(4)]
    bd=[d['Bd'][i][0]/(25*scales[i]) for i in range(4)]
    observer=[d['L'][i][j]*scales[j]/scales[i] for i in range(4) for j in range(2)]
    gains=[25*d['K'][i]*scales[i] for i in range(4)]
    return [quantize_q20(c) for c in ad+bd+observer+gains]
