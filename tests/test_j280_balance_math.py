"""Independent hand-computable fixed-point observer/feedback contracts."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import j280_balance_math as fx

Q=1<<20
S=1<<16


def coefficients():
    ad=[Q if i==j else 0 for i in range(4) for j in range(4)]
    return ad+[0]*4+[Q,0,0,Q,0,0,0,0]+[-Q,0,0,0]


class BalanceMathTests(unittest.TestCase):
    def test_truncation_is_toward_zero_for_negative_values(self):
        self.assertEqual([fx.trunc0(x,1) for x in (-5,-3,-1,0,1,3,5)],[-2,-1,0,0,0,1,2])
        self.assertEqual(fx.dot_q20([Q//2],[-3]),-1)

    def test_quantization_half_values_away_from_zero(self):
        self.assertEqual(fx.quantize_q20(0.5/Q),1)
        self.assertEqual(fx.quantize_q20(-0.5/Q),-1)
        for value in (float('nan'),float('inf'),129.0):
            with self.subTest(value=value),self.assertRaises(ValueError):fx.quantize_q20(value)

    def test_immutable_coefficients_and_bad_design_arguments(self):
        out=fx.step_q16((0,0,0,0),(S,0),0,tuple(coefficients()))
        self.assertFalse(out['fault'])
        for kwargs in (dict(r=0),dict(q=(30,True,20,20)),dict(qw=(100,float('nan'))),
                       dict(counts_per_rev=True),dict(adc_per_rad=-1)):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):fx.design_coefficients(**kwargs)

    def test_simple_measurement_feedback_and_sign(self):
        out=fx.step_q16([0]*4,[2*S,-3*S],0,coefficients(),comp_compare=0,comp_gain_q20=0)
        self.assertFalse(out['fault'])
        self.assertEqual(out['state'],[2*S,-3*S,0,0])
        self.assertEqual(out['command'],2)
        self.assertEqual(out['u_model_q16'],2*S)

    def test_prediction_uses_previous_applied_virtual_input(self):
        co=coefficients();co[16]=Q//2;co[20:28]=[0]*8
        out=fx.step_q16([S,0,0,0],[0,0],4*S,co,comp_compare=0,comp_gain_q20=0)
        self.assertEqual(out['state'][0],3*S)
        self.assertEqual(out['command'],3)

    def test_correction_rounds_combined_bias_only_once(self):
        co=coefficients();co[20]=Q//2
        out=fx.step_q16([-1,0,0,0],[0,0],0,co,comp_compare=0,comp_gain_q20=0)
        self.assertEqual(out['state'][0],0)

    def test_compensation_blending_and_saturation(self):
        for value,command,model in ((S//2,4,S//2),(5*S,40,5*S),
                                    (25*S,200,25*S),(1000*S,1000,825*S)):
            with self.subTest(value=value):
                out=fx.step_q16([0]*4,[value,0],0,coefficients())
                self.assertEqual(out['command'],command)
                self.assertEqual(out['u_model_q16'],model)
        negative=fx.step_q16([0]*4,[-1000*S,0],0,coefficients())
        self.assertEqual((negative['command'],negative['u_model_q16']),(-1000,-825*S))

    def test_pwm_integer_quantization_is_included_in_predictor_input(self):
        out=fx.step_q16([0]*4,[S//3,0],0,coefficients())
        self.assertEqual(out['command'],2)
        self.assertEqual(out['u_model_q16'],S//4)

    def test_large_feedback_saturates_before_int32_narrowing(self):
        co=coefficients();co[28]=-128*Q
        out=fx.step_q16([0]*4,[1000*S,0],0,co,comp_compare=0,comp_gain_q20=0)
        self.assertFalse(out['fault'])
        self.assertEqual(out['command'],1000)

    def test_state_overflow_fault_preserves_uncommitted_state(self):
        co=coefficients();co[16]=Q;old=[(1<<31)-1,0,0,0]
        out=fx.step_q16(old,[0,0],S,co)
        self.assertTrue(out['fault'])
        self.assertEqual(out['state'],old)
        self.assertEqual((out['command'],out['u_model_q16']),(0,0))

    def test_error_subtraction_overflow_is_not_wrapped(self):
        co=coefficients();co[20:28]=[0]*8
        out=fx.step_q16([-(1<<31),0,0,0],[(1<<31)-1,0],0,co)
        self.assertTrue(out['fault'])

    def test_bad_coefficients_and_compensation_are_rejected(self):
        for co in ([],[0]*31,[0]*31+[True],[129*Q]+[0]*31):
            with self.subTest(co=co),self.assertRaises(ValueError):fx.step_q16([0]*4,[0]*2,0,co)
        for kw in (dict(max_compare=1126),dict(comp_compare=251),dict(blend_compare=0),dict(comp_gain_q20=6*Q)):
            with self.subTest(kw=kw),self.assertRaises(ValueError):fx.step_q16([0]*4,[0]*2,0,coefficients(),**kw)

    def test_native_unit_conversion_changes_both_observer_bases(self):
        co=fx.design_coefficients()
        self.assertEqual(len(co),32)
        self.assertAlmostEqual(co[26]/Q,7.7363607178,delta=1/Q)
        self.assertAlmostEqual(co[28]/Q,-3.3706179345,delta=1/Q)
        positive=fx.step_q16([0]*4,[0,S],0,co)
        self.assertGreater(positive['command'],0)

    def test_fixed_observer_stays_close_to_independent_float_recurrence(self):
        co=fx.design_coefficients();state=[0]*4;floating=[0.0]*4;previous=0
        for k in range(300):
            y=[round(8*math.sin(k*.03))*S,round(2*math.sin(k*.04))*S]
            pred=[sum(co[4*i+j]/Q*floating[j] for j in range(4))+co[16+i]/Q*previous/S for i in range(4)]
            error=[y[0]/S-pred[0],y[1]/S-pred[1]]
            floating=[pred[i]+co[20+2*i]/Q*error[0]+co[21+2*i]/Q*error[1] for i in range(4)]
            out=fx.step_q16(state,y,previous,co)
            self.assertFalse(out['fault'])
            self.assertLess(max(abs(x/S-z) for x,z in zip(out['state'],floating)),0.02)
            state,previous=out['state'],out['u_model_q16']

    def test_independent_fixed_and_float_controllers_close_the_same_plant(self):
        import j280_balance_design as plant
        design=plant.design((30,200,20,20),.05)
        for case in ({},{'alpha0_deg':-3},{'gain':.7,'adc_scale':.8}):
            with self.subTest(case=case):
                floating=plant.simulate(design['K'],design['L'],design['Ad'],design['Bd'],
                                        case,seconds=6,mode='firmware',substeps=16,seed=1)
                fixed=plant.simulate(None,None,None,None,case,seconds=6,mode='firmware',
                                     substeps=16,seed=1,controller=fx.FixedController(fx.design_coefficients()))
                self.assertTrue(floating['accepted'],floating)
                self.assertTrue(fixed['accepted'],fixed)
                self.assertLess(fixed['tail_alpha_rms_deg'],2.)
                self.assertLess(abs(fixed['tail_alpha_rms_deg']-floating['tail_alpha_rms_deg']),.75)

    def test_fixed_controller_virtual_input_and_state_persist_between_samples(self):
        controller=fx.FixedController(coefficients())
        case={'comp':7.,'blend':1.,'u_max':40.}
        # 1000 compare units in the count-native fixture; duty saturates to40%.
        first=controller((1000*2*math.pi/1040,0.),case)
        self.assertEqual(first['u_applied_percent'],40.)
        self.assertEqual(first['u_model_percent'],33.)
        self.assertEqual(controller.steps,1)
        self.assertEqual(controller.previous_model_q16,825*S)


if __name__=='__main__':unittest.main()
