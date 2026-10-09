import math
import random
import unittest
from indoor_nav.core import Transform2D, rotate_vector, yaw_quaternion
from indoor_nav.velocity import (DeviceCounterGate,VelocityWindow,native_body_velocity,
                                select_velocity,copter457_position_error,position_only_adapter_guard)

class TimingTests(unittest.TestCase):
    def test_duplicate_disorder_and_wrap(self):
        g=DeviceCounterGate()
        self.assertTrue(g.accept(1.,10,0xfffffffe))
        self.assertFalse(g.accept(1.01,11,0xfffffffe))
        self.assertFalse(g.accept(1.02,12,0xfffffffd))
        self.assertTrue(g.accept(1.04,13,0xffffffff))
        self.assertTrue(g.accept(1.08,14,0))
        self.assertTrue(g.accept(1.12,15,1))
    def test_missing_time_and_receipt_rewind(self):
        g=DeviceCounterGate()
        self.assertTrue(g.accept(1.))
        self.assertTrue(g.accept(1.04))
        self.assertEqual(g.reason,'NO_DEVICE_COUNTER_RECEIPT_ONLY')
        self.assertFalse(g.accept(.5))

class VelocityTests(unittest.TestCase):
    def fit(self,fn,count=40,**kw):
        f=VelocityWindow(**kw)
        for i in range(count):f.add(10+i*.04,fn(i*.04))
        return f
    def test_linear_velocity_covariance_delay(self):
        f=self.fit(lambda t:(.3*t,-.2*t))
        d=f.estimate(11.56)
        self.assertTrue(d['valid'])
        self.assertAlmostEqual(d['velocity_world_xy'][0],.3)
        self.assertAlmostEqual(d['velocity_world_xy'][1],-.2)
        self.assertAlmostEqual(d['delay_s'],.24)
        self.assertGreaterEqual(min(d['variance_world_xy']),.05**2)
        self.assertTrue(d['position_velocity_correlated'])
    def test_static_freeze_is_ambiguous_not_valid_zero(self):
        d=self.fit(lambda t:(.3,.4)).estimate(11.56)
        self.assertFalse(d['valid'])
        self.assertEqual(d['reason'],'STATIONARY_OR_FROZEN_AMBIGUOUS')
        self.assertAlmostEqual(d['velocity_world_xy'][0],0)
    def test_initialization_stale_gap_rewind(self):
        f=VelocityWindow();f.add(1,(0,0))
        self.assertFalse(f.estimate(1.01)['valid'])
        f=self.fit(lambda t:(.3*t,0))
        self.assertEqual(f.estimate(12.)['reason'],'STALE_OR_TIME_RESET')
        self.assertTrue(f.add(12.,(.5,0)))
        self.assertEqual(len(f.samples),1)
        self.assertFalse(f.add(11.,(.6,0)))
        self.assertEqual(len(f.samples),0)
    def test_rejected_nonfinite_and_excessive_velocity(self):
        f=VelocityWindow()
        self.assertFalse(f.add(1,(float('nan'),0)))
        d=self.fit(lambda t:(3*t,0)).estimate(11.56)
        self.assertFalse(d['valid']);self.assertEqual(d['reason'],'EXCESSIVE_SPEED')
    def test_huber_window_reduces_outlier_effect(self):
        rng=random.Random(41)
        f=VelocityWindow(position_sigma_m=.01)
        for i in range(40):
            t=i*.04;x=.3*t+rng.gauss(0,.002)
            if i==32:x+=.20
            f.add(10+t,(x,0))
        self.assertLess(abs(f.estimate(11.56)['velocity_world_xy'][0]-.3),.05)
    def test_native_selection_requires_fresh_verified_data(self):
        d=self.fit(lambda t:(.3*t,0)).estimate(11.56)
        self.assertEqual(select_velocity(d,(.5,0,0),False)['source'],'DERIVED_WINDOW')
        self.assertEqual(select_velocity(d,(.5,0,0),True)['source'],'NATIVE_UWB')
        stale=dict(d,sample_age_s=1.,valid=False,source='UNAVAILABLE')
        self.assertFalse(select_velocity(stale,(.5,0,0),True)['valid'])
        self.assertFalse(select_velocity(d,(.5,0,0),True,frozen=True)['valid'])
        older=dict(d,sample_age_s=.3,valid=False,source='UNAVAILABLE')
        self.assertEqual(select_velocity(older,(.5,0,0),True,max_age=.4)['source'],'NATIVE_UWB')
    def test_rotation_never_adds_translation_and_body_frd(self):
        t=Transform2D(math.pi/2,100,-100)
        vx,vy=t.rotate(.3,.1)
        self.assertAlmostEqual(vx,-.1);self.assertAlmostEqual(vy,.3)
        body=rotate_vector(yaw_quaternion(-math.pi/2),(vx,vy,0))
        self.assertAlmostEqual(body[0],.3);self.assertAlmostEqual(body[1],.1)
        frd=(body[0],-body[1],-body[2]);ned=(vy,vx,0)
        self.assertAlmostEqual(frd[1],-.1);self.assertAlmostEqual(ned[0],.3)
    def test_native_and_derived_lever_arm_compensation(self):
        self.assertEqual(native_body_velocity((0,.2,0),(0,0,2),(.1,0,0),yaw_quaternion(0)),(0.,0.,0.))
        f=VelocityWindow()
        for i in range(40):
            t=i*.04;q=yaw_quaternion(t)
            tag=rotate_vector(q,(.1,0,0));body=(tag[0]-tag[0],tag[1]-tag[1])
            f.add(10+t,body)
        self.assertAlmostEqual(f.estimate(11.56)['velocity_world_xy'][0],0)

class TransportTests(unittest.TestCase):
    @staticmethod
    def params():
        p={'GPS_TYPE':0,'AHRS_EKF_TYPE':3,'EK3_ENABLE':1,'VISO_TYPE':3,'EK3_SRC1_POSXY':6,'VISO_POS_M_NSE':.3}
        for i in (1,2,3):
            for f,v in [('VELXY',0),('VELZ',0),('POSZ',1),('YAW',1)]:p['EK3_SRC{}_{}'.format(i,f)]=v
        return p
    def test_old_covariance_saturates_unknown_uses_configured_floor(self):
        self.assertEqual(copter457_position_error((.01,.01,1000)),100.)
        self.assertEqual(copter457_position_error((float('nan'),.01,1000),.3),.3)
    def test_adapter_refuses_hardware_wrong_version_low_noise_or_any_external_velocity(self):
        p=self.params()
        self.assertTrue(position_only_adapter_guard('sitl',(4,5,7),p,.2)[0])
        for profile,fw,changes in [('hardware',(4,5,7),{}),('sitl',(4,6,0),{}),
               ('sitl',(4,5,7),{'VISO_POS_M_NSE':.1}),('sitl',(4,5,7),{'EK3_SRC2_VELXY':6}),
               ('sitl',(4,5,7),{'EK3_SRC3_YAW':7}),('sitl',(4,5,7),{'EK3_SRC1_POSZ':6})]:
            self.assertFalse(position_only_adapter_guard(profile,fw,dict(p,**changes),.2)[0])
        p.pop('EK3_SRC3_VELZ')
        self.assertFalse(position_only_adapter_guard('sitl',(4,5,7),p,.2)[0])
if __name__=='__main__':unittest.main()
