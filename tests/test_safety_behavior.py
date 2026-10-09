"""Exercise actual node methods with ROS message/transport stubs; no FC access."""
import importlib.util
import math
import pathlib
import sys
import types
import unittest
from unittest.mock import patch
from indoor_nav.core import Transform2D, fence_reason, UwbGate, FreezeWindow
ROOT = pathlib.Path(__file__).resolve().parents[1]
class Msg:
    def __init__(self, *args, **kw):
        self.__dict__.update(kw)
        if args:
            self.success, self.message = args

def load(name):
    modules = {}
    for mod, attrs in {
        'rospy': {'get_time': lambda: 10., 'get_param': lambda name, default=None: default, 'logwarn': lambda *a: None},
        'geometry_msgs.msg': dict.fromkeys(['PoseStamped','TransformStamped','TwistStamped','PointStamped'], Msg),
        'mavros_msgs.msg': {'State': Msg},
        'mavros_msgs.srv': dict.fromkeys(['ParamGet','SetMode'], Msg),
        'std_msgs.msg': dict.fromkeys(['String','Float64'], Msg),
        'std_srvs.srv': dict.fromkeys(['Trigger','TriggerResponse'], Msg),
        'indoor_nav.msg': dict.fromkeys(['LocalizationStatus','SafetyStatus'], Msg),
    }.items():
        modules[mod] = types.ModuleType(mod)
        modules[mod].__dict__.update(attrs)
    spec = importlib.util.spec_from_file_location(name, ROOT/'src/indoor_nav/scripts'/ (name+'.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module
S = load('safety_supervisor')
M = load('mission_manager')
BOX = dict(x_min=-1.1,x_max=1.1,y_min=-1.1,y_max=1.1,z_min=.3,z_max=1.3)
class SafetyBehavior(unittest.TestCase):
    def supervisor(self):
        s = S.SafetySupervisor.__new__(S.SafetySupervisor)
        s.cfg = {'frame_id':'world_lab'}
        s.box = BOX
        s.origin_verified = s.param_good = True
        s.operator_ack = s.abort_latched = False
        s.fc = Msg(armed=False,connected=True,mode='STABILIZE')
        s.fc_timestamp = s.localization_timestamp = s.world_timestamp = 10.
        s.localization = Msg(healthy=True,transform_locked=True)
        s.transform = Transform2D()
        s.z_offset = 0.
        s.world = Msg(header=Msg(frame_id='world_lab'),pose=Msg(position=Msg(x=0.,y=0.,z=0.)))
        s.freeze_detector=FreezeWindow()
        s.freeze_snapshot={}
        s.mission_state = 'IDLE'
        s.was_guided_flying = False
        return s
    def test_ack_is_not_ready_and_reset(self):
        s = self.supervisor()
        self.assertEqual(s.checks(10.)[1], 'OPERATOR_NOT_ACKNOWLEDGED')
        s.world.pose.position.x = 2.
        self.assertTrue(s.ack(None).success)
        self.assertEqual(s.checks(10.)[1], 'XY_OUTSIDE_FENCE')
        s.abort_latched = True
        self.assertTrue(s.reset(None).success)
        self.assertFalse(s.operator_ack)
        s.fc.armed = True
        self.assertFalse(s.ack(None).success)
        self.assertFalse(s.reset(None).success)
    def test_ground_and_strict_height(self):
        for armed in (False,True):
            self.assertEqual(fence_reason((0,0,0),BOX,armed), 'Z_BELOW_FENCE' if armed else 'READY')
        self.assertEqual(fence_reason((0,0,2),BOX), 'Z_ABOVE_FENCE')
        self.assertEqual(fence_reason((0,0,float('nan')),BOX),'NONFINITE_WORLD_POSE')
        for x in (-1.1,1.1):
            self.assertEqual(fence_reason((x,0,.8),BOX,True),'READY')
        self.assertEqual(fence_reason((1.1001,0,0),BOX),'XY_OUTSIDE_FENCE')
        hardware = dict(BOX,x_min=.4,y_min=.4)
        self.assertEqual(fence_reason((0,0,0),hardware),'XY_OUTSIDE_FENCE')
    def test_stale_and_wrong_frame(self):
        s = self.supervisor(); s.operator_ack=True
        self.assertTrue(s.checks(10.)[0])
        s.world.header.frame_id='odom'
        self.assertEqual(s.checks(10.)[1],'INVALID_WORLD_FRAME')
        s.world.header.frame_id='world_lab'; s.localization_timestamp=9.
        self.assertEqual(s.checks(10.)[1],'LOCALIZATION_UNHEALTHY')
    def test_start_never_arms_or_switches(self):
        m = M.MissionManager.__new__(M.MissionManager)
        m.mode='IDLE'; m.route=[{}]
        m.safestat=Msg(ready=True,abort_latched=False)
        m.locstat=Msg(healthy=True,transform_locked=True)
        m.transform=Transform2D(); m.world=Msg(); m.world_stamp=10.
        for armed,mode in [(False,'GUIDED'),(True,'STABILIZE')]:
            m.state=Msg(connected=True,armed=armed,mode=mode)
            self.assertFalse(m.start(None).success)
            self.assertEqual((m.state.armed,m.state.mode),(armed,mode))
    def test_dynamic_increment_and_faults(self):
        t=Transform2D(math.pi/2,.2,-.2)
        a=t.apply(0,0); b=t.apply(.1,.2)
        self.assertAlmostEqual(b[0]-a[0],-.2)
        self.assertAlmostEqual(b[1]-a[1],.1)
        gate=UwbGate()
        self.assertTrue(gate.evaluate(1,0,0,.01))
        self.assertTrue(gate.evaluate(1.1,.05,0,.01))
        self.assertFalse(gate.evaluate(1.2,1,0,.01))
        self.assertFalse(gate.fresh(1.4))
    def test_freeze_requires_motion_evidence(self):
        f=FreezeWindow()
        for i in range(27):
            triggered=f.observe(10+i*.04,(0.,0.),(.3,0.))
        self.assertTrue(triggered)
        self.assertEqual(f.snapshot['motion_source'],'EKF_VELOCITY_HEURISTIC')
        f.reset('TEST')
        for i in range(27):
            triggered=f.observe(10+i*.04,(0.,0.),(0.,0.))
        self.assertFalse(triggered)
if __name__ == '__main__':
    unittest.main()
