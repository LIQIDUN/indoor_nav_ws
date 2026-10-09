import math
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from test_pipeline_ros_messages import ROS, Pub
from test_mock_faults_ros import _TransportFixture
if ROS:
    import rospy
    from mavros_msgs.msg import State
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from indoor_nav.velocity_ros import VelocityShadow
    from indoor_nav.external_nav import PositionOnlyAdapter
    from indoor_nav.core import Transform2D, yaw_quaternion
    import test_velocity

@unittest.skipUnless(ROS,'ROS messages required')
class ShadowROS(_TransportFixture):
    def test_shadow_only_and_stale_or_freeze_stops_publication(self):
        shadow=VelocityShadow({},'sitl',(0,0,0),'world_lab')
        for i in range(30):
            self.now=10+i*.04
            shadow.observe(self.now,(.3*i*.04,0),(0,0),(.9,0,0),yaw_quaternion(0),(0,0,0))
        d=shadow.report(self.now,True,'OK',Transform2D(math.pi/2,20,30),yaw_quaternion(math.pi/2))
        self.assertEqual(d['selected']['source'],'DERIVED_WINDOW')
        self.assertFalse(d['native_verified'])
        self.assertFalse(d['forwarded_to_fc'])
        self.assertAlmostEqual(d['velocity_nav_xy'][0],0,places=8)
        self.assertAlmostEqual(d['velocity_nav_xy'][1],.3,places=8)
        self.assertAlmostEqual(shadow.publisher.messages[-1].twist.twist.linear.x,.3)
        count=len(shadow.publisher.messages)
        d=shadow.report(self.now,True,'OK',Transform2D(),yaw_quaternion(0),frozen=True)
        self.assertFalse(d['selected']['valid'])
        self.assertEqual(len(shadow.publisher.messages),count)
        d=shadow.report(self.now+1,True,'OK',Transform2D(),yaw_quaternion(0))
        self.assertFalse(d['selected']['valid'])
        self.assertEqual(len(shadow.publisher.messages),count)
    def test_native_requires_evidence(self):
        with self.assertRaises(ValueError):VelocityShadow({'native':{'verified':True}},'hardware',(0,0,0),'world_lab')
    def test_adapter_readonly_validation_and_fail_closed(self):
        state=State(connected=True,armed=True)
        adapter=PositionOnlyAdapter('sitl',{'enabled':True},lambda:state)
        adapter.validate(None)
        self.assertIsNone(adapter.checked_at)
        state.armed=False
        vehicle=SimpleNamespace(autopilot=3,type=2,available_info=2,
                                flight_sw_version=(4<<24)|(5<<16)|(7<<8))
        params=test_velocity.TransportTests.params()
        params.update(SIM_SPEEDUP=1,VISO_POS_X=0,VISO_POS_Y=0,VISO_POS_Z=0)
        calls=[]
        def proxy(name,_type):
            calls.append(name)
            if name=='/mavros/vehicle_info_get':
                return lambda **kw:SimpleNamespace(success=True,vehicles=[vehicle])
            self.assertEqual(name,'/mavros/param/get')
            return lambda param_id:SimpleNamespace(success=True,
                value=SimpleNamespace(real=float(params[param_id]),integer=0))
        with patch.object(rospy,'wait_for_service'),patch.object(rospy,'ServiceProxy',side_effect=proxy):
            adapter.validate(None)
            self.assertEqual(adapter.checked_at,self.now)
            state.armed=True  # only initial validation requires disarmed
            adapter.validate(None)
            self.assertEqual(adapter.checked_at,self.now)
            vehicle.type=1  # ArduPlane version number is insufficient
            adapter.validate(None)
            self.assertIsNone(adapter.checked_at)
        self.assertTrue(all(c in ('/mavros/vehicle_info_get','/mavros/param/get') for c in calls))

    def test_adapter_preserves_ros_covariance_and_checks_freshness(self):
        with self.assertRaises(ValueError):PositionOnlyAdapter('hardware',{'enabled':True},lambda:State())
        a=PositionOnlyAdapter('sitl',{'enabled':True},lambda:State(connected=True))
        msg=Odometry();msg.pose.covariance[0]=.01;msg.pose.covariance[7]=.01;msg.pose.covariance[14]=1000
        self.assertIsNone(a.adapt(msg,self.now))
        a.params=test_velocity.TransportTests.params();a.firmware=(4,5,7);a.checked_at=self.now
        corrected=a.adapt(msg,self.now)
        self.assertTrue(math.isnan(corrected.pose.covariance[0]))
        self.assertEqual(msg.pose.covariance[0],.01)
        self.assertEqual(msg.pose.covariance[14],1000)
        self.assertEqual(corrected.pose.covariance[14],1000)
        self.assertIsNone(a.adapt(msg,self.now+6))
        a.checked_at=self.now;a.params['EK3_SRC2_VELXY']=6
        self.assertIsNone(a.adapt(msg,self.now))
if __name__=='__main__':unittest.main()
