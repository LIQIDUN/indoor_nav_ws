import json
import unittest
from unittest.mock import patch
from test_pipeline_ros_messages import ROS, module, Pub
if ROS:
    import rospy
    from geometry_msgs.msg import Pose
    from gazebo_msgs.msg import ModelStates
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, Range
    from mavros_msgs.msg import State
    from indoor_nav.core import Transform2D

@unittest.skipUnless(ROS, 'ROS generated messages required')
class _TransportFixture(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.params={}
        self.patches=[patch.object(rospy,'get_time',side_effect=lambda:self.now),
                      patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(self.now)),
                      patch.object(rospy,'get_param',side_effect=lambda n,d=None:self.params.get(n,d)),
                      patch.object(rospy,'Publisher',Pub)]
        for n in ('init_node','Subscriber','Service','Timer','logwarn','logwarn_throttle'):
            self.patches.append(patch.object(rospy,n))
        for p in self.patches:p.start()
        self.addCleanup(lambda:[p.stop() for p in reversed(self.patches)])
        self.m=module('mock_uwb_node').MockUwb()
        self.truth()
    def truth(self,x=0.):
        pose=Pose();pose.position.x=x;pose.position.z=.8
        self.m.gazebo_cb(ModelStates(name=['iris'],pose=[pose]))
        self.m.clock_cb(Clock(clock=rospy.Time.from_sec(self.now)))

@unittest.skipUnless(ROS, 'ROS generated messages required')
class MockFaults(_TransportFixture):
    def test_fresh_stationary_truth_keeps_publishing(self):
        for i in range(30):
            self.now+=.04;self.truth();self.m.generate(None);self.m.flush(None)
        self.assertEqual(len(self.m.air_pub.messages),30)
        self.assertEqual(json.loads(self.m.diag_pub.messages[-1].data)['reason'],'OK')
    def test_freeze_injection_keeps_range_and_changes_truth(self):
        self.m.generate(None);self.m.flush(None)
        old=list(self.m.air_pub.messages[-1].pos_3d)
        self.params['/indoor_nav/mock/uwb_sim/freeze']=True
        self.now+=.04;self.truth(.2);self.m.generate(None);self.m.flush(None)
        self.assertEqual(list(self.m.air_pub.messages[-1].pos_3d),old)
        self.assertEqual(len(self.m.range_pub.messages),2)
        d=json.loads(self.m.diag_pub.messages[-1].data)
        self.assertEqual(d['reason'],'FREEZE_INJECTED')
        self.assertEqual(d['truth_xyz'][0],.2)
    def test_drop_all_stops_uwb_but_not_range(self):
        self.params['/indoor_nav/mock/uwb_sim/drop_all']=True
        self.m.generate(None);self.m.flush(None)
        self.assertEqual(len(self.m.air_pub.messages),0)
        self.assertEqual(len(self.m.range_pub.messages),1)
    def test_missing_invalid_truth_clears_delayed_queue(self):
        self.m.base_cfg['delay_ms']=500
        self.m.generate(None)
        self.assertEqual(len(self.m.queue),1)
        self.m.gazebo_cb(ModelStates(name=['iris'],pose=[]))
        self.assertIsNone(self.m.truth)
        self.assertEqual(len(self.m.queue),0)
        self.m.generate(None)
        self.assertEqual(json.loads(self.m.diag_pub.messages[-1].data)['reason'],'GAZEBO_MODEL_MISSING_OR_INVALID')
        self.truth(float('nan'));self.assertIsNone(self.m.truth)
    def test_clock_stall_drops_truth_even_if_pose_callbacks_continue(self):
        self.now+=.4
        p=Pose();p.position.z=.8
        self.m.gazebo_cb(ModelStates(name=['iris'],pose=[p]))
        self.m.generate(None)
        self.assertEqual(len(self.m.range_pub.messages),0)
        self.assertEqual(json.loads(self.m.diag_pub.messages[-1].data)['reason'],'GAZEBO_CLOCK_STALLED_OR_MISSING')

@unittest.skipUnless(ROS, 'ROS generated messages required')
class EstimatorPublication(_TransportFixture):
    def test_localization_diagnostics_identifies_disconnect_and_range_loss(self):
        w=module('world_estimator').WorldEstimator()
        w.transform=Transform2D();w.raw_xy=(0.,0.);w.z_offset=0.
        w.uwb.evaluate(self.now,0.,0.)
        imu=Imu();imu.orientation.w=1.;imu.header.stamp=rospy.Time.from_sec(self.now)
        w.imu_cb(imu)
        r=Range(range=.8,min_range=.1,max_range=2.);r.header.stamp=rospy.Time.from_sec(self.now)
        w.range_cb(r);w.state_cb(State(connected=True))
        w.publish(None);self.assertEqual(len(w.world_pub.messages),1)
        w.state_cb(State(connected=False));w.publish(None)
        self.assertEqual(len(w.world_pub.messages),1)
        self.assertEqual(json.loads(w.diagnostics_pub.messages[-1].data)['reason'],'FC_LOST')
        w.state_cb(State(connected=True));self.now+=.25
        w.uwb.evaluate(self.now,0.,0.)
        imu.header.stamp=rospy.Time.from_sec(self.now);w.imu_cb(imu)
        w.range_cb(r)  # old header replay must not refresh range timer
        w.publish(None)
        d=json.loads(w.diagnostics_pub.messages[-1].data)
        self.assertEqual(d['reason'],'RANGE_STALE')
        self.assertEqual(d['range_rejection'],'STALE_OR_FUTURE_HEADER')
        self.assertEqual(len(w.world_pub.messages),1)

if __name__=='__main__':unittest.main()
