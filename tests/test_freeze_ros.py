"""Actual supervisor and mock callbacks, generated messages, mocked transport only."""
import json
import unittest
from unittest.mock import patch
from test_pipeline_ros_messages import ROS, module, Pub
if ROS:
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped, Pose
    from gazebo_msgs.msg import ModelStates
    from rosgraph_msgs.msg import Clock
    from mavros_msgs.msg import State
    from indoor_nav.msg import LocalizationStatus
    from indoor_nav.core import Transform2D

@unittest.skipUnless(ROS, 'ROS generated messages required')
class SupervisorFreeze(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.cfg={'profile':'sitl','expected_profile':'sitl','frame_id':'world_lab',
                  'geofence':dict(x_min=-1.1,x_max=1.1,y_min=-1.1,y_max=1.1,z_min=.3,z_max=1.3),
                  'operator_verified_fixed_ekf_origin':True,
                  'freeze_detection':{'enabled':True,'window_s':1.,'min_speed_mps':.2,'movement_epsilon_m':.015}}
        def param(name, default=None):
            return self.cfg if name=='/indoor_nav/safety' else default
        self.patches=[patch.object(rospy,'get_param',side_effect=param),
                      patch.object(rospy,'get_time',side_effect=lambda:self.now),
                      patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(self.now))]
        for n in ('init_node','Subscriber','Service','Timer','logwarn','logerr'):
            self.patches.append(patch.object(rospy,n))
        self.patches.append(patch.object(rospy,'Publisher',Pub))
        for p in self.patches:p.start()
        self.addCleanup(lambda:[p.stop() for p in reversed(self.patches)])
        self.s=module('safety_supervisor').SafetySupervisor()
        self.s.transform=Transform2D(yaw=1.5707963267948966)
        self.s.z_offset=0.; self.s.param_good=True; self.s.operator_ack=True

    def cycle(self, i, uwb_x=0., truth_x=0., velocity=.3):
        self.now=10+i*.04
        stamp=rospy.Time.from_sec(self.now)
        w=PoseStamped();w.header.stamp=stamp;w.header.frame_id='world_lab';w.pose.position.z=.8
        w.pose.position.x=uwb_x
        v=TwistStamped();v.header.stamp=stamp;v.header.frame_id='map';v.twist.linear.y=velocity
        u=PointStamped();u.header.stamp=stamp;u.header.frame_id='uwb_anchor';u.point.x=uwb_x
        p=Pose();p.position.x=truth_x
        loc=LocalizationStatus(healthy=True,transform_locked=True,uwb_age_s=.01,laser_age_s=.02,reason='OK')
        self.s.fc_cb(State(connected=True,armed=True,mode='GUIDED'))
        self.s.loc_cb(loc);self.s.world_cb(w);self.s.velocity_cb(v);self.s.accepted_uwb_cb(u)
        self.s.truth_cb(ModelStates(name=['iris'],pose=[p]));self.s.clock_cb(Clock(clock=stamp))
        return self.s.freeze_check(self.now)

    def test_trigger_snapshot_and_land_gate(self):
        for i in range(27):
            triggered=self.cycle(i,truth_x=.3*i*.04)
        self.assertTrue(triggered)
        self.assertEqual(self.s.checks(self.now)[1],'POSSIBLE_UWB_FREEZE')
        self.s.was_guided_flying=True;self.s.mission_state='RUNNING'
        with patch.object(self.s,'request_land') as land:
            self.s.tick(None)
            land.assert_called_once()
        snapshot=json.loads(self.s.abort_diagnostics.messages[-1].data)
        self.assertEqual(snapshot['reason'],'POSSIBLE_UWB_FREEZE')
        self.assertGreater(len(snapshot['freeze']['samples']),20)
        row=snapshot['freeze']['samples'][-1]
        self.assertAlmostEqual(row['velocity_world'][0],.3)
        self.assertAlmostEqual(row['velocity_world'][1],0.,places=8)
        for key in ('world_xyz','world_header_age_s','uwb_header_age_s','velocity_header_age_s','truth_age_s'):
            self.assertIn(key,row['context'])
        self.now+=1.
        self.s.freeze_check(self.now)
        self.assertEqual(json.loads(self.s.abort_diagnostics.messages[-1].data),snapshot)
        self.assertEqual(len(self.s.publisher.messages),0)

    def test_publisher_stop_retains_window_in_abort_snapshot(self):
        for i in range(27):
            self.cycle(i,uwb_x=.3*i*.04,truth_x=.3*i*.04)
        self.now+=.4
        self.s.fc_cb(State(connected=True,armed=True,mode='GUIDED'))
        self.s.loc_cb(self.s.localization)
        self.s.was_guided_flying=True;self.s.mission_state='RUNNING'
        with patch.object(self.s,'request_land'):
            self.s.tick(None)
        snapshot=json.loads(self.s.abort_diagnostics.messages[-1].data)
        self.assertEqual(snapshot['reason'],'WORLD_POSE_STALE')
        self.assertGreater(len(snapshot['freeze']['previous_window']['samples']),20)
        self.assertFalse(snapshot['freeze']['triggered'])

    def test_stationary_truth_does_not_trigger_despite_ekf_speed(self):
        for i in range(27):self.assertFalse(self.cycle(i,truth_x=.006*i*.04,uwb_x=.006*i*.04,velocity=.205))
        self.assertEqual(self.s.freeze_snapshot['classification'],'STATIONARY_WITH_EKF_VELOCITY_DISAGREEMENT')
        self.assertTrue(self.s.checks(self.now)[0])

    def test_old_header_replayed_with_new_receipt_is_not_fresh(self):
        self.cycle(0)
        self.now+=.4
        self.s.world_cb(self.s.world)
        self.assertFalse(self.s.freeze_check(self.now))
        self.assertEqual(self.s.freeze_snapshot['classification'],'LOCALIZATION_OR_FC_UNAVAILABLE')
        self.cycle(11)
        self.s.velocity.header.stamp=rospy.Time.from_sec(9.)
        self.s.velocity_cb(self.s.velocity)
        self.assertFalse(self.s.freeze_check(self.now))
        self.assertEqual(self.s.checks(self.now)[1],'VELOCITY_STALE_OR_TRANSFORM_MISSING')

    def test_invalid_frame_and_truth_failure_not_freeze(self):
        self.cycle(0)
        self.s.velocity.header.frame_id='base_link'
        self.s.freeze_check(self.now)
        self.assertEqual(self.s.checks(self.now)[1],'INVALID_VELOCITY_FRAME')
        self.cycle(1)
        self.s.truth_cb(ModelStates(name=[],pose=[]))
        self.s.freeze_check(self.now)
        self.assertEqual(self.s.checks(self.now)[1],'GAZEBO_MODEL_MISSING_OR_INVALID')
        self.cycle(2)
        self.s.clock_progress_receipt=self.now-.4
        self.s.freeze_check(self.now)
        self.assertEqual(self.s.checks(self.now)[1],'GAZEBO_CLOCK_STALLED_OR_MISSING')

    def test_clock_rewind_is_latched_until_disarmed_reset(self):
        self.cycle(0)
        self.s.clock_cb(Clock(clock=rospy.Time.from_sec(1.)))
        self.cycle(1)
        self.s.freeze_check(self.now)
        self.assertEqual(self.s.checks(self.now)[1],'GAZEBO_CLOCK_REWIND')
        self.assertFalse(self.s.reset(None).success)
        self.s.fc.armed=False
        self.assertTrue(self.s.reset(None).success)
        self.assertFalse(self.s.gazebo_clock_rewound)

    def test_new_world_messages_are_not_new_uwb_samples(self):
        self.cycle(0)
        stamp=self.s.accepted_uwb[0].header.stamp
        for i in range(1,5):
            self.cycle(i)
            self.s.accepted_uwb[0].header.stamp=stamp
            self.s.freeze_check(self.now)
        # Repeated source stamp cannot accumulate a complete window.
        self.assertFalse(self.s.freeze_snapshot['triggered'])

if __name__=='__main__':unittest.main()
