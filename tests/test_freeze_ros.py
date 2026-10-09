"""Actual supervisor and mock callbacks, generated messages, mocked transport only."""
import json
import math
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


    def test_odometry_decode_preserves_stamp_and_stale_guard(self):
        from nav_msgs.msg import Odometry
        self.cycle(0)
        msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='base_link'
        msg.header.stamp=rospy.Time.from_sec(self.now-.31)
        msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=.4
        self.s.odometry_velocity_cb(msg)
        self.assertEqual(self.s.velocity.header.stamp,msg.header.stamp)
        self.s.freeze_check(self.now)
        self.assertEqual(self.s.freeze_snapshot['classification'],'VELOCITY_STALE_OR_TRANSFORM_MISSING')
        msg.header.stamp=rospy.Time.from_sec(self.now+.01)
        self.s.odometry_velocity_cb(msg);self.s.freeze_check(self.now)
        self.assertEqual(self.s.freeze_snapshot['classification'],'VELOCITY_STALE_OR_TRANSFORM_MISSING')
        msg.header.frame_id='base_link'
        self.s.odometry_velocity_cb(msg)
        self.assertIsNone(self.s.velocity)
        self.assertEqual(self.s.velocity_source_error,'INVALID_VELOCITY_FRAME')

    def test_decoded_velocity_drives_world_predictive_fence(self):
        from nav_msgs.msg import Odometry
        self.cycle(0)
        self.s.cfg['freeze_detection']['enabled']=False
        self.s.was_guided_flying=True
        self.s.world.pose.position.x=.9
        msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='base_link'
        msg.header.stamp=rospy.Time.from_sec(self.now)
        # Body forward at yaw=90 is nav north; calibration yaw=90 maps it to world +x.
        msg.pose.pose.orientation.z=math.sqrt(.5)
        msg.pose.pose.orientation.w=math.sqrt(.5)
        msg.twist.twist.linear.x=1.
        self.s.odometry_velocity_cb(msg)
        self.assertAlmostEqual(self.s.velocity.twist.linear.y,1.)
        self.assertEqual(self.s.checks(self.now)[1],'PREDICTED_FENCE_BREACH')

    def test_first_target_uses_existing_deadline_without_forwarding(self):
        from std_msgs.msg import String
        self.cycle(0)
        self.s.mission_state_cb(String(data='RUNNING'))
        self.s.tick(None)
        self.assertFalse(self.s.abort_latched)
        self.assertEqual(len(self.s.publisher.messages),0)
        self.cycle(12)
        # Duplicate RUNNING must not reset the first-target deadline.
        self.s.mission_state_cb(String(data='RUNNING'))
        self.s.tick(None)
        self.assertFalse(self.s.abort_latched)
        self.cycle(13)
        with patch.object(self.s,'request_land'):
            self.s.tick(None)
        self.assertTrue(self.s.abort_latched)
        self.assertEqual(self.s.abort_reason,'SETPOINT_TIMEOUT')
        self.assertEqual(len(self.s.publisher.messages),0)

    def test_small_future_velocity_waits_without_retimestamping(self):
        from nav_msgs.msg import Odometry
        self.cycle(0)
        old=self.s.velocity
        m=Odometry();m.header.frame_id='map';m.child_frame_id='base_link'
        m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=.8
        m.header.stamp=rospy.Time.from_sec(self.now+.005)
        self.s.odometry_velocity_cb(m)
        self.s.freeze_check(self.now)
        self.assertIs(self.s.velocity,old)
        self.assertIsNotNone(self.s.velocity_pending)
        received=self.now
        self.now+=.006
        self.s.freeze_check(self.now)
        self.assertIsNone(self.s.velocity_pending)
        self.assertAlmostEqual(self.s.velocity.twist.linear.x,.8)
        self.assertEqual(self.s.velocity.header.stamp,m.header.stamp)
        self.assertEqual(self.s.velocity_stamp,received)
        self.assertGreaterEqual(self.s.message_age(self.s.velocity,self.now),0.)

    def test_large_future_and_expired_pending_velocity_fail_closed(self):
        from nav_msgs.msg import Odometry
        self.cycle(0)
        m=Odometry();m.header.frame_id='map';m.child_frame_id='base_link'
        m.pose.pose.orientation.w=1.
        m.header.stamp=rospy.Time.from_sec(self.now+.021)
        self.s.odometry_velocity_cb(m)
        self.assertIsNone(self.s.velocity)
        self.assertEqual(self.s.velocity_source_error,'FUTURE_VELOCITY_HEADER')
        m.header.stamp=rospy.Time.from_sec(self.now+.005)
        self.s.odometry_velocity_cb(m)
        self.now+=.31
        self.s.release_pending_velocity(self.now)
        self.assertIsNone(self.s.velocity)
        self.assertIsNone(self.s.velocity_pending)
        self.assertEqual(self.s.velocity_source_error,'PENDING_VELOCITY_EXPIRED')

if __name__=='__main__':unittest.main()
