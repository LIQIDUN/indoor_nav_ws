import importlib.util
import struct
import threading
import unittest
from unittest.mock import Mock, patch

from test_pipeline_ros_messages import ROS
from test_v0 import ROOT, config, feed
from indoor_nav.v0 import Health, Mission

if ROS:
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import Mavlink, State
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Header, UInt8MultiArray
    import test_v0_gate_ros as gate_tests
    gate_module = gate_tests.gate_module
    spec = importlib.util.spec_from_file_location('v0_receipt_manager', ROOT / 'src/indoor_nav/scripts/relative_mission_manager.py')
    manager_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(manager_module)


@unittest.skipUnless(ROS, 'source ROS + devel for actual-message offline callbacks')
class ReceiptCallbacks(unittest.TestCase):
    def setUp(self):
        gate_tests.GateROS.setUp(self)
        self.manager = manager_module.RelativeMissionManager.__new__(manager_module.RelativeMissionManager)
        self.manager.config = config()
        self.manager.health = Health(self.manager.config)
        feed(self.manager.health, 3.)
        self.manager.mission = Mission(self.manager.config)
        self.manager.lock = threading.RLock()
        self.manager.record = Mock()
        self.manager.watchdog_time = 3.
        self.manager.ekf_report = {}

    def pose(self, stamp=102.98, frame='map'):
        message = PoseStamped()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = frame
        message.pose.orientation.w = 1.
        message.pose.position.x, message.pose.position.y, message.pose.position.z = 10., 20., 1.
        return message

    def test_both_nodes_preserve_original_header_but_accept_receipt(self):
        message = self.pose()
        original = message.header.stamp
        with patch.object(gate_module.time, 'monotonic', return_value=3.1), patch.object(rospy, 'get_time', return_value=103.1):
            for node in (self.manager, self.gate):
                node.pose_cb(message)
                self.assertEqual(node.health.rx_ros['pose'], 103.1)
                self.assertEqual(node.health.pose_stamp, 102.98)
                self.assertEqual(node.health.fault, '')
        self.assertEqual(message.header.stamp, original)

    def test_continuing_velocity_with_raw_zero_is_not_a_transport_failure(self):
        message = TwistStamped()
        message.header.frame_id = 'base_link'
        with patch.object(gate_module.time, 'monotonic', return_value=3.1):
            for node in (self.manager, self.gate):
                node.velocity_cb(message)
                self.assertEqual(node.health.rx_ros['velocity'], 103.)
                self.assertEqual(node.health.velocity, (0., 0., 0.))
                self.assertEqual(node.health.fault, '')

    def test_gps_and_ekf_quality_survive_stamp_simplification(self):
        fix = NavSatFix(latitude=30., longitude=110., altitude=200.)
        fix.status.status = 0
        payload = struct.pack('<5fH', .1, .2, .3, .4, 0., 63)
        report = Mavlink(msgid=193, sysid=1, compid=1, framing_status=Mavlink.FRAMING_OK,
                         len=22, payload64=struct.unpack('<3Q', payload.ljust(24, b'\0')))
        with patch.object(gate_module.time, 'monotonic', return_value=3.1):
            for node in (self.manager, self.gate):
                node.gps_cb(fix)
                self.assertTrue(node.health.gps_ok)
                callback = node.ekf_report_cb if node is self.manager else node.ekf_cb
                callback(report)
                self.assertTrue(node.health.ekf_ok)
                report.payload64 = []
                callback(report)
                self.assertFalse(node.health.ekf_ok)
                report.payload64 = struct.unpack('<3Q', payload.ljust(24, b'\0'))
                fix.status.status = -1
                node.gps_cb(fix)
                self.assertFalse(node.health.gps_ok)
                fix.status.status = 0

    def test_frame_error_and_nan_remain_fatal(self):
        self.manager.pose_cb(self.pose(frame='odom'))
        self.assertEqual(self.manager.health.fault, 'POSE_FRAME_INVALID')
        message = self.pose()
        message.pose.position.x = float('nan')
        self.gate.pose_cb(message)
        self.assertEqual(self.gate.health.fault, 'NONFINITE_POSE')

    def test_gate_command_deadline_is_reception_based_and_configurable(self):
        self.gate.heartbeat(Header(stamp=rospy.Time.from_sec(1.)))
        self.gate.target_cb(self.gate.target)
        self.gate.target.header.stamp = rospy.Time.from_sec(1.)
        with patch.object(gate_module.time, 'monotonic', return_value=3.):
            self.gate.heartbeat(Header())
            self.gate.target_cb(self.gate.target)
        self.assertEqual(self.gate.fault(3., 103.), '')
        self.gate.config['msg_timeout']['command_timeout'] = .2
        self.gate.rx_ros['target'] = 102.7
        self.assertEqual(self.gate.fault(3., 103.), 'TARGET_OR_ORIGIN_STALE')

    def test_gate_clock_reset_clears_commands_and_revokes_active_lease(self):
        self.gate.lease.owner = 'test'
        self.gate.health.observe_clock(1.)
        self.assertEqual(self.gate.fault(3.1, 1.), 'ROS_CLOCK_RESET')
        self.assertFalse(self.gate.rx_ros)
        self.assertIsNone(self.gate.target)
        self.gate.lease.step(None, 'test', True, 'GUIDED', 'ROS_CLOCK_RESET', False)
        self.assertTrue(self.gate.lease.revoked)
        self.assertFalse(self.gate.brake_eligible(3.1, 1., 'ROS_CLOCK_RESET'))

    def test_idle_gate_still_requires_a_connected_fresh_fcu_state(self):
        self.gate.status['state'] = 'IDLE'
        self.gate.health.state = (False, False, '')
        self.assertEqual(self.gate.fault(3., 103.), 'DISCONNECTED')
        self.gate.health.state = (True, False, 'STABILIZE')
        self.gate.health.state_time = 0.
        self.gate.health.rx_ros['state'] = 100.
        self.assertEqual(self.gate.fault(3.1, 103.1), 'STATE_RX_TIMEOUT')

    def test_manager_clock_reset_latches_even_when_clock_recovers_before_tick(self):
        for index in range(1, 31):
            feed(self.manager.health, 3. + index / 10)
        self.assertTrue(self.manager.mission.start(6., 106., self.manager.health)[0])
        self.manager.health.observe_clock(1.)
        feed(self.manager.health, 6.1)
        self.manager.check_clock(6.1, 106.1)
        self.assertEqual(self.manager.mission.reason, 'ROS_CLOCK_RESET')
        self.assertIsNone(self.manager.watchdog_time)
        self.assertFalse(self.manager.mission.start(6.1, 106.1, self.manager.health)[0])

    def test_disconnect_and_manual_takeover_latch_before_a_reconnect_callback(self):
        for mode, connected in (('GUIDED', False), ('ALT_HOLD', True)):
            self.setUp()
            self.gate.lease.owner = 'test'
            self.manager.mission.transition('EXEC_WAYPOINT', 3.)
            with patch.object(gate_module.time, 'monotonic', return_value=3.1):
                for node in (self.gate, self.manager):
                    node.state_cb(State(connected=connected, armed=True, mode=mode))
                    node.state_cb(State(connected=True, armed=True, mode='GUIDED'))
            self.assertTrue(self.gate.lease.revoked)
            self.assertFalse(self.gate.exit_result['requested'])
            self.assertEqual(self.manager.mission.state, 'ABORT_LATCHED')

    def test_acquisition_messages_cannot_keep_missing_navigation_pose_alive(self):
        acquisition_receiver = Mock()
        routes = {'/motor_mic/audio_raw': acquisition_receiver,
                  '/sound_sensor/audio_raw': acquisition_receiver,
                  '/mavros/local_position/pose': self.manager.pose_cb}
        with patch.object(manager_module.time, 'monotonic', return_value=3.1), patch.object(rospy, 'get_time', return_value=103.1):
            routes['/mavros/local_position/pose'](self.pose())
        pose_rx = self.manager.health.rx_ros['pose']
        for index in range(1000):
            routes['/motor_mic/audio_raw'](UInt8MultiArray(data=[index % 256] * 192))
            routes['/sound_sensor/audio_raw'](UInt8MultiArray(data=[index % 256] * 64))
        self.assertEqual(acquisition_receiver.call_count, 2000)
        self.assertEqual(self.manager.health.rx_ros['pose'], pose_rx)
        self.assertEqual(self.manager.health.reason(3.7, 103.7), 'POSE_RX_TIMEOUT')


if __name__ == '__main__':
    unittest.main()
