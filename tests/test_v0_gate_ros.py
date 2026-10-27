import importlib.util
import json
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch

from test_pipeline_ros_messages import ROS
from test_v0 import config, feed
from indoor_nav.v0 import Health

if ROS:
    from geometry_msgs.msg import PoseStamped
    import rospy
    from std_msgs.msg import String
    spec = importlib.util.spec_from_file_location('v0_health_gate', Path(__file__).resolve().parents[1] / 'src/indoor_nav/scripts/v0_nav_health.py')
    gate_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gate_module)


@unittest.skipUnless(ROS, 'ROS messages required')
class GateROS(unittest.TestCase):
    def setUp(self):
        self.gate = gate_module.Watchdog.__new__(gate_module.Watchdog)
        self.gate.config = config()
        self.gate.health = Health(self.gate.config)
        feed(self.gate.health, 3.)
        self.gate.health.update_ekf_origin((1., 2., 3.))
        self.gate.graph_time = self.gate.received = self.gate.status_time = self.gate.target_time = 3.
        self.gate.graph_reason = ''
        self.gate.previous_mono = None
        self.gate.enabled = True
        self.gate.yaml_sha256 = 'hash'
        self.gate.status = {'state': 'EXEC_WAYPOINT', 'yaml_sha256': 'hash', 'control_enabled': True}
        self.gate.target = PoseStamped()
        self.gate.target.header.frame_id = 'map'
        self.gate.target.header.stamp = rospy.Time.from_sec(103.)
        self.gate.target.pose.orientation.w = 1.
        self.gate.target.pose.position.x = 10.
        self.gate.target.pose.position.y = 20.
        self.gate.target.pose.position.z = 1.
        self.gate.origin = self.gate.target
        self.gate.lock = threading.RLock()
        self.gate.record = Mock()
        self.clock = patch.object(gate_module.rospy, 'get_time', return_value=103.)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_independent_pose_validation(self):
        self.assertEqual(self.gate.fault(3., 103.), '')
        self.gate.health.pose_time = 2.
        self.assertEqual(self.gate.fault(3., 103.), 'POSE_STALE')

    def test_manager_healthy_cannot_mask_gate_graph_or_config_failure(self):
        self.gate.graph_reason = 'OTHER_CONTROL_PUBLISHER'
        self.assertEqual(self.gate.fault(3., 103.), 'OTHER_CONTROL_PUBLISHER')
        self.gate.graph_reason = ''
        self.gate.status['yaml_sha256'] = 'different'
        self.assertEqual(self.gate.fault(3., 103.), 'MANAGER_GATE_CONFIG_MISMATCH')

    def test_invalid_targets_and_time(self):
        self.gate.target.pose.position.x = math.nan
        self.assertEqual(self.gate.fault(3., 103.), 'INVALID_TARGET_FRAME_OR_POSITION')
        self.gate.target.pose.position.x = 10.
        self.gate.previous_mono, self.gate.previous_ros = 2.9, 104.
        self.assertEqual(self.gate.fault(3., 103.), 'GATE_CLOCK_OR_CYCLE_TIMEOUT')

    def test_no_blind_mode_request_or_dry_run_control(self):
        self.gate.config['control']['exit_action'] = 'brake_if_position_healthy'
        with patch.object(gate_module.rospy, 'ServiceProxy') as proxy:
            self.gate.revoke_action('POSE_STALE', False)
            self.gate.enabled = False
            self.gate.revoke_action('MISSION_HEARTBEAT_TIMEOUT', True)
            proxy.assert_not_called()
        self.assertFalse(self.gate.exit_result['requested'])

    def test_only_explicit_healthy_exit_requests_brake(self):
        self.gate.config['control']['exit_action'] = 'brake_if_position_healthy'
        with patch.object(gate_module.rospy, 'wait_for_service'), patch.object(gate_module.rospy, 'ServiceProxy') as proxy:
            proxy.return_value.return_value.mode_sent = True
            self.gate.revoke_action('MANAGER_REVOKED', True)
            proxy.return_value.assert_called_once_with(0, 'BRAKE')
        self.assertTrue(self.gate.exit_result['requested'])

    def test_malformed_status_cannot_refresh_authorization(self):
        before = self.gate.status_time
        self.gate.status_cb(String(data=json.dumps(['not', 'a', 'status'])))
        self.assertEqual(self.gate.status, {})
        self.assertEqual(self.gate.status_time, before)
