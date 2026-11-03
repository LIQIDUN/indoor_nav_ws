import copy
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_pipeline_ros_messages import ROS
from test_v0 import feed
from test_v0_research import research_config
from indoor_nav.v0 import Health, Mission

if ROS:
    import test_v0_gate_ros as gate_tests
    from test_v0_receipt_ros import manager_module


@unittest.skipUnless(ROS, 'ROS messages required')
class ResearchGateConsistency(unittest.TestCase):
    def setUp(self):
        gate_tests.GateROS.setUp(self)
        self.config = research_config()
        self.gate.config = self.config
        self.gate.health = Health(self.config)
        feed(self.gate.health, 3.)
        self.gate.health.update_ekf_origin((1., 2., 3.))
        self.health = copy.deepcopy(self.gate.health)
        self.mission = Mission(self.config)
        self.manager = manager_module.RelativeMissionManager.__new__(manager_module.RelativeMissionManager)
        self.manager.config = self.config
        self.manager.health = self.health
        self.manager.watchdog_time = self.manager.limit_time = 3.
        self.manager.watchdog_fault = self.manager.limit_error = ''

    def test_start_and_speed_warning_are_not_vetoed_by_watchdog(self):
        self.assertTrue(self.mission.start(3., 103., self.health, self.manager.external_reason(3.))[0])
        for index in range(31, 43):
            now = index / 10
            for health in (self.health, self.gate.health):
                feed(health, now, velocity=(.4, 0., 0.))
            self.gate.graph_time = self.gate.received = self.gate.status_time = self.gate.target_time = now
            self.gate.rx_ros = dict(heartbeat=100 + now, status=100 + now, target=100 + now)
            self.manager.watchdog_time = now
            self.mission.tick(now, 100 + now, self.health, self.manager.external_reason(now))
            self.assertEqual(self.gate.fault(now, 100 + now), '')
        self.assertEqual(len(self.mission.warnings), 1)
        self.assertNotEqual(self.mission.state, 'ABORT_LATCHED')

    def test_both_nodes_keep_same_localization_faults(self):
        for fault in ('POSE_JUMP', 'EXCESSIVE_TILT', 'EKF_ORIGIN_CHANGED'):
            self.health.fault = self.gate.health.fault = fault
            self.assertEqual(self.health.reason(3., 103.), fault)
            self.assertEqual(self.gate.fault(3., 103.), fault)
        self.health.fault = self.gate.health.fault = ''
        self.health.gps_ok = self.gate.health.gps_ok = False
        self.assertEqual(self.health.reason(3., 103.), 'GPS_EKF_UNHEALTHY')
        self.assertEqual(self.gate.fault(3., 103.), 'GPS_EKF_UNHEALTHY')

    def test_stale_target_and_target_or_actual_fence_remain_hard_interlocks(self):
        self.gate.target_time = 2.
        self.assertEqual(self.gate.fault(3., 103.), 'TARGET_OR_ORIGIN_STALE')
        self.gate.target_time = 3.
        self.gate.target = copy.deepcopy(self.gate.origin)
        self.gate.target.pose.position.x += .81
        self.assertEqual(self.gate.fault(3., 103.), 'GATE_ORIGIN_FENCE')
        self.gate.target.pose.position.x -= .81
        self.gate.health.pose = ((10., 20., 1.21), 0.)
        self.assertEqual(self.gate.fault(3., 103.), 'GATE_ORIGIN_FENCE')

    def test_research_freeze_diagnostic_is_still_sustained_hard_failure(self):
        for index in range(31, 50):
            for health in (self.health, self.gate.health):
                feed(health, index / 10, velocity=(.15, 0., 0.))
        self.assertEqual(self.health.reason(4.9, 104.9), 'POSE_VELOCITY_FREEZE_INCONSISTENCY')
        self.assertEqual(self.gate.health.reason(4.9, 104.9), 'POSE_VELOCITY_FREEZE_INCONSISTENCY')


@unittest.skipUnless(ROS, 'ROS service types required')
class ResearchParameterReader(unittest.TestCase):
    def setUp(self):
        self.manager = manager_module.RelativeMissionManager.__new__(manager_module.RelativeMissionManager)
        self.manager.config = research_config()
        self.manager.closed = threading.Event()
        self.manager.closed.wait = Mock(side_effect=lambda _timeout: self.manager.closed.set())
        self.manager.lock = threading.RLock()
        self.manager.snapshot = {}
        self.manager.record = Mock()
        self.manager.limit_time = None
        self.manager.limit_error = ''
        self.parameters = dict(WPNAV_SPEED=150, WPNAV_SPEED_UP=80, WPNAV_SPEED_DN=60,
                               WPNAV_ACCEL=300, WPNAV_ACCEL_Z=200, GUID_OPTIONS=0,
                               AHRS_EKF_TYPE=3, EK3_ENABLE=1, EK3_SRC1_POSXY=3)
        self.version = 4 << 24 | 5 << 16 | 7 << 8

    def run_reader(self):
        def get_parameter(name):
            return SimpleNamespace(success=name in self.parameters,
                                   value=SimpleNamespace(integer=self.parameters.get(name, 0), real=0.))
        vehicle = SimpleNamespace(available_info=2, autopilot=3, type=2,
                                  flight_sw_version=self.version, flight_custom_version=[])
        info = Mock(return_value=SimpleNamespace(success=True, vehicles=[vehicle]))
        def proxy(name, _service):
            return info if name.endswith('vehicle_info_get') else get_parameter
        self.manager.closed.clear()
        with patch.object(manager_module.rospy, 'is_shutdown', return_value=False), \
                patch.object(manager_module.rospy, 'wait_for_service'), \
                patch.object(manager_module.rospy, 'ServiceProxy', side_effect=proxy):
            self.manager.read_limits()

    def test_performance_recorded_deduplicated_and_missing_values_warn(self):
        self.run_reader()
        self.assertEqual(self.manager.limit_error, '')
        self.assertEqual(self.manager.snapshot['parameters'], self.parameters)
        self.assertEqual(len(self.manager.snapshot['performance_diagnostics']), 5)
        self.manager.record.assert_called_once()
        self.run_reader()
        self.manager.record.assert_called_once()
        self.parameters.pop('WPNAV_SPEED')
        self.run_reader()
        self.assertIsNone(self.manager.snapshot['parameters']['WPNAV_SPEED'])
        self.assertEqual(self.manager.limit_error, '')

    def test_missing_interface_unknown_firmware_and_strict_performance_still_block(self):
        self.parameters.pop('GUID_OPTIONS')
        self.run_reader()
        self.assertIn('GUID_OPTIONS', self.manager.limit_error)
        self.parameters['GUID_OPTIONS'] = 0
        self.version = 4 << 24 | 6 << 16
        self.run_reader()
        self.assertIn('unsupported firmware', self.manager.limit_error)
        self.version = 4 << 24 | 5 << 16 | 7 << 8
        self.manager.config['motion']['speed_enforcement'] = 'verify_fcu_limit'
        self.run_reader()
        self.assertIn('WPNAV_SPEED', self.manager.limit_error)
