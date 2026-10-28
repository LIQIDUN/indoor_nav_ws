import copy
import math
import unittest

import yaml

from test_v0 import ROOT, config, feed
from indoor_nav.v0 import ConfigError, ControlLease, Health, Mission, load_config, validate_config


class TimeoutConfiguration(unittest.TestCase):
    def raw(self):
        return yaml.safe_load((ROOT / 'src/indoor_nav/config/mission_v0.yaml').read_text())

    def test_defaults_and_actual_heartbeat_override(self):
        raw = self.raw()
        self.assertEqual(validate_config(raw)['msg_timeout']['state_timeout'], 3.)
        del raw['msg_timeout']
        self.assertEqual(set(validate_config(raw)['msg_timeout'].values()), {.5})

    def test_legacy_yaml_keeps_original_deadlines(self):
        path = ROOT / 'docs/v0/evidence/20261027T143709-multi/mission.yaml'
        cfg = load_config(path.read_text())
        self.assertEqual(cfg['msg_timeout']['pose_timeout'], .35)
        self.assertEqual(cfg['msg_timeout']['state_timeout'], 3.)
        self.assertEqual(cfg['msg_timeout']['command_timeout'], .5)
        self.assertEqual(validate_config(cfg), cfg)

    def test_conflicts_unknown_fields_and_invalid_values(self):
        for value in (True, None, '0.5', math.nan, math.inf, -.5, 0, 11):
            raw = self.raw()
            raw['msg_timeout']['pose_timeout'] = value
            with self.subTest(value=value), self.assertRaises(ConfigError):
                validate_config(raw)
        for changes in ({'state': .5}, {'command_timeout': 1.1}):
            raw = self.raw()
            raw['msg_timeout'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ConfigError):
                validate_config(raw)
        raw = self.raw()
        raw['safety']['pose_timeout_s'] = .35
        with self.assertRaises(ConfigError):
            validate_config(raw)


class ReceiptHealth(unittest.TestCase):
    def setUp(self):
        self.config = config()
        self.health = Health(self.config)
        for index in range(31):
            feed(self.health, index / 10)

    def test_header_rewind_offset_repeat_and_zero_are_diagnostics(self):
        for stamp, diagnostic in ((103.05, ''), (102.98, 'RAW_STAMP_REWIND'),
                                  (103., 'RAW_STAMP_REPEAT'), (104., 'RAW_STAMP_OFFSET'),
                                  (0., 'RAW_STAMP_INVALID')):
            health = copy.deepcopy(self.health)
            health.update_pose((10., 20., 1.), (0, 0, 0, 1), stamp, 3.1, 103.1, 'map')
            health.update_velocity((0, 0, 0), stamp, 3.1, 103.1, 'map')
            self.assertEqual(health.reason(3.1, 103.1), '')
            self.assertEqual(health.pose_stamp, stamp)
            self.assertEqual(health.rx_ros['pose'], 103.1)
            self.assertEqual(health.stamp_diagnostics['pose'], diagnostic)
        self.health.update_pose((10., 20., 1.), (0, 0, 0, 1), 102.98, 3.1, 103.1, 'map')
        self.assertEqual(self.health.stamp_diagnostics['pose'], 'RAW_STAMP_REWIND')
        self.assertTrue(Mission(self.config).start(3.1, 103.1, self.health)[0])

    def test_true_pose_loss_with_other_streams_alive(self):
        self.health.update_state(True, True, 'GUIDED', 103.6, 3.6, 103.6)
        self.health.update_velocity((0, 0, 0), 103.6, 3.6, 103.6, 'map')
        self.assertEqual(self.health.reason(3.6, 103.6), 'POSE_RX_TIMEOUT')

    def test_ros_receipt_age_is_checked_independently_of_header_and_steady_age(self):
        self.health.rx_ros['pose'] = 102.
        self.assertEqual(self.health.reason(3., 103.), 'POSE_RX_TIMEOUT')
        self.health.rx_ros['pose'] = 103.
        self.health.pose_time = 2.
        self.assertEqual(self.health.reason(3., 103.), 'POSE_RX_TIMEOUT')

    def test_disconnect_drops_old_samples_and_requires_fresh_reconnection(self):
        self.health.update_state(False, False, '', 103.1, 3.1, 103.1)
        self.assertEqual(self.health.reason(3.1, 103.1), 'DISCONNECTED')
        self.assertIsNone(self.health.pose)
        self.assertFalse(self.health.gps_ok)
        self.health.update_pose((10., 20., 1.), (0, 0, 0, 1), 103.2, 3.2, 103.2, 'map')
        self.assertIsNone(self.health.pose)
        self.health.update_state(True, True, 'GUIDED', 103.3, 3.3, 103.3)
        self.assertEqual(self.health.reason(3.3, 103.3), 'POSE_RX_TIMEOUT')
        feed(self.health, 3.4)
        self.assertEqual(self.health.reason(3.4, 103.4), '')
        self.assertFalse(self.health.stable_hover(3.4))

    def test_ros_zero_and_reinitialization_clear_buffers_and_abort_task(self):
        mission = Mission(self.config)
        self.assertTrue(mission.start(3., 103., self.health)[0])
        for ros_now in (0., 1.):
            health = copy.deepcopy(self.health)
            task = copy.deepcopy(mission)
            task.tick(3.1, ros_now, health)
            self.assertEqual(task.state, 'ABORT_LATCHED')
            self.assertIsNone(task.target)
            self.assertIsNone(health.pose)
            self.assertFalse(health.rx_ros)
            self.assertTrue(health.clock_reset_pending)
            health.clock_reset_pending = False
            feed(health, 3.2)
            task.tick(3.2, 103.2, health)
            self.assertEqual(task.state, 'ABORT_LATCHED')
            self.assertFalse(task.start(3.2, 103.2, health)[0])
        fresh = Health(self.config)
        fresh.update_state(True, True, 'GUIDED', 0., 1., 0.)
        self.assertEqual(fresh.reason(1., 0.), 'ROS_TIME_UNINITIALIZED')

    def test_duplicate_headers_do_not_hide_jumps_or_freezes(self):
        self.health.update_pose((11., 20., 1.), (0, 0, 0, 1), 103., 3.1, 103.1, 'map')
        self.assertEqual(self.health.fault, 'POSE_JUMP')
        self.setUp()
        for index in range(31, 52):
            now = index / 10
            feed(self.health, now, velocity=(.15, 0, 0))
            self.health.update_pose((10., 20., 1.), (0, 0, 0, 1), 103., now, 100 + now, 'map')
        self.assertEqual(self.health.reason(5.1, 105.1), 'POSE_VELOCITY_FREEZE_INCONSISTENCY')

    def test_continuing_callbacks_cannot_mask_ekf_loss(self):
        feed(self.health, 3.1)
        self.health.ekf_ok = False
        self.assertEqual(self.health.reason(3.1, 103.1), 'GPS_EKF_UNHEALTHY')

    def test_unknown_disarmed_state_cannot_reset_or_prepare_gate(self):
        lease = ControlLease()
        lease.step('IDLE', 'test', False, '', 'DISCONNECTED', False)
        self.assertFalse(lease.observed_idle)
        lease.step('EXEC_WAYPOINT', 'test', True, 'GUIDED', '', True)
        self.assertTrue(lease.revoked)
        lease.step('IDLE', 'test', False, '', 'STATE_RX_TIMEOUT', False)
        self.assertTrue(lease.revoked)
        lease.step('IDLE', 'test', False, 'STABILIZE', '', False)
        self.assertFalse(lease.revoked)

    def test_audio_business_does_not_participate_in_health(self):
        for name in ('relative_mission_manager.py', 'v0_nav_health.py'):
            source = (ROOT / 'src/indoor_nav/scripts' / name).read_text()
            for token in ('/motor_mic/', '/sound_sensor/', 'auto_bag_recorder', 'rosbag record'):
                self.assertNotIn(token, source)
        self.assertEqual(self.health.reason(3., 103.), '')


if __name__ == '__main__':
    unittest.main()
