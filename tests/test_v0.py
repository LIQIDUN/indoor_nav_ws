import copy
import math
from pathlib import Path
import sys
import struct
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/indoor_nav/src'))
from indoor_nav.v0 import (ConfigError, ControlLease, Health, Mission, decode_ekf_report, load_config, normalized_quaternion,
                           quaternion_yaw, relative_target, validate_config,
                           verify_fcu_limits, wrap_pi, yaw_quaternion)


def config():
    result = load_config((ROOT / 'src/indoor_nav/config/mission_v0.yaml').read_text())
    result['navigation']['source'] = 'fc_ekf_sitl_gps'
    return result


def feed(health, now, position=(10., 20., 1.), velocity=(0., 0., 0.), yaw=0.):
    health.state, health.state_time = (True, True, 'GUIDED'), now
    health.gps_ok = health.ekf_ok = True
    health.gps_time = health.ekf_time = now
    health.update_pose(position, yaw_quaternion(yaw), 100 + now, now, 100 + now, 'map')
    health.update_velocity(velocity, 100 + now, now, 100 + now, 'map')


class Coordinates(unittest.TestCase):
    def test_cardinal_yaws_and_z_up(self):
        waypoint = {'x': 1., 'y': .5, 'z': .2}
        for yaw, expected in ((0, (11, 19.5, 1.2)), (math.pi/2, (10.5, 21, 1.2)),
                              (-math.pi/2, (9.5, 19, 1.2)), (math.pi, (9, 20.5, 1.2))):
            with self.subTest(yaw=yaw):
                actual = relative_target((10., 20., 1.), yaw, waypoint)
                for result, target in zip(actual, expected):
                    self.assertAlmostEqual(result, target)

    def test_quaternion_normalization_and_wrap(self):
        for angle in (0., math.pi/2, -math.pi/2, math.pi, 3*math.pi, -5*math.pi):
            quat = tuple(7 * value for value in yaw_quaternion(angle))
            self.assertAlmostEqual(wrap_pi(quaternion_yaw(quat) - angle), 0.)
        self.assertAlmostEqual(wrap_pi(-math.pi + .01 - (math.pi - .01)), .02)

    def test_invalid_quaternion(self):
        for quat in ((0., 0., 0., 0.), (math.nan, 0, 0, 1), (0, 0, math.inf, 1), (0, 1)):
            with self.subTest(quat=quat), self.assertRaises(ValueError):
                normalized_quaternion(quat)

    def test_nonfinite_position(self):
        with self.assertRaises(ValueError):
            relative_target((math.nan, 0, 0), 0, {'x': 1, 'y': 0, 'z': 0})


class Schema(unittest.TestCase):
    def test_sample_and_defaults(self):
        sample = ROOT / 'docs/mission_v0_copter.yaml'
        if sample.exists():
            self.assertEqual(load_config(sample.read_text())['control']['setpoint_rate_hz'], 10.)
        self.assertEqual(config()['safety']['state_timeout_s'], 3.)

    def test_bad_schemas(self):
        mutations = [lambda cfg: cfg.update(schema_version=True),
                     lambda cfg: cfg.update(unknown=3),
                     lambda cfg: cfg.update(waypoints=[]),
                     lambda cfg: cfg['waypoints'].append(copy.deepcopy(cfg['waypoints'][0])),
                     lambda cfg: cfg['waypoints'][0].pop('hold_s'),
                     lambda cfg: cfg['waypoints'][0].update(hold_s=-1),
                     lambda cfg: cfg['waypoints'][0].update(x=math.inf),
                     lambda cfg: cfg['waypoints'][0].update(x=10000),
                     lambda cfg: cfg['waypoints'][0].update(x=.7, y=.7),
                     lambda cfg: cfg['waypoints'][0].update(z=.3),
                     lambda cfg: cfg['waypoints'][0].update(yaw=90),
                     lambda cfg: cfg['motion'].update(speed_xy_mps=.01),
                     lambda cfg: cfg['motion'].update(speed_xy_mps=math.nan),
                     lambda cfg: cfg['start_policy'].update(auto_arm=True),
                     lambda cfg: cfg['start_policy'].update(auto_takeoff='false'),
                     lambda cfg: cfg['safety'].update(prevent_auto_resume=False),
                     lambda cfg: cfg['control'].update(dry_run_default=False),
                     lambda cfg: cfg['control'].update(setpoint_rate_hz=100)]
        for index, mutate in enumerate(mutations):
            candidate = config()
            mutate(candidate)
            with self.subTest(index=index), self.assertRaises(ConfigError):
                validate_config(candidate)

    def test_duplicate_yaml_keys(self):
        with self.assertRaises(ConfigError):
            load_config('schema_version: 1\nschema_version: 1')

    def test_launch_is_navigation_only(self):
        tree = ET.parse(ROOT / 'src/indoor_nav/launch/indoor_nav_v0.launch')
        self.assertEqual(len(tree.findall('.//include')), 0)
        self.assertEqual({node.attrib['pkg'] for node in tree.findall('.//node')}, {'indoor_nav'})

    def test_no_production_unsafe_apis(self):
        for filename in ('relative_mission_manager.py', 'v0_nav_health.py'):
            text = (ROOT / 'src/indoor_nav/scripts' / filename).read_text()
            for forbidden in ('pymavlink', 'mavutil', 'serial.Serial', 'CommandBool', 'CommandTOL',
                              '/cmd/arming', '/cmd/takeoff', '/cmd/land', 'ParamSet', 'rosbag',
                              'world_lab', '/odometry/out'):
                self.assertNotIn(forbidden, text)
        manager = (ROOT / 'src/indoor_nav/scripts/relative_mission_manager.py').read_text()
        self.assertNotIn('SetMode', manager)
        self.assertNotIn('self.command_pub', manager)
        gate = (ROOT / 'src/indoor_nav/scripts/v0_nav_health.py').read_text()
        self.assertEqual(gate.count("SetMode)(0, 'BRAKE')"), 1)
        self.assertNotIn("SetMode)(0, 'GUIDED')", gate)


class FcuLimits(unittest.TestCase):
    def test_ekf_report_wire_layout_and_mavlink2_truncation(self):
        payload = struct.pack('<5fH', .1, .2, .3, .4, 0., 63)
        words = struct.unpack('<3Q', payload.ljust(24, b'\0'))
        self.assertTrue(decode_ekf_report(words, 21)['healthy'])
        self.assertEqual(decode_ekf_report(words, 22)['flags'], 63)
        for flags in (0, 47, 128+63, 1024+63, 32768+63):
            payload = struct.pack('<5fH', .1, .2, .3, .4, 0., flags)
            self.assertFalse(decode_ekf_report(struct.unpack('<3Q', payload.ljust(24, b'\0')), 22)['healthy'])
        with self.assertRaises(ValueError):
            decode_ekf_report([], 22)

    def setUp(self):
        self.parameters = dict(WPNAV_SPEED=20, WPNAV_SPEED_UP=20, WPNAV_SPEED_DN=20,
                               WPNAV_ACCEL=50, WPNAV_ACCEL_Z=50, GUID_OPTIONS=0,
                               AHRS_EKF_TYPE=3, EK3_ENABLE=1, EK3_SRC1_POSXY=3)

    def test_units(self):
        self.assertAlmostEqual(verify_fcu_limits((4, 5, 7), self.parameters, config()), .2)

    def test_unknown_versions_and_parameters(self):
        with self.assertRaises(ValueError):
            verify_fcu_limits((4, 6, 0), self.parameters, config())
        for key, value in (('WPNAV_SPEED', 21), ('WPNAV_SPEED', .2), ('WPNAV_SPEED', math.nan),
                           ('GUID_OPTIONS', 64), ('EK3_SRC1_POSXY', 6), ('WPNAV_ACCEL', 500)):
            parameters = dict(self.parameters, **{key: value})
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                verify_fcu_limits((4, 5, 7), parameters, config())
        self.parameters.pop('WPNAV_SPEED')
        with self.assertRaises(ValueError):
            verify_fcu_limits((4, 5, 7), self.parameters, config())


class NavigationHealth(unittest.TestCase):
    def setUp(self):
        self.health = Health(config())
        for index in range(31):
            feed(self.health, index / 10)

    def test_stationary_not_freeze(self):
        self.assertEqual(self.health.reason(3., 103.), '')
        self.assertTrue(self.health.stable_hover(3.))

    def test_ekf_origin_change_latches_even_without_pose_jump(self):
        self.health.update_ekf_origin((30., 110., 200.))
        self.health.update_ekf_origin((30., 110., 200.))
        self.assertEqual(self.health.fault, '')
        self.health.update_ekf_origin((30.00001, 110., 200.))
        self.assertEqual(self.health.reason(3., 103.), 'EKF_ORIGIN_CHANGED')

    def test_stale_pose_and_duplicate_stamp(self):
        self.health.update_pose((10., 20., 1.), (0, 0, 0, 1), 103., 3.3, 103.3, 'map')
        self.assertEqual(self.health.pose_time, 3.)
        self.assertEqual(self.health.reason(3.4, 103.4), 'POSE_STALE')

    def test_jump_nan_quaternion_and_frames(self):
        cases = [((11., 20., 1.), (0, 0, 0, 1), 103.1, 'map'),
                 ((10., 20., 1.), (0, 0, 0, 0), 103.1, 'map'),
                 ((math.nan, 20., 1.), (0, 0, 0, 1), 103.1, 'map'),
                 ((10., 20., 1.), (0, 0, 0, 1), 104., 'map'),
                 ((10., 20., 1.), (0, 0, 0, 1), 102.9, 'map'),
                 ((10., 20., 1.), (0, 0, 0, 1), 103.1, 'odom')]
        for position, quat, stamp, frame in cases:
            health = copy.deepcopy(self.health)
            health.update_pose(position, quat, stamp, 3.1, 103.1, frame)
            self.assertTrue(health.fault)

    def test_freeze_requires_sustained_velocity_not_single_spike(self):
        feed(self.health, 3.1, velocity=(.15, 0, 0))
        self.assertEqual(self.health.reason(3.1, 103.1), '')
        for index in range(32, 50):
            feed(self.health, index / 10, velocity=(.15, 0, 0))
        self.assertEqual(self.health.reason(4.9, 104.9), 'POSE_VELOCITY_FREEZE_INCONSISTENCY')

    def test_upstream_is_explicit_not_required_by_gps_source_name(self):
        self.health.source = 'fc_ekf_fake_gps'
        self.assertEqual(self.health.reason(3., 103.), '')
        self.health.require_upstream = True
        self.assertEqual(self.health.reason(3., 103.), 'UPSTREAM_UNVERIFIED_OR_STALE')
        self.health.upstream_time, self.health.upstream_ok = 3., True
        self.assertEqual(self.health.reason(3., 103.), '')


    def test_mavros_1201_velocity_local_enu_with_child_label(self):
        self.health.update_velocity((.05, .02, .01), 103.1, 3.1, 103.1, 'base_link')
        self.assertEqual(self.health.velocity, (.05, .02, .01))
        self.assertEqual(self.health.fault, '')
        self.health.update_velocity((0, 0, 0), 103.2, 3.2, 103.2, 'unknown')
        self.assertEqual(self.health.fault, 'VELOCITY_FRAME_OR_STAMP')


class Leases(unittest.TestCase):
    def setUp(self):
        self.lease = ControlLease()
        self.lease.step('IDLE', 'test', False, 'STABILIZE', '', False)
        self.assertEqual(self.lease.step('HOLD_INITIAL', 'test', True, 'GUIDED', '', True), (True, False))

    def test_healthy_program_loss_requests_one_guarded_exit(self):
        self.assertEqual(self.lease.step('EXEC_WAYPOINT', 'test', True, 'GUIDED', 'MISSION_HEARTBEAT_TIMEOUT', True), (False, True))
        self.assertEqual(self.lease.step('EXEC_WAYPOINT', 'test', True, 'GUIDED', '', True), (False, False))

    def test_position_or_link_loss_never_blindly_brakes(self):
        for reason in ('POSE_STALE', 'DISCONNECTED', 'GPS_EKF_UNHEALTHY'):
            with self.subTest(reason=reason):
                lease = ControlLease()
                lease.step('IDLE', 'test', False, 'STABILIZE', '', False)
                lease.step('HOLD_INITIAL', 'test', True, 'GUIDED', '', True)
                self.assertEqual(lease.step('EXEC_WAYPOINT', 'test', True, 'GUIDED', reason, False), (False, False))
                self.assertTrue(lease.revoked)

    def test_manual_mode_exit_is_never_overridden(self):
        self.assertEqual(self.lease.step('EXEC_WAYPOINT', 'test', True, 'ALT_HOLD', '', True), (False, False))
        self.assertEqual(self.lease.step('EXEC_WAYPOINT', 'test', True, 'GUIDED', '', True), (False, False))

    def test_restart_cannot_acquire_live_mission(self):
        lease = ControlLease()
        self.assertEqual(lease.step('EXEC_WAYPOINT', 'test', True, 'GUIDED', '', True), (False, False))
        self.assertEqual(lease.reason, 'GATE_RESTART_DURING_MISSION')

    def test_abort_stop_id_change_and_explicit_reset(self):
        for state, mission_id in (('ABORT_LATCHED', 'test'), ('STOPPED', 'test'), ('EXEC_WAYPOINT', 'other')):
            self.setUp()
            self.assertEqual(self.lease.step(state, mission_id, True, 'GUIDED', '', True), (False, True))
            self.lease.step('IDLE', 'test', True, 'GUIDED', '', True)
            self.assertTrue(self.lease.revoked)
            self.lease.step('IDLE', 'test', False, 'STABILIZE', '', False)
            self.assertFalse(self.lease.revoked)

class StateMachine(unittest.TestCase):
    def setUp(self):
        self.config = config()
        self.health = Health(self.config)
        for index in range(31):
            feed(self.health, index / 10)
        self.mission = Mission(self.config)
        self.assertTrue(self.mission.start(3., 103., self.health)[0])

    def advance(self, end, position=(10., 20., 1.), velocity=(0, 0, 0), yaw=0.):
        start = self.mission.last_tick
        count = round((end - start) * 10)
        for index in range(1, count + 1):
            now = round(start + index / 10, 6)
            feed(self.health, now, position, velocity, yaw)
            self.mission.tick(now, 100 + now, self.health)

    def test_double_start_and_latched_abort(self):
        self.assertFalse(self.mission.start(3.1, 103.1, self.health)[0])
        self.mission.abort(3.1, 'TEST')
        self.assertIsNone(self.mission.target)
        self.assertFalse(self.mission.start(3.2, 103.2, self.health)[0])
        self.mission.tick(3.3, 103.3, self.health)
        self.assertEqual(self.mission.state, 'ABORT_LATCHED')

    def test_mode_exit_and_no_resume(self):
        self.health.state = (True, True, 'ALT_HOLD')
        self.mission.tick(3.1, 103.1, self.health)
        self.assertEqual(self.mission.reason, 'MODE_LOST')
        self.advance(3.2)
        self.assertEqual(self.mission.state, 'ABORT_LATCHED')

    def test_disarm_disconnect_ekf_and_watchdog(self):
        for state, reason in (((True, False, 'GUIDED'), 'DISARMED'), ((False, True, 'GUIDED'), 'DISCONNECTED')):
            mission = copy.deepcopy(self.mission)
            self.health.state = state
            mission.tick(3.1, 103.1, self.health)
            self.assertEqual(mission.reason, reason)
        feed(self.health, 3.1)
        self.mission.tick(3.1, 103.1, self.health, 'OTHER_CONTROL_PUBLISHER')
        self.assertEqual(self.mission.state, 'ABORT_LATCHED')

    def test_fixed_origin_despite_yaw_change(self):
        self.advance(5.2)
        origin = self.mission.origin
        self.advance(5.3, yaw=math.pi/2)
        self.assertEqual(self.mission.origin, origin)
        self.assertEqual(self.mission.yaw0, 0)
        self.mission.index = 1
        self.mission.next_waypoint(5.3)
        self.assertEqual(self.mission.target, (10.3, 20., 1.))
        self.mission.index = 2
        self.mission.next_waypoint(5.3)
        self.assertEqual(self.mission.target, (10.3, 19.7, 1.))

    def test_arrival_continuous_stability_and_dwell_reset(self):
        self.advance(6.)
        self.assertEqual(self.mission.state, 'EXEC_WAYPOINT')
        self.advance(6.1, position=(10.13, 20, 1))
        self.assertIsNone(self.mission.stable_since)
        self.advance(7.5)
        self.assertEqual(self.mission.state, 'EXEC_WAYPOINT')
        self.advance(7.8)
        self.assertEqual(self.mission.state, 'DWELL')
        self.advance(7.9, position=(10.13, 20, 1))
        self.assertEqual(self.mission.state, 'EXEC_WAYPOINT')

    def test_timeout(self):
        self.mission.index = 1
        self.mission.next_waypoint(3.)
        self.advance(18.2)
        self.assertEqual(self.mission.reason, 'WAYPOINT_TIMEOUT')

    def test_pose_stale(self):
        self.mission.tick(3.4, 103.4, self.health)
        self.assertEqual(self.mission.reason, 'POSE_STALE')

    def test_fence_and_sustained_overspeed(self):
        self.advance(3.1, position=(10., 20., 1.21))
        self.assertEqual(self.mission.reason, 'ORIGIN_FENCE')
        self.setUp()
        self.advance(3.2, velocity=(.31, 0, 0))
        self.assertNotEqual(self.mission.state, 'ABORT_LATCHED')
        self.advance(4.1, velocity=(.31, 0, 0))
        self.assertEqual(self.mission.reason, 'SUSTAINED_OVERSPEED')

    def test_clock_and_cycle_faults(self):
        for now, ros_now, expected in ((3.1, 102.9, 'ROS_CLOCK_ANOMALY'),
                                        (3.1, 104., 'POSE_HEADER_STALE'),
                                        (3.6, 103.6, 'POSE_STALE')):
            mission = copy.deepcopy(self.mission)
            mission.tick(now, ros_now, self.health)
            self.assertEqual(mission.reason, expected)

    def test_normal_finish_does_not_stop_hold(self):
        self.config['waypoints'] = [dict(id='one', x=0, y=0, z=0, hold_s=.2)]
        self.advance(7.)
        self.assertEqual(self.mission.state, 'FINISH_HOLD')
        self.assertEqual(self.mission.target, self.mission.origin)
        self.advance(8.)
        self.assertEqual(self.mission.state, 'FINISH_HOLD')


if __name__ == '__main__':
    unittest.main()
