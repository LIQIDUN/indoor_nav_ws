import copy
import math
import unittest

from test_v0 import ROOT, config, feed
from indoor_nav.v0 import (ConfigError, Health, Mission, load_config, performance_diagnostics,
                           relative_target, validate_config, verify_fcu_limits)


def research_config():
    return load_config((ROOT / 'src/indoor_nav/config/mission_v0_research.yaml').read_text())


class ResearchMission(unittest.TestCase):
    def setUp(self):
        self.config = research_config()
        self.health = Health(self.config)
        feed(self.health, 3.)
        self.mission = Mission(self.config)

    def advance(self, end, position=(10., 20., 1.), velocity=(0., 0., 0.), yaw=0.):
        while self.mission.last_tick < end - 1e-8:
            now = min(end, self.mission.last_tick + .1)
            feed(self.health, now, position=position, velocity=velocity, yaw=yaw)
            self.mission.tick(now, 100 + now, self.health)

    def test_research_starts_without_hover_history_and_freezes_reference(self):
        self.assertFalse(self.health.stable_hover(3.))
        self.assertTrue(self.mission.start(3., 103., self.health)[0])
        self.assertEqual(self.mission.state, 'HOLD_INITIAL')
        self.assertEqual(self.mission.origin, (10., 20., 1.))
        self.advance(5.2, yaw=math.pi / 2)
        self.assertFalse(self.mission.start(5.2, 105.2, self.health)[0])
        self.assertEqual(self.mission.yaw0, 0.)
        self.assertEqual(self.mission.origin, (10., 20., 1.))
        self.mission.index = 2
        self.mission.next_waypoint(5.2)
        self.assertEqual(self.mission.target, (10.3, 20.3, 1.))

    def test_strict_start_still_requires_hover(self):
        self.assertEqual(Mission(config()).start(3., 103., self.health), (False, 'HOVER_NOT_STABLE'))

    def test_start_hard_interlocks(self):
        cases = [('DISARMED', (True, False, 'GUIDED')),
                 ('MODE_LOST', (True, True, 'ALT_HOLD')),
                 ('DISCONNECTED', (False, True, 'GUIDED'))]
        for reason, state in cases:
            with self.subTest(reason=reason):
                health = copy.deepcopy(self.health)
                health.state = state
                self.assertEqual(Mission(self.config).start(3., 103., health), (False, reason))
        health = Health(self.config)
        health.update_state(True, True, 'GUIDED', 103., 3., 103.)
        self.assertEqual(Mission(self.config).start(3., 103., health), (False, 'POSE_RX_TIMEOUT'))
        self.assertEqual(self.mission.start(3., 103., self.health, 'FCU_LIMITS_NOT_VERIFIED'),
                         (False, 'FCU_LIMITS_NOT_VERIFIED'))

    def test_overspeed_warns_once_without_transition_and_records_peak(self):
        self.mission.start(3., 103., self.health)
        self.advance(4.2, velocity=(.4, 0., 0.))
        self.advance(4.8, velocity=(0., 0., 0.))
        self.advance(5.8, velocity=(.4, 0., 0.))
        warnings = [event for event in self.mission.events if event.get('severity') == 'WARNING']
        self.assertEqual([event['reason'] for event in warnings], ['SUSTAINED_OVERSPEED'])
        self.assertNotEqual(self.mission.state, 'ABORT_LATCHED')
        self.assertEqual(self.health.speed_peaks['xy_mps'], .4)
        self.assertEqual(self.health.current_speed()['xy_mps'], .4)

    def test_timeout_waits_once_then_can_arrive_finish_and_abort(self):
        self.config['waypoints'] = [dict(id='one', x=.3, y=0., z=0., hold_s=.2)]
        self.mission.start(3., 103., self.health)
        self.mission.next_waypoint(3.)
        target = self.mission.target
        self.advance(25.)
        self.assertEqual(self.mission.state, 'EXEC_WAYPOINT')
        self.assertEqual(self.mission.target, target)
        self.assertEqual(self.mission.index, 0)
        self.assertEqual(len(self.mission.warnings), 1)
        self.advance(25.1, position=(10.2, 20., 1.))
        self.advance(27., position=target)
        self.assertEqual(self.mission.state, 'FINISH_HOLD')
        self.mission.abort(27., 'OPERATOR_ABORT')
        self.assertIsNone(self.mission.target)
        self.assertFalse(self.mission.start(27., 127., self.health)[0])
        self.mission.tick(27.1, 127.1, self.health)
        self.assertEqual(self.mission.state, 'ABORT_LATCHED')

    def test_research_retains_position_fence_timeouts_and_origin_faults(self):
        for reason, position in (('ORIGIN_FENCE', (10., 20., 1.21)),
                                 ('POSE_JUMP', (11., 20., 1.))):
            with self.subTest(reason=reason):
                self.setUp()
                self.mission.start(3., 103., self.health)
                self.advance(3.1, position=position)
                self.assertEqual(self.mission.reason, reason)
        self.setUp()
        self.mission.start(3., 103., self.health)
        self.mission.tick(3.6, 103.6, self.health)
        self.assertEqual(self.mission.reason, 'POSE_RX_TIMEOUT')
        self.setUp()
        self.health.update_ekf_origin((1., 2., 3.))
        self.mission.start(3., 103., self.health)
        self.health.update_ekf_origin((1.001, 2., 3.))
        self.advance(3.1)
        self.assertEqual(self.mission.reason, 'EKF_ORIGIN_CHANGED')

    def test_configuration_strict_defaults_and_rejects_unknown_policies(self):
        strict = config()
        self.assertEqual(strict['motion']['overspeed_action'], 'abort')
        self.assertEqual(strict['arrival']['timeout_action'], 'abort')
        for section, key, value in (('motion', 'overspeed_action', 'ignore'),
                                    ('arrival', 'timeout_action', 'skip'),
                                    ('motion', 'speed_enforcement', 'disable'),
                                    ('start_policy', 'require_stable_hover', 0),
                                    ('start_policy', 'auto_arm', True)):
            candidate = copy.deepcopy(self.config)
            candidate[section][key] = value
            with self.subTest(key=key), self.assertRaises(ConfigError):
                validate_config(candidate)

    def test_level_and_research_have_explicit_left_square_not_strict_right_square(self):
        level = load_config((ROOT / 'src/indoor_nav/config/mission_v0_level.yaml').read_text())
        self.assertEqual(level['waypoints'], self.config['waypoints'])
        self.assertTrue(level['start_policy']['require_stable_hover'])
        self.assertGreater(config()['waypoints'][2]['y'], 0)
        self.assertLess(level['waypoints'][2]['y'], 0)
        for yaw in (0., math.pi / 2, math.pi, -math.pi / 2):
            for waypoint in level['waypoints']:
                self.assertEqual(relative_target((0., 0., 1.), yaw, waypoint)[2], 1.)


class TiltProtection(unittest.TestCase):
    def check(self, roll, pitch, yaw, excessive, scale=1.):
        cosine_roll, sine_roll = math.cos(roll / 2), math.sin(roll / 2)
        cosine_pitch, sine_pitch = math.cos(pitch / 2), math.sin(pitch / 2)
        cosine_yaw, sine_yaw = math.cos(yaw / 2), math.sin(yaw / 2)
        quaternion = (sine_roll * cosine_pitch * cosine_yaw - cosine_roll * sine_pitch * sine_yaw,
                      cosine_roll * sine_pitch * cosine_yaw + sine_roll * cosine_pitch * sine_yaw,
                      cosine_roll * cosine_pitch * sine_yaw - sine_roll * sine_pitch * cosine_yaw,
                      cosine_roll * cosine_pitch * cosine_yaw + sine_roll * sine_pitch * sine_yaw)
        health = Health(research_config())
        feed(health, 3.)
        health.update_pose((10., 20., 1.), tuple(value * scale for value in quaternion),
                           103.1, 3.1, 103.1, 'map')
        self.assertEqual(health.fault, 'EXCESSIVE_TILT' if excessive else '')

    def test_yaw_roll_pitch_combination_and_normalization(self):
        for yaw in (0., math.pi / 2, math.pi, -math.pi / 2):
            with self.subTest(yaw=yaw):
                self.check(0., 0., yaw, False)
                self.check(.36, 0., yaw, True)
                self.check(0., .36, yaw, True)
                self.check(.26, .26, yaw, True)
                self.check(.2, .2, yaw, False, scale=1.00001)
                self.check(.36, 0., yaw, True, scale=.99999)

    def test_invalid_quaternions_rejected(self):
        for quaternion in ((0., 0., 0., 0.), (math.nan, 0., 0., 1.),
                           (0., math.inf, 0., 1.), (0., 0., 1.)):
            health = Health(research_config())
            feed(health, 3.)
            health.update_pose((10., 20., 1.), quaternion, 103.1, 3.1, 103.1, 'map')
            self.assertTrue(health.fault)

    def test_exact_limit_normalization_roundoff_not_false_positive(self):
        for yaw in (0., math.pi / 2, math.pi, -math.pi / 2):
            for scale in (.99999, 1., 1.00001):
                with self.subTest(yaw=yaw, scale=scale):
                    self.check(.35, 0., yaw, False, scale)
                    self.check(0., .35, yaw, False, scale)
                    self.check(.350001, 0., yaw, True, scale)
                    self.check(0., .350001, yaw, True, scale)


class ResearchParameters(unittest.TestCase):
    def test_performance_only_is_diagnostic_unknown_interface_still_blocks(self):
        parameters = dict(WPNAV_SPEED=150, WPNAV_SPEED_UP=80, WPNAV_SPEED_DN=60,
                          WPNAV_ACCEL=300, WPNAV_ACCEL_Z=200, GUID_OPTIONS=0,
                          AHRS_EKF_TYPE=3, EK3_ENABLE=1, EK3_SRC1_POSXY=3)
        configuration = research_config()
        self.assertEqual(verify_fcu_limits((4, 5, 7), parameters, configuration), 1.5)
        self.assertEqual(len(performance_diagnostics(parameters, configuration)), 5)
        for key in ('GUID_OPTIONS', 'AHRS_EKF_TYPE', 'EK3_ENABLE', 'EK3_SRC1_POSXY'):
            with self.subTest(key=key), self.assertRaises(ValueError):
                verify_fcu_limits((4, 5, 7), dict(parameters, **{key: 99}), configuration)
        with self.assertRaises(ValueError):
            verify_fcu_limits((4, 6, 0), parameters, configuration)
        parameters.pop('WPNAV_SPEED')
        self.assertIsNone(verify_fcu_limits((4, 5, 7), parameters, configuration))
        self.assertTrue(performance_diagnostics(parameters, configuration))
