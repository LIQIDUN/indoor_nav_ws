"""ROS-independent, fail-closed relative mission contract and state machine."""
import copy
import math
import re
import struct
from collections import deque

import yaml


class ConfigError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ConfigError('duplicate YAML key: ' + str(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def fields(mapping, required, optional=()):
    if not isinstance(mapping, dict):
        raise ConfigError('expected mapping')
    missing = set(required) - set(mapping)
    unknown = set(mapping) - set(required) - set(optional)
    if missing or unknown:
        raise ConfigError('missing={} unknown={}'.format(sorted(missing), sorted(unknown)))


def bounded(mapping, key, lower, upper):
    value = mapping[key]
    if not finite_number(value) or not lower <= value <= upper:
        raise ConfigError('{} outside [{}, {}]'.format(key, lower, upper))


SAFETY_DEFAULTS = {
    'state_timeout_s': 3.0, 'velocity_timeout_s': 0.5,
    'gps_timeout_s': 3.0, 'ekf_timeout_s': 3.0,
    'future_tolerance_s': 0.1, 'jump_distance_m': 0.35,
    'overspeed_duration_s': 0.8, 'max_speed_z_mps': 0.20,
    'hover_time_s': 2.0, 'hover_speed_mps': 0.08,
    'hover_radius_m': 0.08, 'hover_yaw_rad': 0.15,
    'max_tilt_rad': 0.35, 'initial_hold_s': 2.0,
    'clock_tolerance_s': 0.5, 'cycle_timeout_s': 0.5,
    'freeze_window_s': 1.5, 'freeze_distance_m': 0.005,
    'freeze_speed_mps': 0.12, 'upstream_timeout_s': 1.0,
    'watchdog_timeout_s': 0.6,
}


def validate_config(config):
    config = copy.deepcopy(config)
    fields(config, ('schema_version', 'mission_id', 'navigation', 'reference',
                    'start_policy', 'control', 'motion', 'arrival', 'safety',
                    'waypoints', 'finish'))
    if type(config['schema_version']) is not int or config['schema_version'] != 1:
        raise ConfigError('unsupported schema_version')
    if not isinstance(config['mission_id'], str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,80}', config['mission_id']):
        raise ConfigError('invalid mission_id')
    expected = {
        'reference': {'origin': 'pose_at_mission_start', 'heading': 'yaw_at_mission_start', 'axes': 'forward_right_up'},
        'start_policy': {'require_operator_start': True, 'require_manual_takeoff': True,
                         'require_stable_hover': True, 'require_guided_mode': True,
                         'auto_arm': False, 'auto_takeoff': False, 'auto_land': False},
        'finish': {'action': 'hold_and_wait_pilot'},
    }
    for section, values in expected.items():
        fields(config[section], values)
        if any(type(config[section][key]) is not type(value) or config[section][key] != value
               for key, value in values.items()):
            raise ConfigError('unsupported policy: ' + section)
    fields(config['navigation'], ('vehicle', 'source'))
    if config['navigation']['vehicle'] != 'ArduCopter' or config['navigation']['source'] not in (
            'fc_ekf_fake_gps', 'fc_ekf_sitl_gps'):
        raise ConfigError('unsupported navigation source')
    fields(config['control'], ('dry_run_default', 'setpoint_rate_hz', 'yaw_policy'))
    if config['control']['dry_run_default'] is not True or config['control']['yaw_policy'] != 'hold_initial_yaw':
        raise ConfigError('V0 requires safe defaults and initial yaw')
    bounded(config['control'], 'setpoint_rate_hz', 5, 10)
    motion = config['motion']
    fields(motion, ('speed_xy_mps', 'max_speed_xy_mps', 'speed_enforcement'))
    bounded(motion, 'speed_xy_mps', 0.2, 0.5)
    bounded(motion, 'max_speed_xy_mps', motion['speed_xy_mps'], 0.6)
    if motion['speed_enforcement'] != 'verify_fcu_limit':
        raise ConfigError('unsupported speed enforcement')
    arrival = config['arrival']
    fields(arrival, ('xy_tolerance_m', 'z_tolerance_m', 'stable_time_s', 'max_waypoint_time_s'))
    for key, lower, upper in (('xy_tolerance_m', .01, .2), ('z_tolerance_m', .01, .15),
                              ('stable_time_s', .2, 10), ('max_waypoint_time_s', 1, 120)):
        bounded(arrival, key, lower, upper)
    safety = config['safety']
    required = ('max_offset_xy_m', 'max_offset_z_m', 'pose_timeout_s',
                'abort_on_mode_change', 'abort_on_localization_loss', 'prevent_auto_resume')
    fields(safety, required, SAFETY_DEFAULTS)
    for key in required[3:]:
        if safety[key] is not True:
            raise ConfigError('mandatory safety policy: ' + key)
    for key, value in SAFETY_DEFAULTS.items():
        safety.setdefault(key, value)
        bounded(safety, key, .001, 10)
    bounded(safety, 'max_offset_xy_m', .1, 2)
    bounded(safety, 'max_offset_z_m', .05, .5)
    bounded(safety, 'pose_timeout_s', .1, 1)
    for key in ('cycle_timeout_s', 'watchdog_timeout_s', 'velocity_timeout_s'):
        bounded(safety, key, .2, 1)
    if safety['jump_distance_m'] > .5 or safety['max_tilt_rad'] > .6:
        raise ConfigError('unsafe jump/tilt gate')
    if arrival['max_waypoint_time_s'] <= arrival['stable_time_s']:
        raise ConfigError('waypoint deadline must exceed stable time')
    waypoints = config['waypoints']
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 100:
        raise ConfigError('expected 1..100 waypoints')
    identifiers = set()
    for waypoint in waypoints:
        fields(waypoint, ('id', 'x', 'y', 'z', 'hold_s'))
        identifier = waypoint['id']
        if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,80}', identifier) or identifier in identifiers:
            raise ConfigError('invalid or duplicate waypoint id')
        identifiers.add(identifier)
        for key in ('x', 'y', 'z'):
            bounded(waypoint, key, -2, 2)
        bounded(waypoint, 'hold_s', 0, 60)
        if math.hypot(waypoint['x'], waypoint['y']) > safety['max_offset_xy_m'] or abs(waypoint['z']) > safety['max_offset_z_m']:
            raise ConfigError('waypoint outside origin fence')
    return config


def load_config(text):
    return validate_config(yaml.load(text, Loader=UniqueLoader))


def wrap_pi(angle):
    if not finite_number(angle):
        raise ValueError('nonfinite angle')
    return math.atan2(math.sin(angle), math.cos(angle))


def normalized_quaternion(quaternion):
    if len(quaternion) != 4 or not all(finite_number(value) for value in quaternion):
        raise ValueError('invalid quaternion')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('zero/invalid quaternion norm')
    return tuple(value / norm for value in quaternion)


def quaternion_yaw(quaternion):
    quat_x, quat_y, quat_z, quat_w = normalized_quaternion(quaternion)
    return math.atan2(2 * (quat_w * quat_z + quat_x * quat_y),
                      1 - 2 * (quat_y * quat_y + quat_z * quat_z))


def yaw_quaternion(yaw):
    yaw = wrap_pi(yaw)
    return (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))


def relative_target(origin, yaw, waypoint):
    if not all(finite_number(value) for value in (*origin, yaw, waypoint['x'], waypoint['y'], waypoint['z'])):
        raise ValueError('nonfinite target')
    forward, right, upward = (waypoint[key] for key in ('x', 'y', 'z'))
    return (origin[0] + forward * math.cos(yaw) + right * math.sin(yaw),
            origin[1] + forward * math.sin(yaw) - right * math.cos(yaw),
            origin[2] + upward)


def verify_fcu_limits(version, parameters, config):
    if tuple(version) != (4, 5, 7):
        raise ValueError('unsupported firmware/units; only audited Copter 4.5.7')
    bounds = {'WPNAV_SPEED': (20, config['motion']['speed_xy_mps'] * 100),
              'WPNAV_SPEED_UP': (10, config['safety']['max_speed_z_mps'] * 100),
              'WPNAV_SPEED_DN': (10, config['safety']['max_speed_z_mps'] * 100),
              'WPNAV_ACCEL': (50, 100), 'WPNAV_ACCEL_Z': (50, 100),
              'GUID_OPTIONS': (0, 0), 'AHRS_EKF_TYPE': (3, 3),
              'EK3_ENABLE': (1, 1), 'EK3_SRC1_POSXY': (3, 3)}
    for key, (lower, upper) in bounds.items():
        value = parameters.get(key)
        if not finite_number(value) or not lower <= value <= upper + 1e-5:
            raise ValueError('FCU limit/source mismatch: {}={}'.format(key, value))
    return parameters['WPNAV_SPEED'] / 100.


def decode_ekf_report(payload64, length):
    if not 20 <= length <= 26 or len(payload64) * 8 < length:
        raise ValueError('invalid EKF_STATUS_REPORT payload size')
    payload = struct.pack('<{}Q'.format(len(payload64)), *payload64)[:length].ljust(26, b'\0')
    velocity, horizontal, vertical, compass, terrain, flags = struct.unpack_from('<5fH', payload)
    ratios = (velocity, horizontal, vertical, compass, terrain)
    healthy = (flags & 0x37 == 0x37 and not flags & (128 | 1024 | 32768) and
               all(math.isfinite(value) and 0 <= value <= 1 for value in ratios))
    return {'flags': flags, 'ratios': ratios, 'healthy': healthy}


class Health:
    def __init__(self, config):
        self.cfg = config['safety']
        self.source = config['navigation']['source']
        self.pose = None
        self.pose_time = None
        self.pose_stamp = None
        self.velocity = None
        self.velocity_time = None
        self.state = (False, False, '')
        self.state_time = None
        self.gps_time = None
        self.gps_ok = False
        self.ekf_time = None
        self.ekf_ok = False
        self.upstream_time = None
        self.upstream_ok = False
        self.history = deque()
        self.velocity_history = deque()
        self.fault = ''
        self.ekf_origin = None

    def update_ekf_origin(self, origin):
        if len(origin) != 3 or not all(finite_number(value) for value in origin):
            self.fault = 'INVALID_EKF_ORIGIN'
            return
        if self.ekf_origin is not None and any(abs(actual - previous) > tolerance for actual, previous, tolerance in
                                                zip(origin, self.ekf_origin, (1e-8, 1e-8, .001))):
            self.fault = 'EKF_ORIGIN_CHANGED'
        self.ekf_origin = tuple(origin)

    def update_pose(self, position, quaternion, stamp, now, ros_now, frame):
        try:
            normalized = normalized_quaternion(quaternion)
            if len(position) != 3 or not all(finite_number(value) for value in (*position, stamp, now, ros_now)):
                raise ValueError('NONFINITE_POSE')
            if frame != 'map' or stamp <= 0 or ros_now - stamp > self.cfg['pose_timeout_s'] or stamp - ros_now > self.cfg['future_tolerance_s']:
                raise ValueError('POSE_FRAME_OR_STAMP')
            if self.pose_stamp is not None:
                if stamp < self.pose_stamp:
                    raise ValueError('POSE_STAMP_REWIND')
                if stamp == self.pose_stamp:
                    return
                distance = math.dist(position, self.pose[0])
                if distance > self.cfg['jump_distance_m']:
                    raise ValueError('POSE_JUMP')
            self.pose = (tuple(position), quaternion_yaw(normalized))
            self.pose_time, self.pose_stamp = now, stamp
            self.history.append((now, tuple(position), self.pose[1]))
            duration = max(self.cfg['hover_time_s'], self.cfg['freeze_window_s'])
            while len(self.history) > 1 and now - self.history[1][0] > duration:
                self.history.popleft()
            if abs(normalized[0]) ** 2 + abs(normalized[1]) ** 2 > math.sin(self.cfg['max_tilt_rad'] / 2) ** 2:
                raise ValueError('EXCESSIVE_TILT')
        except (ValueError, TypeError, OverflowError) as error:
            self.fault = str(error)

    def update_velocity(self, velocity, stamp, now, ros_now, frame):
        if frame not in ('map', 'base_link') or not all(finite_number(value) for value in (*velocity, stamp)) or stamp <= 0 or not -self.cfg['future_tolerance_s'] <= ros_now - stamp <= self.cfg['velocity_timeout_s']:
            self.fault = 'VELOCITY_FRAME_OR_STAMP'
            return
        self.velocity, self.velocity_time = tuple(velocity), now
        self.velocity_history.append((now, tuple(velocity)))
        duration = max(self.cfg['hover_time_s'], self.cfg['freeze_window_s'])
        while len(self.velocity_history) > 1 and now - self.velocity_history[1][0] > duration:
            self.velocity_history.popleft()

    def reason(self, now, ros_now):
        if self.fault:
            return self.fault
        connected, armed, mode = self.state
        for stamp, timeout, reason in (
                (self.state_time, self.cfg['state_timeout_s'], 'STATE_STALE'),
                (self.pose_time, self.cfg['pose_timeout_s'], 'POSE_STALE'),
                (self.velocity_time, self.cfg['velocity_timeout_s'], 'VELOCITY_STALE'),
                (self.gps_time, self.cfg['gps_timeout_s'], 'GPS_STALE'),
                (self.ekf_time, self.cfg['ekf_timeout_s'], 'EKF_STALE')):
            if stamp is None or now - stamp > timeout:
                return reason
        if not connected:
            return 'DISCONNECTED'
        if not armed:
            return 'DISARMED'
        if mode != 'GUIDED':
            return 'MODE_LOST'
        if not -self.cfg['future_tolerance_s'] <= ros_now - self.pose_stamp <= self.cfg['pose_timeout_s']:
            return 'POSE_HEADER_STALE'
        if not self.gps_ok or not self.ekf_ok:
            return 'GPS_EKF_UNHEALTHY'
        if self.source == 'fc_ekf_fake_gps' and (not self.upstream_ok or self.upstream_time is None or now - self.upstream_time > self.cfg['upstream_timeout_s']):
            return 'UPSTREAM_UNVERIFIED_OR_STALE'
        if self.history and now - self.history[0][0] >= self.cfg['freeze_window_s']:
            window = [sample for sample in self.history if now - sample[0] <= self.cfg['freeze_window_s'] + .1]
            velocities = [sample for sample in self.velocity_history if now - sample[0] <= self.cfg['freeze_window_s'] + .1]
            if window and velocities and now - velocities[0][0] >= self.cfg['freeze_window_s'] - .1 and all(
                    math.hypot(*sample[1][:2]) > self.cfg['freeze_speed_mps'] for sample in velocities):
                if max(math.dist(sample[1], self.pose[0]) for sample in window) < self.cfg['freeze_distance_m']:
                    return 'POSE_VELOCITY_FREEZE_INCONSISTENCY'
        return ''

    def stable_hover(self, now):
        if not self.history or now - self.history[0][0] < self.cfg['hover_time_s'] or self.velocity is None:
            return False
        recent = [sample for sample in self.history if now - sample[0] <= self.cfg['hover_time_s'] + .1]
        velocities = [sample for sample in self.velocity_history if now - sample[0] <= self.cfg['hover_time_s'] + .1]
        return (velocities and now - velocities[0][0] >= self.cfg['hover_time_s'] - .1 and
                all(math.sqrt(sum(value * value for value in sample[1])) <= self.cfg['hover_speed_mps'] for sample in velocities) and
                all(math.dist(sample[1], self.pose[0]) <= self.cfg['hover_radius_m'] and
                    abs(wrap_pi(sample[2] - self.pose[1])) <= self.cfg['hover_yaw_rad'] for sample in recent))


class Mission:
    ACTIVE = ('HOLD_INITIAL', 'EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD')

    def __init__(self, config):
        self.config = config
        self.state = 'IDLE'
        self.reason = ''
        self.origin = None
        self.yaw0 = None
        self.target = None
        self.index = 0
        self.events = []
        self.last_tick = None
        self.last_ros = None
        self.phase_time = None
        self.waypoint_time = None
        self.stable_since = None
        self.overspeed_since = None

    def transition(self, state, now, reason=''):
        self.state, self.reason, self.phase_time = state, reason, now
        self.events.append({'state': state, 'wp': self.index, 'reason': reason, 'mono': now})

    def start(self, now, ros_now, health, external_reason=''):
        if self.state != 'IDLE':
            return False, 'explicit reset/re-authorization required'
        self.transition('WAIT_FOR_READY', now)
        reason = external_reason or health.reason(now, ros_now)
        if reason or not health.stable_hover(now):
            self.transition('IDLE', now, reason or 'HOVER_NOT_STABLE')
            return False, self.reason
        self.transition('CAPTURE_ORIGIN', now)
        self.origin, self.yaw0 = tuple(health.pose[0]), health.pose[1]
        self.target = self.origin
        self.last_tick, self.last_ros = now, ros_now
        self.transition('HOLD_INITIAL', now)
        return True, 'origin frozen; initial hold; no mode request'

    def abort(self, now, reason):
        if self.state != 'ABORT_LATCHED':
            self.transition('ABORT_LATCHED', now, reason)
        self.target = None

    def next_waypoint(self, now):
        self.target = relative_target(self.origin, self.yaw0, self.config['waypoints'][self.index])
        self.waypoint_time, self.stable_since = now, None
        self.transition('EXEC_WAYPOINT', now)

    def tick(self, now, ros_now, health, external_reason=''):
        if self.state not in self.ACTIVE:
            return
        safety = self.config['safety']
        delta = now - self.last_tick
        clock_delta = ros_now - self.last_ros
        self.last_tick, self.last_ros = now, ros_now
        reason = external_reason or health.reason(now, ros_now)
        if delta < 0 or delta > safety['cycle_timeout_s']:
            reason = reason or 'CONTROL_CYCLE_TIMEOUT'
        if clock_delta < -1e-6 or abs(clock_delta - delta) > safety['clock_tolerance_s']:
            reason = reason or 'ROS_CLOCK_ANOMALY'
        if health.pose and self.origin:
            position = health.pose[0]
            if math.hypot(position[0] - self.origin[0], position[1] - self.origin[1]) > safety['max_offset_xy_m'] or abs(position[2] - self.origin[2]) > safety['max_offset_z_m']:
                reason = reason or 'ORIGIN_FENCE'
        if health.velocity:
            overspeed = (math.hypot(*health.velocity[:2]) > self.config['motion']['max_speed_xy_mps'] or
                         abs(health.velocity[2]) > safety['max_speed_z_mps'])
            self.overspeed_since = (now if self.overspeed_since is None else self.overspeed_since) if overspeed else None
            if self.overspeed_since is not None and now - self.overspeed_since >= safety['overspeed_duration_s']:
                reason = reason or 'SUSTAINED_OVERSPEED'
        if reason:
            self.abort(now, reason)
            return
        if self.state == 'HOLD_INITIAL':
            if now - self.phase_time >= safety['initial_hold_s']:
                self.next_waypoint(now)
            return
        if self.state == 'FINISH_HOLD':
            return
        arrival = self.config['arrival']
        if now - self.waypoint_time > arrival['max_waypoint_time_s'] + (self.config['waypoints'][self.index]['hold_s'] if self.state == 'DWELL' else 0):
            self.abort(now, 'WAYPOINT_TIMEOUT')
            return
        position = health.pose[0]
        inside = (math.hypot(position[0] - self.target[0], position[1] - self.target[1]) <= arrival['xy_tolerance_m'] and
                  abs(position[2] - self.target[2]) <= arrival['z_tolerance_m'])
        if not inside:
            self.stable_since = None
            if self.state == 'DWELL':
                self.transition('EXEC_WAYPOINT', now, 'DWELL_INTERRUPTED')
            return
        if self.state == 'EXEC_WAYPOINT':
            if self.stable_since is None:
                self.stable_since = now
            if now - self.stable_since >= arrival['stable_time_s']:
                self.transition('DWELL', now, 'ARRIVED_STABLE')
        if self.state == 'DWELL' and now - self.phase_time >= self.config['waypoints'][self.index]['hold_s']:
            self.events.append({'state': 'DWELL_COMPLETE', 'wp': self.index, 'reason': '', 'mono': now})
            self.index += 1
            if self.index == len(self.config['waypoints']):
                self.transition('FINISH_HOLD', now)
            else:
                self.next_waypoint(now)
