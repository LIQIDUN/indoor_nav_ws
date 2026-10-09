#!/usr/bin/env python3
"""Separate safety gate for mission setpoints and EKF/FC health.

Not a certified failsafe. FC's independent EKF/RC/battery failsafes remain
required, and the human must be able to switch ALT_HOLD/STABILIZE directly.
"""
import math
import json
import threading
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from indoor_nav.navigation_velocity import paired_odometry_velocity
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet, SetMode
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, TriggerResponse
from indoor_nav.msg import LocalizationStatus, SafetyStatus
from indoor_nav.core import Transform2D, inside_box, finite, fence_reason, FreezeWindow


class SafetySupervisor:
    def __init__(self):
        rospy.init_node('safety_supervisor')
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('Safety watchdog requires /use_sim_time=false; /clock is monitored separately')
        self.cfg = rospy.get_param('/indoor_nav/safety', {})
        self.params_cfg = rospy.get_param('/indoor_nav/ardupilot_expected', {})
        self.box = self.cfg.get('geofence', {})
        frame = rospy.get_param('/indoor_nav/localization/world/frame_id', 'world_lab')
        if self.cfg.get('profile') != self.cfg.get('expected_profile') or self.cfg.get('frame_id') != frame:
            raise ValueError('Safety profile/frame mismatch: ' + repr(self.cfg))
        for axis in ('x', 'y', 'z'):
            lo, hi = self.box[axis+'_min'], self.box[axis+'_max']
            if not finite(lo, hi) or lo >= hi:
                raise ValueError('Invalid geofence bounds')
        rospy.logwarn('Effective safety config: %s', json.dumps(self.cfg, sort_keys=True))
        self.diagnostics = rospy.Publisher('/indoor_nav/safety_diagnostics', String, queue_size=1, latch=True)
        self.fc = State()
        self.fc_timestamp = 0.
        self.world = None
        self.world_timestamp = 0.
        self.localization = None
        self.localization_timestamp = 0.
        self.goal = None
        self.goal_timestamp = 0.
        self.transform = None
        self.z_offset = None
        self.mission_state = 'IDLE'
        self.mission_start_stamp = None
        self.abort_latched = False
        self.abort_reason = ''
        self.operator_ack = False
        self.origin_verified = bool(self.cfg.get('operator_verified_fixed_ekf_origin', False))
        self.check_fc_params = bool(self.cfg.get('check_fc_params', True))
        self.param_good = False
        self.param_reason = 'FC_PARAM_CHECK_PENDING'
        self.was_guided_flying = False
        self.last_land_attempt = 0.
        self.velocity_frame = rospy.get_param('/mavros/local_position/frame_id', 'map')
        self.velocity_body_frame = rospy.get_param(
            '/mavros/local_position/tf/child_frame_id', 'base_link')
        self.velocity_lock = threading.RLock()
        self.velocity_pending = None
        self.velocity_future_hold_limit_s = .020
        self.velocity_source_error = None
        self.velocity = None
        self.velocity_stamp = 0.
        f = self.cfg.get('freeze_detection', {})
        self.freeze_detector = FreezeWindow(f.get('window_s', 1.),
            f.get('movement_epsilon_m', .015), f.get('min_speed_mps', .20))
        self.accepted_uwb = None
        self.truth = None
        self.truth_error = 'GAZEBO_TRUTH_MISSING'
        self.gazebo_clock_rewound = False
        self.clock = None
        self.clock_progress_receipt = None
        self.freeze_snapshot = {}
        self.abort_snapshot = None
        self.last_freeze_classification = None
        self.abort_diagnostics = rospy.Publisher('/indoor_nav/abort_snapshot', String, queue_size=1, latch=True)
        rospy.Subscriber('/indoor_nav/accepted_uwb', PointStamped, self.accepted_uwb_cb, queue_size=30)
        if self.cfg.get('profile') == 'sitl':
            from gazebo_msgs.msg import ModelStates
            from rosgraph_msgs.msg import Clock
            self.model_name = rospy.get_param('/indoor_nav/mock/ground_truth/model_name', 'iris')
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_cb, queue_size=1)
            rospy.Subscriber('/clock', Clock, self.clock_cb, queue_size=1)
        self.last_pub_reason = ''
        self.publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=3)
        self.status = rospy.Publisher('/indoor_nav/safety_status', SafetyStatus,
                                      queue_size=3, latch=True)
        rospy.Subscriber('/mavros/state', State, self.fc_cb, queue_size=5)
        rospy.Subscriber('/indoor_nav/localization_status', LocalizationStatus,
                         self.loc_cb, queue_size=5)
        rospy.Subscriber('/indoor_nav/world_pose', PoseStamped, self.world_cb, queue_size=5)
        rospy.Subscriber('/indoor_nav/desired_setpoint', PoseStamped, self.goal_cb, queue_size=5)
        rospy.Subscriber('/indoor_nav/nav_from_world', TransformStamped,
                         self.tf_cb, queue_size=1)
        rospy.Subscriber('/indoor_nav/z_offset_m', Float64, self.z_cb, queue_size=1)
        rospy.Subscriber('/indoor_nav/mission_state', String,
                         self.mission_state_cb, queue_size=5)
        rospy.Subscriber('/mavros/local_position/odom', Odometry,
                         self.odometry_velocity_cb, queue_size=5)
        rospy.Service('/indoor_nav/safety/acknowledge', Trigger, self.ack)
        rospy.Service('/indoor_nav/safety/reset', Trigger, self.reset)
        rospy.Timer(rospy.Duration(.05), self.tick)
        rospy.Timer(rospy.Duration(3.), self.validate_params)

    def mission_state_cb(self, msg):
        if msg.data == 'RUNNING' and self.mission_state != 'RUNNING':
            self.mission_start_stamp = rospy.get_time()
        self.mission_state = msg.data

    def fc_cb(self, msg):
        if msg.connected != self.fc.connected:
            rospy.logwarn('FC connection transition: connected=%s receipt_s=%.6f armed=%s mode=%s',
                          msg.connected, rospy.get_time(), msg.armed, msg.mode)
        self.fc = msg
        self.fc_timestamp = rospy.get_time()

    def loc_cb(self, msg):
        self.localization = msg
        self.localization_timestamp = rospy.get_time()

    def world_cb(self, msg):
        self.world = msg
        self.world_timestamp = rospy.get_time()

    def goal_cb(self, msg):
        self.goal = msg
        self.goal_timestamp = rospy.get_time()

    def tf_cb(self, m):
        q = m.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.transform = Transform2D(yaw, m.transform.translation.x,
                                    m.transform.translation.y)

    def z_cb(self, msg):
        self.z_offset = msg.data

    def odometry_velocity_cb(self, msg):
        """Undo paired FLU encoding; never consume a future measurement."""
        received = rospy.get_time()
        try:
            xyz = paired_odometry_velocity(msg, self.velocity_frame,
                                           self.velocity_body_frame)
        except ValueError as exc:
            with self.velocity_lock:
                self.velocity_source_error = str(exc)
                self.velocity = None
                self.velocity_pending = None
            return
        velocity = TwistStamped()
        velocity.header.stamp = msg.header.stamp
        velocity.header.frame_id = msg.header.frame_id
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z = xyz
        future = msg.header.stamp.to_sec()-received
        with self.velocity_lock:
            self.velocity_source_error = None
            if future > 0:
                if future <= self.velocity_future_hold_limit_s:
                    # Keep the last genuinely fresh sample until this stamp
                    # matures, exactly as the estimator's IMU pending path.
                    self.velocity_pending = (velocity, received)
                else:
                    self.velocity = None
                    self.velocity_pending = None
                    self.velocity_source_error = 'FUTURE_VELOCITY_HEADER'
                return
            self.velocity_pending = None
            self.velocity = velocity
            self.velocity_stamp = received

    def release_pending_velocity(self, now):
        if getattr(self, 'velocity_pending', None) is None:
            return
        with self.velocity_lock:
            if self.velocity_pending is None:
                return
            velocity, received = self.velocity_pending
            age = now-velocity.header.stamp.to_sec()
            if not 0 <= now-received <= .30:
                self.velocity_pending = None
                self.velocity_source_error = 'PENDING_VELOCITY_EXPIRED'
            elif 0 <= age <= .30:
                self.velocity = velocity
                self.velocity_stamp = received  # Original receipt, never refreshed.
                self.velocity_pending = None

    def velocity_cb(self, msg):
        self.velocity = msg
        self.velocity_stamp = rospy.get_time()

    def ack(self, _req):
        if self.fc.armed:
            return TriggerResponse(False, 'DISARM before acknowledging safety')
        if not self.origin_verified:
            return TriggerResponse(False, 'Fixed EKF origin not manually verified in safety.yaml')
        self.operator_ack = True
        ok, reason = self.checks(rospy.get_time())
        return TriggerResponse(True, 'Operator acknowledged; next check: '+reason+'. Does NOT arm or start flight.')

    def reset(self, _req):
        if self.fc.armed:
            return TriggerResponse(False, 'Cannot reset abort while armed')
        self.abort_latched = False
        self.abort_reason = ''
        self.freeze_detector.reset('OPERATOR_RESET')
        self.gazebo_clock_rewound = False
        self.was_guided_flying = False
        self.operator_ack = False
        return TriggerResponse(True, 'Abort reset, repeat /safety/acknowledge before flight')

    def validate_params(self, _event):
        if not self.check_fc_params:
            self.param_good = True
            self.param_reason = 'PARAM_CHECK_DISABLED_FOR_EXPLICIT_SITL'
            return
        expected = dict(self.params_cfg.get('ekf', {}))
        expected.update(self.params_cfg.get('mtf01p', {}))
        if not expected:
            self.param_good = False
            self.param_reason = 'EMPTY_EXPECTED_PARAMETER_LIST'
            return
        try:
            rospy.wait_for_service('/mavros/param/get', timeout=.3)
            srv = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            for name, val in expected.items():
                r = srv(param_id=str(name))
                if not r.success:
                    self.param_good = False
                    self.param_reason = 'MISSING_FC_PARAM:' + name
                    return
                actual = float(r.value.integer) if float(r.value.real) == 0. else float(r.value.real)
                if abs(actual-float(val)) > .01:
                    self.param_good = False
                    self.param_reason = '{}={} expected {}'.format(name, actual, val)
                    return
            self.param_good = True
            self.param_reason = 'OK'
        except (rospy.ROSException, rospy.ServiceException) as ex:
            self.param_good = False
            self.param_reason = 'PARAM_SERVICE:' + str(ex)
        if not self.param_good:
            rospy.logwarn_throttle(5., 'FC param verification: %s', self.param_reason)

    def accepted_uwb_cb(self, msg):
        self.accepted_uwb = (msg, rospy.get_time())

    def truth_cb(self, msg):
        try:
            i = msg.name.index(self.model_name)
            p = msg.pose[i].position
            if not finite(p.x, p.y, p.z):
                raise ValueError('nonfinite pose')
            twist = msg.twist[i].linear if i < len(msg.twist) else None
            if twist is not None and not finite(twist.x, twist.y, twist.z):
                raise ValueError('nonfinite Gazebo twist')
            self.truth = ((p.x, p.y), rospy.get_time(),
                          (twist.x, twist.y) if twist is not None else None)
            self.truth_error = ''
        except (ValueError, IndexError):
            self.truth = None
            self.truth_error = 'GAZEBO_MODEL_MISSING_OR_INVALID'

    def clock_cb(self, msg):
        value, now = msg.clock.to_sec(), rospy.get_time()
        if self.clock is None or value != self.clock:
            self.clock_progress_receipt = now
        if self.clock is not None and value < self.clock:
            self.gazebo_clock_rewound = True
            self.freeze_detector.reset('GAZEBO_CLOCK_REWIND')
            self.truth = None
            self.truth_error = 'GAZEBO_CLOCK_REWIND'
        self.clock = value

    @staticmethod
    def message_age(msg, now):
        return now-msg.header.stamp.to_sec()

    def freeze_check(self, now):
        """SITL requires independent truth; never silently fall back to EKF."""
        self.release_pending_velocity(now)
        f = self.cfg.get('freeze_detection', {})
        if not f.get('enabled', False):
            self.freeze_detector.reset('DISABLED')
            self.freeze_snapshot = dict(self.freeze_detector.snapshot)
            return False
        pending_velocity = self.velocity_pending
        context = {'now_s': now, 'world_receipt_age_s': now-self.world_timestamp,
                   'world_header_age_s': self.message_age(self.world, now) if self.world else None,
                   'velocity_receipt_age_s': now-self.velocity_stamp,
                   'velocity_header_age_s': self.message_age(self.velocity, now) if self.velocity else None,
                   'world_xyz': [self.world.pose.position.x, self.world.pose.position.y,
                                 self.world.pose.position.z] if self.world else None,
                   'velocity_frame': self.velocity.header.frame_id if self.velocity else None,
                   'velocity_source': '/mavros/local_position/odom',
                   'velocity_source_error': self.velocity_source_error,
                   'velocity_pending_header_s': (pending_velocity[0].header.stamp.to_sec()
                                                  if pending_velocity else None),
                   'velocity_future_hold_limit_s': self.velocity_future_hold_limit_s,
                   'uwb_age_s': self.localization.uwb_age_s if self.localization else None,
                   'range_age_s': self.localization.laser_age_s if self.localization else None,
                   'fc_connected': self.fc.connected, 'fc_age_s': now-self.fc_timestamp,
                   'fc_armed': self.fc.armed, 'fc_mode': self.fc.mode,
                   'mission_state': self.mission_state}
        self.freeze_snapshot = context
        def unavailable(reason):
            previous = self.freeze_detector.snapshot
            self.freeze_detector.reset(reason)
            context.update(self.freeze_detector.snapshot)
            if previous.get('samples'):
                context['previous_window'] = previous
            return False
        if (not self.localization or not self.localization.healthy or
                not 0 <= now-self.localization_timestamp <= .30 or
                not self.fc.connected or not 0 <= now-self.fc_timestamp <= 1.):
            return unavailable('LOCALIZATION_OR_FC_UNAVAILABLE')
        if (self.world is None or not 0 <= now-self.world_timestamp <= .30 or
                not 0 <= self.message_age(self.world, now) <= .30):
            return unavailable('WORLD_POSE_STALE')
        if self.accepted_uwb is None:
            return unavailable('ACCEPTED_UWB_MISSING')
        uwb, received = self.accepted_uwb
        stamp = uwb.header.stamp.to_sec()
        context.update(uwb_receipt_age_s=now-received, uwb_header_age_s=now-stamp,
                       uwb_sample_stamp_s=stamp, uwb_frame=uwb.header.frame_id)
        if not (0 <= now-received <= .30 and 0 <= now-stamp <= .30):
            return unavailable('ACCEPTED_UWB_STALE')
        if uwb.header.frame_id != 'uwb_anchor':
            return unavailable('INVALID_UWB_FRAME')
        if (self.velocity is None or not 0 <= now-self.velocity_stamp <= .30 or
                not 0 <= self.message_age(self.velocity, now) <= .30 or
                self.transform is None):
            return unavailable('VELOCITY_STALE_OR_TRANSFORM_MISSING')
        # Paired Odometry has been decoded into navigation ENU. Check its frame.
        if self.velocity.header.frame_id != self.velocity_frame:
            return unavailable('INVALID_VELOCITY_FRAME')
        v = self.velocity.twist.linear
        c, sn = math.cos(self.transform.yaw), math.sin(self.transform.yaw)
        velocity_world = (c*v.x+sn*v.y, -sn*v.x+c*v.y)
        context['velocity_nav_xy'] = [v.x, v.y]
        context['velocity_world_xy'] = list(velocity_world)
        truth_xy = None
        if self.cfg.get('profile') == 'sitl':
            if self.gazebo_clock_rewound:
                return unavailable('GAZEBO_CLOCK_REWIND')
            context['truth_age_s'] = now-self.truth[1] if self.truth else None
            context['gazebo_clock_s'] = self.clock
            context['clock_progress_age_s'] = (now-self.clock_progress_receipt
                                               if self.clock_progress_receipt is not None else None)
            if self.truth is None or not 0 <= now-self.truth[1] <= .30:
                return unavailable(self.truth_error or 'GAZEBO_TRUTH_STALE')
            if self.clock_progress_receipt is None or not 0 <= now-self.clock_progress_receipt <= .30:
                return unavailable('GAZEBO_CLOCK_STALLED_OR_MISSING')
            truth_xy = self.truth[0]
            context['truth_velocity_xy'] = self.truth[2]
            context['truth_speed_mps'] = math.hypot(*self.truth[2]) if self.truth[2] is not None else None
        triggered = self.freeze_detector.observe(stamp, (uwb.point.x, uwb.point.y),
                                                  velocity_world, truth_xy, dict(context))
        context.update(self.freeze_detector.snapshot)
        classification = context['classification']
        if classification != self.last_freeze_classification:
            rospy.logwarn('Freeze classification: %s', json.dumps(context))
            self.last_freeze_classification = classification
        return triggered

    def checks(self, now):
        if not self.origin_verified:
            return False, 'FIXED_ORIGIN_NOT_VERIFIED'
        if not self.operator_ack:
            return False, 'OPERATOR_NOT_ACKNOWLEDGED'
        if self.abort_latched:
            return False, 'ABORT_LATCHED:' + self.abort_reason
        if not self.param_good:
            return False, 'PARAM_MISMATCH:' + self.param_reason
        if not self.fc.connected or not 0 <= now-self.fc_timestamp <= 1.:
            return False, 'FC_DISCONNECTED'
        if (not self.localization or not 0 <= now-self.localization_timestamp <= .30 or
                not self.localization.healthy or not self.localization.transform_locked):
            return False, 'LOCALIZATION_UNHEALTHY'
        if not self.transform or self.z_offset is None or not finite(self.z_offset):
            return False, 'CALIBRATION_TRANSFORM_MISSING'
        if self.world is None or not 0 <= now-self.world_timestamp <= .30:
            return False, 'WORLD_POSE_STALE'
        pos = (self.world.pose.position.x, self.world.pose.position.y,
               self.world.pose.position.z)
        if self.world.header.frame_id != self.cfg['frame_id']:
            return False, 'INVALID_WORLD_FRAME'
        reason = fence_reason(pos, self.box, self.fc.armed,
                              self.mission_state in ('RUNNING', 'COMPLETE'))
        if reason != 'READY':
            return False, reason
        if self.cfg.get('freeze_detection', {}).get('enabled', False):
            unavailable = self.freeze_snapshot.get('classification', '')
            if unavailable in ('WORLD_POSE_STALE', 'ACCEPTED_UWB_MISSING', 'ACCEPTED_UWB_STALE',
                               'INVALID_UWB_FRAME', 'VELOCITY_STALE_OR_TRANSFORM_MISSING',
                               'INVALID_VELOCITY_FRAME', 'GAZEBO_MODEL_MISSING_OR_INVALID',
                               'GAZEBO_TRUTH_MISSING', 'GAZEBO_TRUTH_STALE', 'GAZEBO_CLOCK_REWIND',
                               'GAZEBO_CLOCK_STALLED_OR_MISSING', 'GAZEBO_POSE_TWIST_INCONSISTENT', 'INVALID_SAMPLE'):
                return False, unavailable
        # Predict near-future horizontal motion before hitting the hard fence.
        # Velocity is FC EKF output (not an independent collision sensor).
        horizon = float(self.cfg.get('mission', {}).get('fence_prediction_s', 0.35))
        if (self.was_guided_flying and self.fc.armed and self.fc.mode == 'GUIDED' and
                horizon > 0 and self.velocity is not None and
                0 <= now-self.velocity_stamp < .30 and
                0 <= self.message_age(self.velocity, now) < .30 and
                self.velocity.header.frame_id == self.velocity_frame):
            vx = self.velocity.twist.linear.x
            vy = self.velocity.twist.linear.y
            if finite(vx, vy):
                # Decoded velocity is nav ENU. Convert using inverse world rotation.
                c, s = math.cos(self.transform.yaw), math.sin(self.transform.yaw)
                wx, wy = c*vx+s*vy, -s*vx+c*vy
                px = pos[0] + horizon*wx
                py = pos[1] + horizon*wy
                if not (self.box['x_min'] <= px <= self.box['x_max'] and
                        self.box['y_min'] <= py <= self.box['y_max']):
                    return False, 'PREDICTED_FENCE_BREACH'
        if self.freeze_snapshot.get('triggered', False):
            return False, 'POSSIBLE_UWB_FREEZE'
        return True, 'READY'

    def abort(self, reason):
        if self.abort_latched:
            return
        self.abort_latched = True
        self.abort_reason = reason
        self.abort_snapshot = {'event_s': rospy.get_time(), 'reason': reason,
                               'freeze': self.freeze_snapshot,
                               'localization_reason': self.localization.reason if self.localization else None}
        payload = json.dumps(self.abort_snapshot)
        self.abort_diagnostics.publish(String(data=payload))
        rospy.logerr('INDOOR NAV ABORT SNAPSHOT: %s', payload)
        rospy.logerr('INDOOR NAV ABORT: %s', reason)

    def request_land(self, now):
        if not self.fc.connected or not self.fc.armed or self.fc.mode != 'GUIDED':
            return
        if now-self.last_land_attempt < 1.0:
            return
        self.last_land_attempt = now
        try:
            srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            result = srv(base_mode=0, custom_mode='LAND')
            rospy.logerr('Failsafe LAND request sent, mode_sent=%s', result.mode_sent)
        except rospy.ServiceException as ex:
            rospy.logerr('LAND mode request failed: %s; pilot RC override required', ex)

    def tick(self, _event):
        # Sample time and velocity under one lock: a concurrently arriving
        # message must not be compared with an earlier watchdog epoch.
        with self.velocity_lock:
            now = rospy.get_time()
            self.freeze_check(now)
            ok, reason = self.checks(now)
        guided = self.fc.armed and self.fc.mode == 'GUIDED'
        active = self.mission_state in ('RUNNING', 'COMPLETE')
        # RC direct override always wins; never force LAND after leaving GUIDED.
        if guided and active and ok:
            if self.goal is None or now-self.goal_timestamp > float(
                    self.cfg.get('mission', {}).get('setpoint_timeout_s', .50)):
                # A RUNNING state and its first target use different ROS
                # connections. Give the first target the SAME configured
                # deadline; forward nothing while waiting. Never extend it
                # with repeated RUNNING messages or after forwarding began.
                started = getattr(self, 'mission_start_stamp', None)
                timeout = float(self.cfg.get('mission', {}).get('setpoint_timeout_s', .50))
                if self.was_guided_flying or started is None or now-started >= timeout:
                    self.abort('SETPOINT_TIMEOUT')
            elif self.goal.header.frame_id != 'odom':
                self.abort('INVALID_SETPOINT_FRAME')
            else:
                gx, gy = self.transform.inverse(self.goal.pose.position.x,
                                                 self.goal.pose.position.y)
                gz = self.goal.pose.position.z - self.z_offset
                if not inside_box((gx, gy, gz), self.box):
                    self.abort('SETPOINT_OUTSIDE_GEOFENCE')
                elif not finite(gx, gy, gz):
                    self.abort('NAN_SETPOINT')
                else:
                    self.was_guided_flying = True
                    self.publisher.publish(self.goal)
        elif guided and self.was_guided_flying and not ok:
            self.abort(reason)
        elif guided and self.was_guided_flying and not active:
            self.abort('MISSION_STOPPED')
        if self.abort_latched:
            # Not a guarantee of landing: flight controller must retain RC and
            # native failsafes independently. Reattempt until mode changes.
            self.request_land(now)
        msg = SafetyStatus()
        msg.header.stamp = rospy.Time.now()
        msg.state = 'ABORT' if self.abort_latched else ('FLYING' if guided and active and ok else
                       'READY' if ok else 'WAIT')
        msg.ready = ok and not self.abort_latched
        msg.abort_latched = self.abort_latched
        msg.reason = self.abort_reason if self.abort_latched else reason
        self.status.publish(msg)
        p = self.world.pose.position if self.world else None
        self.diagnostics.publish(String(data=json.dumps({
            'freeze': {k: v for k, v in self.freeze_snapshot.items() if k not in ('samples', 'previous_window')},
            'reason': msg.reason, 'profile': self.cfg.get('profile'),
            'config_source': self.cfg.get('config_source'), 'geofence': self.box,
            'world_xyz': [p.x, p.y, p.z] if p else None,
            'frame_id': self.world.header.frame_id if self.world else None,
            'armed': self.fc.armed, 'mode': self.fc.mode,
            'world_age_s': now-self.world_timestamp,
            'localization_age_s': now-self.localization_timestamp,
            'uwb_age_s': self.localization.uwb_age_s if self.localization else None,
            'laser_age_s': self.localization.laser_age_s if self.localization else None})))


if __name__ == '__main__':
    SafetySupervisor()
    rospy.spin()
