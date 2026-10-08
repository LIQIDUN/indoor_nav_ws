#!/usr/bin/env python3
"""Separate safety gate for mission setpoints and EKF/FC health.

Not a certified failsafe. FC's independent EKF/RC/battery failsafes remain
required, and the human must be able to switch ALT_HOLD/STABILIZE directly.
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet, SetMode
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, TriggerResponse
from indoor_nav.msg import LocalizationStatus, SafetyStatus
from indoor_nav.core import Transform2D, inside_box, finite


class SafetySupervisor:
    def __init__(self):
        rospy.init_node('safety_supervisor')
        self.cfg = rospy.get_param('/indoor_nav/safety', {})
        self.params_cfg = rospy.get_param('/indoor_nav/ardupilot_expected', {})
        self.box = self.cfg.get('geofence', {})
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
        self.abort_latched = False
        self.abort_reason = ''
        self.operator_ack = False
        self.origin_verified = bool(self.cfg.get('operator_verified_fixed_ekf_origin', False))
        self.check_fc_params = bool(self.cfg.get('check_fc_params', True))
        self.param_good = False
        self.param_reason = 'FC_PARAM_CHECK_PENDING'
        self.was_guided_flying = False
        self.last_land_attempt = 0.
        self.velocity = None
        self.velocity_stamp = 0.
        self.freeze_start = None
        self.freeze_anchor_xy = None
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
                         lambda m: setattr(self, 'mission_state', m.data), queue_size=5)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped,
                         self.velocity_cb, queue_size=5)
        rospy.Service('/indoor_nav/safety/acknowledge', Trigger, self.ack)
        rospy.Service('/indoor_nav/safety/reset', Trigger, self.reset)
        rospy.Timer(rospy.Duration(.05), self.tick)
        rospy.Timer(rospy.Duration(3.), self.validate_params)

    def fc_cb(self, msg):
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

    def velocity_cb(self, msg):
        self.velocity = msg
        self.velocity_stamp = rospy.get_time()

    def ack(self, _req):
        if self.fc.armed:
            return TriggerResponse(False, 'DISARM before acknowledging safety')
        if not self.origin_verified:
            return TriggerResponse(False, 'Fixed EKF origin not manually verified in safety.yaml')
        self.operator_ack = True
        return TriggerResponse(True, 'Operator acknowledged. This does NOT arm or start flight.')

    def reset(self, _req):
        if self.fc.armed:
            return TriggerResponse(False, 'Cannot reset abort while armed')
        self.abort_latched = False
        self.abort_reason = ''
        self.was_guided_flying = False
        self.operator_ack = False
        return TriggerResponse(True, 'Abort reset, repeat /safety/acknowledge before flight')

    def validate_params(self, _event):
        if not self.check_fc_params:
            self.param_good = True
            self.param_reason = 'PARAM_CHECK_DISABLED_FOR_EXPLICIT_SITL'
            return
        expected = self.params_cfg.get('ekf', {})
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

    def freeze_check(self, now):
        f = self.cfg.get('freeze_detection', {})
        if not f.get('enabled', False) or self.world is None or self.velocity is None:
            return False
        if now-self.velocity_stamp > .30:
            self.freeze_start = None
            return False
        v = math.hypot(self.velocity.twist.linear.x, self.velocity.twist.linear.y)
        xy = (self.world.pose.position.x, self.world.pose.position.y)
        if v < float(f.get('min_speed_mps', .20)):
            self.freeze_anchor_xy = xy
            self.freeze_start = None
            return False
        if self.freeze_anchor_xy is None:
            self.freeze_anchor_xy = xy
        if math.hypot(xy[0]-self.freeze_anchor_xy[0], xy[1]-self.freeze_anchor_xy[1]) > float(
                f.get('movement_epsilon_m', .015)):
            self.freeze_anchor_xy = xy
            self.freeze_start = None
            return False
        if self.freeze_start is None:
            self.freeze_start = now
        return now-self.freeze_start > float(f.get('window_s', 1.0))

    def checks(self, now):
        if not self.origin_verified:
            return False, 'FIXED_ORIGIN_NOT_VERIFIED'
        if not self.operator_ack:
            return False, 'OPERATOR_NOT_ACKNOWLEDGED'
        if self.abort_latched:
            return False, 'ABORT_LATCHED:' + self.abort_reason
        if not self.param_good:
            return False, 'PARAM_MISMATCH:' + self.param_reason
        if not self.fc.connected or now-self.fc_timestamp > 1.:
            return False, 'FC_DISCONNECTED'
        if (not self.localization or now-self.localization_timestamp > .30 or
                not self.localization.healthy or not self.localization.transform_locked):
            return False, 'LOCALIZATION_UNHEALTHY'
        if not self.transform or self.z_offset is None or not finite(self.z_offset):
            return False, 'CALIBRATION_TRANSFORM_MISSING'
        if self.world is None or now-self.world_timestamp > .30:
            return False, 'WORLD_POSE_STALE'
        pos = (self.world.pose.position.x, self.world.pose.position.y,
               self.world.pose.position.z)
        if not inside_box(pos, self.box):
            # Pre-takeoff while unarmed may be below z_min, allow READY after
            # x/y tests; but not while GUIDED+armed and in active route.
            if self.fc.armed and self.fc.mode == 'GUIDED' and self.was_guided_flying:
                return False, 'GEOFENCE_BREACH'
            if not (self.box['x_min'] <= pos[0] <= self.box['x_max'] and
                    self.box['y_min'] <= pos[1] <= self.box['y_max']):
                return False, 'XY_OUTSIDE_FENCE'
        # Predict near-future horizontal motion before hitting the hard fence.
        # Velocity is FC EKF output (not an independent collision sensor).
        horizon = float(self.cfg.get('mission', {}).get('fence_prediction_s', 0.35))
        if (self.was_guided_flying and self.fc.armed and self.fc.mode == 'GUIDED' and
                horizon > 0 and self.velocity is not None and
                now-self.velocity_stamp < .30):
            vx = self.velocity.twist.linear.x
            vy = self.velocity.twist.linear.y
            if finite(vx, vy):
                # velocity_local is nav ENU. Convert into world using inverse rotation.
                c, s = math.cos(self.transform.yaw), math.sin(self.transform.yaw)
                wx, wy = c*vx+s*vy, -s*vx+c*vy
                px = pos[0] + horizon*wx
                py = pos[1] + horizon*wy
                if not (self.box['x_min'] <= px <= self.box['x_max'] and
                        self.box['y_min'] <= py <= self.box['y_max']):
                    return False, 'PREDICTED_FENCE_BREACH'
        if self.freeze_check(now):
            return False, 'POSSIBLE_UWB_FREEZE'
        return True, 'READY'

    def abort(self, reason):
        if self.abort_latched:
            return
        self.abort_latched = True
        self.abort_reason = reason
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
        now = rospy.get_time()
        ok, reason = self.checks(now)
        guided = self.fc.armed and self.fc.mode == 'GUIDED'
        active = self.mission_state in ('RUNNING', 'COMPLETE')
        # RC direct override always wins; never force LAND after leaving GUIDED.
        if guided and active and ok:
            if self.goal is None or now-self.goal_timestamp > float(
                    self.cfg.get('mission', {}).get('setpoint_timeout_s', .50)):
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


if __name__ == '__main__':
    SafetySupervisor()
    rospy.spin()
