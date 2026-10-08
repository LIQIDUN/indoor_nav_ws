#!/usr/bin/env python3
"""Nooploop LinkTrack -> fixed world_lab -> MAVROS ODOMETRY.

V1 is NOT an INS/Kalman filter. ExternalNav XY is from UWB. FC retains
RangeFinder Z and OpticalFlow Vxy. Odometry Z mirrors FC local Z to avoid
mixed-source vertical reset risks in some ArduPilot versions. Do not arm before
handheld frame checks and SITL source-mixing tests.
"""
import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from mavros_msgs.msg import State
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from nlink_parser.msg import LinktrackNodeframe2, LinktrackAnchorframe0
from indoor_nav.msg import LocalizationStatus
from indoor_nav.core import (Transform2D, UwbGate, GroundCalibration, finite,
                             quaternion_yaw, yaw_quaternion, quaternion_product,
                             normalize_quaternion, roll_pitch, rotate_vector)


def get_quat(msg):
    return (msg.x, msg.y, msg.z, msg.w)


def assign_quat(dst, q):
    dst.x, dst.y, dst.z, dst.w = q


class WorldEstimator:
    def __init__(self):
        rospy.init_node('world_estimator')
        self.cfg = rospy.get_param('/indoor_nav/localization', {})
        self.world_cfg = self.cfg.get('world', {})
        ucfg = self.cfg.get('uwb', {})
        ref = self.world_cfg.get('ground_reference', {})
        align = self.world_cfg.get('yaw_alignment', {})
        out = self.cfg.get('output', {})
        height = self.cfg.get('height', {})
        self.lock = threading.RLock()
        self.aircraft_id = int(ucfg.get('aircraft_tag_id', -1))
        self.body_to_tag_flu = [float(v) for v in ucfg.get('body_to_tag_flu_m', [0.,0.,0.])]
        if len(self.body_to_tag_flu) != 3 or not finite(*self.body_to_tag_flu):
            raise ValueError('body_to_tag_flu_m must contain 3 finite numbers')
        self.ground_id = int(ref.get('tag_id', -1))
        self.require_ground = bool(ref.get('enabled', False))
        self.require_anchor_verified = bool(self.world_cfg.get('anchor_map_verified', False))
        self.ground_window = float(ref.get('calibration_time_s', 8.0))
        self.ground_raw = deque(maxlen=600)
        self.ground_cal = GroundCalibration(ref.get('physical_xy', [0., 0.]),
             ref.get('min_samples', 25), ref.get('max_allowed_offset_m', 0.30),
             ref.get('max_std_m', 0.10))
        self.yaw_cfg = align
        self.transform = None
        self.bias = (0., 0.)
        self.z_offset = None
        self.nav_origin_xy = self.world_cfg.get('nav_xy_at_world_origin', [0., 0.])
        self.fc_state = State()
        self.fc_stamp = 0.
        self.fc_pose = None
        self.fc_pose_stamp = 0.
        self.height = None
        self.height_stamp = 0.
        self.height_cfg = height
        self.uwb = UwbGate(ucfg.get('max_jump_m', .30),
                           ucfg.get('max_implied_speed_mps', 1.50),
                           ucfg.get('timeout_s', .20),
                           ucfg.get('noise_floor_m', .10),
                           ucfg.get('max_eop_xy_m', .50))
        self.raw_xy = None
        self.eop = 0.
        self.freeze_first = None
        self.freeze_position = None
        self.freeze_reason = ''
        self.twist_pose_stale = float(self.cfg.get('fc_pose_timeout_s', .40))
        self.range_timeout = float(height.get('timeout_s', .20))
        self.output_enabled = bool(out.get('external_nav_enabled', False))
        self.output_rate = float(out.get('odom_rate_hz', 30.))
        self.status_pub = rospy.Publisher('/indoor_nav/localization_status', LocalizationStatus,
                                           queue_size=10, latch=True)
        self.world_pub = rospy.Publisher('/indoor_nav/world_pose', PoseStamped, queue_size=10)
        self.nav_odom_pub = rospy.Publisher('/indoor_nav/ap_odom', Odometry, queue_size=10)
        self.fc_odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        self.tf_pub = rospy.Publisher('/indoor_nav/nav_from_world', TransformStamped,
                                      queue_size=1, latch=True)
        self.z_offset_pub = rospy.Publisher('/indoor_nav/z_offset_m', Float64,
                                           queue_size=1, latch=True)
        rospy.Subscriber(ucfg.get('aircraft_topic', '/nlink_linktrack_nodeframe2'),
                         LinktrackNodeframe2, self.uwb_cb, queue_size=30)
        if self.require_ground:
            rospy.Subscriber(ref.get('topic', '/nlink_linktrack_anchorframe0'),
                             LinktrackAnchorframe0, self.ground_cb, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=5)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=5)
        rospy.Subscriber(height.get('range_topic', '/mavros/rangefinder/rangefinder'),
                         Range, self.range_cb, queue_size=5)
        rospy.Service('/indoor_nav/calibrate', Trigger, self.calibrate)
        rospy.Timer(rospy.Duration(1.0/self.output_rate), self.publish)
        rospy.logwarn('world_estimator: external nav forwarding=%s; only calibrated frames publish',
                      self.output_enabled)

    def state_cb(self, m):
        with self.lock:
            self.fc_state = m
            self.fc_stamp = rospy.get_time()

    def pose_cb(self, m):
        with self.lock:
            q = get_quat(m.pose.orientation)
            if not finite(m.pose.position.x, m.pose.position.y, m.pose.position.z, *q):
                return
            try:
                normalize_quaternion(q)
            except ValueError:
                return
            self.fc_pose = m
            self.fc_pose_stamp = rospy.get_time()

    def range_cb(self, m):
        with self.lock:
            if not finite(m.range):
                return
            valid_low = max(float(m.min_range), float(self.height_cfg.get('min_valid_m', .10)))
            valid_high = min(float(m.max_range), float(self.height_cfg.get('max_valid_m', 2.)))
            if valid_low <= float(m.range) <= valid_high:
                self.height = float(m.range)
                self.height_stamp = rospy.get_time()

    def ground_cb(self, m):
        now = rospy.get_time()
        with self.lock:
            if self.transform is not None or self.fc_state.armed:
                return
            for tag in m.nodes:
                if int(tag.id) == self.ground_id and finite(*tag.pos_3d[:2]):
                    self.ground_raw.append((now, float(tag.pos_3d[0]), float(tag.pos_3d[1])))
                    break

    def uwb_cb(self, m):
        if int(m.id) != self.aircraft_id:
            return
        now = rospy.get_time()
        with self.lock:
            try:
                x, y = float(m.pos_3d[0]), float(m.pos_3d[1])
                ex, ey = float(m.eop_3d[0]), float(m.eop_3d[1])
            except (IndexError, TypeError, ValueError):
                return
            eop = math.hypot(ex, ey)
            if self.uwb.evaluate(now, x, y, eop):
                self.raw_xy = (x, y)
                self.eop = eop
                if self.freeze_position is None or math.hypot(x-self.freeze_position[0],
                                            y-self.freeze_position[1]) > float(
                                               self.cfg.get('freeze', {}).get('movement_epsilon_m', .015)):
                    self.freeze_first = now
                    self.freeze_position = (x, y)
                    self.freeze_reason = ''
            # A single jump can be rejected, but the freshness timer is based
            # on LAST ACCEPTED measurement, never a freshly rejected packet.

    def calibrate(self, _req):
        with self.lock:
            if self.fc_state.armed:
                return TriggerResponse(False, 'DISARM first; world transform locked in flight')
            if self.transform is not None:
                return TriggerResponse(False, 'Already calibrated; restart node while DISARMED to recalibrate')
            if not self.fc_state.connected or rospy.get_time()-self.fc_stamp > 2.:
                return TriggerResponse(False, 'FC not connected')
            if not self.require_anchor_verified:
                return TriggerResponse(False, 'anchor_map_verified=false')
            if not self.uwb.fresh(rospy.get_time()):
                return TriggerResponse(False, 'Aircraft UWB missing or stale')
            if self.fc_pose is None or rospy.get_time()-self.fc_pose_stamp > self.twist_pose_stale:
                return TriggerResponse(False, 'FC local pose missing or stale')
            if self.height is None or rospy.get_time()-self.height_stamp > self.range_timeout:
                return TriggerResponse(False, 'RangeFinder ROS topic missing or stale')
            if self.yaw_cfg.get('method', 'fixed_start_pose') != 'fixed_start_pose':
                return TriggerResponse(False, 'Unsupported V1 yaw alignment method')
            if not self.yaw_cfg.get('require_nose_along_world_x', True):
                return TriggerResponse(False, 'Require physical nose alignment with world +X')
            bias = (0., 0.)
            if self.require_ground:
                now = rospy.get_time()
                self.ground_cal.samples = [(x, y) for t, x, y in self.ground_raw
                                      if 0 <= now-t <= self.ground_window + .2]
                if (not self.ground_raw or now-self.ground_raw[-1][0] > 0.5 or
                    not any(now-t >= self.ground_window*.8 for t, _, _ in self.ground_raw)):
                    return TriggerResponse(False, 'Ground tag has not been stable for calibration window')
                try:
                    bias = self.ground_cal.result()
                except ValueError as exc:
                    return TriggerResponse(False, str(exc))
            yaw = quaternion_yaw(get_quat(self.fc_pose.pose.orientation))
            # Physical aircraft +X is aligned with world +X at calibration.
            # ROS FC yaw is the heading of body X in the navigation ENU frame.
            if not finite(yaw, *bias, *self.nav_origin_xy):
                return TriggerResponse(False, 'Nonfinite transform')
            self.bias = bias
            self.z_offset = self.fc_pose.pose.position.z - self.height_m()
            if not finite(self.z_offset):
                return TriggerResponse(False, 'invalid height offset')
            self.z_offset_pub.publish(Float64(data=self.z_offset))
            self.transform = Transform2D(yaw, self.nav_origin_xy[0], self.nav_origin_xy[1])
            self.publish_transform()
            rospy.logwarn('Calibration LOCKED: yaw world->nav=%.2f deg, T1 bias=(%.3f,%.3f)',
                          math.degrees(yaw), bias[0], bias[1])
            return TriggerResponse(True, 'CALIBRATED and LOCKED; check +X/+Y hand motion before flight')

    def publish_transform(self):
        tr = TransformStamped()
        tr.header.stamp = rospy.Time.now()
        tr.header.frame_id = 'odom'
        tr.child_frame_id = self.world_cfg.get('frame_id', 'world_lab')
        tr.transform.translation.x = self.transform.tx
        tr.transform.translation.y = self.transform.ty
        tr.transform.translation.z = 0.
        assign_quat(tr.transform.rotation, yaw_quaternion(self.transform.yaw))
        self.tf_pub.publish(tr)

    def height_m(self):
        if self.height is None or self.fc_pose is None:
            return None
        q = get_quat(self.fc_pose.pose.orientation)
        roll, pitch = roll_pitch(q)
        return self.height * math.cos(roll) * math.cos(pitch) + float(
                self.height_cfg.get('range_to_body_z_m', 0.))

    def health(self, now):
        if self.transform is None:
            return False, 'NOT_CALIBRATED'
        if not self.fc_state.connected or now-self.fc_stamp > 2.:
            return False, 'FC_LOST'
        if not self.uwb.fresh(now):
            return False, 'UWB_STALE_OR_REJECTED'
        if self.fc_pose is None or now-self.fc_pose_stamp > self.twist_pose_stale:
            return False, 'FC_POSE_STALE'
        if self.height is None or now-self.height_stamp > self.range_timeout:
            return False, 'RANGE_STALE'
        return True, 'OK'

    def publish(self, _event):
        now = rospy.get_time()
        with self.lock:
            # Optional coarse freeze heuristic: position static while FC EKF
            # reports movement is implemented in the supervisor, not this
            # estimator, to avoid falsely rejecting a stationary aircraft.
            ok, reason = self.health(now)
            st = LocalizationStatus()
            st.header.stamp = rospy.Time.now()
            st.calibrated = self.transform is not None
            st.transform_locked = self.transform is not None
            st.healthy = ok
            st.armed = self.fc_state.armed
            st.flight_controller_connected = self.fc_state.connected
            st.uwb_age_s = (now-self.uwb.last[0]) if self.uwb.last else 1e6
            st.laser_age_s = (now-self.height_stamp) if self.height_stamp else 1e6
            st.eop_xy_m = self.eop
            st.rejected_count = self.uwb.rejected
            st.reason = reason
            self.status_pub.publish(st)
            if not ok:
                return
            q = normalize_quaternion(get_quat(self.fc_pose.pose.orientation))
            q_world = normalize_quaternion(quaternion_product(yaw_quaternion(-self.transform.yaw), q))
            tag_offset_world = rotate_vector(q_world, self.body_to_tag_flu)
            xw = self.raw_xy[0]-self.bias[0]-tag_offset_world[0]
            yw = self.raw_xy[1]-self.bias[1]-tag_offset_world[1]
            zn = self.fc_pose.pose.position.z
            z_world = self.height_m()
            if not finite(xw, yw, z_world, zn):
                return
            xn, yn = self.transform.apply(xw, yw)
            world = PoseStamped()
            world.header.stamp = st.header.stamp
            world.header.frame_id = self.world_cfg.get('frame_id', 'world_lab')
            world.pose.position.x, world.pose.position.y, world.pose.position.z = xw, yw, z_world
            assign_quat(world.pose.orientation,
                        q_world)
            self.world_pub.publish(world)
            odom = Odometry()
            odom.header.stamp = st.header.stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = xn, yn
            # IMPORTANT: transmit FC's own current local Z (not 0 / UWB Z).
            # ExternalNav Z may influence EKF even when POSZ!=6 in some versions.
            odom.pose.pose.position.z = zn
            assign_quat(odom.pose.pose.orientation, q)
            odom.pose.covariance = [0.] * 36
            xy_variance = max(float(self.cfg.get('output', {}).get('min_xy_sigma_m', .10))**2,
                              self.eop*self.eop if self.eop > 0 else .12**2)
            odom.pose.covariance[0] = xy_variance
            odom.pose.covariance[7] = xy_variance
            odom.pose.covariance[14] = 1e3
            for index in (21, 28, 35):
                odom.pose.covariance[index] = 1e3
            odom.twist.covariance = [0.] * 36
            for index in (0, 7, 14, 21, 28, 35):
                odom.twist.covariance[index] = 1e3
            self.nav_odom_pub.publish(odom)
            if self.output_enabled:
                self.fc_odom_pub.publish(odom)


if __name__ == '__main__':
    WorldEstimator()
    rospy.spin()
