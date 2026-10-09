#!/usr/bin/env python3
"""Nooploop LinkTrack -> fixed world_lab -> MAVROS ODOMETRY.

V1 is NOT an INS/Kalman filter. ExternalNav XY is from UWB. FC retains
RangeFinder Z and OpticalFlow Vxy. Odometry Z is estimated from the range sensor
without feeding back EKF local_position (prevents ExternalNav cold-start deadlock).
Do not arm before
handheld frame checks and SITL source-mixing tests.
"""
import math
import json
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Imu
from mavros_msgs.msg import State
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64, String
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
        self.nav_z_offset = float(self.world_cfg.get('nav_z_offset_m', 0.0))
        if len(self.nav_origin_xy) != 2 or not finite(*self.nav_origin_xy, self.nav_z_offset):
            raise ValueError('Invalid fixed navigation origin offsets')
        self.fc_state = State()
        self.fc_stamp = 0.
        self.fc_pose = None
        self.fc_pose_stamp = 0.
        self.fc_imu = None
        self.fc_imu_stamp = 0.
        self.imu_timeout = float(self.cfg.get('imu_attitude_timeout_s', 0.40))
        self.height = None
        self.height_stamp = 0.
        self.height_header_stamp = 0.
        self.height_cfg = height
        self.uwb = UwbGate(ucfg.get('max_jump_m', .30),
                           ucfg.get('max_implied_speed_mps', 1.50),
                           ucfg.get('timeout_s', .20),
                           ucfg.get('noise_floor_m', .10),
                           ucfg.get('max_eop_xy_m', .50))
        self.raw_xy = None
        self.eop = 0.
        self.last_publish_reason = None
        self.last_world_publish = None
        self.raw_uwb = None
        self.imu_rejection = ''
        self.range_rejection = ''
        self.twist_pose_stale = float(self.cfg.get('fc_pose_timeout_s', .40))
        self.range_timeout = float(height.get('timeout_s', .20))
        self.output_enabled = bool(out.get('external_nav_enabled', False))
        self.output_rate = float(out.get('odom_rate_hz', 30.))
        self.status_pub = rospy.Publisher('/indoor_nav/localization_status', LocalizationStatus,
                                           queue_size=10, latch=True)
        self.uwb_sample_pub = rospy.Publisher('/indoor_nav/accepted_uwb', PointStamped, queue_size=30)
        self.diagnostics_pub = rospy.Publisher('/indoor_nav/localization_diagnostics', String, queue_size=1, latch=True)
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
        # Local EKF pose is kept only for diagnostics; NEVER required for calibration/output.
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=5)
        rospy.Subscriber(self.cfg.get('imu_attitude_topic', '/mavros/imu/data'),
                         Imu, self.imu_cb, queue_size=10)
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

    def imu_cb(self, m):
        with self.lock:
            if not 0 <= rospy.get_time()-m.header.stamp.to_sec() <= self.imu_timeout:
                self.imu_rejection = 'STALE_OR_FUTURE_HEADER'
                return
            self.imu_rejection = ''
            q = get_quat(m.orientation)
            if not finite(*q):
                return
            try:
                normalize_quaternion(q)
            except ValueError:
                return
            if m.orientation_covariance[0] == -1:
                return
            self.fc_imu = m
            self.fc_imu_stamp = rospy.get_time()

    def range_cb(self, m):
        with self.lock:
            if not 0 <= rospy.get_time()-m.header.stamp.to_sec() <= self.range_timeout:
                self.range_rejection = 'STALE_OR_FUTURE_HEADER'
                return
            if not finite(m.range):
                self.range_rejection = 'NONFINITE_RANGE'
                return
            valid_low = max(float(m.min_range), float(self.height_cfg.get('min_valid_m', .10)))
            valid_high = min(float(m.max_range), float(self.height_cfg.get('max_valid_m', 2.)))
            if valid_low <= float(m.range) <= valid_high:
                self.height = float(m.range)
                self.height_stamp = rospy.get_time()
                self.height_header_stamp = m.header.stamp.to_sec()
                self.range_rejection = ''
            else:
                self.range_rejection = 'OUT_OF_RANGE:' + str(m.range)

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
            self.raw_uwb = {"receipt_s": now, "xy": [x, y], "eop_xy": [ex, ey],
                            "device_local_time": m.local_time, "device_system_time": m.system_time}
            eop = math.hypot(ex, ey)
            if self.uwb.evaluate(now, x, y, eop):
                self.raw_xy = (x, y)
                self.eop = eop
                sample = PointStamped()
                sample.header.stamp = rospy.Time.now()
                sample.header.frame_id = 'uwb_anchor'
                sample.point.x, sample.point.y = x, y
                self.uwb_sample_pub.publish(sample)
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
            if self.fc_imu is None or not 0 <= rospy.get_time()-self.fc_imu_stamp <= self.imu_timeout or not 0 <= rospy.get_time()-self.fc_imu.header.stamp.to_sec() <= self.imu_timeout:
                return TriggerResponse(False, 'FC IMU attitude missing or stale')
            if self.height is None or not 0 <= rospy.get_time()-self.height_stamp <= self.range_timeout or not 0 <= rospy.get_time()-self.height_header_stamp <= self.range_timeout:
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
            yaw = quaternion_yaw(get_quat(self.fc_imu.orientation))
            # Physical aircraft +X is aligned with world +X at calibration.
            # ROS FC yaw is the heading of body X in the navigation ENU frame.
            if not finite(yaw, *bias, *self.nav_origin_xy):
                return TriggerResponse(False, 'Nonfinite transform')
            self.bias = bias
            self.z_offset = self.nav_z_offset
            if not finite(self.z_offset):
                return TriggerResponse(False, 'invalid height offset')
            self.z_offset_pub.publish(Float64(data=self.z_offset))
            self.transform = Transform2D(yaw, self.nav_origin_xy[0], self.nav_origin_xy[1])
            self.publish_transform()
            rospy.logwarn('Calibration LOCKED (NO EKF local-pose dependency): yaw world->nav=%.2f deg, T1 bias=(%.3f,%.3f)',
                          math.degrees(yaw), bias[0], bias[1])
            return TriggerResponse(True, 'CALIBRATED and LOCKED; check +X/+Y hand motion before flight')

    def publish_transform(self):
        tr = TransformStamped()
        tr.header.stamp = rospy.Time.now()
        tr.header.frame_id = 'odom'
        tr.child_frame_id = self.world_cfg.get('frame_id', 'world_lab')
        tr.transform.translation.x = self.transform.tx
        tr.transform.translation.y = self.transform.ty
        tr.transform.translation.z = self.nav_z_offset
        assign_quat(tr.transform.rotation, yaw_quaternion(self.transform.yaw))
        self.tf_pub.publish(tr)

    def height_m(self):
        if self.height is None or self.fc_imu is None:
            return None
        q = get_quat(self.fc_imu.orientation)
        roll, pitch = roll_pitch(q)
        return self.height * math.cos(roll) * math.cos(pitch) + float(
                self.height_cfg.get('range_to_body_z_m', 0.))

    def health(self, now):
        if self.transform is None:
            return False, 'NOT_CALIBRATED'
        if not self.fc_state.connected or not 0 <= now-self.fc_stamp <= 2.:
            return False, 'FC_LOST'
        if not self.uwb.fresh(now):
            return False, 'UWB_STALE_OR_REJECTED'
        if self.fc_imu is None or not 0 <= now-self.fc_imu_stamp <= self.imu_timeout or not 0 <= now-self.fc_imu.header.stamp.to_sec() <= self.imu_timeout:
            return False, 'FC_IMU_STALE'
        if self.height is None or not 0 <= now-self.height_stamp <= self.range_timeout or not 0 <= now-self.height_header_stamp <= self.range_timeout:
            return False, 'RANGE_STALE'
        return True, 'OK'

    def publish(self, _event):
        now = rospy.get_time()
        with self.lock:
            # Freeze evidence is gathered on accepted UWB samples by supervisor.
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
            diagnostic = {'stamp_s': now, 'healthy': ok, 'reason': reason,
                          'world_publish_enabled': ok, 'last_world_publish_s': self.last_world_publish,
                          'fc_connected': self.fc_state.connected, 'fc_age_s': now-self.fc_stamp,
                          'imu_age_s': now-self.fc_imu_stamp, 'range_age_s': now-self.height_stamp,
                          'range_header_age_s': now-self.height_header_stamp,
                          'range_rejection': self.range_rejection, 'imu_rejection': self.imu_rejection,
                          'imu_header_age_s': now-self.fc_imu.header.stamp.to_sec() if self.fc_imu else None,
                          'uwb_accepted_age_s': now-self.uwb.last[0] if self.uwb.last else None,
                          'uwb_received_age_s': now-self.uwb.last_rx if self.uwb.last_rx is not None else None,
                          'uwb_gate_reason': self.uwb.reason, 'uwb_rejected': self.uwb.rejected,
                          'raw_uwb': self.raw_uwb, 'accepted_xy': self.raw_xy,
                          'uwb_timestamp_basis': 'ROS_RECEIPT_NO_SYNCHRONIZED_DEVICE_HEADER'}
            self.diagnostics_pub.publish(String(data=json.dumps(diagnostic)))
            if reason != self.last_publish_reason:
                rospy.logwarn('Localization publication transition: %s', json.dumps(diagnostic))
                self.last_publish_reason = reason
            if not ok:
                return
            q = normalize_quaternion(get_quat(self.fc_imu.orientation))
            q_world = normalize_quaternion(quaternion_product(yaw_quaternion(-self.transform.yaw), q))
            tag_offset_world = rotate_vector(q_world, self.body_to_tag_flu)
            xw = self.raw_xy[0]-self.bias[0]-tag_offset_world[0]
            yw = self.raw_xy[1]-self.bias[1]-tag_offset_world[1]
            z_world = self.height_m()
            zn = z_world + self.nav_z_offset
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
            self.last_world_publish = now
            odom = Odometry()
            odom.header.stamp = st.header.stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = xn, yn
            # Independent range-derived Z; EKF POSZ must remain Baro in this test.
            # Large ROS covariance alone cannot guarantee FC ignores ODOMETRY Z.
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
