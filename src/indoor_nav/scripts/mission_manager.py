#!/usr/bin/env python3
"""Manual-start YAML mission, conservative position slewing.

This node never arms or changes FC modes. Pilot must first take off to a safe
hover and enter GUIDED. Only an explicit /mission/start service starts route.
Targets go to the independent safety supervisor, NOT directly to MAVROS.
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, TriggerResponse
from indoor_nav.msg import LocalizationStatus, SafetyStatus
from indoor_nav.core import (Transform2D, bounded_step, inside_box, finite,
                             wrap_pi, quaternion_yaw, yaw_quaternion)


class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')
        self.cfg = rospy.get_param('/indoor_nav/mission', {})
        self.safety = rospy.get_param('/indoor_nav/safety', {})
        self.loc = rospy.get_param('/indoor_nav/localization', {})
        if self.cfg.get('frame_id') != self.loc.get('world', {}).get('frame_id', 'world_lab'):
            raise ValueError('Mission frame != localization world frame')
        self.route = self.cfg.get('waypoints', [])
        box = self.safety.get('geofence', {})
        self.box = box
        for point in self.route:
            p = tuple(float(point[k]) for k in ('x', 'y', 'z'))
            if not inside_box(p, box) or not finite(float(point.get('yaw_deg', 0.))):
                raise ValueError('Illegal mission waypoint outside safety geofence')
        self.state = State()
        self.locstat = None
        self.safestat = None
        self.world = None
        self.world_stamp = 0.
        self.transform = None
        self.mode = 'IDLE'
        self.index = 0
        self.setpoint = None
        self.z_offset = None
        self.stable_since = None
        self.hold_since = None
        self.tick_prev = None
        self.target_pub = rospy.Publisher('/indoor_nav/desired_setpoint', PoseStamped, queue_size=5)
        self.state_pub = rospy.Publisher('/indoor_nav/mission_state', String, queue_size=5, latch=True)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=5)
        rospy.Subscriber('/indoor_nav/localization_status', LocalizationStatus,
                         lambda m: setattr(self, 'locstat', m), queue_size=5)
        rospy.Subscriber('/indoor_nav/safety_status', SafetyStatus,
                         lambda m: setattr(self, 'safestat', m), queue_size=5)
        rospy.Subscriber('/indoor_nav/world_pose', PoseStamped, self.pose_cb, queue_size=5)
        rospy.Subscriber('/indoor_nav/nav_from_world', TransformStamped,
                         self.transform_cb, queue_size=1)
        rospy.Subscriber('/indoor_nav/z_offset_m', Float64,
                         lambda m: setattr(self, 'z_offset', m.data), queue_size=1)
        rospy.Service('/indoor_nav/mission/start', Trigger, self.start)
        rospy.Service('/indoor_nav/mission/stop', Trigger, self.stop)
        rospy.Timer(rospy.Duration(.05), self.tick)
        self.state_pub.publish(String(data=self.mode))

    def state_cb(self, msg):
        self.state = msg
        if self.mode == 'RUNNING' and (not msg.connected or not msg.armed or msg.mode != 'GUIDED'):
            self.mode = 'STOPPED_BY_PILOT'

    def pose_cb(self, msg):
        self.world = msg
        self.world_stamp = rospy.get_time()

    def transform_cb(self, msg):
        q = msg.transform.rotation
        yaw = quaternion_yaw((q.x, q.y, q.z, q.w))
        self.transform = Transform2D(yaw, msg.transform.translation.x,
                                    msg.transform.translation.y)

    def start(self, _request):
        now = rospy.get_time()
        if self.mode == 'RUNNING':
            return TriggerResponse(False, 'Already RUNNING')
        if not self.route:
            return TriggerResponse(False, 'mission.yaml has no waypoints')
        if not self.safestat or not self.safestat.ready or self.safestat.abort_latched:
            return TriggerResponse(False, 'Safety supervisor not READY: ' + (self.safestat.reason if self.safestat else 'NO_STATUS'))
        if not self.locstat or not self.locstat.healthy or not self.locstat.transform_locked:
            return TriggerResponse(False, 'Localization not calibrated/healthy')
        if not self.transform or self.world is None or now-self.world_stamp > .30:
            return TriggerResponse(False, 'World pose/transform not ready')
        if not self.state.connected or not self.state.armed or self.state.mode != 'GUIDED':
            return TriggerResponse(False, 'Pilot must ARM, take off, and enter GUIDED manually')
        xyz = (self.world.pose.position.x, self.world.pose.position.y,
               self.world.pose.position.z)
        if not inside_box(xyz, self.box):
            return TriggerResponse(False, 'Current position outside safety volume')
        self.index = 0
        self.mode = 'RUNNING'
        self.setpoint = xyz
        self.stable_since = None
        self.hold_since = None
        self.tick_prev = now
        self.state_pub.publish(String(data=self.mode))
        return TriggerResponse(True, 'Mission started: no auto arm / no auto takeoff')

    def stop(self, _request):
        self.mode = 'MANUAL_STOP'
        self.state_pub.publish(String(data=self.mode))
        # Safety supervisor will notice setpoint timeout; mode is not changed here.
        return TriggerResponse(True, 'Mission setpoints stopped; PILOT TAKE CONTROL')

    def tick(self, _event):
        if self.mode not in ('RUNNING', 'COMPLETE'):
            return
        now = rospy.get_time()
        if (not self.safestat or not self.safestat.ready or not self.locstat or
                not self.locstat.healthy or not self.state.armed or
                self.state.mode != 'GUIDED' or self.transform is None or
                self.world is None or now-self.world_stamp > .30):
            self.mode = 'ABORT'
            self.state_pub.publish(String(data=self.mode))
            return
        wp = self.route[self.index if self.mode == 'RUNNING' else -1]
        target = tuple(float(wp[k]) for k in ('x', 'y', 'z'))
        current = (self.world.pose.position.x, self.world.pose.position.y,
                   self.world.pose.position.z)
        if not inside_box(current, self.box):
            self.mode = 'ABORT'
            self.state_pub.publish(String(data=self.mode))
            return
        motion = self.cfg.get('motion', {})
        max_xy = min(float(motion.get('horizontal_speed_mps', .3)),
                     float(self.safety.get('mission', {}).get('max_xy_speed_mps', .4)))
        max_z = min(float(motion.get('vertical_speed_mps', .15)),
                    float(self.safety.get('mission', {}).get('max_z_speed_mps', .20)))
        dt = min(.15, max(0., now-self.tick_prev))
        self.tick_prev = now
        if self.mode == 'RUNNING':
            self.setpoint = bounded_step(self.setpoint, target, dt, max_xy, max_z)
        else:
            self.setpoint = target  # Continue holding final point until pilot takes over
        xy_n = self.transform.apply(self.setpoint[0], self.setpoint[1])
        # Z uses the fixed calibrated navigation offset.
        # Supervisor recomputes safety in world frame before forwarding.
        offset = self.z_offset
        if offset is None or not finite(offset):
            self.mode = 'ABORT'
            return
        target_nav = PoseStamped()
        target_nav.header.stamp = rospy.Time.now()
        target_nav.header.frame_id = 'odom'
        target_nav.pose.position.x, target_nav.pose.position.y = xy_n
        target_nav.pose.position.z = self.setpoint[2] + float(offset)
        theta = self.transform.yaw + math.radians(float(wp.get('yaw_deg', 0.)))
        q = yaw_quaternion(theta)
        target_nav.pose.orientation.x, target_nav.pose.orientation.y = q[0], q[1]
        target_nav.pose.orientation.z, target_nav.pose.orientation.w = q[2], q[3]
        self.target_pub.publish(target_nav)
        arrival = self.cfg.get('arrival', {})
        xy_tol = float(arrival.get('xy_tolerance_m', .15))
        z_tol = float(arrival.get('z_tolerance_m', .08))
        yaw_tol = math.radians(float(arrival.get('yaw_tolerance_deg', 10.)))
        cq = self.world.pose.orientation
        yaw = quaternion_yaw((cq.x, cq.y, cq.z, cq.w))
        if self.mode == 'COMPLETE':
            return
        arrived = (math.hypot(current[0]-target[0], current[1]-target[1]) < xy_tol and
                   abs(current[2]-target[2]) < z_tol and
                   abs(wrap_pi(yaw-math.radians(float(wp.get('yaw_deg', 0.))))) < yaw_tol)
        if arrived:
            if self.stable_since is None:
                self.stable_since = now
            if now-self.stable_since >= float(arrival.get('stable_time_s', 1.)):
                if self.hold_since is None:
                    self.hold_since = now
                if now-self.hold_since >= float(wp.get('hold_s', 2.)):
                    self.index += 1
                    self.stable_since = None
                    self.hold_since = None
                    if self.index == len(self.route):
                        self.mode = 'COMPLETE'
                        self.state_pub.publish(String(data=self.mode))
                        rospy.logwarn('Mission complete. Supervisor will hold final waypoint until pilot override.')
        else:
            self.stable_since = None
            self.hold_since = None


if __name__ == '__main__':
    MissionManager()
    rospy.spin()
