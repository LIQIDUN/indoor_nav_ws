#!/usr/bin/env python3
"""Single FCU motion outlet with a revocable mission lease and guarded exit."""
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

import rosgraph
import rospy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Mavlink, State
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Header, String

from indoor_nav.v0 import ControlLease, Health, decode_ekf_report, load_config, quaternion_yaw


class Watchdog:
    def __init__(self):
        raw = Path(rospy.get_param('~mission_file')).expanduser().read_bytes()
        self.config = load_config(raw.decode())
        self.yaml_sha256 = hashlib.sha256(raw).hexdigest()
        self.health = Health(self.config)
        self.lease = ControlLease()
        self.enabled = bool(rospy.get_param('~enable_control', False)) and not bool(rospy.get_param('~dry_run', True))
        self.received = self.status_time = self.target_time = self.graph_time = None
        self.rx_ros = {}
        self.status, self.target, self.origin = {}, None, None
        self.graph_reason = 'GRAPH_NOT_READY'
        self.previous_ros = self.previous_mono = self.previous_publish = None
        self.max_publish_gap = 0.
        self.exit_result = None
        self.lock = threading.RLock()
        self.closed = threading.Event()
        root = Path(rospy.get_param('~evidence_dir', str(Path.home() / '.ros/v0_nav'))).expanduser()
        directory = root / ('gate-{}-{}-{}'.format(time.strftime('%Y%m%dT%H%M%S'), os.getpid(), time.monotonic_ns()))
        directory.mkdir(parents=True, exist_ok=False)
        self.log = (directory / 'events.jsonl').open('x', buffering=1)
        self.publisher = rospy.Publisher('/v0_nav/watchdog', String, queue_size=1, latch=True)
        self.command_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1) if self.enabled else None
        rospy.Subscriber('/v0_nav/heartbeat', Header, self.heartbeat, queue_size=1)
        rospy.Subscriber('/v0_nav/status', String, self.status_cb, queue_size=1)
        rospy.Subscriber('/v0_nav/target_pose', PoseStamped, self.target_cb, queue_size=1)
        rospy.Subscriber('/v0_nav/origin_pose', PoseStamped, self.origin_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_cb, queue_size=1)
        rospy.Subscriber('/mavros/global_position/gp_origin', GeoPointStamped, self.ekf_origin_cb, queue_size=1)
        rospy.Subscriber('/mavlink/from', Mavlink, self.ekf_cb, queue_size=100)
        rospy.Subscriber('/v0_nav/upstream_healthy', Bool, self.upstream_cb, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        threading.Thread(target=self.graph, daemon=True).start()

    def record(self, event, **values):
        self.log.write(json.dumps(dict(event=event, wall_s=time.time(), ros_s=rospy.get_time(),
                                       mono_s=time.monotonic(), **values)) + '\n')

    def heartbeat(self, message):
        with self.lock:
            if self.command_received('heartbeat'):
                self.received = time.monotonic()

    def command_received(self, name):
        ros_now = rospy.get_time()
        if not self.health.observe_clock(ros_now):
            return False
        self.rx_ros[name] = ros_now
        return True

    def status_cb(self, message):
        with self.lock:
            try:
                result = json.loads(message.data)
                if not isinstance(result, dict):
                    raise ValueError('status must be an object')
                if not self.command_received('status'):
                    return
                self.status = result
                self.status_time = time.monotonic()
            except ValueError:
                self.status = {}

    def target_cb(self, message):
        with self.lock:
            if self.command_received('target'):
                self.target, self.target_time = message, time.monotonic()

    def origin_cb(self, message):
        with self.lock:
            self.origin = message

    def state_cb(self, message):
        with self.lock:
            self.health.update_state(message.connected, message.armed, message.mode,
                                     message.header.stamp.to_sec(), time.monotonic(), rospy.get_time())
            if self.lease.owner is not None and not self.lease.revoked and (not message.connected or not message.armed or message.mode != 'GUIDED'):
                self.lease.revoked = True
                self.lease.reason = 'DISCONNECTED' if not message.connected else 'MANUAL_MODE_EXIT_OR_DISARM'
                self.revoke_action(self.lease.reason, False)

    def pose_cb(self, message):
        with self.lock:
            position, quaternion = message.pose.position, message.pose.orientation
            self.health.update_pose((position.x, position.y, position.z),
                                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                                    message.header.stamp.to_sec(), time.monotonic(), rospy.get_time(), message.header.frame_id)

    def velocity_cb(self, message):
        with self.lock:
            velocity = message.twist.linear
            self.health.update_velocity((velocity.x, velocity.y, velocity.z), message.header.stamp.to_sec(),
                                        time.monotonic(), rospy.get_time(), message.header.frame_id)

    def gps_cb(self, message):
        with self.lock:
            now = time.monotonic()
            if not self.health.receive('gps', message.header.stamp.to_sec(), now, rospy.get_time()):
                return
            self.health.gps_time = now
            self.health.gps_ok = (message.status.status >= 0
                                  and all(math.isfinite(value) for value in (message.latitude, message.longitude, message.altitude)))

    def ekf_origin_cb(self, message):
        with self.lock:
            self.health.update_ekf_origin((message.position.latitude, message.position.longitude, message.position.altitude))

    def ekf_cb(self, message):
        if message.msgid != 193 or (message.sysid, message.compid) != (1, 1) or message.framing_status != Mavlink.FRAMING_OK:
            return
        with self.lock:
            now = time.monotonic()
            if not self.health.receive('ekf', message.header.stamp.to_sec(), now, rospy.get_time()):
                return
            self.health.ekf_time = now
            try:
                self.health.ekf_ok = decode_ekf_report(message.payload64, message.len)['healthy']
            except (ValueError, TypeError, OverflowError):
                self.health.ekf_ok = False

    def upstream_cb(self, message):
        with self.lock:
            self.health.upstream_time, self.health.upstream_ok = time.monotonic(), message.data is True

    def graph(self):
        master = rosgraph.Master(rospy.get_name())
        while not self.closed.is_set() and not rospy.is_shutdown():
            try:
                publishers, _, _ = master.getSystemState()
                reasons = []
                for topic, nodes in publishers:
                    if topic == '/mavros/setpoint_position/local' and set(nodes) - {'/v0_nav_watchdog'}:
                        reasons.append('OTHER_POSITION_PUBLISHER')
                    if topic in ('/mavros/setpoint_raw/local', '/mavros/setpoint_velocity/cmd_vel',
                                 '/mavros/setpoint_velocity/cmd_vel_unstamped', '/mavros/setpoint_raw/attitude') and nodes:
                        reasons.append('OTHER_CONTROL_PUBLISHER')
                    if topic in ('/v0_nav/heartbeat', '/v0_nav/status', '/v0_nav/target_pose', '/v0_nav/origin_pose') and set(nodes) - {'/v0_nav_manager'}:
                        reasons.append('DUPLICATE_MANAGER')
                self.graph_reason, self.graph_time = ','.join(reasons), time.monotonic()
            except Exception as error:
                self.graph_reason = 'ROS_GRAPH_UNAVAILABLE:' + type(error).__name__
            self.closed.wait(.2)

    def fault(self, now, ros_now):
        if not self.health.observe_clock(ros_now):
            return 'ROS_TIME_UNINITIALIZED'
        if self.health.clock_reset_pending:
            self.received = self.status_time = self.target_time = None
            self.status, self.target, self.origin = {}, None, None
            self.rx_ros.clear()
            self.health.clock_reset_pending = False
            return 'ROS_CLOCK_RESET'
        if self.graph_time is None or now - self.graph_time > .6:
            return 'GRAPH_STALE'
        if self.graph_reason:
            return self.graph_reason
        timeout = self.config['msg_timeout']['command_timeout']
        for name, received, reason in (('heartbeat', self.received, 'MISSION_HEARTBEAT_TIMEOUT'),
                                       ('status', self.status_time, 'MISSION_STATUS_TIMEOUT')):
            ros_received = self.rx_ros.get(name)
            if received is None or ros_received is None or not 0 <= now - received <= timeout or not 0 <= ros_now - ros_received <= timeout:
                return reason
        if self.status.get('yaml_sha256') != self.yaml_sha256 or self.status.get('control_enabled') is not self.enabled:
            return 'MANAGER_GATE_CONFIG_MISMATCH'
        if self.previous_mono is not None and (ros_now < self.previous_ros or abs((ros_now-self.previous_ros)-(now-self.previous_mono)) > self.config['safety']['clock_tolerance_s'] or now-self.previous_mono > self.config['safety']['cycle_timeout_s']):
            return 'GATE_CLOCK_OR_CYCLE_TIMEOUT'
        if not self.health.state[0]:
            return 'DISCONNECTED'
        if self.health.receipt_expired('state', now, ros_now):
            return 'STATE_RX_TIMEOUT'
        if self.status.get('state') not in ('HOLD_INITIAL', 'EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD'):
            return ''
        if self.health.ekf_origin is None:
            return 'EKF_ORIGIN_NOT_OBSERVED'
        reason = self.health.reason(now, ros_now)
        if reason:
            return reason
        if (self.target_time is None or self.rx_ros.get('target') is None or
                not 0 <= now - self.target_time <= timeout or not 0 <= ros_now - self.rx_ros['target'] <= timeout or
                self.target is None or self.origin is None):
            return 'TARGET_OR_ORIGIN_STALE'
        for message in (self.target, self.origin):
            position, quaternion = message.pose.position, message.pose.orientation
            try:
                quaternion_yaw((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
            except ValueError:
                return 'INVALID_TARGET_QUATERNION'
            if message.header.frame_id != 'map' or not all(math.isfinite(value) for value in (position.x, position.y, position.z)):
                return 'INVALID_TARGET_FRAME_OR_POSITION'
        origin = self.origin.pose.position
        for position in (self.target.pose.position, self.health.pose[0]):
            values = (position.x, position.y, position.z) if hasattr(position, 'x') else position
            if math.hypot(values[0]-origin.x, values[1]-origin.y) > self.config['safety']['max_offset_xy_m'] or abs(values[2]-origin.z) > self.config['safety']['max_offset_z_m']:
                return 'GATE_ORIGIN_FENCE'
        return ''

    def revoke_action(self, reason, can_brake):
        self.exit_result = {'reason': reason, 'requested': False, 'mode_sent': False}
        if self.enabled and self.config['control']['exit_action'] == 'brake_if_position_healthy' and can_brake:
            try:
                rospy.wait_for_service('/mavros/set_mode', timeout=.3)
                reply = rospy.ServiceProxy('/mavros/set_mode', SetMode)(0, 'BRAKE')
                self.exit_result.update(requested=True, mode_sent=reply.mode_sent)
            except (rospy.ROSException, rospy.ServiceException) as error:
                self.exit_result['error'] = str(error)
        self.record('LEASE_REVOKED', **self.exit_result, warning='mode_sent is not confirmed braking; observe FCU and RC takeover')

    def brake_eligible(self, now, ros_now, reason):
        if reason not in ('', 'MISSION_HEARTBEAT_TIMEOUT', 'MISSION_STATUS_TIMEOUT',
                          'TARGET_OR_ORIGIN_STALE'):
            return False
        if self.health.reason(now, ros_now) or self.health.ekf_origin is None:
            return False
        age = self.status.get('limit_age_s')
        return (self.status.get('firmware') == [4, 5, 7] and type(age) in (int, float)
                and math.isfinite(age) and age >= 0 and self.status_time is not None
                and now-self.status_time+age < 5.)

    def run(self):
        period = 1. / self.config['control']['setpoint_rate_hz']
        while not rospy.is_shutdown() and not self.closed.is_set():
            began = time.monotonic()
            with self.lock:
                now, ros_now = time.monotonic(), rospy.get_time()
                reason = self.fault(now, ros_now)
                can_brake = self.brake_eligible(now, ros_now, reason)
                previously_revoked = self.lease.revoked
                forward, brake = self.lease.step(self.status.get('state'), self.status.get('mission_id'),
                                                self.health.state[1], self.health.state[2], reason, can_brake)
                if self.lease.revoked and not previously_revoked:
                    self.revoke_action(self.lease.reason, brake)
                if forward and self.command_pub is not None:
                    self.command_pub.publish(self.target)
                    if self.previous_publish is not None:
                        self.max_publish_gap = max(self.max_publish_gap, now-self.previous_publish)
                    self.previous_publish = now
                output_reason = self.lease.reason if self.lease.revoked else reason
                self.publisher.publish(String(data=json.dumps({'healthy': not output_reason, 'reason': output_reason,
                    'owner': self.lease.owner, 'revoked': self.lease.revoked, 'exit_result': self.exit_result,
                    'mode': self.health.state[2], 'max_publish_gap_s': self.max_publish_gap,
                    'rx_ros_s': dict(self.health.rx_ros), 'header_stamps_s': dict(self.health.header_stamps),
                    'stamp_diagnostics': dict(self.health.stamp_diagnostics),
                    'speed': self.health.current_speed(), 'speed_peaks': self.health.speed_peaks,
                    'target_rx_ros_s': self.rx_ros.get('target'),
                    'target_header_stamp_s': None if self.target is None else self.target.header.stamp.to_sec()})))
                self.previous_ros, self.previous_mono = ros_now, now
            self.closed.wait(max(0., period-(time.monotonic()-began)))

    def shutdown(self):
        self.closed.set()
        with self.lock:
            if not self.log.closed:
                self.record('GATE_SHUTDOWN', warning='no guaranteed FCU action if the sole outlet exits; RC/FCU failsafe required')
                self.log.close()


if __name__ == '__main__':
    rospy.init_node('v0_nav_watchdog')
    Watchdog().run()
