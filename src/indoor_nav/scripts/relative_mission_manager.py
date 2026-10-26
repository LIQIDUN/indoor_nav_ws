#!/usr/bin/env python3
"""V0 only: one MAVROS position publisher, no arming, mode or parameter writes."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus, Mavlink, State
from mavros_msgs.srv import ParamGet, VehicleInfoGet
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger, TriggerResponse

from indoor_nav.v0 import Health, Mission, decode_ekf_report, load_config, verify_fcu_limits, yaw_quaternion


class RelativeMissionManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.started = time.monotonic()
        self.path = Path(rospy.get_param('~mission_file')).expanduser().resolve()
        raw = self.path.read_bytes()
        self.config = load_config(raw.decode('utf-8'))
        self.health = Health(self.config)
        self.mission = Mission(self.config)
        self.control = bool(rospy.get_param('~enable_control', False)) and not bool(rospy.get_param('~dry_run', True))
        self.watchdog_time = None
        self.watchdog_fault = 'WATCHDOG_NOT_READY'
        self.limit_time = None
        self.limit_error = 'FCU_LIMITS_NOT_VERIFIED'
        self.snapshot = {}
        self.ekf_report = {}
        self.sequence = 0
        self.event_index = 0
        self.previous_publish = None
        self.max_publish_gap = 0.
        self.max_pose_age = 0.
        workspace = Path(__file__).resolve().parents[3]
        def git(*arguments):
            return subprocess.check_output(['git', '-C', str(workspace), *arguments], text=True, timeout=3).strip()
        try:
            commit, dirty = git('rev-parse', 'HEAD'), bool(git('status', '--porcelain'))
        except (OSError, subprocess.SubprocessError):
            commit, dirty = 'UNKNOWN', True
        self.provenance = {
            'yaml_path': str(self.path), 'yaml_sha256': hashlib.sha256(raw).hexdigest(),
            'git_commit': commit, 'git_dirty': dirty,
            'navigation_source': self.config['navigation']['source'],
            'control_enabled': self.control, 'setpoint_topic': '/mavros/setpoint_position/local',
            'pid': os.getpid(), 'ros_master': os.environ.get('ROS_MASTER_URI', ''),
        }
        self.provenance['code_sha256'] = {
            str(path.relative_to(workspace)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (workspace / 'src/indoor_nav/src/indoor_nav/v0.py',
                         workspace / 'src/indoor_nav/scripts/relative_mission_manager.py',
                         workspace / 'src/indoor_nav/scripts/v0_nav_health.py')}
        root = Path(rospy.get_param('~evidence_dir', str(Path.home() / '.ros/v0_nav'))).expanduser()
        self.directory = root / ('{}-{}-{}'.format(time.strftime('%Y%m%dT%H%M%S'), os.getpid(), time.monotonic_ns()))
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'loaded.yaml').write_bytes(raw)
        (self.directory / 'provenance.json').write_text(json.dumps(self.provenance, indent=2))
        self.log = (self.directory / 'events.jsonl').open('x', buffering=1)
        self.status_pub = rospy.Publisher('/v0_nav/status', String, queue_size=2, latch=True)
        self.event_pub = rospy.Publisher('/v0_nav/event', String, queue_size=30)
        self.origin_pub = rospy.Publisher('/v0_nav/origin_pose', PoseStamped, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher('/v0_nav/target_pose', PoseStamped, queue_size=2)
        self.heartbeat_pub = rospy.Publisher('/v0_nav/heartbeat', Header, queue_size=1)
        self.command_pub = (rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
                            if self.control else None)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=5)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_cb, queue_size=1)
        rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self.ekf_cb, queue_size=1)
        rospy.Subscriber('/mavlink/from', Mavlink, self.ekf_report_cb, queue_size=100)
        rospy.Subscriber('/v0_nav/upstream_healthy', Bool, self.upstream_cb, queue_size=1)
        rospy.Subscriber('/v0_nav/watchdog', String, self.watchdog_cb, queue_size=1)
        rospy.Service('/v0_nav/start', Trigger, self.start)
        rospy.Service('/v0_nav/abort', Trigger, self.abort)
        rospy.Service('/v0_nav/stop', Trigger, self.stop)
        rospy.Service('/v0_nav/reset', Trigger, self.reset)
        self.record('BOOT', provenance=self.provenance)
        rospy.on_shutdown(self.shutdown)
        threading.Thread(target=self.read_limits, daemon=True).start()

    def record(self, event, **details):
        self.sequence += 1
        data = {'event': event, 'seq': self.sequence, 'mission_id': self.config['mission_id'],
                'wall_s': time.time(), 'ros_s': rospy.get_time(),
                'mono_delta_s': time.monotonic() - self.started, **details}
        encoded = json.dumps(data, allow_nan=False)
        self.log.write(encoded + '\n')
        self.event_pub.publish(String(data=encoded))

    def state_cb(self, message):
        with self.lock:
            self.health.state = (message.connected, message.armed, message.mode)
            self.health.state_time = time.monotonic()
            if self.mission.state in Mission.ACTIVE and (not message.connected or not message.armed or message.mode != 'GUIDED'):
                self.mission.abort(time.monotonic(), 'FCU_STATE_CHANGED')

    def pose_cb(self, message):
        with self.lock:
            position, quaternion = message.pose.position, message.pose.orientation
            self.health.update_pose((position.x, position.y, position.z),
                                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                                    message.header.stamp.to_sec(), time.monotonic(),
                                    rospy.get_time(), message.header.frame_id)

    def velocity_cb(self, message):
        with self.lock:
            velocity = message.twist.linear
            self.health.update_velocity((velocity.x, velocity.y, velocity.z),
                                        message.header.stamp.to_sec(), time.monotonic(),
                                        rospy.get_time(), message.header.frame_id)

    def valid_stamp(self, stamp, timeout):
        age = rospy.get_time() - stamp.to_sec()
        return stamp.to_sec() > 0 and -.1 <= age <= timeout

    def gps_cb(self, message):
        with self.lock:
            self.health.gps_time = time.monotonic()
            self.health.gps_ok = (message.status.status >= 0 and
                                  all(math.isfinite(value) for value in (message.latitude, message.longitude, message.altitude)) and
                                  self.valid_stamp(message.header.stamp, self.config['safety']['gps_timeout_s']))

    def ekf_cb(self, message):
        with self.lock:
            self.health.ekf_time = time.monotonic()
            self.health.ekf_ok = (message.attitude_status_flag and message.velocity_horiz_status_flag and
                                 message.velocity_vert_status_flag and message.pos_horiz_abs_status_flag and
                                 (message.pos_vert_abs_status_flag or message.pos_vert_agl_status_flag) and
                                 not message.const_pos_mode_status_flag and not message.gps_glitch_status_flag and
                                 not message.accel_error_status_flag and
                                 self.valid_stamp(message.header.stamp, self.config['safety']['ekf_timeout_s']))

    def upstream_cb(self, message):
        with self.lock:
            self.health.upstream_time, self.health.upstream_ok = time.monotonic(), message.data

    def ekf_report_cb(self, message):
        if message.msgid != 193 or (message.sysid, message.compid) != (1, 1) or message.framing_status != Mavlink.FRAMING_OK:
            return
        with self.lock:
            try:
                self.ekf_report = decode_ekf_report(message.payload64, message.len)
                self.health.ekf_ok = self.ekf_report['healthy'] and self.valid_stamp(message.header.stamp, self.config['safety']['ekf_timeout_s'])
                self.health.ekf_time = time.monotonic()
            except (ValueError, TypeError, OverflowError):
                self.health.ekf_ok = False

    def watchdog_cb(self, message):
        with self.lock:
            try:
                result = json.loads(message.data)
                self.watchdog_fault = '' if result['healthy'] is True else str(result['reason'])
                self.watchdog_time = time.monotonic()
            except (ValueError, KeyError, TypeError):
                self.watchdog_fault = 'INVALID_WATCHDOG'

    def read_limits(self):
        names = ('WPNAV_SPEED', 'WPNAV_SPEED_UP', 'WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z',
                 'GUID_OPTIONS', 'AHRS_EKF_TYPE', 'EK3_ENABLE', 'EK3_SRC1_POSXY')
        while not self.closed.is_set() and not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/mavros/vehicle_info_get', timeout=1.)
                info = rospy.ServiceProxy('/mavros/vehicle_info_get', VehicleInfoGet)(0, 0, False)
                if not info.success or len(info.vehicles) != 1:
                    raise ValueError('vehicle identity unavailable')
                vehicle = info.vehicles[0]
                if not vehicle.available_info & 2 or vehicle.autopilot != 3 or vehicle.type not in (2, 3, 13, 14, 15, 29):
                    raise ValueError('not identified ArduCopter')
                encoded = vehicle.flight_sw_version
                version = ((encoded >> 24) & 255, (encoded >> 16) & 255, (encoded >> 8) & 255)
                rospy.wait_for_service('/mavros/param/get', timeout=1.)
                get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)
                parameters = {}
                for name in names:
                    reply = get_param(name)
                    if not reply.success:
                        raise ValueError('missing FCU parameter: ' + name)
                    parameters[name] = reply.value.integer if reply.value.integer != 0 else reply.value.real
                verify_fcu_limits(version, parameters, self.config)
                if self.config['navigation']['source'] == 'fc_ekf_sitl_gps':
                    if not rospy.get_param('/v0_sitl/isolated', False) or rospy.get_param('/mavros/fcu_url', '') != 'udp://127.0.0.1:14672@':
                        raise ValueError('SITL-only source requires isolated test route')
                    if not get_param('SIM_SPEEDUP').success:
                        raise ValueError('SIM_SPEEDUP missing')
                snapshot = {'firmware': list(version), 'custom_version': vehicle.flight_custom_version,
                            'parameters': parameters}
                with self.lock:
                    self.snapshot, self.limit_time, self.limit_error = snapshot, time.monotonic(), ''
            except (ValueError, rospy.ROSException, rospy.ServiceException, OSError) as error:
                with self.lock:
                    self.limit_error = str(error)
            self.closed.wait(2.)

    def external_reason(self, now):
        if self.watchdog_time is None or now - self.watchdog_time > self.config['safety']['watchdog_timeout_s']:
            return 'WATCHDOG_STALE'
        if self.watchdog_fault:
            return self.watchdog_fault
        if self.limit_error or self.limit_time is None or now - self.limit_time > 5.:
            return self.limit_error or 'FCU_LIMIT_SNAPSHOT_STALE'
        return ''

    def pose_message(self, target):
        message = PoseStamped()
        message.header.stamp, message.header.frame_id = rospy.Time.now(), 'map'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = target
        quaternion = yaw_quaternion(self.mission.yaw0)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = quaternion
        return message

    def start(self, _request):
        with self.lock:
            now = time.monotonic()
            success, reason = self.mission.start(now, rospy.get_time(), self.health, self.external_reason(now))
            self.record('START_ACCEPTED' if success else 'START_REJECTED', reason=reason, fcu=self.snapshot)
            if success:
                self.origin_pub.publish(self.pose_message(self.mission.origin))
            return TriggerResponse(success, reason)

    def abort(self, _request):
        with self.lock:
            self.mission.abort(time.monotonic(), 'OPERATOR_ABORT_NO_FCU_MODE_CHANGE')
            self.record('OPERATOR_ABORT', warning='stopping output does not cancel FCU last target; RC takeover required')
            return TriggerResponse(True, 'ABORT_LATCHED; RC TAKEOVER; no landing/mode command sent')

    def stop(self, _request):
        with self.lock:
            if self.mission.state != 'FINISH_HOLD':
                return TriggerResponse(False, 'stop only after finish; use abort for active task')
            self.mission.target = None
            self.mission.transition('STOPPED', time.monotonic(), 'OPERATOR_STOP_RC_TAKEOVER')
            return TriggerResponse(True, 'output stopped; FCU may retain final target')

    def reset(self, _request):
        with self.lock:
            if self.health.state[1] or not self.health.state[0] or self.mission.state in Mission.ACTIVE:
                return TriggerResponse(False, 'reset requires connected DISARMED vehicle and inactive mission')
            self.mission = Mission(self.config)
            self.health = Health(self.config)
            self.event_index = 0
            self.record('EXPLICIT_RESET', authorization='a new start is still required')
            return TriggerResponse(True, 'IDLE; wait for new stable telemetry and explicit start')

    def run(self):
        period = 1. / self.config['control']['setpoint_rate_hz']
        while not rospy.is_shutdown() and not self.closed.is_set():
            began = time.monotonic()
            with self.lock:
                now, ros_now = time.monotonic(), rospy.get_time()
                self.mission.tick(now, ros_now, self.health, self.external_reason(now))
                for event in self.mission.events[self.event_index:]:
                    self.record('TRANSITION', **event)
                self.event_index = len(self.mission.events)
                if self.mission.target is not None and self.mission.state in Mission.ACTIVE:
                    message = self.pose_message(self.mission.target)
                    self.target_pub.publish(message)
                    if self.command_pub is not None:
                        self.command_pub.publish(message)
                        if self.previous_publish is not None:
                            self.max_publish_gap = max(self.max_publish_gap, now - self.previous_publish)
                        self.previous_publish = now
                if self.health.pose_time is not None:
                    self.max_pose_age = max(self.max_pose_age, now - self.health.pose_time)
                self.heartbeat_pub.publish(Header(seq=self.sequence, stamp=rospy.Time.now(), frame_id='v0_nav'))
                status = {'state': self.mission.state, 'mission_id': self.config['mission_id'],
                          'wp': self.mission.index, 'reason': self.mission.reason,
                          'health': self.health.reason(now, ros_now), 'gate': self.external_reason(now),
                          'control_enabled': self.control, 'evidence_dir': str(self.directory),
                          'pose_age_s': None if self.health.pose_time is None else now - self.health.pose_time,
                          'max_publish_gap_s': self.max_publish_gap}
                status['ekf_report'] = self.ekf_report
                self.status_pub.publish(String(data=json.dumps(status)))
            self.closed.wait(max(0., period - (time.monotonic() - began)))

    def shutdown(self):
        self.closed.set()
        with self.lock:
            self.mission.target = None
            if not self.log.closed:
                self.record('SHUTDOWN', state=self.mission.state, max_publish_gap_s=self.max_publish_gap,
                            max_pose_age_s=self.max_pose_age,
                            warning='no guaranteed braking; last FCU target may persist')
                self.log.close()


if __name__ == '__main__':
    rospy.init_node('v0_nav_manager')
    RelativeMissionManager().run()
