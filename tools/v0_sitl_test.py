#!/usr/bin/env python3
"""Explicit, identity-gated simulation flight driver. Never imported by launches."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

import psutil
import rosgraph
import rosnode
import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import HomePosition, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, ParamGet, SetMode
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Trigger

from indoor_nav.v0 import quaternion_yaw

ROOT = Path(__file__).resolve().parents[1]
MASTER = 'http://127.0.0.1:11421'


def test_waypoints(case):
    if case in ('single', 'yaw', 'distance_short', 'distance_long', 'finish_stop'):
        distance = .15 if case == 'distance_short' else .5 if case == 'distance_long' else .25
        return [dict(id='forward', x=distance, y=0., z=0., hold_s=1.)]
    if case == 'altitude':
        return [dict(id='up', x=0., y=0., z=.1, hold_s=1.),
                dict(id='return', x=0., y=0., z=0., hold_s=1.)]
    return [dict(id='forward', x=.25, y=0., z=0., hold_s=1.),
            dict(id='right', x=.25, y=.25, z=0., hold_s=1.),
            dict(id='back', x=0., y=.25, z=.1, hold_s=1.),
            dict(id='return', x=0., y=0., z=0., hold_s=1.)]


def active_bag_bytes(directory, process, pattern='*.bag.active'):
    if process.poll() is not None:
        raise RuntimeError('recorder exited before live recording check')
    return sum(path.stat().st_size for path in directory.glob(pattern))


def identity():
    if os.environ.get('ROS_MASTER_URI') != MASTER or not rospy.get_param('/v0_sitl/isolated', False):
        raise RuntimeError('not the isolated V0 ROS master')
    if rospy.get_param('/mavros/fcu_url') != 'udp://127.0.0.1:14672@':
        raise RuntimeError('wrong FCU URL')
    runtime = Path(rospy.get_param('/v0_sitl/runtime')).resolve()
    if runtime.parent != ROOT / '.sitl_runtime/v0':
        raise RuntimeError('wrong SITL runtime')
    sitl = psutil.Process(int((runtime / 'arducopter.pid').read_text()))
    expected = Path.home() / 'github/ardupilot/build/sitl/bin/arducopter'
    if Path(sitl.exe()).resolve() != expected.resolve() or Path(sitl.cwd()).resolve() != runtime or '--model' not in sitl.cmdline() or 'gazebo' not in sitl.cmdline():
        raise RuntimeError('SITL process identity mismatch')
    proxy = psutil.Process(int((runtime / 'mavproxy.pid').read_text()))
    if '--master=tcp:127.0.0.1:5760' not in proxy.cmdline() or '--out=127.0.0.1:14672' not in proxy.cmdline():
        raise RuntimeError('proxy route mismatch')
    if sum(arg.startswith('--out') for arg in proxy.cmdline()) != 1:
        raise RuntimeError('multiple proxy routes')
    if not any(connection.raddr and connection.raddr.ip == '127.0.0.1' and connection.raddr.port == 5760
               for connection in proxy.connections(kind='inet')):
        raise RuntimeError('proxy is not connected to verified SITL')
    master = rosgraph.Master(rospy.get_name())
    owner = master.lookupService('/mavros/cmd/arming')
    if '127.0.0.1' not in owner:
        raise RuntimeError('remote arming service')
    node_uri = master.lookupNode('/mavros')
    code, _, pid = xmlrpc.client.ServerProxy(node_uri).getPid(rospy.get_name())
    if code != 1 or 'mavros_node' not in psutil.Process(pid).name():
        raise RuntimeError('MAVROS identity mismatch')
    if not rospy.ServiceProxy('/mavros/param/get', ParamGet)('SIM_SPEEDUP').success:
        raise RuntimeError('not SIM firmware')
    return {'runtime': str(runtime), 'sitl_pid': sitl.pid, 'proxy_pid': proxy.pid, 'mavros_pid': pid}


def wait_identity(timeout=75):
    deadline = time.monotonic() + timeout
    while True:
        try:
            return identity()
        except (RuntimeError, rospy.ServiceException, rosgraph.MasterError, rosgraph.MasterException):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.5)


class FlightTest:
    def __init__(self, args):
        self.args = args
        self.directory = ROOT / 'docs/v0/evidence' / (time.strftime('%Y%m%dT%H%M%S') + '-' + args.case)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.samples = (self.directory / 'telemetry.jsonl').open('x', buffering=1)
        self.event_file = (self.directory / 'events.jsonl').open('x', buffering=1)
        self.pose = self.velocity = self.truth = self.target = self.echo = None
        self.origin = None
        self.state = State()
        self.status = {}
        self.events = []
        self.bag = self.navigation = None
        self.mock = None
        self.lock = threading.RLock()
        self.sample_rows = []
        self.arrival_rows = []
        self.graph = []
        self.watchdog = {}
        self.armed_by_test = False
        self.suspended_proxy = None
        self.truth_origin = self.yaw_delta = None
        self.max_rss = 0
        self.cpu_samples = []
        self.phase = 'created'
        self.suspended_manager = None
        self.replacement_mavros = None
        self.auto_recorder = None
        self.home = None
        self.pose_fault = False
        self.pose_forwarder = rospy.Publisher('/v0_sitl/pose_fault', PoseStamped, queue_size=1) if args.case == 'pose_loss' else None
        self.process = psutil.Process()
        self.receptions = []
        self.last_receptions = {}
        rospy.Subscriber('/mavros/state', State, lambda message: self.receive('state', message), queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, lambda message: setattr(self, 'home', message), queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, lambda message: self.receive('velocity', message), queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_cb, queue_size=1)
        rospy.Subscriber('/v0_nav/status', String, lambda message: setattr(self, 'status', json.loads(message.data)), queue_size=10)
        rospy.Subscriber('/v0_nav/event', String, self.event_cb, queue_size=100)
        rospy.Subscriber('/v0_nav/watchdog', String, lambda message: setattr(self, 'watchdog', json.loads(message.data)), queue_size=10)
        rospy.Subscriber('/v0_nav/target_pose', PoseStamped, lambda message: setattr(self, 'target', message), queue_size=1)
        rospy.Subscriber('/v0_nav/origin_pose', PoseStamped, lambda message: setattr(self, 'origin', message), queue_size=1)
        rospy.Subscriber('/mavros/setpoint_raw/target_local', PositionTarget, lambda message: setattr(self, 'echo', message), queue_size=1)

    def receive(self, stream, message):
        mono, ros_now = time.monotonic(), rospy.get_time()
        with self.lock:
            previous = self.last_receptions.get(stream)
            self.receptions.append(dict(stream=stream, ros_rx_s=ros_now, mono_rx_s=mono,
                                        header_s=message.header.stamp.to_sec(), frame=message.header.frame_id,
                                        interval_s=mono-previous if previous is not None else None))
            self.last_receptions[stream] = mono
            setattr(self, stream, message)

    def pose_cb(self, message):
        self.receive('pose', message)
        if self.pose_forwarder is not None and not self.pose_fault:
            self.pose_forwarder.publish(message)

    def truth_cb(self, message):
        if 'iris' in message.name:
            index = message.name.index('iris')
            self.truth = (message.pose[index], message.twist[index])

    def event_cb(self, message):
        event = json.loads(message.data)
        self.events.append(event)
        self.event_file.write(json.dumps(event) + '\n')
        if event.get('state') == 'DWELL':
            row = self.row()
            row['wp'] = event['wp']
            self.arrival_rows.append(row)

    @staticmethod
    def xyz(position):
        return [position.x, position.y, position.z]

    @staticmethod
    def yaw(quaternion):
        return quaternion_yaw((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

    def row(self):
        result = {'wall_s': time.time(), 'ros_s': rospy.get_time(), 'mono_s': time.monotonic(),
                  'mode': self.state.mode, 'armed': self.state.armed,
                  'status': self.status, 'phase': self.phase, 'watchdog': self.watchdog}
        if self.home:
            result['home_altitude_m'] = self.home.geo.altitude
        if self.pose:
            result['pose'] = self.xyz(self.pose.pose.position)
            result['yaw'] = self.yaw(self.pose.pose.orientation)
            result['pose_age_s'] = rospy.get_time() - self.pose.header.stamp.to_sec()
        if self.velocity:
            result['velocity'] = self.xyz(self.velocity.twist.linear)
        if self.truth:
            result['truth'] = self.xyz(self.truth[0].position)
            result['truth_velocity'] = self.xyz(self.truth[1].linear)
            result['truth_yaw'] = self.yaw(self.truth[0].orientation)
        if self.target:
            result['target'] = self.xyz(self.target.pose.position)
        if self.echo:
            result['echo'] = self.xyz(self.echo.position)
        if self.origin and self.truth_origin is not None and self.target:
            delta = [result['target'][index] - self.xyz(self.origin.pose.position)[index] for index in range(3)]
            result['expected_truth'] = [self.truth_origin[0] + delta[0]*math.cos(self.yaw_delta) - delta[1]*math.sin(self.yaw_delta),
                                        self.truth_origin[1] + delta[0]*math.sin(self.yaw_delta) + delta[1]*math.cos(self.yaw_delta),
                                        self.truth_origin[2] + delta[2]]
        return result

    def wait(self, condition, timeout, label):
        self.phase = label
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if condition():
                return
            if self.status.get('state') == 'ABORT_LATCHED' and (label in ('mission complete', 'hover', 'FCU limit readiness') or label.startswith('execution before')):
                raise RuntimeError('navigation aborted: ' + json.dumps(self.status))
            self.sample()
            time.sleep(.1)
        raise RuntimeError('timeout: {} state={} nav={}'.format(label, self.state, self.status))

    def sample(self):
        row = self.row()
        self.samples.write(json.dumps(row) + '\n')
        self.sample_rows.append(row)
        if self.navigation:
            try:
                processes = [psutil.Process(self.navigation.pid), *psutil.Process(self.navigation.pid).children(recursive=True)]
                self.max_rss = max(self.max_rss, sum(process.memory_info().rss for process in processes))
            except psutil.Error:
                pass
        self.cpu_samples.append(psutil.cpu_percent())

    def mode(self, name):
        identity()
        response = rospy.ServiceProxy('/mavros/set_mode', SetMode)(0, name)
        if not response.mode_sent:
            raise RuntimeError('mode rejected: ' + name)
        self.wait(lambda: self.state.mode == name, 8, 'manual takeover' if name == 'ALT_HOLD' else 'mode ' + name)

    def execute(self):
        self.wait(lambda: self.state.connected and self.pose and self.truth, 60, 'telemetry')
        verified = wait_identity()
        marker = Path(verified['runtime']) / 'flight_claim.json'
        with marker.open('x') as stream:
            json.dump({'evidence': str(self.directory), 'wall_s': time.time()}, stream)
        inputs = [ROOT / name for name in ('src/indoor_nav/src/indoor_nav/v0.py',
                  'src/indoor_nav/scripts/relative_mission_manager.py', 'src/indoor_nav/scripts/v0_nav_health.py',
                  'src/indoor_nav/launch/indoor_nav_v0.launch', 'src/indoor_nav/config/v0_sitl_gps.parm',
                  'tools/v0_sitl_test.py', 'tools/v0_sitl_instance.sh')]
        self.inputs = {'git_commit': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                      'git_dirty': bool(subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True).strip()),
                      'sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs},
                      'runtime': verified['runtime'], 'original_hover_gate': {'truth_z_gt_m': .85, 'speed_norm_lt_mps': .06, 'timeout_s': 35}}
        template = self.args.mission_file.expanduser().resolve()
        raw_template = template.read_bytes()
        self.inputs['mission_template'] = {'path': str(template), 'sha256': hashlib.sha256(raw_template).hexdigest()}
        config = yaml.safe_load(raw_template.decode())
        config['navigation']['source'] = 'fc_ekf_sitl_gps'
        config['control']['exit_action'] = 'brake_if_position_healthy'
        config['mission_id'] = self.directory.name
        config['waypoints'] = test_waypoints(self.args.case)
        mission_path = self.directory / 'mission.yaml'
        mission_path.write_text(yaml.safe_dump(config, sort_keys=False))
        self.inputs['mission_yaml_sha256'] = hashlib.sha256(mission_path.read_bytes()).hexdigest()
        (self.directory / 'inputs.json').write_text(json.dumps(self.inputs, indent=2))
        bag_topics = ['/mavros/state', '/mavros/local_position/pose', '/mavros/local_position/velocity_local',
                      '/mavros/global_position/raw/fix', '/mavros/global_position/gp_origin', '/mavros/home_position/home', '/mavros/estimator_status', '/mavlink/from',
                      '/mavros/setpoint_position/local', '/mavros/setpoint_raw/target_local',
                      '/v0_nav/status', '/v0_nav/event', '/v0_nav/target_pose', '/v0_nav/origin_pose',
                      '/v0_nav/heartbeat', '/v0_nav/watchdog', '/sound_sensor/audio_raw', '/motor_mic/audio_raw', '/vibration_data']
        self.bag = subprocess.Popen(['rosbag', 'record', '-O', str(self.directory / 'flight.bag'), *bag_topics],
                                    stdout=(self.directory / 'rosbag.txt').open('w'), stderr=subprocess.STDOUT)
        if self.args.case in ('f3', 'f3_auto_bag'):
            self.mock = subprocess.Popen(['python3', str(ROOT / 'tools/v0_mock_acquisition.py')],
                                         stdout=(self.directory / 'mock.txt').open('w'), stderr=subprocess.STDOUT)
        if self.args.case == 'f3_auto_bag':
            source = Path.home() / 'github/onboard_ros_sensor/ros_sensor/src/rs485_driver_sound/scripts/auto_bag_recorder.py'
            identity()
            self.inputs['acquisition'] = {'path': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                                         'kind': 'read-only local recorder source with synthetic sensor topics; no hardware driver'}
            (self.directory / 'inputs.json').write_text(json.dumps(self.inputs, indent=2))
            self.auto_recorder = subprocess.Popen(['python3', str(source), '__name:=v0_sitl_auto_recorder',
                    '_save_dir:=' + str(self.directory / 'auto'), '_bag_prefix:=isolated',
                    '_topics:=/mavros/state /sound_sensor/audio_raw /motor_mic/audio_raw /v0_nav/status /v0_nav/watchdog'],
                    stdout=(self.directory / 'auto_recorder.txt').open('w'), stderr=subprocess.STDOUT)
        self.navigation = subprocess.Popen(['roslaunch', 'indoor_nav', 'indoor_nav_v0.launch',
                                            'mission_file:=' + str(mission_path),
                                            'enable_control:=' + ('false' if self.args.case == 'dry_run' else 'true'),
                                            'dry_run:=' + ('true' if self.args.case == 'dry_run' else 'false'),
                                            'pose_topic:=' + ('/v0_sitl/pose_fault' if self.args.case == 'pose_loss' else '/mavros/local_position/pose'),
                                            'evidence_dir:=' + str(self.directory / 'navigation')],
                                           stdout=(self.directory / 'navigation.txt').open('w'), stderr=subprocess.STDOUT)
        self.wait(lambda: 'gate' in self.status and not self.status['gate'], 70, 'FCU limit readiness')
        parameters = {}
        for name in ('GPS_TYPE', 'GPS_TYPE2', 'EK3_SRC1_POSXY', 'EK3_SRC1_VELXY', 'EK3_SRC1_POSZ',
                     'EK3_SRC1_VELZ', 'EK3_SRC1_YAW', 'VISO_TYPE', 'FS_GCS_ENABLE', 'FS_EKF_ACTION',
                     'FS_THR_ENABLE', 'GUID_TIMEOUT', 'WPNAV_SPEED', 'WPNAV_SPEED_UP', 'WPNAV_SPEED_DN',
                     'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'GUID_OPTIONS'):
            reply = rospy.ServiceProxy('/mavros/param/get', ParamGet)(name)
            if not reply.success:
                raise RuntimeError('missing test parameter snapshot: ' + name)
            parameters[name] = reply.value.integer if reply.value.integer else reply.value.real
        self.inputs['fcu_parameters'] = parameters
        (self.directory / 'inputs.json').write_text(json.dumps(self.inputs, indent=2))
        if self.state.armed:
            raise RuntimeError('vehicle already armed; refusing ownership')
        if self.args.case == 'dry_run':
            with (self.directory / 'readonly_smoke.json').open('x') as output:
                subprocess.run(['python3', str(ROOT / 'tools/test_v0_relative_waypoints.py'), '--ros-smoke'],
                               stdout=output, stderr=subprocess.STDOUT, check=True, timeout=15)
            return {'result': 'PASS', 'case': 'dry_run', 'identity': verified, 'armed': False,
                    'graph': rosgraph.Master(rospy.get_name()).getSystemState(), 'evidence': str(self.directory)}
        self.mode('GUIDED')
        identity()
        arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for attempt in range(15):
            if arm(True).success:
                self.armed_by_test = True
                break
            time.sleep(2)
        if not self.armed_by_test:
            raise RuntimeError('prearm checks refused; no bypass')
        self.wait(lambda: self.state.armed, 8, 'ARM feedback')
        takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)(0., 0., 0., 0., 1.)
        if not takeoff.success:
            raise RuntimeError('takeoff rejected')
        self.wait(lambda: self.truth[0].position.z > .85 and self.velocity and
                  math.sqrt(sum(value**2 for value in self.xyz(self.velocity.twist.linear))) < .06, 35, 'hover')
        self.preparation = self.row()
        if self.args.case in ('multi', 'yaw'):
            response = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)(False, 115, 0, 90, 20, 1, 0, 0, 0, 0)
            if not response.success:
                raise RuntimeError('SITL yaw preparation rejected')
            time.sleep(6)
        time.sleep(3)
        self.truth_origin = self.xyz(self.truth[0].position)
        self.yaw_delta = self.yaw(self.truth[0].orientation) - self.yaw(self.pose.pose.orientation)
        self.graph = rosgraph.Master(rospy.get_name()).getSystemState()
        publishers = dict(self.graph[0])
        if publishers.get('/mavros/setpoint_position/local') != ['/v0_nav_watchdog']:
            raise RuntimeError('not a single gated FCU publisher')
        response = rospy.ServiceProxy('/v0_nav/start', Trigger)()
        if not response.success:
            raise RuntimeError('start refused: ' + response.message)
        second = rospy.ServiceProxy('/v0_nav/start', Trigger)()
        if second.success:
            raise RuntimeError('double start accepted')
        if self.args.case in ('f3', 'f3_auto_bag'):
            return self.active_parallel_faults(verified)
        if self.args.case == 'link_fault':
            return self.link_fault(verified)
        if self.args.case in ('abort', 'manager_kill', 'manager_exit', 'manager_pause', 'mavros_loss', 'pose_loss', 'concurrent', 'gate_pause', 'gate_kill', 'mode_exit', 'publisher_conflict'):
            return self.safety_fault(verified)
        if self.args.case == 'brake_probe':
            self.wait(lambda: self.velocity and math.hypot(*self.xyz(self.velocity.twist.linear)[:2]) > .12, 15, 'moving before BRAKE')
            before = self.row()
            began = time.monotonic()
            self.mode('BRAKE')
            self.wait(lambda: self.velocity and math.hypot(*self.xyz(self.velocity.twist.linear)[:2]) < .04, 6, 'BRAKE settling')
            after = self.row()
            elapsed = time.monotonic() - began
            self.mode('LAND')
            self.wait(lambda: not self.state.armed, 35, 'disarm')
            self.armed_by_test = False
            return {'result': 'PASS', 'case': self.args.case, 'identity': verified,
                    'before': before, 'after': after, 'elapsed_s': elapsed,
                    'evidence': str(self.directory)}
        self.wait(lambda: self.status.get('state') == 'FINISH_HOLD', 110, 'mission complete')
        self.sample()
        if not self.state.armed or self.state.mode != 'GUIDED':
            raise RuntimeError('navigation changed arm or mode at completion')
        time.sleep(.5)
        actual_arrivals = [event for event in self.events if event.get('state') == 'DWELL']
        if len(actual_arrivals) != len(config['waypoints']):
            raise RuntimeError('missing arrival events')
        active = [row for row in self.sample_rows if row.get('status', {}).get('state') in ('EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD')]
        max_speed = max(math.hypot(*row['velocity'][:2]) for row in active if 'velocity' in row)
        max_truth_speed = max(math.hypot(*row['truth_velocity'][:2]) for row in active if 'truth_velocity' in row)
        if max_speed > config['motion']['max_speed_xy_mps'] or max_truth_speed > config['motion']['max_speed_xy_mps']:
            raise RuntimeError('measured speed exceeds configured ceiling')
        errors = []
        for row in self.arrival_rows:
            if 'expected_truth' not in row:
                raise RuntimeError('missing truth/origin evidence')
            horizontal = math.hypot(*(row['truth'][index] - row['expected_truth'][index] for index in range(2)))
            vertical = abs(row['truth'][2] - row['expected_truth'][2])
            errors.append({'wp': row['wp'], 'xy_m': horizontal, 'z_m': vertical})
            if horizontal > .15 or vertical > .11:
                raise RuntimeError('truth arrival mismatch: ' + str(errors[-1]))
        if self.args.case == 'finish_stop':
            if not rospy.ServiceProxy('/v0_nav/stop', Trigger)().success:
                raise RuntimeError('normal finish stop rejected')
            self.wait(lambda: self.state.mode == 'BRAKE' and self.status.get('state') == 'STOPPED', 5, 'finish stop guarded exit')
        else:
            self.mode('ALT_HOLD')
            self.wait(lambda: self.status.get('state') == 'ABORT_LATCHED', 4, 'abort')
            self.mode('GUIDED')
            time.sleep(.5)
            if self.status.get('state') != 'ABORT_LATCHED':
                raise RuntimeError('automatic resume after mode exit')
        takeover = {'state': self.status, 'mode': self.state.mode}
        self.mode('LAND')
        self.wait(lambda: not self.state.armed, 35, 'disarm')
        self.armed_by_test = False
        return {'result': 'PASS', 'case': self.args.case, 'identity': verified,
                'waypoints': len(config['waypoints']), 'arrival_truth_errors': errors,
                'max_fcu_speed_xy_mps': max_speed, 'max_truth_speed_xy_mps': max_truth_speed,
                'max_pose_header_age_s': max(row['pose_age_s'] for row in active),
                'max_setpoint_gap_s': max(row['status']['max_publish_gap_s'] for row in active),
                'preparation': self.preparation, 'gate': self.watchdog,
                'mode_exit_latched': takeover, 'graph': self.graph,
                'max_navigation_tree_rss_bytes': self.max_rss,
                'system_cpu_percent_max': max(self.cpu_samples),
                'evidence': str(self.directory)}

    def node_process(self, name):
        identity()
        uri = rosgraph.Master(rospy.get_name()).lookupNode(name)
        code, _, pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        if code != 1:
            raise RuntimeError('node PID unavailable')
        return psutil.Process(pid)

    def safety_fault(self, verified):
        case = self.args.case
        self.wait(lambda: self.status.get('state') == 'EXEC_WAYPOINT' and self.velocity and
                  math.hypot(*self.xyz(self.velocity.twist.linear)[:2]) > .12, 15, 'execution before fault')
        before = self.row()
        began = time.monotonic()
        responses = []
        if case == 'abort':
            responses = [rospy.ServiceProxy('/v0_nav/abort', Trigger)().success]
        elif case == 'concurrent':
            services = ['start', 'abort', 'stop', 'start', 'abort', 'stop']
            with ThreadPoolExecutor(max_workers=6) as pool:
                responses = list(pool.map(lambda name: (name, rospy.ServiceProxy('/v0_nav/' + name, Trigger)().success), services))
            if any(success for name, success in responses if name != 'abort'):
                raise RuntimeError('concurrent start/stop accepted during active/aborted task')
        elif case == 'manager_kill':
            self.node_process('/v0_nav_manager').kill()
        elif case == 'manager_exit':
            killed, failed = rosnode.kill_nodes(['/v0_nav_manager'])
            if failed:
                raise RuntimeError('manager normal exit failed')
        elif case in ('manager_pause', 'gate_pause'):
            self.suspended_manager = self.node_process('/v0_nav_manager' if case == 'manager_pause' else '/v0_nav_watchdog')
            self.suspended_manager.suspend()
        elif case == 'pose_loss':
            self.pose_fault = True
        elif case == 'mavros_loss':
            self.node_process('/mavros').kill()
        elif case == 'mode_exit':
            self.mode('ALT_HOLD')
        elif case == 'gate_kill':
            self.node_process('/v0_nav_watchdog').kill()
        elif case == 'publisher_conflict':
            self.conflicting_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        if case == 'gate_pause':
            self.wait(lambda: self.status.get('state') == 'ABORT_LATCHED', 3, 'gate pause abort')
            self.suspended_manager.resume()
            self.suspended_manager = None
        brake_expected = case in ('abort', 'concurrent', 'manager_kill', 'manager_exit', 'manager_pause')
        if brake_expected:
            self.wait(lambda: self.state.mode == 'BRAKE' and self.watchdog.get('revoked'), 5, 'guarded BRAKE feedback')
            self.wait(lambda: self.velocity and math.hypot(*self.xyz(self.velocity.twist.linear)[:2]) < .04, 6, 'fault braking settled')
        else:
            self.wait(lambda: self.status.get('state') == 'ABORT_LATCHED' and
                      (self.watchdog.get('revoked') or case == 'gate_kill'), 4, 'fault latched')
            if (self.watchdog.get('exit_result') or {}).get('requested'):
                raise RuntimeError('blind BRAKE request with invalid position/link/cycle')
        after = self.row()
        if self.suspended_manager:
            self.suspended_manager.resume()
            self.suspended_manager = None
        self.pose_fault = False
        if case == 'mode_exit':
            if self.state.mode != 'ALT_HOLD':
                raise RuntimeError('manual takeover overridden')
            self.mode('GUIDED')
        if case == 'mavros_loss':
            self.replacement_mavros = subprocess.Popen(['roslaunch', 'mavros', 'apm.launch',
                    'fcu_url:=udp://127.0.0.1:14672@', 'gcs_url:='],
                    stdout=(self.directory / 'mavros_recovery.txt').open('w'), stderr=subprocess.STDOUT)
            self.wait(lambda: self.state.connected and self.pose and rospy.get_time()-self.pose.header.stamp.to_sec() < .2,
                      35, 'MAVROS reconnect')
            recovered = wait_identity()
            if recovered['mavros_pid'] == verified['mavros_pid']:
                raise RuntimeError('MAVROS was not replaced')
        for attempt in range(15):
            self.sample()
            time.sleep(.1)
        if case not in ('manager_kill', 'manager_exit'):
            if self.status.get('state') != 'ABORT_LATCHED' or rospy.ServiceProxy('/v0_nav/start', Trigger)().success:
                raise RuntimeError('failure recovery resumed the mission')
        result = {'result': 'PASS', 'case': case, 'identity': verified, 'before': before, 'after': after,
                  'fault_to_observed_response_s': after['mono_s']-began, 'guarded_brake_expected': brake_expected,
                  'responses': responses, 'recovery_did_not_resume': True, 'evidence': str(self.directory)}
        self.mode('LAND')
        self.wait(lambda: not self.state.armed, 35, 'disarm')
        self.armed_by_test = False
        return result

    def active_parallel_faults(self, verified):
        self.wait(lambda: self.status.get('state') == 'EXEC_WAYPOINT', 10, 'execution before audio fault')
        self.mock.send_signal(signal.SIGINT)
        self.mock.wait(timeout=5)
        for attempt in range(15):
            self.sample()
            time.sleep(.1)
        if self.state.mode != 'GUIDED' or self.status.get('state') == 'ABORT_LATCHED':
            raise RuntimeError('audio fault changed navigation mode/task')
        self.mock = subprocess.Popen(['python3', str(ROOT / 'tools/v0_mock_acquisition.py')],
                                     stdout=(self.directory / 'mock_restart.txt').open('w'), stderr=subprocess.STDOUT)
        self.bag.send_signal(signal.SIGINT)
        self.bag.wait(timeout=10)
        for attempt in range(5):
            self.sample()
            time.sleep(.1)
        if self.state.mode != 'GUIDED' or self.status.get('state') == 'ABORT_LATCHED':
            raise RuntimeError('recorder exit stopped navigation')
        self.bag = subprocess.Popen(['rosbag', 'record', '-O', str(self.directory / 'after_recorder.bag'),
                                    '/sound_sensor/audio_raw', '/motor_mic/audio_raw', '/v0_nav/watchdog', '/mavros/state'],
                                   stdout=(self.directory / 'rosbag_restart.txt').open('w'), stderr=subprocess.STDOUT)
        self.wait(lambda: active_bag_bytes(self.directory, self.bag, 'after_recorder*.bag.active') > 0,
                  10, 'replacement recorder live file')
        if self.auto_recorder:
            self.wait(lambda: active_bag_bytes(self.directory / 'auto', self.auto_recorder) > 0,
                      10, 'automatic ARM recorder live file')
        self.wait(lambda: self.status.get('state') == 'EXEC_WAYPOINT' and self.status.get('wp', 0) >= 1, 30, 'execution before node kill')
        before = active_bag_bytes(self.directory, self.bag, 'after_recorder*.bag.active')
        automatic_before = sum(path.stat().st_size for path in (self.directory / 'auto').glob('*.bag.active'))
        killed, failed = rosnode.kill_nodes(['/v0_nav_manager'])
        if failed:
            raise RuntimeError('manager kill failed')
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            self.sample()
            time.sleep(.1)
        after = active_bag_bytes(self.directory, self.bag, 'after_recorder*.bag.active')
        automatic_after = sum(path.stat().st_size for path in (self.directory / 'auto').glob('*.bag.active'))
        if self.mock.poll() is not None or self.bag.poll() is not None or after <= before:
            raise RuntimeError('acquisition did not survive active manager kill')
        if not self.watchdog.get('revoked') or self.watchdog.get('reason') not in (
                'MISSION_HEARTBEAT_TIMEOUT', 'MISSION_STATUS_TIMEOUT', 'TARGET_OR_ORIGIN_STALE'):
            raise RuntimeError('watchdog did not observe manager loss')
        if self.state.mode != 'BRAKE' or not self.state.armed:
            raise RuntimeError('unexpected FCU behavior after manager kill')
        self.mode('LAND')
        self.wait(lambda: not self.state.armed, 35, 'disarm')
        self.armed_by_test = False
        if self.auto_recorder:
            self.wait(lambda: not list((self.directory / 'auto').glob('*.bag.active')) and bool(list((self.directory / 'auto').glob('*.bag'))), 10, 'automatic DISARM bag closure')
            if self.auto_recorder.poll() is not None or automatic_after <= automatic_before:
                raise RuntimeError('automatic recorder did not survive manager exit')
        return {'result': 'PASS', 'case': self.args.case, 'identity': verified,
                'single_master_single_mavros_graph': self.graph,
                'audio_killed_during_execution_no_mode_or_abort': True,
                'manager_killed_during_execution': killed,
                'watchdog_observed_manager_timeout': self.watchdog,
                'audio_and_bag_survived_manager_kill': True,
                'bag_growth_after_kill_bytes': after-before,
                'recorder_exit_did_not_stop_navigation': True,
                'FCU_after_kill': 'armed/BRAKE with independently healthy position; no automatic landing',
                'max_navigation_tree_rss_bytes': self.max_rss,
                'system_cpu_percent_max': max(self.cpu_samples),
                'load_kind': 'synthetic PCM-sized UInt8MultiArray with sequence prefix; not physical PCM/ESC',
                'automatic_bag_growth_after_kill_bytes': automatic_after-automatic_before,
                'real_auto_bag_recorder': 'local source ARM/DISARM test with mock load PASS' if self.auto_recorder else 'NOT_USED',
                'evidence': str(self.directory)}

    def link_fault(self, verified):
        self.wait(lambda: self.status.get('state') == 'EXEC_WAYPOINT', 10, 'execution before link fault')
        verified = identity()
        self.suspended_proxy = psutil.Process(verified['proxy_pid'])
        began = time.monotonic()
        self.suspended_proxy.suspend()
        self.wait(lambda: self.status.get('state') == 'ABORT_LATCHED', 2, 'abort')
        elapsed = time.monotonic() - began
        abort = dict(self.status)
        time.sleep(1.)
        self.suspended_proxy.resume()
        self.suspended_proxy = None
        for attempt in range(20):
            self.sample()
            time.sleep(.1)
        if self.status.get('state') != 'ABORT_LATCHED' or rospy.ServiceProxy('/v0_nav/start', Trigger)().success:
            raise RuntimeError('link recovery resumed mission')
        self.mode('LAND')
        self.wait(lambda: not self.state.armed, 35, 'disarm')
        self.armed_by_test = False
        return {'result': 'PASS', 'case': 'link_fault', 'identity': verified,
                'fault': 'SITL-only MAVProxy SIGSTOP/SIGCONT; FCU/ROS data path halted, not real USB unplug',
                'abort_latency_s': elapsed, 'abort': abort, 'recovery_did_not_resume': True,
                'mode_before_test_cleanup': 'GUIDED', 'evidence': str(self.directory)}

    def cleanup(self):
        if self.suspended_manager:
            self.suspended_manager.resume()
            self.suspended_manager = None
        if self.suspended_proxy:
            self.suspended_proxy.resume()
            self.suspended_proxy = None
        if self.armed_by_test:
            try:
                wait_identity()
                rospy.ServiceProxy('/mavros/set_mode', SetMode)(0, 'LAND')
                deadline = time.monotonic() + 35
                while self.state.armed and time.monotonic() < deadline:
                    time.sleep(.2)
            except Exception as error:
                self.event_file.write(json.dumps({'cleanup_error': str(error)}) + '\n')
        cleanup = {'armed_before': self.state.armed, 'armed_after': self.state.armed,
                   'connected': self.state.connected, 'mode': self.state.mode, 'sim_stopped_by_test': False}
        if self.replacement_mavros and self.state.connected and not self.state.armed:
            stopped = subprocess.run(['python3', str(ROOT / 'tools/v0_sitl_stop.py'), '--sitl'],
                                     stdout=(self.directory / 'recovery_sim_stop.txt').open('w'), stderr=subprocess.STDOUT, timeout=70)
            cleanup['sim_stopped_by_test'] = stopped.returncode == 0
        for process in (self.navigation, self.mock, self.bag, self.replacement_mavros, self.auto_recorder):
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
        cleanup.update(processes_exited=all(process is None or process.poll() is not None for process in
                                            (self.navigation, self.mock, self.bag, self.replacement_mavros, self.auto_recorder)))
        (self.directory / 'cleanup.json').write_text(json.dumps(cleanup, indent=2))
        with self.lock:
            (self.directory / 'receptions.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in self.receptions))
        self.samples.close()
        self.event_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sitl', action='store_true', required=True)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--mission-file', type=Path, default=ROOT / 'src/indoor_nav/config/mission_v0.yaml')
    parser.add_argument('--case', choices=('dry_run', 'single', 'multi', 'yaw', 'altitude', 'distance_short', 'distance_long',
                        'finish_stop', 'f3', 'f3_auto_bag', 'link_fault', 'brake_probe', 'abort', 'manager_kill',
                        'manager_exit', 'manager_pause', 'mavros_loss', 'pose_loss', 'concurrent', 'gate_pause',
                        'gate_kill', 'mode_exit', 'publisher_conflict'), default='single')
    args = parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != MASTER:
        raise RuntimeError('wrong master')
    rospy.init_node('v0_sitl_test', anonymous=True)
    test = FlightTest(args)
    began = time.time()
    try:
        result = test.execute()
    except Exception as error:
        result = {'result': 'FAIL', 'case': args.case, 'error': str(error), 'evidence': str(test.directory)}
    finally:
        try:
            test.cleanup()
        except Exception as error:
            (test.directory / 'cleanup.json').write_text(json.dumps({'armed_after': True, 'connected': False,
                         'processes_exited': False, 'error': str(error)}, indent=2))
    cleanup = json.loads((test.directory / 'cleanup.json').read_text())
    result['cleanup'] = cleanup
    if cleanup['armed_after'] or not cleanup['connected'] or not cleanup['processes_exited']:
        result.update(result='FAIL', cleanup_failure='test-owned processes or simulator still active/armed')
    result.update(start_wall_s=began, end_wall_s=time.time())
    (test.directory / 'summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
