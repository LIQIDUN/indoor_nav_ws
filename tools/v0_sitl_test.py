#!/usr/bin/env python3
"""Explicit, identity-gated simulation flight driver. Never imported by launches."""
import argparse
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
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, ParamGet, SetMode
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Trigger

from indoor_nav.v0 import quaternion_yaw

ROOT = Path(__file__).resolve().parents[1]
MASTER = 'http://127.0.0.1:11421'


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
        self.truth_origin = self.yaw_delta = None
        self.max_rss = 0
        self.cpu_samples = []
        self.process = psutil.Process()
        rospy.Subscriber('/mavros/state', State, lambda message: setattr(self, 'state', message), queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, lambda message: setattr(self, 'pose', message), queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, lambda message: setattr(self, 'velocity', message), queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_cb, queue_size=1)
        rospy.Subscriber('/v0_nav/status', String, lambda message: setattr(self, 'status', json.loads(message.data)), queue_size=10)
        rospy.Subscriber('/v0_nav/event', String, self.event_cb, queue_size=100)
        rospy.Subscriber('/v0_nav/watchdog', String, lambda message: setattr(self, 'watchdog', json.loads(message.data)), queue_size=10)
        rospy.Subscriber('/v0_nav/target_pose', PoseStamped, lambda message: setattr(self, 'target', message), queue_size=1)
        rospy.Subscriber('/v0_nav/origin_pose', PoseStamped, lambda message: setattr(self, 'origin', message), queue_size=1)
        rospy.Subscriber('/mavros/setpoint_raw/target_local', PositionTarget, lambda message: setattr(self, 'echo', message), queue_size=1)

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
                  'status': self.status}
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
        verified = identity()
        config = yaml.safe_load((ROOT / 'src/indoor_nav/config/mission_v0.yaml').read_text())
        config['navigation']['source'] = 'fc_ekf_sitl_gps'
        config['mission_id'] = self.directory.name
        if self.args.case == 'single':
            config['waypoints'] = [dict(id='forward', x=.25, y=0., z=0., hold_s=1.)]
        else:
            config['waypoints'] = [dict(id='forward', x=.25, y=0., z=0., hold_s=1.),
                                   dict(id='right', x=.25, y=.25, z=0., hold_s=1.),
                                   dict(id='back', x=0., y=.25, z=.1, hold_s=1.),
                                   dict(id='return', x=0., y=0., z=0., hold_s=1.)]
        mission_path = self.directory / 'mission.yaml'
        mission_path.write_text(yaml.safe_dump(config, sort_keys=False))
        bag_topics = ['/mavros/state', '/mavros/local_position/pose', '/mavros/local_position/velocity_local',
                      '/mavros/global_position/raw/fix', '/mavros/estimator_status', '/mavlink/from', '/gazebo/model_states',
                      '/mavros/setpoint_position/local', '/mavros/setpoint_raw/target_local',
                      '/v0_nav/status', '/v0_nav/event', '/v0_nav/target_pose', '/v0_nav/origin_pose',
                      '/v0_nav/heartbeat', '/v0_nav/watchdog', '/sound_sensor/audio_raw', '/motor_mic/audio_raw', '/vibration_data']
        self.bag = subprocess.Popen(['rosbag', 'record', '-O', str(self.directory / 'flight.bag'), *bag_topics],
                                    stdout=(self.directory / 'rosbag.txt').open('w'), stderr=subprocess.STDOUT)
        if self.args.case == 'f3':
            self.mock = subprocess.Popen(['python3', str(ROOT / 'tools/v0_mock_acquisition.py')],
                                         stdout=(self.directory / 'mock.txt').open('w'), stderr=subprocess.STDOUT)
        self.navigation = subprocess.Popen(['roslaunch', 'indoor_nav', 'indoor_nav_v0.launch',
                                            'mission_file:=' + str(mission_path), 'enable_control:=true', 'dry_run:=false',
                                            'evidence_dir:=' + str(self.directory / 'navigation')],
                                           stdout=(self.directory / 'navigation.txt').open('w'), stderr=subprocess.STDOUT)
        self.wait(lambda: 'gate' in self.status and not self.status['gate'], 70, 'FCU limit readiness')
        if self.state.armed:
            raise RuntimeError('vehicle already armed; refusing ownership')
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
        if self.args.case == 'multi':
            response = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)(False, 115, 0, 90, 20, 1, 0, 0, 0, 0)
            if not response.success:
                raise RuntimeError('SITL yaw preparation rejected')
            time.sleep(6)
        time.sleep(3)
        self.truth_origin = self.xyz(self.truth[0].position)
        self.yaw_delta = self.yaw(self.truth[0].orientation) - self.yaw(self.pose.pose.orientation)
        self.graph = rosgraph.Master(rospy.get_name()).getSystemState()
        response = rospy.ServiceProxy('/v0_nav/start', Trigger)()
        if not response.success:
            raise RuntimeError('start refused: ' + response.message)
        second = rospy.ServiceProxy('/v0_nav/start', Trigger)()
        if second.success:
            raise RuntimeError('double start accepted')
        if self.args.case == 'f3':
            return self.active_parallel_faults(verified)
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
        parallel = None
        if self.args.case == 'f3':
            parallel = self.parallel_faults()
        return {'result': 'PASS', 'case': self.args.case, 'identity': verified,
                'waypoints': len(config['waypoints']), 'arrival_truth_errors': errors,
                'max_fcu_speed_xy_mps': max_speed, 'max_truth_speed_xy_mps': max_truth_speed,
                'max_pose_header_age_s': max(row['pose_age_s'] for row in active),
                'max_setpoint_gap_s': max(row['status']['max_publish_gap_s'] for row in active),
                'mode_exit_latched': takeover, 'graph': self.graph,
                'max_navigation_tree_rss_bytes': self.max_rss,
                'system_cpu_percent_max': max(self.cpu_samples), 'parallel': parallel,
                'evidence': str(self.directory)}

    def parallel_faults(self):
        self.navigation.send_signal(signal.SIGINT)
        self.navigation.wait(timeout=12)
        before = (self.directory / 'flight.bag.active').stat().st_size
        time.sleep(2)
        after = (self.directory / 'flight.bag.active').stat().st_size
        if self.mock.poll() is not None or self.bag.poll() is not None or after <= before:
            raise RuntimeError('acquisition did not survive navigation shutdown')
        self.mock.send_signal(signal.SIGINT)
        self.mock.wait(timeout=5)
        if not self.state.connected or self.bag.poll() is not None:
            raise RuntimeError('audio shutdown affected MAVROS/recording')
        return {'navigation_shutdown_acquisition_survived': True,
                'audio_shutdown_mavros_survived': True, 'bag_growth_after_navigation_bytes': after-before,
                'load_kind': 'synthetic UInt8MultiArray PCM-sized blocks, not physical microphones',
                'kill_during_active_mission': 'NOT_TESTED_IN_THIS_CASE',
                'real_auto_bag_recorder': 'UNAVAILABLE'}

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
        self.wait(lambda: self.status.get('state') == 'EXEC_WAYPOINT' and self.status.get('wp', 0) >= 1, 30, 'execution before node kill')
        before = (self.directory / 'flight.bag.active').stat().st_size
        killed, failed = rosnode.kill_nodes(['/v0_nav_manager'])
        if failed:
            raise RuntimeError('manager kill failed')
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            self.sample()
            time.sleep(.1)
        after = (self.directory / 'flight.bag.active').stat().st_size
        if self.mock.poll() is not None or self.bag.poll() is not None or after <= before:
            raise RuntimeError('acquisition did not survive active manager kill')
        if self.watchdog.get('reason') != 'MISSION_HEARTBEAT_TIMEOUT':
            raise RuntimeError('watchdog did not observe manager loss')
        if self.state.mode != 'GUIDED' or not self.state.armed:
            raise RuntimeError('unexpected FCU behavior after manager kill')
        self.mode('LAND')
        self.wait(lambda: not self.state.armed, 35, 'disarm')
        self.armed_by_test = False
        return {'result': 'PASS', 'case': 'f3', 'identity': verified,
                'single_master_single_mavros_graph': self.graph,
                'audio_killed_during_execution_no_mode_or_abort': True,
                'manager_killed_during_execution': killed,
                'watchdog_observed_heartbeat_timeout': self.watchdog,
                'audio_and_bag_survived_manager_kill': True,
                'bag_growth_after_kill_bytes': after-before,
                'FCU_after_kill': 'remained armed/GUIDED; no autonomous watchdog braking',
                'max_navigation_tree_rss_bytes': self.max_rss,
                'system_cpu_percent_max': max(self.cpu_samples),
                'load_kind': 'synthetic PCM-sized UInt8MultiArray with sequence prefix; not physical PCM/ESC',
                'real_auto_bag_recorder': 'UNAVAILABLE', 'evidence': str(self.directory)}

    def cleanup(self):
        if self.armed_by_test:
            try:
                identity()
                rospy.ServiceProxy('/mavros/set_mode', SetMode)(0, 'LAND')
                deadline = time.monotonic() + 35
                while self.state.armed and time.monotonic() < deadline:
                    time.sleep(.2)
            except Exception as error:
                self.event_file.write(json.dumps({'cleanup_error': str(error)}) + '\n')
        for process in (self.navigation, self.mock, self.bag):
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.terminate()
        self.samples.close()
        self.event_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sitl', action='store_true', required=True)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--case', choices=('single', 'multi', 'f3'), default='single')
    args = parser.parse_args()
    rospy.init_node('v0_sitl_test', anonymous=True)
    identity()
    test = FlightTest(args)
    began = time.time()
    try:
        result = test.execute()
    except Exception as error:
        result = {'result': 'FAIL', 'case': args.case, 'error': str(error), 'evidence': str(test.directory)}
    finally:
        test.cleanup()
    result.update(start_wall_s=began, end_wall_s=time.time())
    (test.directory / 'summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
