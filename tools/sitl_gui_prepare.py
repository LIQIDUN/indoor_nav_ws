#!/usr/bin/env python3
"""Internal GUI startup preparation, not a launcher. Never calibrates/ACKs/arms.

Only the locally owned Copter 4.5.7 GUI instance may receive interval, logging,
prearm-check and origin requests. EKF source parameters are checked, not written.
"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import time
import xmlrpc.client

import psutil
import rosgraph
import rospy
from geographic_msgs.msg import GeoPointStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import ParamGet, ParamSet, MessageInterval, VehicleInfoGet, CommandLong
from indoor_nav.velocity import position_only_adapter_guard

WS = Path(__file__).resolve().parents[1]
RUN = WS / '.sitl_runtime'
GUI = RUN / 'gui_instance'


def parse_home(text):
    values = tuple(float(x) for x in text.split(','))
    if (len(values) != 4 or not all(math.isfinite(x) for x in values)
            or not -90 <= values[0] <= 90 or not -180 <= values[1] <= 180):
        raise ValueError('Expected finite lat,lon,alt,heading with valid WGS84 coordinates')
    return values


def origin_matches(actual, home):
    return (abs(actual.latitude-home[0]) <= 1e-6
            and abs(actual.longitude-home[1]) <= 1e-6
            and abs(actual.altitude-home[2]) <= .5)


class Preparation:
    def __init__(self, args):
        self.args = args
        self.home = parse_home(args.home)
        self.record = {'started_wall_s': time.time(), 'events': [], 'success': False,
                       'automatic_calibration': False, 'automatic_ack': False}
        self.state = None
        self.origin = None
        self.pin = None
        self.sub = rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=10)
        self.origin_sub = rospy.Subscriber('/mavros/global_position/gp_origin', GeoPointStamped,
                                          self.on_origin, queue_size=10)

    def on_state(self, msg):
        self.state = (msg, time.monotonic())

    def on_origin(self, msg):
        self.origin = (msg.position, time.monotonic())

    def event(self, event_name, **data):
        item = dict(event=event_name, wall_s=time.time(), **data)
        self.record['events'].append(item)
        self.save()
        print(json.dumps(item), flush=True)

    def save(self):
        target = GUI / 'prepare.json'
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(self.record, indent=2))
        temp.replace(target)

    def identity(self):
        if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11311':
            raise RuntimeError('GUI preparation requires Master 11311')
        if rospy.get_param('/mavros/fcu_url') != 'udp://127.0.0.1:{}@'.format(self.args.port):
            raise RuntimeError('Wrong MAVROS loopback endpoint')
        if rospy.get_param('/indoor_nav/safety/profile') != 'sitl':
            raise RuntimeError('Not SITL profile')
        if self.state is None or time.monotonic()-self.state[1] > 1.5:
            raise RuntimeError('FC state missing/stale')
        state = self.state[0]
        if not state.connected or state.armed:
            raise RuntimeError('Requires connected DISARMED FC')
        ap = (Path(self.args.ap_root) / 'build/sitl/bin/arducopter').resolve()
        candidates = []
        for p in psutil.process_iter(['exe', 'cwd']):
            if p.info['exe'] and Path(p.info['exe']).resolve() == ap:
                if p.info['cwd'] and Path(p.info['cwd']).resolve() == GUI.resolve():
                    candidates.append(p)
        if len(candidates) != 1:
            raise RuntimeError('Expected exactly one locally owned GUI SITL')
        fc = candidates[0]
        cmd = fc.cmdline()
        if '--model' not in cmd or cmd[cmd.index('--model')+1] != 'gazebo':
            raise RuntimeError('Wrong SITL backend')
        if not any(c.status == psutil.CONN_LISTEN and c.laddr.port == 5760
                   for c in fc.connections(kind='tcp')):
            raise RuntimeError('GUI SITL does not own TCP5760')
        master = rosgraph.Master(rospy.get_name())
        uri = master.lookupNode('/mavros')
        code, _, pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        shell = psutil.Process(int((RUN / 'mavros.shell.pid').read_text()))
        if code != 1 or pid not in [c.pid for c in shell.children(recursive=True)]:
            raise RuntimeError('MAVROS not owned by GUI terminal')
        mp = psutil.Process(pid)
        conns = mp.connections(kind='inet')
        if not any(c.type == socket.SOCK_DGRAM and c.laddr.ip == '127.0.0.1'
                   and c.laddr.port == self.args.port for c in conns):
            raise RuntimeError('MAVROS has no expected loopback socket')
        peers = [c.raddr for c in conns if c.type == socket.SOCK_DGRAM and c.raddr]
        if any(c.ip != '127.0.0.1' for c in peers):
            raise RuntimeError('Non-local MAVROS UDP peer')
        proxies = []
        for p in psutil.process_iter(['cmdline']):
            argv = p.info['cmdline'] or []
            if not any(Path(x).name == 'mavproxy.py' for x in argv):
                continue
            if not any(c.raddr and c.raddr.ip == '127.0.0.1' and c.raddr.port == 5760
                       for c in p.connections(kind='tcp')):
                continue
            if not any('127.0.0.1:{}'.format(self.args.port) in x for x in argv):
                continue
            if any(c.raddr and c.raddr.ip not in ('127.0.0.1', '::1')
                   for c in p.connections(kind='inet')):
                raise RuntimeError('Non-local MAVProxy route')
            proxies.append(p)
        if len(proxies) != 1:
            raise RuntimeError('Ambiguous MAVProxy to GUI SITL route')
        provider = master.lookupService('/mavros/param/get')
        for name in ('param/set', 'set_message_interval', 'cmd/command', 'vehicle_info_get'):
            if master.lookupService('/mavros/'+name) != provider:
                raise RuntimeError('Mixed MAVROS service providers')
        pin = (fc.pid, fc.create_time(), pid, proxies[0].pid, proxies[0].create_time())
        if self.pin is not None and self.pin != pin:
            raise RuntimeError('Instance changed during preparation')
        return pin

    def get_param(self, name):
        r = rospy.ServiceProxy('/mavros/param/get', ParamGet)(param_id=name)
        if not r.success:
            raise RuntimeError('Missing FC parameter '+name)
        return r.value.real or float(r.value.integer)

    def run(self):
        deadline = time.monotonic()+60
        last = ''
        while time.monotonic() < deadline:
            try:
                self.pin = self.identity()
                break
            except Exception as exc:
                last = str(exc)
                time.sleep(.5)
        else:
            raise RuntimeError('Identity check timeout: '+last)
        info = rospy.ServiceProxy('/mavros/vehicle_info_get', VehicleInfoGet)(sysid=0, compid=0, get_all=False)
        if not info.success or len(info.vehicles) != 1:
            raise RuntimeError('Ambiguous FC/version')
        v = info.vehicles[0]
        if (v.sysid, v.compid, v.autopilot, v.type, v.flight_sw_version >> 8) != (1, 1, 3, 2, 0x040507):
            raise RuntimeError('Requires local Copter 4.5.7 sysid1/compid1')
        self.event('IDENTITY', pin=self.pin)
        names = ['GPS_TYPE', 'SIM_SPEEDUP', 'AHRS_EKF_TYPE', 'EK3_ENABLE', 'VISO_TYPE',
                 'EK3_SRC1_POSXY', 'VISO_POS_M_NSE', 'VISO_POS_X', 'VISO_POS_Y', 'VISO_POS_Z']
        names += ['EK3_SRC{}_{}'.format(i, f) for i in (1, 2, 3)
                  for f in ('VELXY', 'VELZ', 'POSZ', 'YAW')]
        # MAVROS connected precedes completion of its initial parameter download.
        # Wait for the cache, without accepting missing values or writing EKF sources.
        deadline = time.monotonic()+40
        while True:
            try:
                values = {name: self.get_param(name) for name in names}
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                self.identity()
                time.sleep(.5)
        self.event('EKF_PARAMETERS', values=values)
        ok, reason = position_only_adapter_guard('sitl', (4, 5, 7), values, .2)
        if (not ok or values['SIM_SPEEDUP'] <= 0 or values['EK3_SRC1_VELXY'] != 0
                or values['EK3_SRC1_VELZ'] != 0
                or any(values['VISO_POS_'+axis] != 0 for axis in 'XYZ')):
            raise RuntimeError('EKF position-only configuration rejected: '+reason)
        truth = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
        pose = truth.pose[truth.name.index('iris')]
        if (pose.position.z > .08 or math.hypot(pose.position.x, pose.position.y) > .05
                or max(abs(pose.orientation.x), abs(pose.orientation.y), abs(pose.orientation.z)) > .02):
            raise RuntimeError('Requires initial ground pose and +X nose alignment')
        for name, value in (('ARMING_CHECK', 1), ('LOG_DISARMED', 1)):
            before = self.get_param(name)
            self.identity()
            r = rospy.ServiceProxy('/mavros/param/set', ParamSet)(param_id=name, value=ParamValue(integer=value, real=0.))
            if not r.success or self.get_param(name) != value:
                raise RuntimeError('SITL parameter readback failed: '+name)
            self.event('SITL_PARAMETER', name=name, before=before, after=value)
        # Stream groups remain owned by MAVProxy at 25Hz. HB has an explicit interval.
        self.identity()
        r = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)(message_id=0, message_rate=5.)
        if not r.success:
            raise RuntimeError('HEARTBEAT 5Hz request rejected')
        self.event('HEARTBEAT_REQUEST', rate_hz=5.)
        pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=1)
        deadline = time.monotonic()+5
        while pub.get_num_connections() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('Origin publisher has no MAVROS subscriber')
            time.sleep(.1)
        msg = GeoPointStamped()
        msg.header.frame_id = 'earth'
        msg.position.latitude, msg.position.longitude, msg.position.altitude = self.home[:3]
        started = time.monotonic()
        for _ in range(5):
            self.identity()
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            time.sleep(.2)
        self.identity()
        r = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)(broadcast=False, command=512,
            confirmation=0, param1=49., param2=0., param3=0., param4=0., param5=0., param6=0., param7=0.)
        if not r.success:
            raise RuntimeError('GPS_GLOBAL_ORIGIN readback request rejected')
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            if self.origin is not None and self.origin[1] >= started:
                if not origin_matches(self.origin[0], self.home):
                    raise RuntimeError('EKF origin readback differs from requested home')
                self.event('ORIGIN_ACCEPTED', lat=self.origin[0].latitude,
                           lon=self.origin[0].longitude, alt=self.origin[0].altitude)
                self.record['success'] = True
                self.save()
                print('SITL PREPARED. Manually calibrate, verify origin, then safety/acknowledge. No ARM.', flush=True)
                return
            time.sleep(.1)
        raise RuntimeError('No fresh EKF origin readback')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ap-root', required=True)
    parser.add_argument('--home', required=True)
    parser.add_argument('--port', type=int, default=14552)
    args = parser.parse_args()
    parse_home(args.home)
    socket.setdefaulttimeout(3)
    rospy.init_node('sitl_gui_prepare', anonymous=True)
    task = Preparation(args)
    try:
        task.run()
    except Exception as exc:
        task.event('FAILED', reason=str(exc))
        raise SystemExit('SITL PREPARATION FAILED: '+str(exc))


if __name__ == '__main__':
    main()
