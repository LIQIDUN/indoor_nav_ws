#!/usr/bin/env python3
"""Stop only the verified, disarmed V0 simulation launcher and its children."""
import argparse
import os
from pathlib import Path
import signal
import socket
import time
import xmlrpc.client

import psutil
import rospy
from mavros_msgs.msg import State


def wait_exit(process, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            gone = not process.is_running() or process.status() == psutil.STATUS_ZOMBIE
        except psutil.NoSuchProcess:
            gone = True
        if gone:
            return
        time.sleep(.1)
    raise psutil.TimeoutExpired(timeout, pid=process.pid)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sitl', action='store_true', required=True)
    parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11421':
        raise RuntimeError('wrong master')
    socket.setdefaulttimeout(2.)
    code, _, _ = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/v0_sitl_stop_probe')
    if code != 1:
        raise RuntimeError('isolated master unavailable; no processes signalled')
    rospy.init_node('v0_sitl_stop', anonymous=True)
    state = rospy.wait_for_message('/mavros/state', State, timeout=5)
    if not state.connected or state.armed:
        raise RuntimeError('requires connected DISARMED simulation')
    root = Path(__file__).resolve().parents[1]
    runtime = Path(rospy.get_param('/v0_sitl/runtime')).resolve()
    if runtime.parent != root / '.sitl_runtime/v0':
        raise RuntimeError('wrong runtime')
    sitl = psutil.Process(int((runtime / 'arducopter.pid').read_text()))
    if Path(sitl.cwd()).resolve() != runtime or Path(sitl.exe()).resolve() != (Path.home() / 'github/ardupilot/build/sitl/bin/arducopter').resolve():
        raise RuntimeError('wrong simulator')
    launcher = sitl.parent()
    if not any(argument.endswith('v0_sitl_instance.sh') for argument in launcher.cmdline()) or '--sitl' not in launcher.cmdline():
        raise RuntimeError('wrong launcher parent')
    launcher.send_signal(signal.SIGTERM)
    try:
        wait_exit(launcher, 45)
    except psutil.TimeoutExpired:
        if sitl.is_running() and sitl.ppid() == launcher.pid:
            sitl.terminate()
            try:
                wait_exit(sitl, 5)
            except psutil.TimeoutExpired:
                sitl.kill()
        wait_exit(launcher, 10)
    print('Requested PID-scoped disarmed SITL shutdown:', launcher.pid, runtime)
