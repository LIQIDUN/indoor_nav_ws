#!/usr/bin/env python3
"""Stop only the verified, disarmed V0 simulation launcher and its children."""
import argparse
import os
from pathlib import Path
import signal

import psutil
import rospy
from mavros_msgs.msg import State


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sitl', action='store_true', required=True)
    parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11421':
        raise RuntimeError('wrong master')
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
    launcher.send_signal(signal.SIGINT)
    try:
        launcher.wait(timeout=8)
    except psutil.TimeoutExpired:
        if sitl.is_running() and sitl.ppid() == launcher.pid:
            sitl.terminate()
            try:
                sitl.wait(timeout=5)
            except psutil.TimeoutExpired:
                sitl.kill()
        launcher.wait(timeout=10)
    print('Requested PID-scoped disarmed SITL shutdown:', launcher.pid, runtime)
