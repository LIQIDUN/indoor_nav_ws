#!/usr/bin/env python3
"""Offline configuration smoke or read-only ROS graph/telemetry snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/indoor_nav/src'))
from indoor_nav.v0 import load_config, relative_target


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, default=ROOT / 'src/indoor_nav/config/mission_v0.yaml')
    parser.add_argument('--ros-smoke', action='store_true')
    args = parser.parse_args()
    raw = args.config.read_bytes()
    config = load_config(raw.decode())
    result = {'result': 'PASS', 'yaml_sha256': hashlib.sha256(raw).hexdigest(),
              'mission_id': config['mission_id'], 'waypoints': len(config['waypoints']),
              'targets_yaw_zero': [relative_target((0., 0., 1.), 0., waypoint) for waypoint in config['waypoints']]}
    if args.ros_smoke:
        import rosgraph
        import rospy
        from mavros_msgs.msg import State
        from std_msgs.msg import String
        rospy.init_node('v0_readonly_smoke', anonymous=True)
        result['state'] = str(rospy.wait_for_message('/mavros/state', State, timeout=5))
        result['status'] = json.loads(rospy.wait_for_message('/v0_nav/status', String, timeout=5).data)
        result['publishers'] = rosgraph.Master(rospy.get_name()).getSystemState()[0]
        if result['status']['control_enabled']:
            raise RuntimeError('read-only smoke expects control disabled')
        controllers = dict(result['publishers']).get('/mavros/setpoint_position/local', [])
        if set(controllers) & {'/v0_nav_manager', '/v0_nav_watchdog'}:
            raise RuntimeError('read-only V0 unexpectedly owns a control publisher')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
