#!/usr/bin/env python3
"""Observation-only watchdog. Never publishes a FCU target or mode request."""
import json
import threading
import time

import rosgraph
import rospy
from std_msgs.msg import Header, String


class Watchdog:
    def __init__(self):
        self.received = None
        self.graph_time = None
        self.graph_reason = 'GRAPH_NOT_READY'
        self.closed = threading.Event()
        self.publisher = rospy.Publisher('/v0_nav/watchdog', String, queue_size=1, latch=True)
        rospy.Subscriber('/v0_nav/heartbeat', Header, self.heartbeat, queue_size=1)
        rospy.on_shutdown(self.closed.set)
        threading.Thread(target=self.graph, daemon=True).start()

    def heartbeat(self, _message):
        self.received = time.monotonic()

    def graph(self):
        master = rosgraph.Master(rospy.get_name())
        while not self.closed.is_set() and not rospy.is_shutdown():
            try:
                publishers, _, _ = master.getSystemState()
                reasons = []
                for topic, nodes in publishers:
                    if topic == '/mavros/setpoint_position/local' and set(nodes) - {'/v0_nav_manager'}:
                        reasons.append('OTHER_POSITION_PUBLISHER')
                    if topic in ('/mavros/setpoint_raw/local', '/mavros/setpoint_velocity/cmd_vel',
                                 '/mavros/setpoint_velocity/cmd_vel_unstamped', '/mavros/setpoint_raw/attitude') and nodes:
                        reasons.append('OTHER_CONTROL_PUBLISHER')
                    if topic == '/v0_nav/heartbeat' and nodes != ['/v0_nav_manager']:
                        reasons.append('DUPLICATE_MANAGER')
                self.graph_reason = ','.join(reasons)
                self.graph_time = time.monotonic()
            except Exception as error:
                self.graph_reason = 'ROS_GRAPH_UNAVAILABLE:' + type(error).__name__
            self.closed.wait(.2)

    def run(self):
        last_reason = None
        while not rospy.is_shutdown():
            now = time.monotonic()
            reason = self.graph_reason
            if self.graph_time is None or now - self.graph_time > .6:
                reason = 'GRAPH_STALE'
            if self.received is None or now - self.received > .5:
                reason = 'MISSION_HEARTBEAT_TIMEOUT'
            self.publisher.publish(String(data=json.dumps({'healthy': not reason, 'reason': reason})))
            if reason and reason != last_reason:
                rospy.logerr('V0 watchdog: %s; observational only, RC/FCU failsafe required', reason)
            last_reason = reason
            self.closed.wait(.1)


if __name__ == '__main__':
    rospy.init_node('v0_nav_watchdog')
    Watchdog().run()
