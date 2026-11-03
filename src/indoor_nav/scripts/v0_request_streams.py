#!/usr/bin/env python3
"""Request telemetry groups through the existing MAVROS connection only."""
import math
import threading
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest


class StreamRequester:
    def __init__(self):
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.connected = False
        self.configured = False
        self.generation = 0
        self.next_try = 0.
        self.settle_s = rospy.get_param('~connection_settle_s', 2.)
        self.retry_s = rospy.get_param('~retry_s', 5.)
        for value in (self.settle_s, self.retry_s):
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError('connection_settle_s/retry_s must be finite and nonnegative')
        if self.retry_s == 0:
            raise ValueError('retry_s must be positive')
        self.streams = []
        for name, stream_id, default in (
                ('EXTENDED_STATUS', StreamRateRequest.STREAM_EXTENDED_STATUS, 5),
                ('POSITION', StreamRateRequest.STREAM_POSITION, 10),
                ('EXTRA1', StreamRateRequest.STREAM_EXTRA1, 10),
                ('EXTRA3', StreamRateRequest.STREAM_EXTRA3, 5),
                ('RAW_SENSORS', StreamRateRequest.STREAM_RAW_SENSORS, 10)):
            hz = rospy.get_param('~' + name.lower() + '_hz', default)
            if type(hz) is not int or not 0 <= hz <= 65535:
                raise ValueError('{} frequency must be an integer in [0, 65535]'.format(name))
            self.streams.append((stream_id, hz, name))
        self.client = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=10)
        rospy.on_shutdown(self.closed.set)

    def on_state(self, message):
        with self.lock:
            connected = bool(message.connected)
            if connected == self.connected:
                return
            self.connected = connected
            self.configured = False
            self.generation += 1
            self.next_try = time.monotonic() + self.settle_s
            rospy.loginfo('FCU connection changed: %s; telemetry setup reset', connected)

    def request_all(self, generation):
        try:
            rospy.wait_for_service('/mavros/set_stream_rate', timeout=3.)
            for stream_id, hz, name in self.streams:
                with self.lock:
                    if not self.connected or generation != self.generation:
                        return False
                if hz == 0:
                    continue
                self.client(StreamRateRequest(stream_id=stream_id, message_rate=hz, on_off=True))
                rospy.loginfo('Requested MAVLink stream %s at %d Hz; verify actual topic rate', name, hz)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn('MAVLink stream setup incomplete; will retry: %s', error)
            return False
        return True

    def step(self):
        with self.lock:
            if not self.connected or self.configured or time.monotonic() < self.next_try:
                return
            generation = self.generation
        configured = self.request_all(generation)
        with self.lock:
            if self.connected and generation == self.generation:
                self.configured = configured
                if not configured:
                    self.next_try = time.monotonic() + self.retry_s

    def run(self):
        while not rospy.is_shutdown() and not self.closed.is_set():
            self.step()
            self.closed.wait(.5)


if __name__ == '__main__':
    rospy.init_node('v0_request_streams')
    StreamRequester().run()
