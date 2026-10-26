#!/usr/bin/env python3
"""SITL communication load only; NOT a physical PCM driver or auto recorder."""
import os
import threading
import time

import rospy
from std_msgs.msg import UInt8MultiArray


if __name__ == '__main__':
    rospy.init_node('v0_mock_audio')
    if not rospy.get_param('/v0_sitl/isolated', False) or os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11421':
        raise RuntimeError('synthetic audio requires isolated SITL master')
    xvf = rospy.Publisher('/sound_sensor/audio_raw', UInt8MultiArray, queue_size=2)
    motor = rospy.Publisher('/motor_mic/audio_raw', UInt8MultiArray, queue_size=2)
    vibration = rospy.Publisher('/vibration_data', UInt8MultiArray, queue_size=2)
    stopped = threading.Event()
    rospy.on_shutdown(stopped.set)
    sequence = 0
    while not rospy.is_shutdown():
        header = sequence.to_bytes(8, 'little')
        xvf.publish(UInt8MultiArray(data=header + bytes(1600 * 6 * 2)))
        motor.publish(UInt8MultiArray(data=header + bytes(4800 * 6 * 3)))
        vibration.publish(UInt8MultiArray(data=header + bytes(64)))
        sequence += 1
        stopped.wait(.1)
