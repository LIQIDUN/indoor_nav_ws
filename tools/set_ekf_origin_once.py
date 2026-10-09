#!/usr/bin/env python3
"""Explicit operator action: publish SET_GPS_GLOBAL_ORIGIN via MAVROS (never automatic).

Only for SITL / stationary disarmed setup. Geodetic origin is a reference, not GPS input.
"""
import argparse
import math
import sys


def main():
    p = argparse.ArgumentParser(description='Request ArduPilot EKF Origin via MAVROS')
    p.add_argument('--lat', type=float, required=True, help='WGS84 latitude degrees')
    p.add_argument('--lon', type=float, required=True, help='WGS84 longitude degrees')
    p.add_argument('--alt', type=float, required=True, help='Altitude in metres AMSL')
    args = p.parse_args()
    if not all(math.isfinite(v) for v in [args.lat, args.lon, args.alt]):
        p.error('all coordinates must be finite')
    if not (-90 <= args.lat <= 90 and -180 <= args.lon <= 180):
        p.error('invalid latitude/longitude')

    import rospy
    from geographic_msgs.msg import GeoPointStamped
    from mavros_msgs.msg import State

    rospy.init_node('set_ekf_origin_once', anonymous=True)
    try:
        state = rospy.wait_for_message('/mavros/state', State, timeout=5.)
    except rospy.ROSException:
        sys.exit('ABORT: MAVROS state unavailable')
    if not state.connected or state.armed:
        sys.exit('ABORT: flight controller must be connected and DISARMED')
    pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=1)
    deadline = rospy.Time.now() + rospy.Duration(5.0)
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if rospy.Time.now() > deadline:
            sys.exit('ABORT: MAVROS global_position subscriber unavailable')
        rospy.sleep(.1)
    msg = GeoPointStamped()
    msg.header.frame_id = 'earth'
    msg.position.latitude = args.lat
    msg.position.longitude = args.lon
    msg.position.altitude = args.alt
    for _ in range(5):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.sleep(.2)
    print('SET_GPS_GLOBAL_ORIGIN sent; delivery/acceptance NOT guaranteed. Check /mavros/global_position/gp_origin and FC logs.')


if __name__ == '__main__':
    main()
