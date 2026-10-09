"""Decode MAVROS local_position Odometry's paired FLU twist and ENU pose.

MAVROS 1.20.1 constructs twist = R(pose)^-1 * LOCAL_POSITION_NED_ENU
in one callback. Using that same pose is an inverse encoding operation, not
interpolation of an independently sampled attitude. Never apply this operation
to velocity_local, whose numeric values are already ENU despite its header.
"""
import math


def paired_odometry_velocity(msg, navigation_frame, body_frame):
    if navigation_frame not in ('map', 'odom') or body_frame != 'base_link':
        raise ValueError('UNSUPPORTED_VELOCITY_CONFIGURATION')
    if msg.header.frame_id != navigation_frame or msg.child_frame_id != body_frame:
        raise ValueError('INVALID_VELOCITY_FRAME')
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    x, y, z, w = q.x, q.y, q.z, q.w
    if not all(math.isfinite(a) for a in (x, y, z, w, v.x, v.y, v.z)):
        raise ValueError('NONFINITE_VELOCITY_OR_ATTITUDE')
    if abs(x*x+y*y+z*z+w*w-1.) > 1e-3:
        raise ValueError('INVALID_VELOCITY_ATTITUDE')
    return ((1-2*(y*y+z*z))*v.x + 2*(x*y-z*w)*v.y + 2*(x*z+y*w)*v.z,
            2*(x*y+z*w)*v.x + (1-2*(x*x+z*z))*v.y + 2*(y*z-x*w)*v.z,
            2*(x*z-y*w)*v.x + 2*(y*z+x*w)*v.y + (1-2*(x*x+y*y))*v.z)
