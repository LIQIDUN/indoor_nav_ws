import math
import unittest
from types import SimpleNamespace as N
from indoor_nav.navigation_velocity import paired_odometry_velocity


def sample(roll=0., pitch=0., yaw=0., v=(1., 0., 0.)):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    q = N(x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy,
          z=cr*cp*sy-sr*sp*cy, w=cr*cp*cy+sr*sp*sy)
    return N(header=N(frame_id='map'), child_frame_id='base_link',
             pose=N(pose=N(orientation=q)),
             twist=N(twist=N(linear=N(x=v[0], y=v[1], z=v[2]))))


class NavigationVelocity(unittest.TestCase):
    def test_yaws(self):
        for yaw in (0., math.pi/2, math.pi, .731):
            with self.subTest(yaw=yaw):
                out = paired_odometry_velocity(sample(yaw=yaw), 'map', 'base_link')
                for a,b in zip(out, (math.cos(yaw), math.sin(yaw), 0.)):
                    self.assertAlmostEqual(a,b)

    def test_pitch_horizontal_projection(self):
        out = paired_odometry_velocity(sample(pitch=.4), 'map', 'base_link')
        self.assertAlmostEqual(out[0], math.cos(.4))
        self.assertAlmostEqual(out[2], -math.sin(.4))

    def test_roll_horizontal_projection(self):
        out = paired_odometry_velocity(sample(roll=.6,v=(0.,1.,0.)), 'map', 'base_link')
        self.assertAlmostEqual(out[1], math.cos(.6))
        self.assertAlmostEqual(out[2], math.sin(.6))

    def test_combined_attitude_inverse_and_norm(self):
        msg = sample(roll=.3,pitch=-.5,yaw=.731,v=(.7,-.4,.2))
        out = paired_odometry_velocity(msg, 'map', 'base_link')
        self.assertAlmostEqual(sum(a*a for a in out), .69)
        q=msg.pose.pose.orientation
        q.x,q.y,q.z = -q.x,-q.y,-q.z
        msg.twist.twist.linear=N(x=out[0],y=out[1],z=out[2])
        back=paired_odometry_velocity(msg,'map','base_link')
        for a,b in zip(back,(.7,-.4,.2)): self.assertAlmostEqual(a,b)

    def test_reject_bad_frames_nan_and_quaternion(self):
        for kind in ('parent','child','nan','quaternion','configuration'):
            msg=sample()
            if kind=='parent': msg.header.frame_id='base_link'
            if kind=='child': msg.child_frame_id='camera'
            if kind=='nan': msg.twist.twist.linear.x=float('nan')
            if kind=='quaternion': msg.pose.pose.orientation.w=0.
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                paired_odometry_velocity(msg,'base_link' if kind=='configuration' else 'map','base_link')
