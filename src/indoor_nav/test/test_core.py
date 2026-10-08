"""Run with: PYTHONPATH=src python3 -m unittest discover -s test -v"""
import math
import unittest
from indoor_nav.core import (Transform2D, UwbGate, GroundCalibration,
                             inside_box, bounded_step, quaternion_yaw,
                             yaw_quaternion, quaternion_product, rotate_vector)


class TestCore(unittest.TestCase):
    def test_transform_roundtrip(self):
        for theta in (0., .2, -1., math.pi/2, 2.7):
            trans = Transform2D(theta, .5, -2.)
            for xy in ((0., 0.), (.3, 1.6), (-1., 5.)):
                self.assertAlmostEqual(trans.inverse(*trans.apply(*xy))[0], xy[0], places=8)
                self.assertAlmostEqual(trans.inverse(*trans.apply(*xy))[1], xy[1], places=8)

    def test_yaw_rotation(self):
        q = quaternion_product(yaw_quaternion(.3), yaw_quaternion(.2))
        self.assertAlmostEqual(quaternion_yaw(q), .5, places=8)

    def test_tag_extrinsic_rotates_with_aircraft_yaw(self):
        dx, dy, dz = rotate_vector(yaw_quaternion(math.pi/2), (.15,0.,0.))
        self.assertAlmostEqual(dx, 0., places=8)
        self.assertAlmostEqual(dy, .15, places=8)
        self.assertAlmostEqual(dz, 0., places=8)

    def test_gate_jump_and_timeout(self):
        g = UwbGate(max_jump=.3, max_speed=1.5, timeout=.2)
        self.assertTrue(g.evaluate(1., 1., 1., .07))
        self.assertTrue(g.evaluate(1.04, 1.05, 1.0, .07))
        self.assertFalse(g.evaluate(1.08, 1.65, 1.0, .07))
        self.assertEqual(g.reason, 'JUMP')
        self.assertTrue(g.fresh(1.10))
        self.assertFalse(g.fresh(1.30))

    def test_eop(self):
        g = UwbGate(max_eop=.3)
        self.assertFalse(g.evaluate(1., 0., 0., .8))

    def test_ground_bias(self):
        c = GroundCalibration((.3, .3), required_samples=4, max_offset=.3)
        for k in range(20):
            c.add((.37+((-1)**k)*.001, .26))
        bx, by = c.result()
        self.assertAlmostEqual(bx, .07)
        self.assertAlmostEqual(by, -.04)

    def test_outlier_reference(self):
        c = GroundCalibration((0., 0.), required_samples=2, max_std=.02)
        c.add((0,0))
        c.add((.5,0))
        with self.assertRaises(ValueError):
            c.result()

    def test_box(self):
        box = {'x_min': .4, 'x_max': 2.6, 'y_min': .4,
               'y_max': 2.6, 'z_min': .3, 'z_max': 1.3}
        self.assertTrue(inside_box((1.,1.,.8), box))
        self.assertFalse(inside_box((2.9,1.,.8), box))
        self.assertFalse(inside_box((1.,1.,float('nan')), box))

    def test_step_limiter(self):
        s = bounded_step((0., 0., 0.), (1., 1., 1.), .05, .30, .20)
        self.assertLessEqual(math.hypot(s[0], s[1]), .015000001)
        self.assertAlmostEqual(s[2], .01)


if __name__ == '__main__':
    unittest.main()
