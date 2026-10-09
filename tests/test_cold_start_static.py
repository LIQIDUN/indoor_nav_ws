#!/usr/bin/env python3
"""Offline smoke checks for the cold-start estimator refactor (ROS not required)."""
import ast
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'src/indoor_nav/scripts/world_estimator.py'
CORE = ROOT / 'src/indoor_nav/src/indoor_nav/core.py'


class ColdStartSmoke(unittest.TestCase):
    def test_no_local_ekf_pose_dependency(self):
        tree = ast.parse(SRC.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'WorldEstimator')
        for func in cls.body:
            if isinstance(func, ast.FunctionDef) and func.name in ('calibrate', 'health', 'publish', 'height_m'):
                names = [n.attr for n in ast.walk(func) if isinstance(n, ast.Attribute)]
                self.assertNotIn('fc_pose', names, func.name)
                self.assertNotIn('fc_pose_stamp', names, func.name)

    def test_imu_topic_and_range_in_use(self):
        text = SRC.read_text()
        self.assertIn("'/mavros/imu/data'", text)
        self.assertIn('self.height_m()', text)
        self.assertIn('self.nav_z_offset', text)

    def test_math_transform(self):
        namespace = {'__name__': 'local_math'}
        exec(compile(CORE.read_text(), str(CORE), 'exec'), namespace)
        T = namespace['Transform2D'](0.5, 1., -2.)
        x, y = T.apply(0.3, 0.7)
        a, b = T.inverse(x, y)
        self.assertAlmostEqual(a, 0.3)
        self.assertAlmostEqual(b, 0.7)


if __name__ == '__main__':
    unittest.main()
