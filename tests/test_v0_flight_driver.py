import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from test_pipeline_ros_messages import ROS

if ROS:
    spec = importlib.util.spec_from_file_location('v0_flight_driver', Path(__file__).resolve().parents[1] / 'tools/v0_sitl_test.py')
    driver = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(driver)


@unittest.skipUnless(ROS, 'ROS messages required')
class FlightDriver(unittest.TestCase):
    def test_single_and_yaw_do_not_fall_through_to_multi(self):
        for case in ('single', 'yaw', 'finish_stop'):
            self.assertEqual(len(driver.test_waypoints(case)), 1)
            self.assertEqual(driver.test_waypoints(case)[0]['x'], .25)

    def test_multi_altitude_and_distances(self):
        self.assertEqual(len(driver.test_waypoints('multi')), 4)
        self.assertEqual([point['z'] for point in driver.test_waypoints('altitude')], [.1, 0.])
        self.assertEqual(driver.test_waypoints('distance_short')[0]['x'], .15)
        self.assertEqual(driver.test_waypoints('distance_long')[0]['x'], .5)

    def test_live_recorder_waits_for_active_file_not_closed_bag(self):
        process = Mock()
        process.poll.return_value = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'after_recorder.bag').write_bytes(b'closed')
            self.assertEqual(driver.active_bag_bytes(path, process), 0)
            (path / 'after_recorder.bag.active').write_bytes(b'live')
            self.assertEqual(driver.active_bag_bytes(path, process), 4)
            process.poll.return_value = 1
            with self.assertRaises(RuntimeError):
                driver.active_bag_bytes(path, process)
