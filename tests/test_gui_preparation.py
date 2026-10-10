"""Guard/origin checks for the GUI-only startup preparation; no FC commands."""
import importlib.util
import pathlib
import types
import unittest
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sitl_gui_prepare', ROOT/'tools/sitl_gui_prepare.py')
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


class GuiPreparationTests(unittest.TestCase):
    def test_home_and_origin_acceptance_includes_altitude(self):
        home = gui.parse_home('-35.363261,149.165230,584,0')
        origin = types.SimpleNamespace(latitude=home[0], longitude=home[1], altitude=583.9986)
        self.assertTrue(gui.origin_matches(origin, home))
        origin.altitude = 585
        self.assertFalse(gui.origin_matches(origin, home))
        origin.altitude = 584
        origin.longitude += .001
        self.assertFalse(gui.origin_matches(origin, home))

    def test_parameter_event_can_record_parameter_name(self):
        task = gui.Preparation.__new__(gui.Preparation)
        task.record = {'events': []}
        task.save = lambda: None
        task.event('SITL_PARAMETER', name='ARMING_CHECK', before=0, after=1)
        self.assertEqual(task.record['events'][0]['name'], 'ARMING_CHECK')

    def test_invalid_home_rejected(self):
        for value in ('nan,149,584,0', '91,149,584,0', '0,181,584,0', '0,0,584'):
            with self.assertRaises(ValueError):
                gui.parse_home(value)

    def test_other_ros_master_rejected_before_any_service(self):
        task = gui.Preparation.__new__(gui.Preparation)
        with patch.dict(gui.os.environ, {'ROS_MASTER_URI': 'http://127.0.0.1:11321'}):
            with patch.object(gui.rospy, 'ServiceProxy') as service:
                with self.assertRaisesRegex(RuntimeError, 'Master 11311'):
                    task.identity()
                service.assert_not_called()

    def test_unsafe_velocity_source_rejected_before_mutations(self):
        task = gui.Preparation.__new__(gui.Preparation)
        task.identity = lambda: (1, 2, 3)
        task.event = lambda *a, **k: None
        task.get_param = lambda name: 6 if name == 'EK3_SRC1_VELXY' else 1
        vehicle = types.SimpleNamespace(sysid=1, compid=1, autopilot=3, type=2,
                                        flight_sw_version=0x04050700)
        response = types.SimpleNamespace(success=True, vehicles=[vehicle])
        with patch.object(gui.rospy, 'ServiceProxy', return_value=lambda **kw: response) as service:
            with self.assertRaisesRegex(RuntimeError, 'position-only configuration rejected'):
                task.run()
            self.assertEqual([call.args[0] for call in service.call_args_list], ['/mavros/vehicle_info_get'])
