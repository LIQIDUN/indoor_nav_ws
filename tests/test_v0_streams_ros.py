import importlib.util
import os
import unittest
from unittest.mock import Mock, patch

from test_pipeline_ros_messages import ROS
from test_v0 import ROOT

if ROS:
    import roslaunch
    from mavros_msgs.msg import State
    from mavros_msgs.srv import StreamRateRequest, StreamRateResponse
    spec = importlib.util.spec_from_file_location('v0_streams', ROOT / 'src/indoor_nav/scripts/v0_request_streams.py')
    streams = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(streams)


@unittest.skipUnless(ROS, 'source ROS Noetic for actual generated messages')
class StreamRequests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.parameters = {}
        for name, options in (
                ('get_param', dict(side_effect=lambda key, default: self.parameters.get(key, default))),
                ('Subscriber', {}), ('on_shutdown', {}), ('loginfo', {}), ('logwarn', {}),
                ('wait_for_service', {})):
            mocked = patch.object(streams.rospy, name, **options)
            mocked.start()
            self.addCleanup(mocked.stop)
        self.client = Mock(return_value=StreamRateResponse())
        proxy = patch.object(streams.rospy, 'ServiceProxy', return_value=self.client)
        self.proxy = proxy.start()
        self.addCleanup(proxy.stop)
        clock = patch.object(streams.time, 'monotonic', side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        self.requester = streams.StreamRequester()

    def connect(self):
        self.requester.on_state(State(connected=True))
        self.now += 2.
        self.requester.step()

    def test_five_groups_correct_rates_empty_response_no_controls(self):
        self.requester.step()
        self.client.assert_not_called()
        self.requester.on_state(State(connected=True))
        self.requester.step()
        self.client.assert_not_called()
        self.now += 2.
        self.requester.step()
        requests = [entry.args[0] for entry in self.client.call_args_list]
        self.assertEqual([(request.stream_id, request.message_rate, request.on_off) for request in requests],
                         [(2, 5, True), (6, 10, True), (10, 10, True), (12, 5, True), (1, 10, True)])
        self.assertEqual(StreamRateRequest.STREAM_EXTENDED_STATUS, 2)
        self.assertEqual(StreamRateResponse.__slots__, [])
        self.assertTrue(self.requester.configured)
        self.requester.step()
        self.requester.on_state(State(connected=True))
        self.requester.step()
        self.assertEqual(self.client.call_count, 5)
        self.proxy.assert_called_once_with('/mavros/set_stream_rate', streams.StreamRate)

    def test_reconnect(self):
        self.connect()
        self.requester.on_state(State(connected=False))
        self.requester.step()
        self.assertFalse(self.requester.configured)
        self.connect()
        self.assertEqual(self.client.call_count, 10)

    def test_partial_failure_retries_after_delay(self):
        self.client.side_effect = [StreamRateResponse(), streams.rospy.ServiceException('unavailable')]
        self.connect()
        self.assertFalse(self.requester.configured)
        self.client.side_effect = None
        self.requester.step()
        self.assertEqual(self.client.call_count, 2)
        self.now += 5.
        self.requester.step()
        self.assertTrue(self.requester.configured)
        self.assertEqual(self.client.call_count, 7)

    def test_wait_for_service_timeout_is_retryable(self):
        streams.rospy.wait_for_service.side_effect = streams.rospy.ROSException('timeout')
        self.connect()
        self.client.assert_not_called()
        self.assertFalse(self.requester.configured)
        streams.rospy.wait_for_service.side_effect = None
        self.now += 5.
        self.requester.step()
        self.assertEqual(self.client.call_count, 5)

    def test_zero_skips_and_invalid_config_rejected(self):
        self.parameters['~extended_status_hz'] = 0
        self.requester = streams.StreamRequester()
        self.connect()
        self.assertEqual(self.client.call_count, 4)
        for key, value in (('~position_hz', -1), ('~position_hz', 65536),
                           ('~position_hz', 1.5), ('~position_hz', True),
                           ('~retry_s', 0), ('~connection_settle_s', float('nan'))):
            self.parameters = {key: value}
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                streams.StreamRequester()

    def test_disconnect_reconnect_during_request_does_not_mark_new_link_configured(self):
        def reconnect(_request):
            self.requester.on_state(State(connected=False))
            self.requester.on_state(State(connected=True))
            return StreamRateResponse()
        self.client.side_effect = reconnect
        self.connect()
        self.assertFalse(self.requester.configured)
        self.assertEqual(self.client.call_count, 1)
        self.client.side_effect = None
        self.now += 2.
        self.requester.step()
        self.assertTrue(self.requester.configured)
        self.assertEqual(self.client.call_count, 6)

    def test_script_executable_installed_and_no_control_apis(self):
        path = ROOT / 'src/indoor_nav/scripts/v0_request_streams.py'
        self.assertTrue(os.access(path, os.X_OK))
        self.assertIn('scripts/v0_request_streams.py', (ROOT / 'src/indoor_nav/CMakeLists.txt').read_text())
        for forbidden in ('.success', 'Publisher(', 'SetMode', 'CommandBool', 'CommandTOL',
                          'pymavlink', 'serial.Serial', '/cmd/arming', 'setpoint_'):
            self.assertNotIn(forbidden, path.read_text())


@unittest.skipUnless(ROS, 'ROS launch parser required')
class OnboardLaunch(unittest.TestCase):
    def load(self, *arguments):
        configuration = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(ROOT / 'src/indoor_nav/launch/indoor_nav_real_v0.launch'),
            configuration, argv=list(arguments), verbose=False)
        return configuration

    def test_default_single_mavros_and_readonly_level_config(self):
        configuration = self.load()
        self.assertEqual(sum(node.package == 'mavros' for node in configuration.nodes), 1)
        for name in ('v0_nav_manager', 'v0_nav_watchdog'):
            self.assertIs(configuration.params['/' + name + '/enable_control'].value, False)
            self.assertIs(configuration.params['/' + name + '/dry_run'].value, True)
            self.assertTrue(configuration.params['/' + name + '/mission_file'].value.endswith('/mission_v0_level.yaml'))
        requester = next(node for node in configuration.nodes if node.name == 'v0_request_streams')
        self.assertFalse(requester.respawn)

    def test_existing_mavros_and_mission_override_are_forwarded(self):
        configuration = self.load('start_mavros:=false', 'configure_streams:=false',
                                  'mission_file:=/tmp/reviewed.yaml', 'evidence_dir:=/tmp/v0-review',
                                  'pose_topic:=/test/pose')
        self.assertEqual({node.name for node in configuration.nodes}, {'v0_nav_manager', 'v0_nav_watchdog'})
        for name in ('v0_nav_manager', 'v0_nav_watchdog'):
            self.assertEqual(configuration.params['/' + name + '/mission_file'].value, '/tmp/reviewed.yaml')
            self.assertEqual(configuration.params['/' + name + '/evidence_dir'].value, '/tmp/v0-review')
        for node in configuration.nodes:
            self.assertIn(['/mavros/local_position/pose', '/test/pose'], node.remap_args)
