"""Verify event-topic handling and fail-closed isolation without issuing commands."""
import unittest
from unittest.mock import patch
from test_pipeline_ros_messages import ROS
if ROS:
    import importlib.util
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('flight_runner',Path(__file__).resolve().parents[1]/'tools/indoor_nav_flight_test.py')
    f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)
    from mavros_msgs.msg import State
    from std_msgs.msg import String
    from indoor_nav.msg import SafetyStatus, LocalizationStatus

@unittest.skipUnless(ROS,'ROS environment required')
class FlightRunner(unittest.TestCase):
    def test_latched_mission_state_not_heartbeat(self):
        r=f.Run.__new__(f.Run);r.stage='MISSION'
        r.latest={'/mavlink/ekf_status_report':({'flags':831},10.),'/mavros/state':(State(connected=True,armed=True,mode='GUIDED'),10.),
                  '/indoor_nav/localization_status':(LocalizationStatus(healthy=True),10.),
                  '/indoor_nav/safety_status':(SafetyStatus(ready=True),10.),
                  '/indoor_nav/mission_state':(String(data='RUNNING'),1.)}
        with patch.object(f.time,'time',return_value=10.1):r.health()
        with patch.object(f.time,'time',return_value=12.), self.assertRaisesRegex(RuntimeError,'STALE:/mavros/state'):r.health()
        r.latest['/mavlink/ekf_status_report']=({'flags':1024},10.)
        with patch.object(f.time,'time',return_value=10.1), self.assertRaisesRegex(RuntimeError,'EKF_STATUS'):r.health()
        r.latest['/mavlink/ekf_status_report']=({'flags':831},10.)
        r.latest['/indoor_nav/mission_state']=(String(data='ABORT'),1.)
        with patch.object(f.time,'time',return_value=10.1), self.assertRaisesRegex(RuntimeError,'MISSION:ABORT'):r.health()

    def test_wrong_master_rejected_before_fc_services(self):
        with patch.dict(f.os.environ,{'ROS_MASTER_URI':'http://127.0.0.1:11311'}), patch.object(f.rospy,'ServiceProxy') as service:
            with self.assertRaisesRegex(RuntimeError,'Wrong ROS master'):f.identity()
            service.assert_not_called()

    def test_wrong_fcu_url_rejected_before_fc_services(self):
        with patch.dict(f.os.environ,{'ROS_MASTER_URI':f.MASTER}), patch.object(f.rospy,'get_param',return_value='/dev/ttyACM0'), patch.object(f.rospy,'ServiceProxy') as service:
            with self.assertRaisesRegex(RuntimeError,'FC URL'):f.identity()
            service.assert_not_called()
