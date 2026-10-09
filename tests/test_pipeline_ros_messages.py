"""No master, FC or Gazebo needed: actual callbacks and generated ROS messages.
Run after catkin_make and sourcing devel/setup.bash.
"""
import importlib.util
import pathlib
import unittest
from unittest.mock import patch
try:
    import rospy
    import yaml
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import Imu
    from mavros_msgs.msg import State
    from rosgraph_msgs.msg import Clock
    ROS = True
except ImportError:
    ROS = False
ROOT = pathlib.Path(__file__).resolve().parents[1]

def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'src/indoor_nav/scripts'/(name+'.py'))
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj

class Pub:
    def __init__(self, *args, **kwargs):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg)

@unittest.skipUnless(ROS, 'source ROS + devel/setup.bash for real-message pipeline')
class Pipeline(unittest.TestCase):
    def test_truth_to_mock_to_world_to_odometry_and_loss(self):
        cfg = ROOT/'src/indoor_nav/config'
        loc = yaml.safe_load((cfg/'localization.yaml').read_text())
        loc['world']['anchor_map_verified'] = True
        loc['height']['range_to_body_z_m'] = -.15
        loc['output']['external_nav_enabled'] = True
        mock = yaml.safe_load((cfg/'sitl.yaml').read_text())
        mock['uwb_sim'].update(noise_std_xy_m=0.,delay_ms=0.)
        mock['ground_tag']['noise_std_xy_m'] = 0.
        now = [1.]
        params = {'/indoor_nav/localization':loc,'/indoor_nav/mock':mock}
        def param(name, default=None):
            return params.get(name, default)
        with patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(now[0])), patch.object(rospy,'get_param',side_effect=param), patch.object(rospy,'get_time',side_effect=lambda:now[0]), patch.object(rospy,'init_node'), patch.object(rospy,'Publisher',Pub), patch.object(rospy,'Subscriber'), patch.object(rospy,'Service'), patch.object(rospy,'Timer'), patch.object(rospy,'logwarn'), patch.object(rospy,'logwarn_throttle'):
            w = module('world_estimator').WorldEstimator()
            m = module('mock_uwb_node').MockUwb()
            pose = Pose(); pose.orientation.w=1.
            truth = ModelStates(name=['iris'],pose=[pose])
            imu = Imu(); imu.orientation.w=1.
            state = State(connected=True,armed=False)
            def cycle():
                m.clock_cb(Clock(clock=rospy.Time.from_sec(now[0]))); m.gazebo_cb(truth); m.generate(None); m.flush(None)
                w.uwb_cb(m.air_pub.messages[-1]); w.ground_cb(m.ground_pub.messages[-1])
                imu.header.stamp=rospy.Time.from_sec(now[0])
                w.range_cb(m.range_pub.messages[-1]); w.imu_cb(imu); w.state_cb(state)
            for i in range(210):
                now[0]=1+i*.04; cycle()
            self.assertIsNone(w.fc_pose)
            self.assertTrue(w.calibrate(None).success)
            w.publish(None)
            a=w.fc_odom_pub.messages[-1].pose.pose.position
            self.assertAlmostEqual(w.world_pub.messages[-1].pose.position.z,0.)
            for i in range(1,11):
                now[0]+=.04
                pose.position.x=.01*i; pose.position.y=.005*i
                cycle(); w.publish(None)
            b=w.fc_odom_pub.messages[-1].pose.pose.position
            self.assertAlmostEqual(b.x-a.x,.1)
            self.assertAlmostEqual(b.y-a.y,.05)
            self.assertEqual(w.fc_odom_pub.messages[-1].pose.covariance[14],1e3)
            count=len(m.air_pub.messages)
            now[0]+=.4; m.generate(None); m.flush(None)
            self.assertEqual(len(m.air_pub.messages),count)
            self.assertFalse(w.health(now[0])[0])
            count=len(w.fc_odom_pub.messages); w.publish(None)
            self.assertEqual(len(w.fc_odom_pub.messages),count)
if __name__ == '__main__':
    unittest.main()
