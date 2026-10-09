#!/usr/bin/env python3
"""Function-level SITL mock. Use independent Gazebo truth, NEVER FC EKF local_pose.

ROS parameters /indoor_nav/mock/uwb_sim/freeze|drop_all|jump_now can be
modified at runtime with rosparam set; no simulated radio/serial protocol.
"""
import math
import json
import random
from collections import deque
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from indoor_nav.core import finite
from nlink_parser.msg import (LinktrackNodeframe2, LinktrackAnchorframe0,
                              LinktrackTag)


class MockUwb:
    def __init__(self):
        rospy.init_node('mock_uwb_node')
        self.cfg = rospy.get_param('/indoor_nav/mock', {})
        self.base_cfg = self.cfg.get('uwb_sim', {})
        self.sim = self.cfg.get('ground_truth', {})
        self.ref = self.cfg.get('ground_tag', {})
        lc = rospy.get_param('/indoor_nav/localization', {})
        self.aircraft_id = int(lc.get('uwb', {}).get('aircraft_tag_id', 0))
        self.ground_id = int(lc.get('world', {}).get('ground_reference', {}).get('tag_id', 1))
        self.truth = None
        self.truth_stamp = None
        self.truth_error = 'TRUTH_MISSING'
        self.clock = None
        self.clock_progress_receipt = None
        self.last_diagnostic_reason = None
        self.diag_pub = rospy.Publisher('/indoor_nav/sitl/mock_diagnostics', String, queue_size=1, latch=True)
        self.last = None
        self.held = None
        self.queue = deque()
        self.rng = random.Random(int(self.base_cfg.get('seed', 41)))
        self.air_pub = rospy.Publisher('/nlink_linktrack_nodeframe2',
            LinktrackNodeframe2, queue_size=10)
        self.ground_pub = rospy.Publisher('/nlink_linktrack_anchorframe0',
            LinktrackAnchorframe0, queue_size=10)
        self.range_pub = rospy.Publisher('/indoor_nav/sitl/mock_range', Range, queue_size=10)
        self.model_name = self.sim.get('model_name', 'iris')
        self.truth_mode = self.sim.get('source', 'gazebo_model_states')
        if self.truth_mode == 'gazebo_model_states':
            rospy.Subscriber('/clock', Clock, self.clock_cb, queue_size=1)
            rospy.Subscriber(self.sim.get('topic', '/gazebo/model_states'),
                             ModelStates, self.gazebo_cb, queue_size=1)
        elif self.truth_mode == 'pose_stamped':
            topic = self.sim.get('topic', '/sim/ground_truth/pose')
            if topic.startswith('/mavros/'):
                raise ValueError('Do NOT feed mock from MAVROS EKF state (feedback loop)')
            rospy.Subscriber(topic, PoseStamped, self.pose_cb, queue_size=1)
        else:
            raise ValueError('unsupported independent ground-truth source')
        hz = float(self.base_cfg.get('rate_hz', 25.))
        rospy.Timer(rospy.Duration(1./hz), self.generate)
        rospy.Timer(rospy.Duration(.01), self.flush)

    def invalidate_truth(self, reason):
        self.truth = None
        self.truth_error = reason
        self.queue.clear()

    def clock_cb(self, msg):
        value, now = msg.clock.to_sec(), rospy.get_time()
        if self.clock is None or value != self.clock:
            self.clock_progress_receipt = now
        if self.clock is not None and value < self.clock:
            self.invalidate_truth('GAZEBO_CLOCK_REWIND')
        self.clock = value

    def gazebo_cb(self, m):
        try:
            i = m.name.index(self.model_name)
            p = m.pose[i].position
            if not finite(p.x, p.y, p.z):
                raise ValueError('nonfinite truth')
            self.truth = (p.x, p.y, p.z)
            self.truth_stamp = rospy.get_time()
            self.truth_error = ''
        except (ValueError, IndexError):
            self.invalidate_truth('GAZEBO_MODEL_MISSING_OR_INVALID')

    def pose_cb(self, m):
        p = m.pose.position
        if not finite(p.x, p.y, p.z) or not 0 <= rospy.get_time()-m.header.stamp.to_sec() <= .3:
            self.invalidate_truth('INVALID_OR_STALE_POSE_TRUTH')
            return
        self.truth_stamp = rospy.get_time()
        self.truth = (p.x, p.y, p.z)
        self.truth_error = ''

    def truth_health(self, now):
        if self.truth is None:
            return self.truth_error or 'TRUTH_MISSING'
        if self.truth_stamp is None or not 0 <= now-self.truth_stamp <= .3:
            return 'TRUTH_STALE_OR_TIME_RESET'
        if self.truth_mode == 'gazebo_model_states' and (self.clock_progress_receipt is None or
                not 0 <= now-self.clock_progress_receipt <= .3):
            return 'GAZEBO_CLOCK_STALLED_OR_MISSING'
        return 'OK'

    def diagnose(self, reason, cfg):
        now = rospy.get_time()
        data = {'stamp_s': now, 'reason': reason, 'model_name': self.model_name,
                'source': self.truth_mode, 'truth_xyz': self.truth,
                'truth_receipt_s': self.truth_stamp,
                'truth_age_s': now-self.truth_stamp if self.truth_stamp is not None else None,
                'gazebo_clock_s': self.clock,
                'clock_progress_age_s': now-self.clock_progress_receipt if self.clock_progress_receipt is not None else None,
                'queue_length': len(self.queue),
                'freeze_injected': bool(cfg.get('freeze', False)),
                'drop_all_injected': bool(cfg.get('drop_all', False)),
                'delay_ms': cfg.get('delay_ms', 0), 'held_uwb_xyz': self.held}
        payload = json.dumps(data)
        self.diag_pub.publish(String(data=payload))
        if reason != self.last_diagnostic_reason:
            rospy.logwarn('Mock publication transition: %s', payload)
            self.last_diagnostic_reason = reason

    def generate(self, _event):
        cfg = dict(self.base_cfg)
        defaults = {'freeze': False, 'drop_all': False, 'jump_now': False,
                    'dropout_probability': 0., 'noise_std_xy_m': .05}
        for p, default in defaults.items():
            cfg[p] = rospy.get_param('/indoor_nav/mock/uwb_sim/' + p, cfg.get(p, default))
        reason = self.truth_health(rospy.get_time())
        if reason != 'OK':
            self.queue.clear()
            self.diagnose(reason, cfg)
            return
        self.diagnose('FREEZE_INJECTED' if cfg.get('freeze') else 'DROP_ALL_INJECTED' if cfg.get('drop_all') else 'OK', cfg)
        px, py, pz = self.truth
        # Range observation is independent of the UWB stream and does not
        # disappear when simulating UWB loss.
        laser_cfg = self.cfg.get('mock_range', {})
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = 'sitl_range'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = .026
        range_msg.min_range = .10
        range_msg.max_range = 2.0
        range_msg.range = float(pz) + float(laser_cfg.get('bias_m', 0.0))
        self.range_pub.publish(range_msg)
        if cfg.get('drop_all', False) or self.rng.random() < float(cfg.get('dropout_probability', 0.)):
            return
        noise = float(cfg.get('noise_std_xy_m', .05))
        bx = float(cfg.get('bias_x_m', .04))
        by = float(cfg.get('bias_y_m', -.03))
        if cfg.get('freeze', False) and self.held is not None:
            pos = self.held
        else:
            pos = (px + bx + self.rng.gauss(0, noise),
                   py + by + self.rng.gauss(0, noise), pz)
            self.held = pos
        if cfg.get('jump_now', False):
            pos = (pos[0] + float(cfg.get('jump_size_m', .50)), pos[1], pos[2])
            rospy.set_param('/indoor_nav/mock/uwb_sim/jump_now', False)
        elif self.rng.random() < float(cfg.get('jump_probability', 0.)):
            pos = (pos[0] + float(cfg.get('jump_size_m', .50)), pos[1], pos[2])
        msg = LinktrackNodeframe2()
        msg.id = self.aircraft_id
        msg.role = 0
        msg.pos_3d = list(pos)
        msg.eop_3d = [max(.01, noise), max(.01, noise), .30]
        msg.vel_3d = [0., 0., 0.]
        msg.voltage = 4.0
        ground = LinktrackAnchorframe0()
        node = LinktrackTag()
        node.id = self.ground_id
        node.role = 0
        gx, gy = self.ref.get('true_xy', [.30, .30])
        node.pos_3d = [float(gx)+float(self.ref.get('bias_x_m', bx))+
                       self.rng.gauss(0., float(self.ref.get('noise_std_xy_m', .01))),
                       float(gy)+float(self.ref.get('bias_y_m', by))+
                       self.rng.gauss(0., float(self.ref.get('noise_std_xy_m', .01))), 0.]
        ground.nodes = [node]
        self.queue.append((rospy.get_time() + float(cfg.get('delay_ms', 0.))/1000., msg, ground))

    def flush(self, _event):
        now = rospy.get_time()
        if self.truth_health(now) != 'OK':
            self.queue.clear()
            return
        while self.queue and self.queue[0][0] <= now:
            _, air, ref = self.queue.popleft()
            self.air_pub.publish(air)
            self.ground_pub.publish(ref)


if __name__ == '__main__':
    MockUwb()
    rospy.spin()
