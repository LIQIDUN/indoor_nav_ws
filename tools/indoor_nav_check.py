#!/usr/bin/env python3
"""Read-only ROS snapshot. Never sends FC commands or mission setpoints."""
import argparse
import json
import sys
import socket
socket.setdefaulttimeout(3)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--status', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    args = parser.parse_args()
    import rospy
    import rosnode
    import rosservice
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from mavros_msgs.srv import ParamGet
    from geographic_msgs.msg import GeoPointStamped
    from indoor_nav.msg import LocalizationStatus, SafetyStatus
    from std_msgs.msg import String
    from indoor_nav.core import inside_box
    # Probe before init_node to avoid indefinite master registration retries.
    rospy.get_master().getPid()
    rospy.init_node('indoor_nav_readonly_check', anonymous=True)
    failures = []
    def check(ok, title, detail=''):
        print(('PASS' if ok else 'FAIL')+' '+title+' '+str(detail))
        if not ok:
            failures.append(title)
    def sample(topic, cls):
        try:
            return rospy.wait_for_message(topic, cls, timeout=3)
        except rospy.ROSException:
            check(False, topic, 'no fresh message; check publisher/connection')
            return None
    state = sample('/mavros/state', State)
    check(bool(state and state.connected), 'FC connected')
    if state:
        print('FC armed={} mode={}'.format(state.armed, state.mode))
    if args.smoke:
        check(bool(state and not state.armed), 'DISARMED smoke prerequisite')
        if not state or state.armed:
            return 1
    cfg = rospy.get_param('/indoor_nav/safety', {})
    print('Effective safety parameters: '+json.dumps(cfg, sort_keys=True))
    check(cfg.get('profile') == cfg.get('expected_profile') and bool(cfg.get('profile')), 'profile matches launch')
    check(cfg.get('check_fc_params') is True, 'FC parameter checking enabled')
    mission = rospy.get_param('/indoor_nav/mission', {})
    frame = rospy.get_param('/indoor_nav/localization/world/frame_id', '')
    check(mission.get('frame_id') == cfg.get('frame_id') == frame, 'consistent world frame')
    for i, wp in enumerate(mission.get('waypoints', [])):
        check(inside_box(tuple(wp[k] for k in ('x','y','z')), cfg['geofence']), 'waypoint '+str(i))
    check(bool(mission.get('waypoints')), 'nonempty route')
    loc = sample('/indoor_nav/localization_status', LocalizationStatus)
    check(bool(loc and loc.healthy and loc.transform_locked), 'localization healthy')
    safety = sample('/indoor_nav/safety_status', SafetyStatus)
    if safety:
        print('Safety ready={} reason={}'.format(safety.ready, safety.reason))
    diag = sample('/indoor_nav/safety_diagnostics', String)
    if diag:
        print('Node snapshot: '+diag.data)
        loaded = json.loads(diag.data)
        check(loaded.get('geofence') == cfg.get('geofence') and loaded.get('profile') == cfg.get('profile'), 'node snapshot matches ROS params')
    pose = sample('/indoor_nav/world_pose', PoseStamped)
    if pose:
        p = pose.pose.position
        print('world XYZ: '+str((p.x,p.y,p.z)))
        check(pose.header.frame_id == frame, 'world pose frame')
    origin = sample('/mavros/global_position/gp_origin', GeoPointStamped)
    check(bool(origin and -90 <= origin.position.latitude <= 90 and -180 <= origin.position.longitude <= 180), 'EKF origin received')
    expected = rospy.get_param('/indoor_nav/ardupilot_expected', {})
    rospy.wait_for_service('/mavros/param/get', timeout=3)
    get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
    for group in expected.values():
        for name, value in group.items():
            r = get(param_id=name)
            actual = r.value.real if r.value.real != 0 else r.value.integer
            check(r.success and abs(actual-value)<.01, name, '{} expected {}'.format(actual,value))
    nodes = rosnode.get_node_names()
    for node in ('/mock_uwb_node','/world_estimator','/safety_supervisor','/mission_manager','/mavros'):
        check(node in nodes, 'node '+node)
    for service in ('/indoor_nav/safety/acknowledge','/indoor_nav/safety/reset','/indoor_nav/mission/start','/indoor_nav/mission/stop'):
        check(service in rosservice.get_service_list(), 'service '+service)
    mock = rospy.get_param('/indoor_nav/mock/ground_truth', {})
    check(mock.get('source') == 'gazebo_model_states' and mock.get('topic') == '/gazebo/model_states', 'independent Gazebo source', mock)
    if '/mock_uwb_node' in nodes:
        from rosgraph.masterapi import Master
        pubs, subs, _ = Master(rospy.get_name()).getSystemState()
        check('/mock_uwb_node' in dict(subs).get('/gazebo/model_states', []), 'actual mock subscription')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(.5)
    for parent, child, expected_q in [('odom_ned','odom',(.7071067811865476,.7071067811865476,0.,0.)),
                                      ('base_link_frd','base_link',(1.,0.,0.,0.))]:
        available = buffer.can_transform(parent, child, rospy.Time(0), rospy.Duration(2))
        check(available, 'TF '+parent+' <- '+child)
        if available:
            tr = buffer.lookup_transform(parent, child, rospy.Time(0)).transform
            q = tr.rotation
            dot = abs(sum(a*b for a,b in zip((q.x,q.y,q.z,q.w),expected_q)))
            check(abs(dot-1.) < 1e-5 and max(abs(v) for v in (tr.translation.x,tr.translation.y,tr.translation.z)) < 1e-5,
                  'TF expected axis conversion '+parent)

    print('WARN FC fusion and dynamic increments require fresh cold-start logs; this snapshot cannot certify flight.')
    return bool(failures)

if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as ex:
        print('FAIL runtime unavailable: '+str(ex))
        sys.exit(1)
