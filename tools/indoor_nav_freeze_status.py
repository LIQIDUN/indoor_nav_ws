#!/usr/bin/env python3
"""Read-only five-second capture for freeze / publisher / MAVROS diagnosis.
Does not reset, acknowledge, change modes, mutate Gazebo, or publish setpoints.
"""
import argparse
import time
import math
import json
import socket
import subprocess
import threading
socket.setdefaulttimeout(3)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration",type=float,default=5.)
    parser.add_argument("--interval",type=float,default=5.)
    args=parser.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (args.duration,args.interval)):
        parser.error("duration and interval must be positive")
    import rospy
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import GetPhysicsProperties
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import Imu, Range
    from mavros_msgs.msg import State
    from diagnostic_msgs.msg import DiagnosticArray
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from indoor_nav.msg import LocalizationStatus
    rospy.get_master().getPid()
    rospy.init_node('indoor_nav_freeze_readonly_capture', anonymous=True)
    model = rospy.get_param('/indoor_nav/mock/ground_truth/model_name', 'iris')
    rows={};lock=threading.Lock(); subscribers=[]
    def callback(msg, topic):
        d={'receipt_s':rospy.get_time(),'receipt_wall_s':time.time()}
        if hasattr(msg,'header'):
            d['header_s']=msg.header.stamp.to_sec();d['frame_id']=msg.header.frame_id
        if isinstance(msg,ModelStates):
            try:
                p=msg.pose[msg.name.index(model)].position
                d['xyz']=[p.x,p.y,p.z]
            except (ValueError,IndexError):d['error']='MODEL_MISSING_OR_MALFORMED'
        elif isinstance(msg,PoseStamped):
            p=msg.pose.position;d['xyz']=[p.x,p.y,p.z]
        elif isinstance(msg,TwistStamped):
            p=msg.twist.linear;d['velocity']=[p.x,p.y,p.z]
        elif isinstance(msg,State):
            d.update(connected=msg.connected,armed=msg.armed,mode=msg.mode)
        elif isinstance(msg,LocalizationStatus):
            d.update(reason=msg.reason,healthy=msg.healthy,uwb_age_s=msg.uwb_age_s,range_age_s=msg.laser_age_s)
        elif isinstance(msg,Clock):d['sim_s']=msg.clock.to_sec()
        elif isinstance(msg,Range):d['range_m']=msg.range
        elif isinstance(msg,String):d['data']=msg.data
        elif isinstance(msg,DiagnosticArray):
            d['statuses']=[{'name':s.name,'level':s.level,'message':s.message,
                            'values':{v.key:v.value for v in s.values}} for s in msg.status]
        with lock:
            rows.setdefault(topic,{'count':0,'first':d,'last':d})
            rows[topic]['count']+=1;rows[topic]['last']=d
    specs=[('/gazebo/model_states',ModelStates),('/clock',Clock),('/mavros/state',State),
           ('/mavros/imu/data',Imu),('/mavros/local_position/velocity_local',TwistStamped),
           ('/indoor_nav/world_pose',PoseStamped),('/indoor_nav/localization_status',LocalizationStatus),
           ('/indoor_nav/sitl/mock_range',Range),('/indoor_nav/abort_snapshot',String),
           ('/indoor_nav/localization_diagnostics',String),('/indoor_nav/sitl/mock_diagnostics',String),
           ('/indoor_nav/accepted_uwb_details',String),('/indoor_nav/uwb_velocity/status',String),
           ('/diagnostics',DiagnosticArray)]
    for topic,cls in specs:
        subscribers.append(rospy.Subscriber(topic,cls,callback,callback_args=topic,queue_size=1))
    checkpoints=[]
    deadline=time.monotonic()+args.duration
    while time.monotonic()<deadline and not rospy.is_shutdown():
        rospy.rostime.wallsleep(min(args.interval,max(0.,deadline-time.monotonic())))
        with lock:
            checkpoints.append({'wall_s':time.time(),'topics':{t:{'count':r['count'],
                'receipt_age_wall_s':time.time()-r['last']['receipt_wall_s'],
                'last':dict(r['last'])} for t,r in rows.items()}})
        proc=subprocess.run(['ps','-eo','pid,comm,stat,pcpu,etime'],text=True,capture_output=True)
        checkpoints[-1]['processes']=[l for l in proc.stdout.splitlines() if any(n in l.split() for n in ('arducopter','gzserver','mavros_node'))]
        for kind,command in [('udp',['ss','-unap']),('tcp',['ss','-tnap'])]:
            result=subprocess.run(command,text=True,capture_output=True)
            checkpoints[-1][kind]=[l for l in result.stdout.splitlines() if any(p in l for p in (':9002 ',':9003 ',':5760 ',':14552 '))]
    with lock:
        data={'capture_s':rospy.get_time(),'checkpoints':checkpoints,'topics':dict(rows),'missing_topics':[t for t,_ in specs if t not in rows]}
    try:
        rospy.wait_for_service('/gazebo/get_physics_properties',timeout=1.)
        physics=rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)()
        data['gazebo_physics']={'pause':physics.pause,'time_step':physics.time_step,
                                'max_update_rate':physics.max_update_rate,'success':physics.success}
    except Exception as ex:data['gazebo_physics_error']=str(ex)
    processes=subprocess.run(['ps','-eo','pid,ppid,comm,stat,pcpu,etime,wchan:24'],check=True,text=True,capture_output=True)
    names=('arducopter','gzserver','gzclient','mavros_node','rosmaster','mavproxy.py')
    data['processes']=[l for l in processes.stdout.splitlines() if any(n in l.split() for n in names)]
    for kind, command in [('udp_sockets', ['ss','-unap']), ('tcp_sockets', ['ss','-tnap'])]:
        result=subprocess.run(command,text=True,capture_output=True,check=False)
        data[kind]=[line for line in result.stdout.splitlines()
                    if any(token in line for token in (':9002 ',':9003 ',':14552 ',':5760 '))]
    print(json.dumps(data,indent=2,sort_keys=True))

if __name__=='__main__':
    try:main()
    except Exception as ex:
        print('FAIL readonly capture: '+str(ex))
        raise SystemExit(1)
