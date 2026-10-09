#!/usr/bin/env python3
"""Read-only synchronized ROS/MAVLink frequency audit, JSONL + JSON summary.
Never sends MAVLink or calls mutation services. Wall-time sampling survives /clock stalls.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
socket.setdefaulttimeout(2.)


def summarize(times,start,end):
    gaps=[b-a for a,b in zip(times,times[1:])]
    return {'count':len(times),'observed_hz':(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
            'count_per_capture_s':len(times)/(end-start),
            'max_internal_gap_s':max(gaps) if gaps else None,
            'start_silence_s':times[0]-start if times else end-start,
            'end_silence_s':end-times[-1] if times else end-start}


def machine_snapshot():
    result={}
    commands={'processes':['ps','-eo','pid,ppid,comm,stat,pcpu,etime,args'],
              'udp':['ss','-unap'],'tcp':['ss','-tnap']}
    for key,cmd in commands.items():
        p=subprocess.run(cmd,text=True,capture_output=True,timeout=3)
        if key=='processes':
            names=('arducopter','gzserver','mavros_node','mavproxy.py','sim_vehicle.py','world_estimator.py','QGroundControl')
            result[key]=[l for l in p.stdout.splitlines() if any(n in l.split()[2] or any(a.endswith('/'+n) or a==n for a in l.split()[6:]) for n in names)]
        else:
            result[key]=[l for l in p.stdout.splitlines() if any(port in l for port in (':5760 ',':14550 ',':14551 ',':14552 ',':9002 ',':9003 '))]
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration',type=float,default=60.)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--read-params',action='store_true')
    args=p.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=86400:p.error('duration must be in (0,86400]')
    import rospy
    from sensor_msgs.msg import Imu,Range
    from geometry_msgs.msg import PoseStamped,TwistStamped,PointStamped,TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from mavros_msgs.msg import State,Mavlink
    from rosgraph_msgs.msg import Clock
    from gazebo_msgs.msg import ModelStates
    from nlink_parser.msg import LinktrackNodeframe2
    from indoor_nav.msg import LocalizationStatus
    rospy.get_master().getPid()
    rospy.init_node('indoor_nav_mavlink_readonly_audit',anonymous=True,disable_signals=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    lock=threading.Lock();times=collections.defaultdict(list);last={};transitions=[];last_reason=None
    stop=threading.Event()
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    start=time.monotonic();wall_start=time.time()
    stream=args.output.open('w')
    def write(row):
        stream.write(json.dumps(row,allow_nan=False)+'\n')
    def callback(msg,topic):
        nonlocal last_reason
        mono=time.monotonic();now=rospy.get_time()
        row={'topic':topic,'elapsed_s':mono-start,'wall_s':time.time(),'ros_s':now}
        key=topic
        if hasattr(msg,'header'):
            stamp=msg.header.stamp.to_sec()
            row.update(header_s=stamp,header_age_s=now-stamp,frame_id=msg.header.frame_id)
        if isinstance(msg,Mavlink):
            key=topic+':{}:{}:{}'.format(msg.sysid,msg.compid,msg.msgid)
            row.update(msgid=msg.msgid,sysid=msg.sysid,compid=msg.compid,seq=msg.seq)
            payload=struct.pack('<{}Q'.format(len(msg.payload64)),*msg.payload64)[:msg.len]
            if msg.msgid==66 and len(payload)>=6:
                rate,ts,tc,sid,on=struct.unpack('<HBBBB',payload[:6])
                row['request_stream']={'rate':rate,'target_system':ts,'target_component':tc,'stream_id':sid,'on':on}
            if msg.msgid==76 and len(payload)>=33:
                values=struct.unpack('<7fHBBB',payload[:33])
                row['command']={'command':values[7],'p1':values[0],'p2':values[1]}
        elif isinstance(msg,String):
            try:row['data']=json.loads(msg.data)
            except ValueError:row['data']=msg.data
        elif isinstance(msg,State):row.update(connected=msg.connected,armed=msg.armed,mode=msg.mode)
        elif isinstance(msg,LocalizationStatus):row.update(healthy=msg.healthy,reason=msg.reason,connected=msg.flight_controller_connected)
        elif isinstance(msg,Clock):row['sim_s']=msg.clock.to_sec()
        elif isinstance(msg,LinktrackNodeframe2):row.update(device_local=msg.local_time,device_system=msg.system_time)
        # Do not serialize arbitrary floating payloads: diagnostics may contain NaN.
        with lock:
            times[key].append(mono);last[topic]=row
            if isinstance(msg,LocalizationStatus) and msg.reason!=last_reason:
                transitions.append(row);last_reason=msg.reason
            try:write(row)
            except ValueError:
                write({'topic':topic,'elapsed_s':mono-start,'serialization':'NONFINITE_DIAGNOSTIC','ros_s':now})
    specs=[('/mavros/imu/data',Imu),('/mavros/imu/data_raw',Imu),
           ('/mavros/local_position/velocity_local',TwistStamped),('/mavros/state',State),
           ('/mavros/odometry/out',Odometry),('/indoor_nav/ap_odom',Odometry),
           ('/indoor_nav/world_pose',PoseStamped),('/indoor_nav/accepted_uwb',PointStamped),
           ('/nlink_linktrack_nodeframe2',LinktrackNodeframe2),('/indoor_nav/sitl/mock_range',Range),
           ('/indoor_nav/localization_status',LocalizationStatus),('/indoor_nav/localization_diagnostics',String),
           ('/indoor_nav/publication_events',String),('/indoor_nav/uwb_velocity/status',String),
           ('/indoor_nav/uwb_velocity/world',TwistWithCovarianceStamped),
           ('/mavlink/from',Mavlink),('/mavlink/to',Mavlink)]
    # Clock/truth are downsampled to avoid recording hundreds of Hz unnecessarily.
    clock_last={}
    def downsample(msg,topic):
        now=time.monotonic()
        if now-clock_last.get(topic,0)>=.1:clock_last[topic]=now;callback(msg,topic)
    subs=[rospy.Subscriber(t,c,callback,callback_args=t,queue_size=500) for t,c in specs]
    subs += [rospy.Subscriber(t,c,downsample,callback_args=t,queue_size=1) for t,c in [('/clock',Clock),('/gazebo/model_states',ModelStates)]]
    metadata={'kind':'metadata','wall_start_s':wall_start,'duration_requested_s':args.duration,
              'params':{},'published_topics':rospy.get_published_topics(),'system':machine_snapshot()}
    for name in ('/use_sim_time','/mavros/fcu_url','/mavros/gcs_url','/mavros/conn','/mavros/plugin_blacklist','/mavros/plugin_whitelist',
                 '/indoor_nav/localization','/indoor_nav/safety/profile','/indoor_nav/sitl_experiments'):
        metadata['params'][name]=rospy.get_param(name,None)
    if args.read_params:
        from mavros_msgs.srv import ParamGet,VehicleInfoGet
        rospy.wait_for_service('/mavros/param/get',timeout=3)
        get=rospy.ServiceProxy('/mavros/param/get',ParamGet)
        names=['SIM_SPEEDUP','EK3_SRC1_VELXY','EK3_SRC1_POSXY','EK3_SRC1_POSZ','VISO_POS_M_NSE']
        names += ['SR{}_{}'.format(i,g) for i in range(4) for g in ('RAW_SENS','EXT_STAT','POSITION','EXTRA1','EXTRA2','EXTRA3')]
        metadata['fc_params_source']='MAVROS param/get cache; not forced FC readback'
        metadata['fc_params']={}
        for name in names:
            r=get(param_id=name)
            metadata['fc_params'][name]=(r.value.real if r.value.real else r.value.integer) if r.success else None
        rospy.wait_for_service('/mavros/vehicle_info_get',timeout=3)
        info=rospy.ServiceProxy('/mavros/vehicle_info_get',VehicleInfoGet)(sysid=0,compid=0,get_all=False)
        metadata['vehicles']=[{'sysid':v.sysid,'compid':v.compid,'autopilot':v.autopilot,'type':v.type,'version':v.flight_sw_version} for v in info.vehicles]
    with lock:write(metadata)
    next_checkpoint=start
    while time.monotonic()-start<args.duration and not rospy.is_shutdown() and not stop.is_set():
        if time.monotonic()>=next_checkpoint:
            snap={'kind':'checkpoint','elapsed_s':time.monotonic()-start,'system':machine_snapshot()}
            with lock:
                snap['last']={t:{'elapsed_s':r['elapsed_s'],'age_wall_s':snap['elapsed_s']-r['elapsed_s']} for t,r in last.items()}
                write(snap);stream.flush()
            next_checkpoint=time.monotonic()+10
        time.sleep(.2)
    for sub in subs:sub.unregister()
    end=time.monotonic()
    with lock:
        summary={'wall_start_s':wall_start,'duration_s':end-start,'ended_by_signal':stop.is_set(),'rates':{t:summarize(v,start,end) for t,v in times.items()},
                 'localization_transitions':transitions,'last':last,'metadata':metadata}
        args.output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
        stream.close()
    print(json.dumps({'summary':str(args.output.with_suffix('.summary.json')),'duration_s':end-start}))

if __name__=='__main__':main()
