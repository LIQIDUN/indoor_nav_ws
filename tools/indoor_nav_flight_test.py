#!/usr/bin/env python3
"""Small isolated-SITL flight runner. No automatic operator acknowledgement.

Start tools/sitl_flight_instance.sh in a persistent service first. prepare does
identity checks, SIM-only log/prearm settings, origin and UWB calibration.
fly requires the existing supervisor genuinely READY after human ACK.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client
import psutil
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, EstimatorStatus, StatusText, Mavlink
from mavros.mavlink import convert_to_bytes
from pymavlink.dialects.v20 import ardupilotmega as dialect
from mavros_msgs.srv import ParamGet, ParamSet, CommandBool, CommandTOL, SetMode, VehicleInfoGet, CommandLong
from mavros_msgs.msg import ParamValue
from std_msgs.msg import String
from std_srvs.srv import Trigger
from indoor_nav.msg import SafetyStatus, LocalizationStatus

WS=Path(__file__).resolve().parents[1]
RUNTIME=WS/'.sitl_runtime/flight_instance'
MASTER='http://127.0.0.1:11321'
AP=Path.home()/'github/ardupilot/build/sitl/bin/arducopter'


def identity():
    """Verify actual local processes, ROS service provider and socket topology."""
    if os.environ.get('ROS_MASTER_URI') != MASTER:
        raise RuntimeError('Wrong ROS master; expected '+MASTER)
    if rospy.get_param('/mavros/fcu_url') != 'udp://127.0.0.1:14572@':
        raise RuntimeError('FC URL is not isolated loopback endpoint')
    if rospy.get_param('/indoor_nav/safety/profile') != 'sitl':
        raise RuntimeError('Not SITL safety profile')
    p=psutil.Process(int((RUNTIME/'arducopter.pid').read_text()))
    if Path(p.exe()).resolve()!=AP.resolve() or Path(p.cwd()).resolve()!=RUNTIME.resolve():
        raise RuntimeError('SITL executable or working directory mismatch')
    if '--model' not in p.cmdline() or 'gazebo' not in p.cmdline():
        raise RuntimeError('Not Gazebo SITL')
    proxy=psutil.Process(int((RUNTIME/'mavproxy.pid').read_text()))
    cmd=proxy.cmdline()
    if cmd.count('--out=127.0.0.1:14572')!=1 or '--master=tcp:127.0.0.1:5760' not in cmd:
        raise RuntimeError('Wrong MAVProxy route')
    if sum(a.startswith('--master') for a in cmd)!=1 or sum(a.startswith('--out') for a in cmd)!=1:
        raise RuntimeError('Multiple MAVProxy routes forbidden')
    conns=proxy.connections(kind='inet')
    if not any(c.raddr and c.raddr.ip=='127.0.0.1' and c.raddr.port==5760 for c in conns):
        raise RuntimeError('No verified MAVProxy-to-local-SITL socket')
    for proc in (p,proxy):
        if any(c.raddr and c.raddr.ip not in ('127.0.0.1','::1') for c in proc.connections(kind='inet')):
            raise RuntimeError('Non-loopback simulator peer')
    master=rosgraph.Master(rospy.get_name())
    uri=master.lookupNode('/mavros')
    code,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
    launch=psutil.Process(int((RUNTIME/'mavros.pid').read_text()))
    if code!=1 or pid not in [c.pid for c in launch.children(recursive=True)]:
        raise RuntimeError('MAVROS node is not owned by isolated launcher')
    mp=psutil.Process(pid)
    if any(c.type==socket.SOCK_DGRAM and c.raddr and c.raddr.ip not in ('127.0.0.1','::1') for c in mp.connections(kind='inet')):
        raise RuntimeError('Non-loopback MAVROS peer')
    if not any(c.type==socket.SOCK_DGRAM and c.laddr.ip=='127.0.0.1' and c.laddr.port==14572 for c in mp.connections(kind='inet')):
        raise RuntimeError('MAVROS has no isolated loopback MAVLink socket')
    for srv in ('/mavros/cmd/arming','/mavros/cmd/takeoff','/mavros/set_mode','/mavros/param/set','/mavros/cmd/command','/mavros/set_message_interval'):
        if master.lookupService(srv)!=master.lookupService('/mavros/cmd/arming'):
            raise RuntimeError('FC command services have different providers')
    st=rospy.wait_for_message('/mavros/state',State,timeout=5)
    if not st.connected: raise RuntimeError('FC disconnected')
    info=rospy.ServiceProxy('/mavros/vehicle_info_get',VehicleInfoGet)(sysid=0,compid=0,get_all=False)
    if not info.success or len(info.vehicles)!=1: raise RuntimeError('Ambiguous autopilot')
    v=info.vehicles[0]
    if (v.sysid,v.compid,v.autopilot,v.type)!=(1,1,3,2) or v.flight_sw_version>>8!=0x040507:
        raise RuntimeError('Unexpected vehicle/firmware')
    get=rospy.ServiceProxy('/mavros/param/get',ParamGet)
    values={}
    for name in ('SIM_SPEEDUP','GPS_TYPE','EK3_SRC1_POSXY','EK3_SRC1_VELXY','EK3_SRC1_POSZ','ARMING_CHECK','LOG_DISARMED','LOG_BACKEND_TYPE'):
        r=get(param_id=name)
        if not r.success: raise RuntimeError('Missing SITL parameter '+name)
        values[name]=r.value.real or r.value.integer
    if values['SIM_SPEEDUP']<=0 or values['GPS_TYPE']!=0 or values['EK3_SRC1_POSXY']!=6 or values['EK3_SRC1_VELXY']!=0:
        raise RuntimeError('SITL/no-GPS/position-only configuration mismatch')
    if not rospy.get_param('/indoor_nav/safety/freeze_detection/enabled'):
        raise RuntimeError('Freeze protection disabled')
    return dict(sitl_pid=p.pid,sitl_created=p.create_time(),proxy_pid=proxy.pid,mavros_pid=pid,
                master=MASTER,params=values,armed=st.armed,mode=st.mode)


class Run:
    def __init__(self,out):
        self.out=out;out.mkdir(parents=True,exist_ok=False)
        self.events=[];self.latest={};self.stage='INITIAL';self.bag=None
        self.decoder=dialect.MAVLink(None)
        self.data=open(out/'telemetry.jsonl','w',buffering=1)
        topics=[('/mavros/global_position/gp_origin',GeoPointStamped),('/mavros/state',State),('/indoor_nav/safety_status',SafetyStatus),
                ('/indoor_nav/localization_status',LocalizationStatus),('/indoor_nav/mission_state',String),
                ('/indoor_nav/world_pose',PoseStamped),('/mavros/local_position/odom',Odometry),
                ('/gazebo/model_states',ModelStates),('/indoor_nav/nav_from_world',TransformStamped),
                ('/mavros/statustext/recv',StatusText),('/mavros/estimator_status',EstimatorStatus),
                ('/indoor_nav/uwb_velocity/status',String),('/indoor_nav/localization_diagnostics',String),('/indoor_nav/safety_diagnostics',String),('/indoor_nav/abort_snapshot',String),
                ('/indoor_nav/uwb_velocity/world',__import__('geometry_msgs.msg',fromlist=['TwistWithCovarianceStamped']).TwistWithCovarianceStamped)]
        self.subs=[rospy.Subscriber(t,typ,lambda m,t=t:self.cb(t,m),queue_size=100) for t,typ in topics]
        self.subs.append(rospy.Subscriber('/mavlink/from',Mavlink,self.ekf_cb,queue_size=100))
    def ekf_cb(self,msg):
        # Copter emits dialect EKF_STATUS_REPORT (193), not necessarily common
        # ESTIMATOR_STATUS (230), so do not infer EKF health from a silent topic.
        if msg.msgid!=193 or (msg.sysid,msg.compid)!=(1,1) or msg.framing_status!=Mavlink.FRAMING_OK:return
        try:data=self.decoder.decode(convert_to_bytes(msg)).to_dict()
        except Exception as exc:data=dict(flags=0,decode_error=str(exc))
        now=time.time();topic='/mavlink/ekf_status_report'
        self.latest[topic]=(data,now)
        self.data.write(json.dumps(dict(time=now,topic=topic,stage=self.stage,data=data))+'\n')
    def event(self,event_name,**kw):
        e=dict(time=time.time(),event=event_name,stage=self.stage,**kw);self.events.append(e)
        (self.out/'events.json').write_text(json.dumps(self.events,indent=2));print(json.dumps(e),flush=True)
    def cb(self,t,m):
        now=time.time();self.latest[t]=(m,now)
        # Bound Gazebo logging to ~25 Hz; rosbag retains the original topic.
        if t=='/gazebo/model_states' and now-getattr(self,'last_truth',0)<.035:return
        if t=='/gazebo/model_states':self.last_truth=now
        def convert(value):
            if hasattr(value,'__slots__'):
                return {k:convert(getattr(value,k)) for k in value.__slots__}
            if isinstance(value,(tuple,list)):return [convert(v) for v in value]
            return value
        self.data.write(json.dumps(dict(time=now,topic=t,stage=self.stage,data=convert(m)))+'\n')
    def get(self,t,age=1.):
        m,ts=self.latest.get(t,(None,0))
        if m is None or (age is not None and time.time()-ts>age):raise RuntimeError('STALE:'+t)
        return m
    def wait(self,predicate,timeout,monitor=False):
        end=time.monotonic()+timeout
        while time.monotonic()<end and not rospy.is_shutdown():
            if monitor:self.health()
            try:
                if predicate():return
            except (KeyError,IndexError):pass
            time.sleep(.05)
        raise RuntimeError('TIMEOUT:'+self.stage)
    def health(self):
        st=self.get('/mavros/state',1.5)
        if not st.connected:raise RuntimeError('FC_LOST')
        loc=self.get('/indoor_nav/localization_status',.4)
        if not loc.healthy:raise RuntimeError('LOCALIZATION:'+loc.reason)
        s=self.get('/indoor_nav/safety_status',.4)
        if s.abort_latched:raise RuntimeError('SAFETY_ABORT:'+s.reason)
        if self.stage in ('TAKEOFF','HOVER','MISSION'):
            ekf=self.get('/mavlink/ekf_status_report',1.)
            flags=ekf['flags']
            if flags & 0x2f != 0x2f or flags & (1024|32768):
                raise RuntimeError('EKF_STATUS:'+str(flags))
        if self.stage=='TAKEOFF':
            p=self.truth()
            box=rospy.get_param('/indoor_nav/safety/geofence')
            if not (box['x_min']<=p.x<=box['x_max'] and box['y_min']<=p.y<=box['y_max'] and p.z<=box['z_max']):
                raise RuntimeError('TAKEOFF_TRUTH_FENCE')
            if not st.armed or st.mode!='GUIDED':raise RuntimeError('TAKEOFF_MODE:'+st.mode)
        if self.stage in ('HOVER','MISSION'):
            if not st.armed or st.mode!='GUIDED':raise RuntimeError('UNEXPECTED_MODE:'+st.mode)
            if not s.ready:raise RuntimeError('SAFETY:'+s.reason)
        if self.stage=='MISSION' and self.get('/indoor_nav/mission_state',None).data in ('ABORT','STOPPED_BY_PILOT','MANUAL_STOP'):
            raise RuntimeError('MISSION:'+self.get('/indoor_nav/mission_state',None).data)
    def truth(self):
        m=self.get('/gazebo/model_states',.4);return m.pose[m.name.index('iris')].position
    def mode(self,name):
        self.assert_instance()
        r=rospy.ServiceProxy('/mavros/set_mode',SetMode)(base_mode=0,custom_mode=name)
        if not r.mode_sent:raise RuntimeError('MODE_REJECTED:'+name)
        self.wait(lambda:self.get('/mavros/state').mode==name,5)
        self.event('MODE',mode=name)
    def assert_instance(self):
        ident=identity()
        if ident['sitl_pid']!=self.ident['sitl_pid'] or ident['sitl_created']!=self.ident['sitl_created']:
            raise RuntimeError('SITL_INSTANCE_CHANGED')
    def prepare(self):
        self.stage='PREPARE';self.ident=identity();self.event('IDENTITY',identity=self.ident)
        if self.ident['armed']:raise RuntimeError('Prepare requires DISARMED')
        self.wait(lambda:'/gazebo/model_states' in self.latest,10)
        p=self.truth()
        if p.z>.08 or math.hypot(p.x,p.y)>.05:raise RuntimeError('Not at initial ground origin')
        m=self.get('/gazebo/model_states');q=m.pose[m.name.index('iris')].orientation
        if max(abs(q.x),abs(q.y),abs(q.z))>.02:raise RuntimeError('Ground nose not aligned +X')
        self.assert_instance()
        from mavros_msgs.srv import MessageInterval
        r=rospy.ServiceProxy('/mavros/set_message_interval',MessageInterval)(message_id=0,message_rate=5.)
        if not r.success:raise RuntimeError('HEARTBEAT_INTERVAL_REJECTED')
        self.event('HEARTBEAT_REQUEST',message_id=0,rate_hz=5.)
        # Only this isolated copy of EEPROM is modified; restore on next cold start.
        for name,target in (('ARMING_CHECK',1),('LOG_DISARMED',1)):
            old=self.ident['params'][name]
            self.assert_instance()
            r=rospy.ServiceProxy('/mavros/param/set',ParamSet)(param_id=name,value=ParamValue(integer=target,real=0.))
            if not r.success:raise RuntimeError('PARAM_WRITE_FAILED:'+name)
            self.event('SITL_PARAMETER',name=name,before=old,after=target)
        # Explicit, documented home of this launcher, not a measured GPS fix.
        pub=rospy.Publisher('/mavros/global_position/set_gp_origin',GeoPointStamped,queue_size=1)
        self.wait(lambda:pub.get_num_connections()>0,5)
        origin=GeoPointStamped();origin.position.latitude=-35.363261;origin.position.longitude=149.165230;origin.position.altitude=584.
        for _ in range(5):
            self.assert_instance();origin.header.stamp=rospy.Time.now();pub.publish(origin);time.sleep(.2)
        rospy.ServiceProxy('/mavros/cmd/command',CommandLong)(broadcast=False,command=512,confirmation=0,param1=49.,param2=0.,param3=0.,param4=0.,param5=0.,param6=0.,param7=0.)
        self.wait(lambda:'/mavros/global_position/gp_origin' in self.latest,8)
        accepted=self.get('/mavros/global_position/gp_origin',10.)
        if abs(accepted.position.latitude-origin.position.latitude)>1e-6 or abs(accepted.position.longitude-origin.position.longitude)>1e-6:
            raise RuntimeError('ORIGIN_NOT_ACCEPTED')
        self.event('ORIGIN_ACCEPTED',lat=accepted.position.latitude,lon=accepted.position.longitude,alt=accepted.position.altitude)
        self.wait(lambda:'/indoor_nav/localization_status' in self.latest,15)
        locked=self.get('/indoor_nav/localization_status').transform_locked
        for _ in range(12):
            if locked:
                from std_srvs.srv import TriggerResponse
                r=TriggerResponse(success=True,message='Existing locked calibration retained at restored ground origin')
                break
            r=rospy.ServiceProxy('/indoor_nav/calibrate',Trigger)()
            if r.success:break
            time.sleep(1)
        if not r.success:raise RuntimeError('CALIBRATION:'+r.message)
        self.event('CALIBRATED',message=r.message)
        self.wait(lambda:self.get('/indoor_nav/localization_status').healthy,10)
        self.event('AWAIT_HUMAN_ACK',safety_reason=self.get('/indoor_nav/safety_status').reason)
    def fly(self):
        self.stage='PREFLIGHT';self.ident=identity();self.event('IDENTITY',identity=self.ident)
        if self.ident['armed']:raise RuntimeError('Flight must start DISARMED')
        self.wait(lambda:'/indoor_nav/safety_status' in self.latest,5)
        self.health()
        if not self.get('/indoor_nav/safety_status').ready:
            raise RuntimeError('Human safety ACK required:'+self.get('/indoor_nav/safety_status').reason)
        if self.ident['params']['ARMING_CHECK']!=1:raise RuntimeError('All FC arming checks must be enabled')
        self.bag=subprocess.Popen(['rosbag','record','-a','-O',str(self.out/'flight.bag')],stdout=open(self.out/'rosbag.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(2)
        if self.bag.poll() is not None:raise RuntimeError('ROSBAG_FAILED')
        self.mode('GUIDED');self.assert_instance()
        r=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)(value=True)
        self.event('ARM_RESPONSE',success=r.success,result=r.result)
        if not r.success:raise RuntimeError('ARM_REJECTED:'+str(r.result))
        self.wait(lambda:self.get('/mavros/state').armed,5)
        self.stage='TAKEOFF';self.assert_instance()
        r=rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)(min_pitch=0.,yaw=0.,latitude=0.,longitude=0.,altitude=.8)
        self.event('TAKEOFF_RESPONSE',success=r.success,result=r.result)
        if not r.success:raise RuntimeError('TAKEOFF_REJECTED')
        self.wait(lambda:self.truth().z>.68 and self.get('/indoor_nav/safety_status').ready,25,True)
        self.stage='HOVER';self.event('HOVER_REACHED',truth_z=self.truth().z)
        end=time.monotonic()+3
        while time.monotonic()<end:self.health();time.sleep(.05)
        self.stage='MISSION'
        r=rospy.ServiceProxy('/indoor_nav/mission/start',Trigger)()
        self.event('MISSION_START',success=r.success,message=r.message)
        if not r.success:raise RuntimeError('MISSION_START:'+r.message)
        self.wait(lambda:self.get('/indoor_nav/mission_state',None).data=='COMPLETE',180,True)
        self.event('MISSION_COMPLETE')
    def land(self):
        self.stage='LAND';self.assert_instance()
        if self.get('/mavros/state').armed:
            self.mode('LAND')
            self.wait(lambda:self.truth().z<.06 and not self.get('/mavros/state').armed,35)
        self.event('LANDED_DISARMED',truth_z=self.truth().z)
        self.mode('STABILIZE')
        # Only after confirmed landing; preserve abort snapshots in the bag first.
        rospy.ServiceProxy('/indoor_nav/mission/stop',Trigger)()
        rospy.ServiceProxy('/indoor_nav/safety/reset',Trigger)()
        self.event('RESET_REQUIRES_NEW_OPERATOR_ACK')
        self.restore_ground()
    def restore_ground(self):
        self.stage='RESTORE';self.assert_instance()
        if self.get('/mavros/state').armed or self.truth().z>.06:
            raise RuntimeError('Ground restore requires landed/disarmed')
        m=ModelState(model_name='iris',reference_frame='world')
        m.pose.orientation.w=1.
        r=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(model_state=m)
        if not r.success:raise RuntimeError('MODEL_RESET_FAILED')
        self.event('MODEL_RESTORED_ON_GROUND')

    def close(self,result,error=None):
        self.event('RESULT',result=result,error=error)
        if self.bag and self.bag.poll() is None:
            os.killpg(self.bag.pid,signal.SIGINT)
            try:self.bag.wait(timeout=12)
            except subprocess.TimeoutExpired:self.event('ROSBAG_STOP_TIMEOUT')
        for sub in self.subs:sub.unregister()
        self.data.close()
        logs=RUNTIME/'logs'
        import shutil
        if logs.exists():
            for f in logs.glob('*.BIN'):shutil.copy2(f,self.out/f.name)
        shutil.copy2(RUNTIME/'mav.tlog',self.out/'mav.tlog') if (RUNTIME/'mav.tlog').exists() else None
        (self.out/'summary.json').write_text(json.dumps(dict(result=result,error=error,events=self.events),indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','fly'));p.add_argument('--output',required=True);a=p.parse_args()
    rospy.init_node('isolated_sitl_flight_test',anonymous=True)
    run=Run(Path(a.output).resolve());result='FAILED';err=None
    def interrupted(_sig,_frame):raise KeyboardInterrupt('TERMINATED')
    signal.signal(signal.SIGTERM,interrupted)
    try:
        if a.action=='prepare':run.prepare();result='PREPARED_AWAITING_HUMAN_ACK'
        else:run.fly();run.land();result='PASSED'
    except (Exception,KeyboardInterrupt) as exc:
        err=str(exc) or 'OPERATOR_INTERRUPT';run.event('FAILURE',error=err)
        if a.action=='fly' and hasattr(run,'ident'):
            try:run.land()
            except Exception as ex:run.event('RECOVERY_FAILED',error=str(ex))
    finally:run.close(result,err)
    raise SystemExit(0 if result!='FAILED' else 1)

if __name__=='__main__':main()
