import json,math,sys
from pathlib import Path
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
folder=Path(sys.argv[1] if len(sys.argv)>1 else 'docs/evidence/sitl_flight/run_05')
rows=[json.loads(s) for s in open(folder/'telemetry.jsonl')]
events=json.load(open(folder/'events.json'))
start=next(e['time'] for e in events if e['event']=='MISSION_START');end=next(e['time'] for e in events if e['event'] in ('FAILURE','MISSION_COMPLETE'))
tr=next(r['data']['transform'] for r in rows if r['topic']=='/indoor_nav/nav_from_world');q=tr['rotation'];a=math.atan2(2*q['w']*q['z'],1-2*q['z']**2);R=np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])
shift=np.array([tr['translation']['x'],tr['translation']['y']]);cmd=[]
with rosbag.Bag(str(folder/'flight.bag')) as bag:
 for _,m,t in bag.read_messages(topics=['/indoor_nav/desired_setpoint']):
  ts=m.header.stamp.to_sec()
  if start<=ts<=end:cmd.append([ts,*(R@(np.array([m.pose.position.x,m.pose.position.y])-shift)),m.pose.position.z-tr['translation']['z']])
truth=[];ekf=[];uwb=[]
for r in rows:
 if not start<=r['time']<=end:continue
 m=r['data'];t=r['time']
 if r['topic']=='/gazebo/model_states':
  p=m['pose'][m['name'].index('iris')]['position'];truth.append([t,p['x'],p['y'],p['z']])
 elif r['topic']=='/mavros/local_position/odom':
  p=m['pose']['pose']['position'];v=R@(np.array([p['x'],p['y']])-shift);ekf.append([t,*v,p['z']-tr['translation']['z']])
 elif r['topic']=='/indoor_nav/world_pose':
  p=m['pose']['position'];uwb.append([t,p['x'],p['y'],p['z']])
g=np.array(truth);c=np.array(cmd);targets=json.load(open(folder.parent/'retest_approval.json'))['mission']['waypoints'];visits=[];cursor=start
for i,p in enumerate(targets):
 hit=c[(c[:,0]>=cursor)&(np.linalg.norm(c[:,1:3]-np.array([p['x'],p['y']]),axis=1)<1e-5)&(np.abs(c[:,3]-p['z'])<1e-5)]
 if not len(hit):break
 t=hit[0,0];near=g[(g[:,0]>=t)&(np.linalg.norm(g[:,1:3]-[p['x'],p['y']],axis=1)<.12)&(np.abs(g[:,3]-p['z'])<.08)]
 visits.append(dict(index=i+1,target=[p['x'],p['y'],p['z']],command_reached_s=t-start,truth_entered_tolerance_s=float(near[0,0]-start) if len(near) else None))
 gap=np.where(np.diff(hit[:,0])>.12)[0]
 block_end=hit[gap[0],0] if len(gap) else hit[-1,0]
 cursor=block_end+.001
tracking=np.linalg.norm(g[:,1:3]-np.column_stack([np.interp(g[:,0],c[:,0],c[:,k]) for k in (1,2)]),axis=1)
out=dict(mission_duration_s=end-start,waypoint_observations=visits,note='Command target matches and truth entering tolerance do not prove completion of the required stable/hold interval.',truth_command_tracking_xy_rms_m=float(np.sqrt(np.mean(tracking**2))),truth_command_tracking_xy_max_m=float(max(tracking)))
json.dump(out,open(folder/'waypoint_analysis.json','w'),indent=2);print(json.dumps(out,indent=2))
fig,ax=plt.subplots(1,3,figsize=(14,4));plan=np.array([[p['x'],p['y']] for p in targets]);ax[0].plot(plan[:,0],plan[:,1],'k--',alpha=.3,label='Planned route' if any(e['event']=='MISSION_COMPLETE' for e in events) else 'Planned (partly unexecuted)')
for name,data in [('Gazebo',truth),('UWB',uwb),('EKF',ekf),('Command',cmd)]:
 z=np.array(data);ax[0].plot(z[:,1],z[:,2],label=name);ax[1].plot(z[:,0]-start,z[:,1],label=name+' X');ax[2].plot(z[:,0]-start,z[:,2],label=name+' Y')
ax[0].scatter(plan[:,0],plan[:,1],s=18,c='k',alpha=.4);ax[0].set(xlabel='world X (m)',ylabel='world Y (m)',aspect='equal');ax[1].set(xlabel='Time after mission start (s)',ylabel='world X (m)')
ax[2].set(xlabel='Time after mission start (s)',ylabel='world Y (m)')
for a in ax:a.grid(alpha=.3);a.legend(fontsize=8)
fig.suptitle(folder.name+': commanded trajectory vs Gazebo, UWB and EKF');fig.tight_layout(rect=(0,0,1,.94));fig.savefig(folder/'waypoint_tracking.png',dpi=140)
