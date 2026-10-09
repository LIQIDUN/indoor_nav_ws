#!/usr/bin/env python3
"""Offline ROS-telemetry/DataFlash comparison; no ROS or FC control."""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pymavlink import mavutil

def xyz(v):return np.array([v[k] for k in ('x','y','z')],float)
def stamp(m):return m['header']['stamp']['secs']+m['header']['stamp']['nsecs']*1e-9

def stats(errors):
    a=np.asarray(errors,float)
    if not len(a):return dict(samples=0)
    return dict(samples=len(a),rms=float(np.sqrt(np.mean(a*a))),max=float(np.max(a)),p95=float(np.percentile(a,95)))

def analyze(folder):
    summary=json.loads((folder/'summary.json').read_text())
    rows=[json.loads(s) for s in (folder/'telemetry.jsonl').read_text().splitlines()]
    groups={t:[r for r in rows if r['topic']==t] for t in set(r['topic'] for r in rows)}
    truth=groups.get('/gazebo/model_states',[])
    if not truth:return dict(result=summary['result'],error=summary['error'])
    gt=np.array([r['time'] for r in truth]);gp=[];gv=[]
    for r in truth:
        d=r['data'];i=d['name'].index('iris');gp.append(xyz(d['pose'][i]['position']));gv.append(xyz(d['twist'][i]['linear']))
    gp=np.array(gp);gv=np.array(gv)
    def match(t,array):return np.array([np.interp(t,gt,array[:,k]) for k in range(3)])
    transforms=groups.get('/indoor_nav/nav_from_world',[])
    if not transforms: return dict(result=summary['result'],error=summary['error'])
    tr=transforms[-1]['data']['transform'];q=tr['rotation'];theta=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
    R=np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])
    shift=xyz(tr['translation'])
    start=next((e['time'] for e in summary['events'] if e['event']=='TAKEOFF_RESPONSE' and e['success']),None)
    end=next((e['time'] for e in summary['events'] if e['event']=='FAILURE'),None)
    if end is None:end=next((e['time'] for e in summary['events'] if e['event']=='MISSION_COMPLETE'),gt[-1])
    if start is None:start=end
    active=lambda r:start<=r['time']<=end
    ep=[];ev=[];up=[];uv=[];track={};alltraj={}
    for topic,label in (('/mavros/local_position/odom','EKF'),('/indoor_nav/world_pose','UWB'),('/indoor_nav/uwb_velocity/world','UWB velocity')):
        traj=[];veltraj=[]
        for r in groups.get(topic,[]):
            if not active(r):continue
            m=r['data'];t=stamp(m)
            if label=='EKF':
                p=R@(xyz(m['pose']['pose']['position'])-shift)
                q=m['pose']['pose']['orientation'];x,y,z,w=[q[k] for k in ('x','y','z','w')]
                B=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                v=R@B@xyz(m['twist']['twist']['linear'])
                ep.append(np.linalg.norm((p-match(t,gp))[:2]));ev.append(np.linalg.norm((v-match(t,gv))[:2]));traj.append([t,*p]);veltraj.append([t,*v])
            elif label=='UWB':
                p=xyz(m['pose']['position']);up.append(np.linalg.norm((p-match(t,gp))[:2]));traj.append([t,*p])
            else:
                v=xyz(m['twist']['twist']['linear']);uv.append(np.linalg.norm((v-match(t,gv))[:2]));veltraj.append([t,*v])
        alltraj[label]=np.array(traj);track[label]=np.array(veltraj)
    states=groups.get('/mavros/state',[]);gaps=[b['time']-a['time'] for a,b in zip(states,states[1:])]
    reasons={};classes={}
    for r in groups.get('/indoor_nav/safety_diagnostics',[]):
        d=json.loads(r['data']['data']);reasons[d['reason']]=reasons.get(d['reason'],0)+1
        k=d['freeze'].get('classification');classes[k]=classes.get(k,0)+1
    mask=(gt>=start)&(gt<=end);flightpos=gp[mask]
    out=dict(result=summary['result'],error=summary['error'],airborne_test_window_s=max(0,end-start),
             ekf_position_xy_error_m=stats(ep),ekf_velocity_xy_error_mps=stats(ev),
             uwb_position_xy_error_m=stats(up),uwb_velocity_xy_error_mps=stats(uv),
             state_max_gap_s=max(gaps) if gaps else None,safety_reasons=reasons,freeze_classifications=classes,
             truth_position_min=flightpos.min(axis=0).tolist() if len(flightpos) else None,
             truth_position_max=flightpos.max(axis=0).tolist() if len(flightpos) else None,
             comparison='Header time; linear-interpolated Gazebo receipt-time samples; paired Odometry rotation and calibrated inverse yaw. No fitted frame or time lag.')
    # Map boot times with the VISP RTimeUS timestamps from this flight window.
    bins=sorted(folder.glob('*.BIN'));data=[]
    if bins:
        c=mavutil.mavlink_connection(str(bins[-1]))
        while True:
            m=c.recv_match(type=['VISP','XKF3','XKF4','PM','MSG'])
            if m is None:break
            data.append(m.to_dict())
        vis=[m for m in data if m['mavpackettype']=='VISP' and start<=m['RTimeUS']*1e-6<=end]
        out['VISP_PErr_m']=stats([m['PErr'] for m in vis])
        if vis:
            offset=np.median([m['RTimeUS']*1e-6-m['TimeUS']*1e-6 for m in vis])
            xf=[m for m in data if m['mavpackettype']=='XKF3' and start<=m['TimeUS']*1e-6+offset<=end]
            out['innovation_xy_m_by_core']={str(core):stats([math.hypot(m['IPN'],m['IPE']) for m in xf if m['C']==core]) for core in sorted(set(m['C'] for m in xf))}
            out['XKF4_SS_by_core']={str(core):sorted(set(m['SS'] for m in data if m['mavpackettype']=='XKF4' and m['C']==core and start<=m['TimeUS']*1e-6+offset<=end)) for core in (0,1)}
            out['PM_loop_hz']=sorted(set(m['LR'] for m in data if m['mavpackettype']=='PM' and start<=m['TimeUS']*1e-6+offset<=end))
    fig,axes=plt.subplots(1,3,figsize=(14,4))
    axes[0].plot(flightpos[:,0],flightpos[:,1],label='Gazebo') if len(flightpos) else None
    for label,traj in alltraj.items():
        if len(traj):axes[0].plot(traj[:,1],traj[:,2],label=label,alpha=.8)
    axes[0].set(xlabel='world X (m)',ylabel='world Y (m)',title=folder.name+' trajectory',aspect='equal');axes[0].legend()
    axes[1].plot(gt[mask]-start,gp[mask,2],label='Gazebo Z')
    for label in ('EKF','UWB'):
        a=alltraj[label]
        if len(a):axes[1].plot(a[:,0]-start,a[:,3],label=label+' Z')
    axes[1].set(xlabel='time after TAKEOFF (s)',ylabel='height (m)');axes[1].legend()
    axes[2].plot(gt[mask]-start,np.linalg.norm(gv[mask,:2],axis=1),label='Gazebo')
    for label,a in track.items():
        if len(a):axes[2].plot(a[:,0]-start,np.linalg.norm(a[:,1:3],axis=1),label=label)
    axes[2].set(xlabel='time after TAKEOFF (s)',ylabel='horizontal speed (m/s)');axes[2].legend()
    for ax in axes:ax.grid(True,alpha=.3)
    fig.tight_layout();fig.savefig(folder/'trajectory.png',dpi=140);plt.close(fig)
    (folder/'analysis.json').write_text(json.dumps(out,indent=2));return out

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folders',nargs='+');a=p.parse_args()
    for f in a.folders:
        out=analyze(Path(f));print(f,json.dumps({k:v for k,v in out.items() if k not in ('safety_reasons','freeze_classifications')}))
