#!/usr/bin/env python3
"""Read-only paired MAVROS/Gazebo velocity capture; never calls FC services."""
import rospy,time,json,math,threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from indoor_nav.navigation_velocity import paired_odometry_velocity
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--duration',type=float,default=30.)
parser.add_argument('--output',default='/tmp/velocity_frame_online.json')
args=parser.parse_args()
if not 0 < args.duration <= 600: parser.error('duration must be in (0, 600]')
rospy.init_node('velocity_frame_readonly',anonymous=True)
rows=[]; lock=threading.Lock()
def add(kind,data):
 with lock: rows.append(dict(receipt=time.time(),kind=kind,**data))
def odom(m):
 try: v=paired_odometry_velocity(m,'map','base_link');error=None
 except ValueError as e: v=None;error=str(e)
 add('odom',dict(stamp=m.header.stamp.to_sec(),parent=m.header.frame_id,child=m.child_frame_id,enu=v,error=error))
def vel(m): add('velocity_local',dict(stamp=m.header.stamp.to_sec(),frame=m.header.frame_id,enu=[m.twist.linear.x,m.twist.linear.y,m.twist.linear.z]))
def truth(m):
 if 'iris' in m.name:
  i=m.name.index('iris');v=m.twist[i].linear;p=m.pose[i].position
  add('gazebo',dict(velocity=[v.x,v.y,v.z],position=[p.x,p.y,p.z]))
def state(m): add('state',dict(connected=m.connected,armed=m.armed,mode=m.mode))
rospy.Subscriber('/mavros/local_position/odom',Odometry,odom,queue_size=100)
rospy.Subscriber('/mavros/local_position/velocity_local',TwistStamped,vel,queue_size=100)
rospy.Subscriber('/gazebo/model_states',ModelStates,truth,queue_size=20)
rospy.Subscriber('/mavros/state',State,state,queue_size=10)
rospy.Subscriber('/indoor_nav/safety_diagnostics',String,lambda m:add('safety',dict(data=json.loads(m.data))),queue_size=10)
params={p:rospy.get_param(p,None) for p in ['/mavros/local_position/frame_id','/mavros/local_position/tf/frame_id','/mavros/local_position/tf/child_frame_id','/indoor_nav/safety/profile']}
time.sleep(args.duration)
with open(args.output,'w') as f: json.dump(dict(params=params,rows=rows),f)
print(json.dumps(dict(params=params,counts={k:sum(r['kind']==k for r in rows) for k in set(r['kind'] for r in rows)},states=[r for r in rows if r['kind']=='state'][-2:])))
