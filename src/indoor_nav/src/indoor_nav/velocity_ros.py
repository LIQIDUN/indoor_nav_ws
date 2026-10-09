"""Shadow-only ROS presentation. Never publishes any MAVROS observation."""
import json
import math
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from indoor_nav.core import finite, rotate_vector
from indoor_nav.velocity import VelocityWindow, native_body_velocity, select_velocity


class VelocityShadow:
    def __init__(self, cfg, profile, body_to_tag, world_frame):
        self.window=VelocityWindow(**cfg.get('window',{}))
        self.native_cfg=cfg.get('native',{})
        self.native_verified=bool(self.native_cfg.get('verified',False)) and profile!='sitl'
        if self.native_verified and (not self.native_cfg.get('evidence') or
                self.native_cfg.get('units')!='mps' or
                self.native_cfg.get('frame_id')!='uwb_anchor' or
                self.native_cfg.get('reference_point')!='tag'):
            raise ValueError('native UWB requires verified units/frame/Tag reference and evidence')
        self.native_sigma=float(self.native_cfg.get('sigma_mps',.1))
        if not finite(self.native_sigma) or self.native_sigma<=0:
            raise ValueError('invalid native velocity sigma')
        self.profile=profile
        self.body_to_tag=tuple(body_to_tag)
        self.world_frame=world_frame
        self.raw_native=None
        self.native_candidate=None
        self.native_reason='NOT_OBSERVED'
        self.last_shadow_stamp=None
        self.diagnostics=rospy.Publisher('/indoor_nav/uwb_velocity/status',String,queue_size=1,latch=True)
        self.publisher=rospy.Publisher('/indoor_nav/uwb_velocity/world',TwistWithCovarianceStamped,queue_size=10)

    def observe(self, now, raw_xy, bias, native, q_world_body, angular_body):
        self.raw_native=list(native) if len(native)==3 and finite(*native) else None
        self.native_candidate=None
        self.native_reason='UNVERIFIED_NATIVE' if self.profile!='sitl' else 'MOCK_NATIVE_NOT_VALIDATED'
        if q_world_body is None:
            self.window.clear('ATTITUDE_OR_CALIBRATION_UNAVAILABLE')
            return
        offset=rotate_vector(q_world_body,self.body_to_tag)
        body_xy=(raw_xy[0]-bias[0]-offset[0],raw_xy[1]-bias[1]-offset[1])
        self.window.add(now,body_xy)
        if self.raw_native is not None and (angular_body is not None or not any(self.body_to_tag)):
            corrected=native_body_velocity(native,angular_body or (0.,0.,0.),self.body_to_tag,q_world_body)
            if math.hypot(*corrected[:2])<=self.window.max_speed:
                self.native_candidate=corrected
                if self.native_verified:
                    self.native_reason='VERIFIED_NATIVE'
            else:
                self.native_reason='EXCESSIVE_NATIVE_SPEED'
        elif self.native_verified:
            self.native_reason='NATIVE_OR_ANGULAR_RATE_UNAVAILABLE'

    def report(self, now, healthy, health_reason, transform, q_nav_body, frozen=False, ekf=None):
        derived=self.window.estimate(now)
        # Native must be an observation from the same fresh accepted sample.
        native=(self.native_candidate if 0<=derived.get('sample_age_s',float('inf'))<=self.window.timeout else None)
        selected=select_velocity(derived,native,self.native_verified,self.native_sigma,frozen,max_age=self.window.timeout)
        if not healthy:
            selected.update(valid=False,source='UNAVAILABLE',reason='LOCALIZATION:'+health_reason)
        data={'stamp_s':now,'frame_id':self.world_frame,'forwarded_to_fc':False,
              'raw_native_uwb_anchor_xyz':self.raw_native,'native_reason':self.native_reason,
              'native_verified':self.native_verified,'native_body_world_candidate':native,
              'derived':derived,'selected':selected,'ekf':ekf,
              'gazebo_truth':'NOT_AN_ESTIMATOR_INPUT; compare using evaluation capture',
              'position_velocity_correlation':'same UWB input; no independent fusion claim'}
        if selected['velocity_world_xy'] is not None and transform is not None:
            vx,vy=selected['velocity_world_xy']
            vn=transform.rotate(vx,vy)
            data['velocity_nav_xy']=list(vn)
            if selected['variance_world_xy'] is not None:
                a,b=selected['variance_world_xy'];c,s=math.cos(transform.yaw),math.sin(transform.yaw)
                data['covariance_nav_xy']=[c*c*a+s*s*b,c*s*(a-b),c*s*(a-b),s*s*a+c*c*b]
            if q_nav_body is not None:
                inverse=(-q_nav_body[0],-q_nav_body[1],-q_nav_body[2],q_nav_body[3])
                data['body_flu_candidate_assuming_unknown_vz_zero']=list(rotate_vector(inverse,(*vn,0.)))
        stamp=selected.get('sample_stamp_s')
        if selected['valid'] and stamp is not None and stamp!=self.last_shadow_stamp:
            msg=TwistWithCovarianceStamped()
            effective_stamp=stamp-(selected['window_s']/2. if selected['source']=='DERIVED_WINDOW' else 0.)
            msg.header.stamp=rospy.Time.from_sec(effective_stamp)
            msg.header.frame_id=self.world_frame
            msg.twist.twist.linear.x,msg.twist.twist.linear.y=selected['velocity_world_xy']
            cov=[0.]*36
            cov[0],cov[7]=selected['variance_world_xy']
            for i in (14,21,28,35):cov[i]=1e3
            msg.twist.covariance=cov
            self.publisher.publish(msg)
            self.last_shadow_stamp=stamp
        self.diagnostics.publish(String(data=json.dumps(data)))
        return data
