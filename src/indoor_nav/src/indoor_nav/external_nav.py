"""Opt-in, read-only guarded Copter 4.5.7 SITL covariance transport adapter.

No service here writes parameters, arms, changes modes or starts missions.
Unknown covariance does NOT assert measured Z precision. The FC scalar noise
floor must cover the actual XY variance. Diagnostic ROS covariance stays intact.
"""
import copy
import math
import rospy
from mavros_msgs.srv import ParamGet, VehicleInfoGet
from indoor_nav.velocity import position_only_adapter_guard


class PositionOnlyAdapter:
    def __init__(self, profile, config, fc_provider):
        self.enabled=bool(config.get('enabled',False))
        self.profile=profile
        self.required_sigma=float(config.get('minimum_position_sigma_m',.2))
        if not math.isfinite(self.required_sigma) or self.required_sigma <= 0:
            raise ValueError('minimum position sigma must be finite and positive')
        self.fc_provider=fc_provider
        self.params={}
        self.firmware=()
        self.reason='DISABLED'
        self.checked_at=None
        self.initialized_disarmed=False
        if self.enabled:
            if profile!='sitl':
                raise ValueError('Copter covariance adapter is SITL-only')
            self.reason='PARAMETER_AND_VERSION_CHECK_PENDING'
            rospy.Timer(rospy.Duration(3.), self.validate)

    def validate(self, _event):
        # Invalidate the cache before service calls; no stale success on failure.
        self.checked_at=None
        state=self.fc_provider()
        if not state.connected or (not self.initialized_disarmed and state.armed):
            self.reason='FIRST_VALIDATION_REQUIRES_CONNECTED_DISARMED_SITL'
            return
        try:
            rospy.wait_for_service('/mavros/vehicle_info_get',timeout=.3)
            info=rospy.ServiceProxy('/mavros/vehicle_info_get',VehicleInfoGet)(sysid=0,compid=0,get_all=False)
            vehicles=[v for v in info.vehicles if v.autopilot==3 and v.type==2 and v.available_info & 2]
            if not info.success or len(vehicles)!=1:
                self.reason='AUTOPILOT_VERSION_UNAVAILABLE'
                return
            version=vehicles[0].flight_sw_version
            self.firmware=(version>>24,(version>>16)&255,(version>>8)&255)
            rospy.wait_for_service('/mavros/param/get',timeout=.3)
            get=rospy.ServiceProxy('/mavros/param/get',ParamGet)
            names=['SIM_SPEEDUP','GPS_TYPE','AHRS_EKF_TYPE','EK3_ENABLE','VISO_TYPE',
                   'EK3_SRC1_POSXY','VISO_POS_M_NSE','VISO_POS_X','VISO_POS_Y','VISO_POS_Z']
            names += ['EK3_SRC{}_{}'.format(i,f) for i in (1,2,3) for f in ('VELXY','VELZ','POSZ','YAW')]
            params={}
            for name in names:
                r=get(param_id=name)
                if not r.success:
                    self.reason='MISSING_FC_PARAM:'+name
                    return
                params[name]=r.value.real if r.value.real!=0 else float(r.value.integer)
            if params['SIM_SPEEDUP']<=0 or any(params['VISO_POS_'+axis]!=0 for axis in 'XYZ'):
                self.reason='SITL_OR_ZERO_INSTALL_OFFSET_NOT_CONFIRMED'
                return
            ok,self.reason=position_only_adapter_guard(self.profile,self.firmware,params,self.required_sigma)
            self.params=params
            if ok:
                self.checked_at=rospy.get_time()
                self.initialized_disarmed=True
        except (rospy.ROSException,rospy.ServiceException) as ex:
            self.reason='FC_READ_FAILED:'+str(ex)

    def adapt(self, odom, now):
        if not self.enabled:
            return odom
        if self.checked_at is None or not 0 <= now-self.checked_at <= 5.:
            self.reason='ADAPTER_CHECK_STALE_OR_PENDING'
            return None
        sigma=max(self.required_sigma,math.sqrt(max(odom.pose.covariance[0],odom.pose.covariance[7])))
        ok,self.reason=position_only_adapter_guard(self.profile,self.firmware,self.params,sigma)
        if not ok:
            return None
        output=copy.deepcopy(odom)
        # ODOMETRY covariance[0]=NaN is the protocol "unknown" sentinel.
        # MAVROS rotates the matrix; NaNs propagate, including the first URT
        # element. Copter 4.5.7 explicitly uses its configured scalar noise.
        output.pose.covariance[0]=float('nan')
        return output

    def diagnostic(self):
        return {'enabled':self.enabled,'reason':self.reason,'checked_at_s':self.checked_at,
                'firmware':self.firmware,'minimum_position_sigma_m':self.required_sigma,
                'fc_position_sigma_m':self.params.get('VISO_POS_M_NSE'),
                'velocity_forwarding':False}
