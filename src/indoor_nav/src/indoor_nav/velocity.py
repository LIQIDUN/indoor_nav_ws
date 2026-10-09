"""ROS-independent accepted-UWB timing, robust velocity shadow, transport audit."""
import math
from collections import deque
from statistics import median
from indoor_nav.core import finite, rotate_vector


class DeviceCounterGate:
    """Count-only ordering; no assumption about device clock physical units.

    Device reboot/backward jumps fail closed until restart, while uint32 wrap
    is accepted. All-zero counters cannot prove uniqueness and are labelled.
    """
    def __init__(self):
        self.last = None
        self.receipt = None
        self.reason = 'INITIALIZING'
        self.rejected = 0

    def accept(self, receipt, local_time=0, system_time=0):
        if not finite(receipt) or (self.receipt is not None and receipt <= self.receipt):
            self.reason = 'RECEIPT_TIME_NONMONOTONIC'
            self.rejected += 1
            return False
        source = self.last[0] if self.last is not None else ('system_time' if system_time else 'local_time')
        value = int(system_time) if source == 'system_time' else int(local_time)
        if not value and self.last is None:
            self.receipt = receipt
            self.reason = 'NO_DEVICE_COUNTER_RECEIPT_ONLY'
            return True
        if not 0 <= value <= 0xffffffff:
            self.reason = 'INVALID_DEVICE_COUNTER'
            self.rejected += 1
            return False
        if self.last is not None:
            old_source, old_value = self.last
            delta = (value-old_value) & 0xffffffff
            if source != old_source or delta == 0 or delta >= 0x80000000:
                self.reason = ('DEVICE_COUNTER_SOURCE_CHANGED' if source != old_source else
                               'DUPLICATE_DEVICE_PACKET' if delta == 0 else 'DEVICE_TIME_REWIND_OR_REORDER')
                self.rejected += 1
                return False
        self.last = source, value
        self.receipt = receipt
        self.reason = 'ORDERED_DEVICE_COUNTER_RECEIPT_TIME'
        return True


class VelocityWindow:
    """Huber-weighted linear regression on corrected body XY, receipt time.

    The fitted slope describes the window centre, not an instantaneous sample.
    Near-static data are ambiguous with frozen measurements: expose a numeric
    candidate but do not publish it as a valid zero-velocity observation.
    """
    def __init__(self, window_s=.5, min_samples=8, min_span_s=.25,
                 timeout_s=.2, position_sigma_m=.1, speed_sigma_floor_mps=.05,
                 max_sigma_mps=1., max_speed_mps=1.5, static_epsilon_m=.015):
        self.window = float(window_s)
        self.minimum = int(min_samples)
        self.min_span = float(min_span_s)
        self.timeout = float(timeout_s)
        self.pos_sigma = float(position_sigma_m)
        self.sigma_floor = float(speed_sigma_floor_mps)
        self.max_sigma = float(max_sigma_mps)
        self.max_speed = float(max_speed_mps)
        self.epsilon = float(static_epsilon_m)
        values=(self.window,self.min_span,self.timeout,self.pos_sigma,self.sigma_floor,
                self.max_sigma,self.max_speed,self.epsilon)
        if not finite(*values) or min(values) <= 0 or self.minimum < 3 or self.min_span > self.window:
            raise ValueError('invalid velocity configuration')
        self.samples = deque(maxlen=512)
        self.reason = 'INITIALIZING'

    def clear(self, reason):
        self.samples.clear()
        self.reason = reason

    def add(self, stamp, xy):
        if not finite(stamp, *xy):
            self.clear('NONFINITE_SAMPLE')
            return False
        if self.samples:
            dt=stamp-self.samples[-1][0]
            if dt <= 0:
                self.clear('NONMONOTONIC_SAMPLE')
                return False
            if dt > self.timeout:
                self.clear('SAMPLE_GAP')
        self.samples.append((float(stamp), float(xy[0]), float(xy[1])))
        while self.samples and stamp-self.samples[0][0] > self.window+1e-9:
            self.samples.popleft()
        self.reason = 'INITIALIZING'
        return True

    def estimate(self, now):
        result={'valid':False,'source':'UNAVAILABLE','reason':self.reason,
                'sample_count':len(self.samples), 'timestamp_basis':'ROS_RECEIPT',
                'position_velocity_correlated':True, 'velocity_world_xy':None,
                'variance_world_xy':None, 'window_s':0., 'delay_s':None}
        if not self.samples:
            return result
        rows=list(self.samples)
        age=now-rows[-1][0]
        duration=rows[-1][0]-rows[0][0]
        result.update(sample_age_s=age, sample_stamp_s=rows[-1][0], window_s=duration,
                      delay_s=age+duration/2.)
        if not 0 <= age <= self.timeout:
            result['reason']='STALE_OR_TIME_RESET'
            return result
        if len(rows)<self.minimum or duration<self.min_span:
            return result
        ts=[r[0]-rows[-1][0] for r in rows]
        xs, ys=[r[1] for r in rows], [r[2] for r in rows]
        def fit(values, weights):
            total=sum(weights)
            mt=sum(t*w for t,w in zip(ts,weights))/total
            mv=sum(v*w for v,w in zip(values,weights))/total
            sxx=sum(w*(t-mt)**2 for t,w in zip(ts,weights))
            slope=sum(w*(t-mt)*(v-mv) for t,v,w in zip(ts,values,weights))/sxx
            residual=[v-(mv+slope*(t-mt)) for t,v in zip(ts,values)]
            variance=max(self.pos_sigma**2,sum(w*e*e for w,e in zip(weights,residual))/max(1.,total-2.))/sxx
            return slope,residual,max(self.sigma_floor**2,variance)
        weights=[1.]*len(rows)
        for _ in range(3):
            vx,ex,varx=fit(xs,weights);vy,ey,vary=fit(ys,weights)
            residual=[math.hypot(x,y) for x,y in zip(ex,ey)]
            scale=max(self.pos_sigma,1.4826*median(abs(e-median(residual)) for e in residual),1e-6)
            weights=[min(1.,1.5*scale/max(e,1e-9)) for e in residual]
        vx,ex,varx=fit(xs,weights);vy,ey,vary=fit(ys,weights)
        span=math.hypot(max(xs)-min(xs),max(ys)-min(ys))
        reason=('STATIONARY_OR_FROZEN_AMBIGUOUS' if span<=self.epsilon else
                'EXCESSIVE_SPEED' if math.hypot(vx,vy)>self.max_speed else
                'UNCERTAINTY_TOO_HIGH' if max(varx,vary)>self.max_sigma**2 else 'OK')
        result.update(valid=reason=='OK',source='DERIVED_WINDOW' if reason=='OK' else 'UNAVAILABLE',
                      candidate_source='DERIVED_WINDOW',reason=reason,velocity_world_xy=[vx,vy],
                      variance_world_xy=[varx,vary],position_span_m=span,
                      fit_residual_rms_m=math.sqrt(sum(x*x+y*y for x,y in zip(ex,ey))/len(rows)))
        return result


def native_body_velocity(tag_velocity, angular_body, body_to_tag, q_world_body):
    """Tag world-axis velocity -> body-centre world-axis velocity, metres/s."""
    if not finite(*tag_velocity, *angular_body, *body_to_tag):
        raise ValueError('nonfinite native velocity / lever arm')
    wx,wy,wz=angular_body;rx,ry,rz=body_to_tag
    correction=rotate_vector(q_world_body,(wy*rz-wz*ry,wz*rx-wx*rz,wx*ry-wy*rx))
    return tuple(v-d for v,d in zip(tag_velocity,correction))


def select_velocity(derived, native, native_verified=False, native_sigma=.1, frozen=False, max_age=.2):
    if frozen:
        return dict(derived,valid=False,source='UNAVAILABLE',reason='SAFETY_FREEZE_LATCHED')
    if (native_verified and native is not None and finite(*native) and
            0 <= derived.get('sample_age_s', float('inf')) <= max_age):
        return dict(derived,valid=True,source='NATIVE_UWB',reason='VERIFIED_NATIVE',
                    velocity_world_xy=list(native[:2]),variance_world_xy=[native_sigma**2]*2,
                    position_velocity_correlated='DEVICE_ALGORITHM_UNKNOWN')
    return dict(derived)


def copter457_position_error(variances, noise_floor=.2):
    """Reproduce GCS_Common.cpp + AP_VisualOdom_MAV.cpp, NOT a unit conversion."""
    if math.isnan(variances[0]):
        return float(noise_floor)
    error=sum(v*v for v in variances)**(1./3.)
    return min(100.,max(float(noise_floor),error))


def position_only_adapter_guard(profile, firmware, params, required_sigma):
    """Unknown covariance fallback is only safe with checked scalar FC noise.

    Any source set can contribute velocity under EK3_SRC_OPTIONS bit 0. Refuse
    all EXTNAV velocity sources, external height and external yaw in all sets.
    """
    if profile!='sitl' or tuple(firmware)!=(4,5,7):
        return False,'SITL_COPTER_457_REQUIRED'
    expected={'GPS_TYPE':0,'AHRS_EKF_TYPE':3,'EK3_ENABLE':1,'VISO_TYPE':3,
              'EK3_SRC1_POSXY':6,'EK3_SRC1_POSZ':1,'EK3_SRC1_YAW':1}
    for name,value in expected.items():
        if params.get(name)!=value:
            return False,'PARAM_REQUIRED:'+name
    for i in (1,2,3):
        for field in ('VELXY','VELZ','POSZ','YAW'):
            name='EK3_SRC{}_{}'.format(i,field)
            if name not in params or (params[name] == 6 or (field == 'YAW' and params[name] == 7)):
                return False,'UNSAFE_OR_MISSING_SOURCE:'+name
    sigma=params.get('VISO_POS_M_NSE',float('nan'))
    if not finite(sigma, required_sigma) or not required_sigma<=sigma<=5.:
        return False,'VISO_POS_M_NSE_BELOW_REQUIRED_OR_INVALID'
    return True,'CHECKED_POSITION_ONLY_UNKNOWN_COVARIANCE'
