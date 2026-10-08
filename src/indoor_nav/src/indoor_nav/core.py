"""ROS-independent navigation math, validation, and deterministic safety gates.

Conventions: world_lab and nav_enu use right-handed ENU-style axes, metres.
T_nav_world: p_nav = Rz(theta) * p_world + translation.
The yaw offset is CALIBRATED ON THE GROUND ONLY and must be shared by
measurement and mission pipelines. No automatic per-flight origin shift.
"""
import math


def finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def wrap_pi(v):
    return (v + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


def quaternion_product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X + x*W + y*Z - z*Y,
            w*Y - x*Z + y*W + z*X,
            w*Z + x*Y - y*X + z*W,
            w*W - x*X - y*Y - z*Z)


def yaw_quaternion(angle):
    return (0., 0., math.sin(angle/2.), math.cos(angle/2.))


def normalize_quaternion(q):
    n = math.sqrt(sum(v*v for v in q))
    if n < 0.5 or not finite(*q):
        raise ValueError('invalid quaternion')
    return tuple(v/n for v in q)


def roll_pitch(q):
    x, y, z, w = normalize_quaternion(q)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y - z*x))))
    return roll, pitch


class Transform2D:
    def __init__(self, yaw=0., tx=0., ty=0.):
        if not finite(yaw, tx, ty):
            raise ValueError('nonfinite transform')
        self.yaw = float(yaw)
        self.tx = float(tx)
        self.ty = float(ty)

    def apply(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*x - s*y + self.tx, s*x + c*y + self.ty

    def inverse(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        a, b = x-self.tx, y-self.ty
        return c*a+s*b, -s*a+c*b

    def rotate(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c*x-s*y, s*x+c*y


class UwbGate:
    """Reject implausible jumps without rejecting normal 25Hz UWB noise.

    Raw instantaneous implied velocity is noisy (0.05m / .04s=1.25m/s).
    A position-noise margin is essential; this gate is NOT a full EKF.
    """
    def __init__(self, max_jump=0.30, max_speed=1.5, timeout=0.20,
                 noise_floor=0.10, max_eop=0.50):
        self.max_jump = float(max_jump)
        self.max_speed = float(max_speed)
        self.timeout = float(timeout)
        self.noise_floor = float(noise_floor)
        self.max_eop = float(max_eop)
        self.last = None
        self.rejected = 0
        self.reason = 'NO_UWB'
        self.last_rx = None

    def evaluate(self, t, x, y, eop=0.0):
        self.last_rx = t
        if not finite(t, x, y, eop):
            self.rejected += 1
            self.reason = 'NAN_OR_INF'
            return False
        if eop > 0 and eop > self.max_eop:
            self.rejected += 1
            self.reason = 'HIGH_EOP'
            return False
        if self.last is not None:
            dt = t - self.last[0]
            if dt <= 0:
                self.rejected += 1
                self.reason = 'NONMONOTONIC_TIME'
                return False
            distance = math.hypot(x-self.last[1], y-self.last[2])
            # Both max_jump and max_speed matter, but do not use raw
            # |delta|/dt as a hard cutoff at 25Hz (rejects normal noise).
            allowance = max(self.max_jump, self.noise_floor * 3.) + self.max_speed * dt
            if distance > allowance:
                self.rejected += 1
                self.reason = 'JUMP'
                return False
        self.last = (t, x, y)
        self.reason = 'OK'
        return True

    def fresh(self, now):
        return self.last is not None and 0 <= now-self.last[0] <= self.timeout


class GroundCalibration:
    def __init__(self, physical_xy, required_samples=25, max_offset=0.30,
                 max_std=0.10):
        self.physical_xy = tuple(physical_xy)
        self.required_samples = int(required_samples)
        self.max_offset = float(max_offset)
        self.max_std = float(max_std)
        self.samples = []

    def add(self, xy):
        if finite(*xy):
            self.samples.append((float(xy[0]), float(xy[1])))
            self.samples = self.samples[-500:]

    def result(self):
        if len(self.samples) < self.required_samples:
            raise ValueError('insufficient ground tag samples')
        xx = [p[0] for p in self.samples]
        yy = [p[1] for p in self.samples]
        mx, my = sum(xx)/len(xx), sum(yy)/len(yy)
        sx = math.sqrt(sum((v-mx)**2 for v in xx)/len(xx))
        sy = math.sqrt(sum((v-my)**2 for v in yy)/len(yy))
        if math.hypot(sx, sy) > self.max_std:
            raise ValueError('unstable ground tag')
        bx, by = mx-self.physical_xy[0], my-self.physical_xy[1]
        if math.hypot(bx, by) > self.max_offset:
            raise ValueError('ground tag bias exceeds limit')
        return (bx, by)


def inside_box(p, box):
    return (finite(*p) and
            box['x_min'] <= p[0] <= box['x_max'] and
            box['y_min'] <= p[1] <= box['y_max'] and
            box['z_min'] <= p[2] <= box['z_max'])


def bounded_step(current, target, dt, max_xy_speed, max_z_speed):
    """Slew-limit a world-frame position target (not true jerk/acceleration limit)."""
    if dt <= 0:
        return tuple(current)
    dx, dy, dz = (target[i]-current[i] for i in range(3))
    xy = math.hypot(dx, dy)
    limit = max_xy_speed * dt
    scale = min(1., limit/xy) if xy > 1e-10 else 1.
    zz = max(-max_z_speed*dt, min(max_z_speed*dt, dz))
    return (current[0]+dx*scale, current[1]+dy*scale, current[2]+zz)


def rotate_vector(q, p):
    """Rotate FLU body vector p into parent frame using child->parent q."""
    qx, qy, qz, qw = normalize_quaternion(q)
    x, y, z = p
    # v_parent = q * v * conj(q), efficiently via cross products.
    tx = 2. * (qy*z-qz*y)
    ty = 2. * (qz*x-qx*z)
    tz = 2. * (qx*y-qy*x)
    return (x+qw*tx+qy*tz-qz*ty,
            y+qw*ty+qz*tx-qx*tz,
            z+qw*tz+qx*ty-qy*tx)
