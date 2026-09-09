"""
IMU fusion for the Hydrobotics ROV.

BNO085 gives absolute orientation (fused internally, incl. mag), ICM-20948 gyro
runs the high-rate predict step. ESKF holds the two together and estimates the
ICM's gyro bias along the way. Bar10 depth feeds a small 1D KF for Vz.

Usage:
  python3 imu_fusion.py --mock --duration 30 --plot
  python3 imu_fusion.py --rate 50 --icm-rate 200 --plot
  python3 imu_fusion.py --stm32 /dev/ttyUSB0 --plot
"""

import argparse
import csv
import json
import math
import os
import sys
import time
import platform
import threading

import numpy as np

# no smbus2/fcntl on windows
_FORCE_MOCK = platform.system() == "Windows"

try:
    import board, busio
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_CALIBRATION_STATUS,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    BNO_AVAILABLE = True
except ImportError:
    print("[WARN] adafruit-circuitpython-bno08x not found, BNO085 will use mock")
    BNO_AVAILABLE = False

try:
    import icm20948
    ICM_AVAILABLE = True
except ImportError:
    print("[WARN] icm20948 not found, ICM-20948 will use mock")
    ICM_AVAILABLE = False

try:
    import ms5837
    DEPTH_AVAILABLE = True
except ImportError:
    print("[WARN] ms5837 not found, Bar10 will use mock")
    DEPTH_AVAILABLE = False

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


BNO085_ADDR   = 0x4A   # 0x4B if ADR pulled high
ICM20948_ADDR = 0x68   # 0x69 if AD0 pulled high

# noise params - retune on real data
Q_GYRO   = 1e-4
Q_BIAS   = 1e-6
R_MEAS   = 1e-3
P0_ROT   = 1e-2
P0_BIAS  = 1e-4

DEPTH_Q_POS  = 1e-4
DEPTH_Q_VEL  = 1e-2
DEPTH_R_MEAS = 1e-3

MAG_CAL_MIN     = 2      # BNO085 calibration_status level needed to trust yaw
MAG_CAL_TIMEOUT = 30.0   # if mag is still uncalibrated after this long, trust it
MAG_CAL_R_MULT  = 25.0   # anyway rather than let yaw drift forever - inflate R instead
WATCHDOG_DEG  = 15.0
WATCHDOG_TIME = 3.0

BNO_RATE_HZ   = 50
ICM_RATE_HZ   = 200
DEPTH_RATE_HZ = 20
DEPTH_MAX_AGE = 1.0   # seconds - older than this, stop trusting depth for Vz blend

BIAS_FILE  = "imu_bias.json"
ALIGN_FILE = "imu_align.json"


# quaternion helpers

def to_euler(q):
    qw,qx,qy,qz = q
    roll  = math.degrees(math.atan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2)))
    sinp  = max(-1., min(1., 2*(qw*qy-qz*qx)))
    pitch = math.degrees(math.asin(sinp))
    yaw   = math.degrees(math.atan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2)))
    return roll, pitch, yaw

def from_euler(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg/2)
    p = math.radians(pitch_deg/2)
    y = math.radians(yaw_deg/2)
    cr,sr = math.cos(r),math.sin(r)
    cp,sp = math.cos(p),math.sin(p)
    cy,sy = math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]

def angle_between(q1, q2):
    dot = abs(sum(a*b for a,b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


# Error-state KF for attitude: state = quaternion + gyro bias.
# predict() runs off the ICM gyro at ~200Hz, correct() pulls it back to the
# BNO085 quaternion whenever a new one comes in.

class ESKF:

    def __init__(self):
        self.q  = np.array([1., 0., 0., 0.])
        self.b  = np.zeros(3)   # rad/s
        self.P  = np.diag([P0_ROT]*3 + [P0_BIAS]*3).astype(float)
        self._lock = threading.Lock()
        self._last_t = None
        self._ready  = False

        # how long mag_cal has been below MAG_CAL_MIN - if it never comes up
        # (thrusters running, hull steel nearby, whatever) we can't just leave
        # yaw unobserved forever
        self._mag_bad_since  = None
        self._mag_forced_ok  = False

    @staticmethod
    def _qmul(a, b):
        aw,ax,ay,az = a; bw,bx,by,bz = b
        return np.array([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                         aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw])

    @staticmethod
    def _qconj(q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    @staticmethod
    def _qnorm(q):
        n = np.linalg.norm(q)
        return q/n if n > 1e-10 else np.array([1.,0.,0.,0.])

    def predict(self, gx_dps, gy_dps, gz_dps, t):
        with self._lock:
            if not self._ready:
                self._last_t = t
                return
            dt = t - self._last_t
            self._last_t = t
            if dt <= 0 or dt > 0.5:
                return

            w = np.radians([gx_dps, gy_dps, gz_dps]) - self.b
            wx, wy, wz = w

            qw,qx,qy,qz = self.q
            self.q = self._qnorm(self.q + 0.5*dt*np.array([
                -qx*wx - qy*wy - qz*wz,
                 qw*wx + qy*wz - qz*wy,
                 qw*wy - qx*wz + qz*wx,
                 qw*wz + qx*wy - qy*wx,
            ]))

            Wx = np.array([[ 0, -wz,  wy],
                           [wz,   0, -wx],
                           [-wy, wx,   0]])
            F = np.eye(6)
            F[:3, :3] = np.eye(3) - dt*Wx
            F[:3, 3:]  = -dt*np.eye(3)

            # standard angle/bias random-walk discretization (Q_GYRO, Q_BIAS are
            # PSDs, not fixed variances) - rotation block is linear in dt, not
            # dt**2, otherwise P collapses if you run --icm-rate higher and the
            # filter quietly stops trusting the gyro enough to learn bias
            Q = np.zeros((6,6))
            Q[:3,:3] = np.eye(3) * (Q_GYRO * dt + (Q_BIAS/3.0) * dt**3)
            Q[:3,3:] = Q[3:,:3] = -np.eye(3) * (Q_BIAS/2.0) * dt**2
            Q[3:,3:] = np.eye(3) * Q_BIAS * dt
            self.P = F @ self.P @ F.T + Q

    def correct(self, bno_q_in, mag_cal, t):
        bno_q = np.array(bno_q_in, dtype=float)
        with self._lock:
            if not self._ready:
                self.q     = bno_q.copy()
                self._last_t = t
                self._ready  = True
                return

            # same hemisphere
            if np.dot(self.q, bno_q) < 0:
                bno_q = -bno_q

            trust_yaw, r_yaw_mult = self._yaw_trust(mag_cal, t)
            if not trust_yaw:
                # ignore yaw entirely while mag is uncalibrated - overwrite the
                # measurement's yaw with our own so the innovation carries no
                # yaw info (roll/pitch still correct normally)
                b_r, b_p, _ = to_euler(bno_q.tolist())
                _, _, c_y   = to_euler(self.q.tolist())
                bno_q = np.array(from_euler(b_r, b_p, c_y))
                if np.dot(self.q, bno_q) < 0:
                    bno_q = -bno_q

            q_err = self._qnorm(self._qmul(self._qconj(self.q), bno_q))
            innov = 2.0 * q_err[1:]

            H = np.zeros((3,6)); H[:,:3] = np.eye(3)
            R = np.diag([R_MEAS, R_MEAS, R_MEAS * r_yaw_mult])
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)

            dx = K @ innov
            dφ, db = dx[:3], dx[3:]

            dq = np.array([1., dφ[0]/2, dφ[1]/2, dφ[2]/2])
            self.q = self._qnorm(self._qmul(self.q, dq))
            self.b += db

            # joseph form
            IKH = np.eye(6) - K @ H
            self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    def _yaw_trust(self, mag_cal, t):
        """
        Whether to trust BNO yaw this cycle, and what R multiplier to use.
        Normally we just wait for MAG_CAL_MIN. But an ROV with thrusters and a
        steel-ish frame near the compass can sit uncalibrated indefinitely, and
        the yaw gyro bias is only observable through this correction - so past
        MAG_CAL_TIMEOUT we start trusting it anyway with inflated noise rather
        than let yaw drift forever unobserved.
        """
        if mag_cal >= MAG_CAL_MIN:
            self._mag_bad_since = None
            if self._mag_forced_ok:
                print("\n[INFO] mag calibration recovered, yaw trust back to normal")
                self._mag_forced_ok = False
            return True, 1.0

        if self._mag_bad_since is None:
            self._mag_bad_since = t

        if t - self._mag_bad_since > MAG_CAL_TIMEOUT:
            if not self._mag_forced_ok:
                self._mag_forced_ok = True
                print(f"\n\033[33m[WARN] mag uncalibrated for >{MAG_CAL_TIMEOUT:.0f}s, "
                      f"trusting yaw anyway (inflated R)\033[0m")
            return True, MAG_CAL_R_MULT

        return False, 1.0

    def snapshot(self):
        with self._lock:
            return (
                [float(x) for x in self.q],
                [float(x) for x in np.degrees(self.b)],
                float(np.trace(self.P)),
            )

    def get_bias_rad(self):
        with self._lock:
            return self.b.copy()

    def load_bias(self, bias_rad):
        with self._lock:
            self.b = np.array(bias_rad, dtype=float)
            print(f"[INFO] bias pre-loaded: {np.degrees(self.b).round(3)} deg/s")


class Alignment:
    """ICM->BNO rotation matrix. Defaults to identity."""
    def __init__(self):
        self.R = np.eye(3)
        if os.path.exists(ALIGN_FILE):
            try:
                data = json.load(open(ALIGN_FILE))
                self.R = np.array(data["align_matrix"], dtype=float)
                assert self.R.shape == (3,3)
                print(f"[INFO] mounting alignment loaded from {ALIGN_FILE}")
            except Exception as e:
                print(f"[WARN] couldn't load alignment ({e}), using identity")
        else:
            print(f"[INFO] no {ALIGN_FILE} found, assuming sensors are co-aligned")

    def apply(self, gx, gy, gz):
        v = self.R @ np.array([gx, gy, gz])
        return float(v[0]), float(v[1]), float(v[2])


class Watchdog:
    def __init__(self):
        self._start   = None
        self.triggered = False

    def check(self, q_fused, q_bno, t):
        div = angle_between(q_fused, q_bno)
        if div > WATCHDOG_DEG:
            if self._start is None:
                self._start = t
            elif (t - self._start) > WATCHDOG_TIME and not self.triggered:
                self.triggered = True
                print(f"\n\033[1;31m[WATCHDOG] diverged {div:.1f}deg for "
                      f">{WATCHDOG_TIME}s - check sensor!\033[0m")
        else:
            if self.triggered:
                print(f"\n\033[32m[WATCHDOG] resolved ({div:.1f}deg)\033[0m")
                self.triggered = False
            self._start = None
        return div, self.triggered


# Naive gyro+BNO complementary filter, kept purely as a baseline to show the
# ESKF is actually worth the extra complexity. Not used for anything downstream.

class ComplementaryFilter:

    ALPHA = 0.98  # weight on gyro integration between BNO updates

    def __init__(self):
        self.roll = self.pitch = self.yaw = 0.0
        self._last_t = None
        self._ready  = False

    def update(self, bno_rpy, gyro_dps, t):
        if not self._ready:
            self.roll, self.pitch, self.yaw = bno_rpy
            self._last_t = t
            self._ready  = True
            return self.roll, self.pitch, self.yaw

        dt = t - self._last_t
        self._last_t = t
        if dt <= 0 or dt > 0.5:
            return self.roll, self.pitch, self.yaw

        self.roll  = self._step(self.roll,  gyro_dps[0], bno_rpy[0], dt)
        self.pitch = self._step(self.pitch, gyro_dps[1], bno_rpy[1], dt)
        self.yaw   = self._step(self.yaw,   gyro_dps[2], bno_rpy[2], dt)
        return self.roll, self.pitch, self.yaw

    @classmethod
    def _step(cls, angle, rate_dps, measured_deg, dt):
        predicted = angle + rate_dps * dt
        # shortest-path angle diff so this doesn't break across the +-180 wrap
        diff = ((measured_deg - predicted + 180) % 360) - 180
        return predicted + (1 - cls.ALPHA) * diff


class DepthFilter:
    """1D KF on Bar10. State [depth, Vz]."""

    def __init__(self):
        self.x = np.array([0., 0.])
        self.P = np.diag([1.0, 1.0]).astype(float)
        self._lock   = threading.Lock()
        self._last_t = None
        self._last_update_wall = None
        self._ready  = False

    def update(self, depth_meas, t):
        with self._lock:
            if not self._ready:
                self.x = np.array([depth_meas, 0.])
                self._last_t = t
                self._ready  = True
                self._last_update_wall = time.monotonic()
                return

            dt = t - self._last_t
            self._last_t = t
            if dt <= 0 or dt > 1.0:
                return

            F = np.array([[1., dt],
                          [0., 1.]])
            Q = np.array([[DEPTH_Q_POS,            0.],
                          [          0., DEPTH_Q_VEL]]) * dt
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q

            H = np.array([[1., 0.]])
            R = np.array([[DEPTH_R_MEAS]])
            y = depth_meas - (H @ self.x)[0]
            S = (H @ self.P @ H.T)[0, 0] + R[0, 0]
            K = (self.P @ H.T).flatten() / S
            self.x = self.x + K * y

            IKH = np.eye(2) - np.outer(K, H[0])
            self.P = IKH @ self.P @ IKH.T + np.outer(K, K) * R[0, 0]
            self._last_update_wall = time.monotonic()

    def get_state(self):
        """Last known [depth, Vz], regardless of how stale - fine for logging."""
        with self._lock:
            return float(self.x[0]), float(self.x[1])

    def is_fresh(self, max_age=DEPTH_MAX_AGE):
        """
        Whether get_state() reflects a real, recent reading. Use this - not
        "does a depth thread exist" - to decide whether to blend Vz into the
        state vector: a depth source that's never sent real data, or has gone
        silent, should fall back to accel-only rather than pin Vz at a frozen
        or phantom-zero value.
        """
        with self._lock:
            if not self._ready or self._last_update_wall is None:
                return False
            return (time.monotonic() - self._last_update_wall) <= max_age


class StateVector:
    """
    6-DOF state [vx,vy,vz,p,q,r] for the thruster allocator.
    vx/vy drift since we've got no horizontal reference, vz blended with depth.
    """

    GRAVITY     = 9.81
    LEAK        = 0.995
    ACCEL_DEAD  = 0.05
    DEPTH_BLEND = 0.95    # 0=accel only, 1=depth only

    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self._last_t = None

    def reset(self):
        self.vx = self.vy = self.vz = 0.0

    def update(self, ax_b, ay_b, az_b,
               roll_deg, pitch_deg, yaw_deg,
               omega_x, omega_y, omega_z,
               vz_depth=None):
        now = time.monotonic()
        if self._last_t is None:
            self._last_t = now
            return self._sv(0.0, 0.0, 0.0, omega_x, omega_y, omega_z)

        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.5:
            return self._sv(self.vx, self.vy, self.vz, omega_x, omega_y, omega_z)

        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        # subtract gravity projection
        g_bx = -sp        * self.GRAVITY
        g_by =  cp * sr   * self.GRAVITY
        g_bz =  cp * cr   * self.GRAVITY

        ax_lin = ax_b - g_bx
        ay_lin = ay_b - g_by
        az_lin = az_b - g_bz

        if abs(ax_lin) < self.ACCEL_DEAD: ax_lin = 0.0
        if abs(ay_lin) < self.ACCEL_DEAD: ay_lin = 0.0
        if abs(az_lin) < self.ACCEL_DEAD: az_lin = 0.0

        # body -> world
        ax_w = cy*cp*ax_lin + (cy*sp*sr - sy*cr)*ay_lin + (cy*sp*cr + sy*sr)*az_lin
        ay_w = sy*cp*ax_lin + (sy*sp*sr + cy*cr)*ay_lin + (sy*sp*cr - cy*sr)*az_lin
        az_w = -sp   *ax_lin + (cp*sr)          *ay_lin + (cp*cr)          *az_lin

        self.vx = self.LEAK * self.vx + ax_w * dt
        self.vy = self.LEAK * self.vy + ay_w * dt
        self.vz = self.LEAK * self.vz + az_w * dt

        if vz_depth is not None:
            self.vz = (1 - self.DEPTH_BLEND) * self.vz + self.DEPTH_BLEND * vz_depth

        return self._sv(self.vx, self.vy, self.vz, omega_x, omega_y, omega_z)

    @staticmethod
    def _sv(vx, vy, vz, p, q, r):
        return {"vx": vx, "vy": vy, "vz": vz, "p": p, "q": q, "r": r}


class Bar10:
    """Bar10 via ms5837."""
    def __init__(self):
        self.dev = ms5837.MS5837_30BA()
        if not self.dev.init():
            raise RuntimeError("Bar10 init failed - check I2C wiring")
        # freshwater for pool, switch to DENSITY_SALTWATER for sea
        self.dev.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        print("[OK] Bar10 depth sensor")

    def read(self):
        try:
            if self.dev.read():
                return float(self.dev.depth())
            return None
        except Exception as e:
            print(f"[WARN] Bar10 read error: {e}")
            return None


class MockBar10:
    """Descend, hover, ascend."""
    def __init__(self):
        self._t0 = time.monotonic()
        print("[MOCK] Bar10 - simulating descent/hover/ascent profile")

    def read(self):
        import random
        t = time.monotonic() - self._t0
        if t < 10:
            true_depth = 0.2 * t
        elif t < 20:
            true_depth = 2.0 + 0.05*math.sin(0.5*t)
        else:
            true_depth = 2.0 - 0.15 * (t - 20)
        return max(0.0, true_depth + random.gauss(0, 0.002))


class DepthThread(threading.Thread):
    def __init__(self, reader, depth_filter, hz=DEPTH_RATE_HZ):
        super().__init__(daemon=True, name="Depth")
        self._reader = reader
        self._filter = depth_filter
        self._dt     = 1.0 / hz
        self._halt   = threading.Event()
        self._last_raw = 0.0
        self._rlock  = threading.Lock()

    def last_raw_depth(self):
        with self._rlock:
            return self._last_raw

    def stop(self):
        self._halt.set()

    def run(self):
        while not self._halt.is_set():
            t0 = time.monotonic()
            d = self._reader.read()
            if d is not None:
                with self._rlock:
                    self._last_raw = d
                self._filter.update(d, time.monotonic())
            sleep = self._dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)


def save_bias(bias_rad):
    try:
        json.dump({"bias_rad_per_s": [float(x) for x in bias_rad]},
                  open(BIAS_FILE, "w"), indent=2)
        print(f"[INFO] bias saved: {np.degrees(np.array(bias_rad)).round(3)} deg/s")
    except Exception as e:
        print(f"[WARN] couldn't save bias: {e}")

def load_bias_file():
    if not os.path.exists(BIAS_FILE):
        return None
    try:
        b = np.array(json.load(open(BIAS_FILE))["bias_rad_per_s"])
        print(f"[INFO] loaded bias: {np.degrees(b).round(3)} deg/s")
        return b
    except Exception as e:
        print(f"[WARN] couldn't load bias: {e}")
        return None


class BNO085:
    def __init__(self, hz=BNO_RATE_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dev = BNO08X_I2C(i2c, address=BNO085_ADDR)
        interval = int(1_000_000 / hz)
        self.dev.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=interval)
        self.dev.enable_feature(BNO_REPORT_GYROSCOPE,       report_interval=interval)
        self.dev.enable_feature(BNO_REPORT_CALIBRATION_STATUS)
        self._warned = False
        print(f"[OK] BNO085 @ 0x{BNO085_ADDR:02X}, {hz}Hz")

    def read(self):
        try:
            q = self.dev.quaternion
            if q is None:
                return None, None
            qx, qy, qz, qw = q
            try:
                cal = self.dev.calibration_status
                cs  = {"accel": cal[0] if cal else 0,
                       "gyro":  cal[1] if cal else 0,
                       "mag":   cal[2] if cal else 0}
            except Exception:
                cs = {"accel": 0, "gyro": 0, "mag": 0}
            return [qw, qx, qy, qz], cs
        except Exception as e:
            if not self._warned:
                print(f"[WARN] BNO085 read error: {e}")
                self._warned = True
            return None, None


class ICM20948:
    def __init__(self):
        self.dev    = icm20948.ICM20948(i2c_addr=ICM20948_ADDR)
        self._scale = None   # set on first read
        print(f"[OK] ICM-20948 @ 0x{ICM20948_ADDR:02X}")

    def read(self):
        try:
            ax, ay, az, gx, gy, gz = self.dev.read_accelerometer_gyro_data()

            # some versions of the lib return rad/s, some deg/s. stationary gyro
            # should be tiny either way so just look at the magnitude
            if self._scale is None:
                mag = math.sqrt(gx*gx + gy*gy + gz*gz)
                if mag > 1e-6:
                    if mag < 0.5:
                        self._scale = math.degrees(1.0)
                        print(f"[INFO] ICM gyro in rad/s, converting")
                    else:
                        self._scale = 1.0
                        print(f"[INFO] ICM gyro in deg/s")
                else:
                    return None

            return ax, ay, az, gx*self._scale, gy*self._scale, gz*self._scale
        except Exception as e:
            print(f"[WARN] ICM-20948 read error: {e}")
            return None


class MockBNO085:
    def __init__(self, hz=BNO_RATE_HZ):
        self._t0 = time.monotonic()
        print("[MOCK] BNO085")

    def read(self):
        t = time.monotonic() - self._t0
        q = from_euler(15*math.sin(0.3*t), 10*math.sin(0.2*t+0.5), 30*math.sin(0.1*t))
        return q, {"accel": 3, "gyro": 3, "mag": min(3, int(t/2))}


class MockICM20948:
    _BIAS = [0.8, -0.5, 0.4]

    def __init__(self):
        self._t0 = time.monotonic()
        print(f"[MOCK] ICM-20948, hidden bias {self._BIAS}")

    def read(self):
        import random
        t = time.monotonic() - self._t0

        # match BNO mock so gravity projects correctly
        roll  = 15*math.sin(0.3*t)
        pitch = 10*math.sin(0.2*t+0.5)
        r, p = math.radians(roll), math.radians(pitch)

        g = 9.81
        ax = -math.sin(p)              * g + random.gauss(0, 0.05)
        ay =  math.cos(p) * math.sin(r) * g + random.gauss(0, 0.05)
        az =  math.cos(p) * math.cos(r) * g + random.gauss(0, 0.05)

        gx = 0.3*15*math.cos(0.3*t)      + self._BIAS[0] + random.gauss(0, 0.3)
        gy = 0.2*10*math.cos(0.2*t+0.5)  + self._BIAS[1] + random.gauss(0, 0.3)
        gz = 0.1*30*math.cos(0.1*t)      + self._BIAS[2] + random.gauss(0, 0.3)

        return ax, ay, az, gx, gy, gz


class STM32Reader:
    """
    Alternative to direct I2C. STM32 bridges the sensors to the Jetson over UART.
    Packet: IMU,qw,qx,qy,qz,ax,ay,az,gx,gy,gz[,depth_m]\\n
    """

    HEADER       = "IMU"
    FIELDS       = 11   # without depth
    FIELDS_DEPTH = 12   # with depth

    def __init__(self, port, baud=115200, timeout=0.1):
        if not SERIAL_AVAILABLE:
            raise RuntimeError("pyserial not installed - pip3 install pyserial")
        self._ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self._qw = 1.0; self._qx = self._qy = self._qz = 0.0
        self._ax = 0.0; self._ay = 0.0; self._az = 9.81
        self._gx = self._gy = self._gz = 0.0
        self._depth_m = 0.0
        self._has_depth = False
        self._good_packets = 0
        self._bad_packets  = 0
        self._ser.reset_input_buffer()
        print(f"[OK] STM32 on {port} @ {baud} baud")

    def _poll(self):
        try:
            raw = self._ser.readline()
            if not raw:
                return False
            parts = raw.decode("ascii", errors="replace").strip().split(",")
            if parts[0] != self.HEADER or len(parts) not in (self.FIELDS, self.FIELDS_DEPTH):
                self._bad_packets += 1
                return False
            vals = [float(p) for p in parts[1:]]
            self._qw, self._qx, self._qy, self._qz = vals[0:4]
            self._ax, self._ay, self._az           = vals[4:7]
            self._gx, self._gy, self._gz           = vals[7:10]
            if len(parts) == self.FIELDS_DEPTH:
                self._depth_m   = vals[10]
                self._has_depth = True
            self._good_packets += 1
            return True
        except (ValueError, UnicodeDecodeError):
            self._bad_packets += 1
            return False

    def read_bno(self):
        self._poll()
        # no real cal status from stm32, just assume all good
        cal = {"accel": 3, "gyro": 3, "mag": 3}
        return [self._qw, self._qx, self._qy, self._qz], cal

    def read_icm(self):
        self._poll()
        return self._ax, self._ay, self._az, self._gx, self._gy, self._gz

    def read_depth(self):
        if not self._has_depth:
            return None
        return self._depth_m


class PredictThread(threading.Thread):
    """high-rate ICM reads + ekf.predict()"""

    def __init__(self, reader, ekf, alignment, hz=ICM_RATE_HZ):
        super().__init__(daemon=True, name="ICM-predict")
        self._reader  = reader
        self._ekf     = ekf
        self._align   = alignment
        self._dt      = 1.0 / hz
        self._halt  = threading.Event()
        self._gyro  = (0., 0., 0.)
        self._accel = (0., 0., 9.81)
        self._glock = threading.Lock()

    def last_gyro(self):
        with self._glock:
            return self._gyro

    def last_accel(self):
        with self._glock:
            return self._accel

    def get_angular_velocity(self):
        """bias-corrected (wx,wy,wz) deg/s for the control loop"""
        with self._glock:
            gx, gy, gz = self._gyro
        bias_rad = self._ekf.get_bias_rad()
        bias_dps = np.degrees(bias_rad)
        return (gx - bias_dps[0], gy - bias_dps[1], gz - bias_dps[2])

    def stop(self):
        self._halt.set()

    def run(self):
        while not self._halt.is_set():
            t0 = time.monotonic()
            reading = self._reader.read()
            if reading is not None:
                ax, ay, az, gx, gy, gz = reading
                gx, gy, gz = self._align.apply(gx, gy, gz)
                ax, ay, az = self._align.apply(ax, ay, az)
                with self._glock:
                    self._gyro  = (gx, gy, gz)
                    self._accel = (ax, ay, az)
                self._ekf.predict(gx, gy, gz, time.monotonic())
            sleep = self._dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)


FIELDS = [
    "timestamp_s",
    "bno_roll", "bno_pitch", "bno_yaw",
    "bno_qw", "bno_qx", "bno_qy", "bno_qz",
    "cal_accel", "cal_gyro", "cal_mag",
    "icm_ax", "icm_ay", "icm_az",
    "icm_gx", "icm_gy", "icm_gz",
    "cf_roll", "cf_pitch", "cf_yaw",
    "omega_x", "omega_y", "omega_z",
    "fused_roll", "fused_pitch", "fused_yaw",
    "fused_qw", "fused_qx", "fused_qy", "fused_qz",
    "bias_x", "bias_y", "bias_z",
    "err_roll", "err_pitch", "err_yaw",
    "yaw_ok", "P_trace", "divergence", "watchdog",
    "depth_raw_m", "depth_m", "vz_m_per_s",
    "sv_vx", "sv_vy", "sv_vz", "sv_p", "sv_q", "sv_r",
]

def open_log(path):
    f = open(path, "w", newline="")
    w = csv.DictWriter(f, fieldnames=FIELDS)
    w.writeheader()
    return f, w


def show(bno_rpy, fused_rpy, bias, cal, yaw_ok, div, wd, P, t, skipped):
    err = tuple(f-b for f,b in zip(fused_rpy, bno_rpy))
    ec  = "\033[31m" if any(abs(e) > 2 for e in err) else "\033[90m"
    wc  = "\033[31m" if wd else "\033[90m"
    yc  = "\033[32m" if yaw_ok else "\033[33m"
    sys.stdout.write("\033[10A")
    print(f"\033[1;36m{'':14} {'Roll':>9} {'Pitch':>9} {'Yaw':>9}\033[0m")
    print(f"\033[32m{'BNO085':<14} {bno_rpy[0]:>+9.2f}  {bno_rpy[1]:>+9.2f}  {bno_rpy[2]:>+9.2f}\033[0m")
    print(f"\033[36m{'ESKF':<14} {fused_rpy[0]:>+9.2f}  {fused_rpy[1]:>+9.2f}  {fused_rpy[2]:>+9.2f}\033[0m")
    print(f"{ec}{'err':<14} {err[0]:>+9.2f}  {err[1]:>+9.2f}  {err[2]:>+9.2f}\033[0m")
    print(f"\033[33m  bias [{bias[0]:+.3f}, {bias[1]:+.3f}, {bias[2]:+.3f}] deg/s\033[0m")
    print(f"\033[90m  cal a={cal['accel']} g={cal['gyro']} m={cal['mag']}  "
          f"yaw {yc}{'ok' if yaw_ok else 'warming'}\033[0m")
    print(f"{wc}  div={div:.2f}deg  watchdog={'ALERT' if wd else 'ok'}\033[0m")
    print(f"\033[90m  P={P:.2e}  t={t:.1f}s  skipped={skipped}\033[0m")
    print(); print()


def make_plot(csv_path, out_path):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("[WARN] matplotlib not found, skipping plot")
        return

    rows = list(csv.DictReader(open(csv_path)))
    if len(rows) < 2:
        return

    def c(n): return np.array([float(r[n]) for r in rows])
    def has(n): return n in rows[0]

    ts = c("timestamp_s")
    bno_r, bno_p, bno_y    = c("bno_roll"),    c("bno_pitch"),    c("bno_yaw")
    f_r,   f_p,   f_y      = c("fused_roll"),  c("fused_pitch"),  c("fused_yaw")
    e_r,   e_p,   e_y      = c("err_roll"),    c("err_pitch"),    c("err_yaw")
    bx, by, bz             = c("bias_x"),      c("bias_y"),       c("bias_z")
    mag_cal  = c("cal_mag")
    yaw_ok   = c("yaw_ok")
    P_trace  = np.maximum(c("P_trace"), 1e-12)
    div      = c("divergence")
    wd       = c("watchdog")

    has_cf = has("cf_roll")
    if has_cf:
        cf_r, cf_p, cf_y = c("cf_roll"), c("cf_pitch"), c("cf_yaw")

    has_sv = has("sv_vx")
    if has_sv:
        sv_vx, sv_vy, sv_vz = c("sv_vx"), c("sv_vy"), c("sv_vz")
        sv_p,  sv_q,  sv_r  = c("sv_p"),  c("sv_q"),  c("sv_r")

    has_depth = has("depth_m")
    if has_depth:
        depth_raw = c("depth_raw_m")
        depth     = c("depth_m")
        vz        = c("vz_m_per_s")

    # Colours
    BNO_C  = "#00e5ff"
    BIAS_C = "#ff9800"
    EKF_C  = "#a259ff"
    ERR_C  = "#ff4444"
    LIN_C  = "#00e676"  # linear velocity — green
    ANG_C  = "#ff6d00"  # angular rate — orange
    BG     = "#1a1d27"

    # 4 columns: sources / divergence / EKF residual+bias / state vector
    # 3 top rows for roll/pitch/yaw, then extra rows for diagnostics+depth
    ncols = 3 if has_sv else 2
    diag_rows = 1  # bias + mag + Ptrace + watchdog on row 3
    depth_rows = 1 if has_depth else 0
    nrows = 3 + diag_rows + depth_rows

    fig_h = 4 * nrows - 1
    fig = plt.figure(figsize=(6 * ncols, fig_h))
    fig.patch.set_facecolor("#0f1117")
    gs = gridspec.GridSpec(nrows, ncols, figure=fig, hspace=0.55, wspace=0.32)

    def ax_style(ax, title, ylabel="deg"):
        ax.set_facecolor(BG)
        ax.tick_params(colors="#aaa", labelsize=8)
        ax.set_title(title, color="white", fontsize=9, pad=5)
        ax.set_xlabel("time (s)", color="#aaa", fontsize=8)
        ax.set_ylabel(ylabel, color="#aaa", fontsize=8)
        for sp in ax.spines.values(): sp.set_edgecolor("#333344")
        ax.grid(True, color="#2a2a3a", lw=0.5, ls="--")

    CF_C = "#ffb300"

    labels   = ["Roll", "Pitch", "Yaw"]
    bno_cols = [bno_r, bno_p, bno_y]
    f_cols   = [f_r,   f_p,   f_y]
    e_cols   = [e_r,   e_p,   e_y]
    if has_cf:
        cf_cols = [cf_r, cf_p, cf_y]

    if has_sv:
        sv_lin = [sv_vx, sv_vy, sv_vz]
        sv_ang = [sv_p,  sv_q,  sv_r]

    for i, label in enumerate(labels):
        ax0 = fig.add_subplot(gs[i, 0])
        ax0.plot(ts, bno_cols[i], BNO_C, lw=1.6, label="BNO085")
        if has_cf:
            ax0.plot(ts, cf_cols[i], CF_C, lw=1.2, ls="--", label="Comp filter")
        ax0.plot(ts, f_cols[i], EKF_C, lw=1.2, ls="-.", alpha=0.95, label="ESKF")
        if label == "Yaw" and (yaw_ok < 0.5).any():
            ax0.fill_between(ts, bno_cols[i].min()-5, bno_cols[i].max()+5,
                             where=(yaw_ok<0.5), alpha=0.1, color="#ff9800",
                             label="mag warming up")
        ax_style(ax0, f"{label} — All Sources" if has_cf else f"{label} — BNO085 vs ESKF")
        ax0.legend(fontsize=7, facecolor=BG, edgecolor="#444", labelcolor="white")

        ax1 = fig.add_subplot(gs[i, 1])
        rms = np.sqrt(np.mean(e_cols[i]**2))
        ax1.plot(ts, e_cols[i], ERR_C, lw=1.0, label=f"RMS={rms:.3f}deg")
        ax1.axhline(0, color="#555566", lw=0.7, ls=":")
        ax1.fill_between(ts, e_cols[i], alpha=0.15, color=ERR_C)
        ax_style(ax1, f"{label} Error (ESKF − BNO085)", ylabel="delta (deg)")
        ax1t = ax1.twinx()
        bias_col = [bx, by, bz][i]
        ax1t.plot(ts, bias_col, BIAS_C, lw=1.0, ls=":", label="bias (deg/s)")
        ax1t.set_ylabel("bias (deg/s)", color=BIAS_C, fontsize=8)
        ax1t.tick_params(colors=BIAS_C, labelsize=8)
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax1t.get_legend_handles_labels()
        ax1.legend(h1+h2, l1+l2, fontsize=7, facecolor=BG, edgecolor="#444",
                   labelcolor="white")

        if ncols >= 3 and has_sv:
            ax2 = fig.add_subplot(gs[i, 2])
            ax2.plot(ts, sv_lin[i], LIN_C, lw=1.3, label=f"v{'xyz'[i]} (m/s)")
            ax2.axhline(0, color="#555566", lw=0.6, ls=":")
            ax_style(ax2, f"State — {label} axis", ylabel="lin vel (m/s)")
            ax2t = ax2.twinx()
            ax2t.plot(ts, sv_ang[i], ANG_C, lw=1.0, ls="--",
                      label=f"{'pqr'[i]} (deg/s)")
            ax2t.set_ylabel("ang rate (deg/s)", color=ANG_C, fontsize=8)
            ax2t.tick_params(colors=ANG_C, labelsize=8)
            h1, l1 = ax2.get_legend_handles_labels()
            h2, l2 = ax2t.get_legend_handles_labels()
            ax2.legend(h1+h2, l1+l2, fontsize=7, facecolor=BG, edgecolor="#444",
                       labelcolor="white")

    ax_b = fig.add_subplot(gs[3, 0])
    ax_b.plot(ts, bx, "#ff9800", lw=1.2, label="X")
    ax_b.plot(ts, by, "#00e5ff", lw=1.2, label="Y")
    ax_b.plot(ts, bz, "#a259ff", lw=1.2, label="Z")
    ax_b.axhline(0, color="#555566", lw=0.6, ls=":")
    ax_style(ax_b, "gyro bias estimate", ylabel="deg/s")
    ax_b.legend(fontsize=7.5, facecolor=BG, edgecolor="#444", labelcolor="white")

    ax_m = fig.add_subplot(gs[3, 1])
    ax_m.plot(ts, mag_cal, "#ffdd44", lw=1.4, label="mag cal (0-3)")
    ax_m.axhline(MAG_CAL_MIN, color="#ff4444", lw=1.0, ls="--",
                 label=f"yaw trust threshold ({MAG_CAL_MIN})")
    ax_m.set_ylim(-0.2, 3.5); ax_m.set_yticks([0,1,2,3])
    ax_style(ax_m, "BNO085 mag calibration", ylabel="level")
    ax_m.legend(fontsize=7.5, facecolor=BG, edgecolor="#444", labelcolor="white")

    if ncols >= 3:
        ax_p = fig.add_subplot(gs[3, 2])
        ax_p.plot(ts, div, "#ff9800", lw=1.2, label="divergence")
        ax_p.axhline(WATCHDOG_DEG, color="#ff4444", lw=1.0, ls="--",
                     label=f"watchdog ({WATCHDOG_DEG}deg)")
        if wd.any():
            ax_p.fill_between(ts, 0, div, where=(wd>0.5), alpha=0.25,
                              color="#ff4444", label="watchdog active")
        ax_p.fill_between(ts, 0, div, alpha=0.12, color="#ff9800")
        ax_style(ax_p, "divergence / watchdog")
        ax_p.legend(fontsize=7.5, facecolor=BG, edgecolor="#444", labelcolor="white")

    if has_depth:
        ax_dep = fig.add_subplot(gs[4, 0])
        ax_dep.plot(ts, depth_raw, "#666688", lw=0.7, alpha=0.6, label="Bar10 raw")
        ax_dep.plot(ts, depth,     "#00e5ff", lw=1.3,             label="KF depth")
        ax_dep.invert_yaxis()
        ax_style(ax_dep, "depth (Bar10 + KF)", ylabel="depth (m)")
        ax_dep.legend(fontsize=7.5, facecolor=BG, edgecolor="#444", labelcolor="white")

        ax_vz = fig.add_subplot(gs[4, 1])
        ax_vz.plot(ts, vz, "#a259ff", lw=1.3, label="KF Vz")
        ax_vz.axhline(0, color="#555566", lw=0.6, ls=":")
        ax_style(ax_vz, "vertical velocity Vz (from depth KF)", ylabel="Vz (m/s)")
        ax_vz.legend(fontsize=7.5, facecolor=BG, edgecolor="#444", labelcolor="white")

        if ncols >= 3:
            ax_p2 = fig.add_subplot(gs[4, 2])
            ax_p2.semilogy(ts, P_trace, "#a259ff", lw=1.2)
            ax_style(ax_p2, "EKF covariance trace Tr(P) log", ylabel="Tr(P)")

    fig.suptitle("Hydrobotics ROV — IMU fusion (ESKF, full pipeline)",
                 color="white", fontsize=13, fontweight="bold", y=0.998)
    subtitle_bits = [
        f"BNO085 @ {BNO_RATE_HZ}Hz",
        f"ICM predict @ {ICM_RATE_HZ}Hz",
    ]
    if has_sv:    subtitle_bits.append("6-DOF state [vx,vy,vz,p,q,r]")
    if has_depth: subtitle_bits.append("Bar10 depth KF -> Vz")
    fig.text(0.5, 0.002, "  |  ".join(subtitle_bits),
             ha="center", color="#888899", fontsize=7)

    plt.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close()
    print(f"[OK] plot saved to {out_path}")


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("--log",          default="imu_log.csv")
    p.add_argument("--rate",         type=float, default=BNO_RATE_HZ,
                   dest="bno_rate",  help="BNO085 / main loop Hz")
    p.add_argument("--icm-rate",     type=float, default=ICM_RATE_HZ,
                   dest="icm_rate",  help="ICM-20948 predict Hz")
    p.add_argument("--i2c-bus",      type=int,   default=1)
    p.add_argument("--mock",         action="store_true")
    p.add_argument("--duration",     type=float, default=None)
    p.add_argument("--plot",         action="store_true")
    p.add_argument("--plot-out",     default=None)
    p.add_argument("--no-bias-load", action="store_true")
    p.add_argument("--no-bias-save", action="store_true")
    p.add_argument("--no-depth",     action="store_true", help="disable depth sensor")
    p.add_argument("--depth-rate",   type=float, default=DEPTH_RATE_HZ,
                   help=f"Bar10 depth sensor Hz (default {DEPTH_RATE_HZ})")
    p.add_argument("--stm32",        default=None, metavar="PORT",
                   help="Use STM32 UART instead of I2C (e.g. /dev/ttyUSB0 or COM3)")
    p.add_argument("--stm32-baud",   type=int, default=115200,
                   help="STM32 UART baud rate (default 115200)")
    return p.parse_args()


# --- main ---

def main():
    args = get_args()
    mock = args.mock or _FORCE_MOCK

    if args.stm32:
        try:
            stm = STM32Reader(args.stm32, baud=args.stm32_baud)
            # shim wrappers so the rest of the code doesn't care where data comes from
            class _BnoShim:
                def read(self_): return stm.read_bno()
            class _IcmShim:
                def read(self_): return stm.read_icm()
            class _DepthShim:
                def read(self_):
                    return stm.read_depth()
            bno = _BnoShim()
            icm = _IcmShim()
            # can't know at init time whether the firmware will ever send a
            # depth field (stm._has_depth only flips true after a packet with
            # one actually arrives), so always wire the shim up - DepthFilter.
            # is_fresh() is what decides whether we actually trust it later
            depth_source_override = _DepthShim()
        except Exception as e:
            print(f"[FATAL] STM32 init failed: {e}")
            sys.exit(1)
    else:
        bno = MockBNO085(args.bno_rate)   if (mock or not BNO_AVAILABLE) else BNO085(args.bno_rate)
        icm = MockICM20948()              if (mock or not ICM_AVAILABLE) else ICM20948()
        depth_source_override = None

    align    = Alignment()
    ekf      = ESKF()
    watchdog = Watchdog()
    sv       = StateVector()
    cf       = ComplementaryFilter()

    depth_thread = None
    depth_filter = DepthFilter()
    if not args.no_depth:
        try:
            if depth_source_override is not None:
                depth_sensor = depth_source_override
            else:
                depth_sensor = (MockBar10() if (mock or not DEPTH_AVAILABLE) else Bar10())
            depth_thread = DepthThread(depth_sensor, depth_filter, hz=args.depth_rate)
        except Exception as e:
            print(f"[WARN] depth sensor init failed ({e}), continuing without")

    if not args.no_bias_load:
        b = load_bias_file()
        if b is not None:
            ekf.load_bias(b)

    predict_thread = PredictThread(icm, ekf, align, hz=args.icm_rate)
    log_f, log_w   = open_log(args.log)

    print(f"\n[INFO] logging to {args.log}")
    print(f"[INFO] BNO085={args.bno_rate}Hz  ICM={args.icm_rate}Hz"
          + (f"  Depth={args.depth_rate}Hz" if depth_thread else "  Depth=disabled"))
    if args.duration:
        print(f"[INFO] running for {args.duration}s")
    print("\nctrl+c to stop\n")
    print("\n" * 10)

    predict_thread.start()
    if depth_thread:
        depth_thread.start()
    t0_global = time.monotonic()
    n = skipped = 0

    try:
        while True:
            t0 = time.monotonic()
            t  = t0 - t0_global
            if args.duration and t >= args.duration:
                break

            bno_q, cal = bno.read()
            if bno_q is None or cal is None:
                skipped += 1
                time.sleep(1.0 / args.bno_rate)
                continue

            mag_cal = cal["mag"]
            yaw_ok  = mag_cal >= MAG_CAL_MIN
            ekf.correct(bno_q, mag_cal, t0)

            fused_q, bias_dps, P_trace = ekf.snapshot()
            gx, gy, gz = predict_thread.last_gyro()
            ax, ay, az = predict_thread.last_accel()
            wx, wy, wz = predict_thread.get_angular_velocity()
            depth_m, vz = depth_filter.get_state()
            depth_raw   = depth_thread.last_raw_depth() if depth_thread else 0.0
            depth_fresh = depth_filter.is_fresh() if depth_thread else False

            bno_rpy   = to_euler(bno_q)
            fused_rpy = to_euler(fused_q)
            cf_rpy    = cf.update(bno_rpy, (gx, gy, gz), t0)
            err       = tuple(round(f-b, 4) for f,b in zip(fused_rpy, bno_rpy))
            div, wd   = watchdog.check(fused_q, bno_q, t0)

            state = sv.update(
                ax, ay, az,
                fused_rpy[0], fused_rpy[1], fused_rpy[2],
                wx, wy, wz,
                vz_depth=vz if depth_fresh else None,
            )

            log_w.writerow({
                "timestamp_s":  round(t, 4),
                "bno_roll":  round(bno_rpy[0], 4),
                "bno_pitch": round(bno_rpy[1], 4),
                "bno_yaw":   round(bno_rpy[2], 4),
                "bno_qw": round(bno_q[0],6), "bno_qx": round(bno_q[1],6),
                "bno_qy": round(bno_q[2],6), "bno_qz": round(bno_q[3],6),
                "cal_accel": cal["accel"], "cal_gyro": cal["gyro"], "cal_mag": mag_cal,
                "icm_ax": round(ax,4), "icm_ay": round(ay,4), "icm_az": round(az,4),
                "icm_gx": round(gx,4), "icm_gy": round(gy,4), "icm_gz": round(gz,4),
                "cf_roll": round(cf_rpy[0],4), "cf_pitch": round(cf_rpy[1],4),
                "cf_yaw": round(cf_rpy[2],4),
                "omega_x": round(wx,4), "omega_y": round(wy,4), "omega_z": round(wz,4),
                "fused_roll":  round(fused_rpy[0],4),
                "fused_pitch": round(fused_rpy[1],4),
                "fused_yaw":   round(fused_rpy[2],4),
                "fused_qw": round(fused_q[0],6), "fused_qx": round(fused_q[1],6),
                "fused_qy": round(fused_q[2],6), "fused_qz": round(fused_q[3],6),
                "bias_x": round(bias_dps[0],5),
                "bias_y": round(bias_dps[1],5),
                "bias_z": round(bias_dps[2],5),
                "err_roll": err[0], "err_pitch": err[1], "err_yaw": err[2],
                "yaw_ok":    int(yaw_ok),
                "P_trace":   round(P_trace, 10),
                "divergence": round(div, 4),
                "watchdog":   int(wd),
                "depth_raw_m": round(depth_raw, 4),
                "depth_m":     round(depth_m, 4),
                "vz_m_per_s":  round(vz, 4),
                "sv_vx": round(state["vx"], 4),
                "sv_vy": round(state["vy"], 4),
                "sv_vz": round(state["vz"], 4),
                "sv_p":  round(state["p"],  4),
                "sv_q":  round(state["q"],  4),
                "sv_r":  round(state["r"],  4),
            })
            n += 1
            show(bno_rpy, fused_rpy, bias_dps, cal, yaw_ok, div, wd, P_trace, t, skipped)

            sleep = 1.0/args.bno_rate - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        pass
    finally:
        predict_thread.stop()
        predict_thread.join(timeout=2.0)
        if depth_thread:
            depth_thread.stop()
            depth_thread.join(timeout=2.0)
        log_f.close()
        dur = time.monotonic() - t0_global
        print(f"\n\n[DONE] {n} samples in {dur:.1f}s ({n/dur:.1f}Hz)  skipped={skipped}")
        if not args.no_bias_save:
            save_bias(ekf.get_bias_rad())

    if args.plot:
        out = args.plot_out or args.log.replace(".csv", ".png")
        make_plot(args.log, out)


if __name__ == "__main__":
    main()
