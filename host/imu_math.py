"""Python mirror of include/imu_filter.h, used for synthetic data and offline analysis."""
import numpy as np


def accel_roll(a):
    """Roll from gravity (deg). `a` is (..., 3) in g."""
    return np.degrees(np.arctan2(a[..., 1], a[..., 2]))


def accel_pitch(a):
    """Pitch from gravity (deg)."""
    return np.degrees(np.arctan2(-a[..., 0], np.hypot(a[..., 1], a[..., 2])))


class ComplementaryFilter:
    """angle = alpha * (angle + rate * dt) + (1 - alpha) * accel_angle,  alpha = tau / (tau + dt)"""

    def __init__(self, tau_s: float, dt_s: float):
        self.alpha, self.dt = tau_s / (tau_s + dt_s), dt_s
        self.roll = self.pitch = None

    def update(self, accel_g, gyro_dps):
        roll_acc, pitch_acc = accel_roll(np.asarray(accel_g)), accel_pitch(np.asarray(accel_g))
        if self.roll is None:                               # first sample: start from the accelerometer
            self.roll, self.pitch = roll_acc, pitch_acc
        else:
            self.roll = self.alpha * (self.roll + gyro_dps[0] * self.dt) + (1 - self.alpha) * roll_acc
            self.pitch = self.alpha * (self.pitch + gyro_dps[1] * self.dt) + (1 - self.alpha) * pitch_acc
        return self.roll, self.pitch


def run_filter(accel, gyro, dt, tau=0.5):
    """Filter a whole recording: accel (n, 3) in g, gyro (n, 3) in deg/s -> roll, pitch arrays (deg)."""
    f = ComplementaryFilter(tau, dt)
    out = np.array([f.update(a, g) for a, g in zip(accel, gyro)])
    return out[:, 0], out[:, 1]


def synthetic_imu(duration_s=40.0, fs=100, gyro_bias_dps=0.5, seed=0):
    """Simulated IMU on a swinging limb segment, with known ground truth.

    Pitch: still, 30 deg tilt, hold, return, then a 1 Hz swing.
    Error sources:
      gyro  : residual bias (drift after calibration) + white noise
      accel : white noise + linear acceleration during the swing (the sensor is off the pivot)
    Returns t, accel (n,3) g, gyro (n,3) deg/s, true pitch (deg).
    """
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration_s, 1 / fs)
    ramp = 30 * np.clip((t - 5) / 3, 0, 1) * (t < 18) + 30 * np.clip((22 - t) / 3, 0, 1) * (t >= 18)
    swing = 20 * np.sin(2 * np.pi * 1.0 * t) * np.clip((t - 26) / 2, 0, 1)   # fades in over 2 s
    pitch = ramp + swing
    rate = np.gradient(pitch, t)                                  # deg/s
    ang_acc = np.gradient(np.radians(rate), t)                    # rad/s^2

    r = np.radians(pitch)
    lever_m, g0 = 0.3, 9.81                                       # sensor 30 cm from the pivot
    tangential_g = lever_m * ang_acc / g0
    accel = np.column_stack([-np.sin(r) + tangential_g * np.cos(r),
                             np.zeros_like(t),
                             np.cos(r) + tangential_g * np.sin(r)])
    accel += 0.02 * rng.standard_normal(accel.shape)
    gyro = np.column_stack([np.zeros_like(t), rate, np.zeros_like(t)])
    gyro += gyro_bias_dps + 0.3 * rng.standard_normal(gyro.shape)
    return t, accel, gyro, pitch


class AttitudeEKF:
    """Extended Kalman filter for roll and pitch with online gyro-bias estimation.

    State x = [roll, pitch, bias_x, bias_y]  (rad, rad, rad/s, rad/s)

    Predict: nonlinear Euler-angle kinematics driven by the bias-corrected gyro,
        roll_dot  = p + sin(roll) tan(pitch) q + cos(roll) tan(pitch) r
        pitch_dot =     cos(roll) q          - sin(roll) r
    Update:  the accelerometer measures gravity in the body frame,
        h(x) = [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)]   (g)

    Unlike the complementary filter, the EKF (1) learns the gyro bias instead of
    settling at a tau * bias error, (2) uses the full 3-axis kinematics instead of
    a small-angle approximation, and (3) trusts the accelerometer less whenever
    |a| departs from 1 g (linear acceleration), via an adaptive measurement noise.
    """

    def __init__(self, dt_s, gyro_noise_dps=0.3, bias_walk_dps=0.01, accel_noise_g=0.05, accel_dyn_gain=10.0):
        self.dt = dt_s
        g = np.radians(gyro_noise_dps)
        self.Q = np.diag([g**2 * dt_s**2, g**2 * dt_s**2,
                          np.radians(bias_walk_dps)**2 * dt_s, np.radians(bias_walk_dps)**2 * dt_s])
        self.r0 = accel_noise_g**2
        self.dyn_gain = accel_dyn_gain
        self.x = None
        self.P = np.diag([np.radians(5)**2, np.radians(5)**2, np.radians(2)**2, np.radians(2)**2])

    def update(self, accel_g, gyro_dps):
        a = np.asarray(accel_g, dtype=float)
        if self.x is None:                                   # first sample: start from the accelerometer
            self.x = np.array([np.radians(accel_roll(a)), np.radians(accel_pitch(a)), 0.0, 0.0])
            return self.roll, self.pitch

        # ---- predict
        phi, th, bp, bq = self.x
        p, q, r = np.radians(np.asarray(gyro_dps, dtype=float))
        p, q = p - bp, q - bq
        sf, cf, tt, ct = np.sin(phi), np.cos(phi), np.tan(th), np.cos(th)
        dt = self.dt
        self.x = self.x + dt * np.array([p + sf * tt * q + cf * tt * r, cf * q - sf * r, 0.0, 0.0])
        A = np.array([[cf * tt * q - sf * tt * r, (sf * q + cf * r) / ct**2, -1.0, -sf * tt],
                      [-sf * q - cf * r,          0.0,                         0.0, -cf],
                      [0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0]])
        F = np.eye(4) + A * dt
        self.P = F @ self.P @ F.T + self.Q

        # ---- update with the accelerometer
        phi, th = self.x[:2]
        sf, cf, st, ct = np.sin(phi), np.cos(phi), np.sin(th), np.cos(th)
        h = np.array([-st, sf * ct, cf * ct])
        H = np.array([[0.0,      -ct,      0.0, 0.0],
                      [cf * ct,  -sf * st, 0.0, 0.0],
                      [-sf * ct, -cf * st, 0.0, 0.0]])
        dev = np.linalg.norm(a) - 1.0                        # departure from 1 g = linear acceleration
        R = np.eye(3) * self.r0 * (1.0 + self.dyn_gain * dev**2 / self.r0)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (a - h)
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T        # Joseph form keeps P symmetric positive definite
        return self.roll, self.pitch

    @property
    def roll(self):
        return float(np.degrees(self.x[0]))

    @property
    def pitch(self):
        return float(np.degrees(self.x[1]))

    @property
    def gyro_bias_dps(self):
        return np.degrees(self.x[2:])


def run_ekf(accel, gyro, dt, **kwargs):
    """EKF over a whole recording: accel (n, 3) in g, gyro (n, 3) in deg/s -> roll, pitch arrays (deg)."""
    f = AttitudeEKF(dt, **kwargs)
    out = np.array([f.update(a, g) for a, g in zip(accel, gyro)])
    return out[:, 0], out[:, 1]
