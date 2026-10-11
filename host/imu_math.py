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
