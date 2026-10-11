"""Sources of IMU samples: a real serial port, the Wokwi simulator, or a synthetic demo."""
import time

import numpy as np

from .imu_math import ComplementaryFilter, synthetic_imu

COLUMNS = ["t_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "roll_deg", "pitch_deg"]


def parse_line(line: str):
    """One CSV line from the firmware -> array of 9 floats.
    Comment lines ('#'), blank lines, and lines corrupted in transit return None."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) != len(COLUMNS):
        return None
    try:
        return np.array(parts, dtype=float)
    except ValueError:
        return None


def serial_lines(port: str, baud: int = 115200):
    """Yield text lines from a serial port.
    port: 'COM3', '/dev/ttyUSB0', or 'rfc2217://localhost:4000' for the Wokwi simulator."""
    import serial  # imported here so the rest of the package works without pyserial

    with serial.serial_for_url(port, baudrate=baud, timeout=1) as ser:
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("ascii", errors="ignore")


def synthetic_lines(fs: int = 100, realtime: bool = True):
    """Yield firmware-format lines from simulated data, looping indefinitely."""
    t, accel, gyro, _ = synthetic_imu(fs=fs)
    f = ComplementaryFilter(0.5, 1 / fs)
    yield "# synthetic demo stream\n"
    start = time.perf_counter()
    for k in range(10**9):
        i = k % len(t)
        roll, pitch = f.update(accel[i], gyro[i])
        if realtime:
            time.sleep(max(0.0, start + k / fs - time.perf_counter()))
        yield (f"{int(k * 1000 / fs)},{accel[i, 0]:.4f},{accel[i, 1]:.4f},{accel[i, 2]:.4f},"
               f"{gyro[i, 0]:.3f},{gyro[i, 1]:.3f},{gyro[i, 2]:.3f},{roll:.2f},{pitch:.2f}\n")
