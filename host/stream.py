"""Sources of IMU samples: a real serial port, the Wokwi simulator, or a synthetic demo."""
import time

import numpy as np

from .imu_math import AttitudeEKF, synthetic_imu

COLUMNS = ["t_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "roll_deg", "pitch_deg", "yaw_deg"]
# Logs from firmware before the magnetometer was added have no yaw column.
LEGACY_COLUMNS = len(COLUMNS) - 1


def parse_line(line: str):
    """One CSV line from the firmware -> array of len(COLUMNS) floats.
    Lines with the older 9 columns (no yaw) are accepted, with yaw set to NaN.
    Comment lines ('#'), blank lines, and lines corrupted in transit return None."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) not in (LEGACY_COLUMNS, len(COLUMNS)):
        return None
    try:
        values = np.array(parts, dtype=float)
    except ValueError:
        return None
    if len(parts) == LEGACY_COLUMNS:
        values = np.append(values, np.nan)
    return values


USB_SERIAL_HINTS = ("cp210", "ch340", "ch341", "ch343", "ch910", "ch9102", "ftdi", "usb serial", "usb-serial",
                    "usb single serial", "usb to uart", "uart", "slab", "usbserial", "usbmodem", "wchusbserial",
                    "ttyusb", "ttyacm")
# The chip's own USB port (USB JTAG/serial on the ESP32-C6, S3, C3). The firmware prints on the
# USB-to-UART chip instead (CH343 on the C6 DevKit), so these rank below a USB-serial chip.
NATIVE_USB_HINTS = ("jtag",)
NOT_A_BOARD_HINTS = ("bluetooth", "debug-console", "wlan-debug")


def list_serial_ports():
    """[(device, description)] for every serial port the operating system reports."""
    from serial.tools import list_ports
    return [(p.device, p.description or "") for p in list_ports.comports()]


def pick_serial_port(ports):
    """Best guess at the ESP32 among (device, description) pairs, or None if it is unclear.

    Windows names ports COM3, COM4, ...; macOS /dev/cu.usbserial-XXXX, /dev/cu.SLAB_USBtoUART,
    /dev/cu.wchusbserialXXXX or /dev/cu.usbmodemXXXX (the cu. device, not tty.); Linux /dev/ttyUSB0
    or /dev/ttyACM0. With both USB ports of an ESP32-C6 DevKit plugged in, the CH343 port wins over
    the chip's native USB JTAG/serial port.
    """
    scored = []
    for dev, desc in ports:
        text = f"{dev} {desc}".lower()
        if any(h in text for h in NOT_A_BOARD_HINTS):
            continue
        if any(h in text for h in NATIVE_USB_HINTS):
            score = 1
        elif any(h in text for h in USB_SERIAL_HINTS):
            score = 2
        else:
            score = 0
        if dev.startswith("/dev/tty."):        # macOS lists each port twice; the cu. one is right
            score -= 1
        if dev.startswith("/dev/ttyS"):        # Linux built-in serial ports, never the board
            score -= 3
        scored.append((score, dev))
    if not scored:
        return None
    scored.sort(key=lambda c: -c[0])
    if scored[0][0] > 0 and (len(scored) == 1 or scored[1][0] < scored[0][0]):
        return scored[0][1]
    return scored[0][1] if len(scored) == 1 and scored[0][0] >= 0 else None


def serial_lines(port: str, baud: int = 115200):
    """Yield text lines from a serial port.
    port: 'COM3' (Windows), '/dev/cu.usbserial-0001' (macOS), '/dev/ttyUSB0' (Linux),
    or 'rfc2217://localhost:4000' for the Wokwi simulator."""
    import serial  # imported here so the rest of the package works without pyserial

    with serial.serial_for_url(port, baudrate=baud, timeout=1) as ser:
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("ascii", errors="ignore")


def synthetic_lines(fs: int = 100, realtime: bool = True):
    """Yield firmware-format lines from simulated data, looping indefinitely."""
    t, accel, gyro, _ = synthetic_imu(fs=fs)
    f = AttitudeEKF(1 / fs)                 # same default filter as the firmware
    yield "# synthetic demo stream\n"
    start = time.perf_counter()
    for k in range(10**9):
        i = k % len(t)
        roll, pitch = f.update(accel[i], gyro[i])
        if realtime:
            time.sleep(max(0.0, start + k / fs - time.perf_counter()))
        # The simulator has no magnetometer, so yaw is reported as nan (the same as a board without one).
        yield (f"{int(k * 1000 / fs)},{accel[i, 0]:.4f},{accel[i, 1]:.4f},{accel[i, 2]:.4f},"
               f"{gyro[i, 0]:.3f},{gyro[i, 1]:.3f},{gyro[i, 2]:.3f},{roll:.2f},{pitch:.2f},nan\n")
