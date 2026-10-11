"""Live plot and CSV logging of the ESP32 IMU stream.

    python -m host.live_plot --demo                            # synthetic data, no board
    python -m host.live_plot --port auto                       # real board, port found automatically
    python -m host.live_plot --port COM3                       # real board on Windows
    python -m host.live_plot --port /dev/cu.usbserial-0001     # real board on macOS (/dev/ttyUSB0 on Linux)
    python -m host.live_plot --port rfc2217://localhost:4000   # Wokwi simulator (VS Code)
    python -m host.live_plot --list-ports                      # show the serial ports on this computer
"""
import argparse
import sys
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from .stream import COLUMNS, list_serial_ports, parse_line, pick_serial_port, serial_lines, synthetic_lines

WINDOW_S, FS = 10, 100
buffer = deque(maxlen=WINDOW_S * FS)   # newest samples for the plot (appends are thread-safe)


def reader(lines, log_path: Path):
    """Background thread: parse every line, log it to CSV, push it to the plot buffer."""
    with open(log_path, "w", encoding="utf-8", newline="\n") as log:
        log.write(",".join(COLUMNS) + "\n")
        for line in lines:
            sample = parse_line(line)
            if sample is None:
                if line.startswith("#"):
                    print(line.strip())             # firmware status messages
                continue
            log.write(line.strip() + "\n")
            buffer.append(sample)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial port, 'auto', or rfc2217:// URL")
    source.add_argument("--demo", action="store_true", help="synthetic data, no hardware")
    source.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    if args.list_ports or args.port == "auto":
        ports = list_serial_ports()
        if args.list_ports:
            for dev, desc in ports:
                print(f"{dev}    {desc}")
            print(f"{len(ports)} port(s). Most likely ESP32: {pick_serial_port(ports) or 'none found'}")
            return
        args.port = pick_serial_port(ports)
        if args.port is None:
            print("Could not tell which port is the ESP32. Ports found:")
            for dev, desc in ports:
                print(f"  {dev}    {desc}")
            print("Plug the board in with a USB data cable (some cables only charge), install the USB driver "
                  "(CP210x or CH340, see README), then run again or give the port, e.g. --port COM3.")
            sys.exit(1)
        print(f"Using serial port {args.port}")
    if args.port:
        import serial
        try:
            serial.serial_for_url(args.port, baudrate=args.baud, timeout=1).close()
        except serial.SerialException as exc:
            print(f"Could not open {args.port}: {exc}")
            print("If another program has the port open (Arduino IDE or PlatformIO serial monitor), close it. "
                  "On Linux, add yourself to the dialout group: sudo usermod -a -G dialout $USER, then log out "
                  "and back in.")
            sys.exit(1)

    Path("logs").mkdir(exist_ok=True)
    log_path = Path("logs") / f"imu_{datetime.now():%Y%m%d_%H%M%S}.csv"
    lines = synthetic_lines(FS) if args.demo else serial_lines(args.port, args.baud)
    threading.Thread(target=reader, args=(lines, log_path), daemon=True).start()
    print(f"Logging to {log_path}. Close the window to stop.")

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    panels = [("accel (g)", [1, 2, 3], ["x", "y", "z"]),
              ("gyro (deg/s)", [4, 5, 6], ["x", "y", "z"]),
              ("fused tilt (deg)", [7, 8], ["roll", "pitch"])]
    lines_by_panel = []
    for ax, (label, cols, names) in zip(axes, panels):
        lines_by_panel.append([ax.plot([], [], label=n, lw=1.2)[0] for n in names])
        ax.set_ylabel(label)
        ax.legend(loc="upper left", ncol=3, fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()

    def update(_frame):
        if len(buffer) < 2:
            return []
        data = np.array(buffer)                     # snapshot of the newest WINDOW_S seconds
        t = data[:, 0] / 1000
        for ax, (_, cols, _), plot_lines in zip(axes, panels, lines_by_panel):
            for line, col in zip(plot_lines, cols):
                line.set_data(t, data[:, col])
            ax.set_xlim(t[0], t[0] + WINDOW_S)
            lo, hi = data[:, cols].min(), data[:, cols].max()
            pad = 0.1 * (hi - lo) + 1e-3
            ax.set_ylim(lo - pad, hi + pad)
        return [l for group in lines_by_panel for l in group]

    _anim = FuncAnimation(fig, update, interval=33, cache_frame_data=False)   # ~30 fps redraw
    plt.show()


if __name__ == "__main__":
    main()
