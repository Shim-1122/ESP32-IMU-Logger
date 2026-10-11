# ESP32 IMU Logger

![Tests](https://github.com/Shim-1122/ESP32-IMU-Logger/actions/workflows/tests.yml/badge.svg)

ESP32 firmware that reads an MPU6050 IMU at 100 Hz over I2C, estimates roll and pitch with a complementary filter, and streams CSV over serial. A Python host plots and logs the stream in real time. Runs in the Wokwi simulator or on a physical board with no code changes.

![Filter Comparison](figures/filter_comparison.png)

## Sensor Fusion Results

Synthetic data with known ground truth: sensor 30 cm from a pivot; stationary, 30° tilt and return, then a 1 Hz ±20° swing. The gyro has 0.5 °/s residual bias plus white noise. The accelerometer has white noise plus tangential acceleration during the swing.

| Estimate | RMS Error (deg) |
|---|---|
| Gyro only (integrated rate) | 11.6 |
| Accelerometer only (gravity angle) | 9.3 |
| Complementary filter | 2.5 |

```
angle = α · (angle + gyro_rate · dt) + (1 − α) · accel_angle,     α = τ / (τ + dt)
```

τ = 0.5 s and dt = 10 ms give α = 0.980. A constant gyro bias b produces a bounded steady error of τ·b, compared with unbounded drift for pure integration (verified in both test suites).

Regenerate with `python -m host.analyze --simulate`.

## Architecture

```
MPU6050 --I2C 400 kHz--> ESP32 --UART 115200, CSV--> host/live_plot.py --> plot + logs/*.csv
                         100 Hz fixed-rate loop       serial port, or Wokwi via rfc2217://localhost:4000
                         gyro bias calibration
                         complementary filter
```

## Getting Started (Mac or Windows, No Experience Needed)

You do not need to know Python. Each step says exactly what to click or type; copying and pasting is fine.

**1. Install Python (once per computer)**

- **Mac:** open Terminal (press Cmd+Space, type `Terminal`, press Return). Type `python3 --version` and press Return.
  - If it shows Python 3.9 or newer, go to step 2.
  - If a box offers to install the "command line developer tools", click **Install**, wait for it to finish, and check again.
  - Otherwise download the macOS installer from https://www.python.org/downloads/, open it, and click through.
- **Windows:** download the installer from https://www.python.org/downloads/ and open it. On the first screen tick **Add python.exe to PATH**, then click **Install Now**.

**2. Download this project**

Click the green **Code** button at the top of this page, then **Download ZIP**. Unzip it: on a Mac double-click the ZIP in Downloads; on Windows right-click it, choose **Extract All**, then **Extract**. You now have a folder named `ESP32-IMU-Logger-main`.

**3. Start it**

- **Windows:** open the `ESP32-IMU-Logger-main` folder and double-click **start.bat**.
- **Mac:** in Terminal type `cd ` (with a space after it), drag the `ESP32-IMU-Logger-main` folder from Finder into the Terminal window, and press Return. Then type `python3 start.py` and press Return.

The first time, it sets itself up: it creates a private Python environment inside the project folder and downloads the libraries it needs. This takes a few minutes and needs internet. Then a menu appears. Type a number and press Enter:

| Option | What it does | What you get |
| --- | --- | --- |
| 1 | Live plot with simulated data | No hardware needed. Shows accelerometer, gyro and fused tilt scrolling live and saves a log in `logs/`. Close the plot window to stop |
| 2 | Live plot from a real ESP32 | With the board plugged in by USB and running the firmware (see Running with the Firmware). Finds the port automatically |
| 3 | Live plot from the Wokwi simulator | With the simulator running in VS Code (see Running with the Firmware) |
| 4 | List the serial ports | Shows every serial port and which one looks like the ESP32 |
| 5 | Plot a recorded log | Plots a CSV from `logs/` (the newest by default) |
| 6 | Regenerate the filter comparison figure | Rebuilds `figures/filter_comparison.png` and opens it |
| 7 | Run the automated tests | Checks parsing, the filter math and port detection |

To use it again later, repeat step 3; setup is skipped and the menu appears in a few seconds. Type `q` and press Enter to quit.

**Troubleshooting**

| Problem | Fix |
| --- | --- |
| Mac: `command not found: python3` | Install Python (step 1). |
| Windows: `Python was not found`, or the Microsoft Store opens | Install Python from python.org (step 1) and tick **Add python.exe to PATH**. If you already installed it without that box, run the installer again, choose **Modify**, then **Next**, and tick **Add Python to environment variables**. |
| Windows: "Windows protected your PC" when opening start.bat | Click **More info**, then **Run anyway**. The file only runs `py start.py`; you can open it in Notepad to check. |
| `This project needs Python 3.9 or newer` | Install the current Python from https://www.python.org/downloads/ |
| `Could not tell which port is the ESP32` | Use a USB **data** cable (some only charge). Install the USB driver for the chip printed next to the board's USB port: CP2102 or CP2104 uses the [Silicon Labs CP210x driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers); CH340 or CH9102 uses the CH340 driver ([install guide](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all)). On a Mac, allow the driver under System Settings, Privacy and Security. Then pick option 4 to check. |
| `Could not open` the port | Another program is using it. Close the Arduino IDE or PlatformIO serial monitor and try again. |
| The plot window is blank | No data is arriving: check that the firmware is running and finished its 2 s calibration, and that the baud rate is 115200. |
| `Installing libraries failed` or setup stops with an error | Check your internet connection, delete the `.venv` folder inside the project folder, and start again. (On a Mac the folder is hidden: press Cmd+Shift+. in Finder to show it.) |
| A figure did not open by itself | Look in the project's `figures` folder. |

**For developers.** `start.py` only creates `.venv`, installs `requirements.txt` and runs the commands below, so you can also work by hand (Python 3.9 or newer):

```bash
# macOS / Linux
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

```powershell
# Windows (PowerShell or Command Prompt)
py -m venv .venv
.venv\Scripts\activate       # if PowerShell blocks this: Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
pip install -r requirements.txt
```

```bash
python -m host.live_plot --demo              # simulated data
python -m host.live_plot --port auto         # real board (or --port COM3, --port /dev/cu.usbserial-0001)
python -m host.live_plot --list-ports
python -m pytest
```

`python start.py <shortcut>` runs one menu option directly (`python start.py --list` shows the shortcuts). CI runs the tests through `start.py` on macOS, Windows and Linux with Python 3.9 and 3.13.

## Running with the Firmware

The menu options above need only Python. Running the firmware itself needs either a browser (Wokwi) or VS Code with PlatformIO.

### Wokwi (Browser)

1. Create a new ESP32 project at wokwi.com.
2. Replace `sketch.ino` with `src/main.cpp`.
3. Add a file named `imu_filter.h` containing `include/imu_filter.h`.
4. Replace `diagram.json` with the one in this repo.
5. Start the simulation. CSV lines begin after the 2 s gyro calibration. Adjust the MPU6050 sliders to change the inputs.

### Wokwi (VS Code) with Live Plot

1. Install the PlatformIO IDE and Wokwi Simulator extensions; request a free Wokwi license from the command palette.
2. `pio run`
3. Run "Wokwi: Start Simulator" and keep the simulator tab visible.
4. Menu option 3 (or `python -m host.live_plot --port rfc2217://localhost:4000`)

`wokwi.toml` exposes the simulated UART as an RFC2217 server on port 4000, which PySerial opens like a physical port.

### Hardware

| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

`pio run -t upload`, then menu option 2 (or `python -m host.live_plot --port auto`). Port names: `COM3` style on Windows, `/dev/cu.usbserial-0001` or `/dev/cu.wchusbserial...` on macOS (use the `cu.` one), `/dev/ttyUSB0` on Linux.

### Synthetic Demo

Menu option 1 (or `python -m host.live_plot --demo`) runs the same parser, plotter, and logger on simulated data.

## Firmware Notes

- **Register-level I2C (no sensor library):** checks WHO_AM_I at boot, sets ±2 g and ±250 °/s ranges, and reads all 14 data bytes in one burst so all axes share a sample instant.
- **Fixed-rate loop** scheduled on `micros()` rather than `delay()`, so the period does not depend on execution time. Signed comparison handles counter wraparound.
- **Gyro bias calibration:** mean of 200 stationary samples at startup.
- **Buffered reads:** bytes are read into an array before combining, since the evaluation order of two `Wire.read()` calls in one expression is unspecified.
- **Portable filter header:** `include/imu_filter.h` has no Arduino dependencies and is compiled both for the ESP32 and for host unit tests.

Build size: 22% of flash, 7% of RAM.

## Serial Format

Lines beginning with `#` are status messages. Each sample (100 Hz):

```
t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg
```

## Tests

CI runs three jobs (the Python tests on macOS, Windows and Linux with Python 3.9 and 3.13, through `start.py`):

- **Python (pytest):** CSV parsing and corrupt-line rejection, serial port auto-detection, accelerometer angles, τ·b bias bound, fused error below half of the best single sensor
- **Firmware math (C++ compiler, Linux and macOS):** big-endian byte assembly including negative values, filter coefficient, initialization, bias bound, rotation tracking
- **Firmware build (PlatformIO):** compiles the ESP32 firmware

## Project Structure

```
start.py                    Beginner launcher: sets up .venv, shows the menu (start.bat on Windows)
src/main.cpp                ESP32 firmware
include/imu_filter.h        filter math shared by firmware and host tests
diagram.json, wokwi.toml    Wokwi circuit and simulator config
platformio.ini              firmware build config
host/
  stream.py                 serial, Wokwi, and synthetic sources; CSV parser
  imu_math.py               Python port of the filter; synthetic IMU data
  live_plot.py              real-time plot and CSV logging
  analyze.py                plot recorded logs; regenerate the figure
tests/                      Python tests (tests/firmware/: C++ tests)
```

## License

MIT
