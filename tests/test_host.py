import itertools

import numpy as np
import pytest

from host.analyze import filter_comparison, plot_log
from host.imu_math import ComplementaryFilter, accel_pitch, accel_roll, run_filter, synthetic_imu
from host.stream import COLUMNS, parse_line, synthetic_lines


# -------------------------------------------------------------------- parsing
def test_parse_valid_line_with_yaw():
    s = parse_line("1234,0.01,0.02,0.99,0.1,-0.2,0.3,1.5,-2.5,87.25\r\n")
    assert s.shape == (len(COLUMNS),)
    assert s[0] == 1234 and s[-2] == -2.5 and s[-1] == 87.25


def test_parse_legacy_line_without_yaw_pads_nan():
    s = parse_line("1234,0.01,0.02,0.99,0.1,-0.2,0.3,1.5,-2.5\r\n")
    assert s.shape == (len(COLUMNS),)
    assert s[0] == 1234 and s[-2] == -2.5 and np.isnan(s[-1])


def test_parse_firmware_nan_yaw_before_heading_is_ready():
    s = parse_line("1234,0.01,0.02,0.99,0.1,-0.2,0.3,1.5,-2.5,nan")
    assert s is not None and np.isnan(s[-1])


@pytest.mark.parametrize("bad", ["", "# gyro bias", "1,2,3", "1,2,x,4,5,6,7,8,9", "1,2,3,4,5,6,7,8,9,10,11"])
def test_parse_rejects_comments_and_corrupt_lines(bad):
    assert parse_line(bad) is None


def test_demo_stream_is_parseable():
    lines = itertools.islice(synthetic_lines(realtime=False), 1, 201)
    samples = [parse_line(l) for l in lines]
    assert all(s is not None for s in samples)
    assert np.allclose(np.diff([s[0] for s in samples]), 10)       # 100 Hz -> 10 ms apart


# --------------------------------------------------------------------- math
def test_accel_angles():
    r = np.radians(30)
    assert accel_pitch(np.array([-np.sin(r), 0, np.cos(r)])) == pytest.approx(30)
    assert accel_roll(np.array([0, np.sin(r), np.cos(r)])) == pytest.approx(30)


def test_gyro_bias_error_is_bounded_at_tau_times_bias():
    """Pure integration drifts forever; the complementary filter settles at tau * bias."""
    tau, bias, dt = 0.5, 2.0, 0.01
    f = ComplementaryFilter(tau, dt)
    for _ in range(3000):
        f.update([0, 0, 1], [0, bias, 0])
    assert f.pitch == pytest.approx(tau * bias, rel=1e-3)


def test_fusion_beats_either_sensor_alone():
    errors = filter_comparison(out="/tmp/filter_comparison_test.png")
    best_single_sensor = min(errors["accel only"], errors["gyro only"])
    assert errors["complementary"] < 0.5 * best_single_sensor


def test_run_filter_matches_true_angle_when_still():
    t, accel, gyro, truth = synthetic_imu(duration_s=4, gyro_bias_dps=0.0)   # first 5 s are still
    _, pitch = run_filter(accel, gyro, dt=t[1] - t[0])
    assert np.abs(pitch[100:] - truth[100:]).max() < 1.0


# ---- serial port auto-detection (no hardware needed) ----
from host.stream import pick_serial_port  # noqa: E402


def test_pick_port_windows():
    ports = [("COM1", "Communications Port (COM1)"), ("COM4", "Silicon Labs CP210x USB to UART Bridge (COM4)")]
    assert pick_serial_port(ports) == "COM4"


def test_pick_port_macos_prefers_cu_and_skips_bluetooth():
    ports = [("/dev/cu.Bluetooth-Incoming-Port", "n/a"), ("/dev/tty.usbserial-0001", "CP2102 USB to UART"),
             ("/dev/cu.usbserial-0001", "CP2102 USB to UART")]
    assert pick_serial_port(ports) == "/dev/cu.usbserial-0001"


def test_pick_port_linux():
    ports = [("/dev/ttyS0", "ttyS0"), ("/dev/ttyUSB0", "CP2102 USB to UART Bridge Controller")]
    assert pick_serial_port(ports) == "/dev/ttyUSB0"


def test_pick_port_esp32c6_prefers_ch343_over_native_usb():
    # Both USB-C ports of the C6 DevKit plugged in: the firmware prints on the CH343 port.
    mac = [("/dev/cu.usbmodem1101", "USB JTAG/serial debug unit"),
           ("/dev/tty.usbmodem1101", "USB JTAG/serial debug unit"),
           ("/dev/cu.usbmodem5A7A0123451", "USB Single Serial"),
           ("/dev/tty.usbmodem5A7A0123451", "USB Single Serial")]
    assert pick_serial_port(mac) == "/dev/cu.usbmodem5A7A0123451"
    win = [("COM6", "USB JTAG/serial debug unit (COM6)"), ("COM7", "USB-SERIAL CH343 (COM7)")]
    assert pick_serial_port(win) == "COM7"
    # Only the native port plugged in: still picked, since it is the only candidate.
    assert pick_serial_port([("COM6", "USB JTAG/serial debug unit (COM6)")]) == "COM6"


def test_pick_port_none_when_nothing_or_ambiguous():
    assert pick_serial_port([]) is None
    assert pick_serial_port([("/dev/cu.Bluetooth-Incoming-Port", "")]) is None
    assert pick_serial_port([("COM3", "USB Serial Device"), ("COM5", "USB Serial Device")]) is None


# ------------------------------------------------------------------------ EKF
from host.analyze import compare_log, still_segments  # noqa: E402
from host.imu_math import AttitudeEKF, run_ekf  # noqa: E402


def test_ekf_beats_complementary_filter():
    errors = filter_comparison(out="/tmp/filter_comparison_test.png")
    assert errors["EKF"] < 0.25 * errors["complementary"]


@pytest.mark.parametrize("bias", [0.0, 0.5, 2.0])
def test_ekf_learns_gyro_bias(bias):
    t, accel, gyro, truth = synthetic_imu(gyro_bias_dps=bias, seed=3)
    f = AttitudeEKF(t[1] - t[0])
    for a, g in zip(accel, gyro):
        f.update(a, g)
    assert f.gyro_bias_dps[1] == pytest.approx(bias, abs=0.15)


def test_ekf_handles_combined_roll_and_pitch():
    roll, pitch = np.radians(40), np.radians(35)
    a = [-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)]
    r, p = run_ekf(np.tile(a, (300, 1)), np.zeros((300, 3)), dt=0.01)
    assert r[-1] == pytest.approx(40, abs=0.05) and p[-1] == pytest.approx(35, abs=0.05)


def _hold_log(tmp_path, holds, bias=0.5, fs=100, seed=0):
    """Synthetic recording in the firmware CSV format: hold each angle 4 s, 1 s ramps between."""
    rng = np.random.default_rng(seed)
    knots_t, knots_a, t0 = [], [], 0.0
    for i, h in enumerate(holds):
        knots_t += [t0, t0 + 4]
        knots_a += [h, h]
        t0 += 5
    t = np.arange(0, t0 - 1, 1 / fs)
    pitch = np.interp(t, knots_t, knots_a)
    rate = np.gradient(pitch, t)
    r = np.radians(pitch)
    accel = np.column_stack([-np.sin(r), np.zeros_like(t), np.cos(r)]) + 0.02 * rng.standard_normal((len(t), 3))
    gyro = np.column_stack([np.zeros_like(t), rate, np.zeros_like(t)]) + bias + 0.3 * rng.standard_normal((len(t), 3))
    path = tmp_path / "holds.csv"
    rows = np.column_stack([t * 1000, accel, gyro, np.zeros((len(t), 3))])  # roll, pitch, yaw columns
    np.savetxt(path, rows, delimiter=",", header=",".join(COLUMNS), comments="", fmt="%.5f")
    return path


def test_still_segments_finds_each_hold(tmp_path):
    holds = [0, 30, 45, 60]
    data = np.genfromtxt(_hold_log(tmp_path, holds), delimiter=",", names=True)
    gyro = np.column_stack([data["gx_dps"], data["gy_dps"], data["gz_dps"]])
    assert len(still_segments(data["t_ms"] / 1000, gyro)) == len(holds)


def test_compare_log_scores_both_filters_against_holds(tmp_path):
    results = compare_log(_hold_log(tmp_path, [0, 30, 45, 60]), holds=[0, 30, 45, 60])
    assert results["EKF"] < 0.5 and results["complementary"] < 1.0
    assert (tmp_path / "holds_compare.png").exists()


def _write_log(path, header, rows):
    path.write_text(header + "\n" + "\n".join(",".join(str(v) for v in r) for r in rows) + "\n")


@pytest.mark.parametrize("yaw, panels", [(lambda i: 0.5 * i, 4), (lambda i: float("nan"), 3), (None, 3)])
def test_plot_log_adds_heading_panel_only_when_yaw_is_present(tmp_path, monkeypatch, yaw, panels):
    import matplotlib.pyplot as plt
    created = []
    real_subplots = plt.subplots
    monkeypatch.setattr(plt, "subplots", lambda n, *a, **k: created.append(n) or real_subplots(n, *a, **k))
    cols = COLUMNS if yaw is not None else COLUMNS[:-1]          # None: legacy 9-column log
    rows = [[10 * i, 0, 0, 1, 0, 0, 0, 1.0, 2.0] + ([yaw(i)] if yaw is not None else []) for i in range(50)]
    log = tmp_path / "imu_test.csv"
    _write_log(log, ",".join(cols), rows)
    plot_log(log)
    assert created == [panels]
    assert log.with_suffix(".png").exists()
