"""Offline analysis.

    python -m host.analyze logs/imu_20260101_120000.csv   # plot a recorded session
    python -m host.analyze --simulate                     # regenerate figures/filter_comparison.png
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .imu_math import accel_pitch, run_filter, synthetic_imu


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def filter_comparison(out="figures/filter_comparison.png"):
    """Gyro-only, accelerometer-only, and complementary filter vs. known ground truth."""
    t, accel, gyro, truth = synthetic_imu()
    dt = t[1] - t[0]
    gyro_only = truth[0] + np.cumsum(gyro[:, 1]) * dt           # pure integration
    accel_only = accel_pitch(accel)
    _, fused = run_filter(accel, gyro, dt, tau=0.5)

    errors = {"gyro only": rms(gyro_only - truth), "accel only": rms(accel_only - truth),
              "complementary": rms(fused - truth)}

    fig, (ax, ax_zoom) = plt.subplots(2, 1, figsize=(11, 7), height_ratios=[1.3, 1])
    for data, name, style in [(accel_only, "accel only", dict(color="0.7", lw=0.6)),
                              (gyro_only, "gyro only", dict(color="C3", lw=1.2)),
                              (fused, "complementary", dict(color="C0", lw=1.6))]:
        label = f"{name}  (RMS error {errors[name]:.1f} deg)"
        ax.plot(t, data, label=label, **style)
        ax_zoom.plot(t, data, **style)
    for a in (ax, ax_zoom):
        a.plot(t, truth, "k--", lw=1, label="true angle" if a is ax else None)
        a.set_ylabel("pitch (deg)")
        a.grid(alpha=0.3)
    ax.set_title("Pitch Estimate vs. Ground Truth")
    ax.legend(loc="lower left", fontsize=9)
    ax_zoom.set_xlim(30, 34)
    ax_zoom.set_ylim(-35, 35)
    ax_zoom.set_title("Detail: 1 Hz Swing, 30 to 34 s")
    ax_zoom.set_xlabel("time (s)")
    fig.tight_layout()
    Path(out).parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=150)
    return errors


def plot_log(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = (data["t_ms"] - data["t_ms"][0]) / 1000
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    for ax, cols, label in [(axes[0], ["ax_g", "ay_g", "az_g"], "accel (g)"),
                            (axes[1], ["gx_dps", "gy_dps", "gz_dps"], "gyro (deg/s)"),
                            (axes[2], ["roll_deg", "pitch_deg"], "tilt (deg)")]:
        for c in cols:
            ax.plot(t, data[c], lw=1, label=c)
        ax.set_ylabel(label)
        ax.legend(loc="upper left", ncol=3, fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    out = Path(path).with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"Saved {out}  ({len(t)} samples, {t[-1]:.1f} s, mean rate {len(t) / t[-1]:.1f} Hz)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", nargs="?", help="CSV log from live_plot")
    parser.add_argument("--simulate", action="store_true", help="regenerate figures/filter_comparison.png")
    args = parser.parse_args()
    if args.simulate:
        for name, err in filter_comparison().items():
            print(f"{name:>14}: RMS error {err:.2f} deg")
    elif args.log:
        plot_log(args.log)
    else:
        parser.print_help()
