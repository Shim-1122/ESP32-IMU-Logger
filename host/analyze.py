"""Offline analysis.

    python -m host.analyze logs/imu_20260101_120000.csv   # plot a recorded session
    python -m host.analyze --simulate                     # regenerate figures/filter_comparison.png
    python -m host.analyze --compare logs/imu_....csv --holds 0,30,45,60
        # rerun the complementary filter and the EKF offline on a real recording and score
        # both against known hold angles (see "Real-Hardware Test" in the README)
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .imu_math import accel_pitch, run_ekf, run_filter, synthetic_imu


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def filter_comparison(out="figures/filter_comparison.png"):
    """Gyro-only, accelerometer-only, complementary filter, and EKF vs. known ground truth."""
    t, accel, gyro, truth = synthetic_imu()
    dt = t[1] - t[0]
    gyro_only = truth[0] + np.cumsum(gyro[:, 1]) * dt           # pure integration
    accel_only = accel_pitch(accel)
    _, fused = run_filter(accel, gyro, dt, tau=0.5)
    _, ekf = run_ekf(accel, gyro, dt)

    errors = {"gyro only": rms(gyro_only - truth), "accel only": rms(accel_only - truth),
              "complementary": rms(fused - truth), "EKF": rms(ekf - truth)}

    fig, (ax, ax_zoom) = plt.subplots(2, 1, figsize=(11, 7), height_ratios=[1.3, 1])
    for data, name, style in [(accel_only, "accel only", dict(color="0.7", lw=0.6)),
                              (gyro_only, "gyro only", dict(color="C3", lw=1.2)),
                              (fused, "complementary", dict(color="C0", lw=1.4)),
                              (ekf, "EKF", dict(color="C2", lw=1.8))]:
        label = f"{name}  (RMS error {errors[name]:.2f} deg)"
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


def still_segments(t, gyro_dps, min_s=2.0, max_rate_dps=3.0):
    """(start, end) index pairs where the sensor is held still for at least min_s seconds."""
    still = np.linalg.norm(gyro_dps, axis=1) < max_rate_dps
    segs, start = [], None
    for i, s in enumerate(np.append(still, False)):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if t[i - 1] - t[start] >= min_s:
                segs.append((start, i))
            start = None
    return segs


def compare_log(path, holds=None):
    """Rerun both filters on a recorded log. With known hold angles, score each against them.

    Hold test: hold the sensor still at each angle in `holds` (in order, e.g. on a 3D-printed
    wedge or against a protractor) for 3+ s, moving between holds. Each still segment is matched
    to the next angle; the middle half of the segment is averaged to skip settling.
    """
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = (data["t_ms"] - data["t_ms"][0]) / 1000
    dt = float(np.median(np.diff(t)))
    accel = np.column_stack([data["ax_g"], data["ay_g"], data["az_g"]])
    gyro = np.column_stack([data["gx_dps"], data["gy_dps"], data["gz_dps"]])
    _, comp = run_filter(accel, gyro, dt, tau=0.5)
    _, ekf = run_ekf(accel, gyro, dt)

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(t, accel_pitch(accel), color="0.75", lw=0.6, label="accel only")
    ax.plot(t, comp, color="C0", lw=1.4, label="complementary")
    ax.plot(t, ekf, color="C2", lw=1.6, label="EKF")
    results = {}
    if holds:
        segs = still_segments(t, gyro)
        if len(segs) < len(holds):
            print(f"Found only {len(segs)} still segments for {len(holds)} hold angles; scoring the first {len(segs)}.")
        rows = []
        for (a, b), truth in zip(segs, holds):
            q = (b - a) // 4
            sl = slice(a + q, b - q)
            rows.append((truth, comp[sl].mean() - truth, ekf[sl].mean() - truth))
            ax.hlines(truth, t[a], t[b - 1], colors="k", linestyles="--", lw=1)
        rows = np.array(rows)
        results = {"complementary": rms(rows[:, 1]), "EKF": rms(rows[:, 2])}
        print(f"{'hold (deg)':>10} {'comp err':>9} {'EKF err':>8}")
        for truth, ec, ee in rows:
            print(f"{truth:10.1f} {ec:9.2f} {ee:8.2f}")
        print(f"{'RMS':>10} {results['complementary']:9.2f} {results['EKF']:8.2f}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch (deg)")
    ax.set_title("Recorded Session: Complementary Filter vs. EKF")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    out = Path(path).with_name(Path(path).stem + "_compare.png")
    fig.savefig(out, dpi=150)
    print(f"Saved {out}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", nargs="?", help="CSV log from live_plot")
    parser.add_argument("--simulate", action="store_true", help="regenerate figures/filter_comparison.png")
    parser.add_argument("--compare", action="store_true", help="rerun complementary filter and EKF on the log")
    parser.add_argument("--holds", help="comma-separated known hold angles in deg, e.g. 0,30,45,60")
    args = parser.parse_args()
    if args.simulate:
        for name, err in filter_comparison().items():
            print(f"{name:>14}: RMS error {err:.2f} deg")
    elif args.log and args.compare:
        compare_log(args.log, [float(h) for h in args.holds.split(",")] if args.holds else None)
    elif args.log:
        plot_log(args.log)
    else:
        parser.print_help()
