import itertools

import numpy as np
import pytest

from host.analyze import filter_comparison
from host.imu_math import ComplementaryFilter, accel_pitch, accel_roll, run_filter, synthetic_imu
from host.stream import COLUMNS, parse_line, synthetic_lines


# -------------------------------------------------------------------- parsing
def test_parse_valid_line():
    s = parse_line("1234,0.01,0.02,0.99,0.1,-0.2,0.3,1.5,-2.5\r\n")
    assert s.shape == (len(COLUMNS),)
    assert s[0] == 1234 and s[-1] == -2.5


@pytest.mark.parametrize("bad", ["", "# gyro bias", "1,2,3", "1,2,x,4,5,6,7,8,9", "1,2,3,4,5,6,7,8,9,10"])
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
