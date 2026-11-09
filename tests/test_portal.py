"""Portal: packet format, shared state, and the simulated stream it reads in --demo mode."""
import json

from host.portal import SharedState, make_packet
from host.stream import COLUMNS, parse_line, synthetic_lines


def test_packet_is_json_with_rounded_values():
    msg = json.loads(make_packet(12.3456, -4.5, 1234.9))
    assert msg == {"roll": 12.35, "pitch": -4.5, "t_ms": 1234}


def test_packet_includes_yaw_only_when_finite():
    msg = json.loads(make_packet(1.0, 2.0, 5, yaw=87.456))
    assert msg["yaw"] == 87.46
    assert "yaw" not in json.loads(make_packet(1.0, 2.0, 5, yaw=float("nan")))


def test_shared_state_counts_sent_packets():
    state = SharedState()
    assert state.snapshot()["t_ms"] is None
    state.update(1.0, 2.0, None, 100)
    state.update(3.0, 4.0, 90.0, 110)
    snap = state.snapshot()
    assert (snap["roll"], snap["pitch"], snap["yaw"], snap["t_ms"], snap["sent"]) == (3.0, 4.0, 90.0, 110, 2)


def test_demo_stream_lines_parse_into_all_columns():
    lines = synthetic_lines(realtime=False)
    assert next(lines).startswith("#")
    row = parse_line(next(lines))
    assert row is not None and len(row) == len(COLUMNS)
