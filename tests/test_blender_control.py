"""Blender controller: stick mapping, packet parsing, heading, jet construction, and flight from packets.

Runs only where the `bpy` module is installed (pip install bpy); otherwise skipped.
"""
import importlib.util
import socket
import time
from pathlib import Path

import pytest

bpy = pytest.importorskip("bpy")
from mathutils import Vector  # noqa: E402

SCRIPT = Path(__file__).resolve().parent.parent / "blender" / "aircraft_control.py"


def load_controller():
    spec = importlib.util.spec_from_file_location("aircraft_control", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # binds UDP 5005 and registers the timer
    return module


@pytest.fixture
def ctl():
    module = load_controller()
    yield module
    module._stop_previous_listener()


def send(ctl, payload: bytes):
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(payload, ("127.0.0.1", ctl.UDP_PORT))
    finally:
        sender.close()
    time.sleep(0.03)


def run_frames(ctl, n):
    for _ in range(n):
        ctl.poll()
        time.sleep(0.003)


def test_stick_has_deadband_and_keeps_sign(ctl):
    assert ctl.stick(0.0) == 0.0
    assert ctl.stick(ctl.DEADBAND_DEG) == 0.0
    assert ctl.stick(ctl.DEADBAND_DEG + 2.0) == pytest.approx(2.0)
    assert ctl.stick(-(ctl.DEADBAND_DEG + 2.0)) == pytest.approx(-2.0)


def test_wrap180_takes_the_short_way_round(ctl):
    assert ctl.wrap180(190) == pytest.approx(-170)
    assert ctl.wrap180(-190) == pytest.approx(170)
    assert ctl.wrap180(45) == pytest.approx(45)


def test_parse_packet_handles_tilt_with_and_without_yaw(ctl):
    assert ctl.parse_packet(b'{"roll": 1.5, "pitch": -2.0, "t_ms": 9}') == {
        "kind": "tilt", "roll": 1.5, "pitch": -2.0, "yaw": None}
    assert ctl.parse_packet(b'{"roll": 1.5, "pitch": -2.0, "yaw": 87.5}') == {
        "kind": "tilt", "roll": 1.5, "pitch": -2.0, "yaw": 87.5}
    assert ctl.parse_packet(b'{"reset": true}') == {"kind": "reset"}
    assert ctl.parse_packet(b"not json") is None
    assert ctl.parse_packet(b'{"roll": "x"}') is None
    assert ctl.parse_packet(b"[1, 2]") is None


def test_attitude_matrix_turns_nose_with_yaw_and_pitches_up(ctl):
    nose_yawed = ctl.attitude_matrix(0.0, 0.0, 90.0) @ Vector((0, 1, 0))
    assert nose_yawed.x == pytest.approx(-1.0, abs=1e-6)  # 90 deg counter-clockwise from +Y
    nose_up = ctl.attitude_matrix(0.0, 30.0, 0.0) @ Vector((0, 1, 0))
    assert nose_up.z == pytest.approx(0.5, abs=1e-6)       # positive pitch raises the nose


def test_build_jet_creates_parented_parts(ctl):
    root = ctl.get_aircraft()
    parts = {child.name for child in root.children}
    assert {"Fuselage", "Nose", "Wings", "Tailplane", "Fin", "Nozzle", "Canopy"} <= parts
    assert all(child.type == "MESH" for child in root.children)


def test_packets_tilt_and_move_the_jet(ctl):
    send(ctl, b'{"roll": 0.0, "pitch": 0.0, "t_ms": 0}')
    ctl.poll()
    jet = ctl.get_aircraft()
    start_x, start_z = jet.location.x, jet.location.z

    send(ctl, b'{"roll": 20.0, "pitch": 15.0, "t_ms": 1}')
    run_frames(ctl, 60)
    assert jet.location.x > start_x
    assert jet.location.z > start_z
    assert abs(jet.rotation_quaternion.to_euler().y) > 0.1  # banked
    assert abs(jet.rotation_quaternion.to_euler().x) > 0.1  # pitched

    send(ctl, b'{"reset": true}')
    ctl.poll()
    assert tuple(jet.location) == pytest.approx(ctl.START_POS)


def test_yaw_from_portal_turns_the_jet(ctl):
    send(ctl, b'{"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "t_ms": 0}')
    run_frames(ctl, 5)
    send(ctl, b'{"roll": 0.0, "pitch": 0.0, "yaw": 90.0, "t_ms": 1}')
    run_frames(ctl, 60)
    jet = ctl.get_aircraft()
    nose = jet.rotation_quaternion.to_matrix() @ Vector((0, 1, 0))
    assert nose.x == pytest.approx(-1.0, abs=0.02)
    assert nose.y == pytest.approx(0.0, abs=0.02)
