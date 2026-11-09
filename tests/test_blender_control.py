"""Blender controller: stick mapping, packet parsing, jet construction, and flight from packets.

Runs only where the `bpy` module is installed (pip install bpy); otherwise skipped.
"""
import importlib.util
import socket
import time
from pathlib import Path

import pytest

bpy = pytest.importorskip("bpy")

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


def test_stick_has_deadband_and_keeps_sign(ctl):
    assert ctl.stick(0.0) == 0.0
    assert ctl.stick(ctl.DEADBAND_DEG) == 0.0
    assert ctl.stick(ctl.DEADBAND_DEG + 2.0) == pytest.approx(2.0)
    assert ctl.stick(-(ctl.DEADBAND_DEG + 2.0)) == pytest.approx(-2.0)


def test_parse_packet_accepts_tilt_and_reset_and_rejects_junk(ctl):
    assert ctl.parse_packet(b'{"roll": 1.5, "pitch": -2.0, "t_ms": 9}') == ("tilt", 1.5, -2.0)
    assert ctl.parse_packet(b'{"reset": true}') == ("reset", 0.0, 0.0)
    assert ctl.parse_packet(b"not json") is None
    assert ctl.parse_packet(b'{"roll": "x"}') is None
    assert ctl.parse_packet(b"[1, 2]") is None


def test_build_jet_creates_parented_parts(ctl):
    root = ctl.get_aircraft()
    parts = {child.name for child in root.children}
    assert {"Fuselage", "Nose", "Wings", "Tailplane", "Fin", "Nozzle", "Canopy"} <= parts
    assert all(child.type == "MESH" for child in root.children)


def test_packets_tilt_and_move_the_jet(ctl):
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(b'{"roll": 0.0, "pitch": 0.0, "t_ms": 0}', ("127.0.0.1", ctl.UDP_PORT))
        time.sleep(0.05)
        ctl.poll()
        jet = ctl.get_aircraft()
        start_x, start_z = jet.location.x, jet.location.z

        # Hold a roll and pitch well past the deadband for a few frames.
        sender.sendto(b'{"roll": 20.0, "pitch": 15.0, "t_ms": 1}', ("127.0.0.1", ctl.UDP_PORT))
        time.sleep(0.05)
        for _ in range(60):
            ctl.poll()
            time.sleep(0.005)
        assert jet.location.x > start_x
        assert jet.location.z > start_z
        assert abs(jet.rotation_euler.y) > 0.1  # banked
        assert abs(jet.rotation_euler.x) > 0.1  # pitched

        sender.sendto(b'{"reset": true}', ("127.0.0.1", ctl.UDP_PORT))
        time.sleep(0.05)
        ctl.poll()
        assert tuple(jet.location) == pytest.approx(ctl.START_POS)
    finally:
        sender.close()
