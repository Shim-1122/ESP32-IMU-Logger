"""Blender: fly a procedural jet with the board.

Run in Blender's Text Editor, then: python -m host.portal --port auto (or --demo)

Builds the jet if OBJECT_NAME is missing. Each frame, newest UDP packet ->
  attitude: roll/pitch from tilt, yaw (if sent) about world Z
  position: tilt past deadband slides (roll) / climbs (pitch); level = hold
Packets: {"roll", "pitch", "yaw"?, "t_ms"} or {"reset": true}. Re-run replaces the listener.
"""
import json
import math
import socket
import time

import bpy
from mathutils import Matrix

OBJECT_NAME = "Aircraft"
UDP_PORT = 5005            # = portal --udp-port
ROLL_SIGN = 1.0            # -1 flips bank
PITCH_SIGN = 1.0           # -1 flips nose up/down
YAW_SIGN = 1.0             # -1 flips turn direction
SMOOTHING = 0.3            # 0..1, higher = snappier
DEADBAND_DEG = 3.0         # tilt below this: no motion
SPEED_PER_DEG = 0.6        # m/s per deg past deadband
START_POS = (0.0, 0.0, 10.0)
POS_LIMITS_X = (-40.0, 40.0)
POS_LIMITS_Z = (1.0, 60.0)
TIMER_INTERVAL = 1.0 / 60  # s


# ----------------------------------------------------------------- pure helpers
def stick(angle_deg: float) -> float:
    """Tilt -> speed: 0 in deadband, linear past it."""
    if abs(angle_deg) <= DEADBAND_DEG:
        return 0.0
    return math.copysign(abs(angle_deg) - DEADBAND_DEG, angle_deg)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap180(deg: float) -> float:
    """Wrap to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def parse_packet(data: bytes):
    """-> {'kind': 'tilt', roll, pitch, yaw|None}, {'kind': 'reset'}, or None if malformed."""
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(msg, dict):
        return None
    if msg.get("reset"):
        return {"kind": "reset"}
    try:
        roll = float(msg["roll"])
        pitch = float(msg["pitch"])
    except (KeyError, TypeError, ValueError):
        return None
    yaw = None
    if "yaw" in msg:
        try:
            yaw = float(msg["yaw"])
        except (TypeError, ValueError):
            yaw = None
    return {"kind": "tilt", "roll": roll, "pitch": pitch, "yaw": yaw}


def attitude_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Matrix:
    """R = Rz(yaw) Rx(pitch) Ry(roll): pitch about X (wings), roll about Y (nose), yaw about world Z."""
    rz = Matrix.Rotation(math.radians(YAW_SIGN * yaw_deg), 4, "Z")
    rx = Matrix.Rotation(math.radians(PITCH_SIGN * pitch_deg), 4, "X")
    ry = Matrix.Rotation(math.radians(ROLL_SIGN * roll_deg), 4, "Y")
    return rz @ rx @ ry


# ------------------------------------------------------------------- the jet
def _material(name, rgb):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    mat.diffuse_color = (*rgb, 1.0)
    return mat


def _part(kind, name, loc, scale, rot, parent, mat):
    if kind == "cube":
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc)
    elif kind == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=1.0, location=loc)
    elif kind == "cone":
        bpy.ops.mesh.primitive_cone_add(radius1=1.0, radius2=0.0, depth=1.0, location=loc)
    else:  # sphere
        bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0, location=loc)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = scale
    obj.rotation_euler = rot
    # bake scale/rotation into the mesh
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    obj.parent = parent
    obj.data.materials.append(mat)
    return obj


def build_jet(name: str = OBJECT_NAME):
    """Jet from primitives (nose +Y, wings X, up +Z) under one empty; returns the empty."""
    root = bpy.data.objects.new(name, None)
    root.empty_display_type = "ARROWS"
    root.empty_display_size = 3.0
    bpy.context.collection.objects.link(root)
    root.location = START_POS
    root.rotation_mode = "QUATERNION"

    body = _material("JetBody", (0.82, 0.84, 0.87))
    wing = _material("JetWing", (0.40, 0.42, 0.46))
    canopy = _material("JetCanopy", (0.15, 0.35, 0.70))
    nozzle = _material("JetNozzle", (0.08, 0.08, 0.08))
    pi2 = math.pi / 2

    _part("cylinder", "Fuselage", (0, -0.5, 0), (0.6, 0.6, 7.0), (pi2, 0, 0), root, body)
    _part("cone", "Nose", (0, 3.8, 0), (0.6, 0.6, 1.6), (-pi2, 0, 0), root, body)
    _part("cube", "Wings", (0, -0.3, 0), (9.0, 1.1, 0.12), (0, 0, 0), root, wing)
    _part("cube", "Tailplane", (0, -3.4, 0.1), (3.6, 0.9, 0.08), (0, 0, 0), root, wing)
    _part("cube", "Fin", (0, -3.4, 0.9), (0.08, 1.1, 1.6), (0, 0, 0), root, wing)
    _part("cylinder", "Nozzle", (0, -4.1, 0), (0.35, 0.35, 0.8), (pi2, 0, 0), root, nozzle)
    _part("sphere", "Canopy", (0, 1.6, 0.5), (0.45, 0.9, 0.35), (0, 0, 0), root, canopy)
    return root


def get_aircraft():
    obj = bpy.data.objects.get(OBJECT_NAME)
    if obj is None:
        obj = build_jet(OBJECT_NAME)
    return obj


# ------------------------------------------------------------------ runtime
def _stop_previous_listener():
    old = globals().get("_aircraft_state")
    if old is None:
        return
    if bpy.app.timers.is_registered(old["poll"]):
        bpy.app.timers.unregister(old["poll"])
    old["sock"].close()


_stop_previous_listener()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("127.0.0.1", UDP_PORT))
sock.setblocking(False)

target = {"roll": 0.0, "pitch": 0.0, "yaw": None}     # newest packet, deg
current = {"roll": 0.0, "pitch": 0.0, "yaw": None}    # smoothed, applied
last_time = time.monotonic()


def drain_packets():
    """Drain packets; keep newest tilt, return True on reset."""
    reset_requested = False
    while True:
        try:
            data, _ = sock.recvfrom(1024)
        except (BlockingIOError, OSError):
            break
        parsed = parse_packet(data)
        if parsed is None:
            continue
        if parsed["kind"] == "reset":
            reset_requested = True
            continue
        target["roll"], target["pitch"] = parsed["roll"], parsed["pitch"]
        if parsed["yaw"] is not None:
            target["yaw"] = parsed["yaw"]
    return reset_requested


def poll():
    global last_time
    now = time.monotonic()
    dt = min(now - last_time, 0.1)  # cap: no jump after a stall
    last_time = now

    reset_requested = drain_packets()
    obj = get_aircraft()

    if reset_requested:
        obj.location = START_POS
        for key in ("roll", "pitch"):
            current[key] = target[key] = 0.0
        current["yaw"] = None
        target["yaw"] = None

    for key in ("roll", "pitch"):
        current[key] += (target[key] - current[key]) * SMOOTHING
    if target["yaw"] is not None:
        if current["yaw"] is None:
            current["yaw"] = target["yaw"]
        else:  # short way round (179 -> -179)
            current["yaw"] = wrap180(current["yaw"] + wrap180(target["yaw"] - current["yaw"]) * SMOOTHING)
    yaw = current["yaw"] if current["yaw"] is not None else 0.0  # no heading yet: +Y

    obj.rotation_quaternion = attitude_matrix(current["roll"], current["pitch"], yaw).to_quaternion()

    # roll -> slide (world X), pitch -> climb (world Z)
    x, z = obj.location.x, obj.location.z
    x += ROLL_SIGN * stick(current["roll"]) * SPEED_PER_DEG * dt
    z += PITCH_SIGN * stick(current["pitch"]) * SPEED_PER_DEG * dt
    obj.location.x = clamp(x, *POS_LIMITS_X)
    obj.location.z = clamp(z, *POS_LIMITS_Z)
    return TIMER_INTERVAL


get_aircraft()
bpy.app.timers.register(poll, first_interval=0.0)
globals()["_aircraft_state"] = {"sock": sock, "poll": poll}
print(f"Portal listener on UDP {UDP_PORT}; flying '{OBJECT_NAME}'.")
