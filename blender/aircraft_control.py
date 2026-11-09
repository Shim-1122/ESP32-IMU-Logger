"""Blender script: fly a procedural jet with the tilt board.

Open this file in Blender's Text Editor and press Run Script. Then start the portal:
    python -m host.portal --port auto        (or --demo to test without the board)

The script builds an open-source-friendly jet from primitives if no object called
OBJECT_NAME exists. Each frame it reads the newest UDP packet from the portal and:
  * banks and pitches the jet to match the board (roll and pitch), and
  * moves the jet: tilting past the deadband slides it sideways (roll) and up or down
    (pitch). Holding the board level keeps it still.

Packets: {"roll": deg, "pitch": deg, "t_ms": n} or {"reset": true} to re-centre the jet.
Re-running the script replaces the previous listener.
"""
import json
import math
import socket
import time

import bpy
from mathutils import Euler

OBJECT_NAME = "Aircraft"
UDP_PORT = 5005            # must match the portal's --udp-port
ROLL_SIGN = 1.0            # -1.0 flips the bank direction
PITCH_SIGN = 1.0           # -1.0 flips nose-up / nose-down
SMOOTHING = 0.3            # 0..1, higher follows the board more closely
DEADBAND_DEG = 3.0         # tilt smaller than this does not move the jet
SPEED_PER_DEG = 0.6        # metres per second for each degree past the deadband
START_POS = (0.0, 0.0, 10.0)
POS_LIMITS_X = (-40.0, 40.0)
POS_LIMITS_Z = (1.0, 60.0)
TIMER_INTERVAL = 1.0 / 60  # seconds between screen updates


# ----------------------------------------------------------------- pure helpers
def stick(angle_deg: float) -> float:
    """Tilt -> velocity command. Zero inside the deadband, linear outside it."""
    if abs(angle_deg) <= DEADBAND_DEG:
        return 0.0
    return math.copysign(abs(angle_deg) - DEADBAND_DEG, angle_deg)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_packet(data: bytes):
    """Return ('tilt', roll, pitch), ('reset', 0, 0), or None for anything malformed."""
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(msg, dict):
        return None
    if msg.get("reset"):
        return ("reset", 0.0, 0.0)
    try:
        return ("tilt", float(msg["roll"]), float(msg["pitch"]))
    except (KeyError, TypeError, ValueError):
        return None


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
    # Bake scale and rotation into the mesh so the parts carry their own shape.
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    obj.parent = parent
    obj.data.materials.append(mat)
    return obj


def build_jet(name: str = OBJECT_NAME):
    """Build a simple delta-free jet: nose along +Y, wings along X, up +Z.

    Returns the empty that parents every part, so one rotation moves the whole aircraft.
    """
    root = bpy.data.objects.new(name, None)
    root.empty_display_type = "ARROWS"
    root.empty_display_size = 3.0
    bpy.context.collection.objects.link(root)
    root.location = START_POS

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

target = {"roll": 0.0, "pitch": 0.0}    # newest tilt from the portal, degrees
current = {"roll": 0.0, "pitch": 0.0}   # smoothed tilt applied to the jet
last_time = time.monotonic()


def drain_packets():
    """Read every waiting packet; tilt values are kept, resets are applied immediately."""
    reset_requested = False
    while True:
        try:
            data, _ = sock.recvfrom(1024)
        except (BlockingIOError, OSError):
            break
        parsed = parse_packet(data)
        if parsed is None:
            continue
        kind, roll, pitch = parsed
        if kind == "reset":
            reset_requested = True
        else:
            target["roll"], target["pitch"] = roll, pitch
    return reset_requested


def poll():
    global last_time
    now = time.monotonic()
    dt = min(now - last_time, 0.1)  # cap so a stall does not teleport the jet
    last_time = now

    reset_requested = drain_packets()
    obj = get_aircraft()

    if reset_requested:
        obj.location = START_POS
        current["roll"] = current["pitch"] = 0.0
        target["roll"] = target["pitch"] = 0.0

    for key in ("roll", "pitch"):
        current[key] += (target[key] - current[key]) * SMOOTHING

    # Attitude: pitch about X (wings), roll about Y (nose axis).
    obj.rotation_mode = "XYZ"
    obj.rotation_euler = Euler(
        (math.radians(PITCH_SIGN * current["pitch"]), math.radians(ROLL_SIGN * current["roll"]), 0.0),
        "XYZ",
    )

    # Position: roll slides the jet sideways, pitch climbs or descends it.
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
