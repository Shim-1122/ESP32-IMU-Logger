"""Portal: ESP32 tilt data -> Blender, with a live status page.

Reads the board over USB serial (or simulated data with --demo), sends each sample to
Blender as a JSON UDP packet, and serves a status page at http://127.0.0.1:8080.

    python -m host.portal --port auto             # real board
    python -m host.portal --demo                  # simulated tilt, no hardware
    python -m host.portal --reset                 # send one packet that re-centres the aircraft

Packet format (UTF-8 JSON):  {"roll": 1.23, "pitch": -4.5, "yaw": 87.0, "t_ms": 12345}
                             {"reset": true}
"yaw" is left out until the magnetometer has produced a heading.
"""
import argparse
import json
import math
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .stream import COLUMNS, list_serial_ports, parse_line, pick_serial_port, serial_lines, synthetic_lines

ROLL_IDX = COLUMNS.index("roll_deg")
PITCH_IDX = COLUMNS.index("pitch_deg")
YAW_IDX = COLUMNS.index("yaw_deg")
TIME_IDX = COLUMNS.index("t_ms")

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>IMU portal</title>
<style>
 body { font-family: system-ui, sans-serif; background: #111; color: #eee; margin: 2rem; }
 .v { font-size: 2.5rem; font-variant-numeric: tabular-nums; }
 .label { color: #888; font-size: .9rem; }
 .row { display: flex; gap: 3rem; }
 .bar { height: 14px; background: #333; width: 320px; position: relative; margin-top: .4rem; }
 .mark { position: absolute; top: 0; bottom: 0; width: 4px; background: #4fc3f7; left: 158px; }
</style></head>
<body>
<div class="row">
 <div><div class="label">roll (deg)</div><div class="v" id="roll">--</div>
      <div class="bar"><div class="mark" id="rollbar"></div></div></div>
 <div><div class="label">pitch (deg)</div><div class="v" id="pitch">--</div>
      <div class="bar"><div class="mark" id="pitchbar"></div></div></div>
 <div><div class="label">heading (deg)</div><div class="v" id="yaw">--</div></div>
</div>
<p class="label" id="status">waiting for data</p>
<script>
async function tick() {
  try {
    const s = await (await fetch('/state')).json();
    if (s.t_ms !== null) {
      document.getElementById('roll').textContent = s.roll.toFixed(1);
      document.getElementById('pitch').textContent = s.pitch.toFixed(1);
      const clamp = v => Math.max(-90, Math.min(90, v));
      document.getElementById('rollbar').style.left = (158 + clamp(s.roll) / 90 * 158) + 'px';
      document.getElementById('pitchbar').style.left = (158 + clamp(s.pitch) / 90 * 158) + 'px';
      document.getElementById('yaw').textContent = s.yaw === null ? '--' : s.yaw.toFixed(1);
      document.getElementById('status').textContent = 'last sample at ' + s.t_ms + ' ms, packets sent: ' + s.sent;
    }
  } catch (e) { document.getElementById('status').textContent = 'portal not reachable'; }
  setTimeout(tick, 100);
}
tick();
</script>
</body></html>
"""


class SharedState:
    """Latest sample, shared between the serial loop and the web handler."""

    def __init__(self):
        self.lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = None
        self.t_ms = None
        self.sent = 0

    def update(self, roll, pitch, yaw, t_ms):
        with self.lock:
            self.roll, self.pitch, self.yaw, self.t_ms = roll, pitch, yaw, t_ms
            self.sent += 1

    def snapshot(self):
        with self.lock:
            return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw, "t_ms": self.t_ms, "sent": self.sent}


def make_packet(roll: float, pitch: float, t_ms: float, yaw: float = math.nan) -> bytes:
    msg = {"roll": round(roll, 2), "pitch": round(pitch, 2), "t_ms": int(t_ms)}
    if math.isfinite(yaw):
        msg["yaw"] = round(yaw, 2)
    return json.dumps(msg).encode("utf-8")


def make_handler(state: SharedState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/state":
                body = json.dumps(state.snapshot()).encode("utf-8")
                self._send(200, "application/json", body)
            elif self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
            else:
                self._send(404, "text/plain", b"not found")

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):  # keep the terminal quiet
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description="ESP32 -> Blender portal with a live status page")
    ap.add_argument("--port", default="auto", help="serial port, 'auto', or rfc2217://localhost:4000")
    ap.add_argument("--demo", action="store_true", help="use simulated tilt data instead of a board")
    ap.add_argument("--reset", action="store_true", help="send one reset packet to Blender and exit")
    ap.add_argument("--blender-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=5005, help="must match UDP_PORT in aircraft_control.py")
    ap.add_argument("--web-port", type=int, default=8080, help="status page port on 127.0.0.1")
    args = ap.parse_args()

    dest = (args.blender_host, args.udp_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if args.reset:
        sock.sendto(json.dumps({"reset": True}).encode("utf-8"), dest)
        print("Reset sent.")
        return

    if args.demo:
        lines = synthetic_lines()
        source = "simulated data"
    else:
        port = args.port
        if port == "auto":
            port = pick_serial_port(list_serial_ports())
            if port is None:
                raise SystemExit("Could not tell which port is the ESP32. Run with --port <device>.")
        lines = serial_lines(port)
        source = port

    state = SharedState()
    server = ThreadingHTTPServer(("127.0.0.1", args.web_port), make_handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Reading {source}. Status page: http://127.0.0.1:{args.web_port}  Blender: {dest[0]}:{dest[1]}")
    print("Ctrl+C to stop.")

    try:
        for line in lines:
            row = parse_line(line)
            if row is None:  # comment, blank, or corrupted line
                continue
            roll, pitch, yaw = float(row[ROLL_IDX]), float(row[PITCH_IDX]), float(row[YAW_IDX])
            t_ms = float(row[TIME_IDX])
            sock.sendto(make_packet(roll, pitch, t_ms, yaw), dest)
            state.update(roll, pitch, yaw if math.isfinite(yaw) else None, t_ms)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.shutdown()
        sock.close()


if __name__ == "__main__":
    main()
