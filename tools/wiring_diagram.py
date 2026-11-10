"""Generate figures/wiring_diagram.svg (ESP32-C6 DevKit + ICM-20948). Usage: python tools/wiring_diagram.py figures/wiring_diagram.svg"""
import sys

W, H = 1000, 712
out = []
add = out.append
COL = {"vcc": "#d62828", "gnd": "#222222", "scl": "#1f6fd1", "sda": "#2a9d3c"}
FONT = "font-family='Helvetica, Arial, sans-serif'"

# Header order on the board, antenna end first (read from the board's silkscreen)
LEFT = ["GND", "3V3", "RST", "4", "5", "6", "7", "0", "1", "8", "10", "11", "12", "13", "5V"]
RIGHT = ["GND", "2", "3", "TX", "RX", "15", "23", "22", "21", "20", "19", "18", "9", "GND", "5V"]
BX, BY, BW = 350, 110, 300
P0, PITCH = 150, 24
BH = P0 + PITCH * 14 + 34 - BY
LPX, RPX = BX + 12, BX + BW - 12

USED_L = {1: "vcc"}
USED_R = {0: "gnd", 7: "scl", 8: "sda"}
AVOID = {"4", "5", "8", "9", "15", "12", "13", "TX", "RX"}


def py(i):
    return P0 + PITCH * i


def text(x, y, s, size=13, anchor="start", weight="normal", fill="#222"):
    add(f"<text x='{x}' y='{y}' {FONT} font-size='{size}' text-anchor='{anchor}' font-weight='{weight}' fill='{fill}'>{s}</text>")


def wire(points, color, hops=()):
    d = f"M{points[0][0]},{points[0][1]}"
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if y0 == y1:
            hs = sorted([h for h in hops if h[1] == y0 and min(x0, x1) < h[0] < max(x0, x1)],
                        key=lambda h: h[0], reverse=x1 < x0)
            for hx, _ in hs:
                r = 6
                d += f" H{hx - r} a{r},{r} 0 0 1 {2 * r},0" if x1 > x0 else f" H{hx + r} a{r},{r} 0 0 0 {-2 * r},0"
            d += f" H{x1}"
        else:
            d += f" V{y1}"
    add(f"<path d='{d}' fill='none' stroke='{color}' stroke-width='3' stroke-linejoin='round' stroke-linecap='round'/>")


def dot(x, y, color):
    add(f"<circle cx='{x}' cy='{y}' r='4.5' fill='{color}'/>")


add(f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}'>")
add("<defs><marker id='arrow' markerWidth='8' markerHeight='8' refX='6' refY='4' orient='auto'>"
    "<path d='M0,0 L8,4 L0,8 z' fill='#ffffff'/></marker></defs>")
add(f"<rect width='{W}' height='{H}' fill='#ffffff'/>")
text(W / 2, 32, "ESP32 IMU Logger: wiring (ESP32-C6 DevKit)", 22, "middle", "bold")
text(W / 2, 54, "One ICM-20948 9-axis IMU on the I2C bus: 3V3, GND, SDA GPIO 21, SCL GPIO 22", 14, "middle", fill="#555")

# ---- board
add(f"<rect x='{BX + 100}' y='{BY - 20}' width='100' height='26' rx='3' fill='#c9cfd6' stroke='#8a939c'/>")
text(BX + 150, BY - 3, "antenna", 11, "middle", fill="#333")
add(f"<rect x='{BX}' y='{BY}' width='{BW}' height='{BH}' rx='10' fill='#1b1b1f'/>")
add(f"<rect x='{BX + 95}' y='{BY + 16}' width='110' height='150' rx='4' fill='#d9dde2' stroke='#9aa1a8'/>")
text(BX + 150, BY + 82, "ESP32-C6", 16, "middle", "bold", "#1b1b1f")
text(BX + 150, BY + 101, "module", 12, "middle", fill="#1b1b1f")
text(BX + 150, BY + 120, "1 RISC-V core", 11, "middle", fill="#1b1b1f")
add(f"<circle cx='{BX + 150}' cy='{BY + 186}' r='6' fill='#f2f2f2' stroke='#999'/>")
text(BX + 150, BY + 206, "RGB LED (GPIO 8)", 10, "middle", fill="#b9bec4")
for bx, label in ((BX + 105, "BOOT"), (BX + 195, "RST")):
    add(f"<rect x='{bx - 11}' y='{BY + BH - 112}' width='22' height='22' rx='3' fill='#c7ccd1'/>")
    text(bx, BY + BH - 118, label, 10, "middle", fill="#b9bec4")
for ux, label, sub in ((BX + 95, "CH343", "use this port"), (BX + 205, "ESP32C6", "native USB")):
    add(f"<rect x='{ux - 28}' y='{BY + BH - 14}' width='56' height='26' rx='9' fill='#b8bec4' stroke='#7a828a'/>")
    text(ux, BY + BH - 22, label, 10, "middle", "bold", "#e8eaed")
    text(ux, BY + BH + 30, sub, 11, "middle", "bold" if label == "CH343" else "normal", "#d62828" if label == "CH343" else "#555")
text(BX + 150, BY + BH + 48, "USB-C to computer: power and serial at 115200 baud", 12, "middle", fill="#333")

for side, names, px, used in (("L", LEFT, LPX, USED_L), ("R", RIGHT, RPX, USED_R)):
    for i, n in enumerate(names):
        u = i in used
        fill = "#f2c94c" if u else ("#7a4b4b" if n in AVOID else "#6b7785")
        add(f"<circle cx='{px}' cy='{py(i)}' r='{5.5 if u else 4}' fill='{fill}'/>")
        label = ("GPIO " + n) if n.isdigit() else n
        color = "#ffffff" if u else ("#d98c8c" if n in AVOID else "#9aa6b2")
        if side == "L":
            text(px + 13, py(i) + 4, label, 12 if u else 11, weight="bold" if u else "normal", fill=color)
        else:
            text(px - 13, py(i) + 4, label, 12 if u else 11, "end", weight="bold" if u else "normal", fill=color)


def module(x, y, w, h, title, sub, pins, side, axes_xy):
    add(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='8' fill='#123f8c'/>")
    text(x + w / 2, y + 24, title, 15, "middle", "bold", "#ffffff")
    text(x + w / 2, y + 41, sub, 11, "middle", fill="#c8d6f0")
    coords = {}
    for i, (name, used) in enumerate(pins):
        yy = y + 64 + i * 19
        px = x if side == "left" else x + w
        add(f"<circle cx='{px}' cy='{yy}' r='{5 if used else 3.5}' fill='{'#f2c94c' if used else '#7f93b8'}'/>")
        if side == "left":
            text(px + 10, yy + 4, name, 12, weight="bold" if used else "normal", fill="#ffffff" if used else "#a9b8d4")
        else:
            text(px - 10, yy + 4, name, 12, "end", weight="bold" if used else "normal", fill="#ffffff" if used else "#a9b8d4")
        coords[name] = (px, yy)
    ax, ay = axes_xy
    add(f"<line x1='{ax}' y1='{ay}' x2='{ax}' y2='{ay - 34}' stroke='#ffffff' stroke-width='2.5' marker-end='url(#arrow)'/>")
    add(f"<line x1='{ax}' y1='{ay}' x2='{ax - 34}' y2='{ay}' stroke='#ffffff' stroke-width='2.5' marker-end='url(#arrow)'/>")
    text(ax, ay - 40, "X", 12, "middle", "bold", "#ffffff")
    text(ax - 42, ay + 4, "Y", 12, "end", "bold", "#ffffff")
    add(f"<circle cx='{ax}' cy='{ay}' r='6' fill='none' stroke='#ffffff' stroke-width='2'/>")
    add(f"<circle cx='{ax}' cy='{ay}' r='2' fill='#ffffff'/>")
    text(ax + 12, ay + 18, "Z up", 11, "start", "bold", "#ffffff")
    return coords


icm = module(770, 136, 190, 214, "ICM-20948", "9-axis IMU board, I2C 0x69",
             [("VCC", True), ("GND", True), ("SCL", True), ("SDA", True), ("AD0", False), ("INT", False),
              ("FSYNC", False)], "left", (900, 322))
text(865, 372, "accel + gyro + AK09916 magnetometer", 11, "middle", fill="#555")
text(865, 388, "(magnetometer at I2C 0x0C on the same bus)", 11, "middle", fill="#555")

V3, GR, G22, G21 = (LPX, py(1)), (RPX, py(0)), (RPX, py(7)), (RPX, py(8))
CX, DX, BXg, AX = 676, 696, 716, 736
mv, mg, mc, md = icm["VCC"], icm["GND"], icm["SCL"], icm["SDA"]
TOP = 78
LX = 320
wire([mv, (AX, mv[1]), (AX, TOP), (LX, TOP), (LX, V3[1]), V3], COL["vcc"])
wire([mg, (BXg, mg[1]), (BXg, GR[1]), GR], COL["gnd"])
wire([mc, (CX, mc[1]), (CX, G22[1]), G22], COL["scl"])
wire([md, (DX, md[1]), (DX, G21[1]), G21], COL["sda"])

text(865, 414, "I2C: Wire, 400 kHz", 12, "middle", "bold", "#333")
text(865, 430, "SDA GPIO 21, SCL GPIO 22", 11, "middle", fill="#555")

# ---- legend
LXl, LYl = 40, 200
items = [("vcc", "3V3 power"), ("gnd", "Ground"), ("scl", "SCL (clock)"), ("sda", "SDA (data)")]
for i, (k, label) in enumerate(items):
    yy = LYl + i * 20
    add(f"<line x1='{LXl}' y1='{yy}' x2='{LXl + 28}' y2='{yy}' stroke='{COL[k]}' stroke-width='3'/>")
    text(LXl + 36, yy + 4, label, 12, fill="#333")
add(f"<circle cx='{LXl + 14}' cy='{LYl + 84}' r='4' fill='#7a4b4b'/>")
text(LXl + 36, LYl + 88, "do not use (boot, USB, UART)", 12, fill="#333")

# ---- notes
NY = 598
add(f"<rect x='24' y='{NY}' width='952' height='100' rx='6' fill='#f4f6f8' stroke='#d5dbe1'/>")
notes = [
    "Power the ICM-20948 board from 3V3. Pin order differs between boards: match the printed labels, or use its Qwiic / STEMMA QT connector.",
    "Plug the USB-C port marked CH343: the firmware prints on it. Avoid GPIO 4, 5, 8, 9, 15 (boot straps, LED), 12, 13 (native USB), TX, RX.",
    "Leave AD0, INT and FSYNC unconnected (the firmware finds the IMU at 0x69 or 0x68). Most breakout boards include I2C pull-ups.",
    "Mount the IMU board flat, in the same plane as the ESP32-C6. X (forward), Y (left), Z (up) as printed; yaw is 0 when X points at magnetic north.",
]
for i, n in enumerate(notes):
    text(38, NY + 22 + i * 21, n, 12, fill="#333")

add("</svg>")
open(sys.argv[1], "w").write("\n".join(out) + "\n")
print("wrote", sys.argv[1])
