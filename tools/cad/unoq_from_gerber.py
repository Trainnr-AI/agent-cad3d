"""Exact Arduino UNO Q (ABX00162 / ABX00173) placement data from Arduino's own manufacturing files -> JSON for parts/uno_q.py.

Source: Arduino's CAD package ABX00162-cad-files.zip (docs.arduino.cc; Gerber RS-274X + Excellon, Cadence Allegro 17.4),
licensed CC BY-SA 4.0 by Arduino - the derived numbers keep that licence and credit (written into the JSON).

  uv run --with gerbonara python tools/cad/unoq_from_gerber.py <unzipped ABX00162-cad-files/ABX00162-cad-files> <out.json>

Board frame (as parts/uno_q.py): origin at the board's corner under the USB-C end, x along the 68.58 side, y along the 53.34 side,
z up from the PCB's underside. Gerber frame -> board frame: x = 68.58 - gy, y = gx (checked: the 4 mounting holes land exactly on
the datasheet's). Exact from the files: every drilled hole (Excellon repeat codes R## expanded), the header pins (32 plated 1.05 holes),
SPI2 (6 plated 1.21), the USB-C shell slots and locating pegs, the paste pads of the USB-C / Qwiic / JCTL / power button. Bodies above
the board (heights, housing outlines) come from the datasheet and the parts' own standards - marked in the JSON.
"""
import json
import re
import sys
from pathlib import Path

LEN, WID = 68.58, 53.34
D = Path(sys.argv[1])
out = Path(sys.argv[2])


def board(gx, gy):
    return (round(LEN - gy, 3), round(gx, 3))


# ---- drill (Excellon, with repeat codes)
def drills(path):
    tools, cur, x, y, holes = {}, None, None, None, []
    for line in open(path, errors="ignore"):
        line = line.strip()
        if line.startswith(";"):
            continue
        m = re.match(r"^T(\d+)C([\d.]+)$", line)
        if m:
            tools[m.group(1)] = float(m.group(2))
            continue
        m = re.match(r"^T(\d+)$", line)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^R(\d+)(?:X(-?\d+))?(?:Y(-?\d+))?", line)
        if m and cur:
            dx, dy = int(m.group(2) or 0) / 1e4, int(m.group(3) or 0) / 1e4
            for _ in range(int(m.group(1))):
                x, y = x + dx, y + dy
                holes.append((tools[cur], board(x, y)))
            continue
        mx, my = re.search(r"X(-?\d+)", line), re.search(r"Y(-?\d+)", line)
        if (mx or my) and cur:
            x = int(mx.group(1)) / 1e4 if mx else x
            y = int(my.group(1)) / 1e4 if my else y
            holes.append((tools[cur], board(x, y)))
    return holes


holes = [h for h in drills(next(D.glob("*-1-8.drl"))) if h[0] >= 0.5]
by = lambda d: sorted(p for dd, p in holes if abs(dd - d) < 1e-6)
pins = by(1.05)
assert len(pins) == 32, len(pins)

# ---- paste pads (top) via gerbonara
from gerbonara import GerberFile
from gerbonara.utils import MM


def pads(layer):
    res = []
    for o in GerberFile.open(D / f"{layer}.art").objects:
        for p in o.to_primitives(unit=MM):
            k = type(p).__name__
            if k == "Rectangle":
                xs, ys = (p.x - p.w / 2, p.x + p.w / 2), (p.y - p.h / 2, p.y + p.h / 2)
            elif k == "Circle":
                xs, ys = (p.x - p.r, p.x + p.r), (p.y - p.r, p.y + p.r)
            else:
                continue
            a, b = board(xs[0], ys[0]), board(xs[1], ys[1])
            res.append((min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1])))
    return res


P = pads("pmt")
inside = lambda r: [p for p in P if r[0] <= p[0] and p[1] <= r[1] and r[2] <= p[2] and p[3] <= r[3] and (p[1] - p[0]) * (p[3] - p[2]) > 0.2]
r3 = lambda t: [round(v, 3) for v in t]

# ---- header pins with names (UNO Q pinout, ABX00162-full-pinout.pdf): D0 at the far end, BOOT at the USB-C end
row_dig = sorted(p for p in pins if abs(p[1] - 50.8) < 0.01)
row_pwr = sorted(p for p in pins if abs(p[1] - 2.54) < 0.01)
dig_hi = [p for p in row_dig if p[0] < 43.0]                      # 10 pins: SCL .. D8 (x rising)
dig_lo = [p for p in row_dig if p[0] > 43.0]                      # 8 pins: D7 .. D0
pwr = [p for p in row_pwr if p[0] < 48.0]                         # 8 pins: BOOT .. VIN
ana = [p for p in row_pwr if p[0] > 48.0]                         # 6 pins: A0 .. A5
assert (len(dig_hi), len(dig_lo), len(pwr), len(ana)) == (10, 8, 8, 6)
names = {}
for p, n in zip(dig_hi, ["SCL", "SDA", "AREF", "GND_D", "D13", "D12", "D11", "D10", "D9", "D8"]):
    names[n] = p
for p, n in zip(dig_lo, ["D7", "D6", "D5", "D4", "D3", "D2", "D1_TX", "D0_RX"]):
    names[n] = p
for p, n in zip(pwr, ["BOOT", "IOREF", "RESET", "3V3", "5V", "GND_P1", "GND_P2", "VIN"]):
    names[n] = p
for p, n in zip(ana, ["A0", "A1", "A2", "A3", "A4", "A5"]):
    names[n] = p
housing = lambda g: r3((g[0][0] - 1.27, g[-1][0] + 1.27, g[0][1] - 1.27, g[0][1] + 1.27))

usb = inside((-1, 8.5, 32.5, 43.5))
qw = inside((62, 68.6, 10.5, 18.5))
jctl = inside((12, 25, 40, 49))
btn = inside((0, 3, 45.5, 54))
peg = by(0.5)
qsig = sorted(p for p in qw if p[1] - p[0] < 1.7)                 # the 4 signal pads (1.55 long)
qmnt = sorted(p for p in qw if p[1] - p[0] >= 1.7)                # the 2 nail pads
usb_cy = round((peg[0][1] + peg[1][1]) / 2, 3)
data = {
    "source": "Arduino UNO Q CAD files (ABX00162-cad-files.zip, docs.arduino.cc), CC BY-SA 4.0 (c) Arduino; Gerber/Excellon -> board frame",
    "board": {"length": LEN, "width": WID, "pcb": 1.6, "holes_3.2": by(3.2)},
    "pins": {n: list(p) for n, p in names.items()},
    "headers": {"digital_high": housing(dig_hi), "digital_low": housing(dig_lo), "power": housing(pwr), "analog": housing(ana),
                "height": 8.5, "height_note": "datasheet: tallest top part 8.5 above the PCB (user: ~1 cm incl. the board)"},
    "spi2_pins": by(1.21),
    "jctl_pads": [r3(p) for p in sorted(jctl)],
    "usbc": {"pegs": peg, "slots_x": [[1.42, 2.92], [6.17, 7.27]], "centre_y": usb_cy,
             "signal_pads_x": r3((min(p[0] for p in usb if p[1] - p[0] < 1.0), max(p[1] for p in usb if p[1] - p[0] < 1.0))),
             "front_x": -0.55, "front_note": "silkscreen outline of the receptacle (datasheet drawing: ~1 past the edge)",
             "width": 8.94, "height": 3.0, "height_note": "datasheet"},
    "qwiic": {"signal_pads": [r3(p) for p in qsig], "nail_pads": [r3(p) for p in qmnt],
              "pin1_y": round((qsig[0][2] + qsig[0][3]) / 2, 3), "centre_y": round(sum((p[2] + p[3]) / 2 for p in qsig) / 4, 3),
              "mouth": "+x (the board's far edge)", "body_note": "JST SH SM04B-SRSS-TB: housing 6.0 wide x 4.25 deep x 2.95 high (JST)"},
    "button_pads": [r3(p) for p in sorted(btn)],
    "other_holes": {"0.8_np": by(0.8)},
}
out.write_text(json.dumps(data, indent=1))
print(f"pins {len(names)}, SPI2 {len(data['spi2_pins'])}, JCTL pads {len(jctl)}, USB-C pads {len(usb)}, Qwiic pads {len(qw)}, button pads {len(btn)} -> {out}")
for n in ("D0_RX", "D1_TX", "GND_D", "GND_P2", "VIN"):
    print(f"  {n:7s} {names[n]}")
print("  USB-C centre y", usb_cy, "| Qwiic centre y", data["qwiic"]["centre_y"], "pin 1 y", data["qwiic"]["pin1_y"])
