"""Arduino UNO Q (ABX00162 / ABX00173), placed from Arduino's own manufacturing files (runs inside Blender), in place of the plain
'uno_q' box of body_v2.py (the box is hidden, not deleted).

2026-11-27: rebuilt from data/unoq_board.json (tools/cad/unoq_from_gerber.py: Arduino's Gerber / drill files, CC BY-SA 4.0). Was drawn
from the datasheet with the ports scaled from its picture: the Qwiic sat at the wrong end (it is at the far end, mouth to the far
edge), each header row was one box (they are two housings each), pins were up to 0.27 off, the USB-C 0.45 too far out.

Board frame (mm): origin at the board's corner under the USB-C end, x along the 68.58 side, y along the 53.34 side, z up from the
PCB's underside. EXACT (from the files): outline holes, the 32 header pins and their names, SPI2's 6 pins, JCTL's 10 pads, the
USB-C's slots / pegs / pads, the Qwiic's pads, the bottom connectors' pegs. FROM THE PARTS' STANDARDS / DATASHEET: heights (headers
8.5 above the PCB - the user measured ~1 cm incl. the board -, USB-C 3.0, Qwiic 2.95), the Qwiic housing (JST SM04B-SRSS-TB 6.0 x
4.25), the USB-C body length. APPROXIMATE: the power button's body (only its 2 pads are in the files), the low chips / LED matrix.

Anchors (custom properties, LAYOUT mm = the frame bay_parts.py / wiring.py use): unoq_headers["pins"] = {name: [x, y, z_top]} for every
socket (D0_RX, D1_TX, GND_D, VIN, GND_P1, GND_P2, A0..A5, ...), unoq_usbc["mouth"], unoq_pins["qwiic_mouth"] (+ the axis each faces).

ARGS = {"robot": "duck_scs0009", "collection": "electronics", "usb_end": "rear", "board_json": ".../data/unoq_board.json",
        "joint_lib": ..., "base_lib": ..., "dims": ...}
"""
import json
import math

import bpy
import numpy as np
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
J = json.load(open(args["board_json"]))
coll = bpy.data.collections.get(args.get("collection", "electronics")) or bpy.data.collections.new(args.get("collection", "electronics"))
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
OWN = ("unoq_pcb", "unoq_headers", "unoq_usbc", "unoq_pins", "unoq_bottom_connectors", "unoq_top_parts")
for n in OWN:                                                       # only this builder's own objects (not exploded / animation copies)
    if n in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[n], do_unlink=True)
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
box, cyl, hull, new = L["add_box"], L["add_cyl"], L["add_hull"], L["new_bm"]

LEN, WID, PCB = J["board"]["length"], J["board"]["width"], J["board"]["pcb"]
HOLES = [tuple(h) for h in J["board"]["holes_3.2"]]
CH = 3.6                                                            # far-end corner chamfers (datasheet drawing)
old = bpy.data.objects["uno_q"]
P = np.array([((ML.inverted() @ old.matrix_world @ v.co) * 1000)[:] for v in old.data.vertices])
x_rear, y0 = P[:, 0].min(), (P[:, 1].min() + P[:, 1].max()) / 2 - WID / 2
lid = bpy.data.objects.get("body_lid")
z_pcb = float(np.array([((ML.inverted() @ lid.matrix_world @ v.co) * 1000)[2] for v in lid.data.vertices]).max())   # on the lid's stand-offs
if args.get("usb_end", "rear") == "rear":
    MB = ML @ Matrix.Translation(Vector((x_rear, y0, z_pcb)) * MM)
else:
    MB = ML @ Matrix.Translation(Vector((x_rear + LEN, y0 + WID, z_pcb)) * MM) @ Matrix.Rotation(math.pi, 4, "Z")
to_layout = lambda p: [round(c, 3) for c in ((ML.inverted() @ MB @ (Vector(p) * MM)) * 1000)]
dir_layout = lambda d: [round(c, 4) for c in (ML.inverted().to_3x3() @ MB.to_3x3() @ Vector(d)).normalized()]


def part(name, bms, rgba, subs=()):
    if subs:
        o = L["build_part"](name, [L["combine_cutters"](bms) if len(bms) > 1 else bms[0]], list(subs), coll, L["material"](name, rgba))
    else:
        o = L["to_object"](name, L["combine_cutters"](bms) if len(bms) > 1 else bms[0], coll, L["material"](name, rgba))
    mw = o.matrix_world.copy()
    o.parent = old.parent
    o.matrix_world = mw
    return o


# ---- PCB
outline = [(0, 0), (LEN - CH, 0), (LEN, CH), (LEN, WID - CH), (LEN - CH, WID), (0, WID)]
part("unoq_pcb", [hull(new(), [(x, y, z) for x, y in outline for z in (0.0, PCB)], MB)], (0.0, 0.5, 0.55, 1),
     [cyl(new(), x, y, -0.5, PCB + 0.5, 3.2, 24, MB) for x, y in HOLES])

# ---- the four female headers (two per side), 8.5 above the PCB, a 1.0 square socket over every pin
HH = J["headers"]["height"]
hdr = [box(new(), x0, x1, ya, yb, PCB, PCB + HH, MB) for x0, x1, ya, yb in
       (J["headers"][k] for k in ("digital_high", "digital_low", "power", "analog"))]
sockets = [box(new(), x - 0.5, x + 0.5, y - 0.5, y + 0.5, PCB + HH - 6.0, PCB + HH + 0.5, MB) for x, y in J["pins"].values()]
h = part("unoq_headers", hdr, (0.05, 0.05, 0.05, 1), sockets)
h["pins"] = {n: to_layout((x, y, PCB + HH)) for n, (x, y) in J["pins"].items()}      # socket centre at the header's top
h["pins_board"] = {n: [x, y, PCB + HH] for n, (x, y) in J["pins"].items()}
h["pin_axis"] = dir_layout((0, 0, 1))                                                  # a plug goes in along -axis

# ---- USB-C receptacle: 8.94 wide on the pegs' centre line, front at the silkscreen outline, rear past the shell's rear slots
U = J["usbc"]
ux0, ux1, ucy, uw = U["front_x"], U["slots_x"][1][1] - 0.4, U["centre_y"], U["width"]
u = part("unoq_usbc", [box(new(), ux0, ux1, ucy - uw / 2, ucy + uw / 2, PCB, PCB + U["height"], MB)], (0.75, 0.75, 0.78, 1))
u["mouth"], u["mouth_axis"] = to_layout((ux0, ucy, PCB + U["height"] / 2)), dir_layout((-1, 0, 0))

# ---- JCTL 2 x 5 and SPI2 2 x 3 male pin headers (2.5 base + square pins to 8.5); Qwiic; power button
jp = J["jctl_pads"]
jx = sorted({round((p[0] + p[1]) / 2, 2) for p in jp})
jy = (sum((p[2] + p[3]) / 2 for p in jp) / len(jp))
jrows = (jy - 1.27, jy + 1.27)
sp = J["spi2_pins"]
sx, sy = sorted({p[0] for p in sp}), sorted({p[1] for p in sp})
pins_bms = [box(new(), jx[0] - 1.27, jx[-1] + 1.27, jy - 2.54, jy + 2.54, PCB, PCB + 2.5, MB)]
pins_bms += [box(new(), x - 0.32, x + 0.32, y - 0.32, y + 0.32, PCB + 2.4, PCB + 8.5, MB) for x in jx for y in jrows]
pins_bms += [box(new(), sx[0] - 1.27, sx[-1] + 1.27, sy[0] - 1.27, sy[-1] + 1.27, PCB, PCB + 2.5, MB)]
pins_bms += [box(new(), x - 0.32, x + 0.32, y - 0.32, y + 0.32, PCB + 2.4, PCB + 8.5, MB) for x in sx for y in sy]
Q = J["qwiic"]
qcy, qx1 = Q["centre_y"], max(p[1] for p in Q["nail_pads"])         # mouth flush with the nail pads' outer ends
qx0 = qx1 - 4.25
pins_bms += [box(new(), qx0, qx1, qcy - 3.0, qcy + 3.0, PCB, PCB + 2.95, MB)]           # JST SM04B-SRSS-TB housing
pins_bms += [box(new(), p[0], p[1], p[2], p[3], PCB, PCB + 0.8, MB) for p in Q["nail_pads"]]   # its two nails
bp = J["button_pads"]
bx0, by0, by1 = min(p[0] for p in bp), min(p[2] for p in bp), max(p[3] for p in bp)
pins_bms += [box(new(), bx0, bx0 + 5.2, by0, by1, PCB, PCB + 1.5, MB),                         # power button (APPROXIMATE body)
             cyl(new(), bx0 + 2.6, (by0 + by1) / 2, PCB + 1.4, PCB + 2.0, 3.0, 24, MB)]
pp = part("unoq_pins", pins_bms, (0.2, 0.2, 0.22, 1))
pp["qwiic_mouth"], pp["qwiic_axis"] = to_layout((qx1, qcy, PCB + 1.5)), dir_layout((1, 0, 0))
pp["qwiic_pin1_y_board"] = Q["pin1_y"]
pp["jctl_pins_board"] = [[x, y] for x in jx for y in jrows]

# ---- bottom connectors (JMISC / JMEDIA, 60 pins each, under the board: datasheet drawing, matched by the files' 0.8 pegs) and
#      the low top parts (chips, LED matrix)
part("unoq_bottom_connectors", [box(new(), 10.18, 10.18 + 3.01, 7.43, 7.43 + 38.5, -1.9, 0.0, MB),
                                box(new(), LEN - 8.53 - 3.01, LEN - 8.53, 7.43, 7.43 + 38.5, -1.9, 0.0, MB)], (0.85, 0.8, 0.6, 1))
part("unoq_top_parts", [box(new(), 26.0, 60.0, 8.0, 46.0, PCB, PCB + 2.0, MB)], (0.1, 0.1, 0.12, 1))
old.hide_set(True)
old.hide_render = True
print("UNO Q (Arduino's files): PCB underside at layout z %.1f, USB-C end at the %s; USB-C mouth %s, Qwiic mouth %s"
      % (z_pcb, args.get("usb_end", "rear"), u["mouth"], pp["qwiic_mouth"]))
for n in ("D0_RX", "D1_TX", "GND_D", "GND_P2", "VIN"):
    print("  pin %-7s layout %s" % (n, list(h["pins"][n])))
