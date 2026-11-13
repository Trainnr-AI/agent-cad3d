"""Arduino UNO Q (ABX00162 / ABX00173) drawn from its datasheet (section 12 Mechanical Information, 4.1 Pinout), in place of the
plain 'uno_q' box of body_v2.py. Runs inside Blender; the old box is hidden, not deleted.

Board frame (mm): origin at the board's corner under the USB-C end, x along the 68.58 side, y along the 53.34 side, z up from the
PCB's underside. EXACT from the drawing: outline 68.58 x 53.34, 4 holes R1.6 at (13.97, 2.54) (15.24, 50.80) (66.04, 7.62)
(66.04, 35.56), two 3.6 x 45 deg corner chamfers at the far end, tallest top part 8.5 above the PCB, USB-C 3 high and 1 past the
edge, bottom connectors 3.01 wide x 38.5 long starting 10.18 / 8.53 from the ends and 7.43 from the y = 0 edge, underside parts
under 2 mm. SCALED from the drawing (+-0.5): where the USB-C, button, JCTL, JSPI and Qwiic sit, header start / end.

ARGS = {"robot": "duck_scs0009", "collection": "electronics", "usb_end": "rear", "joint_lib": ..., "base_lib": ..., "dims": ...}
"""
import bpy
import numpy as np
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
coll = bpy.data.collections.get(args.get("collection", "electronics")) or bpy.data.collections.new(args.get("collection", "electronics"))
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in [o for o in bpy.data.objects if o.name.startswith("unoq_")]:
    bpy.data.objects.remove(o, do_unlink=True)
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
box, cyl, hull, new = L["add_box"], L["add_cyl"], L["add_hull"], L["new_bm"]

LEN, WID, PCB = 68.58, 53.34, 1.6
HOLES = [(13.97, 2.54), (15.24, 50.80), (66.04, 7.62), (66.04, 35.56)]
CH = 3.6
old = bpy.data.objects["uno_q"]
P = np.array([((ML.inverted() @ old.matrix_world @ v.co) * 1000)[:] for v in old.data.vertices])
x_rear, y0, zlid = P[:, 0].min(), (P[:, 1].min() + P[:, 1].max()) / 2 - WID / 2, None
screws = bpy.data.objects.get("uno_screws_M3x8")
lid = bpy.data.objects.get("body_lid")
z_pcb = float(np.array([((ML.inverted() @ lid.matrix_world @ v.co) * 1000)[2] for v in lid.data.vertices]).max())   # on the lid's stand-offs
# board frame -> layout: USB-C end at the rear (x_rear) or at the front
if args.get("usb_end", "rear") == "rear":
    MB = ML @ Matrix.Translation(Vector((x_rear, y0, z_pcb)) * MM)
else:
    MB = ML @ Matrix.Translation(Vector((x_rear + LEN, y0 + WID, z_pcb)) * MM) @ Matrix.Rotation(3.141592653589793, 4, "Z")


def part(name, bms, rgba):
    o = L["to_object"](name, L["combine_cutters"](bms) if len(bms) > 1 else bms[0], coll, L["material"](name, rgba))
    mw = o.matrix_world.copy()
    o.parent = old.parent
    o.matrix_world = mw
    return o


outline = [(0, 0), (LEN - CH, 0), (LEN, CH), (LEN, WID - CH), (LEN - CH, WID), (0, WID)]
pcb = L["build_part"]("unoq_pcb", [hull(new(), [(x, y, z) for x, y in outline for z in (0.0, PCB)], MB)],
                      [cyl(new(), x, y, -0.5, PCB + 0.5, 3.2, 24, MB) for x, y in HOLES], coll, L["material"]("unoq_pcb", (0.0, 0.5, 0.55, 1)))
mw = pcb.matrix_world.copy()
pcb.parent = old.parent
pcb.matrix_world = mw
part("unoq_headers", [box(new(), 17.3, 66.3, 50.8 - 1.27, 50.8 + 1.27, PCB, PCB + 8.5, MB),          # JDIGITAL (18 pins, D0 nearest the far end)
                      box(new(), 26.4, 64.6, 2.54 - 1.27, 2.54 + 1.27, PCB, PCB + 8.5, MB)], (0.05, 0.05, 0.05, 1))   # JANALOG (14 pins: power + A0..A5)
part("unoq_usbc", [box(new(), -1.0, 6.4, 33.3, 42.3, PCB, PCB + 3.0, MB)], (0.75, 0.75, 0.78, 1))                      # USB-C, 1 mm past the edge
part("unoq_pins", [box(new(), 12.0, 24.5, 42.5, 47.6, PCB, PCB + 8.5, MB),                            # JCTL 2 x 5 (1.8 V debug / boot)
                   box(new(), 62.6, 67.7, 23.0, 30.7, PCB, PCB + 8.5, MB),                            # JSPI 2 x 3
                   box(new(), 18.5, 24.5, 0.3, 4.6, PCB, PCB + 3.0, MB),                              # Qwiic (I2C4, 3.3 V)
                   box(new(), 2.4, 6.6, 47.4, 51.6, PCB, PCB + 2.0, MB)], (0.2, 0.2, 0.22, 1))        # power button
part("unoq_bottom_connectors", [box(new(), 10.18, 10.18 + 3.01, 7.43, 7.43 + 38.5, -1.9, 0.0, MB),    # JMISC / JMEDIA, 60 pins each, under the board
                                box(new(), LEN - 8.53 - 3.01, LEN - 8.53, 7.43, 7.43 + 38.5, -1.9, 0.0, MB)], (0.85, 0.8, 0.6, 1))
part("unoq_top_parts", [box(new(), 26.0, 60.0, 8.0, 46.0, PCB, PCB + 2.0, MB)], (0.1, 0.1, 0.12, 1))  # chips, LED matrix (low)
old.hide_set(True)
old.hide_render = True
print("UNO Q drawn: PCB underside at z %.1f, USB-C end at the %s (x %.1f)" % (z_pcb, args.get("usb_end", "rear"), x_rear if args.get("usb_end", "rear") == "rear" else x_rear + LEN))
print("mounting holes (layout x, y):", [tuple(round(c, 2) for c in ((ML.inverted() @ MB @ (Vector((x, y, 0)) * MM)) * 1000)[:2]) for x, y in HOLES])
if screws:
    S = np.array([((ML.inverted() @ screws.matrix_world @ v.co) * 1000)[:] for v in screws.data.vertices])
    cs = []
    for hx, hy in [tuple(((ML.inverted() @ MB @ (Vector((x, y, 0)) * MM)) * 1000)[:2]) for x, y in HOLES]:
        near = S[(abs(S[:, 0] - hx) < 3.5) & (abs(S[:, 1] - hy) < 3.5)]
        cs.append("none" if not len(near) else "%.2f, %.2f off" % (near[:, 0].mean() - hx, near[:, 1].mean() - hy))
    print("existing M3 screws against the exact holes:", cs)
U = np.array([((ML.inverted() @ MB @ (Vector(p) * MM)) * 1000)[:] for p in ((-1.0, 33.3, PCB), (6.4, 42.3, PCB + 3.0))])
print("USB-C socket (layout): x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f" % (U[:, 0].min(), U[:, 0].max(), U[:, 1].min(), U[:, 1].max(), U[:, 2].min(), U[:, 2].max()))
