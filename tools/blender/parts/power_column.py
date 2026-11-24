"""Hardware dummies for the all-Dupont bay (2026-11-25). Runs inside Blender after body_v2.py (with "mini560_hood": true).
  Mini560 standing in the box on the hood's rear wall: PCB, components (toward the bay), 4 straight pins with Dupont
  housings into the chamber behind, 3 heat-shrink splices in the same chamber.
  The two Dupont housings on the switch's lugs (switch on the left-wall plate, lugs pointing inward across the bay).
Layout mm (y = offset from the centre plane CY = -100), the trunk frame + HOME as body_v2.py.
ARGS = {"robot": "duck_scs0009", "collection": "power_hw", "joint_lib": ..., "base_lib": ..., "dims": ...,
        "switch_x": -35.2, "switch_z": 108.0, "lug_y": -3.1}
"""
import bpy
from mathutils import Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "power_hw")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):
    bpy.data.objects.remove(o, do_unlink=True)
Mt = bpy.data.objects[f"{R}:trunk_base"].matrix_world.copy()
HOME = Vector((0.0, -100.0, 118.8))
CY = -100.0


def box(x0, x1, y0, y1, z0, z1, bm=None):
    return L["add_box"](bm or L["new_bm"](), x0 - HOME.x, x1 - HOME.x, y0 + CY - HOME.y, y1 + CY - HOME.y, z0 - HOME.z, z1 - HOME.z, Mt)


def part(name, bm, rgba):
    return L["to_object"](name, bm, coll, L["material"](name + "_mat", rgba))


# ---- Mini560 in the hood box (body_v2.py "mini560_hood", 2026-11-25 harness layout): the PCB at the BACK of the box (x -77.7..-76.5),
#      parts toward the back plate, 4 straight pins + Dupont housings toward the bay, ending 6.5 mm short of the inner wall (wire space).
#      Lower pin row = IN (closest to the pigtail from below), upper row = OUT; y -11.5 = +, y -3.9 = -.  Anchors = where the wires leave.
BX1 = -54.3                                                  # the hood's rear face = the box's inner side; box 30.2 deep
PCB = (BX1 - 30.2 + 6.8, BX1 - 30.2 + 8.0)                   # (-77.7, -76.5)
Z0, ZC = 120.2, 120.2 + 15.0                                 # board bottom, centre
PLUG_END = PCB[1] + 14.5                                     # -62.0: the housings' ends (wires leave here, toward the bay)
part("mini560_pcb", box(PCB[0], PCB[1], -16.7, 1.3, Z0, Z0 + 30.0), (0.1, 0.25, 0.7, 1))          # box y -17.6..+2.2, walls 0.8
part("mini560_parts", box(PCB[0] - 4.2, PCB[0], -16.2, 0.8, Z0 + 2.0, Z0 + 28.0), (0.3, 0.3, 0.32, 1))
bm = L["new_bm"]()
for z in (ZC - 13.0, ZC + 13.0):                             # pad pairs 26 apart along the board
    for y in (-11.5, -3.9):
        box(PCB[1] - 0.01, PLUG_END, y - 1.27, y + 1.27, z - 1.27, z + 1.27, bm)   # 14 mm housing + 0.5 over the pin's root
ob = part("mini560_plugs", bm, (0.1, 0.1, 0.1, 1))
ob["anchors"] = {"in_pos": [PLUG_END, CY - 11.5, ZC - 13.0], "in_neg": [PLUG_END, CY - 3.9, ZC - 13.0],      # full layout y (CY + offset),
                 "out_pos": [PLUG_END, CY - 11.5, ZC + 13.0], "out_neg": [PLUG_END, CY - 3.9, ZC + 13.0]}     # as wiring.py's wp() expects
bm = L["new_bm"]()
SPL = (PLUG_END + 0.2, PLUG_END + 4.2)                       # 3 heat-shrink splices, O 4 x 14, across the wire space, 0.5 short of the window
for z in (ZC - 6.0, ZC, ZC + 6.0):
    box(SPL[0], SPL[1], -15.0, -1.0, z - 2.0, z + 2.0, bm)
ob = part("splices_x3", bm, (0.15, 0.15, 0.15, 1))
ob["anchors"] = {"sw_out": [SPL[0] + 2.0, CY - 8.0, ZC + 6.0],   # switched +: switch -> Mini560 IN+ and Uno Q VIN
                 "bat_neg": [SPL[0] + 2.0, CY - 8.0, ZC],        # battery -: pigtail black -> Mini560 IN- and Uno Q GND
                 "bat_pos": [SPL[0] + 2.0, CY - 8.0, ZC - 6.0]}  # battery +: pigtail red -> jumper to the switch's COM lug

# ---- switch lug housings (two of the three lugs): the lugs stand in a VERTICAL row (bay_switch_joints: z 103.3 / 108 / 112.7, 4.7 apart)
#      at x -35.2, pointing inward (-y) out of the body's face at y -3.1; housings on COM (middle) and the lower lug (lever UP = on)
sx, sz, ly = args.get("switch_x", -35.2), args.get("switch_z", 108.0), args.get("lug_y", -3.1)
bm = L["new_bm"]()
for z in (sz, sz - 4.7):
    box(sx - 1.27, sx + 1.27, ly - 0.5 - 14.5, ly - 0.5, z - 1.27, z + 1.27, bm)   # 0.5 off the body's face, 14.5 long -> ends at y -18.1
ob = part("switch_lug_plugs", bm, (0.1, 0.1, 0.1, 1))
ob["anchors"] = {"com": [sx, CY + ly - 15.0, sz], "on": [sx, CY + ly - 15.0, sz - 4.7]}
print("power hardware dummies placed:", [o.name for o in coll.objects])
