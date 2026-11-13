"""Exploded view of the head v2 (copies '<name>.hx' in ARGS["collection"], replaced on re-run).
Offsets are in layout axes (front +x, left +y, up +z), mm.
ARGS = {"collection": "head_exploded", "offset": [0, -0.3, 0.1], "gap": 1.0}
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
COLL = args.get("collection", "head_exploded")
G = args.get("gap", 1.0)
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
T = bpy.data.objects[args.get("robot", "duck_scs0009") + ":trunk_base"].matrix_world
R3 = T.to_3x3()
off = {
    "head_dome": (0, 0, 62),
    "dome_screws_M2x6st": (0, 0, -50),
    "head_base": (0, 0, 0),
    # head-roll servo: comes out upward and back; its screws go in from behind, the bearing sits in front
    "scs_head_roll_case": (-14, 0, 30), "scs_head_roll_spline": (-14, 0, 30), "scs_head_roll_case_screws": (-14, 0, 30),
    "head_roll_ear_screws_M2x6st": (-34, 0, 30),
    "head_roll_brg_6702ZZ": (14, 0, 30),
    "scs_head_roll_horn_cross": (26, 0, 30), "scs_head_roll_horn_screw": (26, 0, 30),
    # beak servo: slides in from the middle of the head toward the left wall; screws from the right
    "scs_mouth_case": (0, -22, 30), "scs_mouth_spline": (0, -22, 30), "scs_mouth_case_screws": (0, -22, 30),
    "mouth_ear_screws_M2x6st": (0, -44, 30),
    "scs_mouth_horn_cross": (0, 14, 30), "scs_mouth_horn_screw": (0, 28, 30),
    # jaw: up through the bill-plate slots from below
    "head_jaw": (0, 0, -32),
    "jaw_pivot_screw_M2x6st": (0, 30, 12),
    # face panel: rides up into the dome's front groove with the dome; the camera board slides into it from below
    "head_face": (62, 0, 40), "cam_ov5640_b": (38, 0, 40), "cam_screws_M2x6st": (20, 0, 40),
}
n = 0
for name, d in off.items():
    o = bpy.data.objects.get(name)
    if o is None:
        print("  (missing)", name)
        continue
    c = bpy.data.objects.new(name + ".hx", o.data)
    coll.objects.link(c)
    shift = R3 @ (Vector(d) * G * 0.001) + Vector(args.get("offset", (0, -0.3, 0.1)))
    c.matrix_world = Matrix.Translation(shift) @ o.matrix_world
    n += 1
print(f"{COLL}: {n} elements")
