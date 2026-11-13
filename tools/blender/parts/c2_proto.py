"""Type-C2 joint test rig (full kit horn, uncut): one SCS0009 with the full X horn, the C2 fixed part (base, pads, saddle walls,
bearing bridge + post) and the C2 moving part (full-X horn plate + boss in the 6702ZZ) with a link strip, bearing and kit
screw. Built away from the robot in its own collection, to verify the joint type before it goes into the legs and head.
ARGS = {"joint_lib": ..., "base_lib": ..., "dims": ..., "servo_script": ".../scs0009.py", "parts_dir": ...,
        "collection": "proto_c2", "location": [0.35, -0.10, 0.05], "link_deg": -90, "link_len": 26}
"""
import math

import bpy
from mathutils import Matrix

args = globals()["ARGS"]
COLL = args.get("collection", "proto_c2")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):
    bpy.data.objects.remove(o, do_unlink=True)
NAME = "scs_proto_c2"
ns = {"ARGS": {"name": NAME, "collection": COLL, "location": tuple(args.get("location", (0.35, -0.10, 0.05))), "horn": "cross",
               "horn_trim_r": None, "horn_angle_deg": 45.0, "exact": True, "screws": True, "parts_dir": args["parts_dir"]}}
exec(open(args["servo_script"]).read(), ns)
bpy.context.view_layer.update()
M = bpy.data.objects[NAME].matrix_world.copy()

L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
a = math.radians(args.get("link_deg", -90.0))
ln = args.get("link_len", 26.0)
hw = 5.0
ux, uy = math.cos(a), math.sin(a)
px, py = -uy, ux
strip = [(ux * ln + px * s, uy * ln + py * s) for s in (-hw, hw)]                     # a link leaving the plate (stands in for the shin etc.)
sa, ss = L["c2_stator_features"](M)
stator = L["build_part"]("proto_c2_stator", [L["combine_cutters"](sa)], ss, coll, L["material"]("proto_stator", (0.25, 0.45, 0.75, 1)))
ca, cs = L["c2_cap_features"](M)
cap = L["build_part"]("proto_c2_cap", [L["combine_cutters"](ca)], cs, coll, L["material"]("proto_cap", (0.35, 0.65, 0.45, 1)))
ra, rs, rr = L["c2_rotor_features"](M, strip, angle_deg=45.0)
rotor = L["build_part"]("proto_c2_rotor", ra, rs, coll, L["material"]("proto_rotor", (0.9, 0.9, 0.9, 1)), rr)
hw_objs = L["hardware_C2"]("proto_c2_", M, coll)
out = []
for o in (stator, cap, rotor):
    n, nm = L["shells"](o), L["manifold"](o) if "manifold" in L else None
    out.append(f"{o.name}: shells {n}")
print("C2 test rig built: " + " | ".join(out) + f" | hardware: {[o.name for o in hw_objs]}")
