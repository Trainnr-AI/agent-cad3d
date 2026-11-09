"""Draw servo boxes from a servos JSON (find_servos.py / fit_servo_boxes.py). Runs inside Blender.

ARGS = {"json": "/abs/original_servos.json", "collection": "original_servo_boxes",
        "color": [0.2, 0.5, 1.0, 1], "parent_collection": "xgoduck_original"}
Each box is parented to the joint empty of the robot collection when it can be found, so it
follows the joint when posed.
"""
import json

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
COLL = args.get("collection", "servo_boxes")
old = bpy.data.collections.get(COLL)
if old:
    for ob in list(old.all_objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
mat = bpy.data.materials.get(f"{COLL}_mat") or bpy.data.materials.new(f"{COLL}_mat")
mat.diffuse_color = args.get("color", (0.2, 0.5, 1.0, 1))

for r in json.load(open(args["json"])):
    lx, wy, hz = r["long_mm"] / 1000, r["wide_mm"] / 1000, r["shaft_len_mm"] / 1000
    name = f"servo_{r.get('joint', 'unmatched')}"
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=(lx * 1.03, wy * 1.03, hz * 1.03), verts=bm.verts)   # 3% proud so it shows over the mesh
    # mark the shaft: small cylinder on the +shaft face at the shaft position
    out = bmesh.ops.create_cone(bm, cap_ends=True, segments=16, radius1=0.002, radius2=0.002, depth=0.004)
    sx = -r.get("shaft_offset_long_mm", 0) / 1000
    bmesh.ops.translate(bm, verts=out["verts"], vec=Vector((sx, 0, 0)))
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    R = Matrix((r["long_axis"], r["wide_axis"], r["shaft_axis"])).transposed()
    ob.matrix_world = Matrix.Translation(Vector(r["centre"])) @ R.to_4x4()
    ob["joint"] = r.get("joint", "")
    ob.display_type = "SOLID"
print(f"{len(coll.objects)} boxes in {COLL}")
