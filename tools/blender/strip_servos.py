"""Cut the original servo bodies out of a scaled copy of a robot's visual meshes. Runs inside Blender.

Uses the servo boxes from find_servos.py / fit_servo_boxes.py (measured on the ORIGINAL at
scale 1). Each box is re-expressed relative to its holder body, scaled by ARGS["scale"], placed
on the same body of the target, and every mesh face lying inside it (inflated by
ARGS["inflate_mm"]) is deleted from the target's meshes. Run once after import_mjcf.py with
the same scale; the original collection is not touched.

ARGS = {"original": "xgoduck_original", "target": "duck_scs0009", "json": "/abs/original_servos.json",
        "scale": 0.92, "inflate_mm": 0.6, "child_held": ["head_yaw"]}
"""
import json

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
O, T = args["original"], args["target"]
S = args["scale"]
INF = args.get("inflate_mm", 0.6) / 1000
bpy.context.view_layer.update()


def bodies(c):
    return {o.name.split(":", 1)[1]: o for o in bpy.data.collections[c].objects if o.type == "EMPTY" and o.name.startswith(c + ":")}


bo, bt = bodies(O), bodies(T)
jo = {e["joint"]: e for e in bo.values() if "joint" in e}
mesh_owner = {o.name.split(":", 1)[1].split(".")[0]: o.parent for o in bpy.data.collections[O].objects if o.type == "MESH"}
target_meshes = [o for o in bpy.data.collections[T].objects if o.type == "MESH" and o.name.startswith(T + ":")]

boxes = []
for r in json.load(open(args["json"])):
    j = r["joint"]
    holder = mesh_owner.get(r["link"].split(".")[0]) or (jo[j] if j in args.get("child_held", []) else jo[j].parent)
    W = Matrix.Translation(Vector(r["centre"])) @ Matrix((r["long_axis"], r["wide_axis"], r["shaft_axis"])).transposed().to_4x4()
    L = holder.matrix_world.inverted() @ W
    L.translation *= S                                         # body-relative position scales with the robot
    Wt = bt[holder.name.split(":", 1)[1]].matrix_world @ L
    half = Vector((r["long_mm"], r["wide_mm"], r["shaft_len_mm"])) * (S / 2000) + Vector((INF, INF, INF))
    boxes.append((j, Wt, half))

removed = {}
for ob in target_meshes:
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    mw = ob.matrix_world
    world = [mw @ v.co for v in bm.verts]
    inside = [False] * len(bm.verts)
    for _, Wt, half in boxes:
        inv = Wt.inverted()
        for i, p in enumerate(world):
            if not inside[i]:
                q = inv @ p
                inside[i] = abs(q.x) <= half.x and abs(q.y) <= half.y and abs(q.z) <= half.z
    faces = [f for f in bm.faces if all(inside[v.index] for v in f.verts)]
    if faces:
        bmesh.ops.delete(bm, geom=faces, context="FACES")
        bm.to_mesh(me)
        removed[ob.name.split(":", 1)[1]] = len(faces)
    bm.free()
    me.update()
print(f"removed old-servo faces from {len(removed)} meshes: " + ", ".join(f"{k} {v}" for k, v in sorted(removed.items())))
