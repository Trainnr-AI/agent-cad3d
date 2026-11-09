"""Place a replacement servo at every joint of a (scaled) robot, matching the original servos.
Runs inside Blender.

Inputs: the original robot collection (import_mjcf.py, scale 1), the servos JSON from
find_servos.py / fit_servo_boxes.py, and the target collection (import_mjcf.py with "scale").
Rule (holds for every XGO-Duck joint): the joint origin lies on the original servo's output
face. The new servo goes on the same joint axis with its HORN TOP at the joint origin, its
body extending to the same side, and its long axis (shaft -> far end) in the same direction.
It is parented to the body that held the original servo, so it moves correctly when posed.

ARGS = {"original": "xgoduck_original", "target": "duck_scs0009", "json": "/abs/original_servos.json",
        "servo_script": "/abs/tools/blender/parts/scs0009.py", "horn_top_mm": 4.3,
        "child_held": ["head_yaw", "head_pitch", "left_hip_pitch", ...]}   # joints whose servo sits in the child body
"""
import json

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
orig = bpy.data.collections[args["original"]]
tgt = bpy.data.collections[args["target"]]
HORN = args.get("horn_top_mm", 4.3) / 1000
bpy.context.view_layer.update()


def joint_empties(coll):
    return {e["joint"]: e for e in coll.objects if e.type == "EMPTY" and "joint" in e}


oj, tj = joint_empties(orig), joint_empties(tgt)
mesh_owner = {o.name.split(":", 1)[1].split(".")[0]: o.parent for o in orig.objects if o.type == "MESH"}
servo_code = open(args["servo_script"]).read()

placed = []
for r in json.load(open(args["json"])):
    j = r["joint"]
    eo, et = oj[j], tj[j]
    # holder body in the ORIGINAL; map to the same-named body in the target
    if r["link"] in mesh_owner:
        holder_o = mesh_owner[r["link"]]
    else:
        holder_o = eo if j in args.get("child_held", []) else eo.parent
    holder_t = bpy.data.objects[f"{args['target']}:{holder_o.name.split(':', 1)[1]}"]

    # original servo frame expressed in the joint (child body) frame
    Mj = eo.matrix_world
    inv = Mj.inverted()
    a = (inv.to_3x3() @ Vector(r["shaft_axis"])).normalized()
    centre = inv @ Vector(r["centre"])
    d = 1 if centre.dot(a) > 0 else -1                   # body lies on this side of the joint origin
    far = -(1 if r["shaft_offset_long_mm"] > 0 else -1) * (inv.to_3x3() @ Vector(r["long_axis"])).normalized()
    z = -d * a                                           # SCS0009 +Z points out of the shaft, away from its body
    x = (far - z * far.dot(z)).normalized()
    y = z.cross(x)
    origin = a * 0                                       # joint origin = horn top
    origin = origin - z * HORN                           # case top sits HORN below the horn top
    M_local = Matrix.Translation(origin) @ Matrix((x, y, z)).transposed().to_4x4()

    name = f"scs_{j}"
    ns = {"ARGS": {"name": name, "collection": args["target"], "location": (0, 0, 0), "horn": "cross",
                   "horn_trim_r": args.get("horn_trim_r")}}
    exec(servo_code, ns)
    root = bpy.data.objects[name]
    root.empty_display_size = 0.006
    # world pose in the TARGET: same joint-frame placement, then re-parent to the holder body
    bpy.context.view_layer.update()
    W = et.matrix_world @ M_local
    root.parent = holder_t
    root.matrix_parent_inverse.identity()
    root.matrix_world = W
    root["servo_joint"] = j
    root["holder_body"] = holder_t.name
    root["joint_body"] = et.name                         # pose is defined relative to this joint frame
    root["M_joint_local"] = [v for row in M_local for v in row]
    placed.append(name)

bpy.context.view_layer.update()
print(f"placed {len(placed)} servos in {args['target']}")
