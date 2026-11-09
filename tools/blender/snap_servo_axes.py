"""Snap every placed servo (scs_* root) exactly onto its joint axis and optionally slide it along that
axis; rewrite the layout JSON's M_joint_local to match. Runs inside Blender.

The original servo fits left the shaft axes up to ~0.6 deg off the joint axes; a real joint rotates
about the servo axis, so the model must have them identical. Sliding a servo along its own axis
changes nothing kinematically (used to put the shin's knee plate and ankle floor in one plane).
ARGS = {"json": "/abs/layout_uniform_0p92.json", "axial_mm": {"left_ankle": 6.37, "right_ankle": 6.37}}
       axial_mm: translation along the servo's +Z (toward its horn), applied once (recorded on the root).
"""
import json

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
layout = json.load(open(args["json"])) if args.get("json") else None
roots = [o for o in bpy.data.objects if o.type == "EMPTY" and o.name.startswith("scs_") and "servo_joint" in o]
bpy.context.view_layer.update()
lines = []
for root in roots:
    j = root["servo_joint"]
    jb = bpy.data.objects[root["joint_body"]]
    Wj = jb.matrix_world
    axis_local = Vector(jb["joint_axis"]) if "joint_axis" in jb else Vector((0, 0, 1))
    ja = (Wj.to_3x3() @ axis_local).normalized()
    W = root.matrix_world.copy()
    z = W.to_3x3().col[2].normalized()
    sign = 1.0 if z.dot(ja) > 0 else -1.0
    target = ja * sign
    ang = z.angle(target)
    if ang > 1e-6:
        R = z.rotation_difference(target).to_matrix().to_4x4()
        T = Matrix.Translation(W.translation)
        W = T @ R @ T.inverted() @ W
    shift = args.get("axial_mm", {}).get(j, 0.0)
    if shift and not root.get("axial_shift_mm"):
        W = Matrix.Translation(target * shift * 0.001) @ W
        root["axial_shift_mm"] = shift
    root.matrix_world = W
    bpy.context.view_layer.update()
    M_local = Wj.inverted() @ root.matrix_world
    root["M_joint_local"] = [v for row in M_local for v in row]
    if layout and j in layout.get("servos", {}):
        layout["servos"][j]["M_joint_local"] = [v for row in M_local for v in row]
        if shift:
            layout["servos"][j]["axial_shift_mm"] = shift
    lines.append(f"{j}: tilt {ang * 57.2958:.3f} deg fixed" + (f", slid {shift} mm" if shift and root.get('axial_shift_mm') == shift else ""))
if layout:
    json.dump(layout, open(args["json"], "w"), indent=1)
print("\n".join(lines))
