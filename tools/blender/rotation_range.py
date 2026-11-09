"""Free rotation range of a joint assembly (e.g. parts/scs0009_joint.py). Runs inside Blender.

Rotates ARGS["rotor"] about its local Z in ARGS["step_deg"] steps, each way, until any mesh under
the rotor intersects any other mesh under ARGS["root"] (triangle-level BVH test). Objects whose
name contains one of ARGS["ignore"] are skipped (e.g. the bearing, which spans both sides).
Restores the rotor afterwards.

ARGS = {"root": "dm_joint", "rotor": "dm_rotor", "step_deg": 2, "ignore": ["bearing"],
        "allow": [["spline", "case"]]}   # intended contacts (the output shaft leaves the case)
"""
import math

import bpy
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
root = bpy.data.objects[args["root"]]
rotor = bpy.data.objects[args["rotor"]]
ign = args.get("ignore", [])
dg = bpy.context.evaluated_depsgraph_get()
moving = [o for o in rotor.children_recursive if o.type == "MESH" and not any(i in o.name for i in ign)]
under_rotor = set(rotor.children_recursive)
fixed = [o for o in root.children_recursive if o.type == "MESH" and o not in under_rotor
         and not any(i in o.name for i in ign)]


def tree(o):
    bpy.context.view_layer.update()
    mw = o.matrix_world
    me = o.data
    return BVHTree.FromPolygons([mw @ v.co for v in me.vertices], [p.vertices[:] for p in me.polygons])


fixed_trees = [(o.name, tree(o)) for o in fixed]
r0 = rotor.rotation_euler.copy()


def clash(angle):
    rotor.rotation_euler = (r0.x, r0.y, angle)
    bpy.context.view_layer.update()
    for m in moving:
        tm = tree(m)
        for n, tf in fixed_trees:
            if any((a in m.name and b in n) or (b in m.name and a in n) for a, b in args.get("allow", [])):
                continue
            if tm.overlap(tf):
                return f"{m.name} x {n}"
    return None


at_zero = clash(0.0)
limits = {}
for sign in (1, -1):
    a = 0
    hit = None
    while a < 180:
        a += args.get("step_deg", 2)
        hit = clash(sign * math.radians(a))
        if hit:
            break
    limits[sign] = (a - args.get("step_deg", 2), hit)
rotor.rotation_euler = r0
bpy.context.view_layer.update()
print(f"at 0°: {'CLASH ' + at_zero if at_zero else 'clear'}")
for sign, (a, hit) in limits.items():
    print(f"  {'+' if sign > 0 else '-'} direction: free to {sign * a:+d}°" + (f"  (then {hit})" if hit else "  (full 180°)"))
