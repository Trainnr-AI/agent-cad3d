"""Export one STL per robot body (in the body's own frame, metres) from what is visible in Blender.
Runs inside Blender. Used to put the redesigned parts into the MuJoCo model (tools/sim/apply_body_meshes.py).

Per body: every visible mesh whose nearest robot-body ancestor is that body; servo horns go to the
body their joint moves (they turn with the output); objects without a body ancestor can be assigned
with ARGS["assign"]. Writes <out_dir>/<body>.stl and a JSON manifest listing what went into each.

ARGS = {"robot": "duck_scs0009", "collections": ["duck_scs0009", "leg_v1", "neck_v1"],
        "assign": {"left_brg_ankle": "ankle_left"}, "out_dir": "/abs/sim/assets_v1"}
"""
import json
import os
import struct

import bpy

args = globals()["ARGS"]
R = args["robot"]
out = args["out_dir"]
os.makedirs(out, exist_ok=True)
bpy.context.view_layer.update()

bodies = {o.name: o for o in bpy.data.collections[R].objects if o.type == "EMPTY" and o.name.startswith(R + ":")}


def body_name(empty):
    return empty.name.split(":", 1)[1].split(".")[0]


def owner(o):
    if o.parent is not None and "servo_joint" in o.parent and "_horn" in o.name:
        # the horn belongs to the side of the joint that does NOT hold the servo
        jb, holder = bpy.data.objects[o.parent["joint_body"]], bpy.data.objects[o.parent["holder_body"]]
        return jb.parent if holder == jb else jb
    p = o.parent
    while p is not None and p.name not in bodies:
        p = p.parent
    return p


groups = {}
seen = set()
for cname in args["collections"]:
    for o in bpy.data.collections[cname].all_objects:
        if o.type != "MESH" or o.name in seen or not o.visible_get() or o.hide_get() or o.hide_render \
                or o.name.endswith(".x") or any(o.name.startswith(p) for p in args.get("exclude_prefix", [])):
            continue
        seen.add(o.name)
        b = owner(o)
        if b is None and o.name in args.get("assign", {}):
            b = bodies[f"{R}:{args['assign'][o.name]}"] if f"{R}:{args['assign'][o.name]}" in bodies else \
                next(e for n, e in bodies.items() if body_name(e) == args["assign"][o.name])
        if b is None:
            print("  (skipped, no body):", o.name)
            continue
        groups.setdefault(b, []).append(o)

dg = bpy.context.evaluated_depsgraph_get()
manifest = {}
for b, objs in groups.items():
    inv = b.matrix_world.inverted()
    tris = []
    for o in objs:
        oe = o.evaluated_get(dg)
        me = oe.to_mesh()
        me.calc_loop_triangles()
        M = inv @ o.matrix_world
        vs = [M @ v.co for v in me.vertices]
        for t in me.loop_triangles:
            tris.append(tuple(vs[i] for i in t.vertices))
        oe.to_mesh_clear()
    name = body_name(b)
    path = os.path.join(out, f"{name}.stl")
    with open(path, "wb") as f:
        f.write(b"agent-cad3d body export".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, c, d in tris:
            n = (c - a).cross(d - a)
            n = n.normalized() if n.length > 0 else n
            f.write(struct.pack("<12fH", n.x, n.y, n.z, a.x, a.y, a.z, c.x, c.y, c.z, d.x, d.y, d.z, 0))
    manifest[name] = {"stl": os.path.basename(path), "triangles": len(tris), "objects": sorted(o.name for o in objs)}
    print(f"  {name:16} {len(objs):3d} objects, {len(tris):6d} triangles")
json.dump(manifest, open(os.path.join(out, "manifest.json"), "w"), indent=1)
print(f"exported {len(manifest)} bodies to {out}")
