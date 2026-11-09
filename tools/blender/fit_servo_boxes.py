"""Recover servo poses that are fused into other geometry, by fitting a servo box on a joint axis.
Runs inside Blender on a collection made by import_mjcf.py.

For each joint in ARGS["joints"], candidate boxes of size (long, wide, shaft) are placed with
their shaft axis on the joint axis and the shaft at ARGS["shaft_offset_long_mm"] from the box
centre along the long axis. We scan the long-axis direction around the joint axis in 90° steps
(the CAD is square to the link frames; finer steps let diagonal boxes win by accident), the box
position along the axis, and score each candidate by how many mesh vertices lie on its
surface. The best candidate per joint is appended to the JSON written by find_servos.py.

ARGS = {"collection": "xgoduck_original", "joints": ["left_hip_yaw", ...], "box_mm": [34, 20, 29],
        "shaft_offset_long_mm": 7.5, "json": "/abs/original_servos.json",
        "holder_mesh": {"left_hip_yaw": "trunk_base", ...}}   # score only vertices of the mesh that
                                                              # holds the servo (from the CAD parts list)
"""
import json
import math

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
coll = bpy.data.collections[args["collection"]]
L, W, H = [v / 1000 for v in args["box_mm"]]
OFF = args.get("shaft_offset_long_mm", 7.5) / 1000
SURF = 0.0003
bpy.context.view_layer.update()

verts_by_mesh = {}
for ob in coll.objects:
    if ob.type == "MESH":
        mw = ob.matrix_world
        verts_by_mesh[ob.name.split(":", 1)[1].split(".")[0]] = [mw @ v.co for v in ob.data.vertices]
holder_mesh = args.get("holder_mesh", {})

joints = {e["joint"]: e for e in coll.objects if e.type == "EMPTY" and "joint" in e}
records = json.load(open(args["json"]))
records = [r for r in records if r.get("joint") not in args["joints"]]


import numpy as np

HALF = np.array([L / 2, W / 2, H / 2])


def score(centre, ax_long, ax_wide, ax_shaft):
    R = np.array([list(ax_long), list(ax_wide), list(ax_shaft)])      # rows = box axes
    loc = np.abs((near - np.array(list(centre))) @ R.T)               # |box-local coords|
    inside = np.all(loc <= HALF + SURF, axis=1)
    on_face = np.min(HALF - loc, axis=1) < SURF
    return int(np.count_nonzero(inside & on_face))


def obb(r):
    return (np.array(r["centre"]), np.array([r["long_axis"], r["wide_axis"], r["shaft_axis"]]),
            np.array([r["long_mm"], r["wide_mm"], r["shaft_len_mm"]]) / 2000)


def overlap(A, B):
    """separating-axis test for two oriented boxes; True if they intersect by > 0.2 mm"""
    (ca, Ra, ha), (cb, Rb, hb) = A, B
    d = cb - ca
    axes = list(Ra) + list(Rb) + [np.cross(x, y) for x in Ra for y in Rb]
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        if np.abs(Ra @ ax) @ ha + np.abs(Rb @ ax) @ hb - abs(d @ ax) <= 0.0002:
            return False
    return True


for name in args["joints"]:
    e = joints[name]
    placed = [obb(r) for r in records]
    mw = e.matrix_world
    p = mw.translation.copy()
    a = (mw.to_3x3() @ Vector(e["joint_axis"])).normalized()
    ref = (mw.to_3x3() @ Vector((1, 0, 0))).normalized()
    pool = verts_by_mesh[holder_mesh[name]] if name in holder_mesh else [v for vs in verts_by_mesh.values() for v in vs]
    near = np.array([list(v) for v in pool if (v - p).length < 0.06])
    best = None
    for k in range(4):                                    # CAD parts are square to the link frame: 90° steps
        ax_long = (Matrix.Rotation(k * math.pi / 2, 3, a) @ ref).normalized()
        ax_wide = a.cross(ax_long).normalized()
        for s in range(-90, 91):                          # box centre along the axis, 0.5 mm steps
            centre = p + a * (s * 0.0005) - ax_long * OFF
            sc = score(centre, ax_long, ax_wide, a)
            if best is not None and sc <= best[0]:
                continue
            cand = (np.array(list(centre)), np.array([list(ax_long), list(ax_wide), list(a)]), HALF)
            if any(overlap(cand, P) for P in placed):    # a real servo can't overlap another one
                continue
            if best is None or sc > best[0]:
                best = (sc, centre, ax_long, ax_wide, s * 0.5)
    sc, centre, ax_long, ax_wide, along = best
    records.append({"link": holder_mesh.get(name, "(fitted)"), "fitted": True, "joint": name, "centre": list(centre), "size_mm": args["box_mm"],
                    "shaft_axis": list(a), "long_axis": list(ax_long), "wide_axis": list(ax_wide),
                    "shaft_len_mm": args["box_mm"][2], "long_mm": args["box_mm"][0], "wide_mm": args["box_mm"][1],
                    "shaft_offset_long_mm": args.get("shaft_offset_long_mm", 7.5), "shaft_offset_wide_mm": 0.0,
                    "fit_score": sc, "centre_along_axis_mm": along})
    print(f"  {name:16} fitted: score {sc} surface verts, box centre {along:+.1f} mm along axis")

json.dump(records, open(args["json"], "w"), indent=1)
print(f"{len(records)} servos in {args['json']}")
