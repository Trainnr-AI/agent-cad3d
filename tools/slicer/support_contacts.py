"""Map slicer support-interface points back onto each part in the assembled model and name what the touched surface mates with.
Runs inside Blender (headless: tools/blender/run_headless.py). Read-only on the scene.

For each "<orientation>/<part>" in pts.json: undo the export (print-up rotation + on-bed centring, see export_print.py) using the
G-code wall footprint, cast a ray from each interface point up (print frame) onto the part (<= 1.5 mm), then list every other
visible mesh in the scene within `mate_mm` of that contact. A contact next to a bearing / servo / horn / nut is a FIT surface:
support scars there spoil the fit.

ARGS = {"pts": "opts.json", "out": "contacts.json", "scene": "Scene", "mate_mm": 0.4,
        "details": false}     # details: also per-target face classes (axial = shoulder, radial = bore) + radius, for fit targets
`bbox_fit_err_mm` = footprint mismatch (G-code walls vs mesh, x + y). ~0.84 = normal (wall path centres); much larger means the
contacts of that slice are misplaced - do not trust them.
"""
import json
import re

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

A = ARGS  # noqa: F821 (provided by the runner)
P = json.load(open(A["pts"]))
MATE = A.get("mate_mm", 0.4) / 1000
FIT = re.compile(A.get("fit_regex", r"_brg_|horn|_case|spline|nut|pcb|imu|cam_ov5640|ubec"))
AX = {"px": (1, 0, 0), "mx": (-1, 0, 0), "py": (0, 1, 0), "my": (0, -1, 0), "pz": (0, 0, 1), "mz": (0, 0, -1)}
sc = bpy.data.scenes[A.get("scene", "Scene")]
vl = sc.view_layers[0]
cache = {}


def tree(o):
    if o.name not in cache:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.transform(o.matrix_world)
        bm.normal_update()
        lo = Vector([min(v.co[i] for v in bm.verts) for i in range(3)])
        hi = Vector([max(v.co[i] for v in bm.verts) for i in range(3)])
        cache[o.name] = (BVHTree.FromBMesh(bm), lo, hi, [v.co.copy() for v in bm.verts])
        bm.free()
    return cache[o.name]


others = [o for o in sc.objects if o.type == "MESH" and o.visible_get(view_layer=vl) and len(o.data.polygons)]
for o in others:
    tree(o)
res = {}
for key, d in P.items():
    tag, part = key.split("/")
    ob = bpy.data.objects[part]
    R = Vector(AX[tag]).rotation_difference(Vector((0, 0, 1))).to_matrix()
    Ri = R.inverted()
    t_part = tree(ob)[0]
    rv = [R @ (v * 1000) for v in tree(ob)[3]]
    rlo = Vector([min(v[i] for v in rv) for i in range(3)])
    rhi = Vector([max(v[i] for v in rv) for i in range(3)])
    w = d["wall_bbox"]
    off = Vector(((w[0] + w[2]) / 2 - (rlo.x + rhi.x) / 2, (w[1] + w[3]) / 2 - (rlo.y + rhi.y) / 2, -rlo.z))
    fit_err = abs((w[2] - w[0]) - (rhi.x - rlo.x)) + abs((w[3] - w[1]) - (rhi.y - rlo.y))
    upw = Ri @ Vector((0, 0, 1))
    groups, miss, det = {}, 0, {}
    for x, y, z in d["pts"]:
        pw = (Ri @ (Vector((x, y, z)) - off)) / 1000
        hit = t_part.ray_cast(pw, upw, 0.0015)
        if hit[0] is None:
            miss += 1
            continue
        c, nrm = hit[0], hit[1]
        mates = []
        for o in others:
            if o.name == part:
                continue
            tr, olo, ohi, _ = cache[o.name]
            if any(c[i] < olo[i] - MATE or c[i] > ohi[i] + MATE for i in range(3)):
                continue
            if tr.find_nearest(c, MATE)[0] is not None:
                mates.append(o.name)
        mk = ",".join(sorted(mates)) or "-"
        g = groups.setdefault(mk, {"n": 0, "lo": [1e9] * 3, "hi": [-1e9] * 3})
        g["n"] += 1
        for i in range(3):
            g["lo"][i] = round(min(g["lo"][i], c[i] * 1000), 1)
            g["hi"][i] = round(max(g["hi"][i], c[i] * 1000), 1)
        if A.get("details"):
            for m in mates:
                if not FIT.search(m):
                    continue
                to = bpy.data.objects[m]
                dims = to.dimensions
                ax = min(range(3), key=lambda i: dims[i])        # bearing / horn axis = thinnest local axis
                axis = (to.matrix_world.to_3x3() @ Vector([1.0 if i == ax else 0.0 for i in range(3)])).normalized()
                ctr = sum((to.matrix_world @ Vector(b) for b in to.bound_box), Vector()) / 8
                rel = c - ctr
                r = (rel - axis * rel.dot(axis)).length * 1000
                a = abs(nrm.dot(axis))
                e = det.setdefault(m, {"axial_face": 0, "radial_face": 0, "other": 0, "r_min": 1e9, "r_max": 0})
                e["axial_face" if a > 0.9 else "radial_face" if a < 0.2 else "other"] += 1
                e["r_min"], e["r_max"] = round(min(e["r_min"], r), 1), round(max(e["r_max"], r), 1)
    res[key] = {"print_up": tag, "bbox_fit_err_mm": round(fit_err, 2), "points": len(d["pts"]), "no_hit": miss, "groups": groups}
    if det:
        res[key]["fit_faces"] = det
    print(key, "fit", round(fit_err, 2), "pts", len(d["pts"]), {k: v["n"] for k, v in groups.items()})
json.dump(res, open(A["out"], "w"), indent=1)
print("support contacts written:", A["out"])
