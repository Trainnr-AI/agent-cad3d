"""(tools/blender/checks/place_screen.py, 2026-11-28) Candidate UBEC places vs the legs' REAL motions (headless). Per box: walking envelope clearance, STAND->SIT path clearance, and the
largest hip pitch (leg back) still clear by >= 1.5 mm with the knee anywhere in -45..+68 (the folded -90 is forbidden by the hip maps).
Poses whose leg already enters today's bay (top box / column boxes, 0.5 mm) are skipped: they are forbidden today.
ARGS = {"motion", "limits", "boxes": {name: [x0,x1,y0,y1,z0,z1]}}"""
import bpy, json, math, itertools
import numpy as np
from mathutils import Matrix, Vector, Quaternion
MM = 0.001; R = "duck_scs0009"
M = json.load(open(ARGS["motion"])); J = json.load(open(ARGS["limits"]))
trunk = bpy.data.objects[R + ":trunk_base"]
joints = {e["joint"]: e for e in bpy.data.collections[R].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(R + ":")}
q0 = {e.name: e.rotation_quaternion.copy() for e in joints.values()}
for pname, bname in M["parts"].items():
    p, b = bpy.data.objects.get(pname), bpy.data.objects.get(f"{R}:{bname}")
    if p is None or b is None: continue
    q = p.parent
    while q is not None and not q.name.startswith(R + ":"): q = q.parent
    if q is not None: continue
    mw = p.matrix_world.copy(); p.parent = b; p.matrix_world = mw
bpy.context.view_layer.update()
LEGB = {R + ":" + n for n in ("yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left", "yaw2roll_2", "hip_r", "upper_leg_right", "leg_2", "ankle_right")}
pts, distal = {}, set()
for o in bpy.data.objects:
    if o.type != "MESH" or not len(o.data.vertices) or not o.visible_get() or o.name.startswith(R + ":"): continue
    q = o.parent
    while q is not None and not q.name.startswith(R + ":"): q = q.parent
    if q is None or q.name == R + ":trunk_base": continue
    a = np.empty(len(o.data.vertices) * 3); o.data.vertices.foreach_get("co", a); a = a.reshape(-1, 3)
    if len(a) > 1500: a = a[:: len(a) // 1500 + 1]
    pts[o.name] = np.c_[a, np.ones(len(a))]
    if any(k in o.name for k in ("foot", "shin", "knee_cap", "thigh", "ankle")): distal.add(o.name)
print("moving parts", len(pts), "distal", len(distal))
REF = [np.array(b, float) for b in ([-93, 1, -129.2, -70.8, 123, 157], [-53, -5, -118.5, -81.5, 57, 92], [-53, -5, -122, -78, 92, 123])]
boxes = {k: np.array(v, float) for k, v in ARGS["boxes"].items()}


def pose(q):
    for e in joints.values(): e.rotation_quaternion = q0[e.name]
    for jn, ang in q.items():
        if jn in joints: e = joints[jn]; e.rotation_quaternion = q0[e.name] @ Quaternion(Vector(e["joint_axis"]), math.radians(ang))
    bpy.context.view_layer.update()


def both(hp, kn, an, hr, hy):
    q = {}
    for s, sg in (("left", 1), ("right", -1)):
        q.update({f"{s}_hip_pitch": sg * hp, f"{s}_knee": sg * kn, f"{s}_ankle": sg * an, f"{s}_hip_roll": sg * hr, f"{s}_hip_yaw": sg * hy})
    return q


def measure(q):
    pose(q)
    Mi = np.array((trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)).inverted())
    W = {n: (P @ (Mi @ np.array(bpy.data.objects[n].matrix_world)).T)[:, :3] / MM for n, P in pts.items()}
    for n in distal:
        for b in REF:
            if np.any(np.all((W[n] > b[[0, 2, 4]] + 0.5) & (W[n] < b[[1, 3, 5]] - 0.5), 1)): return None
    out = {}
    for k, b in boxes.items():
        m = 1e9
        for n, X in W.items():
            d = np.maximum(np.maximum(b[[0, 2, 4]] - X, X - b[[1, 3, 5]]), 0)
            dist = np.sqrt((d ** 2).sum(1)); dist[np.all((X > b[[0, 2, 4]]) & (X < b[[1, 3, 5]]), 1)] = -1
            m = min(m, float(dist.min()))
        out[k] = m
        WHO[k] = min(((float(np.sqrt((np.maximum(np.maximum(b[[0, 2, 4]] - X, X - b[[1, 3, 5]]), 0) ** 2).sum(1)).min()), n) for n, X in W.items()))[1]
    return out


res = {k: {"walk": 1e9, "sit": 1e9, "hp": {}} for k in boxes}
WHO = {}
WW = {}
WHIT = {k: 0 for k in boxes}
WN = 0
skipped = 0
for hp, kn, an, hr, hy in itertools.product([-45.8, -30, 0, 30, 45.8], [-55, -30, 0, 30, 55], [-23, 0, 23], [-10.9, 0, 10.9], [-6.3, 0, 12.6]):
    r = measure(both(hp, kn, an, hr, hy))
    if r is None: skipped += 1; continue
    WN += 1
    for k, v in r.items():
        if v < 1.5: WHIT[k] += 1
        if v < res[k]["walk"]: WW[k] = (round(v, 1), WHO[k], dict(hip_pitch=hp, knee=kn, ankle=an, roll=hr, yaw=hy))
        res[k]["walk"] = min(res[k]["walk"], v)
S = J["keyframes"]["SIT"]
for t in np.linspace(0, 1, 11):
    r = measure({j: t * v for j, v in S.items()})
    if r is None: skipped += 1; continue
    for k, v in r.items(): res[k]["sit"] = min(res[k]["sit"], v)
for hp in (30, 45.8, 55, 60, 65, 70, 75, 80):
    for kn, an, hr, hy in itertools.product([-45, -20, 0, 30, 55, 68], [-23, 0, 23], [-12, 0, 18.5], [-6.3, 0, 19]):
        r = measure(both(hp, kn, an, hr, hy))
        if r is None: skipped += 1; continue
        for k, v in r.items(): res[k]["hp"][hp] = min(res[k]["hp"].get(hp, 1e9), v)
pose({})
print("skipped (already hit today's bay):", skipped)
for k in boxes: print("WALK", k, f"{WHIT[k]}/{WN} walking poses within 1.5 mm; worst", WW.get(k))
for k, v in res.items():
    ok = [h for h in sorted(v["hp"]) if all(v["hp"][g] >= 1.5 for g in v["hp"] if g <= h)]
    print(f"BOX {k}: walking {v['walk']:.1f} mm | sit path {v['sit']:.1f} mm | leg back clear (>=1.5) up to hip pitch {max(ok) if ok else 'none'} | "
          + " ".join(f"{h}:{c:.1f}" for h, c in sorted(v["hp"].items())))
