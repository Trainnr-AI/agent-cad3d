"""Horn vs its groove, for every servo. Runs inside Blender at the zero pose.

The horn is the modelled kit X ("star") horn (parts/scs0009.py: hub D6.5 x 4, plate 1.6, arms 5.6 -> 3.6 wide, 23.88 tip to tip,
or trimmed flat at r 6 for the joints whose horn sits inside a bearing). The groove is cut in the part the horn drives (joint_v2.py:
0.20 mm clearance on every side, crush ribs 0.25 mm into the arms, pocket 0.1 mm above the horn's top).
Everything is measured in the servo's own frame (mm; +Z out of the shaft):
  arms      horn arm directions (deg, mod 90) and the groove's arm directions; their difference
  reach     horn arm length from the shaft axis, groove arm length, end gap
  side gap  gap from each arm's side faces to the groove wall (median; ribs excluded) - designed 0.20
  hub gap   radial gap round the hub - designed 0.20
  ribs      number of rib contacts and their deepest bite into the horn - designed 0.25
  top gap   from the horn's top face to the groove's ceiling - designed 0.10 (open = through)
  depth     how much of the horn's plate thickness is inside the groove (should be all of it)
  screw     the clamp hole above the horn screw: diameter found (designed 2.3, counterbore 4.2)

ARGS = {"servos": ["left_knee", ...] (default: all), "outputs": {"left_knee": "left_shin", ...} (default: servo_review.json),
        "review": "/abs/servo_review.json", "out": "/abs/horn_check.json"}
"""
import json
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
review = {r["servo"][4:]: r for r in json.load(open(args["review"]))}
outputs = args.get("outputs") or {j: r["output"][0][0] for j, r in review.items() if r["output"]}


def frame_bm(o, Fi):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.transform(Matrix.Scale(1000, 4) @ Fi @ o.matrix_world)
    bm.normal_update()
    return bm


D3 = [Vector(d).normalized() for d in ((0.31, 0.17, 0.93), (-0.62, 0.71, 0.33), (0.45, -0.83, -0.33))]


def inside(t, p):
    k = 0
    for d in D3:
        n, q = 0, Vector(p)
        for _ in range(40):
            h = t.ray_cast(q, d)
            if not h[0]:
                break
            n += 1
            q = h[0] + d * 1e-4
        k += n % 2
    return k >= 2


def radial(t, z, ang, r0=0.05, r1=40.0):
    """first wall hit going out from the axis at height z, angle ang (deg); None if open"""
    d = Vector((math.cos(math.radians(ang)), math.sin(math.radians(ang)), 0))
    h = t.ray_cast(Vector((0, 0, z)) + d * r0, d, r1)
    return (h[0] - Vector((0, 0, z))).length if h[0] else None


rows = []
for j in args.get("servos", sorted(outputs)):
    root = bpy.data.objects[f"scs_{j}"]
    horn = bpy.data.objects[f"scs_{j}_horn_cross"]
    part = bpy.data.objects[outputs[j]]
    Fi = root.matrix_world.inverted()
    hb, pb = frame_bm(horn, Fi), frame_bm(part, Fi)
    ht, pt = BVHTree.FromBMesh(hb), BVHTree.FromBMesh(pb)
    H = np.array([v.co[:] for v in hb.verts])
    z0, z1 = H[:, 2].min(), H[:, 2].max()
    rH = np.hypot(H[:, 0], H[:, 1])
    plate = H[H[:, 2] > z1 - 1.0]                                     # the arms' plate (top 1 mm)
    zp = z1 - 0.8                                                     # a height inside the plate
    # horn outline and groove outline, every 0.5 deg, at the plate's height
    angs = np.arange(0, 360, 0.5)
    r_h = np.array([max([(h[0] - Vector((0, 0, zp))).length for h in [ht.ray_cast(Vector((0, 0, zp)) + Vector((math.cos(math.radians(a)), math.sin(math.radians(a)), 0)) * 40, -Vector((math.cos(math.radians(a)), math.sin(math.radians(a)), 0)), 40)] if h[0]] or [0]) for a in angs])
    r_g = np.array([radial(pt, zp, a, 0.05) or np.nan for a in angs])
    # arm directions: peaks of the horn outline / of the groove outline (mod 90)
    def arm_dir(r):
        r = np.nan_to_num(r)
        k = np.argsort(-r)[:40]
        a = (angs[k] % 90.0)
        c = math.degrees(math.atan2(np.sin(np.radians(a * 4)).mean(), np.cos(np.radians(a * 4)).mean())) / 4
        return c % 90.0
    ah, ag = arm_dir(r_h), arm_dir(r_g)
    da = ((ag - ah + 45) % 90) - 45
    reach_h = float(np.max(r_h))
    reach_c = float(np.median([r_h[i] for i, a in enumerate(angs) if abs(((a - ah + 45) % 90) - 45) < 0.3]))     # along the arm's centre line
    tips = [a for a in angs if abs(((a - ah + 45) % 90) - 45) < 0.3]
    reach_g = float(np.nanmedian([radial(pt, zp, a) or np.nan for a in tips]))
    # side gap: from points on the arms' side faces, along the face normal, to the groove wall
    side, ribs, rib_depth = [], 0, 0.0
    for f in hb.faces:
        n = f.normal
        c = f.calc_center_median()
        if abs(n.z) > 0.2 or math.hypot(c.x, c.y) < 3.6 or c.z < z1 - 1.5:
            continue
        for w in (0.25, 0.5, 0.75):
            p = f.verts[0].co.lerp(f.verts[1].co, w).lerp(f.verts[2].co, 0.5) if len(f.verts) >= 3 else c
            p = (p + c) / 2
            if inside(pt, p):
                ribs += 1
                rib_depth = max(rib_depth, pt.find_nearest(p)[3])
                continue
            hh = ht.ray_cast(p + n * 1e-3, n, 3.0)                     # facing more horn (a hole's wall): not an outer side
            if hh[0]:
                continue
            h = pt.ray_cast(p + n * 1e-4, n, 5.0)
            if h[0]:
                side.append((h[0] - p).length)
    side = np.array(side)
    # hub gap
    hub = [radial(pt, z0 + 1.0, a) for a in range(0, 360, 5)]
    hub_r = np.median(rH[(H[:, 2] < z1 - 1.8)]) if (H[:, 2] < z1 - 1.8).any() else None
    hub_horn = max(rH[H[:, 2] < z0 + 1.5]) if (H[:, 2] < z0 + 1.5).any() else None
    hub_gap = (min([h for h in hub if h]) - hub_horn) if hub_horn is not None and any(hub) else None
    # top gap: above the plate's top face, over the arms
    tops = []
    for a in np.arange(ah, ah + 360, 90):
        for r in (4.0, 6.0, 8.0):
            if r > reach_h - 0.5:
                continue
            p = Vector((r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z1 + 1e-3))
            h = pt.ray_cast(p, Vector((0, 0, 1)), 10.0)
            tops.append((h[0].z - z1) if h[0] else np.inf)
    # depth: is the plate inside the groove? (groove wall present beside the arm at the plate's bottom and top)
    zb = z1 - 1.6 + 0.15
    d_lo = radial(pt, zb, ah + 45)                                    # between two arms, at the plate's bottom: groove wall exists?
    # screw hole above the horn screw (clamp hole through the part, on the axis)
    up = pt.ray_cast(Vector((0, 0, z1 + 0.5)), Vector((0, 0, 1)), 30.0)
    hole = [radial(pt, z1 + 1.5 + k, a) for k in (0.0, 3.0) for a in range(0, 360, 30)]
    hole_d = [2 * np.nanmedian([h for h in hole[:12] if h] or [np.nan]), 2 * np.nanmedian([h for h in hole[12:] if h] or [np.nan])]
    row = dict(servo=j, part=part.name, horn="trimmed r%.1f" % reach_h if reach_h < 9 else "full cross r%.2f" % reach_h,
               arm_horn_deg=round(ah, 1), arm_groove_deg=round(ag, 1), arm_misalign_deg=round(da, 2),
               reach_horn=round(reach_h, 2), reach_groove=round(reach_g, 2), end_gap=round(reach_g - reach_c, 2),
               side_gap_median=round(float(np.median(side)), 3) if len(side) else None, side_gap_min=round(float(side.min()), 3) if len(side) else None,
               side_gap_max=round(float(np.percentile(side, 95)), 3) if len(side) else None,
               hub_gap=round(float(hub_gap), 2) if hub_gap is not None else None, rib_points=ribs, rib_depth=round(rib_depth, 3),
               top_gap=round(float(np.median(tops)), 2) if tops else None, plate_in_groove=d_lo is not None and d_lo < 12,
               hole_d=[round(float(x), 2) for x in hole_d], open_above=up[0] is None)
    rows.append(row)
    hb.free()
    pb.free()
json.dump(rows, open(args["out"], "w"), indent=1)
print(f"{'servo':16} {'part':22} {'horn':16} {'arms h/g/diff':>17} {'reach h/g':>11} {'end':>5} {'side med/min':>12} {'hub':>5} {'ribs':>9} {'top':>5} hole d")
for r in rows:
    print(f"{r['servo']:16} {r['part']:22} {r['horn']:16} {r['arm_horn_deg']:5.1f}/{r['arm_groove_deg']:5.1f}/{r['arm_misalign_deg']:+5.2f} "
          f"{r['reach_horn']:5.2f}/{r['reach_groove']:5.2f} {r['end_gap']:5.2f} {r['side_gap_median']}/{r['side_gap_min']} {r['hub_gap']} "
          f"{r['rib_points']:3d}@{r['rib_depth']:.2f} {r['top_gap']} {r['hole_d']}")
