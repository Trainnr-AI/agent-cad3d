"""OPTIONAL covers (the duck works without them): thigh covers and a neck cover. Runs inside Blender at the zero pose, after
the legs, neck and wiring are built (it measures the hubs and plugs it has to enclose from the scene).

  <side>_thigh_cover   closes the thigh's OUTER side, TOP and REAR (as the original's thigh shell): a plate with the original
                       thigh outline grown just enough for the hubs on the top and rear, walls on the top and rear down to the
                       thigh plate, a short lip elsewhere (the shin's ankle end passes close to the knee). 2x M2x6: one from the
                       outside into a post on the thigh's web, one from the top into a boss at the thigh's top rear corner.
  mounts               the post and the boss are part of the thigh (parts/leg_v2.py, "cover_mounts"); their places are read from the thigh.

ARGS = {"robot": "duck_scs0009", "collection": "covers", "sides": ["left", "right"], "joint_lib": ..., "base_lib": ..., "dims": ...}
"""
import math

import bpy
import numpy as np
from mathutils import Matrix, Vector, geometry

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "covers")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
box, cyl, hull, new = L["add_box"], L["add_cyl"], L["add_hull"], L["new_bm"]
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
MLi = ML.inverted()
mat_cover = L["material"]("leg_cover", (0.85, 0.87, 0.9, 1))
steel = L["material"]("hw_steel", (0.6, 0.62, 0.66, 1))
report = []


def lay(o):
    return np.array([((MLi @ o.matrix_world @ v.co) / MM)[:] for v in o.data.vertices])


def finish(name, part, parent):
    ok, vol = L["manifold"](part)
    n = L["shells"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["optional"] = True
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    mw = part.matrix_world.copy()
    part.parent = parent
    part.matrix_world = mw
    return part


for side in args.get("sides", ["left", "right"]):
    for n_ in (f"{side}_thigh_cover", f"{side}_thigh_cover_screws_M2x6st"):
        if n_ in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects[n_], do_unlink=True)
    F = bpy.data.objects[f"scs_{side}_hip_pitch"].matrix_world.copy()        # origin on the hip-pitch shaft at the tower top, +Z out of the shaft (inboard)
    Fi = F.inverted()
    thigh = bpy.data.objects[f"{side}_thigh"]
    fr = lambda p: (Fi @ (ML @ (Vector(p) * MM))) / MM                        # layout mm -> servo frame mm
    hpo = (MLi @ F.translation) / MM
    kno = (MLi @ bpy.data.objects[f"scs_{side}_knee"].matrix_world.translation) / MM

    def proj(pts_xz):
        return [tuple(fr((x, hpo.y, z))[:2]) for x, z in pts_xz]

    def plate(z0, z1, pts_xz):
        return hull(new(), [(x, y, z) for x, y in proj(pts_xz) for z in (min(z0, z1), max(z0, z1))], F)

    # what has to be covered: the thigh's hubs and their plugs (wiring collection), in the servo frame
    T_ = side[0].upper() + "T"
    cont = [o for o in bpy.data.collections["wiring"].objects if o.type == "MESH" and (o.name.startswith(f"wire_hub_{T_}") or
            (o.name.startswith("wire_plug_") and any(k in o.name for k in (f"lead_{side}_hip", f"lead_{side}_knee", f"lead_{side}_ankle", f"{side}_thigh", f"{side}_body_to_thigh"))))]
    Pt = lay(thigh)
    cont = [o for o in cont if (lay(o).min(0) > Pt.min(0) - 14.0).all() and (lay(o).max(0) < Pt.max(0) + 14.0).all()]      # only what sits on this thigh (not a cable's far end)
    Pc = np.concatenate([lay(o) for o in cont])
    Ps = np.array([fr(p)[:] for p in Pc])                                      # servo frame: z most negative = outermost
    z_out = Ps[:, 2].min()
    _om = bpy.data.objects[f"{R}:{side}_hip_pitch"]                            # the original thigh at our scale: its side outline
    _ov = [(MLi @ _om.matrix_world @ v.co) / MM for v in _om.data.vertices]
    OUT = [(_ov[i].x, _ov[i].z) for i in geometry.convex_hull_2d([Vector((v.x, v.z)) for v in _ov])]
    c0 = Vector((sum(x for x, _ in OUT) / len(OUT), sum(z for _, z in OUT) / len(OUT)))

    def grown(off):
        return [(x + off * math.cos(2 * math.pi * q / 16), z + off * math.sin(2 * math.pi * q / 16)) for x, z in OUT for q in range(16)]

    def inside(pts, x, z):
        h = [pts[i] for i in geometry.convex_hull_2d([Vector(p) for p in pts])]
        return all((h[(i + 1) % len(h)][0] - h[i][0]) * (z - h[i][1]) - (h[(i + 1) % len(h)][1] - h[i][1]) * (x - h[i][0]) >= 0 for i in range(len(h))) or \
               all((h[(i + 1) % len(h)][0] - h[i][0]) * (z - h[i][1]) - (h[(i + 1) % len(h)][1] - h[i][1]) * (x - h[i][0]) <= 0 for i in range(len(h)))

    RIM, CV_T, GAP = 1.0, 1.0, 0.8
    assert "cover_post" in thigh.keys(), "rebuild the leg with parts/leg_v2.py (cover_mounts) first"
    px, py, ZS_TIP = thigh["cover_post"]                                       # post on the web; ZS_TIP = inner face of the outer plate
    bx_, bz0_, bz1_, yb = thigh["cover_boss"]                                  # boss for the top screw
    off = thigh["cover_grow"]                                                  # the original outline grown evenly by this much ...
    if z_out - GAP < ZS_TIP - 0.05:
        report.append(f"{side}: WARNING hubs / plugs reach {ZS_TIP - z_out + GAP:.2f} mm past the cover plate's place")
    top_rear = Pc[(Pc[:, 2] > kno.z + 9.0)]                                    # ... and out to whatever of the hubs / plugs above the knee's swing lies beyond that
    ext = [(x + sx * GAP, z + sz * GAP) for x, _, z in top_rear for sx in (-1, 1) for sz in (-1, 1)]
    OUTER = grown(off) + [(x + (RIM if x > c0.x else -RIM), z + (RIM if z > c0.y else -RIM)) for x, z in ext]
    INNER = grown(off - RIM) + ext
    ZS_DEEP, ZS_LIP = -0.3, L["J_BOTTOM"] - 1.5
    shell = plate(ZS_TIP - CV_T, ZS_DEEP, OUTER)
    room = plate(ZS_TIP, ZS_DEEP + 1.0, INNER)
    hyp = max(((OUT[q], OUT[(q + 1) % len(OUT)]) for q in range(len(OUT))),
              key=lambda e: (Vector(e[1]) - Vector(e[0])).length if (e[1][0] - e[0][0]) * (e[1][1] - e[0][1]) > 0 else 0.0)   # the long sloping front edge
    hA, hB = Vector(hyp[0]), Vector(hyp[1])
    hd = (hB - hA).normalized()
    hn = Vector((-hd.y, hd.x))
    if hn.dot(Vector((kno.x, kno.z + 20.0)) - hA) < 0:
        hn = -hn
    cut_front = plate(ZS_LIP, ZS_DEEP + 2.0, [tuple(hA + hd * a + hn * b) for a in (-150.0, 150.0) for b in (-150.0, 14.0)])
    cut_knee = plate(ZS_LIP + 0.01, ZS_DEEP + 2.01, [(x, z) for x in (-150.0, 150.0) for z in (-100.0, kno.z + 9.0)])
    zt = bz1_ + 2.25                                                           # top of the cover's top wall over the boss (flat there)
    Mz = ML @ Matrix.Translation(Vector((bx_, 0, 0)) * MM)
    pad = hull(new(), [(x, y, z) for x, y in proj([(bx_ - 3.4, zt - 2.0), (bx_ + 3.4, zt - 2.0), (bx_ - 3.4, zt - 0.02), (bx_ + 3.4, zt - 0.02)]) for z in (-0.5, -6.6)], F)   # seat under the top screw's head
    holes = [cyl(new(), px, py, ZS_TIP - CV_T - 0.5, ZS_TIP + 0.5, L["CLEAR_D"], 16, F),
             cyl(new(), 0, yb, zt - 2.5, zt + 0.5, L["CLEAR_D"], 16, Mz)]
    cover = L["build_part"](f"{side}_thigh_cover", [shell], [room, cut_front, cut_knee], coll, mat_cover)
    L["apply_bool"](cover, pad, "UNION", coll)                                 # after the room is cut: the pad hangs under the top wall, on the boss
    for h_ in holes:
        L["apply_bool"](cover, h_, "DIFFERENCE", coll)
    L["heal_slivers"](cover, cover.name)
    finish(f"{side}_thigh_cover", cover, thigh.parent)
    scr = L["pan_screw_dummy"](f"{side}_thigh_cover_screws_M2x6st", [(F, px, py, ZS_TIP - CV_T, +1), (Mz, 0, yb, zt, -1)], coll, steel)
    mw = scr.matrix_world.copy()
    scr.parent = thigh.parent
    scr.matrix_world = mw
    report.append(f"{side}: outline grown {off:.2f} mm, outer plate {(-ZS_TIP + L['J_BOTTOM']):.1f} mm beyond the servos' end faces")
# =========================================================== neck cover (optional, snaps on - no screws)
# One C-shaped piece put on from the left: a cap over the hub on the cradle's front, a cap over the hub on its rear, joined by a strap
# across the servos' left side (between the body's horn plate below and the head-pitch U's arm above). Open underneath (cables).
# Held by: the strap against the servo towers (right), two 0.8 mm snap tabs behind the cradle floor's right face (left), the caps
# against the cradle's ends (front / rear), a rib in the 2 mm gap between the two servo cases (up / down).
if "neck_cover" in bpy.data.objects and not args.get("neck", False):       # dropped on the user's decision (2026-11-12): it cost too much neck / head range
    bpy.data.objects.remove(bpy.data.objects["neck_cover"], do_unlink=True)
if args.get("neck", False):
    for n_ in ("neck_cover",):
        if n_ in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects[n_], do_unlink=True)
    cradle = bpy.data.objects["neck_cradle"]
    Pcr = lay(cradle)
    cx0, cx1 = Pcr[:, 0].min(), Pcr[:, 0].max()
    y_floor = Pcr[:, 1].min()                                                  # the floor plate's right face
    sn, sh = lay(bpy.data.objects["scs_neck_pitch_case"]), lay(bpy.data.objects["scs_head_pitch_case"])
    y_tower = max(sn[:, 1].max(), sh[:, 1].max())                              # tower tops (left)
    z_gap = (sn[:, 2].max(), sh[:, 2].min())                                   # the gap between the two cases
    wires = [o for o in bpy.data.collections["wiring"].objects if o.type == "MESH" and o.name.startswith(("wire_hub_N", "wire_hub_C", "wire_plug_"))]

    def group(test):
        P = [lay(o) for o in wires if o.name.startswith(("wire_hub_N", "wire_hub_C")) or True]
        P = [q for q in P if test(q) and q[:, 2].min() > 120 and q[:, 2].max() < 170 and q[:, 1].min() > y_floor - 6 and q[:, 1].max() < y_tower + 2]
        return np.concatenate(P)

    PN = group(lambda q: q[:, 0].min() > cx1 - 0.5 and q[:, 0].max() < cx1 + 14)
    PC = group(lambda q: q[:, 0].max() < cx0 + 0.5 and q[:, 0].min() > cx0 - 14)
    G, W = 0.8, 1.0
    yl = max(PN[:, 1].max(), PC[:, 1].max()) + G                               # caps' left inner face
    yr = min(PN[:, 1].min(), PC[:, 1].min()) - G                               # right inner face
    adds, cuts = [], []
    spans = {}
    for tag, P, sgn in (("front", PN, 1), ("rear", PC, -1)):
        xe = (P[:, 0].max() + G) if sgn > 0 else (P[:, 0].min() - G)           # cavity's far face
        xc = (cx1 + 0.8) if sgn > 0 else (cx0 - 0.8)                           # cavity's face toward the cradle
        z0, z1 = P[:, 2].min() - G, P[:, 2].max() + G
        xa, xb = sorted((xc, xe + sgn * W))
        adds.append(box(new(), xa, xb, yr - W, yl + W, z0 - W, z1 + W, ML))
        ca, cb = sorted((xc - sgn * 3.0, xe))
        cuts.append(box(new(), ca, cb, yr, yl, z0 - W - 1.0, z1, ML))          # the room, open toward the cradle and underneath
        spans[tag] = (xc, xe + sgn * W, z0 - W, z1 + W)
        # snap tab behind the floor plate's right face
        ta, tb = sorted((xc, xc - sgn * 1.6))
        adds.append(box(new(), ta, tb, yr - W, y_floor - 0.25, z0 + 4.0, z1 - 4.0, ML))
    zs0, zs1 = max(spans["front"][2], spans["rear"][2], 143.0), min(spans["front"][3], spans["rear"][3], 158.0)
    ys0, ys1 = y_tower + 0.5, y_tower + 1.5                                    # strap, 0.5 off the towers
    adds.append(box(new(), spans["rear"][0], spans["front"][0], ys0, ys1, zs0, zs1, ML))                       # strap along the servos' left side
    for xa in (spans["front"][0], spans["rear"][0] - 1.0):
        adds.append(box(new(), xa, xa + 1.0, yl + W - 0.3, ys1, zs0, zs1, ML))                              # returns joining the strap to the caps' left walls
    adds.append(box(new(), cx0 + 2.0, cx1 - 2.0, ys0 - (y_tower - (min(sn[:, 1].max(), sh[:, 1].max()) - 3.8)) - 1.5, ys0 + 0.2, z_gap[0] + 0.3, z_gap[1] - 0.3, ML))   # rib into the gap between the two servo cases
    cuts.append(box(new(), spans["rear"][1] - 0.5, spans["rear"][0] - 0.2, yl - 7.0, yl + W + 0.5, spans["rear"][3] - 5.0, spans["rear"][3] + 0.5, ML))        # notch, rear cap top left: the cable on to the U's hub
    nc = L["build_part"]("neck_cover", [L["combine_cutters"](adds)], [L["combine_cutters"](cuts)], coll, mat_cover)
    finish("neck_cover", nc, cradle.parent)
    report.append(f"neck: caps x {spans['rear'][1]:.1f}..{spans['front'][1]:.1f}, y {yr - W:.1f}..{yl + W:.1f}; strap z {zs0:.1f}..{zs1:.1f} at y {ys0:.1f}..{ys1:.1f}")
print("covers: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
