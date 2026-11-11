"""XGO-Duck leg, joints v2 (exact SCS0009): thigh, shin, foot + foot bearing plate for one side.
Runs inside Blender after the servos are placed (scs_<side>_<joint> roots at the zero pose).

  thigh   type-C stator x2 (hip pitch + knee caps with saddle walls, one plate + bridge)
  shin    type-C rotor at the knee (trimmed horn, 16x22x4) + type-F stator for the ankle servo
          (floor with captive M3 nut, pads, saddle walls, cable channel)
  foot    type-F rotor: the original pad shape (scaled mesh) + full-cross horn ear (outboard)
  foot_bearing_plate   693ZZ on an M3x8 axle (inboard), in a slot in the pad's side, 2x M2x6 pan-head self-tappers into the pad
Hip parts (yaw connector, L-bracket, body plate) are left to leg_orig_style.py for now.

ARGS = {"side": "left", "collection": "leg_v2", "joint_lib": "/abs/parts/joint_v2.py",
        "base_lib": "/abs/parts/joint_orig_style.py", "dims": "/abs/parts/scs0009_dims.py",
        "foot_mesh": "duck_scs0009:left_ankle", "hardware": True, "jig": True}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
SIDE = args.get("side", "left")
COLL = args.get("collection", "leg_v2")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
prefix = f"{SIDE}_"
for o in [o for o in coll.objects if o.name.startswith(prefix) or o.name.startswith("horn_trim_jig")]:
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()

F = {j: bpy.data.objects[f"scs_{SIDE}_{j}"].matrix_world.copy() for j in ("hip_pitch", "knee", "ankle")}
mat_dark = L["material"]("leg_dark", (0.16, 0.18, 0.24, 1))
mat_white = L["material"]("leg_white", (0.93, 0.93, 0.9, 1))
mat_foot = L["material"]("leg_foot", (0.98, 0.72, 0.15, 1))


def fr(M, p_world_mm):
    return (M.inverted() @ (Vector(p_world_mm) * MM)) / MM


def wr(M, p_frame_mm):
    return (M @ (Vector(p_frame_mm) * MM)) / MM


def plate(M, z0, z1, outline_frame_mm):
    return L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in outline_frame_mm for z in (z0, z1)], M)


def disc_pts(cx, cy, r, n=48):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def proj(M, pts_world_mm):
    return [tuple(fr(M, p)[:2]) for p in pts_world_mm]


def box_w(x0, x1, y0, y1, z0, z1):
    return L["add_box"](L["new_bm"](), x0, x1, y0, y1, z0, z1)


def axis_pt(j):
    return wr(F[j], (0, 0, 0))


report = []


def finish(name, part):
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["manifold"] = ok
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    return part


# ------------------------------------------------------------------ thigh = hip-pitch cap + knee cap (one plate + bridge)
ca1, cs1 = L["cap_features"](F["hip_pitch"], include_plate=False)
ca2, cs2 = L["cap_features"](F["knee"], include_plate=False)


def cap_rect(j):
    ring = [(L["CAP_RING_R"] * math.cos(2 * math.pi * i / 48), L["CAP_RING_R"] * math.sin(2 * math.pi * i / 48), 0) for i in range(48)]
    return [wr(F[j], (x, y, 0)) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])] + [wr(F[j], p) for p in ring]


hp_plate = plate(F["hip_pitch"], 0.0, L["CAP_T"], proj(F["hip_pitch"], cap_rect("hip_pitch")))
kn_plate = plate(F["hip_pitch"], 0.02, L["CAP_T"] - 0.02, proj(F["hip_pitch"], cap_rect("knee")))
kn_top = wr(F["knee"], (L["J_EAR_X"][1], 0, 0))
hp_far = wr(F["hip_pitch"], (L["J_EAR_X"][1], 0, 0))
d = (hp_far - kn_top).normalized()
nrm = d.cross(wr(F["hip_pitch"], (0, 0, 1)) - wr(F["hip_pitch"], (0, 0, 0))).normalized()
bridge_pts = [kn_top + nrm * 4.5, kn_top - nrm * 4.5, hp_far + nrm * 4.5, hp_far - nrm * 4.5]
bridge = plate(F["hip_pitch"], 0.04, L["CAP_T"] - 0.04, proj(F["hip_pitch"], bridge_pts))
# two holes through the bridge, 7 mm apart: a 2.5 mm zip tie through them holds the cable bundle
tie = []
bmid = (kn_top + hp_far) / 2
for sgn in (-1, 1):
    px, py, _ = fr(F["hip_pitch"], bmid + d * 3.5 * sgn)
    tie.append(L["add_cyl"](L["new_bm"](), px, py, -1.0, L["CAP_T"] + 1.0, 3.0, 14, F["hip_pitch"]))
# knee retention: one bayonet hook on the cap plate, opposite the shin's strip (the shin goes on with the knee bent 116 deg
# backwards, outside its +-90 deg range; checked clear of the thigh, its servos and hubs at that angle)
an_k = fr(F["knee"], axis_pt("ankle"))
KNEE_PHI = math.degrees(math.atan2(an_k[1], an_k[0])) + 180.0
KNEE_ASM = args.get("knee_assemble_deg", -116.0 if SIDE == "left" else 116.0)
KNEE_RANGE = args.get("knee_range_deg", (-90.0, 90.0))
RET = args.get("retention", "none")                                   # "clip" (separate keeper, screwed on) | "bayonet" (hook on the thigh) | "none"
hook = L["bayonet_hook"](F["knee"], KNEE_PHI) if RET == "bayonet" else []
seat = L["keeper_seat_cuts"](F["knee"], KNEE_PHI) if RET == "clip" else []
hpo, kno = axis_pt("hip_pitch"), axis_pt("knee")


def yT(zs):
    """world y of the plane at servo-frame z = zs (0 = tower top / plate face, -25.25 = servo bottom; same for both thigh servos)"""
    return wr(F["hip_pitch"], (0, 0, zs)).y


# web with the ORIGINAL thigh's side outline (rounded triangle), 2 mm, on the print-bed side of the plate
web = []
_om = bpy.data.objects.get(args.get("thigh_mesh", f"duck_scs0009:{SIDE}_hip_pitch"))
if _om is not None and args.get("thigh_outline", True):
    from mathutils import geometry
    _ov = [_om.matrix_world @ v.co / MM for v in _om.data.vertices]
    _hull = [_ov[i] for i in geometry.convex_hull_2d([Vector((v.x, v.z)) for v in _ov])]
    THIGH_OUTLINE = [(v.x, v.z) for v in _hull]
    web_full = plate(F["hip_pitch"], L["CAP_T"] - 2.0, L["CAP_T"] - 0.03, proj(F["hip_pitch"], [(x, hpo.y, z) for x, z in THIGH_OUTLINE]))
    # Relief: unlike the original (its shin is a plate that stays inboard of the thigh), OUR shin carries the ankle servo's cradle in
    # the thigh plate's plane, and from ~68 deg of forward knee bend that cradle swings through the lower front of the triangle.
    # Cut what it sweeps up to 90 deg (cradle: from 0.6 mm inside the ear tip outward, half-width CAP_HW + 0.3).
    ank = axis_pt("ankle")
    r_in = math.hypot(ank.x - kno.x, ank.z - kno.z) - max(L["J_EAR_X"]) - 0.6           # distance in the side view (the two axes are not in one plane)
    Wc = L["CAP_HW"] + 0.3

    def cradle_at(deg):
        t = math.radians(deg)
        u, n = Vector((math.sin(t), -math.cos(t))), Vector((math.cos(t), math.sin(t)))       # along the shin (forward for +), across it
        return [Vector((kno.x, kno.z)) + u * r + n * w for r in (r_in, 80.0) for w in (-Wc, Wc)]

    web_cuts = []
    for q, (d0, d1) in enumerate(((50, 60), (60, 70), (70, 80), (80, 90))):
        pts = cradle_at(d0) + cradle_at(d1)
        web_cuts.append(plate(F["hip_pitch"], L["CAP_T"] - 2.5 - 0.01 * q, L["CAP_T"] + 0.5 + 0.01 * q, proj(F["hip_pitch"], [(v.x, hpo.y, v.y) for v in pts])))
    web = [L["combine_cutters"]([web_full], web_cuts)]
    report.append(f"thigh web: original outline x {min(x for x, _ in THIGH_OUTLINE):.1f}..{max(x for x, _ in THIGH_OUTLINE):.1f}, z {min(z for _, z in THIGH_OUTLINE):.1f}..{max(z for _, z in THIGH_OUTLINE):.1f}")
# shelf for the kit hub on the thigh's top: level with the hip-pitch servo's top face, behind that servo's rear end, over the knee
# servo's upper end (the hub's place: wiring.py face_hub C). A wall when printed (the thigh prints on its plate).
shelf_top = hpo.z + L["J_BODY_HW"]
hp_rear = min(wr(F["hip_pitch"], (x, 0, 0)).x for x in L["J_EAR_X"])               # rear ear tip of the hip-pitch servo
ya_, yb_ = yT(0.3), yT((L["J_BOTTOM"] + L["J_EAR_Z"][0]) / 2 - 1.2 - 10.3)
top_shelf = [box_w(kno.x - 5.7, hp_rear - 0.3, min(ya_, yb_), max(ya_, yb_), shelf_top - 1.6, shelf_top)] if args.get("top_shelf", True) else []
# two posts for the OPTIONAL cable cover (they stand on the web, out to the plane just beyond the plugged hubs; positions clear
# of both legs' hub layouts, see wiring.py plane_hub): rear top corner behind the knee servo, and on the long front edge
CV_H = 2.3 + 0.6 + 6.0 + 0.35                                          # one layer of hubs (1.5 mm foam tape + PCB + socket) + gap, beyond the servo's end face
CV_POSTS = [(kno.x - 8.7, kno.z + 24.2), (hpo.x + 0.8, hpo.z - 10.0)]
POST_DS, CV_OFF, CV_T = (5.0, 4.2), 4.1, 1.0                           # the front post stands in a 4.4 mm wedge between hub A and the outline
ZS_TIP = L["J_BOTTOM"] - CV_H
posts, post_pilots = [], []
if web and args.get("thigh_posts", False):          # off: the cover has to be reshaped for the hubs on the thigh's top and rear
    for i_p, (px_, pz_) in enumerate(CV_POSTS):
        fx, fy = proj(F["hip_pitch"], [(px_, hpo.y, pz_)])[0]
        posts.append(L["add_cyl"](L["new_bm"](), fx, fy, ZS_TIP, L["CAP_T"] - 1.0 - 0.01 * i_p, POST_DS[i_p], 28, F["hip_pitch"]))
        post_pilots.append(L["add_cyl"](L["new_bm"](), fx, fy, ZS_TIP - 0.5, ZS_TIP + 5.5, L["PILOT_D"], 16, F["hip_pitch"]))
thigh = finish("thigh", L["build_part"](f"{prefix}thigh", [hp_plate, kn_plate, bridge] + web + top_shelf + posts + ca1 + ca2 + hook, cs1 + cs2 + tie + seat + post_pilots, coll, mat_dark))
if posts and args.get("thigh_cover", True):
    # OPTIONAL cover: flat plate with the original outline grown by CV_OFF (both hubs with their plugs need 2 - 2.5 mm more than
    # the original outline) + a rim that reaches 2 mm over the servos' end faces; 2x M2x6 pan-head self-tappers into the posts
    def grown(off, zs0, zs1):
        pts = [(x + off * math.cos(2 * math.pi * q / 16), z + off * math.sin(2 * math.pi * q / 16)) for x, z in THIGH_OUTLINE for q in range(16)]
        return plate(F["hip_pitch"], min(zs0, zs1), max(zs0, zs1), proj(F["hip_pitch"], [(x, hpo.y, z) for x, z in pts]))

    # closed on the OUTER side (plate), the TOP and the REAR (walls down to the thigh plate, as the original housing); along the
    # long front edge and round the knee only a short lip: the shin's ankle end passes 16.65 mm from the knee axis at full bend
    ZS_DEEP, ZS_LIP, RIM_T = -0.3, L["J_BOTTOM"] - 1.5, 0.8                # the lip stops 1.5 mm short of the servos' end faces: the foot's horn ear passes there
    shell = grown(CV_OFF, ZS_TIP - CV_T, ZS_DEEP)
    room = grown(CV_OFF - RIM_T, ZS_TIP, ZS_DEEP + 1.0)
    hyp = max(((THIGH_OUTLINE[q], THIGH_OUTLINE[(q + 1) % len(THIGH_OUTLINE)]) for q in range(len(THIGH_OUTLINE))),
              key=lambda e: (Vector(e[1]) - Vector(e[0])).length if (e[1][0] - e[0][0]) * (e[1][1] - e[0][1]) > 0 else 0.0)   # the long sloping front edge
    hA, hB = Vector(hyp[0]), Vector(hyp[1])
    hd = (hB - hA).normalized()
    hn = Vector((-hd.y, hd.x))
    if hn.dot(Vector((kno.x, kno.z + 20.0)) - hA) < 0:
        hn = -hn                                                       # into the thigh
    cut_pts = [hA + hd * a + hn * b for a in (-150.0, 150.0) for b in (-150.0, 14.0)]
    cut_front = plate(F["hip_pitch"], ZS_LIP, ZS_DEEP + 2.0, proj(F["hip_pitch"], [(q.x, hpo.y, q.y) for q in cut_pts]))
    cut_knee = plate(F["hip_pitch"], ZS_LIP + 0.01, ZS_DEEP + 2.01, proj(F["hip_pitch"], [(x, hpo.y, z) for x in (-150.0, 150.0) for z in (-100.0, kno.z + 9.0)]))
    cv_holes = [L["add_cyl"](L["new_bm"](), *proj(F["hip_pitch"], [(px_, hpo.y, pz_)])[0], ZS_TIP - CV_T - 0.5, ZS_TIP + 0.5, L["CLEAR_D"], 16, F["hip_pitch"]) for px_, pz_ in CV_POSTS]
    cover = finish("thigh_cover", L["build_part"](f"{prefix}thigh_cover", [shell], [room, cut_front, cut_knee] + cv_holes, coll, L["material"]("leg_cover", (0.85, 0.87, 0.9, 1))))
    cover["optional"] = True
    CV_SCREWS = [(F["hip_pitch"], *proj(F["hip_pitch"], [(px_, hpo.y, pz_)])[0], ZS_TIP - CV_T, +1) for px_, pz_ in CV_POSTS]

# ------------------------------------------------------------------ shin = ONE 4 mm plate: knee rotor (type C) at the top,
# ankle floor (type F) at the bottom. The ankle servo is slid along its axis (snap_servo_axes.py) so its
# floor lies in the knee plate's plane: the inboard face is flat (print bed), everything grows outboard.
kn, an = axis_pt("knee"), axis_pt("ankle")
kp0, kp1 = wr(F["knee"], (0, 0, L["PLATE_Z0"])).y, wr(F["knee"], (0, 0, L["PLATE_Z1"])).y
fl0, fl1 = wr(F["ankle"], (0, 0, L["FLOOR_Z"][1])).y, wr(F["ankle"], (0, 0, L["FLOOR_Z"][0])).y
if abs(kp0 - fl0) > 0.05 or abs(kp1 - fl1) > 0.05:
    raise RuntimeError(f"shin plates not coplanar: knee plate y {kp0:.2f}..{kp1:.2f}, ankle floor y {fl0:.2f}..{fl1:.2f} "
                       f"-> slide the ankle servo by {kp0 - fl0:+.2f} mm (snap_servo_axes.py axial_mm)")
# ankle cradle footprint (ankle frame x: ear tip to ear tip; y: cap half-width) in the knee frame's plate plane
cradle = [wr(F["ankle"], (x, y, L["FLOOR_Z"][1])) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])]
top_edge = max(p.z for p in cradle)                                  # upper edge of the cradle rectangle (world z)
strip = [(kn.x - 7, kn.y, top_edge - 1.5), (kn.x + 7, kn.y, top_edge - 1.5)]   # 1.5 mm into the cradle plate: fused
la, ls, lr = L["link_features"](F["knee"], proj(F["knee"], strip))   # knee ring + 14 mm strip down to the cradle (20 clips the hip-pitch cradle at knee +90)
cradle_plate = plate(F["knee"], L["PLATE_Z0"] + 0.02, L["PLATE_Z1"] - 0.02, proj(F["knee"], cradle))
fa, fs = L["floor_features"](F["ankle"], include_floor=False, saddle=args.get("ankle_saddle", True), cable=args.get("cable", True))
if RET == "bayonet":
    ls += L["bayonet_rim_cuts"](F["knee"], KNEE_PHI, KNEE_RANGE[0], KNEE_RANGE[1], KNEE_ASM)
elif RET == "clip":
    ls += L["bayonet_rim_cuts"](F["knee"], KNEE_PHI, KNEE_RANGE[0], KNEE_RANGE[1], 0.0, notch=False)
shin = finish("shin", L["build_part"](f"{prefix}shin", la + [L["combine_cutters"]([cradle_plate] + fa)], ls + fs, coll, mat_white, lr))   # cradle = plate + pads + walls unioned first (simple shapes: robust)

# ------------------------------------------------------------------ foot = the ORIGINAL pad (scaled mesh: outline, rounded
# edges, 5 deg canted sole, top face) with the original ears replaced by ours: horn ear (outboard, one piece with
# the pad) and a separate bearing plate (inboard) let into a slot in the pad's side. The pad top is cut only where
# the shin + servo sweep over the ankle range (envelope computed from the built shin, not assumed).
import numpy as np

fm = bpy.data.objects[args["foot_mesh"]]
an = axis_pt("ankle")
ax_w = (F["ankle"].to_3x3() @ Vector((0, 0, 1))).normalized()          # ankle axis, toward the horn (outboard)
if abs(abs(ax_w.y) - 1.0) > 1e-4:
    raise RuntimeError(f"ankle axis is not along world Y: {tuple(round(c, 4) for c in ax_w)}")
SY = 1.0 if ax_w.y > 0 else -1.0


def y_of(t):
    """world y (mm) of the plane at ankle-frame z = t"""
    return an.y + SY * t


pad = bmesh.new()
pad.from_mesh(fm.data)
pad.transform(fm.matrix_world)
bmesh.ops.remove_doubles(pad, verts=pad.verts[:], dist=2e-6)           # STL import: weld coincident vertices
# (the original mesh has open edges, all in its ears above the pad top; the pad is checked closed after they are cut off)
pad.normal_update()
tops = {}
for f in pad.faces:                                                    # pad top = the largest face group that looks straight up
    if f.normal.z > 0.999:
        k = round(f.calc_center_median().z / MM, 2)
        tops[k] = tops.get(k, 0.0) + f.calc_area()
PAD_TOP = max(tops, key=tops.get) - 0.05                               # 0.05 under the original top: no cut coplanar with it
zmin = min(v.co.z for v in pad.verts) / MM
xs_pad = [v.co.x / MM for v in pad.verts]
ys_pad = [v.co.y / MM for v in pad.verts]
bmesh.ops.bisect_plane(pad, geom=pad.verts[:] + pad.edges[:] + pad.faces[:], plane_co=Vector((0, 0, PAD_TOP * MM)),
                       plane_no=Vector((0, 0, 1)), clear_outer=True)
bmesh.ops.holes_fill(pad, edges=[e for e in pad.edges if e.is_boundary], sides=0)
bmesh.ops.recalc_face_normals(pad, faces=pad.faces[:])
if any(not e.is_manifold for e in pad.edges):
    raise RuntimeError("pad did not close after removing the original ears")

# ---- swept envelope of everything on the shin side, over the ankle range (+-90 deg as the original model)
ANKLE_RANGE = args.get("ankle_range_deg", (-90.0, 90.0))
SWEEP_CLEAR = 0.5
T0, T1 = L["FLOOR_Z"][0] - 0.5, 0.2                                    # axial extent of the shin side (floor underside .. tower top)
sw_names = [shin.name] + [f"scs_{SIDE}_ankle_{k}" for k in ("case", "case_screws", "spline")] + [f"cable_stub_{SIDE}_ankle"]
samples = []
for nme in sw_names:
    ob = bpy.data.objects.get(nme)
    if ob is None:
        report.append(f"foot: sweep source {nme} missing")
        continue
    # points every <= 0.4 mm over every FACE (a slice between two ends of a flat wall has no vertex or edge in it)
    tb = bmesh.new()
    tb.from_mesh(ob.data)
    tb.transform(ob.matrix_world)
    bmesh.ops.triangulate(tb, faces=tb.faces[:])
    T = np.array([[v.co[:] for v in f.verts] for f in tb.faces]) / MM          # (n, 3, 3)
    tb.free()
    longest = np.max(np.linalg.norm(T - np.roll(T, 1, axis=1), axis=2), axis=1)
    n_div = np.maximum(1, np.ceil(longest / 0.4).astype(int))
    parts = []
    for n in np.unique(n_div):
        Tn = T[n_div == n]
        for a in range(n + 1):
            for b in range(n + 1 - a):
                parts.append(Tn[:, 0] + (Tn[:, 1] - Tn[:, 0]) * (a / n) + (Tn[:, 2] - Tn[:, 0]) * (b / n))
    samples.append(np.vstack(parts))
samples = np.vstack(samples)
s_dx, s_dz, s_t = samples[:, 0] - an.x, samples[:, 2] - an.z, (samples[:, 1] - an.y) * SY
GX0, GSTEP, GN = -45.0, 0.5, 181
CAP_Z = (PAD_TOP - an.z) + 2.5                                         # cutters are flat above the pad top
thetas = np.radians(np.arange(ANKLE_RANGE[0], ANKLE_RANGE[1] + 0.5, 1.0))
sweep_cuts, trough_min = [], 0.0
n_sl = 9
for i_sl in range(n_sl):
    ta, tb = T0 + (T1 - T0) * i_sl / n_sl, T0 + (T1 - T0) * (i_sl + 1) / n_sl
    k = (s_t >= ta - SWEEP_CLEAR) & (s_t <= tb + SWEEP_CLEAR) & (np.hypot(s_dx, s_dz) < 60.0)
    h = np.full(GN, 99.0)
    if k.any():
        r, phi = np.hypot(s_dx[k], s_dz[k]), np.arctan2(s_dx[k], s_dz[k])
        for th in thetas:
            x, z = r * np.sin(phi + th), r * np.cos(phi + th)
            ix = np.round((x - GX0) / GSTEP).astype(int)
            ok = (ix >= 0) & (ix < GN)
            np.minimum.at(h, ix[ok], z[ok])
    hc = np.minimum(np.minimum(h, np.roll(h, 1)), np.roll(h, -1)) - SWEEP_CLEAR      # +-0.5 sideways, 0.5 down
    hc = np.minimum(hc, CAP_Z)
    trough_min = min(trough_min, float(hc.min()))
    if hc.min() > (PAD_TOP - an.z) - 0.01:
        continue                                                       # nothing reaches the pad in this slice
    keep = [0] + [q for q in range(1, GN - 1) if abs(hc[q] - hc[q - 1]) > 1e-6 or abs(hc[q] - hc[q + 1]) > 1e-6] + [GN - 1]
    chain = [(GX0 + GSTEP * q, float(hc[q])) for q in keep]
    ya, yb = y_of(ta - 0.011 * (i_sl + 1)), y_of(tb + 0.3 + 0.007 * i_sl)             # overlaps the next slice; distinct end planes
    cb = bmesh.new()                                                   # a closed grid of planar quads (no concave n-gons)
    rows = [[[cb.verts.new(Vector((an.x + px, yy, an.z + pz)) * MM) for px, pz in line] for yy in (ya, yb)]
            for line in (chain, [(px, CAP_Z + 20.0) for px, _ in chain])]
    (b0, b1), (t0, t1) = rows
    for q in range(len(chain) - 1):
        cb.faces.new((b0[q], b0[q + 1], b1[q + 1], b1[q]))             # underside (the envelope)
        cb.faces.new((t0[q], t0[q + 1], t1[q + 1], t1[q]))             # top
        cb.faces.new((b0[q], b0[q + 1], t0[q + 1], t0[q]))             # end caps
        cb.faces.new((b1[q], b1[q + 1], t1[q + 1], t1[q]))
    for q in (0, len(chain) - 1):
        cb.faces.new((b0[q], b1[q], t1[q], t0[q]))                     # the two short sides
    bmesh.ops.recalc_face_normals(cb, faces=cb.faces[:])
    if any(not e.is_manifold for e in cb.edges) or cb.calc_volume() <= 0:
        raise RuntimeError(f"sweep cutter {i_sl} is not a closed solid")
    sweep_cuts.append(cb)
report.append(f"foot: pad top z {PAD_TOP:.2f} ({an.z - PAD_TOP:.2f} under the ankle axis), sweep trough down to {an.z + trough_min:.2f} "
              f"(radius {-trough_min:.2f}), {len(sweep_cuts)} slices, {len(samples)} samples")

# ---- horn ear (outboard): full-cross horn plate + a lug that joins it to the pad's side wall
EAR_HX, EAR_Z0 = 14.0, 11.0
reach = [(an.x - EAR_HX, an.y, EAR_Z0), (an.x + EAR_HX, an.y, EAR_Z0)]
ha, hs, hr = L["horn_plate_features"](F["ankle"], proj(F["ankle"], reach), angle_deg=45.0)
yo, yw, yi = y_of(L["ARM_Z"][1]), y_of(L["ARM_Z"][0] - 0.75), y_of(L["ARM_Z"][0] - 3.05)
lug_dz = abs(yo - yw) * 1.43                                           # 55 deg underside: prints without support, stays off the ground
lug = L["add_hull"](L["new_bm"](), [(an.x + sx * EAR_HX, y, z) for sx in (-1, 1)
                                    for y, z in ((yo, EAR_Z0), (yw, EAR_Z0 - lug_dz), (yi, EAR_Z0 - lug_dz + 1.5), (yi, PAD_TOP - 0.03), (yo, PAD_TOP - 0.03))])
# the horn (it turns with the foot) and the servo's tower top inside the pad's outboard wall
horn_room = L["add_cyl"](L["new_bm"](), 0, 0, -0.4, L["ARM_Z"][0] - 0.02, 2 * (L["HORN_CROSS_L"] + 0.7), 64, F["ankle"])

# ---- bearing plate (inboard): 693ZZ seat; its lower end sits in a slot in the pad's side, 2x M2x6 pan-head
# self-tappers along the axle direction (heads in 2.3 mm counterbores -> 3.5 mm of thread in the pad)
BP_Z0, BP_Z1 = L["BP_INNER_Z"] - L["BP_T"], L["BP_INNER_Z"]
BP_HX, SLOT_FLOOR, BP_CBORE, BP_SCREW_Z = 9.0, zmin + 3.6, 2.3, 7.4
slot = box_w(an.x - BP_HX - 0.15, an.x + BP_HX + 0.15, min(y_of(BP_Z0 - 2.0), y_of(BP_Z1 + 0.1)), max(y_of(BP_Z0 - 2.0), y_of(BP_Z1 + 0.1)),
             SLOT_FLOOR, PAD_TOP + 5.0)
scr_xy = [tuple(fr(F["ankle"], (an.x + dx, an.y, BP_SCREW_Z))[:2]) for dx in (-5.5, 5.5)]
pad_pilots = L["screw_holes"](F["ankle"], scr_xy, BP_Z1, 0.0, 4.0, +1, csk=False)
if args.get("foot_debug"):
    L["B"]["PROBE"].update(point=Vector((an.x + 20, an.y + SY * -15, zmin + 6)) * MM, part=f"{prefix}foot")
foot = finish("foot", L["build_part"](f"{prefix}foot", [pad, lug] + ha, sweep_cuts + [horn_room, slot] + hs + pad_pilots, coll, mat_foot, hr))

plate_pts = [(an.x - BP_HX, an.y, SLOT_FLOOR + 0.1), (an.x + BP_HX, an.y, SLOT_FLOOR + 0.1), (an.x - BP_HX, an.y, PAD_TOP), (an.x + BP_HX, an.y, PAD_TOP)]
ba, bs = L["bearing_plate_features"](F["ankle"], proj(F["ankle"], plate_pts))
bs += L["screw_holes"](F["ankle"], scr_xy, BP_Z0, L["BP_T"], 0.0, +1, csk=False, cbore=BP_CBORE)
foot_bp = finish("foot_bearing_plate", L["build_part"](f"{prefix}foot_bearing_plate", ba, bs, coll, mat_foot))
M_scr = [(F["ankle"], x, y, BP_Z0 + BP_CBORE, +1) for x, y in scr_xy]

# ------------------------------------------------------------------ hips (type C, as v1 but with the v2 features)
if args.get("hips", True):
    for j in ("hip_yaw", "hip_roll"):
        F[j] = bpy.data.objects[f"scs_{SIDE}_{j}"].matrix_world.copy()
    hr, hp, hy = axis_pt("hip_roll"), axis_pt("hip_pitch"), axis_pt("hip_yaw")

    # l_bracket = hip-roll rotor plate + hip-pitch rotor plate meeting at a corner
    def far_face(j, coord, toward):
        a = getattr(wr(F[j], (0, 0, L["PLATE_Z0"])), coord)
        b = getattr(wr(F[j], (0, 0, L["PLATE_Z1"])), coord)
        return a if abs(a - toward) > abs(b - toward) else b

    # inboard flats: at 0.92x the two legs' roll joints sit so close that their round bearing rings meet when
    # both hips yaw outward. Flat-cut the inboard side of the roll cap (0.8 mm wall left at the bearing seat)
    # and of the L-bracket's ring (to just outside the Ø17.6 shoulder).
    other = bpy.data.objects[f"scs_{'right' if SIDE == 'left' else 'left'}_hip_roll"].matrix_world.translation / MM
    s_in = 1.0 if fr(F["hip_roll"], other).y > 0 else -1.0

    def inboard_flat(y_keep, z0, z1):
        ys_ = sorted((s_in * y_keep, s_in * 40.0))
        return L["add_box"](L["new_bm"](), -40.0, 40.0, ys_[0], ys_[1], z0, z1, F["hip_roll"])

    FLAT_CAP = args.get("flat_cap", L["SEAT_D"] / 2 + 0.8)
    FLAT_LINK = args.get("flat_link", L["SHOULDER_D"] / 2 + 0.5)
    corner_x = far_face("hip_roll", "x", hp.x)
    corner_y = far_face("hip_pitch", "y", hr.y)
    ra, rs, rr = L["link_features"](F["hip_roll"], proj(F["hip_roll"], [(corner_x, corner_y, hr.z - 7), (corner_x, corner_y, hr.z + 7)]))
    pa, ps, pr = L["link_features"](F["hip_pitch"], proj(F["hip_pitch"], [(corner_x, corner_y, hp.z - 7), (corner_x, corner_y, hp.z + 7)]))
    if FLAT_LINK:
        ra[0] = L["combine_cutters"]([ra[0]], [inboard_flat(FLAT_LINK, L["PLATE_Z0"] - 1.0, L["PLATE_Z1"] + 1.0)])
    finish("l_bracket", L["build_part"](f"{prefix}l_bracket", ra + pa, rs + ps, coll, mat_white, rr + pr))

    # yaw_connector = hip-yaw rotor plate + hip-roll cap (saddle walls optional: they hang beside the yaw servo)
    YAW_HW = args.get("yaw_hw", 8.5)   # strip half-width: 2.7 mm rails either side of the ear channel below
    ya, ys, yr = L["link_features"](F["hip_yaw"], proj(F["hip_yaw"], [(wr(F["hip_roll"], (0, 0, 0)).x + L["CAP_T"], hr.y - YAW_HW, 0),
                                                                       (wr(F["hip_roll"], (0, 0, 0)).x + L["CAP_T"], hr.y + YAW_HW, 0)]))
    # the roll servo's shaft-end ear reaches 2.6 mm into the yaw plate's underside: channel for it along the
    # servo's insertion path (roll -Z), 0.3 clearance; 1.1 mm of plate + the boss stay above it
    ch = args.get("ear_ch", {})
    ear_ch = L["add_box"](L["new_bm"](), L["J_EAR_X"][0] - 0.35, L["J_BODY_X"][0] + ch.get("x1", 0.3), -L["J_EAR_HW"] - ch.get("side", 0.3),
                          L["J_EAR_HW"] + ch.get("side", 0.3), ch.get("z0", -45.0), L["J_EAR_Z"][1] + ch.get("top", 0.02), F["hip_roll"])
    if args.get("ear_channel", True):
        ya[0] = L["combine_cutters"]([ya[0]], [ear_ch])      # cut the simple plate first (robust), then union the rest
    rca, rcs = L["cap_features"](F["hip_roll"], saddle=args.get("roll_saddle", True))
    if FLAT_CAP:
        rca[0] = L["combine_cutters"]([rca[0]], [inboard_flat(FLAT_CAP, -1.0, L["CAP_T"] + 1.0)])
    trim = L["add_annulus"](L["new_bm"](), 9.2, 45.0, -20.0, L["PLATE_Z0"] + 0.01, F["hip_yaw"])   # nothing above the yaw plate but the boss (0.01 into the plate top: no shared plane)
    finish("yaw_connector", L["build_part"](f"{prefix}yaw_connector", ya + rca, ys + rcs + [trim], coll, mat_dark, yr))

    # hip_yaw_support (body plate) = both hip-yaw caps
    if args.get("with_support"):
        other = "right" if SIDE == "left" else "left"
        Fo = bpy.data.objects[f"scs_{other}_hip_yaw"].matrix_world.copy()
        for o in [o for o in coll.objects if o.name.startswith("hip_yaw_support")]:
            bpy.data.objects.remove(o, do_unlink=True)
        c1, s1 = L["cap_features"](F["hip_yaw"], include_plate=False, saddle=args.get("yaw_saddle", True))
        c2, s2 = L["cap_features"](Fo, include_plate=False, saddle=args.get("yaw_saddle", True))
        cs = [wr(M, (x, y, 0)) for M in (F["hip_yaw"], Fo) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])]
        sp = plate(F["hip_yaw"], 0.0, L["CAP_T"], proj(F["hip_yaw"], cs))
        finish("hip_yaw_support", L["build_part"]("hip_yaw_support", [sp] + c1 + c2, s1 + s2, coll, mat_dark))

if args.get("hardware", True):
    # the bearing plate's two M2x6 pan-head self-tappers: along the axle direction, through the plate (2.5 under the head) into the pad (3.5)
    L["pan_screw_dummy"](f"{prefix}foot_plate_screws_M2x6st", M_scr, coll)
    if posts and args.get("thigh_cover", True):
        L["pan_screw_dummy"](f"{prefix}thigh_cover_screws_M2x6st", CV_SCREWS, coll)
    L["hardware_C"](f"{prefix}knee_", F["knee"], coll)
    L["hardware_C"](f"{prefix}hip_pitch_", F["hip_pitch"], coll)
    L["hardware_F"](f"{prefix}ankle_", F["ankle"], coll)
    if args.get("hips", True):
        L["hardware_C"](f"{prefix}hip_roll_", F["hip_roll"], coll)
        L["hardware_C"](f"{prefix}hip_yaw_", F["hip_yaw"], coll)
if args.get("jig", True):
    Mj = Matrix.Translation(Vector(args.get("jig_location", (0.30, 0.30, 0.0))))
    L["trim_jig"]("horn_trim_jig_r6", coll, Mj)

print(f"{SIDE} leg v2: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
