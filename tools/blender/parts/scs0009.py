"""Feetech SCS0009 servo model — exact geometry. Runs inside Blender.

Geometry source: Feetech's own CAD, assembly SCS009-20201118-S (Pro/E, STEP exported 2023-01-10,
distributed by Feetech resellers), kept at vendor/feetech/SCS009-20230110-S.stp
(sha256 cdaa6e07...45167; fetched by scripts/fetch_vendor.sh). tools/cad/scs0009_from_step.py converts
its four solids (top case, core middle, bottom case, output gear) to the STLs in parts/scs0009_mesh/
(5 µm chordal tolerance) and measures every cylinder/plane in the B-rep; the constants in scs0009_dims.py are those
measurements, cross-checked against the FEETECH SCS0009 product specification A/0 (2020-11-23) §6/§9.
Not in the STEP: screws (modelled here from the hole features — positions exact, screw sizes
inferred), horns (kit lengths from Feetech's listing; thickness/hub NOMINAL until measured), motor/PCB.

Frame (the joint frame): origin on the output-shaft axis at the tower top face (25.25 mm above the
case bottom), +Z out of the shaft, +X along the body toward the connector end, +Y right-handed.
STEP frame -> joint frame: (X, Y, Z) = (z_step + 5.7, x_step, y_step - 23.8) mm.  Units: metres.

ARGS = {"name": "scs0009", "collection": "scs0009", "location": (x, y, z),
        "horn": "disc"|"arm"|"cross"|None, "horn_trim_r": mm | None, "horn_angle_deg": 0,
        "exact": True (use the STEP meshes; False = box envelope), "parts_dir": "/abs/tools/blender/parts" | None,
        "screws": True (4 case screws + horn screw), "ear_screws": None | thread length mm}
Objects: <name> (root empty = joint frame), <name>_case, <name>_spline (output gear, real teeth),
         <name>_horn_<type>, <name>_case_screws, <name>_horn_screw, <name>_ear_screws.
"""
import math
import os
import struct

import bmesh
import bpy
from mathutils import Matrix, Vector

MM = 0.001


def find_parts_dir():
    """tools/blender/parts of this repo: from ARGS, $AGENT_CAD3D_ROOT, the .blend's folder or the cwd."""
    a = globals().get("ARGS", {})
    if a.get("parts_dir"):
        return a["parts_dir"]
    if a.get("mesh_dir"):
        return os.path.dirname(a["mesh_dir"].rstrip("/"))
    cands = [os.environ.get("AGENT_CAD3D_ROOT"), os.path.dirname(bpy.data.filepath) if bpy.data.filepath else None,
             os.getcwd()]
    for c in cands:
        if not c:
            continue
        d = os.path.abspath(c)
        for _ in range(6):
            p = os.path.join(d, "tools", "blender", "parts")
            if os.path.isfile(os.path.join(p, "scs0009_dims.py")):
                return p
            d = os.path.dirname(d)
    raise FileNotFoundError("tools/blender/parts/scs0009_dims.py not found: pass ARGS['parts_dir'] or set AGENT_CAD3D_ROOT")


PARTS_DIR = find_parts_dir()
exec(open(os.path.join(PARTS_DIR, "scs0009_dims.py")).read())     # all dimensions (datasheet + joint frame)

args = globals().get("ARGS", {})
NAME = args.get("name", "scs0009")
COLL = args.get("collection", "scs0009")
LOC = Vector(args.get("location", (0, 0, 0)))
HORN = args.get("horn", "disc")
EXACT = args.get("exact", True)
SCREWS = args.get("screws", True)
EAR_SCREWS = args.get("ear_screws")

coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)


def mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


BLACK = mat("scs0009_case", (0.04, 0.04, 0.045, 1))
BRASS = mat("scs0009_spline", (0.75, 0.6, 0.25, 1))
WHITE = mat("scs0009_horn", (0.95, 0.95, 0.93, 1))
STEEL = mat("scs0009_screw", (0.55, 0.56, 0.58, 1))


def to_frame(x, y, z):
    """Datasheet mm -> joint frame metres."""
    return Vector(((x - SHAFT_X) * MM, y * MM, (z - TOWER_TOP) * MM))


def box(bm, x0, x1, y0, y1, z0, z1):
    out = bmesh.ops.create_cube(bm, size=1.0)
    for v in out["verts"]:
        v.co = to_frame(x0 if v.co.x < 0 else x1, y0 if v.co.y < 0 else y1, z0 if v.co.z < 0 else z1)


def cyl(bm, cx, cy, z0, z1, d, segs=32):
    out = bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=d / 2 * MM, radius2=d / 2 * MM,
                                depth=(z1 - z0) * MM)
    bmesh.ops.translate(bm, verts=out["verts"], vec=to_frame(cx, cy, (z0 + z1) / 2))
    return out


def screw(bm, cx, cy, z_head0, head_d, head_h, shank_d, z_tip):
    """Pan-head screw along -Z: head from z_head0 up head_h, shank from z_head0 down to z_tip (datasheet mm)."""
    cyl(bm, cx, cy, z_head0, z_head0 + head_h, head_d, 24)
    cyl(bm, cx, cy, z_tip, z_head0 + 0.01, shank_d, 16)


def replace_object(name, bm_or_mesh, material):
    old = bpy.data.objects.get(name)
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
    if isinstance(bm_or_mesh, bpy.types.Mesh):
        me = bm_or_mesh
    else:
        me = bpy.data.meshes.new(name)
        bm_or_mesh.normal_update()
        bm_or_mesh.to_mesh(me)
        bm_or_mesh.free()
    if material and material.name not in [m.name for m in me.materials if m]:
        me.materials.append(material)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


# ---------------------------------------------------------------- exact meshes (shared datablocks)
def find_mesh_dir():
    p = args.get("mesh_dir") or os.path.join(PARTS_DIR, "scs0009_mesh")
    return p if os.path.isdir(p) else None


def read_stl_joint_frame(path):
    """Binary STL in the STEP frame (mm) -> welded (verts, faces) in the joint frame (m)."""
    import numpy as np
    b = open(path, "rb").read()
    n = struct.unpack_from("<I", b, 80)[0]
    if len(b) != 84 + 50 * n:
        raise ValueError(f"{path}: not a binary STL")
    rec = np.frombuffer(b, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n, offset=84)
    V = rec["v"].reshape(-1, 3).astype(np.float64)
    W = np.stack([V[:, 2] + 5.7, V[:, 0], V[:, 1] - 23.8], axis=1) * MM      # STEP (x,y,z) -> joint (X,Y,Z)
    key = np.round(W / 1e-7).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return (uniq * 1e-7).tolist(), inv.reshape(-1, 3).tolist()


def shared_mesh(part, files, material):
    """One mesh datablock per part, rebuilt only when the STL files change."""
    stamp = ";".join(f"{os.path.basename(f)}:{os.path.getsize(f)}:{int(os.path.getmtime(f))}" for f in files)
    name = f"scs0009_mesh:{part}"
    me = bpy.data.meshes.get(name)
    if me is not None and me.get("src") == stamp:
        return me
    verts, faces = [], []
    for f in files:
        v, fc = read_stl_joint_frame(f)
        off = len(verts)
        verts += v
        faces += [[i + off for i in tri] for tri in fc]
    if me is not None:
        me.name = name + ".old"
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.validate(verbose=False)
    me.update()
    me.materials.append(material)
    me["src"] = stamp
    return me


# ---------------------------------------------------------------- root empty = joint frame
root = bpy.data.objects.get(NAME) or bpy.data.objects.new(NAME, None)
if root.name not in coll.objects:
    coll.objects.link(root)
root.empty_display_type = "ARROWS"
root.empty_display_size = 0.012
root.location = LOC
for k, v in {"model": "Feetech SCS0009", "stall_torque_Nm_6V": 0.2256, "rated_torque_Nm_6V": 0.0736,
             "no_load_speed_rad_s_6V": 10.47, "mass_kg": MASS_KG, "voltage_range_V": "4-7.4",
             "range_deg": 300, "resolution_deg": 0.293, "spline": "20T OD3.95 root3.45, teeth z 25.95-28.45",
             "horn_screw": "M2.0x4 self-tapping into a 1.6 x 5 hole", "boss": "D5.76 x 0.3 on the tower top (25.55)",
             "case_screws": "4 from the bottom at x 5.85/26.65, y +-4.8 (~D1.0 x 20)",
             "cable_exit": "slot in the shaft-end face, bottom, centred, ~3 wide x 2.3 tall x 1.7 deep",
             "roof": "flat 21.45 to x=20.84, then slopes 6.5 deg down to ~20.7 at the connector end",
             "source": "Feetech STEP SCS009-20201118-S (2023-01-10) + spec A/0 2020-11-23"}.items():
    root[k] = v

parts = []
mesh_dir = find_mesh_dir() if EXACT else None
if EXACT and mesh_dir is None:
    print(f"{NAME}: exact meshes not found (tools/blender/parts/scs0009_mesh) — falling back to the box envelope")

if mesh_dir:
    files = {p: os.path.join(mesh_dir, f"scs0009_{p}.stl") for p in ("topcase", "coremiddle", "bottomcase", "outputgear")}
    case = replace_object(f"{NAME}_case", shared_mesh("case", [files["topcase"], files["coremiddle"], files["bottomcase"]], BLACK), None)
    spline = replace_object(f"{NAME}_spline", shared_mesh("outputgear", [files["outputgear"]], BRASS), None)
    case["exact"] = spline["exact"] = True
else:
    # Box envelope: body to the case top (the sloping roof is ignored: conservative), round-ended tower, boss, ears.
    def solid(name, x0, x1, y0, y1, z0, z1):
        bm = bmesh.new()
        box(bm, x0, x1, y0, y1, z0, z1)
        return replace_object(name, bm, BLACK)

    def drill(ob, hx):
        bm = bmesh.new()
        cyl(bm, hx, 0, EAR_Z0 - 1, EAR_Z1 + 1, HOLE_D, 24)
        cutter = replace_object(f"{NAME}_tmp_cut", bm, BLACK)
        mod = ob.modifiers.new("hole", "BOOLEAN")
        mod.operation, mod.object, mod.solver = "DIFFERENCE", cutter, "EXACT"
        with bpy.context.temp_override(object=ob, active_object=ob):
            bpy.ops.object.modifier_apply(modifier=mod.name)
        bpy.data.objects.remove(cutter, do_unlink=True)

    case = solid(f"{NAME}_case", BODY_X0, BODY_X1, -BODY_W / 2, BODY_W / 2, 0.0, CASE_TOP)
    pieces = []
    bm = bmesh.new()
    cyl(bm, SHAFT_X, 0, CASE_TOP, TOWER_TOP, 2 * TOWER_R, 48)
    box(bm, SHAFT_X, TOWER_X1, -TOWER_R, TOWER_R, CASE_TOP, TOWER_TOP)
    cyl(bm, SHAFT_X, 0, TOWER_TOP, BOSS_TOP, BOSS_D, 32)
    box(bm, SHAFT_X, BOSS_TONGUE_X1, -BOSS_TONGUE_HW, BOSS_TONGUE_HW, TOWER_TOP, BOSS_TOP)
    pieces.append(replace_object(f"{NAME}_tower", bm, BLACK))
    for i, (x0, x1) in enumerate(((EAR_TIP_L, BODY_X0), (BODY_X1, EAR_TIP_R))):
        ear = solid(f"{NAME}_ear{i}", x0, x1, -EAR_W / 2, EAR_W / 2, EAR_Z0, EAR_Z1)
        drill(ear, HOLE_X[i])
        pieces.append(ear)
    with bpy.context.temp_override(active_object=case, selected_editable_objects=[case] + pieces):
        bpy.ops.object.join()
    bm = bmesh.new()
    cyl(bm, SHAFT_X, 0, NECK_Z0, SPLINE_Z0, NECK_D, 32)
    cyl(bm, SHAFT_X, 0, SPLINE_Z0, SPLINE_Z1, SPLINE_OD, SPLINE_TEETH)
    spline = replace_object(f"{NAME}_spline", bm, BRASS)
    case["exact"] = spline["exact"] = False
parts += [case, spline]

# ---------------------------------------------------------------- horn (kit lengths, nominal section)
# Lengths are shaft centre -> outermost point of the ROUND tip; the straight arm ends W_TIP/2 before it.
z_hub0 = BOSS_TOP                                  # hub sits on the boss
HORN_ANGLE = math.radians(args.get("horn_angle_deg", 0.0))
if HORN:
    bm = bmesh.new()
    z_plate0 = z_hub0 + HORN_HUB_H - HORN_PLATE_T
    cyl(bm, SHAFT_X, 0, z_hub0, z_hub0 + HORN_HUB_H, HORN_HUB_D, 40)
    if HORN == "disc":
        cyl(bm, SHAFT_X, 0, z_plate0, z_plate0 + HORN_PLATE_T, HORN_DISC_D, 64)
    else:
        arms = 1 if HORN == "arm" else 4
        reach = HORN_ARM_L if HORN == "arm" else HORN_CROSS_L
        trim = args.get("horn_trim_r")          # arms cut flat at this radius (fits inside a bearing bore)
        if trim and HORN != "arm":
            arm_l, w_end, round_tip = trim, HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * trim / HORN_CROSS_STRAIGHT, False   # the full horn, cut: same taper
        else:
            arm_l, w_end, round_tip = reach - HORN_ARM_W_TIP / 2, HORN_ARM_W_TIP, True
        for i in range(arms):  # tapered arm: shaft centre -> arm end, plus a round tip
            rot = Matrix.Rotation(HORN_ANGLE + i * 1.5707963267948966, 3, "Z")
            outline = [(0, -HORN_ARM_W_ROOT / 2), (arm_l, -w_end / 2),
                       (arm_l, w_end / 2), (0, HORN_ARM_W_ROOT / 2)]
            verts = []
            for z in (z_plate0, z_plate0 + HORN_PLATE_T):
                for ax, ay in outline:
                    p = rot @ Vector((ax, ay, 0))
                    verts.append(bm.verts.new(to_frame(SHAFT_X + p.x, p.y, z)))
            b, t = verts[:4], verts[4:]
            bm.faces.new(b[::-1])
            bm.faces.new(t)
            for k in range(4):
                bm.faces.new((b[k], b[(k + 1) % 4], t[(k + 1) % 4], t[k]))
            if round_tip:
                tip = rot @ Vector((arm_l, 0, 0))
                cyl(bm, SHAFT_X + tip.x, tip.y, z_plate0, z_plate0 + HORN_PLATE_T, HORN_ARM_W_TIP, 24)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)      # hand-built arm prisms: make every shell face outward
    horn = replace_object(f"{NAME}_horn_{HORN}", bm, WHITE)
    # detail as on the real horn (kit photo): hub bored for the spline and the screw, screw-head recess, a row of
    # small holes along each arm. One boolean on the finished horn (use_self: its arms and hub overlap).
    cut = bmesh.new()
    top = z_hub0 + HORN_HUB_H
    cyl(cut, SHAFT_X, 0, z_hub0 - 0.2, z_hub0 + (SPLINE_Z1 - SPLINE_Z0) + 0.3, SPLINE_OD + 0.05, 24)      # spline socket
    cyl(cut, SHAFT_X, 0, z_hub0, top + 0.2, 2.2, 16)                                                       # screw hole
    cyl(cut, SHAFT_X, 0, top - 0.8, top + 0.2, 3.9, 20)                                                    # head recess
    if HORN != "disc":
        limit = (args.get("horn_trim_r") or 1e9) - 1.2
        n_arms = 1 if HORN == "arm" else 4
        for i in range(n_arms):
            rot = Matrix.Rotation(HORN_ANGLE + i * 1.5707963267948966, 3, "Z")
            for r in HORN_HOLE_R:
                if r <= limit:
                    q = rot @ Vector((r, 0, 0))
                    cyl(cut, SHAFT_X + q.x, q.y, top - HORN_PLATE_T - 0.2, top + 0.2, HORN_HOLE_D, 10)
    else:
        for i in range(8):
            q = Matrix.Rotation(i * 0.7853981633974483, 3, "Z") @ Vector((HORN_DISC_D / 2 - 2.2, 0, 0))
            cyl(cut, SHAFT_X + q.x, q.y, top - HORN_PLATE_T - 0.2, top + 0.2, HORN_HOLE_D, 10)
    cutter = replace_object(f"{NAME}_horn_cut_tmp", cut, None)
    mod = horn.modifiers.new("detail", "BOOLEAN")
    mod.operation, mod.object = "DIFFERENCE", cutter
    for attr, val in (("solver", "EXACT"), ("use_self", True)):
        try:
            setattr(mod, attr, val)
        except (TypeError, AttributeError):
            pass
    bpy.context.view_layer.update()
    detailed = bpy.data.meshes.new_from_object(horn.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    horn.modifiers.clear()
    old_me = horn.data
    horn.data = detailed
    bpy.data.meshes.remove(old_me)
    bpy.data.objects.remove(cutter, do_unlink=True)
    horn["nominal"] = True
    horn["angle_deg"] = args.get("horn_angle_deg", 0.0)
    parts.append(horn)

# ---------------------------------------------------------------- screws (positions exact, sizes inferred)
if SCREWS:
    bm = bmesh.new()
    d_thr, z_tips, d_head, h_head, z_head0 = CASE_SCREW
    for x, z_tip in zip(CASE_SCREW_X, z_tips):
        for y in CASE_SCREW_Y:
            cyl(bm, x, y, z_head0, z_head0 + h_head, d_head, 24)
            cyl(bm, x, y, z_head0 + h_head - 0.01, z_tip, d_thr, 16)
    parts.append(replace_object(f"{NAME}_case_screws", bm, STEEL))
    if HORN and args.get("horn_screw", True):               # off where the joint places its own horn screw (refresh_servos)
        bm = bmesh.new()
        d_thr, l_thr, d_head, h_head = HORN_SCREW
        screw(bm, SHAFT_X, 0, z_hub0 + HORN_HUB_H, d_head, h_head, d_thr, z_hub0 + HORN_HUB_H - l_thr)
        parts.append(replace_object(f"{NAME}_horn_screw", bm, STEEL))
if EAR_SCREWS:
    bm = bmesh.new()
    d_thr, d_head, h_head = EAR_SCREW
    for hx in HOLE_X:
        screw(bm, hx, 0, EAR_Z1, d_head, h_head, d_thr, EAR_Z1 - EAR_SCREWS)
    parts.append(replace_object(f"{NAME}_ear_screws", bm, STEEL))

for p in parts:
    p.parent = root
    p.matrix_parent_inverse.identity()

bpy.context.view_layer.update()
pts = [p.matrix_world @ Vector(c) for p in [case, spline] for c in p.bound_box]
size = [max(getattr(q, a) for q in pts) - min(getattr(q, a) for q in pts) for a in "xyz"]
print(f"{NAME}: {'exact' if mesh_dir else 'envelope'} case+spline (mm) {tuple(round(s / MM, 2) for s in size)}  "
      f"[datasheet: 32.5 x 12.1 x 28.45]  tris case={len(case.data.polygons)} gear={len(spline.data.polygons)}")
