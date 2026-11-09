"""Feetech SCS0009 servo envelope model. Runs inside Blender.

Source: FEETECH SCS0009 product specification A/0 (2020-11-23), §9 outline drawing (mm).
Frame (the joint frame): origin on the output-shaft axis at the top face of the case
(z=25.25 mm from the bottom), +Z out of the shaft, +X along the body length away from the
shaft (toward the connector end). Units: metres.

ARGS = {"name": "scs0009", "collection": "scs0009", "location": (x, y, z), "horn": "disc"|"arm"|"cross"|None}
Horn geometry is NOMINAL (not in the datasheet) until measured — see HORN_* below.
"""
import bmesh
import bpy
from mathutils import Matrix, Vector

MM = 0.001
# ---- Datasheet dimensions (mm), x measured from the left ear tip, z from the case bottom ----
EAR_TIP_L, EAR_TIP_R = 0.0, 32.5
BODY_X0, BODY_X1 = 4.6, 27.9            # 23.3 long
SHAFT_X = 10.55                         # 2.0 (hole) + 8.55; 5.7 from body centre (16.25)
HOLE_X = (2.0, 30.5)                    # 2 x Ø2, 28.5 apart
HOLE_D = 2.0
BODY_W = 12.1                           # y
EAR_W = 11.0
EAR_Z0, EAR_Z1 = 16.8, 18.4             # flange thickness 1.6
CASE_TOP = 21.45                        # main case top
TOWER_X1 = 19.7                         # gear tower spans BODY_X0..TOWER_X1
TOWER_TOP = 25.25
SPLINE_OD, SPLINE_H, SPLINE_TEETH = 3.95, 3.2, 20
# ---- Horn (NOMINAL — replace with caliper measurements) ----
HORN_HUB_D, HORN_HUB_H = 6.5, 4.0       # hub that sits over the spline
HORN_PLATE_T = 1.6
HORN_DISC_D = 21.0
HORN_ARM_L, HORN_ARM_W_ROOT, HORN_ARM_W_TIP = 15.5, 5.6, 3.6   # single arm: shaft centre -> arm tip
HORN_CROSS_L = 10.5                    # cross (X) horn: shaft centre -> arm tip; same widths
# Horn holes are ~1 mm linkage holes (user photo, 2026-11-09): not usable for M2 screws.

args = globals().get("ARGS", {})
NAME = args.get("name", "scs0009")
COLL = args.get("collection", "scs0009")
LOC = Vector(args.get("location", (0, 0, 0)))
HORN = args.get("horn", "disc")

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


def replace_object(name, bm, material):
    old = bpy.data.objects.get(name)
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
    me = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()
    me.materials.append(material)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


# Root empty = joint frame
root = bpy.data.objects.get(NAME) or bpy.data.objects.new(NAME, None)
if root.name not in coll.objects:
    coll.objects.link(root)
root.empty_display_type = "ARROWS"
root.empty_display_size = 0.012
root.location = LOC
for k, v in {"model": "Feetech SCS0009", "stall_torque_Nm_6V": 0.2256, "rated_torque_Nm_6V": 0.0736,
             "no_load_speed_rad_s_6V": 10.47, "mass_kg": 0.0132, "voltage_range_V": "4-7.4",
             "range_deg": 300, "resolution_deg": 0.293, "spline": "20T OD3.95", "horn_screw": "M2x4",
             "source": "FEETECH SCS0009 spec A/0 2020-11-23"}.items():
    root[k] = v

# Case: lower body + gear tower (one closed box each), ears drilled separately, then joined
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
pieces = [solid(f"{NAME}_tower", BODY_X0, TOWER_X1, -BODY_W / 2, BODY_W / 2, CASE_TOP, TOWER_TOP)]
for i, (x0, x1) in enumerate(((EAR_TIP_L, BODY_X0), (BODY_X1, EAR_TIP_R))):
    ear = solid(f"{NAME}_ear{i}", x0, x1, -EAR_W / 2, EAR_W / 2, EAR_Z0, EAR_Z1)
    drill(ear, HOLE_X[i])
    pieces.append(ear)
with bpy.context.temp_override(active_object=case, selected_editable_objects=[case] + pieces):
    bpy.ops.object.join()

# Spline (20T approximated as a 20-sided cylinder at OD)
bm = bmesh.new()
cyl(bm, SHAFT_X, 0, TOWER_TOP, TOWER_TOP + SPLINE_H, SPLINE_OD, SPLINE_TEETH)
spline = replace_object(f"{NAME}_spline", bm, BRASS)

parts = [case, spline]

# Horn (nominal)
if HORN:
    bm = bmesh.new()
    z_hub0 = TOWER_TOP + 0.3                     # small gap above the case
    z_plate0 = z_hub0 + HORN_HUB_H - HORN_PLATE_T
    cyl(bm, SHAFT_X, 0, z_hub0, z_hub0 + HORN_HUB_H, HORN_HUB_D, 40)
    if HORN == "disc":
        cyl(bm, SHAFT_X, 0, z_plate0, z_plate0 + HORN_PLATE_T, HORN_DISC_D, 64)
    else:
        arms = 1 if HORN == "arm" else 4
        arm_l = HORN_ARM_L if HORN == "arm" else HORN_CROSS_L
        trim = args.get("horn_trim_r")          # arms cut flat at this radius (fits inside a bearing bore)
        if trim and HORN != "arm":
            arm_l = trim
        for i in range(arms):  # tapered arm: shaft centre -> tip, plus a round tip
            rot = Matrix.Rotation(i * 1.5707963267948966, 3, "Z")
            w_end = HORN_ARM_W_TIP if not trim else HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * trim / HORN_CROSS_L
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
            if not trim:                            # untrimmed arms end in a round tip
                tip = rot @ Vector((arm_l, 0, 0))
                cyl(bm, SHAFT_X + tip.x, tip.y, z_plate0, z_plate0 + HORN_PLATE_T, HORN_ARM_W_TIP, 24)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)      # hand-built arm prisms: make every shell face outward
    horn = replace_object(f"{NAME}_horn_{HORN}", bm, WHITE)
    horn["nominal"] = True
    parts.append(horn)

for p in parts:
    p.parent = root
    p.matrix_parent_inverse.identity()

bpy.context.view_layer.update()
pts = [p.matrix_world @ Vector(c) for p in [case, spline] for c in p.bound_box]
size = [max(getattr(q, a) for q in pts) - min(getattr(q, a) for q in pts) for a in "xyz"]
print(f"{NAME}: case+spline envelope (mm) {tuple(round(s / MM, 2) for s in size)}  [datasheet: 32.5 x 12.1 x 28.45]")
