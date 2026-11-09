"""SCS0009 joint kit: holder (stator) + bracket & idler plate (rotor) + all fasteners.
Runs inside Blender.

Same frame as parts/scs0009.py: origin on the output-shaft axis at the top face of the
servo case, +Z out of the shaft, +X along the body toward the connector end. Metres.

Parts (printed):
  holder       pocket gripping the servo body; ears screw onto a ledge; Ø10 idler boss
               under the floor, coaxial with the shaft.
  bracket      output arm + web. The kit's cross (X) horn sits in a matching X pocket under
               the arm: the arms bear on the pocket walls, so torque is carried positively
               (no friction, no tiny horn screws). One M2x6 machine screw clamps bracket +
               horn onto the output shaft's M2 thread. (A round disc in a round recess only
               holds by friction, and its ~1 mm holes can't take M2 screws.)
  idler_plate  separate plate carrying the 10x15x3 bearing; slides axially onto the boss,
               then screws to the end of the web. (A one-piece U can't be assembled: the
               boss must enter the bearing along the axis.)
Fasteners (all M2 self-tapping unless noted):
  2x ear screw (pan, from the servo kit) - servo ears -> holder ledge
  1x M2x6 MACHINE screw, pan head (new)  - bracket + horn -> output shaft thread
  2x M2x6 countersunk (XGO BOM item 7)   - idler plate -> web
  1x bearing 10x15x3 (XGO BOM item 10)

Rotate `<name>_rotor` about local Z to move the joint. Each object carries `asm_step`
and `explode_mm` (offset along the joint axis for exploded views; see tools/blender/explode.py).

ARGS = {"name": "j", "collection": "joint_kit", "location": (x,y,z), "servo_script": "/abs/parts/scs0009.py"}
Horn values come from parts/scs0009.py (NOMINAL until measured); the pocket follows them.
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

MM = 0.001
args = globals().get("ARGS", {})
NAME = args.get("name", "joint")
COLL = args.get("collection", "joint_kit")
LOC = Vector(args.get("location", (0, 0, 0)))

# ---- Servo (datasheet, mm; x from left ear tip, z from case bottom) ----
SHAFT_X, TOP = 10.55, 25.25
BODY_X0, BODY_X1, BODY_W = 4.6, 27.9, 12.1
EAR_X0, EAR_X1, EAR_W, EAR_Z0, EAR_Z1 = 0.0, 32.5, 11.0, 16.8, 18.4
HOLE_X = (2.0, 30.5)
TOWER_X1 = 19.7
# ---- Fits & printing (mm) ----
CLEAR = 0.2                    # per side, servo body in pocket
WALL = 1.6
FLOOR = 1.6
LEDGE_H = 4.0                  # ledge under the ears
PILOT_D = 1.6                  # M2 self-tapping pilot
CLEAR_D = 2.3                  # M2 clearance
CSK_D = 4.2                    # M2 countersink diameter (90°)
CABLE_W, CABLE_H = 4.0, 3.5    # cable exit at the connector end, bottom
# ---- Horn: read from the servo model so the pocket always matches the horn ----
HORN_CLEAR = 0.20              # per side in the X pocket: deliberately loose so walls never bind
RIB_INTERF = 0.25              # crush ribs reach this far INTO the horn: zero play despite FDM error
RIB_R = 0.45                   # rib radius at full size (tapers to a lead-in at the pocket mouth)
RIB_AT = (0.35, 0.8)           # rib positions along each arm, fraction of arm length
ARM_ABOVE = 2.0                # bracket material above the horn
# ---- Bearing 10x15x3 ----
BRG_ID, BRG_OD, BRG_T = 10.0, 15.0, 3.0
BRG_GAP = 0.5                  # holder floor to bearing face
SEAT_CLEAR = 0.1
# ---- Bracket ----
ARM_T_IDLE = BRG_T + 1.2       # seat depth + retaining lip
WEB_X0, WEB_X1 = -18.0, -13.0  # web (joint frame, mm), 5 thick so the plate screws have meat
WEB_W = 22.0
PLATE_SCREW_Y = 7.0

PLA_G_CM3 = 1.24


def F(x, y, z):
    """datasheet mm -> joint-frame metres"""
    return Vector(((x - SHAFT_X) * MM, y * MM, (z - TOP) * MM))


def J(x, y, z):
    """joint-frame mm -> metres"""
    return Vector((x * MM, y * MM, z * MM))


coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for ob in [o for o in coll.objects if o.name.startswith(f"{NAME}_")]:   # clean rebuild
    bpy.data.objects.remove(ob, do_unlink=True)


def material(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


HOLDER = material("joint_holder", (0.98, 0.75, 0.15, 1))
BRACKET = material("joint_bracket", (0.95, 0.95, 0.93, 1))
PLATE = material("joint_plate", (0.55, 0.8, 0.95, 1))
STEEL = material("joint_steel", (0.6, 0.62, 0.66, 1))
SCREW = material("joint_screw", (0.15, 0.15, 0.17, 1))


def new_obj(name, bm, mat=None):
    old = bpy.data.objects.get(name)
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
    me = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()
    if mat:
        me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def box(p0, p1):
    bm = bmesh.new()
    out = bmesh.ops.create_cube(bm, size=1.0)
    for v in out["verts"]:
        v.co = Vector((p0.x if v.co.x < 0 else p1.x, p0.y if v.co.y < 0 else p1.y, p0.z if v.co.z < 0 else p1.z))
    return bm


def cone(center, z0, z1, d0, d1, segs=48, bm=None):
    """axis along Z: diameter d0 at z0, d1 at z1 (metres)"""
    bm = bm or bmesh.new()
    out = bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=d0 / 2, radius2=d1 / 2, depth=z1 - z0)
    bmesh.ops.translate(bm, verts=out["verts"], vec=Vector((center.x, center.y, (z0 + z1) / 2)))
    return bm


def cyl(center, z0, z1, d, segs=48, bm=None):
    return cone(center, z0, z1, d, d, segs, bm)


def hull(points):
    bm = bmesh.new()
    for p in points:
        bm.verts.new(p)
    bmesh.ops.convex_hull(bm, input=bm.verts)
    return bm


def boolean(target, cutter_bm, op):
    cutter = new_obj(f"{NAME}_tmp_bool", cutter_bm)
    mod = target.modifiers.new("b", "BOOLEAN")
    mod.operation, mod.object, mod.solver = op, cutter, "EXACT"
    with bpy.context.temp_override(object=target, active_object=target):
        bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def csk_hole(target, c, z_face, depth_mm, direction):
    """Countersunk M2 clearance hole entering at z_face (metres) going `direction` (-1 down, +1 up)."""
    z_in = z_face - direction * 0.5 * MM
    z_out = z_face + direction * (depth_mm + 0.5) * MM
    boolean(target, cyl(c, min(z_in, z_out), max(z_in, z_out), CLEAR_D * MM, 20), "DIFFERENCE")
    h = (CSK_D - CLEAR_D) / 2 * MM          # 90° countersink depth
    z_cone_end = z_face + direction * h
    d_face, d_end = (CSK_D + 0.4) * MM, CLEAR_D * MM
    zs = sorted([(z_face - direction * 0.2 * MM, d_face), (z_cone_end, d_end)])
    boolean(target, cone(c, zs[0][0], zs[1][0], zs[0][1], zs[1][1], 24), "DIFFERENCE")


def pilot_hole(target, c, z_face, depth_mm, direction):
    z0, z1 = z_face - direction * 0.5 * MM, z_face + direction * depth_mm * MM
    boolean(target, cyl(c, min(z0, z1), max(z0, z1), PILOT_D * MM, 16), "DIFFERENCE")


def screw(name, tip_base, direction, length_mm, head="csk", label="M2x6 countersunk"):
    """Head top at tip_base (metres, on the part surface), shank going `direction` (+1/-1 along Z)."""
    bm = bmesh.new()
    L = length_mm * MM
    if head == "csk":                # head is part of the length; flush with the surface
        hh = (3.8 - 2.0) / 2 * MM
        cone(Vector((0, 0)), -hh, 0, 2.0 * MM, 3.8 * MM, 24, bm)
        cyl(Vector((0, 0)), -L, -hh + 0.01 * MM, 1.9 * MM, 16, bm)
    else:                            # pan head sits on the surface
        cyl(Vector((0, 0)), 0, 1.3 * MM, 3.6 * MM, 24, bm)
        cyl(Vector((0, 0)), -L, 0.01 * MM, 1.9 * MM, 16, bm)
    if direction > 0:
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(math.pi, 3, "X"))
    bmesh.ops.translate(bm, verts=bm.verts, vec=tip_base)
    ob = new_obj(name, bm, SCREW)
    ob["fastener"] = label
    return ob


def mass_g(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    v = bm.calc_volume() * 1e6
    bm.free()
    return v * PLA_G_CM3


def is_manifold(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    ok = all(e.is_manifold for e in bm.edges)
    bm.free()
    return ok


# =========================================================== servo (+ horn constants)
SV = {"ARGS": {"name": f"{NAME}_servo", "collection": COLL, "location": (0, 0, 0), "horn": "cross"}}
exec(open(args["servo_script"]).read(), SV)
HUB_D, HUB_H, PLATE_T = SV["HORN_HUB_D"], SV["HORN_HUB_H"], SV["HORN_PLATE_T"]
X_L, X_W_ROOT, X_W_TIP = SV["HORN_CROSS_L"], SV["HORN_ARM_W_ROOT"], SV["HORN_ARM_W_TIP"]
HORN_TOP = 0.3 + HUB_H                     # horn top above case top (servo model: 0.3 gap)
PLATE_Z0 = HORN_TOP - PLATE_T
ARM_R = X_L + X_W_TIP / 2 + HORN_CLEAR + 1.6   # pocket + 1.6 mm wall

# =========================================================== holder (stator)
px0, px1 = BODY_X0 - CLEAR, BODY_X1 + CLEAR
pw = BODY_W / 2 + CLEAR
STYLE = args.get("holder_style", "sleeve")
if STYLE == "frame":
    # direct mount: only the parent link's joint features - floor (with the pivot boss), ONE side
    # wall, and two pads under the ears. Open on top and on the +Y side; the servo is not boxed in.
    holder = new_obj(f"{NAME}_holder", box(F(px0 - WALL, -(pw + WALL), -FLOOR), F(px1 + WALL, pw, 0.0)), HOLDER)
    boolean(holder, box(F(px0 - WALL, -(pw + WALL), -0.5), F(px1 + WALL, -pw, EAR_Z0)), "UNION")          # side wall
    for x0, x1 in ((EAR_X0, px0), (px1, EAR_X1)):                                                       # ear pads
        boolean(holder, box(F(x0, -(pw + WALL) + 0.01, EAR_Z0 - LEDGE_H), F(x1 + 0.3, EAR_W / 2, EAR_Z0)), "UNION")
else:
    holder = new_obj(f"{NAME}_holder", box(F(px0 - WALL, -(pw + WALL), -FLOOR), F(px1 + WALL, pw + WALL, EAR_Z0)), HOLDER)
    boolean(holder, box(F(EAR_X0, -(pw + WALL), EAR_Z0 - LEDGE_H), F(EAR_X1, pw + WALL, EAR_Z0)), "UNION")
floor_z = (-FLOOR - TOP) * MM
boolean(holder, cyl(Vector((0, 0)), floor_z - (BRG_GAP + BRG_T) * MM, floor_z + 0.5 * MM, BRG_ID * MM), "UNION")
boolean(holder, box(F(px0, -pw, 0.0), F(px1, pw, EAR_Z0 + 1)), "DIFFERENCE")                 # servo pocket
for hx in HOLE_X:
    pilot_hole(holder, F(hx, 0, 0), F(0, 0, EAR_Z0).z, LEDGE_H, -1)
if STYLE != "frame":
    boolean(holder, box(F(px1 - 0.5, -CABLE_W / 2, 0.0), F(px1 + WALL + 0.5, CABLE_W / 2, CABLE_H)), "DIFFERENCE")

# =========================================================== bracket (rotor, output side + web)
arm_z0 = PLATE_Z0 - 0.1                    # bracket underside just below the horn plate
arm_z1 = HORN_TOP + 0.1 + ARM_ABOVE
idle_z1 = -(TOP + FLOOR + BRG_GAP)        # top face of idler plate (joint frame, mm)
idle_z0 = idle_z1 - ARM_T_IDLE


CHILD = args.get("child_style", "u_bracket")
LINK_L = args.get("link_len_mm", 38.0)        # plates: axis -> far-end spacer, along -X
END_R = 6.0                                   # plates: half-width at the far end


def arm_points(z0, z1):
    pts = []
    for z in (z0, z1):
        for k in range(48):
            a = 2 * math.pi * k / 48
            pts.append(J(ARM_R * math.cos(a), ARM_R * math.sin(a), z))
            if CHILD == "plates":             # plate runs along -X to the far end (like 2x_shank.stl)
                pts.append(J(-LINK_L + END_R * math.cos(a), END_R * math.sin(a), z))
        if CHILD != "plates":
            for y in (-WEB_W / 2, WEB_W / 2):
                pts.append(J(WEB_X0, y, z))
    return pts


bracket = new_obj(f"{NAME}_bracket", hull(arm_points(arm_z0, arm_z1)), BRACKET)
if CHILD != "plates":
    boolean(bracket, box(J(WEB_X0, -WEB_W / 2, idle_z1), J(WEB_X1, WEB_W / 2, arm_z1 - 0.01)), "UNION")
# X pocket (horn arms + tip rounds + hub), open to the underside
pz0, pz1 = (arm_z0 - 0.5) * MM, (HORN_TOP + 0.1) * MM
c = HORN_CLEAR
for k in range(4):
    rot = Matrix.Rotation(k * math.pi / 2, 3, "Z")
    r0 = -0.6                                      # root corners hide inside the hub cut (no coincident verts)
    quad = [(r0, -X_W_ROOT / 2 - c), (X_L, -X_W_TIP / 2 - c), (X_L, X_W_TIP / 2 + c), (r0, X_W_ROOT / 2 + c)]
    pts = [rot @ J(x, y, 0) + Vector((0, 0, z)) for x, y in quad for z in (pz0, pz1)]
    boolean(bracket, hull(pts), "DIFFERENCE")
    tip = rot @ J(X_L, 0, 0)
    boolean(bracket, cyl(tip, pz0, pz1, (X_W_TIP + 2 * c) * MM, 24), "DIFFERENCE")
boolean(bracket, cyl(Vector((0, 0)), pz0, pz1, (HUB_D + 2 * c) * MM, 40), "DIFFERENCE")
# crush ribs: vertical, tapered half-cylinders on both side walls of every arm. Fully
# grown they protrude HORN_CLEAR + RIB_INTERF from the wall; at the mouth they start small.
lead = 0.6 * MM                                    # lead-in height at the mouth
for k in range(4):
    rot = Matrix.Rotation(k * math.pi / 2, 3, "Z")
    for f in RIB_AT:
        along = f * X_L
        half_w = X_W_ROOT / 2 + (X_W_TIP - X_W_ROOT) / 2 * f       # horn half-width here
        for side in (-1, 1):
            protrude = HORN_CLEAR + RIB_INTERF
            centre = rot @ J(along, side * (half_w + HORN_CLEAR + RIB_R - protrude), 0)
            z_mouth = (arm_z0 + 0.1) * MM                # just inside the mouth: no coplanar faces
            ring = lambda r, z: [centre + Vector((r * math.cos(t), r * math.sin(t), z))
                                 for t in (2 * math.pi * i / 16 for i in range(16))]
            pts = ring(0.1 * MM, z_mouth) + ring(RIB_R * MM, z_mouth + lead) + ring(RIB_R * MM, pz1 + 0.4 * MM)
            boolean(bracket, hull(pts), "UNION")        # one convex solid per rib
boolean(bracket, cyl(Vector((0, 0)), pz0, (arm_z1 + 1) * MM, CLEAR_D * MM, 20), "DIFFERENCE")   # centre screw
if CHILD == "plates":
    # the two plates are joined far from the joint (in the duck: by the next servo / link body);
    # here a spacer block stands in for it. Countersunk M2x10 from each plate into the spacer.
    plate_holes = [J(-LINK_L, s * 3.0, 0) for s in (-1, 1)]
    spacer = new_obj(f"{NAME}_spacer", box(J(-LINK_L - 5, -END_R, idle_z1), J(-LINK_L + 5, END_R, arm_z0)), BRACKET)
    for c in plate_holes:
        csk_hole(bracket, c, arm_z1 * MM, arm_z1 - arm_z0, -1)
        pilot_hole(spacer, c, arm_z0 * MM, 7.0, -1)
        pilot_hole(spacer, c, idle_z1 * MM, 7.0, +1)
else:
    spacer = None
    plate_holes = [J((WEB_X0 + WEB_X1) / 2, s * PLATE_SCREW_Y, 0) for s in (-1, 1)]
    for c in plate_holes:
        pilot_hole(bracket, c, idle_z1 * MM, 6.5, +1)

# =========================================================== idler plate (rotor, bearing side)
plate = new_obj(f"{NAME}_idler_plate", hull(arm_points(idle_z0, idle_z1)), PLATE)
boolean(plate, cyl(Vector((0, 0)), (idle_z1 - BRG_T) * MM, (idle_z1 + 1) * MM, (BRG_OD + 2 * SEAT_CLEAR) * MM, 64), "DIFFERENCE")
boolean(plate, cyl(Vector((0, 0)), (idle_z0 - 1) * MM, (idle_z1 + 1) * MM, 12.0 * MM, 48), "DIFFERENCE")
for c in plate_holes:
    csk_hole(plate, c, idle_z0 * MM, ARM_T_IDLE, +1)

# =========================================================== bearing (visual)
bearing = new_obj(f"{NAME}_bearing_10x15x3", cyl(Vector((0, 0)), (idle_z1 - BRG_T) * MM, idle_z1 * MM, BRG_OD * MM), STEEL)
boolean(bearing, cyl(Vector((0, 0)), (idle_z1 - BRG_T - 1) * MM, (idle_z1 + 1) * MM, BRG_ID * MM), "DIFFERENCE")
bearing["fastener"] = "bearing 10x15x3"

# =========================================================== fasteners
screws_stator = [screw(f"{NAME}_screw_ear{i}", F(hx, 0, EAR_Z1), -1, 6, "pan", "M2x6 pan self-tapping (servo kit)")
                 for i, hx in enumerate(HOLE_X)]
screw_centre = screw(f"{NAME}_screw_centre", J(0, 0, arm_z1), -1, 6, "pan", "M2x6 machine screw, pan head")
PL = 10 if CHILD == "plates" else 6
screws_plate = [screw(f"{NAME}_screw_plate{i}", c + J(0, 0, idle_z0), +1, PL, "csk", f"M2x{PL} countersunk") for i, c in enumerate(plate_holes)]
screws_top = ([screw(f"{NAME}_screw_top{i}", c + J(0, 0, arm_z1), -1, 10, "csk", "M2x10 countersunk") for i, c in enumerate(plate_holes)]
              if CHILD == "plates" else [])


# =========================================================== hierarchy + assembly metadata
def empty(name, parent=None, size=0.01):
    e = bpy.data.objects.get(name) or bpy.data.objects.new(name, None)
    if e.name not in coll.objects:
        coll.objects.link(e)
    e.empty_display_type, e.empty_display_size = "ARROWS", size
    e.parent = parent
    return e


root = empty(f"{NAME}_joint")
root.location = LOC
rotor = empty(f"{NAME}_rotor", root, 0.008)
rotor.rotation_mode = "XYZ"
rotor.rotation_euler = (0, 0, 0)
rotor["joint_axis"] = [0, 0, 1]

servo_root = bpy.data.objects[f"{NAME}_servo"]
servo_root.parent = root
servo_root.matrix_parent_inverse.identity()
servo_parts = [servo_root]

# (object, parent, assembly step, explode offset along axis in mm, caption)
plan = [(holder, root, 1, -34, "1  Parent link: ear pads + floor boss" if STYLE == "frame" else "1  Holder")]
plan += [(o, root, 2, 0, "2  Servo onto parent frame (ears on pads)" if STYLE == "frame" else "2  Servo into holder") for o in servo_parts]
plan += [(o, root, 3, 10, "3  2x ear screws (servo kit)") for o in screws_stator]
horn = bpy.data.objects.get(f"{NAME}_servo_horn_cross")
if horn:
    plan.append((horn, rotor, 4, 22, "4  X horn presses into plate pocket (crush ribs)" if CHILD == "plates"
                 else "4  X horn presses into bracket pocket"))
plan.append((bracket, rotor, 5, 44, "5  Centre servo (511); horn-side plate+horn onto spline" if CHILD == "plates"
             else "5  Centre servo (511); bracket+horn onto spline"))
plan.append((screw_centre, rotor, 6, 70, "6  M2x6 machine screw into shaft"))
plan.append((bearing, root, 7, -50, "7  Bearing into idler plate"))
plan.append((plate, rotor, 8, -72, "8  Bearing-side plate onto boss" if CHILD == "plates" else "8  Idler plate onto boss"))
if CHILD == "plates":
    plan.append((spacer, rotor, 9, -14, ""))       # far-end stand-in for the next link: no caption
    plan += [(o, rotor, 9, 62, "9  Far end: 2x M2x10 csk, top plate -> next link") for o in screws_top]
    plan += [(o, rotor, 10, -96, "10 Far end: 2x M2x10 csk, bottom plate -> next link") for o in screws_plate]
else:
    plan += [(o, rotor, 9, -96, "9  2x M2x6 csk into web") for o in screws_plate]

for ob, parent, step, off, cap in plan:
    ob.parent = parent
    ob.matrix_parent_inverse.identity()
    ob["asm_step"], ob["explode_mm"], ob["caption"] = step, off, cap
if horn:   # the horn is a child of the servo root by default; keep it on the rotor
    horn.location = (0, 0, 0)
spline = bpy.data.objects.get(f"{NAME}_servo_spline")
if spline:  # the output spline turns with the joint, so it belongs to the rotor too
    spline.parent = rotor
    spline.matrix_parent_inverse.identity()
    spline.location = (0, 0, 0)

bpy.context.view_layer.update()
report = []
for ob in [o for o in (holder, bracket, plate, spacer) if o]:
    ob["mass_g_solid_PLA"] = round(mass_g(ob), 2)
    ob["manifold"] = is_manifold(ob)
    report.append(f"{ob.name.split('_', 1)[1]} {ob['mass_g_solid_PLA']} g{'' if ob['manifold'] else ' NOT MANIFOLD'}")
print(f"{NAME}: " + ", ".join(report) + " (solid PLA)")
n_fast = len([o for o, *_ in plan if "fastener" in o])
print(f"  fasteners: {n_fast} | X pocket L{X_L}+{HORN_CLEAR}, {4 * 2 * len(RIB_AT)} crush ribs @ {RIB_INTERF} mm interference | arm {arm_z0:.2f}..{arm_z1:.2f} mm (r {ARM_R:.1f}), idler plate {idle_z0:.2f}..{idle_z1:.2f} mm")
print(f"  centre screw M2x6 tip at {arm_z1 - 6:.2f} mm above case top (stock M2x4 tip: {HORN_TOP - 4:.2f})")
