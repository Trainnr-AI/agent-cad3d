"""Wiring of the duck as cables in place: every servo lead to its hub, the hub-to-hub extension cables, and the
power / signal wires between battery, regulator, servo adapter, Uno Q and balance sensor. Runs inside Blender.

Cables are smooth curves through waypoints; each waypoint belongs to a part ("owner"), so the same wiring can be
drawn on the assembled robot or on an exploded copy of it (ARGS["exploded"]: a collection of '<name>.x' copies,
e.g. from explode_all.py - every waypoint follows its owner's copy). Nothing is checked for collisions here;
the lengths are reported against the kit's cables (lead 150 mm, extension 165 mm).

Bus tree (kit hubs have 3 sockets; the adapter is assumed to offer ONE bus socket - worst case):
  adapter - S2 - S1;  S2 - left chain;  S1 - right chain + head chain
  leg chain:  body hub A (hip yaw) - body hub B (hip roll) - thigh hub A (hip pitch) - thigh hub B (knee, ankle)
  head chain: body hub (neck pitch) - cradle hub (head pitch) - head hub on the head-yaw servo (head yaw, head roll)

ARGS = {"kit": "/abs/parts/cable_kit.py", "joint_lib": ..., "base_lib": ..., "dims": ..., "collection": "wiring",
        "exploded": None | "duck_v2_exploded", "scale": 1.0}
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
K = {"ARGS": {"lib": True}, "__name__": "kit"}
exec(open(args["kit"]).read(), K)
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
EX = bpy.data.collections.get(args["exploded"]) if args.get("exploded") else None
SC = args.get("scale", 1.0)
COLL = args.get("collection", "wiring_exploded" if EX else "wiring")
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
bpy.context.view_layer.update()
(cx0, cx1), (cy0, cy1), (cz0, cz1) = L["J_CABLE"]
m_lead, m_ext = K["WHITE"], K["_mat"]("kit_ext", (0.75, 0.12, 0.1, 1))
m_pwr, m_sig = K["_mat"]("kit_power", (1.0, 0.55, 0.0, 1)), K["_mat"]("kit_signal", (0.2, 0.5, 1.0, 1))


def copy_of(name):
    for o in EX.objects:
        if o.name == name + ".x" or o.name.startswith(name + ".x."):
            return o
    return None


def xf(owner):
    """matrix that carries an assembled-pose world point of `owner` to where that part is drawn"""
    if EX is None:
        return Matrix.Identity(4)
    c = copy_of(owner)
    if c is None:
        return Matrix.Identity(4)
    return c.matrix_world @ bpy.data.objects[owner].matrix_world.inverted()


def W(owner, p):
    return xf(owner) @ Vector(p)


def servo_pt(j, p_frame_mm):
    return bpy.data.objects["scs_" + j].matrix_world @ (Vector(p_frame_mm) * MM)


def exit_pts(j):
    """the lead's first two waypoints: in the slot, and 7 mm straight out of it (both move with the servo)"""
    own = f"scs_{j}_case"
    z = (cz0 + cz1) / 2
    return [(own, servo_pt(j, (cx0, 0, z))), (own, servo_pt(j, (cx0 - 7.0, 0, z)))]


def axis(j):
    return bpy.data.objects["scs_" + j].matrix_world.translation.copy()


hubs = {}                                     # name -> dict(owner, M (zero pose, world), kind, turn (may the hub be turned on its seat))
specs = []                                    # cables as written below; built at the end, once every hub's sockets are assigned
report = []
routes = []                                   # (name, [(owner, world point at the zero pose)], kit length or None)
PI = 3.14159265


def add_hub(name, owner, M, kind="hub", turn=True):
    hubs[name] = dict(owner=owner, M=M, kind=kind, turn=turn)


def H(name):
    """a cable end in one of this hub's sockets (which one is chosen at the end, by where the cable comes from)"""
    return ("@", name)


def SOLDER(name, top=False):
    """a cable end soldered to the bus board (no plug): under its middle, or (top) from above at its rear end beside the screw"""
    return ("@S", name, top)


def centre(name):
    return hubs[name]["M"].translation.copy()


def cable(name, pts, width, thick, mat, limit=None):
    specs.append((name, pts, width, thick, mat, limit))


# ------------------------------------------------------------------ hubs: 6 kit hubs on the limbs + one bus board in the body.
#   thigh    one on the hip-pitch servo's top face, one on the knee servo's rear face (foam tape; held down by its cover)
#   neck     one on the cradle's rear side wall, 0.30 of the neck length up (foam tape + zip tie round the cradle)
#   head     one on the flat bottom of the head-yaw servo (foam tape + zip tie)
#   body     bus board between the two hip-yaw servos, on the deck's three posts + one M2x6 (five leads plug in from above;
#            the cables to the two thighs, the neck and the servo adapter are soldered to it and leave underneath)
SEAT = 2.3                                    # PCB centre above the seat face: 1.5 mm foam tape under the solder side


def servo_hub(name, j, owner):
    """hub lying flat on a servo's bottom face, centred on the case"""
    M = bpy.data.objects["scs_" + j].matrix_world @ Matrix.Translation(Vector((5.7, 0.0, L["J_BOTTOM"] - SEAT)) * MM) @ Matrix.Rotation(PI, 4, "X")
    add_hub(name, owner, M)


FACE_Z = (L["J_BOTTOM"] + L["J_EAR_Z"][0]) / 2            # middle of a servo's free side face (bottom .. ear underside)


def face_side(j, world_dir):
    """+1 / -1: which of servo j's two large side faces (frame y = +-J_BODY_HW) looks most along world_dir"""
    return 1.0 if (bpy.data.objects["scs_" + j].matrix_world.to_3x3() @ Vector((0, 1, 0))).dot(Vector(world_dir)) > 0 else -1.0


THIGH_SEAT = 1.6                                         # thigh hubs: 0.8 mm double-sided tape (PCB centre 1.6 above the face)
HUB_OUT = 5.0                                            # ... and 5 mm toward the servo's bottom, so the plug facing the thigh plate has room


def face_hub(name, j, owner, sgn, xs, out=None, seat=None):
    """hub lying flat on one of a servo's two LARGE side faces, sockets facing away from the servo. Not turned by the solver:
    socket C opens toward the shaft side (inboard, where the cable from the body / the shin arrives), A and B along the servo."""
    Ms = bpy.data.objects["scs_" + j].matrix_world
    R = Matrix(((0, sgn, 0), (0, 0, sgn), (1, 0, 0))).to_4x4()             # hub x = servo z, hub y = sgn * servo x, hub z = sgn * servo y
    add_hub(name, owner, Ms @ Matrix.Translation(Vector((xs, sgn * (L["J_BODY_HW"] + (THIGH_SEAT if seat is None else seat)), FACE_Z - (HUB_OUT if out is None else out))) * MM) @ R, turn=False)


LAYER2 = 7.5                                             # a second hub taped on the sockets of the one below: PCB centre 7.5 mm further out


def plane_hub(name, j, owner, dx, dz, k, layer=0):
    """hub flat on servo j's END face (bottom), sockets outboard; centre dx, dz (world mm at the zero pose) from the shaft axis,
    turned k quarter turns in the thigh's side view (hub x = (cos, sin) in world X-Z). Not turned by the solver."""
    import math
    Ms = bpy.data.objects["scs_" + j].matrix_world
    out = (Ms.to_3x3() @ Vector((0, 0, -1))).normalized()
    X = Vector((round(math.cos(k * PI / 2)), 0, round(math.sin(k * PI / 2))))
    M = Matrix((X, out.cross(X), out)).transposed().to_4x4()
    M.translation = Ms.translation + Vector((dx, 0, dz)) * MM + out * (-L["J_BOTTOM"] + SEAT + layer * LAYER2) * MM
    add_hub(name, owner, M, turn=False)


def sp(j, x, y, z):
    """a point in servo j's frame (mm), riding on that servo"""
    return (f"scs_{j}_case", servo_pt(j, (x, y, z)))


def wp(owner, x, y, z):
    """a point given in layout coordinates (world mm at the usual place), riding on `owner`"""
    t = bpy.data.objects["duck_scs0009:trunk_base"].matrix_world
    return (owner, t @ (Vector((x, y + 100.0, z - 118.8)) * MM))


ZC = (cz0 + cz1) / 2                                     # cable height in the servo frame (just above the bottom)
XS, YS = cx0, L["J_BODY_HW"] + 1.4                       # slot face; a lead lying against the servo's side
trunk_M = bpy.data.objects["duck_scs0009:trunk_base"].matrix_world
THIGH_HUBS = args.get("thigh_hubs", "bottom")           # "bottom": on the two servos' end faces (verified layout) | "faces": top / rear large faces (trial)
# Fixed body: two hubs. N on the chest plate in front of the neck pedestal (the chest pocket is clear of everything that moves, but
# holds only ONE plugged hub - hub_seats.py + a pair search); G standing upright in the gap between the two hip-yaw servos, its
# third socket pointing down into the deck's cable slot.
def body_hub(name, x, y, z, zdir, xdir):
    Z, X = Vector(zdir), Vector(xdir)
    M = Matrix((X, Z.cross(X), Z)).transposed().to_4x4()
    M.translation = Vector((x, y + 100.0, z - 118.8)) * MM
    add_hub(name, "body_frame", trunk_M @ M, turn=False)


body_hub("N", 47.2, -90.2, 95.5, (1, 0, 0), (0, -1, 0))
body_hub("G", 16.0, -103.6, 109.5, (0, 1, 0), (0, 0, -1))
for s in ("left", "right"):
    T = s[0].upper() + "T"
    # thigh hubs on the servos' LARGE faces: A on top of the hip-pitch servo, B on the rear of the knee servo (under one cover each)
    if THIGH_HUBS == "faces":
        # A in the middle of the top face; B high on the rear face (13.5 above the knee axis): below that the shin's ankle end sweeps past
        face_hub(T + "A", f"{s}_hip_pitch", f"scs_{s}_hip_pitch_case", face_side(f"{s}_hip_pitch", (0, 0, 1)), 5.7)
        face_hub(T + "B", f"{s}_knee", f"scs_{s}_knee_case", face_side(f"{s}_knee", (-1, 0, 0)), 13.5)
    else:
        # flat on the two servos' end faces (the thigh's outer side). Offsets (mm, from each servo's shaft axis, in the thigh's
        # side view) and quarter turns come from a packing search (pack3.py): the only arrangement in which both hubs with all
        # six plugs stay within 2 - 2.5 mm of the ORIGINAL thigh outline. The hub is not mirror-symmetric, so the two legs differ.
        (ax_, az_, ak_), (bx_, bz_, bk_) = {"left": ((-7.5, 2.4, 3), (1.3, 6.7, 1)), "right": ((-11.5, 3.4, 3), (2.3, 7.7, 1))}[s]
        plane_hub(T + "A", f"{s}_hip_pitch", f"scs_{s}_hip_pitch_case", ax_, az_, ak_)
        plane_hub(T + "B", f"{s}_knee", f"scs_{s}_knee_case", bx_, bz_, bk_)
        # no stacking: C on TOP of the hip-pitch servo (hip-yaw + hip-roll leads), D on the REAR of the knee servo, 13.5 above the knee
        # axis (cable from the body). Both checked exactly against the hip parts, body, shin and foot over hip pitch x roll x yaw and
        # knee x ankle (scratch top_hub_test.py / rear_hub_test.py): clear.
        # Only 1.2 mm toward the outer side, so neither overhangs the outer-side hubs' plugs; C sits toward the REAR of the top (29.5
        # from the hip-pitch shaft: clear of hub A's plugs), on the thigh's printed shelf level with the servo's top face.
        face_hub(T + "C", f"{s}_hip_pitch", f"scs_{s}_hip_pitch_case", face_side(f"{s}_hip_pitch", (0, 0, 1)), 29.5, out=1.2)
        face_hub(T + "D", f"{s}_knee", f"scs_{s}_knee_case", face_side(f"{s}_knee", (-1, 0, 0)), 13.5, out=1.2, seat=2.2)   # 2.2: its board clears the servo holder's side wall (1.5 proud) and hub B's board
Mn = bpy.data.objects["scs_neck_pitch"].matrix_world
pH = Mn.inverted() @ axis("head_pitch") / MM            # head-pitch axis in the neck frame
# Cradle hub: on the cradle's REAR side wall, 0.30 of the neck length up, C socket toward the head. Found by a search over the
# cradle's three free faces (13 heights x 4 turns, hub + its three plug rooms against everything on the body and on the head-pitch
# U): no seat clears the whole range; this one keeps neck -135..+19.5 deg and head pitch -76.5..+105 deg.
un = Vector((pH.x, pH.y, 0)).normalized()
vn = Vector((0, 0, 1)).cross(un)
C_SIDE, C_MIDZ = -7.6, -19.05                            # the rear side wall's face and the cradle's mid-depth, in the neck frame
Mc_ = Matrix((un, (-vn).cross(un), -vn)).transposed().to_4x4()
Mc_.translation = (un * pH.length * 0.30 + vn * (C_SIDE - SEAT) + Vector((0, 0, C_MIDZ))) * MM
add_hub("C", "neck_cradle", Mn @ Mc_, turn=False)
cC, cN, cU = centre("C"), (Mn.to_3x3() @ -vn).normalized(), (Mn.to_3x3() @ un).normalized()
servo_hub("Y", "head_yaw", "scs_head_yaw_case")


def c_lead(j):
    """type-C servo: the lead comes out of the slot and wraps round the bottom edge"""
    return [sp(j, XS, 0, ZC), sp(j, XS - 3.0, 0, ZC), sp(j, XS - 3.0, 0, L["J_BOTTOM"] - 3.0)]


def f_lead(j, side=1, ys=None):
    """type-F servo: folded in the pad's pocket, out sideways, then along the servo's side (ys: how far out it runs)"""
    xf = (L["J_EAR_X"][0] + L["J_BODY_X"][0]) / 2
    ys = ys or YS + 1.6
    return [sp(j, XS, 0, ZC), sp(j, xf, 0, ZC), sp(j, xf, side * ys, ZC), sp(j, -2.0, side * ys, ZC), sp(j, 8.0, side * ys, ZC)]


YW = L["CAP_HW"] + 2.0                                  # outside a saddle wall
YG = 8.4                                                # the gap beside the bus board (board +-6.7, servo faces +-10)


# ------------------------------------------------------------------ servo leads (150 mm each)
bay = bpy.data.objects["servo_adapter"]
bbz = [bay.matrix_world @ Vector(b) for b in bay.bound_box]
adapter = ("servo_adapter", Vector((max(v.x for v in bbz) - 0.002, sum(v.y for v in bbz) / 8, max(v.z for v in bbz))))      # top, front end (its two sockets)
XG = -8.6                                                                  # the 4.5 mm gap between the battery's front end and the pocket's front wall
PORT = [wp("electronics_enclosure", -9.5, -88.0, 89.8), wp("electronics_enclosure", -4.5, -88.0, 89.8)]      # through the keel's front wall, under the deck (body_v2.py: PORT)


def riser(y):
    """from the top of the keel's contents through the shelf's notch, then up the gap in front of the battery"""
    return [wp("electronics_enclosure", -10.5, y, 100.8), wp("electronics_enclosure", XG, y, 105.5), wp("electronics_enclosure", XG, y, 117.5)]


for s, m in (("left", 1), ("right", -1)):
    T = s[0].upper() + "T"
    kn, sb = f"{s}_knee", face_side(f"{s}_knee", (-1, 0, 0))             # sb: the knee servo's rear large face
    HWf = L["J_BODY_HW"] + 1.5
    yo = -100.0 + m * 72.3                                              # the plane the first layer's plugs lie in, outside the servos' end faces
    # hip yaw: out of the front face at the top, folded OUTBOARD, down the servo's outboard front corner, across to the front of the
    # thigh's top (near the hip-pitch axis, so the path changes little when the thigh swings), into hub C's front socket
    yw_ = m * (L["J_BODY_HW"] + 1.0)                                    # outboard side of this servo, in its own frame
    cable(f"lead_{s}_hip_yaw", [sp(f"{s}_hip_yaw", XS, 0, ZC), sp(f"{s}_hip_yaw", XS - 0.8, 0, ZC), sp(f"{s}_hip_yaw", XS - 0.8, yw_, ZC), sp(f"{s}_hip_yaw", XS - 0.8, yw_ * 1.3, -8.0),
                               wp(f"{s}_thigh", 31.0, -100.0 + m * 60.0, 96.0), wp(f"{s}_thigh", 24.0, -100.0 + m * 63.0, 93.0), wp(f"{s}_thigh", 6.0, -100.0 + m * 63.0, 92.8), H(T + "C")],
          K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    # hip roll: up out of the servo, over the thigh plate, into hub C's inboard socket
    cable(f"lead_{s}_hip_roll", [sp(f"{s}_hip_roll", XS, 0, ZC), sp(f"{s}_hip_roll", XS - 3.0, 0, ZC), wp(f"{s}_yaw_connector", 2.0, -100.0 + m * 14.0, 90.0),
                                wp(f"{s}_thigh", -4.0, -100.0 + m * 33.0, 92.5), wp(f"{s}_thigh", -9.5, -100.0 + m * 38.0, 94.0), H(T + "C")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    # hip pitch: out of the front end, straight into hub A's front socket
    cable(f"lead_{s}_hip_pitch", c_lead(f"{s}_hip_pitch") + [H(T + "A")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    # knee: out of the lower end, up the servo's FRONT face, over its top end, down into hub B's upper socket
    cable(f"lead_{s}_knee", [sp(kn, XS, 0, ZC), sp(kn, XS - 2.5, 0, ZC), sp(kn, XS - 2.5, -sb * HWf, ZC + 2.0), sp(kn, 19.0, -sb * HWf, -20.0),
                            sp(kn, 30.5, -sb * 2.0, L["J_BOTTOM"] - 5.0), H(T + "B")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    # ankle: out sideways (rear), up the ankle servo's rear side, across the knee, up behind the knee servo into hub B's rear socket
    cable(f"lead_{s}_ankle", f_lead(f"{s}_ankle", -1, YW) + [sp(f"{s}_ankle", 16.5, -YW, ZC), sp(kn, -7.5, sb * YS, L["J_BOTTOM"] - 1.5),
                                                      sp(kn, 0.0, sb * (YS + 8.5), L["J_BOTTOM"] - 6.0), H(T + "B")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    # links: C (top) -> D (rear) round the thigh's top rear corner; D -> A down the rear and round to the outer side; A -> B on the outer side
    cable(f"ext_{s}_thigh_C_to_D", [H(T + "C"), H(T + "D")], K["EXT_W"], K["EXT_T"], m_ext, 165)
    cable(f"ext_{s}_thigh_D_to_A", [H(T + "D"), wp(f"{s}_thigh", -31.0, -100.0 + m * 70.0, 58.0), wp(f"{s}_thigh", -33.0, yo, 78.0), wp(f"{s}_thigh", -14.0, yo - m * 1.5, 91.5), H(T + "A")],
          K["EXT_W"], K["EXT_T"], m_ext, 165)
    cable(f"ext_{s}_thigh_A_to_B", [H(T + "A"), wp(f"{s}_thigh", 4.0, yo - m * 1.5, 63.5), H(T + "B")], K["EXT_W"], K["EXT_T"], m_ext, 165)
    # the one cable that crosses the hip: (left: straight from the adapter's first socket, out of the keel's port; right: from hub G, down the
    # deck slot) -> under the deck -> behind the roll servo -> over the thigh plate's rear -> hub D's inboard socket
    start = [adapter] + PORT if s == "left" else [H("G"), wp("body_frame", 14.5, -100.4, 90.0)]
    cable(f"ext_{s}_body_to_thigh", start + [wp(f"{s}_yaw_connector", -1.0, -100.0 + m * 12.0, 86.0), wp(f"{s}_thigh", -2.0, -100.0 + m * 33.0, 92.0),
                                             wp(f"{s}_thigh", -24.0, -100.0 + m * 33.0, 92.0), wp(f"{s}_thigh", -31.5, -100.0 + m * 37.5, 84.0), H(T + "D")],
          K["EXT_W"], K["EXT_T"], m_ext, 165)
# adapter's second socket -> hub G: up the riser in front of the battery, through the notch at the top of the pocket's front wall
cable("ext_adapter_to_gap", [adapter] + riser(-104.5) + [wp("electronics_enclosure", -8.0, -104.5, 121.9), wp("body_frame", -2.5, -102.5, 118.5), H("G")], K["EXT_W"], K["EXT_T"], m_ext, 165)
# hub G -> chest hub N: forward out of the gap, beside the neck pedestal, down to the chest
cable("ext_gap_to_chest", [H("G"), wp("body_frame", 38.0, -106.0, 104.0), wp("body_frame", 50.0, -106.0, 99.0), H("N")], K["EXT_W"], K["EXT_T"], m_ext, 165)
# neck pitch: out sideways at the front of the cradle's foot, down in front of the pedestal to chest hub N
cable("lead_neck_pitch", f_lead("neck_pitch", 1, 9.2)[:3] + [wp("body_frame", 50.0, -112.0, 104.0), H("N")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
# head pitch: out sideways at the back of the cradle's top, down its side, round the floor's edge to the hub on its back
cable("lead_head_pitch", f_lead("head_pitch", 1, 9.2) + [("neck_cradle", cC + cU * 0.030 + cN * 0.004), ("neck_cradle", cC + cU * 0.024 + cN * 0.005), H("C")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
cable("lead_head_yaw", c_lead("head_yaw") + [H("Y")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
cable("lead_head_roll", [sp("head_roll", XS, 0, ZC), sp("head_roll", XS - 3.0, 0, ZC), wp("yaw_block", 16.0, -94.0, 221.5), H("Y")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
# extension up the neck: chest hub N -> beside the pedestal -> up behind it -> the hub on the cradle's rear side wall
cable("ext_body_to_cradle", [H("N"), wp("body_frame", 40.0, -114.0, 97.0), wp("body_frame", 29.6, -106.0, 110.0),
                            wp("body_frame", 27.5, -104.5, 124.5), ("neck_cradle", cC - cU * 0.014 + cN * 0.010), H("C")],
      K["EXT_W"], K["EXT_T"], m_ext, 165)
# extension to the head: hub on the cradle -> up the cradle's back edge -> behind the U's top plate -> beside the yaw block -> hub on the head-yaw servo
cable("ext_cradle_to_head", [H("C"), ("neck_cradle", cC + cU * 0.026 + cN * 0.009), wp("head_pitch_u", 21.5, -113.0, 188.0),
                            wp("yaw_block", 22.0, -116.0, 201.0), wp("scs_head_yaw_case", 31.0, -120.5, 226.0), H("Y")], K["EXT_W"], K["EXT_T"], m_ext, 165)

# ------------------------------------------------------------------ power and signal
def top_of(n):
    o = bpy.data.objects[n]
    c = sum((o.matrix_world @ Vector(b) for b in o.bound_box), Vector()) / 8
    return (n, Vector((c.x, c.y, max((o.matrix_world @ Vector(b)).z for b in o.bound_box))))


rear = wp("electronics_enclosure", -47.0, -96.0, 119.0)                    # the pocket's open rear part, above the battery
cable("power_battery_to_regulator", [top_of("battery_3s")] + riser(-93.6)[::-1] + [wp("electronics_enclosure", -14.0, -89.0, 90.0), top_of("ubec")], 2.4, 2.4, m_pwr)
cable("power_regulator_to_adapter_6V", [top_of("ubec"), wp("electronics_enclosure", -30.0, -88.0, 88.0), wp("electronics_enclosure", -30.0, -106.0, 94.5), top_of("servo_adapter")], 2.4, 2.4, m_pwr)
cable("power_battery_to_uno_VIN", [top_of("battery_3s"), rear, wp("electronics_enclosure", -47.0, -96.0, 131.5)], 1.8, 1.8, m_pwr)
cable("signal_uno_UART_to_adapter", [wp("electronics_enclosure", -47.0, -104.0, 131.5), wp("electronics_enclosure", -47.0, -104.0, 118.0), wp("electronics_enclosure", -20.0, -100.0, 122.6),
                                    wp("electronics_enclosure", -9.0, -100.0, 120.5)] + riser(-100.0)[::-1] + [wp("electronics_enclosure", -14.0, -106.0, 93.5), top_of("servo_adapter")], 2.5, 1.0, m_sig)
cable("signal_imu_qwiic_to_uno", [top_of("imu"), wp("electronics_enclosure", -20.0, -94.8, 68.0), wp("electronics_enclosure", -15.0, -94.8, 78.0), wp("electronics_enclosure", -14.0, -94.8, 92.0),
                                 wp("electronics_enclosure", -11.5, -96.5, 99.5)] + riser(-97.0) + [wp("electronics_enclosure", -20.0, -98.0, 122.4), wp("electronics_enclosure", -47.5, -108.0, 119.0),
                                 wp("electronics_enclosure", -47.5, -108.0, 131.5)], 2.0, 1.0, m_sig)


# ------------------------------------------------------------------ sockets: every cable end gets its own socket and a plug.
# A hub may be turned on its seat in 90 deg steps; the turn and the socket of each cable are the ones with the shortest
# runs from the cables' last waypoints to the sockets (a cable arriving from behind a socket's mouth counts 30 mm extra).
import itertools


def mouths(h, k=0):
    M = h["M"] @ Matrix.Rotation(k * PI / 2, 4, "Z")
    n = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
    ms = K["bus_mouths"](M) if h["kind"] == "bus" else K["socket_mouths"](M)
    return [(p, d, n if h["kind"] != "bus" else (M.to_3x3() @ Vector((0, 1, 0))).normalized()) for p, d in ms], M


ends = {}                                                                  # hub -> [(cable, end index 0 / -1, neighbour point)]
for name, pts, *_ in specs:
    for e, nb in ((0, 1), (-1, -2)):
        if pts[e][0] == "@":
            ends.setdefault(pts[e][1], []).append((name, e, Vector(pts[nb][1]) if pts[nb][0] not in ("@", "@S") else centre(pts[nb][1])))
assign = {}                                                                # (cable, end) -> (owner, mouth, direction, plug's thin axis)
ROOM = K["PLUG_ROOM"] * MM
for hn, lst in ends.items():
    h = hubs[hn]
    best = None
    for k in (range(4) if h["turn"] else (0,)):
        ms, M = mouths(h, k)
        for perm in itertools.permutations(range(len(ms)), len(lst)):
            cost = 0.0
            for (cn, e, nbp), si in zip(lst, perm):
                p, d, n = ms[si]
                cost += (nbp - (p + d * ROOM)).length + (0.03 if (nbp - p).dot(d) < 0 else 0.0)
            if best is None or cost < best[0]:
                best = (cost, k, perm)
    _, k, perm = best
    ms, M = mouths(h, k)
    h["M"] = M
    for (cn, e, nbp), si in zip(lst, perm):
        assign[(cn, e)] = (h["owner"], *ms[si])
        h.setdefault("used", {})[si] = cn
for hn, h in hubs.items():                                                 # draw the boards
    (K["bus_board"] if h["kind"] == "bus" else K["hub"])("wire_hub_" + hn, xf(h["owner"]) @ h["M"], coll)


def plug_at(cn, e, owner, p, d, n):
    X, Z = (-d).normalized(), n.normalized()
    M = Matrix((X, Z.cross(X), Z)).transposed().to_4x4()
    M.translation = p + d * 0.0035                                         # 6 mm of the plug is in the socket, 3.5 mm stands out
    K["plug"](f"wire_plug_{cn}_{'a' if e == 0 else 'b'}", xf(owner) @ M, coll)
    return [(owner, p + d * 0.0035), (owner, p + d * ROOM)]


n_sol = [0]
for name, pts, width, thick, mat, limit in specs:
    full = []
    for i, q in enumerate(pts):
        if q[0] == "@":
            e = 0 if i == 0 else -1
            pair = plug_at(name, e, *assign[(name, e)])
            full += pair if e == 0 else pair[::-1]
        elif q[0] == "@S":
            h = hubs[q[1]]
            if q[2]:                                                        # from above, rear end (the saddle walls leave no way down beside the board)
                full.append((h["owner"], h["M"] @ (Vector((-5.5, -1.0, K["BUS"][2] / 2 + 0.8)) * MM)))
            else:
                n_sol[0] += 1                                               # in a row under the board's middle, over the deck slot
                full.append((h["owner"], h["M"] @ (Vector((-8.0 + 4.0 * n_sol[0], 0.0, -K["BUS"][2] / 2 - 0.8)) * MM)))
        else:
            full.append(q)
    ob, length = K["ribbon"]("wire_" + name, [W(o, p) for o, p in full], width * SC, thick * SC, coll, mat)
    if EX is None:
        report.append((name, length, limit))
        routes.append((name, [(o, Vector(p)) for o, p in full], limit))
if EX is None and args.get("motion"):
    # ---- does any cable run out of length when the joints move? Each waypoint rides on its owner part; the path through
    # the moved waypoints (straight segments = the least cable that can connect them) is compared with the kit cable,
    # less `ends_mm` for the two plugs / the bend out of the servo.
    import math
    from mathutils import Quaternion
    R = args.get("robot", "duck_scs0009")
    joints = {e["joint"]: e for e in bpy.data.objects if e.name.startswith(R + ":") and e.type == "EMPTY" and "joint" in e}
    q0 = {n: e.rotation_quaternion.copy() for n, e in joints.items()}
    ENDS = args.get("ends_mm", 6.0)
    local = []
    for name, pts, limit in routes:
        local.append((name, [(o, bpy.data.objects[o].matrix_world.inverted() @ p) for o, p in pts], limit))

    def set_pose(q):
        for jn, e in joints.items():
            e.rotation_quaternion = q0[jn] @ Quaternion(Vector(e["joint_axis"]), q.get(jn, 0.0))
        bpy.context.view_layer.update()

    def r_of(j, k, n):
        lo, hi = joints[j]["joint_range"]
        return lo + (hi - lo) * k / (n - 1)

    poses = [("rest", {})]
    for j in joints:
        for k in range(13):
            poses.append((f"{j} {math.degrees(r_of(j, k, 13)):+.0f}", {j: r_of(j, k, 13)}))
    for s_ in (1, -1):                                        # walking-range combinations, both legs
        for yaw in (-0.11, 0.22):
            for roll in (-0.19, 0.19):
                for p_ in (-0.8, 0.8):
                    q = {}
                    for side, m in (("left", 1), ("right", -1)):
                        q.update({f"{side}_hip_yaw": m * yaw, f"{side}_hip_roll": m * roll, f"{side}_hip_pitch": m * s_ * p_, f"{side}_knee": -m * s_ * p_ * 1.2, f"{side}_ankle": m * s_ * p_ * 0.5})
                    poses.append((f"walk yaw {yaw} roll {roll} pitch {s_ * p_}", q))
    for npz in (-2.356, -1.2, 1.047):                         # neck with the head turned and pitched
        for hp in (-1.2, 1.2):
            for hy in (-2.6, 2.6):
                poses.append((f"neck {npz} head pitch {hp} yaw {hy}", {"neck_pitch": npz, "head_pitch": hp, "head_yaw": hy}))
    worst = {name: (0.0, "") for name, _, _ in local}
    for label, q in poses:
        set_pose(q)
        for name, pts, limit in local:
            w = [bpy.data.objects[o].matrix_world @ p for o, p in pts]
            d = sum((b - a).length for a, b in zip(w, w[1:])) * 1000
            if d > worst[name][0]:
                worst[name] = (d, label)
    set_pose({})
    rest = {name: length for name, length, _ in report}
    print(f"cable reach over {len(poses)} poses (straight path through the moved waypoints + {ENDS:.0f} mm for the ends):")
    bad = 0
    for name, pts, limit in local:
        if not limit:
            continue
        need = worst[name][0] + ENDS
        ok = need <= limit
        bad += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'SHORT'} {name:28} rest {rest[name]:4.0f}  worst {need:4.0f} of {limit}  at {worst[name][1]}")
    print(f"{bad} cable(s) too short at some pose")
elif EX is None:
    print(f"{COLL}: {len(hubs)} kit hubs, {len(report)} cables")
    for hn, h in hubs.items():
        print(f"  {hn:4} sockets: " + ", ".join(f"{'ABC12345'[si] if h['kind'] != 'bus' else si + 1}={cn}" for si, cn in sorted(h.get('used', {}).items())))
    for name, length, limit in report:
        if limit:
            print(f"  {'ok  ' if length <= limit else 'LONG'} {name:28} {length:5.0f} mm (kit cable {limit})")
else:
    print(f"{COLL}: wiring drawn on {EX.name}")
