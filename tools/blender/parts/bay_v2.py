"""Electronics bay v2 - the whole assembly (runs inside Blender): one container + one lid, the boards held without screws, all wires.
Collection `bay_v2` (re-running replaces it); the v1 bay is not touched (its parts are reused as linked copies, they share meshes).

2026-11-27 (user, after printing v1): one closed container; no small protrusions or small separate parts; slide fits instead of
mounts (the IMU's pegs did not work); keep assembly simple; screw holes were too small (M2 would not go in, M3 barely). The Uno Q
upside down under the ceiling, pins down; battery in the centre, the boards' connectors facing it; power leads out and tied outside.

Shape (from the space the robot's real motions leave free, tools/blender/swept_space.py, and the v1 enclosure that passed the
motion check): a COLUMN with v1's footprint (x -53..-5, y +-22, z 56.5..125) and its tongue under the motor body's deck (the same
3 x M2 from below), a wide TOP BOX above the hip-yaw servos (z 125..155, x -93..1, y +-29.2), a LID (z 155..157).

Parts:
- Uno Q upside down, its USB-C end at the FRONT (the camera cable leaves next to the neck), resting on two ledges of the side walls
  (0.8 under each long edge), pressed down by two ribs under the lid. The camera's 90 deg USB-C -> A adapter turns sideways (-y);
  the camera's USB-A plug stays inside, its cable leaves through a notch in the right wall; a bump on the lid covers the adapter.
- Qwiic (the Uno's far end, at the rear) -> ~100 mm cable along the right wall -> the IMU (column, rear wall; notch for the plug).
- Servo bus adapter + IMU + battery: their v1 places in the column (the servo harness to the legs / neck unchanged).
- UBEC-8A (2026-11-28, replaced the Mini560) in a pod outside the top box's rear wall, held down by the lid's extension; splices
  floating in the bundle by the column's top.
- Switch (MTS-1 type, no screw holes / nut): drops into a U-slot of the right wall between two ribs, a stop bar behind it; a tongue
  of the lid closes the slot over its bushing.
- XT30 + balance plug outside the rear wall, all power leads through one slot in it.

Holes (2026-11-27: v1's printed small): M2 clearance 2.8, M2 pilot 2.0 (+0.3..0.4) - check with a hole gauge.
Layout mm (the frame of bay_parts.py / wiring.py). ARGS = {"switch": {...measured...}, "jumpers": {...}, "debug": bool} optional.
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals().get("ARGS", {})
MM = 0.001
R = "duck_scs0009"
COLL = "bay_v2"
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
coll = old or bpy.data.collections.new(COLL)
MAIN = bpy.data.scenes.get("Scene") or bpy.context.window_manager.windows[0].scene   # the model's scene, never an animation scene
if coll.name not in MAIN.collection.children:
    MAIN.collection.children.link(coll)
for _sc in bpy.data.scenes:                                          # (an animation scene that was active once may hold it: unlink)
    if _sc != MAIN and coll.name in _sc.collection.children:
        _sc.collection.children.unlink(coll)
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
MLi = ML.inverted()
S = Matrix.Diagonal((MM, MM, MM, 1.0))
CY, T = -100.0, 2.0
M2_CLEAR, M2_PILOT, M2_CSK = 2.8, 2.0, 4.6
COL = dict(x=(-53.0, -5.0), y=(CY - 22.0, CY + 22.0), z=(57.0, 125.0))   # v1's footprint; floor 1.6 (top 58.6; battery / adapter at 58.9)
# the KEEL (v1's lower column, kept exactly): +-18.5 outside / 1.6 walls up to z 92, 45 deg out to +-22 at 95.5 - the knees (hips yawed
# in, SIT) and feet pass +-19..22 below z 84 (motion check 2026-11-27: a +-22 column all the way down hit the knee caps by 570 mm3)
KEEL = dict(hw=18.5, t=1.6, z1=92.0, z2=95.5, floor=1.6)
TOP = dict(x=(-93.0, 1.0), y=(CY - 29.2, CY + 29.2), z=(123.0, 155.0))   # x -93: free to the rear (swept space), room for the Qwiic plug;
#   floor 123..125 (free to z 122.5): 2 mm lower than first drawn so the Uno sits low enough for the camera adapter under a FLAT lid
LID = (155.0, 157.0)
TONGUE = dict(x=(-5.5, 3.0), y=(CY - 9.0, CY + 9.0), z=(90.0, 94.0))     # under the deck (deck underside z 94.2)
DECK_SCREWS = [(0.5, CY - 5.5), (0.5, CY), (0.5, CY + 5.5)]               # v1's three, M2 x 10 countersunk up from below
IMU_L, IMU_W, IMU_PCB = 27.89, 20.87, 1.6                         # 7Semi BNO086 breakout (BOM row 8)
PX0 = -50.4; PX1 = PX0 + IMU_PCB                                     # its PCB 0.6 off the rear wall, parts toward the battery (2 mm
#   further back than v1: its grooves then clear the servo adapter sliding down past them)
IMU_Y = (CY - IMU_W / 2, CY + IMU_W / 2)
IMU_Z0 = 102.0                                                       # its unused bottom Qwiic socket 0.6 over the servo adapter (top 100.9)
IMU_CH = (101.2, COL["z"][1] + T)                                    # its grooves: z 101.2 .. the column's top 127
X_F = -16.0                                                          # Uno Q PCB edge at its USB-C end (front)
UNO_Z = 148.1                                                        # PCB centre (147.3 .. 148.9): the 90 deg camera adapter turned
#   sideways is 17 wide round the USB-C axis -> its top 0.7 under the lid (was 3.2 over it: a bump on the lid that could not print flat)
# switch from the user's photo (MTS-1 type toggle, metal frame, smooth bushing, 3 pins at 4.7): ESTIMATES until measured
SW = {"frame": (13.0, 9.5, 8.0), "bushing": (5.0, 5.0), "lever": (2.0, 10.0), "pins": (4.7, 4.0)}
SW.update(args.get("switch", {}))


def mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


def bm_box(bm, x0, x1, y0, y1, z0, z1):
    vs = [bm.verts.new(Vector((x, y, z)) * MM) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    for f in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
        bm.faces.new([vs[i] for i in f])


def bm_cyl(bm, x, y, z0, z1, d, axis="Z", n=24, d2=None):
    M = Matrix.Translation(Vector((x, y, (z0 + z1) / 2)) * MM)
    if axis == "Y":                                                   # x, y = (x, z) of the axis; z0, z1 = the y range
        M = Matrix.Translation(Vector((x, (z0 + z1) / 2, y)) * MM) @ Matrix.Rotation(math.pi / 2, 4, "X")
    bmesh.ops.create_cone(bm, cap_ends=True, segments=n, radius1=d / 2 * MM, radius2=(d2 or d) / 2 * MM, depth=(z1 - z0) * MM, matrix=M)


def bm_hull(bm, pts):
    vs = [bm.verts.new(Vector(p) * MM) for p in pts]
    bmesh.ops.convex_hull(bm, input=vs)


PRIM = {"box": bm_box, "cyl": bm_cyl, "hull": bm_hull}


def solid(name, adds, subs, rgba, post=()):
    """adds / subs / post: lists of ('box', x0, x1, y0, y1, z0, z1), ('cyl', x, y, z0, z1, d[, axis]) or ('hull', [pts]) in
    layout mm; exact booleans: union adds, cut subs, then union post (features inside the cut-out space)"""
    def mk(spec):
        bm = bmesh.new()
        PRIM[spec[0]](bm, *spec[1:])
        me = bpy.data.meshes.new(name + "_tmp")
        bm.to_mesh(me)
        bm.free()
        ob = bpy.data.objects.new(name + "_tmp", me)
        coll.objects.link(ob)
        ob.matrix_world = ML
        return ob
    o = mk(adds[0])
    o.name = name
    for kind, specs in (("UNION", adds[1:]), ("DIFFERENCE", subs), ("UNION", post)):
        for sp in specs:
            ob = mk(sp)
            mod = o.modifiers.new("b", "BOOLEAN")
            mod.operation, mod.solver, mod.object = kind, "EXACT", ob
            with bpy.context.temp_override(object=o, active_object=o):
                bpy.ops.object.modifier_apply(modifier=mod.name)
            me = ob.data
            bpy.data.objects.remove(ob, do_unlink=True)
            bpy.data.meshes.remove(me)
    o.data.materials.clear()
    o.data.materials.append(mat(name, rgba))
    o.color = rgba
    bm = bmesh.new()
    bm.from_mesh(o.data)
    o["volume_mm3"] = abs(bm.calc_volume()) / MM ** 3
    o["non_manifold"] = sum(not e.is_manifold for e in bm.edges)
    bm.free()
    o["mass_g_solid_PLA"] = round(o["volume_mm3"] / 1000 * 1.24, 2)
    o["holes_resized"] = "built at the 2026-11-27 sizes (M2 clear 2.8 pilot 2.0): hole_resize.py must skip it"
    return o


def part_obj(name, specs, rgba):
    """a plain (non-printed) part made of primitives, no booleans"""
    bm = bmesh.new()
    for sp in specs:
        PRIM[sp[0]](bm, *sp[1:])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    coll.objects.link(o)
    o.matrix_world = ML
    o.data.materials.append(mat(name, rgba))
    o.color = rgba
    return o


def apply(G, p):
    return (G @ Vector(p).to_4d()).to_3d()


def bbox_layout(o):
    P = [(MLi @ o.matrix_world @ Vector(c)) / MM for c in o.bound_box]
    return Vector([min(p[i] for p in P) for i in range(3)]), Vector([max(p[i] for p in P) for i in range(3)])


# ================================================================== where the Uno Q and the camera adapter go (needed for the cuts)
pcb = bpy.data.objects["unoq_pcb"]
G0 = Matrix.Rotation(math.pi, 4, "Y")                                 # upside down, USB-C end to the front
p0 = [apply(G0, (MLi @ pcb.matrix_world @ Vector(cc)) / MM) for cc in pcb.bound_box]
lo = Vector([min(v[i] for v in p0) for i in range(3)])
hi = Vector([max(v[i] for v in p0) for i in range(3)])
G_uno = Matrix.Translation(Vector((X_F - hi.x, CY - (lo.y + hi.y) / 2, UNO_Z - (lo.z + hi.z) / 2))) @ G0
uy0, uy1 = CY - 26.67, CY + 26.67                                    # the board's long edges
mouth_v1 = Vector(bpy.data.objects["unoq_usbc"]["mouth"])
mouth = apply(G_uno, mouth_v1)
# the 90 deg adapter: C plug into the Uno (-x), its A socket turned to face -y (USB-C is reversible)
G_cam = Matrix.Translation(mouth) @ Matrix.Rotation(-math.pi / 2, 4, "X") @ Matrix.Rotation(math.pi, 4, "Z") @ Matrix.Translation(-mouth_v1)
a_lo, a_hi = bbox_layout(bpy.data.objects["bay_usb_adapter_L"])
pl_lo, pl_hi = bbox_layout(bpy.data.objects["bay_usb_a_plug"])
cam_end = apply(G_cam, ((pl_lo.x + pl_hi.x) / 2, (pl_lo.y + pl_hi.y) / 2, pl_lo.z))   # where the camera cable leaves its plug
ad = [apply(G_cam, Vector((x, y, z))) for x in (a_lo.x, a_hi.x) for y in (a_lo.y, a_hi.y) for z in (a_lo.z, a_hi.z)]
pl = [apply(G_cam, Vector((x, y, z))) for x in (pl_lo.x, pl_hi.x) for y in (pl_lo.y, pl_hi.y) for z in (pl_lo.z, pl_hi.z)]
AD = [Vector([f(p[i] for p in ad) for i in range(3)]) for f in (min, max)]
PL = [Vector([f(p[i] for p in pl) for i in range(3)]) for f in (min, max)]

def group_G(names, rot=None, to_min=None):
    objs = [bpy.data.objects[n] for n in names if n in bpy.data.objects]
    pts = [MLi @ o.matrix_world @ Vector(c) / MM for o in objs for c in o.bound_box]
    c = sum(pts, Vector()) / len(pts)
    Rm = Matrix.Identity(4)
    for ax, deg in (rot or []):
        Rm = Matrix.Translation(c) @ Matrix.Rotation(math.radians(deg), 4, ax) @ Matrix.Translation(-c) @ Rm
    d = Vector()
    if to_min is not None:
        q = [Rm @ p for p in pts]
        lo_ = Vector([min(v[i] for v in q) for i in range(3)])
        d = Vector([to_min[i] - lo_[i] if to_min[i] is not None else 0.0 for i in range(3)])
    return Matrix.Translation(d) @ Rm


def placed_bbox(name, G):
    o = bpy.data.objects[name]
    P = [G @ (MLi @ o.matrix_world @ Vector(c) / MM) for c in o.bound_box]
    return Vector([min(p[i] for p in P) for i in range(3)]), Vector([max(p[i] for p in P) for i in range(3)])


# UBEC-8A (user 2026-11-28, replaces the Mini560: 8 A continuous for the 15 servos, the Mini560 ran at its ~3 A limit). Listing:
# 55 x 28 x 10, 36 g, 6-36 V in, 5.2 / 6.0 / 7.4 / 8.4 V by a jumper, two output leads in parallel, an output switch on a lead. Inside
# the box there is no 55 mm of free floor (Uno headers / plugs, IMU, Qwiic), so it rides OUTSIDE the top box's rear wall in a POD
# printed with the container: the only place screened clear of every leg / neck / head motion (>= 16 mm over 1469 in-limit poses;
# on the lid the jaw passes over it, under the box or beside it the feet reach) and 23.5 mm off the floor in SIT. Flat against the
# wall with a 5 mm channel behind it (wires, air), vents in the pod's back, floor and the lid's extension over it; the lid holds it
# down (no screws). Wires: IN at the +y end (with the jumper header), OUT at the -y end; both into the box through open-top notches in
# the rear wall (laid in from above). Sizes are the listing's - ESTIMATES until measured (ARGS "ubec").
UB = dict(L=55.0, H=28.0, D=10.0, hdr=(5.0, 3.0, 10.0), ch=5.0, end_out=6.0, end_in=8.0)
UB.update(args.get("ubec", {}))
WO = TOP["x"][0]                                                     # rear wall, outer face (-93)
PF1 = TOP["z"][0] + 1.2; PF0 = PF1 - 1.6                             # pod floor (top 124.2)
UBX = (WO - UB["ch"] - UB["D"], WO - UB["ch"])                       # body x -108 .. -98
UBY = (CY - UB["L"] / 2, CY + UB["L"] / 2)
UBZ = (PF1, PF1 + UB["H"])                                           # 124.2 .. 152.2 (lid 155: its ribs press at 152.5)
POD = dict(x=(UBX[0] - 0.3 - 1.6, WO), y=(UBY[0] - UB["end_out"] - 1.6, UBY[1] + UB["end_in"] + 1.6))
PIN = (POD["x"][0] + 1.6, POD["y"][0] + 1.6, POD["y"][1] - 1.6)      # pod inside: back plate's inner face, the two end walls
NOTCH = {"out": (CY - 22.3, CY - 17.3), "in": (CY + 16.8, CY + 21.8)}   # rear-wall notches (y), between the lid-screw pillars
VENT_Y = [CY - 24.0 + 5.0 * k for k in range(11)]                    # vent slot centres (y), 2 wide
ADP = dict(x0=-45.4, x1=-43.8, y0=-116.51, y1=-83.49, z0=58.89, z1=100.91)  # the bus adapter's PCB (exact board, v1 place)
BAT = placed_bbox("battery_3s", Matrix.Identity(4))

# ================================================================== the container
cx, cy, cz, tx, ty, tz = COL["x"], COL["y"], COL["z"], TOP["x"], TOP["y"], TOP["z"]
gx, gy, gz = TONGUE["x"], TONGUE["y"], TONGUE["z"]
PIL = 6.0                                                            # corner pillars for the lid screws (part of the walls)
lid_screws = [(tx[0] + PIL / 2, ty[0] + PIL / 2), (tx[0] + PIL / 2, ty[1] - PIL / 2), (tx[1] - PIL / 2, ty[0] + PIL / 2), (tx[1] - PIL / 2, ty[1] - PIL / 2)]
KI = KEEL["hw"] - KEEL["t"]                                           # keel inner half-width 16.9
UI = 22.0 - T                                                         # upper column inner half-width 20
adds = [("box", cx[0], cx[1], CY - KEEL["hw"], CY + KEEL["hw"], cz[0], KEEL["z2"]),
        ("hull", [(x, CY + s_ * w, z) for x in cx for s_ in (-1, 1) for w, z in ((KEEL["hw"], KEEL["z1"]), (22.0, KEEL["z2"]))]),
        ("box", cx[0], cx[1], cy[0], cy[1], KEEL["z2"], tz[0] + T), ("box", tx[0], tx[1], ty[0], ty[1], tz[0], tz[1]),
        ("box", gx[0], gx[1], gy[0], gy[1], gz[0], gz[1]), ("box", cx[1] - 1.0, gx[0] + 0.5, gy[0], gy[1], gz[0], 100.0)]   # tongue + its web
adds += [("box", x - PIL / 2, x + PIL / 2, y - PIL / 2, y + PIL / 2, tz[0], tz[1]) for x, y in lid_screws]
adds.append(("box", POD["x"][0], WO + 0.5, POD["y"][0], POD["y"][1], PF0, tz[1]))   # UBEC pod (hollowed below)
subs = [("box", cx[0] + T, cx[1] - T, CY - KI, CY + KI, cz[0] + KEEL["floor"], KEEL["z2"] + KEEL["t"]),   # keel inside
        ("hull", [(x, CY + s_ * w, z) for x in (cx[0] + T, cx[1] - T) for s_ in (-1, 1) for w, z in ((KI, KEEL["z1"] + KEEL["t"]), (UI, KEEL["z2"] + KEEL["t"] + 0.01))]),
        ("box", cx[0] + T, cx[1] - T, CY - UI, CY + UI, KEEL["z2"] + KEEL["t"], tz[0] + T + 0.5),             # column inside, open to the top box
        ("box", tx[0] + T, tx[1] - T, ty[0] + PIL, ty[1] - PIL, tz[0] + T, tz[1] + 1.0),               # top box inside round the pillars
        ("box", tx[0] + PIL, tx[1] - PIL, ty[0] + T, ty[1] - T, tz[0] + T, tz[1] + 1.0)]
subs += [("cyl", x, y, tz[1] - 9.0, tz[1] + 1.0, M2_PILOT) for x, y in lid_screws]                 # lid screw pilots, 9 deep
# UBEC pod: inside (open at the top, the lid closes it), the two wire notches through the rear wall (open to the top: the leads are laid
# in before the UBEC drops in), vents: tall slots in the back plate (bridges 2 mm when printed upright), slots in the floor under the body
subs += [("box", PIN[0], WO, PIN[1], PIN[2], PF1, tz[1] + 1.0)]
subs += [("box", WO - 1.0, WO + T + 1.0, y0, y1, 128.0, tz[1] + 1.0) for y0, y1 in NOTCH.values()]
subs += [("box", POD["x"][0] - 1.0, PIN[0] + 0.5, y - 1.0, y + 1.0, PF1 + 3.0, tz[1] - 3.0) for y in VENT_Y]
subs += [("box", UBX[0] + 1.5, UBX[1] - 1.5, y - 1.0, y + 1.0, PF0 - 1.0, PF1 + 0.5) for y in VENT_Y]
subs += [("cyl", x, y, gz[0] - 1.0, gz[1] + 1.0, M2_CLEAR) for x, y in DECK_SCREWS]                # deck screws through the tongue
subs += [("cyl", x, y, gz[0] - 1.0, gz[0] + 1.0, M2_CSK) for x, y in DECK_SCREWS]                  # their countersinks (blocked out)
# power leads out of the LEFT wall beside the battery's top (battery lead, balance lead, the robot pigtail back in), one slot: the
# pack's own leads are only ~8-10 cm (BOM row 4), the rear wall was 106 mm away; outside the left wall is free of every motion for
# x < -24 (swept space)
LEAD_SLOT = ("box", -34.0, -24.0, ty[1] - T - 1.0, ty[1] + 1.0, tz[0] + T + 0.5, tz[0] + T + 10.5)
# switch: U-slot in the right wall (open to the top, closed by the lid's tongue), frame between two ribs
fx, fy, fz = SW["frame"]
bd, bl = SW["bushing"]
SWC = Vector((-22.5, ty[0] + T + fy / 2 + 0.3, tz[0] + T + 1.0 + fz / 2))   # frame centre; right wall, under the Uno's front end: its ribs
#   and stop bar clear the servo adapter's way down into the column (path check 2026-11-27: at x -31 they hung 0.6 into it); -22.5:
#   its frame clears the Uno's front stop on the way down (0.3)
USLOT = bd + 0.6
subs += [LEAD_SLOT, ("cyl", SWC.x, SWC.z, ty[0] - 1.0, ty[0] + T + 1.0, USLOT, "Y"),
         ("box", SWC.x - USLOT / 2, SWC.x + USLOT / 2, ty[0] - 1.0, ty[0] + T + 1.0, SWC.z, tz[1] + 1.0)]
# relief in the keel's +y wall for the adapter's power edge (DC jack, screw terminal, USB-C stand 0.8 past the board; v1's): 0.6 of
# wall left there, between the adapter's two slots
subs.append(("box", ADP["x0"] - 2.3, ADP["x1"] + 12.0, CY + KI - 0.5, CY + KI + 1.0, ADP["z0"] + 4.7, ADP["z0"] + 37.4))
# camera cable notch (right wall, under the lid)
CAMX = cam_end.x
subs += [("box", CAMX - 2.2, CAMX + 2.2, ty[0] - 1.0, ty[0] + T + 1.0, cam_end.z - 2.2, tz[1] + 1.0),
         ("box", cx[1] - T - 1.0, tx[1] + 1.0, CY - 7.0, CY + 0.5, 117.0, tz[0] + 1.0)]          # servo cables out of the column's front (v1
#                                                                                                path; 1 mm up into the floor's underside)
# post: Uno ledges (45 deg underneath, print without support), switch ribs + stop bar
def ledge(x0, x1, yw, yi):
    return ("hull", [(x, y, z) for x in (x0, x1) for y, z in ((yw, UNO_Z - 0.8 - 0.05 - 2.7), (yw, UNO_Z - 0.85), (yi, UNO_Z - 0.85), (yi, UNO_Z - 0.85 - 1.15))])
RX0, RX1 = SWC.x - fx / 2 - 0.3 - 1.6, SWC.x + fx / 2 + 0.3 + 1.6
post = [ledge(-85.0, RX0 - 0.1, ty[0] + T - 0.2, uy0 + 0.8), ledge(-85.0, X_F + 0.3, ty[1] - T + 0.2, uy1 - 0.8)]
if RX1 + 0.1 < X_F - 1.0:
    post.append(ledge(RX1 + 0.1, X_F + 0.3, ty[0] + T - 0.2, uy0 + 0.8))
yb = SWC.y + fy / 2
post += [("box", RX0, RX0 + 1.6, ty[0] + T - 0.1, yb + 1.5, tz[0] + T - 0.1, SWC.z + fz / 2),
         ("box", RX1 - 1.6, RX1, ty[0] + T - 0.1, yb + 1.5, tz[0] + T - 0.1, SWC.z + fz / 2),
         ("box", RX0, RX1, yb + 0.2, yb + 1.5, tz[0] + T - 0.1, tz[0] + T + 2.3)]                        # stop bar behind the frame
# IMU holder (user 2026-11-27: the v1 pegs did not hold it): the BNO086 board (27.89 x 20.87) stands on its end on the column's
# rear wall, its two long edges in vertical grooves, slid down from the top to a stop. Each groove narrows from 1.9 to 1.55 over its
# lowest 3 mm, so the 1.6 PCB wedges in (no screw, no glue); the Qwiic socket at its top end (plug from above). Back flanges 2.45
# deep: the wires to the adapter run down between the grooves.
for yE, s_ in ((IMU_Y[0], -1.0), (IMU_Y[1], 1.0)):                   # each long edge, s_ = outward
    yo, yin = yE + s_ * 0.3, yE - s_ * 1.0                           # groove's outer face, the lips' inner edge
    yw = yo + s_ * 1.3                                               # side wall's outer face
    xw, xl = cx[0] + T - 0.1, PX1 + 0.15 + 1.0                       # into the wall, the lip's front face
    post += [("box", xw, PX0 - 0.15, *sorted((yw, yin)), IMU_CH[0], IMU_CH[1]),                    # back flange
             ("box", xw, xl, *sorted((yo, yw)), IMU_CH[0], IMU_CH[1]),                              # side wall
             ("hull", [(x, y, z) for y in (yo, yin) for x, z in ((PX1 + 0.15, IMU_CH[1]), (xl, IMU_CH[1]), (PX1 - 0.05, IMU_Z0 + 3.0),
                                                                    (PX1 - 0.05, IMU_CH[0]), (xl, IMU_CH[0]))]),   # front lip, tapered
             ("box", PX0 - 0.15, PX1 + 0.15, *sorted((yo, yin)), IMU_CH[0], IMU_Z0),                # stop under the PCB edge
             ("hull", [(x, y, z) for y in (yw, yin) for x, z in ((xw, IMU_CH[0]), (PX0 - 0.15, IMU_CH[0]), (xw, IMU_CH[0] - 2.5))])]  # 45 deg under it
# Servo bus adapter: its long edges in slots between two ribs on each side wall at the BOTTOM only (to +4.7: its parts reach the edges
# above that, and they pass any higher slot on the way down - v1's top slots were dropped, path check 2026-11-27). Its top is held by
# the IMU's grooves behind it (0.25), the IMU's bottom socket over it (0.6) and a foam pad to the battery in front
for sgn in (-1, 1):
    for za, zb, wi in ((cz[0] + KEEL["floor"] - 0.3, ADP["z0"] + 4.7, KI),):
        ye = sorted((CY + sgn * (16.5 - 0.8), CY + sgn * (wi + 0.3)))
        post += [("box", cx[0] + T - 0.3, ADP["x0"] - 0.15, *ye, za, zb), ("box", ADP["x1"] + 0.15, ADP["x1"] + 1.4, *ye, za, zb)]
# Battery (60 x 30 x 20, standing): a channel of two side ribs, 0.2 clear, two crush ridges per side (0.15 into the pack - holds it
# by friction, soft pack), a low rear stop on the +y side (the servo cables rise on the -y side)
for sgn in (-1, 1):
    face = CY + sgn * (15.0 + 0.2)
    post.append(("box", -29.4, -9.5, *sorted((face, CY + sgn * (KI + 0.3))), cz[0] + KEEL["floor"] - 0.3, 100.0))
    post.append(("box", -29.4, -9.5, *sorted((CY + sgn * (KI - 0.5), CY + sgn * (UI + 0.3))), KEEL["z2"] + KEEL["t"] - 0.5, 100.0))   # ties it to the upper wall
    for xr in (-23.0, -13.0):
        post.append(("box", xr - 0.4, xr + 0.4, *sorted((face, face - sgn * 0.35)), cz[0] + KEEL["floor"] + 3.0, 80.0))   # grip the last ~20 mm only
post.append(("box", -29.4, BAT[0].x - 0.3, CY + 11.0, CY + 15.3, cz[0] + KEEL["floor"] - 0.3, 72.0))
# UBEC in its pod: two standoff ribs on the rear wall keep the 5 mm channel (0.3 to the body), low end stops at the back plate
# (under the leads, which leave the ends at mid height)
post += [("box", UBX[1] + 0.3, WO + 0.1, y - 0.8, y + 0.8, PF1 - 0.1, tz[1] - 0.5) for y in (CY - 12.0, CY + 12.0)]
post += [("box", PIN[0] - 0.1, UBX[0] + 3.5, *ys, PF1 - 0.1, PF1 + 4.0) for ys in ((UBY[0] - 1.8, UBY[0] - 0.3), (UBY[1] + 0.3, UBY[1] + 1.8))]
# Uno front stops: on the two ledges just past the board's USB-C edge (pulling the camera plug cannot drag the board forward)
for yw, yi in ((ty[0] + T - 0.2, uy0 + 0.8), (ty[1] - T + 0.2, uy1 - 0.8)):
    post.append(("box", X_F + 0.3, X_F + 1.8, *sorted((yw, yi)), UNO_Z - 0.85 - 2.7, UNO_Z + 0.8))
cont = solid("bay_v2_container", adds, subs, (0.45, 0.62, 0.8, 1), post)

# ================================================================== the lid: flat plate (prints top down, no supports), ribs pressing the Uno, switch tongue
assert max(AD[1].z, PL[1].z) <= LID[0] - 0.5, "camera adapter would reach the lid"
lid_adds = [("box", tx[0], tx[1], ty[0], ty[1], LID[0], LID[1]),
            ("box", -84.0, X_F - 2.0, uy0 + 0.2, uy0 + 1.8, UNO_Z + 0.85, LID[0] + 0.01),
            ("box", -84.0, X_F - 2.0, uy1 - 1.8, uy1 - 0.2, UNO_Z + 0.85, LID[0] + 0.01),
            ("box", SWC.x - USLOT / 2 + 0.2, SWC.x + USLOT / 2 - 0.2, ty[0] + 0.2, ty[0] + T - 0.2, SWC.z + bd / 2 + 0.4, LID[0] + 0.01)]
# over the UBEC pod: the lid runs on to the pod's back, two ribs press the UBEC down (0.3), vent slots (the warm air out)
lid_adds += [("box", POD["x"][0], tx[0] + 1.0, POD["y"][0], POD["y"][1], LID[0], LID[1])]
lid_adds += [("box", UBX[0] + 0.5, UBX[1] - 0.5, y - 0.8, y + 0.8, UBZ[1] + 0.3, LID[0] + 0.01) for y in (CY - 16.5, CY + 18.5)]
lid_subs = [("cyl", x, y, LID[0] - 1.0, LID[1] + 1.0, M2_CLEAR) for x, y in lid_screws]
lid_subs += [("box", UBX[0] + 1.0, WO - 0.5, y - 1.0, y + 1.0, LID[0] - 1.0, LID[1] + 1.0) for y in VENT_Y]
lid = solid("bay_v2_lid", lid_adds, lid_subs, (0.55, 0.72, 0.88, 1))
# lid screws: M2 x 6 button head (ISO 7380, head 3.5 x 1.3 - the horn / ear screw), 4 mm into the pillars' 2.0 pilots
part_obj("v2_lid_screws_M2x6", [s for x, y in lid_screws for s in (("cyl", x, y, LID[1], LID[1] + 1.3, 3.5), ("cyl", x, y, LID[1] - 6.0, LID[1], 2.0))],
         (0.6, 0.6, 0.62, 1))


# ================================================================== the parts (linked copies), group transforms in layout mm
def copy_with(names, G, prefix="v2_"):
    W = ML @ S @ G @ S.inverted() @ MLi
    for n in names:
        o = bpy.data.objects[n]
        cpy = o.copy()
        cpy.name = prefix + n
        cpy.parent = None
        coll.objects.link(cpy)
        cpy.matrix_world = W @ o.matrix_world
        cpy.hide_set(False)
        cpy.hide_viewport = False


def place(names, d=(0, 0, 0), rot=None, to_min=None):
    objs = [bpy.data.objects[n] for n in names if n in bpy.data.objects]
    pts = [MLi @ o.matrix_world @ Vector(c) / MM for o in objs for c in o.bound_box]
    c = sum(pts, Vector()) / len(pts)
    Rm = Matrix.Identity(4)
    for ax, deg in (rot or []):
        Rm = Matrix.Translation(c) @ Matrix.Rotation(math.radians(deg), 4, ax) @ Matrix.Translation(-c) @ Rm
    if to_min is not None:
        q = [Rm @ p for p in pts]
        lo_ = Vector([min(v[i] for v in q) for i in range(3)])
        d = Vector([to_min[i] - lo_[i] if to_min[i] is not None else 0.0 for i in range(3)])
    G = Matrix.Translation(Vector(d)) @ Rm
    copy_with([o.name for o in objs], G)
    return G


# column parts + the two servo cables to the legs / neck: v1 places (identity)
for n in ("battery_3s", "servo_adapter"):
    place([n])
# IMU stood on its end (the used Qwiic socket at the top): the v1 socket / plug stand-in turned with it, the board drawn exact
# (PCB 1.6 + a parts zone 1.2 high, 2.5 in from the edges: the grooves' lips cover the outer 1.0 only)
_g = [bpy.data.objects[n] for n in ("imu", "bay_imu_qwiic")]
_c = sum((MLi @ o.matrix_world @ Vector(cc) / MM for o in _g for cc in o.bound_box), Vector()) / 16
R_i = Matrix.Translation(_c) @ Matrix.Rotation(math.radians(-90.0), 4, "X") @ Matrix.Translation(-_c)
_q = [R_i @ (MLi @ _g[0].matrix_world @ Vector(cc) / MM) for cc in _g[0].bound_box]
_lo = Vector([min(v[i] for v in _q) for i in range(3)])
G_imu = Matrix.Translation(Vector((PX0, IMU_Y[0], IMU_Z0)) - _lo) @ R_i
copy_with(["bay_imu_qwiic"], G_imu)
_top = IMU_Z0 + IMU_L
part_obj("v2_imu", [("box", PX0, PX1, IMU_Y[0], IMU_Y[1], IMU_Z0, _top),
                    ("box", PX1, PX1 + 1.2, IMU_Y[0] + 2.5, IMU_Y[1] - 2.5, IMU_Z0 + 3.0, _top - 3.0)], tuple(bpy.data.objects["imu"].color))
SERVO_CABLES = [n for n in ("wire_ext_adapter_to_gap", "wire_ext_left_body_to_thigh") if n in bpy.data.objects]
copy_with(SERVO_CABLES, Matrix.Identity(4))
# Uno Q + its Qwiic plug; the camera adapter + USB-A plug
copy_with(["unoq_pcb", "unoq_headers", "unoq_usbc", "unoq_pins", "unoq_bottom_connectors", "unoq_top_parts", "unoq_qwiic_plug"], G_uno)
copy_with(["bay_usb_adapter_L", "bay_usb_a_plug"], G_cam)
hdr = bpy.data.objects["unoq_headers"]
UPIN = {k: apply(G_uno, Vector(v)) for k, v in hdr["pins"].items()}
QM = apply(G_uno, Vector(bpy.data.objects["unoq_pins"]["qwiic_mouth"]))
QAX = (G_uno.to_3x3() @ Vector(bpy.data.objects["unoq_pins"]["qwiic_axis"])).normalized()
# UBEC in its pod outside the rear wall (body + the voltage-jumper header at its IN end); splices by the column's top
part_obj("v2_ubec", [("box", UBX[0], UBX[1], UBY[0], UBY[1], UBZ[0], UBZ[1])], (0.06, 0.06, 0.07, 1))
hx, ht, hz = UB["hdr"]
part_obj("v2_ubec_jumper", [("box", UBX[1] - 1.0 - ht, UBX[1] - 1.0, UBY[1] - 0.1, UBY[1] + hx, UBZ[1] - 4.0 - hz, UBZ[1] - 4.0)], (0.95, 0.75, 0.1, 1))
_xm = (UBX[0] + UBX[1]) / 2
UBP = {"in_pos": Vector((_xm - 1.2, UBY[1], 131.0)), "in_neg": Vector((_xm + 1.2, UBY[1], 131.0)),
       "out_pos": Vector((_xm - 1.2, UBY[0], 131.0)), "out_neg": Vector((_xm + 1.2, UBY[0], 131.0))}
G_s = place(["splices_x3"], rot=[("Z", 90.0), ("X", 90.0)], to_min=(-34.0, CY - 15.6, 132.0))   # lying flat over the switch housings
SP = {k: apply(G_s, Vector(v)) for k, v in bpy.data.objects["splices_x3"]["anchors"].items()}
# XT30 pair + balance plug outside the rear wall, by the lead slot (tied to it)
G_x = place(["bay_xt30_pair"], rot=[("Y", 90.0), ("X", 90.0)], to_min=(-58.0, ty[1] + 0.4, tz[0] + T + 0.5))   # lying along the left wall, battery end at the slot
G_b = place(["bay_balance_plug"], rot=[("Z", 90.0)], to_min=(-23.0, ty[1] + 0.4, 130.0))   # standing just in front of the slot, wires down
XA = {k: apply(G_x, Vector(v)) for k, v in bpy.data.objects["bay_xt30_pair"]["anchors"].items()}
BA = apply(G_b, Vector(bpy.data.objects["bay_balance_plug"]["anchors"]["wires"]))

# ---- the switch (photo estimates): frame, bushing through the wall's U-slot, lever, 3 pins + 2 frame legs, Dupont housings on COM / ON
y_face, y_back = SWC.y - fy / 2, SWC.y + fy / 2
part_obj("v2_switch_frame", [("box", SWC.x - fx / 2, SWC.x + fx / 2, y_face, y_back, SWC.z - fz / 2, SWC.z + fz / 2)], (0.75, 0.75, 0.78, 1))
part_obj("v2_switch_bushing", [("cyl", SWC.x, SWC.z, y_face - bl, y_face, bd, "Y")], (0.8, 0.8, 0.82, 1))
part_obj("v2_switch_lever", [("cyl", SWC.x, SWC.z, y_face - bl - SW["lever"][1], y_face - bl + 0.5, SW["lever"][0], "Y", 16, SW["lever"][0] * 1.4)], (0.85, 0.85, 0.87, 1))
pitch, plen = SW["pins"]
part_obj("v2_switch_pins", [("box", SWC.x + k * pitch - 0.4, SWC.x + k * pitch + 0.4, y_back, y_back + plen, SWC.z - 0.25, SWC.z + 0.25) for k in (-1, 0, 1)] +
         [("box", SWC.x + s_ * (fx / 2 - 0.6) - 0.3, SWC.x + s_ * (fx / 2 - 0.6) + 0.3, y_back, y_back + 2.5, SWC.z - 1.0, SWC.z + 1.0) for s_ in (-1, 1)], (0.85, 0.75, 0.4, 1))
SWP = {}
for k, xo in (("com", 0.0), ("on", -pitch)):
    part_obj(f"v2_switch_housing_{k}", [("box", SWC.x + xo - 1.27, SWC.x + xo + 1.27, y_back + 0.5, y_back + 14.5, SWC.z - 1.27, SWC.z + 1.27)], (0.08, 0.08, 0.08, 1))
    SWP[k] = Vector((SWC.x + xo, y_back + 14.5, SWC.z))


# ================================================================== wires: tubes through waypoints (layout mm)
def tube(name, pts, d, rgba):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions, cu.bevel_depth, cu.bevel_resolution, cu.use_fill_caps = "3D", d / 2 * MM, 3, True
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for p_, q in zip(sp.points, pts):
        p_.co = (*(ML @ (Vector(q) * MM)), 1.0)
    o = bpy.data.objects.new(name, cu)
    coll.objects.link(o)
    cu.materials.append(mat(name, rgba))
    o.color = rgba
    L = sum(((Vector(b) - Vector(a)).length for a, b in zip(pts, pts[1:])))
    o["length_mm"] = round(L, 1)
    return L


RED, BLK, YEL, GRN, BLU, ORG = (0.85, 0.1, 0.1, 1), (0.05, 0.05, 0.05, 1), (0.95, 0.85, 0.1, 1), (0.1, 0.75, 0.3, 1), (0.15, 0.3, 0.85, 1), (0.95, 0.55, 0.1, 1)
V = Vector
uart = V(bpy.data.objects["servo_adapter"]["uart_back_xyz"])
term = V(bpy.data.objects["servo_adapter"]["term_back_xyz"])
imu_q = apply(G_imu, V(bpy.data.objects["bay_imu_qwiic"]["anchors"]["cable"]))
bat_top = V((-17.9, CY - 8.0, 119.0))
route = {}

# ---- the 5 jumpers: a 1-pin Dupont plug 10 tall on the header (pins face down now), the wire bending 90 deg (r 2) toward `dir`
DN = V((0, 0, -1))
JUMP = {"D0_RX": (0.4, -1, 0), "D1_TX": (1, 0, 0), "GND_D": (0, -1, 0), "VIN": (0, 1, 0), "GND_P2": (1, 0, 0)}
JUMP.update({k: tuple(v) for k, v in args.get("jumpers", {}).items()})
arc = {}
for n, dvec in JUMP.items():
    top = UPIN[n]
    part_obj(f"v2_plug_{n}", [("box", top.x - 1.27, top.x + 1.27, top.y - 1.27, top.y + 1.27, top.z - 10.0, top.z)], (0.08, 0.08, 0.08, 1))
    d = V(dvec).normalized()
    s = top + DN * 10.4
    arc[n] = [top + DN * 10.0, s] + [s + d * (2.0 * (1 - math.cos(math.pi / 2 * k / 6))) + DN * (2.0 * math.sin(math.pi / 2 * k / 6)) for k in range(1, 7)]
FL = tz[0] + T                                                       # top box floor (top face)
to_sp = lambda k: [V((SP[k].x, SP[k].y, SP[k].z + 4.0)), SP[k]]      # into a splice from above
# UART (D0, D1, GND): down the column BESIDE the IMU (its left, y -85..-87, x -49..-50: between its groove and the side wall), under it
# across behind the servo adapter (x -49.7) to the adapter's UART header (back side)
route["D0_RX"] = arc["D0_RX"] + [V((-76.5, -78.8, FL + 3.4)), V((-53.0, -78.8, FL + 3.4)), V((-50.3, -85.2, FL + 2.0)), V((-50.3, -85.2, 97.5)),
                                 V((-49.7, uart.y + 1.5, 96.5)), uart + V((0, 1.5, 0))]
route["D1_TX"] = arc["D1_TX"] + [V((-73.0, -77.0, FL + 2.6)), V((-53.0, -77.0, FL + 2.6)), V((-48.9, -85.2, FL + 2.0)), V((-48.9, -85.2, 97.5)),
                                 V((-49.0, uart.y + 3.0, 96.5)), uart + V((0, 3.0, 0))]
route["GND_D"] = arc["GND_D"] + [V((-46.0, -84.0, FL + 1.5)), V((-49.6, -86.9, FL + 1.0)), V((-49.6, -86.9, 97.5)), V((-49.7, uart.y, 96.5)), uart]
# Uno power: VIN <- splice SW+, GND <- splice BAT-
route["VIN"] = arc["VIN"] + [V((-50.5, -122.2, FL + 1.2)), V((-45.0, -118.0, 134.0))] + to_sp("sw_out")
route["GND_P2"] = arc["GND_P2"] + [V((-51.0, -124.1, FL + 1.2)), V((-45.5, -120.0, 135.0))] + to_sp("bat_neg")
# UBEC: IN+ <- SW+, IN- <- BAT- (its +y end: round the end into the channel, in through the left notch, over to the splices);
# OUT+/- = BOTH output leads twisted together per pole (its 8 A rating is for the two in parallel; plugs cut off, stripped) -> the
# adapter's power terminal: round the -y end, in through the right notch, over the IMU's right groove, down beside it (y -113..-114),
# under it across behind the adapter (Mini560's path)
for k, sp_, xo in (("in_pos", "sw_out", -1.2), ("in_neg", "bat_neg", 1.2)):
    s0 = UBP[k]
    route["U_" + k] = [s0, s0 + V((0, 4.0, 0)), V((WO - 2.5 + xo, s0.y + 4.0, 131.0)), V((WO - 2.5 + xo, NOTCH["in"][0] + 2.5, 131.0)),
                       V((WO + T + 2.0, NOTCH["in"][0] + 2.5 + xo, 131.5)), V((-50.0, -98.0 - xo / 1.2, 139.0))] + to_sp(sp_)
for k, yd, dy, xo in (("out_pos", -115.3, 0.0, -1.0), ("out_neg", -113.3, 1.6, 1.0)):   # 0.27 off the IMU groove, 0.6 off the keel
    s0 = UBP[k]
    yn = NOTCH["out"][0] + 2.5 + xo
    route["U_" + k] = [s0, s0 + V((0, -4.0, 0)), V((WO - 2.5 + xo, s0.y - 4.0, 131.0)), V((WO - 2.5 + xo, yn, 131.0)), V((WO + T + 2.0, yn, 131.2)),
                       V((-51.5, yd - 2.0, 131.2)), V((-49.8, yd, 131.2)), V((-49.8, yd, 97.0)), V((-49.8, yd, 91.0)),
                       V((-49.8, term.y + dy, 88.0)), V((-49.8, term.y + dy, 83.5)), term + V((0, dy, 0))]
# switch: COM <- splice BAT+, ON -> splice SW+ (the splices lie just over the housings)
route["SW_COM"] = [SWP["com"], SWP["com"] + V((0, 2.0, 0.6))] + to_sp("bat_pos")
route["SW_ON"] = [SWP["on"], SWP["on"] + V((0, 2.0, 0.6))] + to_sp("sw_out")
# robot pigtail: from the XT30's robot end (outside), up over it, in through the lead slot, to the splices BAT+ / BAT-
for k, zo, xs in (("PIG_red", 0.0, -30.0), ("PIG_black", 0.0, -26.9)):
    o_ = XA["out_pos" if k == "PIG_red" else "out_neg"]
    route[k] = [o_, o_ + V((-3.0, 0, 0)), V((o_.x - 4.5, ty[1] + 8.5, o_.z)), V((xs, ty[1] + 8.5, FL + 6.0 + zo)), V((xs, ty[1] - 4.0, FL + 6.0 + zo)),
                V((xs, CY + 2.0, 134.0))] + to_sp("bat_pos" if k == "PIG_red" else "bat_neg")
# battery lead + balance lead: from the battery's top along the right of the floor, out of the slot to the XT30 / balance plug
route["BAT_lead"] = [bat_top, bat_top + V((0, 0, 6.0)), V((-22.0, -84.0, FL + 2.6)), V((-27.5, ty[1] - 3.0, FL + 2.6)), V((-27.5, ty[1] + 2.4, FL + 2.6)),
                     XA["bat_pos"] + V((2.5, 0, 0)), XA["bat_pos"]]
route["BAL_lead"] = [bat_top + V((0, 12.0, 0)), bat_top + V((0, 12.0, 6.0)), V((-20.0, -82.0, FL + 5.0)), V((-25.5, ty[1] - 3.0, FL + 7.5)),
                     V((-25.5, ty[1] + 3.0, FL + 7.5)), V((BA.x, BA.y, BA.z - 2.5)), BA]
# Qwiic: the Uno's socket (rear end, under the lid) -> down by the rear wall -> along the right wall -> round the Mini560's end -> down
# onto the IMU's top socket
q0 = QM + QAX * 2.5
route["QWIIC"] = [q0, q0 + QAX * 0.6, q0 + QAX * 2.0 + V((0, 0, -1.2)), q0 + QAX * 2.6 + V((0, 0, -3.0)), V((q0.x + QAX.x * 2.6, QM.y - 1.0, 136.0)),
                  V((-85.5, -122.0, FL + 1.2)), V((-84.0, -125.8, FL + 1.2)), V((-58.0, -125.8, FL + 1.2)), V((-54.5, -116.0, FL + 2.5)), V((-54.5, -106.0, FL + 4.0)),
                  V((-48.0, -99.5, imu_q.z + 1.0)), V((imu_q.x, imu_q.y, imu_q.z + 2.5)), imu_q]
# camera: USB-A plug (inside) -> out through the right wall's notch -> forward to the neck (v1 cable path from there)
route["CAMERA"] = [cam_end, V((cam_end.x, ty[0] - 3.3, cam_end.z)), V((cam_end.x + 3.0, ty[0] - 6.3, cam_end.z - 0.5)),
                   V((2.0, ty[0] - 6.8, 148.5)), V((10.0, -126.0, 147.0)), V((17.5, -107.0, 146.0))]
WD = {"PIG_red": 3.1, "PIG_black": 3.1, "BAT_lead": 3.1, "BAL_lead": 1.4, "QWIIC": 2.2, "CAMERA": 3.8, "U_in_pos": 1.8, "U_in_neg": 1.8,
      "U_out_pos": 2.0, "U_out_neg": 2.0}                                # UBEC leads ~22 AWG (estimate); OUT: the 2 leads twisted per pole
COLS = {"D0_RX": YEL, "D1_TX": GRN, "GND_D": BLK, "VIN": RED, "GND_P2": BLK, "U_in_pos": RED, "U_in_neg": BLK, "U_out_pos": RED,
        "U_out_neg": BLK, "SW_COM": RED, "SW_ON": RED, "PIG_red": RED, "PIG_black": BLK, "BAT_lead": RED, "BAL_lead": ORG, "QWIIC": BLU,
        "CAMERA": (0.6, 0.6, 0.62, 1)}
lens = {}
for n, pts in route.items():
    lens[n] = tube("v2_wire_" + n, [tuple(p) for p in pts], WD.get(n, 1.4), COLS.get(n, (0.6, 0.6, 0.6, 1)))
print("bay v2: container %.1f g, lid %.1f g (solid PLA), non-manifold %d / %d; %d objects"
      % (cont["mass_g_solid_PLA"], lid["mass_g_solid_PLA"], cont["non_manifold"], lid["non_manifold"], len(coll.objects)))
print("wire lengths (mm):", {k: round(v) for k, v in lens.items()})
print("clearances: Qwiic plug end -> rear wall %.1f, USB-A plug -> front-right pillar %.1f (x), board -> rear pillars %.1f, adapter top %.1f / lid %.1f"
      % (q0.x - (tx[0] + T), (tx[1] - PIL) - PL[1].x, (X_F - 68.58) - (tx[0] + PIL), AD[1].z, LID[0]))
if args.get("debug"):
    f = lambda v: tuple(round(c, 1) for c in v)
    print("UBP", {k: f(v) for k, v in UBP.items()}, "\nSP", {k: f(v) for k, v in SP.items()}, "\nXA", {k: f(v) for k, v in XA.items()}, "BA", f(BA))
    print("pins", {k: f(UPIN[k]) for k in JUMP}, "\nQM", f(QM), f(QAX), "mouth", f(mouth), "cam_end", f(cam_end), "AD", f(AD[0]), f(AD[1]), "PL", f(PL[0]), f(PL[1]), "SWC", f(SWC))
    for n in ("v2_ubec", "v2_ubec_jumper", "v2_splices_x3", "v2_unoq_pcb", "v2_unoq_headers", "v2_bay_xt30_pair", "v2_bay_balance_plug", "v2_switch_lever"):
        a, b = bbox_layout(bpy.data.objects[n])
        print(n, f(a), f(b))
