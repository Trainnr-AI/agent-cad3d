"""XGO-Duck head, v2: the ORIGINAL head parts at our head scale (collection orig_head_parts: orig_head = base tray /
upper bill, orig_head_shell = dome, orig_jaw), changed only where our servos need it. Runs inside Blender at the zero pose.

  head_base   original tray. Inside of the rear tub cleared and replaced by the head-roll servo holder (type-C cap as a
              bulkhead standing on the floor: 6702ZZ seat, ear pads down to the floor, block under the servo). Floor
              opening and the right side wall cut where the yaw block / head-pitch U swing in head roll. 4 screw seats
              for the dome.
  head_dome   original dome. Back squared off by a bulge (user decision: the roll servo's rear corner does not fit
              the original's round back), right skirt trimmed where the U swings, 4 screw posts.

Layout coordinates (mm): world = trunk @ (x, y + 100, z - 118.8); front +x, left +y, up +z.
ARGS = {"robot": "duck_scs0009", "collection": "head_shell_v2", "joint_lib": ..., "base_lib": ..., "dims": ...,
        "parts": ["base", "dome"]}
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "head_shell_v2")
PARTS = args.get("parts", ["base", "dome"])
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
HEAD_ROOM_DUMMIES = ("cam_screws_M2x6st", "head_roll_ear_screws_M2x6st")   # M2 socket heads (2.0) the dome closes over (2026-11-13)
PREFIX = {"base": ("head_base", "mouth_ear_screws"), "dome": ("head_dome", "dome_screws"), "jaw": ("head_jaw", "jaw_pivot_screw"),
          "face": ("head_face", "cam_", "light_")}
for o in list(coll.objects):
    if any(o.name.startswith(p) for k in PARTS for p in PREFIX[k]):
        bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()

trunk = bpy.data.objects[R + ":trunk_base"]
T = trunk.matrix_world.copy()
ML = T @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)          # layout frame (mm through the lib's J())
MLi = ML.inverted()
head = bpy.data.objects[R + ":jaw_soft"]                            # the body that rolls
Fr = bpy.data.objects["scs_head_roll"].matrix_world.copy()
ROLL_AX = Vector((0.0, -100.03, 204.59))
HX, HZ = args.get("hinge", (58.5, 210.8))                          # beak hinge (layout x, z)
MY = args.get("mouth_y", -77.8)                                    # beak servo tower top (layout y); shaft toward +y (left wall)
Fm = ML @ Matrix(((1, 0, 0, HX * MM), (0, 0, 1, MY * MM), (0, -1, 0, HZ * MM), (0, 0, 0, 1)))   # servo frame: X forward, Y down, Z left
Fp = ML @ Matrix(((1, 0, 0, HX * MM), (0, 0, -1, 0), (0, 1, 0, HZ * MM), (0, 0, 0, 1)))        # pivot frame: Z toward the right (-y); z = -y
ARM_Y = (MY + L["ARM_Z"][0], MY + L["ARM_Z"][1])                   # left jaw arm = horn plate (layout y range)
ARM_YR = (2 * ROLL_AX.y - ARM_Y[1], 2 * ROLL_AX.y - ARM_Y[0])      # right jaw arm, mirrored
TRIM = L["TRIM_R"]
DISC_R = TRIM + L["HORN_CLEAR"] + 1.8                              # horn plate radius round the trimmed horn
POST_Y = (ARM_YR[1] + 0.36, ARM_YR[1] + 3.86)                      # jaw pivot post, inboard of the right arm
SLOT_L = (50.0, 72.5, ARM_Y[0] - 0.8, ARM_Y[1] + 0.8)              # bill-plate slots for the jaw arms (x0, x1, y0, y1)
SLOT_R = (53.5, 72.5, ARM_YR[0] - 0.75, POST_Y[0])
report = []
box, cyl, hull, new = L["add_box"], L["add_cyl"], L["add_hull"], L["new_bm"]


def lay_bm(o, tri=True):
    """object -> bmesh in layout mm"""
    bm = bmesh.new()
    bm.from_mesh(o.data)
    if tri:
        bmesh.ops.triangulate(bm, faces=bm.faces)  # before the frame change (n-gons with holes triangulate wrongly once rotated)
    bm.transform(Matrix.Scale(1000, 4) @ MLi @ o.matrix_world)
    return bm


def world_bm(o):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    return bm


def fl(M, p):
    """frame M (mm) -> layout mm"""
    return (MLi @ (M @ (Vector(p) * MM))) / MM


def drop_loose(part):
    """remove loose leftovers of the original (everything but the largest connected piece); returns what was dropped"""
    bm = bmesh.new()
    bm.from_mesh(part.data)
    seen, comps = set(), []
    for v in bm.verts:
        if v.index in seen:
            continue
        st, comp = [v], [v]
        seen.add(v.index)
        while st:
            w = st.pop()
            for e in w.link_edges:
                u = e.other_vert(w)
                if u.index not in seen:
                    seen.add(u.index)
                    st.append(u)
                    comp.append(u)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    out = []
    for comp in comps[1:]:
        P = [(MLi @ (part.matrix_world @ v.co)) / MM for v in comp]
        out.append("%d verts at x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f" % (len(comp), min(p.x for p in P), max(p.x for p in P), min(p.y for p in P),
                                                                      max(p.y for p in P), min(p.z for p in P), max(p.z for p in P)))
        bmesh.ops.delete(bm, geom=comp, context="VERTS")
    bm.to_mesh(part.data)
    bm.free()
    return out


def finish(name, part):
    for d in drop_loose(part):
        print(f"  {name}: dropped a loose leftover, {d}")
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["manifold"] = ok
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    mw = part.matrix_world.copy()
    part.parent = head
    part.matrix_world = mw
    return part


def first_hit(tree, p, d):
    return tree.ray_cast(Vector(p), Vector(d))[0]


# ------------------------------------------------------------------ what the head rolls around (yaw block, U, neck)
def under(o, p):
    while o:
        if o == p:
            return True
        o = o.parent
    return False


def fixed_objects():
    out = []
    for o in bpy.context.view_layer.objects:
        if o.type != "MESH" or under(o, head) or o.hide_get() or o.name.startswith(("wm_", "wire_", "cable_", R + ":", "xgoduck_original:", "fit_", "orig_", "row_")):
            continue
        if any(c.name in ("orig_head_parts", "orig_head_row", "head_fit", "foot_compare", "thigh_compare", COLL) or "explod" in c.name for c in o.users_collection):
            continue
        P = [(MLi @ (o.matrix_world @ Vector(c))) / MM for c in o.bound_box]
        if max(p.z for p in P) < 150 or min(p.x for p in P) > 90 or max(p.x for p in P) < -25 or min(p.y for p in P) > -50 or max(p.y for p in P) < -150:
            continue
        out.append(o)
    return out


def swept_points(step=0.8):
    """points on the fixed parts, in the HEAD's frame for head roll -25..25 (layout mm, N x 3)"""
    pts = []
    for o in fixed_objects():
        bm = lay_bm(o)
        for f in bm.faces:
            a, b, c = (v.co for v in f.verts)
            n = max(1, int(max((b - a).length, (c - a).length, (c - b).length) / step))
            for i in range(n + 1):
                for j in range(n + 1 - i):
                    pts.append(a + (b - a) * (i / n) + (c - a) * (j / n))
        bm.free()
    P = np.array([p[:] for p in pts])
    P = P[(P[:, 2] > 150) & (P[:, 0] > 10) & (P[:, 0] < 60)]
    lo, hi = head["joint_range"]
    out = []
    for ang in np.arange(math.degrees(lo), math.degrees(hi) + 0.01, 1.0):
        a = math.radians(-ang)                                   # the fixed parts turn the other way in the head's frame
        y, z = P[:, 1] - ROLL_AX.y, P[:, 2] - ROLL_AX.z
        out.append(np.stack([P[:, 0], ROLL_AX.y + y * math.cos(a) - z * math.sin(a), ROLL_AX.z + y * math.sin(a) + z * math.cos(a)], axis=1))
    return np.concatenate(out)


SW = swept_points()
m = (SW[:, 2] > 190.5) & (SW[:, 2] < 204.5)
OPEN_X1 = float(SW[m][:, 0].max()) + 0.8                           # front edge of the floor opening
OPEN_Y1 = float(SW[m][:, 1].max()) + 0.8                           # its left edge (the left wall stays)
mr = (SW[:, 1] < -127.4) & (SW[:, 2] > 190.5)
WALL_Z = float(SW[mr][:, 2].max()) + 0.8                           # the right wall must be open up to here
WALL_X = (float(SW[mr][:, 0].min()) - 0.8, float(SW[mr][:, 0].max()) + 0.8)
print('fixed parts:', len(fixed_objects()))
print(f"swing of the neck-side parts in the head: floor opening to x {OPEN_X1:.1f}, y {OPEN_Y1:.1f}; right wall open x {WALL_X[0]:.1f}..{WALL_X[1]:.1f} up to z {WALL_Z:.1f}; "
      f"reach right y {SW[mr][:, 1].min():.1f}, left y {SW[SW[:, 2] > 190.5][:, 1].max():.1f}")
OPEN_X0 = fl(Fr, (0, 0, L["CAP_T"])).x - 0.02                      # from the roll cap plate's front face (the yaw block's plate is 0.4 ahead of it)

# ------------------------------------------------------------------ dome screws (4): layout x, y
REAR_SCREWS = [(5.0, -100.03 - 22.2), (5.0, -100.03 + 22.2)]
FLOOR_TOP = 193.3
PLATE = (199.9, 202.3)                                             # upper bill plate (original)
DOME_Z0 = 197.9


def front_screws(tree):
    """centres of the original's two front screw holes in the bill plate"""
    out = []
    for y0 in (-131.2, -68.8):
        cells = [(x, y) for x in np.arange(71.5, 76.01, 0.1) for y in np.arange(y0 - 1.6, y0 + 1.61, 0.1)
                 if first_hit(tree, (x, y, 180), (0, 0, 1)) is None]
        cx, cy = (np.mean(cells, axis=0) if cells else (73.5, y0 + (1 if y0 < -100 else -1)))
        out.append((round(float(cx), 2), round(float(cy), 2)))
    return out


# =========================================================== head base (tray)
if "base" in PARTS:
    src = bpy.data.objects["orig_head"]
    lt = lay_bm(src)
    tree = BVHTree.FromBMesh(lt)
    FRONT_SCREWS = front_screws(tree)
    head["front_screws"] = [c for p in FRONT_SCREWS for c in p]
    print("front dome screws at", FRONT_SCREWS)
    # inner outline of the rear tub at z 195: outer skin moved in by the wall (+ the lip the dome sits round)
    TW = 3.4
    pts = []
    for x in np.arange(-11.0, 46.81, 0.5):
        a, b = first_hit(tree, (x, -140, 195), (0, 1, 0)), first_hit(tree, (x, -60, 195), (0, -1, 0))
        if a is None or b is None or b.y - a.y < 2 * TW + 2:
            continue
        for y in (a.y + TW, b.y - TW):
            r = first_hit(tree, (-20, y, 195), (1, 0, 0))
            if r is not None and x >= r.x + TW:
                pts.append((x, y))
    tub = hull(new(), [(x, y, z) for x, y in pts for z in (FLOOR_TOP, 204.5)], ML)
    rim = box(new(), -13.0, 17.0, -118.5, -93.0, 198.3, 206.0, ML)             # nothing of the original above 198.3 under the roll servo (it slides in from behind)
    front = box(new(), 47.0, 79.0, -131.0, -69.0, PLATE[1] + 0.02, 204.6, ML)  # ribs on the bill plate
    opening = box(new(), OPEN_X0, OPEN_X1, -140.0, OPEN_Y1, 185.0, 206.0, ML)
    wall = box(new(), WALL_X[0], max(WALL_X[1], OPEN_X1), -140.0, -126.0, 185.1, 206.1, ML)
    cut = L["combine_cutters"]([tub, rim, front, opening, wall])
    # roll servo holder
    ca, cs = L["cap_features"](Fr, include_plate=False, saddle=False)
    yb = (ROLL_AX.z - (FLOOR_TOP - 0.8))                                       # Fr +Y is down: floor level in the frame
    ring = [(L["CAP_RING_R"] * math.cos(2 * math.pi * i / 48), L["CAP_RING_R"] * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    rect = [(x, y) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], yb)]
    plate = hull(new(), [(x, y, z) for x, y in ring + rect for z in (0.0, L["CAP_T"])], Fr)
    pads = [box(new(), x0, x1, -L["J_EAR_HW"], yb, L["J_EAR_Z"][1], 0.6, Fr)
            for x0, x1 in ((L["J_EAR_X"][0] + 0.05, L["J_BODY_X"][0] - 0.2), (L["J_BODY_X"][1] + 0.2, L["J_EAR_X"][1] - 0.05))]
    rest = box(new(), L["J_BODY_X"][0] - 0.6, L["J_BODY_X"][1] + 0.6, L["J_BODY_HW"] + 0.15, yb - 0.04, -9.0, 0.55, Fr)   # block under the servo
    bosses = [cyl(new(), x, y, 191.4, DOME_Z0 - 0.05, 5.6, 32, ML) for x, y in REAR_SCREWS]
    add = [plate] + pads + [rest] + bosses
    # beak servo (lying flat like the original's, shaft to the left wall): ear pads on the wall side of the ears
    ez = (L["J_EAR_Z"][1], L["J_EAR_Z"][1] + 5.6)
    yd = HZ - (PLATE[1] - 0.3)                                                 # Fm +Y is down: bill-plate top in the frame
    xr0, xr1 = L["J_EAR_X"][0] + 0.05, L["J_BODY_X"][0] - 0.2                  # rear ear zone
    xf0, xf1 = L["J_BODY_X"][1] + 0.2, L["J_EAR_X"][1] - 0.05                  # front ear zone
    add.append(box(new(), xf0, xf1, -L["J_EAR_HW"], yd, ez[0], ez[1], Fm))     # front pad: a column on the bill plate
    add.append(box(new(), xr0, xr1, -L["J_EAR_HW"], L["J_EAR_HW"], ez[0], ez[1] - 0.03, Fm))   # rear pad: over the floor opening ...
    ye, yp, zt, zb = MY + ez[0], MY + ez[1], HZ + L["J_EAR_HW"], HZ - L["J_EAR_HW"]
    web = [(ye, zb), (ye, zt), (yp, zt), (-70.6, 207.0), (-70.6, 192.0), (-75.3, 192.0), (-75.3, zb - (ye + 75.3) * -1.0)]
    add.append(hull(new(), [(x, y, z) for y, z in web for x in (HX + xr0 - 1.75, HX + xr0 + 2.05)], ML))    # ... carried to the left wall by a web (45 deg underside)
    for xa in (-2.5, 11.5):                                                    # two rails under the servo body
        add.append(box(new(), xa, xa + 2.0, L["J_BODY_HW"] + 0.15, HZ - (PLATE[0] + 0.6), L["J_BOTTOM"] - 0.3, ez[0] - 1.5, Fm))   # sunk into the plate (it has a 1.3 deep recess here)
    add.append(hull(new(), [(x, y, z) for x in (SLOT_R[0], SLOT_R[0] + 10.0) for y, z in
                            ((POST_Y[0], PLATE[1] - 0.3), (POST_Y[0], HZ + 4.5), (POST_Y[1], HZ + 4.5), (POST_Y[1] + 4.0, PLATE[1] - 0.3))], ML))   # jaw pivot post
    holes = L["screw_holes"](ML, REAR_SCREWS, 191.3, DOME_Z0 - 0.05 - 191.3, 0.0, +1, csk=False, cbore=DOME_Z0 - 0.05 - 191.3 - 2.0)
    holes += L["screw_holes"](ML, FRONT_SCREWS, PLATE[0], PLATE[1] - PLATE[0], 0.0, +1, csk=False)
    holes += [box(new(), sl[0], sl[1], sl[2], sl[3], 189.0, PLATE[1] + 0.5, ML) for sl in (SLOT_L, SLOT_R)]
    holes += [cyl(new(), hx, 0, L["J_EAR_Z"][1] - 0.5, L["J_EAR_Z"][1] + L["EAR_PILOT_C"], L["PILOT_D"], 16, Fm) for hx in L["J_HOLE_X"]]   # beak servo ear screws
    holes += [cyl(new(), 0, 0, -POST_Y[1] - 0.5, -POST_Y[0] + 0.5, L["CLEAR_D"], 16, Fp),                                    # pivot screw: free in the post
              cyl(new(), 0, 0, -POST_Y[1] - 4.5, -POST_Y[1] + 1.5, 3.9, 20, Fp)]                                             # its head, sunk 1.5
    # assembly relief for the yaw block (2026-11-13): it goes onto the roll servo's horn from the FRONT (3.5 mm of travel to clear
    # the horn) - up through the floor opening 4.3 mm forward, then back. Its front end (an inverted T: wide lower plate, narrower
    # block on top, see yaw_block) stood 2.0 mm from the support under the beak servo. Relief = that T + 0.3, from the block's front
    # to 4.6 ahead of it; 0.9 below the beak servo.
    YB_FRONT = 50.5
    # 2026-11-13: 4.3 forward left the block's roll boss 0.18 mm behind the floor opening's rear edge (it scraped it all the way up,
    # lpath 0.2 mm): the block goes up 4.8 forward (0.3 clear), so the relief reaches 5.2 ahead (0.3 from the beak servo's rails)
    YB_TRAVEL = 4.8
    holes.append(L["combine_cutters"]([box(new(), YB_FRONT + 0.3, YB_FRONT + YB_TRAVEL + 0.4, -121.8, -88.7, 190.0, 196.0, ML),
                                        box(new(), YB_FRONT + 0.3, YB_FRONT + YB_TRAVEL + 0.4, -117.8, -93.2, 195.5, 203.9, ML)]))
    sub = L["combine_cutters"](cs + holes)
    base = L["build_part"]("head_base", [world_bm(src)], [cut], coll, L["material"]("head_base", (0.98, 0.72, 0.15, 1)))
    for bm_ in add:                                                            # the holder goes in AFTER the tub is cleared
        L["apply_bool"](base, bm_, "UNION", coll)
    L["apply_bool"](base, sub, "DIFFERENCE", coll)
    L["heal_slivers"](base, "head_base")
    base = finish("head_base", base)
    steel = L["material"]("hw_steel", (0.6, 0.62, 0.66, 1))
    es = L["ear_screw_dummy"]("mouth_ear_screws_M2x6st", Fm, coll, steel, True)
    mw = es.matrix_world.copy()
    es.parent = head
    es.matrix_world = mw
    lt.free()

# =========================================================== head dome
if "dome" in PARTS:
    src = bpy.data.objects["orig_head_shell"]
    FRONT_SCREWS = [tuple(head["front_screws"][i:i + 2]) for i in (0, 2)]
    # bulge: the back squared off round the roll servo's rear corners (both sides, symmetric)
    c = [fl(Fr, (x, y, L["J_BOTTOM"])) for x in (L["J_BODY_X"][1],) for y in (-L["J_BODY_HW"], L["J_BODY_HW"])]   # rear corners, connector end
    xr, yc = c[0].x, c[0].y
    zs = sorted(p.z for p in c)
    BR, IN, BW = 6.6, 3.0, 1.6                                     # sphere radius; centres this far inside the corner; bulge wall
    cen = [(xr + IN - 1.0, y, z) for y in (yc + IN, 2 * ROLL_AX.y - yc - IN) for z in (zs[0] + IN, zs[1] - IN)]
    sp, spi = [], []
    for cx, cy, cz in cen:
        tmp = bmesh.new()
        bmesh.ops.create_icosphere(tmp, subdivisions=3, radius=BR)
        sp += [(v.co.x + cx, v.co.y + cy, max(v.co.z + cz, DOME_Z0 + 0.02)) for v in tmp.verts]
        spi += [(v.co.x * (BR - BW) / BR + cx, v.co.y * (BR - BW) / BR + cy, max(v.co.z * (BR - BW) / BR + cz, DOME_Z0 - 1.0)) for v in tmp.verts]
        tmp.free()
    for cx, cy, cz in [(cen[0][0] + 9.0, y + (-2.0 if y < ROLL_AX.y else 2.0), (zs[0] + zs[1]) / 2 + 2.0) for y in (cen[0][1], cen[2][1])] + \
                      [(cen[0][0] + 8.0, y, zs[1] + 5.0) for y in (cen[0][1], cen[2][1])]:                # blend forward and upward into the dome
        tmp = bmesh.new()
        bmesh.ops.create_icosphere(tmp, subdivisions=3, radius=3.5)
        sp += [(v.co.x + cx, v.co.y + cy, max(v.co.z + cz, DOME_Z0 + 0.02)) for v in tmp.verts]
        spi += [(v.co.x * (3.5 - BW) / 3.5 + cx, v.co.y * (3.5 - BW) / 3.5 + cy, max(v.co.z * (3.5 - BW) / 3.5 + cz, DOME_Z0 - 1.0)) for v in tmp.verts]
        tmp.free()
    bulge = L["combine_cutters"]([hull(new(), sp, ML)], [hull(new(), spi, ML)])     # a shell, open underneath: the base's rim stays free inside it
    posts = [cyl(new(), x, y, DOME_Z0 + 0.02, DOME_Z0 + 8.0, 4.6, 28, ML) for x, y in REAR_SCREWS]
    posts += [cyl(new(), x, y, PLATE[1] + 0.05, PLATE[1] + 8.0, 4.6, 28, ML) for x, y in FRONT_SCREWS]
    add = [bulge] + posts
    hw = abs(yc - ROLL_AX.y) + 0.6
    cav = box(new(), xr - 0.6, 10.0, ROLL_AX.y - hw, ROLL_AX.y + hw, 196.0, zs[1] + 0.6, ML)        # room for the servo body (and its mirror)
    skirt = box(new(), WALL_X[0], max(WALL_X[1], OPEN_X1), -140.0, -127.0, 196.0, WALL_Z, ML)       # right skirt where the U swings
    pil = [cyl(new(), x, y, DOME_Z0 - 0.5, DOME_Z0 + 4.4, 1.6, 16, ML) for x, y in REAR_SCREWS]
    pil += [cyl(new(), x, y, PLATE[1] - 0.5, PLATE[1] + 4.0, 1.6, 16, ML) for x, y in FRONT_SCREWS]
    ribs = box(new(), 46.0, 81.0, -104.5, -71.5, PLATE[1] + 0.1, HZ + DISC_R + 0.5, ML)                # original ribs where the beak servo lies
    sub = L["combine_cutters"]([cav, skirt, ribs] + pil)
    dome = L["build_part"]("head_dome", [world_bm(src)] + add, [sub], coll, L["material"]("head_dome", (0.93, 0.93, 0.9, 1)))
    hb = bpy.data.objects.get("head_base")
    if hb:                                                         # the dome gives way wherever it would cut into the base (rim under the bulge,
        L["apply_bool"](dome, world_bm(hb), "DIFFERENCE", coll)    # and the few tenths the two original parts overlap as registered)
        L["heal_slivers"](dome, "head_dome")
    dome = finish("head_dome", dome)
    L["head_room_cut"](dome, [o for o in (bpy.data.objects.get(n) for n in HEAD_ROOM_DUMMIES) if o], coll)   # socket heads under the dome
    steel = L["material"]("hw_steel", (0.6, 0.62, 0.66, 1))
    scr = L["pan_screw_dummy"]("dome_screws_M2x6st", [(ML, x, y, DOME_Z0 - 0.05 - 2.0, +1) for x, y in REAR_SCREWS] + [(ML, x, y, PLATE[0], +1) for x, y in FRONT_SCREWS], coll, steel)
    mw = scr.matrix_world.copy()
    scr.parent = head
    scr.matrix_world = mw

# =========================================================== jaw (lower bill)
if "jaw" in PARTS:
    src = bpy.data.objects["orig_jaw"]
    bm = lay_bm(src, tri=False)
    P = np.array([v.co[:] for v in bm.verts])
    c = (P.min(0) + P.max(0)) / 2
    # the registration left the jaw back to front: turn it half a turn about y through its middle (bar to the front, under the bill plate)
    bm.transform(Matrix.Translation((c[0], 0, c[2])) @ Matrix.Rotation(math.pi, 4, "Y") @ Matrix.Translation((-c[0], 0, -c[2])))
    JAW_X0 = args.get("jaw_x0", 64.0)
    bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=(JAW_X0, 0, 0), plane_no=(-1, 0, 0), clear_outer=True)   # original outer arms off
    bmesh.ops.holes_fill(bm, edges=[e for e in bm.edges if e.is_boundary], sides=0)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.transform(ML @ Matrix.Scale(MM, 4))
    JT = 192.3                                                                 # arms run 0.8 into the jaw plate (top 193.1)
    foot = [(9.0, HZ - JT), (14.0, HZ - JT)]                                   # where the arms meet the plate (Fm x, y)
    ha, hs, hr = L["horn_plate_features"](Fm, foot, angle_deg=0.0, trim_r=TRIM)          # 0: the trimmed horn's arms (horn_check.py found 45 = turned half an arm)
    ring = [(HX + 4.5 * math.cos(2 * math.pi * i / 32), HZ + 4.5 * math.sin(2 * math.pi * i / 32)) for i in range(32)]
    right = hull(new(), [(x, y, z) for x, z in ring + [(HX + fx, JT) for fx, _ in foot] for y in ARM_YR], ML)
    pilot = cyl(new(), 0, 0, -ARM_YR[1] - 0.5, -ARM_YR[0] + 0.5, L["PILOT_D"], 16, Fp)
    # driver holes under the two front dome screws (they go up through the bill plate; the jaw covered them: 2026-11-13). Ø4.6:
    # the screw's head (Ø3.8) goes in through them too
    fs = [tuple(head["front_screws"][i:i + 2]) for i in (0, 2)] if "front_screws" in head else []
    access = [cyl(new(), x, y, 180.0, PLATE[0] - 0.1, 4.6, 24, ML) for x, y in fs]
    jaw = finish("head_jaw", L["build_part"]("head_jaw", [bm] + ha + [right], hs + [pilot] + access, coll, L["material"]("head_jaw", (0.95, 0.5, 0.12, 1)), hr))
    piv = bpy.data.objects.get("head_jaw_pivot") or bpy.data.objects.new("head_jaw_pivot", None)
    if piv.name not in coll.objects:
        coll.objects.link(piv)
    piv.empty_display_size = 0.008
    piv.parent = None
    piv.matrix_world = Fm
    bpy.context.view_layer.update()
    for o_ in (piv,):
        mw = o_.matrix_world.copy()
        o_.parent = head
        o_.matrix_world = mw
    bpy.context.view_layer.update()
    mw = jaw.matrix_world.copy()
    jaw.parent = piv
    jaw.matrix_world = mw
    steel = L["material"]("hw_steel", (0.6, 0.62, 0.66, 1))
    ps = L["pan_screw_dummy"]("jaw_pivot_screw_M2x6st", [(Fp, 0, 0, -POST_Y[1] + 1.5, +1)], coll, steel)
    mw = ps.matrix_world.copy()
    ps.parent = head
    ps.matrix_world = mw
    # beak servo model: move it to its place (it came from the fit study)
    sm = bpy.data.objects.get("scs_mouth")
    if sm:
        sm.parent = None
        sm.matrix_world = Fm
        bpy.context.view_layer.update()
        mw = sm.matrix_world.copy()
        sm.parent = head
        sm.matrix_world = mw
        for o_ in [sm] + list(sm.children_recursive):
            if o_.name not in coll.objects:
                coll.objects.link(o_)
            for c_ in list(o_.users_collection):
                if c_ != coll:
                    c_.objects.unlink(o_)
        hc = bpy.data.objects.get("scs_mouth_horn_cross")
        if hc:                                                                 # the horn and its screw turn with the jaw
            for n_ in ("scs_mouth_horn_cross", "scs_mouth_horn_screw"):
                o_ = bpy.data.objects.get(n_)
                if o_:                                                     # placed explicitly (the mesh is in the servo's frame = the pivot's frame):
                    o_.parent = piv                                        # re-parenting with a kept world matrix drifted on re-runs
                    o_.matrix_parent_inverse = Matrix.Identity(4)
                    o_.matrix_basis = Matrix.Identity(4)

# =========================================================== face panel (closes the dome's open front, carries the camera)
if "face" in PARTS:
    CAMERA = args.get("camera", "ov5640_b")
    # After the original's FACE part: a D-shaped panel that slides into the 1.9 mm groove inside the dome's front
    # (x 75.6..77.5), a round collar for the camera in the middle and one opening each side for the distance sensor and the
    # light. Ours is a hollow box as deep as the dome's front recess (the beak servo's front ear and the camera board lie in
    # it), and the camera collar points outward as a hood round the lens (user decision). The two side openings sit above the
    # camera row instead of beside it: beside it the servo ear (left) and the camera board (right) are in the way.
    # Camera: Waveshare OV5640 5MP USB Camera (A)/(C): board 18 x 36, camera body 18 x 18 x 19.4. Where the body sits on the board
    # and what is on the board's back are NOMINAL (not measured): body at one end, 2 mm of parts behind, 2.2 in front.
    CAM = dict(len=36.0, h=18.0, t=1.6, body=18.0, depth=19.4, x0=78.4, y_left=-90.6, z0=203.0)
    dt = BVHTree.FromBMesh(lay_bm(bpy.data.objects["head_dome"]))
    C0 = Vector((0.0, ROLL_AX.y, PLATE[1] + 0.05))

    def arch(x, shrink, zmin=None):
        pts = []
        for a in range(0, 181, 3):
            d = Vector((0, math.cos(math.radians(a)), math.sin(math.radians(a))))
            h = dt.ray_cast(Vector((x, C0.y, C0.z + 0.2)), d)[0]
            r = (h - Vector((x, C0.y, C0.z + 0.2))).length - shrink
            pts.append((C0.y + d.y * r, C0.z + 0.2 + d.z * r))
        pts += [(pts[0][0], C0.z), (pts[-1][0], C0.z)]
        return [(y, max(z, zmin)) for y, z in pts] if zmin else pts

    def along_x(y, z):
        return ML @ Matrix(((0, 0, 1, 0), (1, 0, 0, y * MM), (0, 1, 0, z * MM), (0, 0, 0, 1)))    # frame z = layout x

    GX, FX, WALL = (75.75, 77.35), 84.1, 1.6                                    # panel in the groove (0.15 each side); front face
    plate = hull(new(), [(x, y, z) for y, z in arch(76.55, 0.2) for x in GX], ML)
    if CAMERA == "ov5640_b":
        # Waveshare OV5640 5MP USB Camera (B), the only one available (user, 2026-11-12): a 9 x 62 strip. 62 is wider than the dome's
        # inside (61), so it sits in a BROW BAND across the face, in front of the dome's front edge (user: "elongate the head
        # protrusion"); the band stays inside the dome's outside width (68.8). Its LED replaces the separate 5 mm light.
        # Waveshare's outline drawing (user, 2026-11-12), along the board from its socket end (a) and across from its top edge (c):
        # PCB 62 x 9 x 1.0; camera 8.5 x 8.5, 5.6 +- 0.3 high, centred across, at a 29.5 (lens D5.5, FOV 68 deg); socket J1 (5 pins,
        # 1.0 pitch) at the socket end, 3.2 high on the camera side, the plug goes in from the board's end; LED at a ~49.4 (scaled
        # from the drawing); 3 x D2.1 holes at (10.2, 7.0), (60.0, 2.75), (60.0, 6.75); back side flat (grounding pads).
        B = dict(len=62.0, h=9.0, t=1.0, cam=8.5, cam_h=5.6, cam_at=29.5, led_at=49.4, led=(4.0, 3.5), led_h=1.0,
                 holes=[(10.2, 7.0), (60.0, 2.75), (60.0, 6.75)], hdr=(0.0, 4.0, 1.0, 8.0, 3.2), plug=3.3, back=0.5)
        BZ = args.get("brow_z", 210.0)                                          # board's middle height
        bx0 = FX + 0.3                                                          # board's back face (screw heads go into the opening behind)
        bx1 = bx0 + B["t"]
        GAP = B["hdr"][4] + 0.4                                                 # board front to the band's front wall: the socket, 3.2 high
        fx0 = bx1 + GAP                                                         # front wall, inner face
        fx1 = fx0 + 2.5                                                         # front wall, outer face (the camera, 5.6 high, sits 0.5 back). 2.5 (was 2.2,
        #                                                                         2026-11-13): the M2 x 6's tip (bx0 + 6 = fx0 + 1.4), its pilot to fx0 + 1.8 (0.4 past the tip), 0.7 of skin in front
        Y = lambda a: C0.y + (a - B["cam_at"])                                  # lens on the centre line; socket end on the duck's right (cable side)
        Z = lambda c: BZ + B["h"] / 2 - c
        zb0, zb1 = BZ - B["h"] / 2, BZ + B["h"] / 2
        SW = 1.2                                                                # band side walls
        PR = args.get("plug_room", 1.5)                                         # 2026-11-19: room past the plug's end for its 4 wires to turn back
        #                                                                         (0.3 left them nothing; ~1.5 for 0.9 mm wires): the band grows 1.2 at the right
        yo0, yo1 = Y(-B["plug"]) - PR - SW, Y(B["len"]) + 0.3 + SW             # right end: room for the plug before the cable turns back
        band = box(new(), FX - 0.7, fx1, yo0, yo1, zb0 - 0.3 - WALL, zb1 + 0.3 + WALL, ML)
        skin = hull(new(), [(x, y, z) for y, z in arch(79.0, 0.3) for x in (GX[1] - 0.3, FX)], ML)
        MX_ = ML @ Matrix(((0, 0, 1, 0), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 0, 1)))  # frame z = layout x
        # bosses from the board's front face to the front wall, bored for the screw: added LAST (as ribs), because the room cut in
        # front of the board (below) would take them away again (it did until 2026-11-13: the board hung on 1.25 mm of thread)
        # (bored after the build, below). Holes closer than 4.6 share one obround boss: two Ø4 bosses 4 mm apart only touch along a line
        groups = []
        for a, c in B["holes"]:
            g = next((g for g in groups if any(math.hypot(a - a2, c - c2) < 4.6 for a2, c2 in g)), None)
            (g.append((a, c)) if g else groups.append([(a, c)]))
        bosses = [hull(new(), [(Y(a) + 2.0 * math.cos(k * math.pi / 12), Z(c) + 2.0 * math.sin(k * math.pi / 12), z)
                               for a, c in g for k in range(24) for z in (bx1 + 0.02, fx0 + 0.3)], MX_) for g in groups]
        cuts = [box(new(), 74.0, 76.6, -104.0, -77.0, 200.0, 217.6, ML),        # beak servo body front
                box(new(), 74.0, 81.0, -88.4, -77.0, 200.0, 217.61, ML),        # its front ear, pad and screw heads
                hull(new(), [(x, y, z if z > C0.z + 0.01 else C0.z - 1.0) for y, z in arch(79.0, 0.3 + WALL) for x in (GX[1] + 0.02, FX - WALL)], ML),   # the face is a hollow skin, open underneath
                box(new(), FX - WALL - 0.5, FX + 0.31, Y(-B["plug"]) - PR, Y(B["len"]) + 0.2, zb0 - 0.2, zb1 + 0.2, ML),   # opening behind the board: screw heads, plug and cable
                box(new(), bx0 - 0.02, bx1 + 0.1, Y(-B["plug"]) - PR, Y(B["len"]) + 0.3, zb0 - 0.3, zb1 + 0.3, ML),  # the board's place, on past its
                #     end to the plug's (2026-11-19: a 1 mm strip there walled the plug's wire pocket off from the opening behind the board)
                box(new(), bx1 + 0.05, fx0, Y(-B["plug"]) - PR, Y(B["len"]) + 0.3, zb0 - 0.3, zb1 + 0.3, ML),       # room in front of it (parts, socket, plug + its wires' bend)
                box(new(), fx0 - 0.5, fx1 + 0.5, Y(B["cam_at"] - B["cam"] / 2 - 0.25), Y(B["cam_at"] + B["cam"] / 2 + 0.25),
                    BZ - B["cam"] / 2 - 0.25, BZ + B["cam"] / 2 + 0.25, ML),                                          # the camera through the front wall
                box(new(), fx0 - 0.5, fx1 + 0.5, Y(B["led_at"] - B["led"][0] / 2 - 0.4), Y(B["led_at"] + B["led"][0] / 2 + 0.4),
                    BZ - B["led"][1] / 2 - 0.4, BZ + B["led"][1] / 2 + 0.4, ML)]                                    # LED window
        cuts += [cyl(new(), Y(a), Z(c), fx0 - 0.5, fx0 + 1.8, L["PILOT_D"], 16, MX_) for a, c in B["holes"]]   # pilots on into the front wall (tip at fx0 + 1.4)
        # 2026-11-19: pass-through for the camera's JST lead in the D-panel, right side, behind the brow band (hidden): from the head's
        # inside into the face's hollow, then behind the board to the plug (the panel closed it off: no way in but a 1-2 mm hairline)
        cuts.append(box(new(), GX[0] - 0.6, GX[1] + 0.6, Y(-B["plug"]) + 1.8, Y(-B["plug"]) + 8.8, BZ - 3.0, BZ + 1.0, ML))
        face = L["build_part"]("head_face", [plate, skin, band], [L["combine_cutters"](cuts)], coll,
                               L["material"]("head_face", (0.93, 0.93, 0.9, 1)), bosses)
        for a, c in B["holes"]:                                                 # pilots through the bosses (after them) into the front wall
            L["apply_bool"](face, cyl(new(), Y(a), Z(c), bx1 - 0.5, fx0 + 1.8, L["PILOT_D"], 16, MX_), "DIFFERENCE", coll)
        for a, c in B["holes"]:                                                 # driver holes behind the screws (2026-11-13: the skin and the panel
            L["apply_bool"](face, cyl(new(), Y(a), Z(c), GX[0] - 1.0, bx0 - 1.6, 4.6, 24, MX_), "DIFFERENCE", coll)   # were 0.3-5 mm behind the heads)
        face = finish("head_face", face)
        dark = L["material"]("cam_board", (0.05, 0.25, 0.12, 1))
        cb = box(new(), bx0, bx1, Y(0.0), Y(B["len"]), zb0, zb1, ML)
        box(cb, bx1 - 0.02, bx1 + B["cam_h"], Y(B["cam_at"] - B["cam"] / 2), Y(B["cam_at"] + B["cam"] / 2), BZ - B["cam"] / 2, BZ + B["cam"] / 2, ML)
        box(cb, bx1 - 0.02, bx1 + B["led_h"], Y(B["led_at"] - B["led"][0] / 2), Y(B["led_at"] + B["led"][0] / 2), BZ - B["led"][1] / 2, BZ + B["led"][1] / 2, ML)
        box(cb, bx1 - 0.02, bx1 + B["hdr"][4], Y(B["hdr"][0]), Y(B["hdr"][1]), Z(B["hdr"][3]), Z(B["hdr"][2]), ML)          # socket J1
        box(cb, bx1 + 0.3, bx1 + B["hdr"][4] - 0.3, Y(-B["plug"]), Y(B["hdr"][0]) - 0.02, Z(B["hdr"][3] - 0.5), Z(B["hdr"][2] + 0.5), ML)   # its plug
        cam = L["to_object"]("cam_ov5640_b", cb, coll, dark)
        mw = cam.matrix_world.copy()
        cam.parent = head
        cam.matrix_world = mw
        cam["socket_xyz"] = [bx1 + B["hdr"][4] / 2, Y(-B["plug"]), BZ]
        cam["lens_xyz"] = [bx1 + B["cam_h"], Y(B["cam_at"]), BZ]
        scr = L["pan_screw_dummy"]("cam_screws_M2x6st", [(MX_, Y(a), Z(c), bx0, +1) for a, c in B["holes"]], coll)
        mw = scr.matrix_world.copy()
        scr.parent = head
        scr.matrix_world = mw
        _dm = bpy.data.objects.get("head_dome")
        if _dm and not _dm.get("cam_room_cut"):                                 # once per dome (2026-11-19: re-running the face re-cut it each
            L["head_room_cut"](_dm, [scr], coll)                                # time and the stacked cuts broke it into 4 pieces); the upper
            _dm["cam_room_cut"] = True                                          # screw's head is 1.75 under the dome
        for n_ in ("cam_ov5640_usb", "light_led_5mm"):
            if n_ in bpy.data.objects:
                bpy.data.objects.remove(bpy.data.objects[n_], do_unlink=True)
        print(f"face: brow band x {FX - 1.1:.1f}..{fx1:.1f} (dome front edge 82..85.4), y {yo0:.1f}..{yo1:.1f} (dome outside -134.4..-65.6), "
              f"z {zb0 - 0.3 - WALL:.1f}..{zb1 + 0.3 + WALL:.1f}; lens y {Y(B['cam_at']):.1f} z {BZ:.1f}, its front at x {bx1 + B['cam_h']:.1f} (front wall {fx1:.1f})")
    else:
        shell = hull(new(), [(x, y, z) for y, z in arch(79.0, 0.3) for x in (GX[1] - 0.3, FX)], ML)
        yl, zt = CAM["y_left"], CAM["z0"] + CAM["h"]
        yr = yl - CAM["len"]
        yc, zc, hb = yl - CAM["body"] / 2, CAM["z0"] + CAM["h"] / 2, CAM["body"] / 2
        XH = CAM["x0"] + CAM["t"] + CAM["depth"] + 1.0                             # hood ends 1 mm past the lens
        BORE = 2 * math.hypot(hb, hb) + 1.1                                         # round, as the original's collar: clears the square body
        hood = L["combine_cutters"]([cyl(new(), 0, 0, FX - 1.0, XH, BORE + 2 * WALL, 64, along_x(yc, zc))],
                                    [box(new(), 70.0, 110.0, -140.0, -60.0, 180.0, CAM["z0"] - 2.0, ML)])     # flat underneath, level with the bill
        px0, px1 = CAM["x0"] - 0.1, CAM["x0"] + CAM["t"] + 0.2                      # groove for the board (slides in from below)
        SIDE_Z, SIDE_Y = zt + 3.2, 14.0                                             # the two side openings: height, distance from the middle
        cuts = [box(new(), 74.0, 76.6, -104.0, -77.0, 200.0, 217.6, ML),            # beak servo body front
                box(new(), 74.0, 81.0, -88.4, -77.0, 200.0, 217.61, ML),            # its front ear, pad and screw heads
                box(new(), px0, px1, yr - 0.2, yl + 0.2, 200.0, zt + 0.2, ML),      # camera board
                box(new(), 74.0, px0 + 0.05, yr + 1.0, yl - 1.0, 200.0, zt + 0.21, ML),   # room behind the board (1 mm lips hold its ends)
                box(new(), px1 - 0.05, FX - WALL, yr + 1.0, yl - 1.0, 200.0, zt + 0.22, ML),   # and in front
                L["combine_cutters"]([cyl(new(), 0, 0, px1, XH + 1.0, BORE, 64, along_x(yc, zc))],
                                     [box(new(), 70.0, 111.0, -141.0, -59.0, 181.0, CAM["z0"] - 0.4, ML)]),   # camera body through the hood (the hood keeps a floor)
                hull(new(), [(x, y, z) for y, z in arch(79.0, 0.3 + WALL + 0.3, zmin=zt + 0.25) for x in (74.0, FX - WALL)], ML),   # hollow above the camera row
                # (no distance-sensor window: there is no room behind it - camera board below, dome roof above; closed on the user's
                #  decision 2026-11-12, the sensor is to go on the body's chest)
                cyl(new(), 0, 0, FX - WALL - 0.5, FX + 0.5, 5.2, 24, along_x(C0.y + SIDE_Y, SIDE_Z))]                               # 5 mm light (left)
        face = finish("head_face", L["build_part"]("head_face", [plate, shell, hood], [L["combine_cutters"](cuts)], coll, L["material"]("head_face", (0.93, 0.93, 0.9, 1))))
        dark = L["material"]("cam_board", (0.05, 0.25, 0.12, 1))
        cb = box(new(), CAM["x0"], CAM["x0"] + CAM["t"], yr, yl, CAM["z0"], zt, ML)
        box(cb, CAM["x0"] + CAM["t"] - 0.02, CAM["x0"] + CAM["t"] + CAM["depth"], yc - hb, yc + hb, zc - hb, zc + hb, ML)
        cam = L["to_object"]("cam_ov5640_usb", cb, coll, dark)
        mw = cam.matrix_world.copy()
        cam.parent = head
        cam.matrix_world = mw
        led = L["combine_cutters"]([cyl(new(), 0, 0, FX - 7.6, FX + 1.0, 5.0, 24, along_x(C0.y + SIDE_Y, SIDE_Z)),          # 5 mm LED: body in the hole, tip 1 mm proud
                                    cyl(new(), 0, 0, FX - WALL - 1.0, FX - WALL - 0.02, 5.8, 24, along_x(C0.y + SIDE_Y, SIDE_Z))])   # its flange behind the wall
        lo = L["to_object"]("light_led_5mm", led, coll, L["material"]("light_led", (1.0, 0.95, 0.7, 1)))
        mw = lo.matrix_world.copy()
        lo.parent = head
        lo.matrix_world = mw
        print(f"face: lens centre y {yc:.1f} z {zc:.1f}, hood dia {BORE + 2 * WALL:.1f} to x {XH:.1f} (dome front edge 82..85.4, bill plate front 81.1); side openings at y {C0.y - SIDE_Y:.1f} / {C0.y + SIDE_Y:.1f}, z {SIDE_Z:.1f}")

print("head shell v2: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
