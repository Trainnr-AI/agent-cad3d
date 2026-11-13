"""The SCS0009 kit's electrical parts as simple solids, for routing and clearance work. Runs inside Blender.
Sizes are ESTIMATED from the product photo, scaled by the servo and its horns (about +-10 %): replace them
with caliper values. Each servo has ONE fixed lead; the kit adds a 3-socket hub board and an extension cable.

  servo lead       flat 3-wire, 3.0 x 1.0 mm, 150 mm (datasheet), ends in a 3-pin plug
  plug             9.5 long x 10 wide x 5.5 high (3-pin, 2.5 mm pitch, 5264 type)
  hub board        20 x 17 mm green PCB with three SIDE-entry 3-pin sockets (12.3 wide x 9 deep x 6 high) opening in three
                   directions (A and B opposite, C at 90 deg); about 24 x 20 x 7 overall, all three in parallel
  extension cable  three round wires (bundle 4.2 x 1.4), 165 mm between two plugs (180 overall)

ARGS = {"collection": "cable_kit", "location": [0.3, 0.0, 0.0], "path": None}
Functions (exec this file into a dict with ARGS = {"lib": True} to only get them):
  plug(name, M, coll), hub(name, M, coll), ribbon(name, pts_world_m, width_mm, thick_mm, coll, mat)
"""
import bpy
from mathutils import Matrix, Vector

LEAD_W, LEAD_T, LEAD_L = 3.0, 1.0, 150.0
# Read from an 8x enlargement of the kit photo, scaled by the sockets' pin pitch taken as 2.5 mm (Feetech's SCS connector
# is the 3-pin 5264 type). About +-10 %: replace with caliper values.
PLUG = (9.5, 10.0, 5.5)                 # length along the cable, width across the pins, height
HUB = (20.0, 17.0, 1.2)                 # the green PCB; with its sockets the hub is about 24 x 20 x 7
SOCKET = (9.0, 12.3, 6.0)               # depth (along the plug), width, height above the board
# three SIDE-entry sockets (plugs slide in parallel to the board), opening in three directions:
#   A opens toward +Y, B toward -Y (opposite A), C toward +X.  (centre x, centre y, opening direction) on the PCB, mm
SOCKETS = [(-3.9, 4.0, (0, 1)), (-0.8, -5.0, (0, -1)), (9.6, 3.2, (1, 0))]
PLUG_ROOM = 7.0                         # what sticks out of a socket once the plug is in: ~3 mm of plug + the wires' bend
EXT_W, EXT_T, EXT_L = 4.2, 1.4, 165.0
MM = 0.001
_args = globals().get("ARGS", {})


def _mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


WHITE, GREEN, DARK, RED = (_mat("kit_white", (0.92, 0.92, 0.9, 1)), _mat("kit_pcb_green", (0.05, 0.5, 0.15, 1)),
                           _mat("kit_wire_dark", (0.12, 0.12, 0.12, 1)), _mat("kit_wire_red", (0.8, 0.1, 0.1, 1)))


def _box(name, size_mm, M, coll, mat):
    """box with its centre at M's origin; size along M's x, y, z"""
    me = bpy.data.meshes.new(name)
    h = [s * MM / 2 for s in size_mm]
    v = [(sx * h[0], sy * h[1], sz * h[2]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    me.from_pydata(v, [], [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)])
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    ob.matrix_world = M
    return ob


def plug(name, M, coll):
    """3-pin plug; M: origin at the cable entry, +X along the plug (away from the cable), Z = height"""
    return _box(name, PLUG, M @ Matrix.Translation(Vector((PLUG[0] / 2, 0, 0)) * MM), coll, WHITE)


def hub(name, M, coll):
    """hub board; M: origin at the PCB centre, Z = up (component side). Returns [pcb, socket A, B, C]."""
    parts = [_box(name, HUB, M, coll, GREEN)]
    z = (HUB[2] + SOCKET[2]) / 2
    for i, (x, y, (dx, dy)) in enumerate(SOCKETS):
        size = (SOCKET[1], SOCKET[0], SOCKET[2]) if dy else (SOCKET[0], SOCKET[1], SOCKET[2])
        sk = _box(f"{name}_socket{'ABC'[i]}", size, M @ Matrix.Translation(Vector((x, y, z)) * MM), coll, WHITE)
        sk.parent = parts[0]
        sk.matrix_parent_inverse = parts[0].matrix_world.inverted()
        parts.append(sk)
    return parts


def socket_mouths(M):
    """[(mouth point, outward direction)] of the three sockets, in world coordinates"""
    out = []
    for x, y, (dx, dy) in SOCKETS:
        m = Vector((x + dx * SOCKET[0] / 2, y + dy * SOCKET[0] / 2, HUB[2] / 2 + SOCKET[2] / 2)) * MM
        out.append((M @ m, (M.to_3x3() @ Vector((dx, dy, 0))).normalized()))
    return out


# Bus board for the body (not in the kit): 12 x 5-hole 2.54 mm perfboard with five top-entry 3-pin headers (the mate of the
# kit's plug) wired in parallel; the sixth cell takes its one M2 screw and the soldered ends of four extension cables.
BUS = (30.5, 12.7, 1.6)
BUS_SOCKET = (9.9, 5.8, 8.5)            # length along the pins, width, height above the board
BUS_CELLS = [(0.0, 3.81), (10.16, 3.81), (-10.16, 3.81), (0.0, -3.81), (10.16, -3.81)]
BUS_FREE = (-10.16, -3.81)
BUS_PLUG_TOP = 13.0                     # top of an inserted plug above the board's top face


def bus_board(name, M, coll):
    """M: origin at the PCB centre, Z = up (socket side), X = the long side"""
    parts = [_box(name, BUS, M, coll, GREEN)]
    for i, (x, y) in enumerate(BUS_CELLS):
        sk = _box(f"{name}_socket{i + 1}", BUS_SOCKET, M @ Matrix.Translation(Vector((x, y, (BUS[2] + BUS_SOCKET[2]) / 2)) * MM), coll, WHITE)
        sk.parent = parts[0]
        sk.matrix_parent_inverse = parts[0].matrix_world.inverted()
        parts.append(sk)
    return parts


def bus_mouths(M):
    """[(mouth point = top of the socket, outward direction = board normal)]"""
    n = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
    return [(M @ (Vector((x, y, BUS[2] / 2 + BUS_SOCKET[2])) * MM), n) for x, y in BUS_CELLS]


def ribbon(name, pts, width_mm, thick_mm, coll, mat):
    """flat cable along a polyline (world metres), as a bevelled curve; returns (object, length_mm)"""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    sp = cu.splines.new("BEZIER")
    sp.bezier_points.add(len(pts) - 1)
    for bp, p in zip(sp.bezier_points, pts):
        bp.co = p
        bp.handle_left_type = bp.handle_right_type = "AUTO"
    if width_mm > thick_mm * 2.5 and thick_mm > 1.2:        # three loose round wires (extension cable): a round bundle of the same area
        cu.bevel_depth = (width_mm * thick_mm / 3.14159) ** 0.5 * MM
    else:                                                   # bonded flat lead / single wire
        cu.bevel_depth = thick_mm * MM / 2
        cu.extrude = max(0.0, (width_mm - thick_mm) * MM / 2)
    cu.materials.append(mat)
    ob = bpy.data.objects.new(name, cu)
    coll.objects.link(ob)
    bpy.context.view_layer.update()
    return ob, sp.calc_length() * 1000


if not _args.get("lib"):
    COLL = _args.get("collection", "cable_kit")
    old = bpy.data.collections.get(COLL)
    if old:
        for o in list(old.objects):
            bpy.data.objects.remove(o, do_unlink=True)
        bpy.data.collections.remove(old)
    coll = bpy.data.collections.new(COLL)
    bpy.context.scene.collection.children.link(coll)
    O = Vector(_args.get("location", (0.3, 0.0, 0.0)))
    T = lambda x, y, z=0.0: Matrix.Translation(O + Vector((x, y, z)) * MM)
    # servo lead laid out straight: 150 mm + plug
    ribbon("kit_servo_lead_150", [O + Vector((0, y, 0.5)) * MM for y in (0, 50, 100, LEAD_L)], LEAD_W, LEAD_T, coll, WHITE)
    plug("kit_servo_lead_plug", T(0, LEAD_L, 0.5 + PLUG[2] / 2 - 0.5) @ Matrix.Rotation(1.5707963, 4, "Z"), coll)
    hub("kit_hub_board", T(30, 20, HUB[2] / 2), coll)
    ribbon("kit_extension_165", [O + Vector((60, y, 0.7)) * MM for y in (PLUG[0], 60, 120, PLUG[0] + EXT_L)], EXT_W, EXT_T, coll, RED)
    plug("kit_extension_plug_a", T(60, PLUG[0], PLUG[2] / 2) @ Matrix.Rotation(-1.5707963, 4, "Z"), coll)
    plug("kit_extension_plug_b", T(60, PLUG[0] + EXT_L, PLUG[2] / 2) @ Matrix.Rotation(1.5707963, 4, "Z"), coll)
    print(f"{COLL}: lead {LEAD_L:.0f} mm + plug, hub {HUB[0]:.0f}x{HUB[1]:.0f}, extension {EXT_L + 2 * PLUG[0]:.0f} mm overall (sizes estimated from the photo)")
