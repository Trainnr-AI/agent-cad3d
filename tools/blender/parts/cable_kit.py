"""The SCS0009 kit's electrical parts as simple solids, for routing and clearance work. Runs inside Blender.
Sizes are ESTIMATED from the product photo, scaled by the servo and its horns (about +-10 %): replace them
with caliper values. Each servo has ONE fixed lead; the kit adds a 3-socket hub board and an extension cable.

  servo lead       flat 3-wire, 3.0 x 1.0 mm, 150 mm (datasheet), ends in a 3-pin plug
  plug             6.0 long x 8.0 wide x 4.5 high (3-pin, 2.0-2.5 mm pitch)
  hub board        17 x 20 x 1.2 mm green PCB, three 3-pin sockets (8 x 5.5 x 5 each), all in parallel
  extension cable  three round wires (bundle 4.2 x 1.4), 165 mm between two plugs (180 overall)

ARGS = {"collection": "cable_kit", "location": [0.3, 0.0, 0.0], "path": None}
Functions (exec this file into a dict with ARGS = {"lib": True} to only get them):
  plug(name, M, coll), hub(name, M, coll), ribbon(name, pts_world_m, width_mm, thick_mm, coll, mat)
"""
import bpy
from mathutils import Matrix, Vector

LEAD_W, LEAD_T, LEAD_L = 3.0, 1.0, 150.0
PLUG = (6.0, 8.0, 4.5)                  # length along the cable, width across the pins, height
HUB = (17.0, 20.0, 1.2)
SOCKET = (5.5, 8.0, 5.0)                # depth into the board, width, height above the board
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
    """hub board; M: origin at the board centre, Z = up. Sockets: two side by side on -X, one on +X."""
    parts = [_box(name, HUB, M, coll, GREEN)]
    z = (HUB[2] + SOCKET[2]) / 2
    for i, (x, y) in enumerate(((-HUB[0] / 2 + SOCKET[0] / 2, -HUB[1] / 4), (-HUB[0] / 2 + SOCKET[0] / 2, HUB[1] / 4), (HUB[0] / 2 - SOCKET[0] / 2, 0))):
        s = _box(f"{name}_socket{i + 1}", SOCKET, M @ Matrix.Translation(Vector((x, y, z)) * MM), coll, WHITE)
        s.parent = parts[0]
        s.matrix_parent_inverse = parts[0].matrix_world.inverted()
        parts.append(s)
    return parts


def ribbon(name, pts, width_mm, thick_mm, coll, mat):
    """flat cable along a polyline (world metres), as a bevelled curve; returns (object, length_mm)"""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    sp = cu.splines.new("BEZIER")
    sp.bezier_points.add(len(pts) - 1)
    for bp, p in zip(sp.bezier_points, pts):
        bp.co = p
        bp.handle_left_type = bp.handle_right_type = "AUTO"
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
