"""Bay v2 BLOCK-OUT (runs inside Blender): one container in the space the robot's moving parts leave free, the real parts placed in
it as linked copies (they share the parts' meshes; the v1 bay is not touched). For reviewing the layout before any detail.

2026-11-27 (user, after printing v1): no small protrusions or small parts, slide fits instead of mounts, keep it simple to assemble,
bigger screw holes; battery in the centre with the boards' connectors facing it; power leads out and tied outside. The free space
(swept_space.py with the real motions: walking corners, STAND -> SIT, every joint alone within the firmware limits, hip pitch -46..72)
is a narrow column between the legs up to z ~118 and a wide block above it to z ~157 (the head's sweep) - so: a lower COLUMN
(the v1 enclosure's footprint) and a wide TOP BOX, one part, 45 deg steps between them (prints upright without supports), one lid.

Layout mm (the frame of bay_parts.py / wiring.py). ARGS = {} (all numbers below).
"""
import bmesh
import bpy
from mathutils import Matrix, Vector

MM = 0.001
R = "duck_scs0009"
COLL = "bay_v2"
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
coll = old or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
MLi = ML.inverted()
CY = -100.0
T = 2.0                                                              # wall
COL = dict(x=(-65.0, 3.0), y=(CY - 22.0, CY + 22.0), z=(57.0, 120.0))           # lower column (v1 enclosure footprint)
TOP = dict(x=(-90.0, 2.0), y=(CY - 31.0, CY + 31.0), z=(118.0, 157.0))          # top box (wide), lid 2 mm on top included
LID_T = 2.0


def mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


def solid(name, boxes_add, boxes_sub, rgba):
    def bx(bm, x0, x1, y0, y1, z0, z1):
        vs = [bm.verts.new(Vector((x, y, z)) * MM) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
        for f in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
            bm.faces.new([vs[i] for i in f])
    o = None
    for kind, boxes in (("add", boxes_add), ("sub", boxes_sub)):
        for b in boxes:
            bm = bmesh.new()
            bx(bm, *b)
            me = bpy.data.meshes.new(name + "_" + kind)
            bm.to_mesh(me)
            bm.free()
            ob = bpy.data.objects.new(name + "_" + kind, me)
            coll.objects.link(ob)
            ob.matrix_world = ML
            if o is None:
                o = ob
                o.name = name
                continue
            mod = o.modifiers.new("b", "BOOLEAN")
            mod.operation, mod.solver, mod.object = ("UNION" if kind == "add" else "DIFFERENCE"), "EXACT", ob
            with bpy.context.temp_override(object=o, active_object=o):
                bpy.ops.object.modifier_apply(modifier=mod.name)
            bpy.data.objects.remove(ob, do_unlink=True)
    o.data.materials.clear()
    o.data.materials.append(mat(name, rgba))
    o.color = rgba
    return o


# ---- the container: column + top box (+ 45 deg steps under the top box's overhangs), hollow, open on top (the lid closes it)
cx, cy, cz, tx, ty, tz = COL["x"], COL["y"], COL["z"], TOP["x"], TOP["y"], TOP["z"]
step = tz[0] - 8.0                                                  # 45 deg chamfer start under the overhang (blocked out as a step)
add = [(cx[0], cx[1], cy[0], cy[1], cz[0], tz[0] + 1.0), (tx[0], tx[1], ty[0], ty[1], tz[0], tz[1] - LID_T)]
sub = [(cx[0] + T, cx[1] - T, cy[0] + T, cy[1] - T, cz[0] + T, tz[0] + 2.0),
       (tx[0] + T, tx[1] - T, ty[0] + T, ty[1] - T, tz[0] + T, tz[1] - LID_T + 1.0),
       (cx[0] + T, cx[1] - T, cy[0] + T, cy[1] - T, tz[0] - 1.0, tz[0] + T + 0.5)]          # the column opens into the top box
box_o = solid("bay_v2_container", add, sub, (0.45, 0.62, 0.8, 1))
lid_o = solid("bay_v2_lid", [(tx[0], tx[1], ty[0], ty[1], tz[1] - LID_T, tz[1])], [], (0.55, 0.72, 0.88, 1))


# ---- the real parts, moved: optional rotation about the group's centre (axis, deg), then either a translation or "to_min" = where the
#      group's bounding box's low corner goes (layout mm)
def place(names, d=(0, 0, 0), rot=None, to_min=None):
    objs = [bpy.data.objects[n] for n in names if n in bpy.data.objects]
    pts = [MLi @ o.matrix_world @ Vector(c) / MM for o in objs for c in o.bound_box]
    c = sum(pts, Vector()) / len(pts)
    Rm = Matrix.Identity(4)
    if rot:
        Rm = Matrix.Translation(c) @ Matrix.Rotation(__import__("math").radians(rot[1]), 4, rot[0]) @ Matrix.Translation(-c)
    if to_min is not None:
        q = [Rm @ p for p in pts]
        lo = Vector([min(v[i] for v in q) for i in range(3)])
        d = Vector(to_min) - lo
    Tl = Matrix.Translation(Vector(d)) @ Rm
    S = Matrix.Diagonal((MM, MM, MM, 1.0))
    W = ML @ S @ Tl @ S.inverted() @ MLi                               # layout-mm transform -> world
    out = []
    for o in objs:
        cpy = o.copy()                                                  # shares the mesh / curve data
        cpy.name = "v2_" + o.name
        cpy.parent = None
        coll.objects.link(cpy)
        cpy.matrix_world = W @ o.matrix_world
        cpy.hide_set(False)
        cpy.hide_viewport = False
        out.append(cpy)
    return out


UNO = ["unoq_pcb", "unoq_headers", "unoq_usbc", "unoq_pins", "unoq_bottom_connectors", "unoq_top_parts"] + \
      [o.name for o in bpy.data.collections["unoq_wires"].objects if not o.name.startswith("unoq_qwiic_cable")]
# Uno Q upside down under the ceiling (user 2026-11-27): pins down, the board's back 0.2 under the lid; x -70 .. -1.4 (the USB-C end
# at the rear for the camera adapter, the Qwiic at the front end)
ceil = tz[1] - LID_T
uno = place(UNO, rot=("X", 180.0), to_min=(-70.0 - 0.6, CY - 26.67, 0.0))
zs = [(c.matrix_world @ Vector(v))[2] for c in uno for v in c.bound_box]
zt = (MLi @ Vector((0, 0, max(zs))))[2] / MM if False else max((MLi @ c.matrix_world @ Vector(v))[2] / MM for c in uno for v in c.bound_box)
dz = (ceil - 0.2) - zt
S = Matrix.Diagonal((MM, MM, MM, 1.0))
for c in uno:
    c.matrix_world = ML @ S @ Matrix.Translation((0, 0, dz)) @ S.inverted() @ MLi @ c.matrix_world
place(["battery_3s"], (-12.1, 0.0, 0.0))                                # x -40 .. -20, standing in the middle of the column
place(["servo_adapter"], rot=("Z", 180.0), to_min=(-13.6, CY - 16.9, 58.9))   # on the front wall, servo sockets toward the battery
place(["imu", "bay_imu_qwiic"], (-14.6, 0.0, -9.0))                     # on the rear wall of the column, z 95 .. 116
place(["mini560_pcb", "mini560_parts", "mini560_plugs"], rot=("Y", -90.0), to_min=(-88.0, CY - 28.0, tz[0] + T))   # flat, rear section
lo = [min((MLi @ c.matrix_world @ Vector(v))[i] / MM for c in coll.objects if c.name.startswith("v2_unoq") for v in c.bound_box) for i in range(3)]
hi = [max((MLi @ c.matrix_world @ Vector(v))[i] / MM for c in coll.objects if c.name.startswith("v2_unoq") for v in c.bound_box) for i in range(3)]
print("bay v2 block-out: top box", [round(v, 1) for v in (tx[1] - tx[0], ty[1] - ty[0], tz[1] - tz[0])], "mm, overall height", tz[1] - cz[0],
      "| Uno Q + plugs + wires: x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f" % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]), "|", len(coll.objects), "objects")
