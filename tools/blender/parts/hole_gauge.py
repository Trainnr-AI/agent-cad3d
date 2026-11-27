"""Hole gauge (runs inside Blender): one small plate to print BEFORE the robot, with the same printer / profile, to confirm the hole sizes.

2026-11-27 (user's test: M2 would not go into its holes, M3 barely through 3.3 -> every hole +0.4, tools/blender/hole_resize.py). Four
rows, each the model's size in the MIDDLE with -0.2 and +0.2 either side (left -> right: small -> large):
  row 1  M2 pilot (blind, 6 deep)   1.8  2.0  2.2   - an M2 x 6 must screw in by hand-driver, firm, not stripping
  row 2  M2 clearance (through)     2.6  2.8  3.0   - an M2 must drop through without threading
  row 3  M3 pilot (blind, 6 deep)   2.7  2.9  3.1   - an M3 must thread in, firm
  row 4  M3 clearance (through)     3.5  3.7  3.9   - an M3 must pass freely
A notch on the plate's front edge marks the middle (model) column; row 1 is at the notched short edge. If the middle hole of a row is
wrong, tell which of the three is right: hole_resize.py takes the new sizes.

Collection `print_tools`, object `hole_gauge` (mass_g_solid_PLA set: exported with the printed parts). ARGS = {"at": [x, y, z] mm world}
"""
import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals().get("ARGS", {})
MM = 0.001
AT = Vector(args.get("at", (0.0, 300.0, 0.0))) * MM
W, D, H = 38.0, 46.0, 7.0                       # plate (x, y, z)
ROWS = [("M2 pilot", [1.8, 2.0, 2.2], True), ("M2 clearance", [2.6, 2.8, 3.0], False),
        ("M3 pilot", [2.7, 2.9, 3.1], True), ("M3 clearance", [3.5, 3.7, 3.9], False)]
coll = bpy.data.collections.get("print_tools") or bpy.data.collections.new("print_tools")
S = bpy.data.scenes.get("Scene") or bpy.context.scene
if coll.name not in S.collection.children:
    S.collection.children.link(coll)
old = bpy.data.objects.get("hole_gauge")
if old:
    bpy.data.objects.remove(old, do_unlink=True)


def box(x0, x1, y0, y1, z0, z1):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector(((x0 + x1) / 2 + v.co.x * (x1 - x0), (y0 + y1) / 2 + v.co.y * (y1 - y0), (z0 + z1) / 2 + v.co.z * (z1 - z0))) * MM
    return bm


bm = box(0, W, 0, D, 0, H)
me = bpy.data.meshes.new("hole_gauge")
bm.to_mesh(me)
bm.free()
o = bpy.data.objects.new("hole_gauge", me)
coll.objects.link(o)
o.matrix_world = Matrix.Translation(AT)
cut = bmesh.new()
for r, (name, ds, blind) in enumerate(ROWS):
    y = 7.0 + r * 10.5
    for c, d in enumerate(ds):
        x = 8.0 + c * 11.0
        z0 = H - 6.0 if blind else -1.0
        bmesh.ops.create_cone(cut, cap_ends=True, segments=48, radius1=d / 2 * MM, radius2=d / 2 * MM, depth=(H + 1.0 - z0) * MM,
                              matrix=Matrix.Translation(Vector((x, y, (z0 + H + 1.0) / 2)) * MM))
nb = box(8.0 + 11.0 - 1.5, 8.0 + 11.0 + 1.5, -1.0, 1.2, -1.0, H + 1.0)   # notch at the front edge: the middle column
tmpme = bpy.data.meshes.new("_n")
nb.to_mesh(tmpme)
nb.free()
cut.from_mesh(tmpme)
bpy.data.meshes.remove(tmpme)
cme = bpy.data.meshes.new("hole_gauge_cut")
cut.to_mesh(cme)
cut.free()
co = bpy.data.objects.new("hole_gauge_cut", cme)
coll.objects.link(co)
co.matrix_world = o.matrix_world
solvers = [i.identifier for i in bpy.types.BooleanModifier.bl_rna.properties["solver"].enum_items]
mod = o.modifiers.new("holes", "BOOLEAN")
mod.operation, mod.object = "DIFFERENCE", co
mod.solver = "MANIFOLD" if "MANIFOLD" in solvers else "EXACT"
with bpy.context.temp_override(object=o, active_object=o):
    bpy.ops.object.modifier_apply(modifier=mod.name)
bpy.data.objects.remove(co, do_unlink=True)
bpy.data.meshes.remove(cme)
bm = bmesh.new()
bm.from_mesh(o.data)
vol = abs(bm.calc_volume()) / MM ** 3
nm = sum(not e.is_manifold for e in bm.edges)
bm.free()
o["volume_mm3"] = vol
o["mass_g_solid_PLA"] = round(vol / 1000 * 1.24, 2)
o["rows"] = "; ".join(f"row {i + 1} {n}: {d}" for i, (n, d, _b) in enumerate(ROWS))
m = bpy.data.materials.get("hole_gauge") or bpy.data.materials.new("hole_gauge")
m.diffuse_color = (0.95, 0.55, 0.1, 1)
o.data.materials.append(m)
o.color = (0.95, 0.55, 0.1, 1)
print(f"hole gauge: {W} x {D} x {H} mm, {o['mass_g_solid_PLA']} g, non-manifold {nm}; {o['rows']}")
