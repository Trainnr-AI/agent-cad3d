"""Camera vents into the BUILT head face (runs inside Blender, robot at the zero pose): the same slots head_shell_v2.py now cuts
when it builds the face (camera_vent_slots there, read from that file: one definition), cut into the existing head_face in
place - it already carries later in-place edits (hole_resize.py) a rebuild would have to repeat.

2026-11-27 (user: "camera gets hot so add vents for the camera"): a comb of 1.5 mm slots through the brow band's bottom and
top walls over the room in front of the camera board: air in below, out above, past the board's parts.

ARGS = {"builder": "tools/blender/parts/head_shell_v2.py", "robot": "duck_scs0009", "part": "head_face",
        "force": false}   # force: cut again although the part is tagged camera_vents
The old mesh is kept as `<mesh>.pre_vents` (fake user) to undo by hand.
"""
import ast

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals().get("ARGS", {})
MM = 0.001
R = args.get("robot", "duck_scs0009")
o = bpy.data.objects[args.get("part", "head_face")]
if o.get("camera_vents") and not args.get("force"):
    # never SystemExit here: in the live session it quits Blender (2026-11-28)
    raise RuntimeError(f"{o.name}: already has camera vents ({o['camera_vents']}); pass force to cut again")
src = open(args.get("builder", "tools/blender/parts/head_shell_v2.py")).read()
ns = {}
for node in ast.parse(src).body:                                     # VENT + camera_vent_slots only (the rest needs the scene)
    if (isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "VENT" for t in node.targets)) or \
            (isinstance(node, ast.FunctionDef) and node.name == "camera_vent_slots"):
        exec(compile(ast.Module([node], []), "head_shell_v2", "exec"), ns)

# the face's numbers as the builder has them (ov5640_b branch): board strip, lens on the head's centre line, band round it
B = dict(len=62.0, h=9.0, t=1.0, cam_at=29.5, hdr=(0.0, 4.0, 1.0, 8.0, 3.2), plug=3.3)
C0Y, FX, WALL, BZ = -100.03, 84.1, 1.6, 210.0
bx1 = FX + 0.3 + B["t"]
zb0, zb1 = BZ - B["h"] / 2, BZ + B["h"] / 2
Y = lambda a: C0Y + (a - B["cam_at"])
cam = bpy.data.objects.get("cam_ov5640_b")
T = bpy.data.objects[R + ":trunk_base"].matrix_world
ML = T @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
if cam:                                                              # the scene must be at the zero pose: the lens where the builder put it
    want = Vector(cam["lens_xyz"])
    got = Matrix.Scale(1000, 4) @ ML.inverted() @ Vector(max((cam.matrix_world @ v.co for v in cam.data.vertices), key=lambda p: p.x))
    assert abs(got.x - want.x) < 0.05, f"robot not at the zero pose (lens x {got.x:.2f}, built at {want.x:.2f})"
slots = ns["camera_vent_slots"](B, Y, bx1, zb0, zb1, WALL)

bm = bmesh.new()
for x0, x1, y0, y1, z0, z1 in slots:
    g = bmesh.ops.create_cube(bm, size=1.0)
    for v in g["verts"]:
        v.co = Vector(((x0 + x1) / 2 + v.co.x * (x1 - x0), (y0 + y1) / 2 + v.co.y * (y1 - y0), (z0 + z1) / 2 + v.co.z * (z1 - z0))) * MM
cme = bpy.data.meshes.new("_camera_vents_cut")
bm.to_mesh(cme)
bm.free()
cut = bpy.data.objects.new("_camera_vents_cut", cme)
o.users_collection[0].objects.link(cut)
cut.matrix_world = ML


def stats(me):
    b = bmesh.new()
    b.from_mesh(me)
    v = abs(b.calc_volume()) / MM ** 3
    nm = sum(not e.is_manifold for e in b.edges)
    shells, seen = 0, set()
    for f in b.faces:                                                # connected pieces
        if f.index in seen:
            continue
        shells += 1
        stack = [f]
        while stack:
            g = stack.pop()
            if g.index in seen:
                continue
            seen.add(g.index)
            stack += [h for e in g.edges for h in e.link_faces if h.index not in seen]
    b.free()
    return v, nm, shells


v0 = stats(o.data)
old = o.data.copy()
old.name = o.data.name + ".pre_vents"
old.use_fake_user = True
solvers = [i.identifier for i in bpy.types.BooleanModifier.bl_rna.properties["solver"].enum_items]
for solver in [s for s in ("MANIFOLD", "EXACT") if s in solvers]:
    mod = o.modifiers.new("camera_vents", "BOOLEAN")
    mod.operation, mod.object, mod.solver = "DIFFERENCE", cut, solver
    with bpy.context.temp_override(object=o, active_object=o):
        bpy.ops.object.modifier_move_to_index(modifier=mod.name, index=0)
        bpy.ops.object.modifier_apply(modifier=mod.name)
    v1 = stats(o.data)
    if v1[1] == 0 and v1[2] == v0[2] and v1[0] < v0[0]:
        break
    o.data = old.copy()                                              # failed: back to the old mesh, try the next solver
    print(f"{solver}: non-manifold {v1[1]}, shells {v1[2]} - retry")
bpy.data.objects.remove(cut, do_unlink=True)
bpy.data.meshes.remove(cme)
removed = v0[0] - v1[0]
o["camera_vents"] = f"2026-11-27: {len(slots) // 2} slots top + {len(slots) // 2} bottom, {ns['VENT']['w']} wide, pitch {ns['VENT']['pitch']}"
if "mass_g_solid_PLA" in o:
    o["mass_g_solid_PLA"] = round(o["mass_g_solid_PLA"] - removed / 1000 * 1.24, 2)
print(f"{o.name}: {len(slots)} slots ({len(slots) // 2} per wall), removed {removed:.1f} mm3; volume {v1[0]:.0f}, non-manifold {v1[1]}, "
      f"pieces {v1[2]} (before {v0[2]}); slot x {slots[0][0]:.2f}..{slots[0][1]:.2f}, y {slots[0][2]:.2f}..{slots[-1][3]:.2f}")
