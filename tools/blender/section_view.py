"""Cut-away copy of some objects: everything on one side of a plane is removed and the cut is capped,
so the inside of a joint can be seen. Runs inside Blender. Copies are named '<name>.x' in a new
collection placed at ARGS["offset"]; originals are untouched. Re-running replaces the collection.

ARGS = {"objects": ["left_shin", ...], "collection": "ankle_section", "frame": "scs_left_ankle",
        "normal_axis": "x" (plane through the frame origin, normal = that frame axis) or "normal": [x, y, z] (world),
        "offset": [0, -0.7, 0], "scale": 3.0, "point": [x, y, z] (optional: the plane passes through it instead of the frame origin),
        "in_place": {"ref": "duck_scs0009:trunk_base", "spread": 9.0, "viewer": [0.6, 0.5, 0.6]},   # or {"at": [x, y, z], "viewer": ...}; or:
        "align": True}   # turn the cut-away into a standard pose: cut face toward +X, the frame's +Z (servo shaft) up,
                         # the frame's +X (servo length) along -Y; then "offset" is the absolute place of the frame origin.
                         # With align the plane is always the frame's X-Z plane (through shaft and servo length).
"""
import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
COLL = args["collection"]
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
bpy.context.view_layer.update()
F = bpy.data.objects[args["frame"]].matrix_world
p0 = Vector(args["point"]) if "point" in args else F.translation.copy()     # a point of the cutting plane (world m)
if "normal" in args:
    n = Vector(args["normal"]).normalized()
else:
    n = (F.to_3x3() @ Vector({"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[args.get("normal_axis", "x")])).normalized()
S = args.get("scale", 1.0)
if args.get("in_place"):
    # keep the joint's real orientation; put it where it sits in the assembly (positions spread by `spread`
    # about the reference object so neighbouring sections don't overlap); cut through shaft + servo length,
    # removing the half that faces the viewer
    ip = args["in_place"]
    n = (F.to_3x3() @ Vector((0, 1, 0))).normalized()
    if n.dot(Vector(ip.get("viewer", (0.6, 0.5, 0.6)))) < 0:
        n = -n
    if "at" in ip:                      # explicit place of the joint origin (a hand layout that mirrors the body)
        place = Vector(ip["at"])
    else:
        place = (p0 - bpy.data.objects[ip["ref"]].matrix_world.translation) * ip.get("spread", 1.0)
    T = Matrix.Translation(Vector(args.get("offset", (0, 0, 0))) + place) @ Matrix.Scale(S, 4) @ Matrix.Translation(-p0)
elif args.get("align"):
    n = (F.to_3x3() @ Vector((0, 1, 0))).normalized()
    rot = Matrix(((0, 1, 0, 0), (-1, 0, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))        # frame x -> -Y, y -> +X, z -> +Z
    Fn = F.copy()
    T = Matrix.Translation(Vector(args.get("offset", (0, 0, 0)))) @ Matrix.Scale(S, 4) @ rot @ Fn.inverted()
else:
    T = Matrix.Translation(Vector(args.get("offset", (0, 0, 0))) + p0) @ Matrix.Scale(S, 4) @ Matrix.Translation(-p0)
for name in args["objects"]:
    o = bpy.data.objects.get(name)
    if o is None:
        print("  (missing)", name)
        continue
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    res = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=p0, plane_no=n, clear_outer=True)
    cut = [e for e in res["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
    if cut:
        bmesh.ops.holes_fill(bm, edges=cut, sides=0)       # cap every cut loop
        bmesh.ops.triangle_fill(bm, use_beauty=True, edges=[e for e in bm.edges if e.is_boundary])
    if not bm.faces:                       # entirely on the removed side: nothing to show
        bm.free()
        continue
    bm.transform(T)
    me = bpy.data.meshes.new(name + "_sec")
    bm.to_mesh(me)
    bm.free()
    for m in o.data.materials:
        me.materials.append(m)
    c = bpy.data.objects.new(name + ".x", me)
    coll.objects.link(c)
print(f"{COLL}: {len(args['objects'])} objects cut, normal {tuple(round(x, 2) for x in n)}, scale {S}")
