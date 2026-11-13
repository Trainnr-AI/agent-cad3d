"""Colour-code components by kind and label them (name + purpose) beside an exploded view.
Runs inside Blender. Colours go on the mesh data (linked copies share them, so the assembled and
exploded robots match). Labels are text objects in their own collection, facing ARGS["face"]
(+X by default), in two columns with leader lines; re-running replaces them.

ARGS = {"collections": ["duck_scs0009", "leg_v2", "neck_v2", "duck_v2_exploded"],
        "kinds": [[regex, kind, [r, g, b]], ...]            # first match wins, on the object name
        "labels": [{"object": "left_thigh", "text": "Thigh\\nholds hip-pitch + knee servos", "side": "L"}, ...],
        "label_collection": "duck_v2_labels", "exploded": "duck_v2_exploded", "size": 0.0055,
        "auto_labels": {kind: "text"},   # instead of "labels": one label per kind present in `exploded` (first object
                                          # of that kind), on the side its centre lies; "title": "..." adds a heading; "side": "L" | "R" puts all auto labels on one side;
        "view": "+X" (default) | "-Y" | "+Y" | "-X": the side the labels are read from}
"""
import re

import bpy
from mathutils import Vector

args = globals()["ARGS"]


def mat(kind, rgb):
    m = bpy.data.materials.get("cc_" + kind) or bpy.data.materials.new("cc_" + kind)
    m.diffuse_color = (*rgb, 1)
    m.use_nodes = True
    b = next((n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if b:
        b.inputs["Base Color"].default_value = (*rgb, 1)
    return m


kinds = [(re.compile(rx), k, mat(k, rgb)) for rx, k, rgb in args["kinds"]]
count = {}
for cn in args["collections"]:
    for o in bpy.data.collections[cn].all_objects:
        if o.type != "MESH":
            continue
        for rx, k, m in kinds:
            if rx.search(o.name):
                o.data.materials.clear()
                o.data.materials.append(m)
                o["component_kind"] = k
                count[k] = count.get(k, 0) + 1
                break
print("coloured:", ", ".join(f"{k} {n}" for k, n in sorted(count.items())))

# ---------------------------------------------------------------- labels
LC = args.get("label_collection", "labels_cc")
lc = bpy.data.collections.get(LC) or bpy.data.collections.new(LC)
if lc.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(lc)
for o in list(lc.objects):
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()
ex = bpy.data.collections[args["exploded"]]
pts = [o.matrix_world @ Vector(c) for o in ex.objects if o.type == "MESH" for c in o.bound_box]
lo = Vector([min(p[i] for p in pts) for i in range(3)])
hi = Vector([max(p[i] for p in pts) for i in range(3)])
SIZE = args.get("size", 0.0055)
MARGIN = args.get("margin", 0.03)
# the label plane faces the viewer: N = toward the viewer, U = the text's reading direction, Z up
VIEW = {"+X": (Vector((1, 0, 0)), Vector((0, 1, 0)), (1.5707963, 0, 1.5707963)),
        "-Y": (Vector((0, -1, 0)), Vector((1, 0, 0)), (1.5707963, 0, 0)),
        "+Y": (Vector((0, 1, 0)), Vector((-1, 0, 0)), (1.5707963, 0, 3.1415927)),
        "-X": (Vector((-1, 0, 0)), Vector((0, -1, 0)), (1.5707963, 0, -1.5707963))}[args.get("view", "+X")]
N, U, ROT = VIEW
n_plane = max(p.dot(N) for p in pts) + 0.01
u_lo, u_hi = min(p.dot(U) for p in pts), max(p.dot(U) for p in pts)
ink = mat("label_ink", tuple(args.get("ink", (1, 1, 1))))


def at(u, z):
    return N * n_plane + U * u + Vector((0, 0, z))


def find(name):
    """the exploded copy of `name` (copies are '<name>.x', or '<name>.x.001' if an older copy holds the name)"""
    for o in ex.objects:
        if o.name == name + ".x" or o.name.startswith(name + ".x."):
            return o
    raise KeyError(name)


def centre(o):
    return sum((o.matrix_world @ Vector(c) for c in o.bound_box), Vector()) / 8


labels = list(args.get("labels", []))
if args.get("auto_labels"):
    mid_u, seen = (u_lo + u_hi) / 2, set()
    for o in ex.objects:
        k = o.get("component_kind")
        if o.type == "MESH" and k in args["auto_labels"] and k not in seen:
            seen.add(k)
            labels.append({"object": o.name[:o.name.index(".x")], "text": args["auto_labels"][k], "side": args.get("side") or ("L" if centre(o).dot(U) > mid_u else "R")})
if args.get("title"):
    cu = bpy.data.curves.new("title", "FONT")
    cu.body, cu.size, cu.align_x, cu.extrude = args["title"], SIZE * 1.6, "CENTER", 0.0002
    cu.materials.append(ink)
    t = bpy.data.objects.new("title_" + args["exploded"], cu)
    lc.objects.link(t)
    t.location = at((u_lo + u_hi) / 2, hi.z + SIZE * 4)
    t.rotation_euler = ROT
for side in ("L", "R"):                                   # "L" = the +U side (the viewer's right), "R" = the -U side
    items = [l for l in labels if l["side"] == side]
    items.sort(key=lambda l: -centre(find(l["object"])).z)
    n = len(items)
    top, pitch = hi.z + 0.01, max(SIZE * 3.2, (hi.z - lo.z + 0.02) / max(n, 1))
    for i, l in enumerate(items):
        ob = find(l["object"])
        m = ob.data.materials[0] if ob.data.materials else ink
        z = top - i * pitch
        u = u_hi + MARGIN if side == "L" else u_lo - MARGIN
        cu = bpy.data.curves.new("lbl", "FONT")
        cu.body = l["text"]
        cu.size = SIZE
        cu.align_x = "LEFT" if side == "L" else "RIGHT"
        cu.extrude = 0.0002
        cu.materials.append(ink)
        t = bpy.data.objects.new("lbl_" + l["object"], cu)
        lc.objects.link(t)
        t.location = at(u + (SIZE if side == "L" else -SIZE), z)
        t.rotation_euler = ROT
        # colour chip + leader line in the component's colour
        c = bpy.data.curves.new("lead", "CURVE")
        c.dimensions = "3D"
        sp = c.splines.new("POLY")
        sp.points.add(1)
        a, b = at(u, z + SIZE * 0.3), centre(ob)
        sp.points[0].co, sp.points[1].co = (*a, 1), (*b, 1)
        c.bevel_depth = SIZE * 0.07
        c.materials.append(m)
        lc.objects.link(bpy.data.objects.new("lead_" + l["object"], c))
        chip = bpy.data.meshes.new("chip")
        h = SIZE * 0.55
        zc = z + SIZE * 0.3
        chip.from_pydata([tuple(at(u - h, zc - h)), tuple(at(u + h, zc - h)), tuple(at(u + h, zc + h)), tuple(at(u - h, zc + h))], [], [(0, 1, 2, 3)])
        chip.materials.append(m)
        lc.objects.link(bpy.data.objects.new("chip_" + l["object"], chip))
print(f"{len(labels)} labels in {LC}")
