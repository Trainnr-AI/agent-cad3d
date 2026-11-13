"""Size comparison labels: for each robot, a name, a height line and a width line with their values, in a vertical plane in
front of it (readable from the front, +x). Measures the visible meshes of the given collections. Runs inside Blender.
Replaces the collection "dimensions" on each run.
ARGS = {"items": [{"name": "nanoduck", "title": "nanoduck (ours)", "collections": [...]}, ...,
                  {"name": "exploded", "title": "nanoduck, fully exploded (1.5x scale)", "collections": [...], "dims": false}],
        "text_mm": 9, "color": [1, 0.8, 0.1]}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
MM = 0.001
TXT = args.get("text_mm", 9.0) * MM
old = bpy.data.collections.get("dimensions")
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new("dimensions")
bpy.context.scene.collection.children.link(coll)


def mat(name, rgb):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs[0].default_value = (*rgb, 1)
    em.inputs[1].default_value = 1.5
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(em.outputs[0], out.inputs[0])
    m.diffuse_color = (*rgb, 1)
    return m


LINE = mat("dim_line", args.get("color", [1.0, 0.8, 0.1]))
TEXT = mat("dim_text", [0.95, 0.95, 0.95])
FACE = Matrix.Rotation(math.radians(90), 4, "Z") @ Matrix.Rotation(math.radians(90), 4, "X")   # text in the y-z plane, facing +x


def bar(name, a, b, t=0.6):
    """a thin square bar from a to b (metres)"""
    bm = bmesh.new()
    d = (b - a)
    L = d.length
    bmesh.ops.create_cube(bm, size=1.0)
    bm.transform(Matrix.Diagonal((t * MM, t * MM, L, 1)))
    R = d.normalized().to_track_quat("Z", "Y").to_matrix().to_4x4()
    bm.transform(Matrix.Translation((a + b) / 2) @ R)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(LINE)
    o = bpy.data.objects.new(name, me)
    coll.objects.link(o)
    return o


def text(name, body, at, size=TXT, align="CENTER"):
    cu = bpy.data.curves.new(name, "FONT")
    cu.body = body
    cu.size = size
    cu.align_x = align
    cu.align_y = "CENTER"
    cu.materials.append(TEXT)
    o = bpy.data.objects.new(name, cu)
    coll.objects.link(o)
    o.matrix_world = Matrix.Translation(at) @ FACE
    return o


def measure(colls):
    lo, hi = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for cn in colls:
        for o in bpy.data.collections[cn].all_objects:
            if o.type != "MESH" or not o.visible_get():
                continue
            for v in o.data.vertices:
                w = o.matrix_world @ v.co
                for i in range(3):
                    lo[i] = min(lo[i], w[i])
                    hi[i] = max(hi[i], w[i])
    return lo, hi


report = []
for it in args["items"]:
    lo, hi = measure(it["collections"])
    H, W = (hi.z - lo.z) / MM, (hi.y - lo.y) / MM
    x = hi.x + 15 * MM                                            # in front of the robot
    mid_y = (lo.y + hi.y) / 2
    text(f"dim_{it['name']}_title", it["title"], Vector((x, mid_y, hi.z + 22 * MM)), TXT * 1.15)
    if it.get("dims", True):
        g = 12 * MM                                               # gap from the robot
        yl = lo.y - g                                             # height line beside the robot (its -y side)
        bar(f"dim_{it['name']}_H", Vector((x, yl, lo.z)), Vector((x, yl, hi.z)))
        for z in (lo.z, hi.z):
            bar(f"dim_{it['name']}_Htick", Vector((x, yl - 4 * MM, z)), Vector((x, yl + 4 * MM, z)))
        text(f"dim_{it['name']}_Htext", f"H {H:.0f} mm", Vector((x, yl - 6 * MM, (lo.z + hi.z) / 2)), TXT, "RIGHT")
        zw = lo.z - g                                             # width line under the feet
        bar(f"dim_{it['name']}_W", Vector((x, lo.y, zw)), Vector((x, hi.y, zw)))
        for y in (lo.y, hi.y):
            bar(f"dim_{it['name']}_Wtick", Vector((x, y, zw - 4 * MM)), Vector((x, y, zw + 4 * MM)))
        text(f"dim_{it['name']}_Wtext", f"W {W:.0f} mm", Vector((x, mid_y, zw - 10 * MM)), TXT)
    report.append(f"{it['name']}: H {H:.1f} W {W:.1f} D {(hi.x - lo.x) / MM:.1f} mm")
print("dimensions: " + " | ".join(report))
