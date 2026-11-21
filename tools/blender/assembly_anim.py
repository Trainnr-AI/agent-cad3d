"""Animated assembly (runs inside Blender): parts move into their real places in build order, in a SEPARATE scene so the
user's scene is untouched. Copies share the parts' meshes (always the current geometry). Each step: its title on screen,
its parts slide in along a path (mm, world, relative to the part's real place; the last point should be [0, 0, 0]), then its
screws, then the wires that get connected in that step appear. Every move is tested against the parts already in place
(triangle overlap at 12 points along the path); the report says where a moving part would pass through another.
Re-running rebuilds the scene.

ARGS = {"scene": "bay_assembly", "context": ["body_frame"], "fps": 24, "hold": 18,
        "camera": {"target_mm": [-30, -100, 110], "dist_mm": 420, "elev_deg": 38, "azim_deg": [140, 220], "lens": 50},
        "steps": [{"title": "1  ...", "moves": [{"objects": [...], "path": [[-70, 0, 0], [0, 0, 0]], "frames": 36}, ...],
                   "wires": ["wire_..."]}, ...],
        "play": true}
"""
import math

import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
MM = 0.001
src = bpy.data.scenes.get("Scene") or bpy.context.window.scene
name = args.get("scene", "bay_assembly")
old = bpy.data.scenes.get(name)
if old is not None:
    for o in list(old.collection.all_objects):
        d = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if d is not None and d.users == 0 and isinstance(d, (bpy.types.TextCurve, bpy.types.Camera)):
            (bpy.data.curves if isinstance(d, bpy.types.Curve) else bpy.data.cameras).remove(d)
        elif d is not None and d.users == 0 and isinstance(d, bpy.types.Mesh) and d.name.endswith(".asm"):
            bpy.data.meshes.remove(d)                                       # split halves made by this script
    for w in bpy.context.window_manager.windows:
        if w.scene == old:
            w.scene = src
    bpy.data.scenes.remove(old)
sc = bpy.data.scenes.new(name)
for k in ("system", "scale_length", "length_unit"):
    setattr(sc.unit_settings, k, getattr(src.unit_settings, k))
sc.world = src.world
fps, hold = args.get("fps", 24), args.get("hold", 18)
sc.render.fps = fps

prefs = bpy.context.preferences.edit
keep_interp = prefs.keyframe_new_interpolation_type
copies = {}


def copy_of(n):
    if n in copies:
        return copies[n]
    o = bpy.data.objects[n]
    c = bpy.data.objects.new(n + ".asm", o.data)
    c.matrix_world = o.matrix_world.copy()
    c.color = o.color
    for i, s in enumerate(o.material_slots):
        if s.link == "OBJECT" and i < len(c.material_slots):
            c.material_slots[i].link = "OBJECT"
            c.material_slots[i].material = s.material
    sc.collection.objects.link(c)
    c["home"] = list(c.location)
    for hl in args.get("highlight", []):                     # e.g. bearings in orange: an object-level material on the COPY only
        if hl["contains"] in n:
            m = bpy.data.materials.get("anim_highlight_" + hl["contains"]) or bpy.data.materials.new("anim_highlight_" + hl["contains"])
            m.diffuse_color = hl["rgba"]
            if m.use_nodes and m.node_tree:
                bsdf = next((x for x in m.node_tree.nodes if x.type == "BSDF_PRINCIPLED"), None)
                if bsdf:
                    bsdf.inputs["Base Color"].default_value = hl["rgba"]
            for sl in c.material_slots:
                sl.link = "OBJECT"
                sl.material = m
            c.color = hl["rgba"]
    prefs.keyframe_new_interpolation_type = "CONSTANT"
    for f, h in ((1, True),):
        c.hide_viewport = c.hide_render = h
        c.keyframe_insert("hide_viewport", frame=f)
        c.keyframe_insert("hide_render", frame=f)
    copies[n] = c
    return c


def show(c, f):
    prefs.keyframe_new_interpolation_type = "CONSTANT"
    c.hide_viewport = c.hide_render = False
    c.keyframe_insert("hide_viewport", frame=f)
    c.keyframe_insert("hide_render", frame=f)


def key_loc(c, f, off_mm):
    prefs.keyframe_new_interpolation_type = "BEZIER"
    c.location = Vector(c["home"]) + Vector(off_mm) * MM
    c.keyframe_insert("location", frame=f)


def split_copies(n, axis):
    """two copies of n, each holding the loose pieces on one side of its centre along `axis` (screws going in from both sides);
    their own meshes (named n.side.asm), the part's mesh is untouched"""
    import bmesh
    o = bpy.data.objects[n]
    out = []
    for sgn in (1, -1):
        me = o.data.copy()
        me.name = f"{n}.{'p' if sgn > 0 else 'm'}.asm"
        bm = bmesh.new()
        bm.from_mesh(me)
        cen = sum((v.co for v in bm.verts), Vector()) / max(1, len(bm.verts))
        seen, kill = set(), []
        for v in bm.verts:
            if v.index in seen:
                continue
            stack, comp = [v], []
            seen.add(v.index)
            while stack:
                u = stack.pop()
                comp.append(u)
                for e in u.link_edges:
                    w = e.other_vert(u)
                    if w.index not in seen:
                        seen.add(w.index)
                        stack.append(w)
            c = sum((u.co for u in comp), Vector()) / len(comp)
            if (c[axis] - cen[axis]) * sgn < 0:
                kill += comp
        bmesh.ops.delete(bm, geom=kill, context="VERTS")
        bm.to_mesh(me)
        bm.free()
        c = bpy.data.objects.new(me.name, me)
        c.matrix_world = o.matrix_world.copy()
        c.color = o.color
        sc.collection.objects.link(c)
        c["home"] = list(c.location)
        prefs.keyframe_new_interpolation_type = "CONSTANT"
        c.hide_viewport = c.hide_render = True
        c.keyframe_insert("hide_viewport", frame=1)
        c.keyframe_insert("hide_render", frame=1)
        copies[me.name] = c
        out.append((c, sgn))
    return out


def screw_copies(n, pieces):
    """one copy per screw of the stand-in n: each loose piece goes to the screw whose axis line it is nearest to"""
    import bmesh
    o = bpy.data.objects[n]
    M = o.matrix_world
    lines = [(Vector(p["at"]) * MM, Vector(p["axis"]).normalized()) for p in pieces]
    bm0 = bmesh.new()
    bm0.from_mesh(o.data)
    bm0.verts.ensure_lookup_table()
    seen, owner = set(), {}
    for v in bm0.verts:
        if v.index in seen:
            continue
        stack, comp = [v], []
        seen.add(v.index)
        while stack:
            u = stack.pop()
            comp.append(u.index)
            for e in u.link_edges:
                w = e.other_vert(u)
                if w.index not in seen:
                    seen.add(w.index)
                    stack.append(w)
        c = M @ (sum((bm0.verts[i].co for i in comp), Vector()) / len(comp))
        k = min(range(len(lines)), key=lambda j: ((c - lines[j][0]) - lines[j][1] * (c - lines[j][0]).dot(lines[j][1])).length)
        for i in comp:
            owner[i] = k
    bm0.free()
    out = []
    for k in range(len(lines)):
        me = o.data.copy()
        me.name = f"{n}.s{k}.asm"
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.verts.ensure_lookup_table()
        bmesh.ops.delete(bm, geom=[v for v in bm.verts if owner.get(v.index) != k], context="VERTS")
        bm.to_mesh(me)
        bm.free()
        c = bpy.data.objects.new(me.name, me)
        c.matrix_world = M.copy()
        c.color = o.color
        sc.collection.objects.link(c)
        c["home"] = list(c.location)
        prefs.keyframe_new_interpolation_type = "CONSTANT"
        c.hide_viewport = c.hide_render = True
        c.keyframe_insert("hide_viewport", frame=1)
        c.keyframe_insert("hide_render", frame=1)
        out.append((c, lines[k][1]))
    pieces_of[n] = [c for c, _ in out]
    return out


pieces_of = {}                                          # a stand-in shown as separate screws: its copies (moved together later)
report = []
f = 1
for n in args.get("context", []):
    show(copy_of(n), 1)
for si, st in enumerate(args["steps"]):
    t0 = f
    for mv in st.get("moves", []):
        frames = mv.get("frames", 30)
        for n in mv["objects"]:
            if n not in bpy.data.objects:
                report.append(f"  step {si + 1}: missing object {n}")
        names = [n for n in mv["objects"] if n in bpy.data.objects]
        if mv.get("screw_pieces"):                      # each screw drives in along its own axis, from out_mm away (head outward)
            base = Vector(mv.get("base", (0, 0, 0)))
            for n in names:
                for c, axv in screw_copies(n, mv["screw_pieces"]):
                    show(c, f)
                    key_loc(c, f, base + axv * mv.get("out_mm", 12.0))
                    key_loc(c, f + frames, base)
            f += frames + 4
            continue
        path = [Vector(p) for p in mv["path"]]
        if path[-1].length > 1e-9 and not mv.get("hold"):
            path.append(Vector())
        if mv.get("split_axis") is not None:            # the + side follows the path, the - side its mirror along that axis
            ax = mv["split_axis"]
            objs = [(c, sgn) for n in names for c, sgn in split_copies(n, ax)]
        else:
            ax, objs = None, [(c, 1) for n in names for c in (pieces_of[n] if n in pieces_of else [copy_of(n)])]
        for c, sgn in objs:
            show(c, f)
            for k, p in enumerate(path):
                q = p.copy()
                if ax is not None:
                    q[ax] *= sgn
                key_loc(c, f + round(frames * k / max(1, len(path) - 1)), q)
        f += frames + 4
    for n in st.get("wires", []):
        if n in bpy.data.objects:
            show(copy_of(n), f)
        else:
            report.append(f"  step {si + 1}: missing wire {n}")
    f += hold
    st["_span"] = (t0, f)
sc.frame_start, sc.frame_end = 1, f + 24

# ---- camera (slow orbit) with the step titles riding in front of the lens ----
cam_d = bpy.data.cameras.new(name + "_cam")
cam_d.lens = args.get("camera", {}).get("lens", 50)
cam_d.clip_start = 0.005
cam = bpy.data.objects.new(name + "_cam", cam_d)
sc.collection.objects.link(cam)
sc.camera = cam
cfg = args.get("camera", {})
tgt = Vector(cfg.get("target_mm", [-30, -100, 110])) * MM
dist, el = cfg.get("dist_mm", 420) * MM, math.radians(cfg.get("elev_deg", 38))
az0, az1 = [math.radians(a) for a in cfg.get("azim_deg", [140, 220])]
# the camera rides on a pivot at the target that turns about Z (a true orbit), and always looks at the target (Track To): keyframing
# the camera's own location + Euler rotation cut across the circle and swung the view off the robot between the keys
pivot = bpy.data.objects.new(name + "_pivot", None)
sc.collection.objects.link(pivot)
pivot.location = tgt
cam.parent = pivot
cam.location = Vector((math.cos(el), 0.0, math.sin(el))) * dist
tc = cam.constraints.new("TRACK_TO")
tc.target, tc.track_axis, tc.up_axis = pivot, "TRACK_NEGATIVE_Z", "UP_Y"
prefs.keyframe_new_interpolation_type = "LINEAR"
for fr, az in ((1, az0), (sc.frame_end, az1)):
    pivot.rotation_euler = (0.0, 0.0, az)
    pivot.keyframe_insert("rotation_euler", frame=fr)
mat = bpy.data.materials.get("label_text") or bpy.data.materials.new("label_text")
mat.diffuse_color = (0.95, 0.95, 0.95, 1)
D = 0.1                                                        # titles 0.1 m in front of the lens
half_w = D * 18 / cam_d.lens                                   # half the view width there (36 mm sensor)
for si, st in enumerate(args["steps"]):
    cu = bpy.data.curves.new(f"{name}_title_{si + 1:02d}", "FONT")
    cu.body = st["title"]
    cu.size = half_w * args.get("title_size", 0.055)
    cu.align_x, cu.align_y = "LEFT", "TOP"
    t = bpy.data.objects.new(cu.name, cu)
    t.data.materials.append(mat)
    t.color = (0.95, 0.95, 0.95, 1)
    sc.collection.objects.link(t)
    t.parent = cam
    t.location = (-half_w * 0.94, half_w * 9 / 16 * 0.9, -D)
    a, b = st["_span"]
    prefs.keyframe_new_interpolation_type = "CONSTANT"
    for fr, h in ((1, True), (a, False), (b, True)):
        t.hide_viewport = t.hide_render = h
        t.keyframe_insert("hide_viewport", frame=fr)
        t.keyframe_insert("hide_render", frame=fr)
    if si == len(args["steps"]) - 1:                         # the last title stays to the end
        t.hide_viewport = t.hide_render = False
        t.keyframe_insert("hide_viewport", frame=b)
        t.keyframe_insert("hide_render", frame=b)
prefs.keyframe_new_interpolation_type = keep_interp

sc.frame_set(1)
print(f"{name}: {len(args['steps'])} steps, {len(copies)} parts and wires, frames 1..{sc.frame_end} ({sc.frame_end / fps:.0f} s at {fps} fps)")
print("(the moves are path-checked by anim_to_paths.py + assembly_path_check.py, not here)")
for r in report:
    print(r)

if args.get("play", True):
    win = bpy.context.window_manager.windows[0]
    win.scene = sc
    for area in win.screen.areas:
        if area.type == "VIEW_3D":
            area.spaces.active.region_3d.view_perspective = "CAMERA"
            region = next(r for r in area.regions if r.type == "WINDOW")
            with bpy.context.temp_override(window=win, area=area, region=region):
                if not bpy.context.screen.is_animation_playing:
                    bpy.ops.screen.animation_play()
            break
