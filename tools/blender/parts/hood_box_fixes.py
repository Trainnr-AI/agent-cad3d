"""Mini560 box (on the hood, `uno_cover`) - five cuts made IN PLACE (runs inside Blender), the same as body_v2.py now cuts when the
hood is rebuilt (2026-11-25, wire-vs-part review). World / layout mm. Each is applied once (flag on the hood); Manifold boolean, kept only
if the hood stays one shell with no new non-manifold edges.

  battery_lead_slot  the battery's 14 AWG leads (flat 6.2 x 3.1) run to the XT30's top at z 119-120, where the box floor (z 117-120)
                     and its rear wall stood: a slot from the rear wall to the pigtail notch, under the splices (5 mm below them)
  window_skin        the wire window stopped 0.2 mm short of the hood's face: a 0.2 mm skin the jumpers ran through
  pigtail_notch      the pigtail notch widened 1.5 mm toward -x: the 14 AWG pigtails grazed its edge (0.7 mm)
  pcb_groove_a / _b  0.25 mm guide grooves in both side walls for the Mini560 PCB's edges (it had 0.1 mm a side)

ARGS = {} (no arguments)
"""
import bmesh
import bpy
from mathutils import Vector

MM = 0.001
CUTS = {
    "battery_lead_slot": (-59.8, -53.0, -118.2, -109.0, 116.5, 122.0),
    "window_skin": (-53.4, -52.9, -112.0, -99.1, 125.0, 134.0),
    "pigtail_notch": (-60.5, -58.9, -109.5, -101.0, 116.5, 120.5),
    "pcb_groove_a": (-77.95, -76.25, -117.05, -116.75, 119.7, 160.0),
    "pcb_groove_b": (-77.95, -76.25, -98.65, -98.35, 119.7, 160.0),
}


def stats(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    vol, nm = abs(bm.calc_volume()), sum(not e.is_manifold for e in bm.edges)
    seen, shells = set(), 0
    for f in bm.faces:
        if f.index in seen:
            continue
        shells += 1
        stack = [f]
        seen.add(f.index)
        while stack:
            g = stack.pop()
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
    bm.free()
    return vol, nm, shells


hood = bpy.data.objects["uno_cover"]
report = []
for name, (x0, x1, y0, y1, z0, z1) in CUTS.items():
    if hood.get(name):
        report.append(f"{name}: already cut")
        continue
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector(((x0 + x1) / 2 + v.co.x * (x1 - x0), (y0 + y1) / 2 + v.co.y * (y1 - y0), (z0 + z1) / 2 + v.co.z * (z1 - z0))) * MM
    me = bpy.data.meshes.new("hood_cut")
    bm.to_mesh(me)
    bm.free()
    cut = bpy.data.objects.new("hood_cut", me)
    bpy.context.scene.collection.objects.link(cut)
    v0, nm0, sh0 = stats(hood.data)
    mod = hood.modifiers.new("cut", "BOOLEAN")
    mod.operation, mod.solver, mod.object = "DIFFERENCE", "MANIFOLD", cut
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    new = bpy.data.meshes.new_from_object(hood.evaluated_get(dg))
    hood.modifiers.remove(mod)
    bpy.data.objects.remove(cut, do_unlink=True)
    bpy.data.meshes.remove(me)
    v1, nm1, sh1 = stats(new)
    if sh1 != sh0 or nm1 > nm0:
        bpy.data.meshes.remove(new)
        report.append(f"{name}: NOT cut (shells {sh0}->{sh1}, non-manifold {nm0}->{nm1})")
        continue
    old = hood.data
    oname = old.name
    old.name = oname + ".pre_" + name
    new.name = oname
    if not len(new.materials):
        for m in old.materials:
            new.materials.append(m)
    hood.data = new
    hood[name] = True
    if "mass_g_solid_PLA" in hood:
        hood["mass_g_solid_PLA"] = round(v1 * 1e6 * 1.24, 2)
    report.append(f"{name}: removed {(v0 - v1) * 1e9:.1f} mm3, shells {sh1}, non-manifold {nm1}")
print("hood box fixes:\n  " + "\n  ".join(report))
