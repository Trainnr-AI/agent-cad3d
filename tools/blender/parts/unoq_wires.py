"""The wires on the Arduino UNO Q (runs inside Blender): plain Dupont jumper plugs on its headers and the Qwiic cable to the IMU.
2026-11-27 (user): 5 jumper wires - RX (D0), TX (D1), GND on the digital header; VIN, GND on the power header - each a 1-pin male
Dupont plug whose plastic stands ~10 mm over the board's header, the wire leaving its top and bending over 80-100 deg ("depends on
assembly": the direction and angle are per wire, below); and a ~100 mm Qwiic (JST SH 4-pin) cable from the board's Qwiic socket to
the IMU's.

Reads the anchors parts/uno_q.py stores (unoq_headers["pins"], unoq_pins["qwiic_mouth"]) and the IMU plug's (bay_imu_qwiic
["anchors"]["cable"]), so the wires sit on the exact pins. Layout mm (the frame of bay_parts.py / wiring.py). Re-running replaces
the collection.

ARGS = {"collection": "unoq_wires",
        "jumpers": {"D0_RX": {"dir": [-1, 0, 0], "bend_deg": 90}, ...}   (default: the 5 above, toward the USB-C end, 90 deg),
        "plug_h": 10.0, "lead_mm": 20.0, "wire_d": 1.4, "qwiic_route": [[x, y, z], ...] (layout mm, between the two plugs),
        "qwiic_len": 100.0, "qwiic_d": 2.2}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals().get("ARGS", {})
MM = 0.001
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "unoq_wires")
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
coll = old or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
W = lambda p: ML @ (Vector(p) * MM)                                 # layout mm -> world
hdr, pins_o = bpy.data.objects["unoq_headers"], bpy.data.objects["unoq_pins"]
PIN = {k: Vector(v) for k, v in hdr["pins"].items()}
UP = Vector(hdr.get("pin_axis", (0, 0, 1)))
PH, LEAD, WD = args.get("plug_h", 10.0), args.get("lead_mm", 20.0), args.get("wire_d", 1.4)
JUMP = args.get("jumpers") or {n: {"dir": [-1, 0, 0], "bend_deg": 90} for n in ("D0_RX", "D1_TX", "GND_D", "VIN", "GND_P2")}


def mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


def box_obj(name, centre, size, rgba):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector((v.co.x * size[0], v.co.y * size[1], v.co.z * size[2])) * MM
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    coll.objects.link(o)
    o.matrix_world = ML @ Matrix.Translation(Vector(centre) * MM) @ Matrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    o.data.materials.append(mat(name.split(".")[0].rsplit("_", 1)[0] + "_mat", rgba))
    o.color = rgba
    return o


def tube(name, pts, d, rgba):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions, cu.bevel_depth, cu.bevel_resolution, cu.use_fill_caps = "3D", d / 2 * MM, 3, True
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for p_, q in zip(sp.points, pts):
        p_.co = (*W(q), 1.0)
    o = bpy.data.objects.new(name, cu)
    coll.objects.link(o)
    cu.materials.append(mat(name + "_mat", rgba))
    o.color = rgba
    return o, sum(((Vector(b) - Vector(a)).length for a, b in zip(pts, pts[1:])))


COLOR = {"D0_RX": (0.95, 0.85, 0.1, 1), "D1_TX": (0.1, 0.75, 0.3, 1), "GND_D": (0.05, 0.05, 0.05, 1),
         "VIN": (0.85, 0.1, 0.1, 1), "GND_P2": (0.05, 0.05, 0.05, 1)}
report = []
for n, spec in JUMP.items():
    top = PIN[n]                                                        # socket centre at the header's top
    d = Vector(spec.get("dir", (-1, 0, 0))).normalized()
    a = math.radians(spec.get("bend_deg", 90))
    box_obj(f"unoq_plug_{n}", top + UP * (PH / 2), (2.54, 2.54, PH), (0.08, 0.08, 0.08, 1))
    # wire: out of the plug's top, a 2 mm-radius bend through `a`, then the lead
    s, r = top + UP * (PH + 0.4), 2.0
    pts = [top + UP * PH, s]
    for k in range(1, 9):
        t = a * k / 8
        pts.append(s + d * (r * (1 - math.cos(t))) + UP * (r * math.sin(t)))
    end_dir = (d * math.sin(a) + UP * math.cos(a)).normalized()
    pts.append(pts[-1] + end_dir * LEAD)
    tube(f"unoq_wire_{n}", [tuple(p) for p in pts], WD, COLOR.get(n, (0.6, 0.6, 0.6, 1)))
    report.append(f"{n}: plug top z {top.z + PH:.1f}, wire highest z {max(p.z for p in pts):.1f}, leaves toward {tuple(round(c, 2) for c in end_dir)}")

# ---- Qwiic: JST SH plug in the board's socket (its outer 2.5 mm shown), the cable to the IMU's plug
qm, qa = Vector(pins_o["qwiic_mouth"]), Vector(pins_o["qwiic_axis"]).normalized()
qend = qm + qa * 2.5
box_obj("unoq_qwiic_plug", qm + qa * 1.25, (2.5, 5.8, 2.6), (0.92, 0.92, 0.9, 1))
imu = Vector(bpy.data.objects["bay_imu_qwiic"]["anchors"]["cable"])
route = [tuple(qend), tuple(qend + qa * 1.8)] + [tuple(p) for p in args.get("qwiic_route", [
    (20.8, -118.0, 135.0), (19.0, -128.5, 131.0), (14.0, -131.0, 129.0),          # down beside the board's far end, round the corner
    (-31.3, -131.0, 127.5), (-33.0, -124.4, 125.8), (-44.0, -124.3, 122.0),       # back along the outside of the bay (as before)
    (-44.0, -121.3, 118.5)])] + [tuple(imu)]
_, qlen = tube("unoq_qwiic_cable", route, args.get("qwiic_d", 2.2), (0.15, 0.3, 0.85, 1))
QL = args.get("qwiic_len", 100.0)
report.append(f"Qwiic cable: route {qlen:.0f} mm between the plugs (cable ~{QL:.0f} mm): " + (f"fits, {QL - qlen:.0f} mm slack" if qlen <= QL else f"TOO SHORT by {qlen - QL:.0f} mm"))
print("unoq wires:\n  " + "\n  ".join(report))
