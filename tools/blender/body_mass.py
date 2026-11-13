"""Mass, centre of mass and inertia of every robot body from the parts as built in Blender, for the MuJoCo model
(tools/sim/apply_body_inertia.py). Runs inside Blender at the zero pose.

Grouping is the same as export_bodies.py (nearest robot-body ancestor; servo horns go to the body their joint moves).
Mass of each part, by rule (first match wins; every assumption is listed in the output JSON):
  printed parts (custom prop mass_g_solid_PLA)   solid PLA mass x FILL (default 0.75: walls of 1.6..4.5 mm are mostly solid)
  TPU soles                                       mesh volume x 1.21 g/cm3 (solid)
  SCS0009 case                                    13.2 g (datasheet, incl. lead) minus the horn
  servo horn                                      0.8 g (ASSUMED)
  bearings 6702ZZ / 693ZZ                         4.0 g / 1.0 g (ASSUMED, typical catalogue)
  screws, nuts                                    mesh volume x 7.85 g/cm3 (dummies: head + shank)
  electronics                                     ELEC table below (listing or ASSUMED)
  wiring (hubs, plugs, extension / USB / signal cables; servo leads are in the servo mass)
                                                  hub 1.5 g, plug 0.3 g, cable mesh volume x 2.2 g/cm3; spread over the bodies
                                                  nearest to each of its vertices
Inertia: each closed mesh as a uniform solid of its mass (exact volume integrals); open meshes and wiring as point masses.

ARGS = {"robot": "duck_scs0009", "collections": [...], "fill": 0.75, "out": "/abs/sim/body_mass_v2.json"}
"""
import json

import bmesh
import bpy
import numpy as np
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
R = args["robot"]
FILL = args.get("fill", 0.75)
ELEC = {"unoq_pcb": (26.0, "ASSUMED (UNO form factor SBC)"), "battery_3s": (75.0, "Pro-Range 850 mAh 3S 30C listing (57 x 31, 75 g)"),
        "servo_adapter": (12.0, "ASSUMED"), "ubec": (3.0, "Mini560 5 V 5 A buck module (ASSUMED ~3 g)"), "imu": (2.0, "7Semi BNO086 Qwiic breakout (ASSUMED)"),
        "usb_audio": (12.0, "ASSUMED (Waveshare 0.055 kg is the packed weight)"), "usb_hub": (2.0, "ASSUMED"),
        "speaker": (4.0, "ASSUMED"), "power_switch": (2.0, "ASSUMED"), "tof_sensor": (2.0, "ASSUMED"),
        "cam_ov5640_usb": (10.0, "ASSUMED incl. a short cable"), "cam_ov5640_b": (5.0, "ASSUMED: Waveshare OV5640 (B) 9 x 62 strip board + short USB lead"), "light_led_5mm": (0.4, "5 mm LED")}
ZERO = ("unoq_headers", "unoq_usbc", "unoq_pins", "unoq_bottom_connectors", "unoq_top_parts")        # inside the 26 g of the board
bpy.context.view_layer.update()
bodies = {o.name: o for o in bpy.data.collections[R].objects if o.type == "EMPTY" and o.name.startswith(R + ":")}


def body_name(e):
    return e.name.split(":", 1)[1].split(".")[0]


def owner(o):
    if o.parent is not None and "servo_joint" in o.parent and "_horn" in o.name:
        jb, holder = bpy.data.objects[o.parent["joint_body"]], bpy.data.objects[o.parent["holder_body"]]
        return jb.parent if holder == jb else jb
    p = o.parent
    while p is not None and p.name not in bodies:
        p = p.parent
    return p


def tri_arrays(o, M):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    closed = all(e.is_manifold for e in bm.edges) and len(bm.faces) > 0
    T = np.array([[(M @ v.co)[:] for v in f.verts] for f in bm.faces], dtype=np.float64).reshape(-1, 3, 3)
    V = np.array([(M @ v.co)[:] for v in bm.verts])
    bm.free()
    return T, V, closed


def solid_props(T):
    """volume, centroid, second moments (about the frame origin) of a closed triangle mesh, uniform density 1"""
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    vol6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    V = vol6.sum() / 6.0
    if abs(V) < 1e-15:
        return 0.0, None, None
    C = (vol6[:, None] * (a + b + c)).sum(0) / 24.0 / V
    # integral of x_i x_j over each tetra (origin, a, b, c) = vol/20 * (sum_k p_ki p_kj + (sum_k p_ki)(sum_k p_kj))
    s = T.sum(1)
    S = np.einsum("k,kij,kil->jl", vol6, T, T) / 120.0 + np.einsum("k,kj,kl->jl", vol6, s, s) / 120.0
    return V, C, S / V                                                       # S/V: per unit volume -> scale by mass later


def material(o):
    n = o.name
    if n in ELEC:
        return ELEC[n][0], ELEC[n][1]
    if n in ZERO or n.endswith("_case_screws") or n.endswith("_spline"):
        return 0.0, "in the servo / board mass"
    if n.startswith("scs_") and n.endswith("_case"):
        return 13.2 - 0.8, "SCS0009 13.2 g datasheet, less the horn"
    if "horn_cross" in n:
        return 0.8, "ASSUMED"
    if n.endswith("horn_screw"):
        return 0.1, "M2 x 4"
    if "6702ZZ" in n:
        return 4.0, "ASSUMED"
    if "693" in n or n.startswith("brg_"):
        return 1.0, "ASSUMED"
    if n.endswith("_sole"):
        return None, "TPU 1.21 g/cm3 solid"
    if "screw" in n or "nut_M3" in n:
        return None, "steel 7.85 g/cm3"
    if "mass_g_solid_PLA" in o.keys():
        return o["mass_g_solid_PLA"] * FILL, f"PLA solid x {FILL}"
    return None, "UNKNOWN: PLA 1.24 x fill"


groups = {}
seen = set()
for cname in args["collections"]:
    for o in bpy.data.collections[cname].all_objects:
        if o.type != "MESH" or o.name in seen or not o.visible_get() or o.hide_get() or o.name.endswith((".x", ".hx")):
            continue
        seen.add(o.name)
        b = owner(o)
        if b is not None:
            groups.setdefault(b, []).append(o)
acc = {b: dict(m=0.0, mc=np.zeros(3), S=np.zeros((3, 3)), parts=[]) for b in groups}
unknown = []
trees = {}
for b, objs in groups.items():
    inv = b.matrix_world.inverted()
    allT = []
    for o in objs:
        M = inv @ o.matrix_world
        T, V, closed = tri_arrays(o, M)
        allT.append(T)
        m, why = material(o)
        vol, C, S = solid_props(T) if closed else (0.0, None, None)
        vol_cm3 = abs(vol) * 1e6
        if m is None:
            dens = 1.21 if "TPU" in why else 7.85 if "steel" in why else 1.24 * FILL
            m = vol_cm3 * dens
            if why.startswith("UNKNOWN"):
                unknown.append(o.name)
        if m <= 0:
            continue
        if C is None or S is None:                                           # open mesh: its vertices as point masses
            C = V.mean(0)
            S = (V[:, :, None] * V[:, None, :]).mean(0)
        a = acc[b]
        a["m"] += m
        a["mc"] += m * C
        a["S"] += m * S
        a["parts"].append((o.name, round(m, 2), why))
    trees[b] = BVHTree.FromPolygons([tuple(p) for p in np.concatenate(allT).reshape(-1, 3)], [(3 * i, 3 * i + 1, 3 * i + 2) for i in range(sum(len(t) for t in allT))])
# wiring: spread each object's mass over the bodies nearest its vertices (all in world frame here)
wire = bpy.data.collections.get("wiring")
wire_total = 0.0
if wire:
    wtrees = {}
    for b in groups:
        Mw = b.matrix_world
        T = []
        for o in groups[b]:
            T.append(tri_arrays(o, o.matrix_world)[0])
        T = np.concatenate(T).reshape(-1, 3)
        wtrees[b] = BVHTree.FromPolygons([tuple(p) for p in T], [(3 * i, 3 * i + 1, 3 * i + 2) for i in range(len(T) // 3)])
    for o in wire.objects:
        if o.type != "MESH" or o.name.startswith("wire_lead_"):
            continue
        if o.name.startswith("wire_hub_") and "_socket" not in o.name:
            m = 1.5
        elif "_socket" in o.name:
            continue
        elif o.name.startswith("wire_plug_"):
            m = 0.3
        else:
            T, V, closed = tri_arrays(o, o.matrix_world)
            m = abs(solid_props(T)[0]) * 1e6 * 2.2 if closed else 0.0
        if m <= 0:
            continue
        Vw = np.array([(o.matrix_world @ v.co)[:] for v in o.data.vertices])
        Vw = Vw[:: max(1, len(Vw) // 200)]
        mv = m / len(Vw)
        for p in Vw:
            from mathutils import Vector
            best = min(wtrees, key=lambda b: wtrees[b].find_nearest(Vector(p))[3])
            q = np.array((best.matrix_world.inverted() @ Vector(p))[:])
            a = acc[best]
            a["m"] += mv
            a["mc"] += mv * q
            a["S"] += mv * np.outer(q, q)
        wire_total += m
out = {}
for b, a in acc.items():
    m = a["m"]
    c = a["mc"] / m
    S = a["S"] - m * np.outer(c, c)                                          # second moment about the COM
    I = np.trace(S) * np.eye(3) - S                                           # inertia tensor (kg m2 with m in g -> /1000 below)
    I /= 1000.0
    out[body_name(b)] = dict(mass=m / 1000.0, pos=c.tolist(), fullinertia=[I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]],
                             parts=sorted(a["parts"], key=lambda t: -t[1]))
total = sum(v["mass"] for v in out.values())
json.dump(dict(fill=FILL, electronics=ELEC, wiring_g=wire_total, unknown=unknown, total_kg=total, bodies=out), open(args["out"], "w"), indent=1)
for n, v in sorted(out.items(), key=lambda kv: -kv[1]["mass"]):
    print(f"  {n:16} {v['mass'] * 1000:7.1f} g  com {np.round(np.array(v['pos']) * 1000, 1)} mm  ({len(v['parts'])} parts)")
print(f"total {total * 1000:.1f} g (wiring {wire_total:.1f} g); unknown-material parts: {unknown}")
