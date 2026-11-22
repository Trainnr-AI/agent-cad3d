"""Tolerance coupon: the fits of the nanoduck printed at their real sizes, from the SAME joint-library code that cuts the robot's
parts (so a fit that works here works there). Print it first, with the real horn, bearings, nuts, screws and a servo in hand.
Seven small pieces in the collection `coupon`, laid out side by side away from the robot (re-running replaces them):

  coupon_knee_rotor   = the knee shin's horn/bearing plate (c2m_rotor_features): full X horn pocket + crush ribs, clamp hole +
                        counterbore, 6702ZZ seat Ø21.1 x 4 open to the top face, relief under the inner ring. Print seat face DOWN.
  coupon_knee_cap     = the knee cap's centre (C2M constants): Ø14.9 boss into the bearing bore, shoulder on the inner ring,
                        Ø5.2 driver hole. Print top face DOWN (as the real cap).
  coupon_trim_plate   = a trimmed-horn plate (horn_plate_features, trim r 6): hips, head yaw / roll, beak.
  coupon_bar          = 693ZZ seat Ø8.1 + lip (bearing plates), M3 nylon-nut trap (AF 5.75) + Ø3.3 hole, M2 pilot Ø1.7 (blind),
                        M2 clearance Ø2.4, M3 pilot Ø2.5 (blind, Uno Q posts), and the 28 mm bridge (1.6 mm roof over a 28 x 8
                        channel, like the knee holder's base). Print flat (the channel is underneath and prints as a bridge).
  coupon_servo_collar = the servo body (12.1 x 23.3) + 0.25 clearance all round, 1.35 walls, 8 high (the bearing caps' saddles).
  coupon_link_boss_sup / _nosup = a type-C rotor's centre (link_features) on a foot, printed with the joint axis LYING DOWN
                        (print up = +Y), with / without supports: 1 / 2 dimples. See piece 6 below.

ARGS = {"joint_lib": ".../joint_v2.py", "base_lib": ".../joint_orig_style.py", "dims": ".../scs0009_dims.py",
        "collection": "coupon", "origin": [0.30, 0.15, 0.0]}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
COLL = args.get("collection", "coupon")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):                                   # idempotent: only this collection's own objects are replaced
    bpy.data.objects.remove(o, do_unlink=True)

L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
O = Vector(args.get("origin", (0.30, 0.15, 0.0)))
T = lambda x_mm, y_mm: Matrix.Translation(O + Vector((x_mm, y_mm, 0.0)) / 1000.0)
nb, box, cyl, hull, comb = L["new_bm"], L["add_box"], L["add_cyl"], L["add_hull"], L["combine_cutters"]
mat = L["material"]("coupon", (0.95, 0.75, 0.25, 1))
parts = []

# 1. knee rotor: exactly the shin's plate round the horn (no link outline)
M1 = T(0, 0)
ra, rs, rr = L["c2m_rotor_features"](M1)
parts.append(L["build_part"]("coupon_knee_rotor", ra, rs, coll, mat, rr))

# 2. knee cap centre: disc + boss + shoulder + driver hole (the post / arm / screws are left off: they fit the thigh, not the bearing)
M2 = T(36, 0)
z0, z1 = L["C2M_CAP_Z"]
add2 = [cyl(nb(), 0, 0, z0, z1, 2 * L["C2M_CAP_R"], 64, M2),
        cyl(nb(), 0, 0, L["C2M_BOSS"][2], z0 + 0.3, 2 * L["C2M_BOSS"][1], 64, M2),
        cyl(nb(), 0, 0, L["PLATE_Z1"] + 0.1, z0 + 0.2, 2 * L["C2M_SHOULDER_R"], 48, M2)]
sub2 = [cyl(nb(), 0, 0, L["C2M_BOSS"][2] - 0.5, z1 + 0.5, L["C2M_DRIVE_D"], 32, M2)]
parts.append(L["build_part"]("coupon_knee_cap", [comb(add2)], sub2, coll, mat))

# 3. trimmed-horn plate (r 6), as the hips / head yaw / head roll / beak
M3 = T(64, 0)
ta, ts, tr = L["horn_plate_features"](M3, trim_r=L["TRIM_R"])
parts.append(L["build_part"]("coupon_trim_plate", ta, ts, coll, mat, tr))

# 4. bar: 693 seat, M3 nut trap, M2 pilot / clearance, M3 pilot, 28 mm bridge
M4 = T(0, -30)
BT = L["BP_T"]                                                 # 4.8: the bearing plates' thickness
add4 = [box(nb(), -6.0, 72.0, -8.0, 8.0, 0.0, BT, M4)]
sub4 = [cyl(nb(), 0.0, 0.0, BT - L["B693"][2], BT + 0.5, L["BP_SEAT_D"], 48, M4),            # 693ZZ seat Ø8.1 x 4.0 from the top ...
        cyl(nb(), 0.0, 0.0, -0.5, BT - L["B693"][2] + 0.01, L["BP_LIP_D"], 32, M4)]          # ... on a 0.8 lip with a Ø6 hole
af, nt = L["NUT_AF"], L["NUT_T"]                                                              # M3 nut trap from below + Ø3.3 hole
hexr = af / math.sqrt(3)
sub4.append(hull(nb(), [(12.0 + hexr * math.cos(math.radians(60 * i)), hexr * math.sin(math.radians(60 * i)), z)
                        for i in range(6) for z in (-0.5, nt)], M4))
sub4.append(cyl(nb(), 12.0, 0.0, -0.5, BT + 0.5, L["NUT_HOLE_D"], 20, M4))
sub4.append(cyl(nb(), 20.0, 0.0, 0.8, BT + 0.5, L["PILOT_D"], 14, M4))                      # M2 pilot, blind (0.8 floor)
sub4.append(cyl(nb(), 26.0, 0.0, -0.5, BT + 0.5, L["CLEAR_D"], 20, M4))                     # M2 clearance, through
sub4.append(cyl(nb(), 32.0, 0.0, 0.8, BT + 0.5, 2.5, 16, M4))                               # M3 thread-forming pilot (Uno Q posts), blind
sub4.append(box(nb(), 38.0, 66.0, -4.0, 4.0, -0.5, BT - 1.6, M4))                           # 28 mm channel under a 1.6 mm roof
parts.append(L["build_part"]("coupon_bar", [comb(add4)], sub4, coll, mat))

# 5. servo collar: the body + 0.25 all round, 1.35 walls (the saddle walls of the bearing caps), 8 high
M5 = T(36, -30 - 26)
hw, hl = L["J_BODY_HW"] + L["SADDLE_CLEAR"], (L["J_BODY_X"][1] - L["J_BODY_X"][0]) / 2 + L["SADDLE_CLEAR"]
wt = L["SADDLE_T"]
add5 = [box(nb(), -hl - wt, hl + wt, -hw - wt, hw + wt, 0.0, L["SADDLE_DEPTH"], M5)]
sub5 = [box(nb(), -hl, hl, -hw, hw, -0.5, L["SADDLE_DEPTH"] + 0.5, M5)]
parts.append(L["build_part"]("coupon_servo_collar", [comb(add5)], sub5, coll, mat))

# 6. lying-down boss: a type-C rotor's centre (link_features: Ø14.9 boss into the 6702 bore, shoulder on the inner ring, trimmed X
#    pocket inside the boss, clamp hole + counterbore) on a foot, so it prints with the joint axis LYING DOWN - as the L-brackets,
#    yaw connectors, yaw block and head-pitch U must (two bearing axes at right angles). The slicing check (print/slicing/) puts
#    support under the boss (r 7.4) and in the horn pocket (r 6.2) there. Two copies: _sup printed WITH tree supports (as the robot
#    would be), _nosup WITHOUT (does the boss underside / pocket roof come out usable anyway?). Marks on the plate's outer face:
#    1 dimple = _sup, 2 dimples = _nosup. Print up = +Y (foot on the bed).
FOOT_Y = (-16.5, -13.5)
for k, (stem, dimples) in enumerate((("coupon_link_boss_sup", 1), ("coupon_link_boss_nosup", 2))):
    M6 = T(70 + 30 * k, -60)
    la, ls, lr = L["link_features"](M6, [(-7.0, FOOT_Y[0]), (7.0, FOOT_Y[0])])
    la = la + [box(nb(), -7.0, 7.0, FOOT_Y[0], FOOT_Y[1], L["BOSS_Z0"], L["PLATE_Z1"], M6)]   # foot: the full depth, stands steady
    ls = ls + [cyl(nb(), -3.0 + 6.0 * i if dimples == 2 else 0.0, -11.0, L["PLATE_Z1"] - 0.6, L["PLATE_Z1"] + 0.5, 1.6 + 0.07 * i,
                   16 + 2 * i, M6) for i in range(dimples)]
    parts.append(L["build_part"](stem, [comb(la)], ls, coll, mat, lr))

out = []
for o in parts:
    bm = bmesh.new()
    bm.from_mesh(o.data)
    vol = abs(bm.calc_volume()) * 1e9
    nm = sum(not e.is_manifold for e in bm.edges)
    bm.free()
    o["mass_g_solid_PLA"] = round(vol / 1000 * 1.24, 2)
    out.append(f"{o.name}: {L['shells'](o)} piece(s), non-manifold {nm}, {vol / 1000 * 1.24:.1f} g solid, "
               f"{' x '.join(f'{d * 1000:.1f}' for d in o.dimensions)} mm")
print("tolerance coupon built:\n  " + "\n  ".join(out))
