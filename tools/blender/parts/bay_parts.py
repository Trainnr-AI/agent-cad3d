"""Bay connectors and small hardware at their real sizes, in place (runs inside Blender, after body_v2.py + uno_q.py, before
wiring.py). Everything here is something you buy or crimp; body_v2.py draws the printed parts and the boxes of the boards.

  XT30 pair         battery lead (female) mated with the robot's male pigtail, in a cradle on the bay's rear face (right)
  JST-XH 4-pin      the battery's balance plug, beside the XT30 in the rear-right cradle (charge without opening anything)
  MTS-102 switch    bushing M6 x 0.75, nut, washer and lever on the body box body_v2 draws; three solder lugs
  Mini560 pads      IN+/IN- at the -y end, OUT+/OUT- at the +y end (solder pads; wires leave sideways)
  Uno Q plugs       right-angle 2.54 mm male headers: 3 pins on JANALOG 6/7/8 (UART GND, GND, VIN), 2 pins on JDIGITAL D0/D1
  Qwiic plugs       JST-SH 4-pin at the IMU's right socket and the Uno Q's Qwiic socket; the IMU's two sockets
  USB-C -> USB-A    rigid OTG adapter in the Uno Q's USB-C socket + the camera's USB-A plug in it (sizes ASSUMED until measured)

Each object carries anchor points (layout mm) for wiring.py: ob["anchors"] = {name: [x, y, z]}.
Sizes: XT30 (AMASS XT30U) 10.0 x 5.2 per half, mated 26 long incl. solder cups; JST-XH housing 12.4 x 5.75 x 7.0 (pitch 2.5);
JST-SH (Qwiic) plug 6.0 x 4.25 x 2.95; MTS-102 bushing Ø6 x 9, nut AF 8 x 2, washer Ø10 x 0.5, lever Ø3..2 x 10.
ASSUMED (measure): the battery's lead exit points, the USB adapter and USB-A plug sizes, the lever length.

ARGS = {"robot": "duck_scs0009", "joint_lib": ..., "base_lib": ..., "dims": ...,
        "usb_adapter": [23.0, 15.5, 7.2], "usb_a_plug": [32.0, 16.0, 8.0], "lever_len": 10.0}
"""
import math

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
trunk = bpy.data.objects[f"{R}:trunk_base"]
Mt = trunk.matrix_world.copy()
HOME = Vector((0.0, -100.0, 118.8))
coll = bpy.data.collections["electronics"]
for o in [o for o in coll.objects if o.name.startswith("bay_")]:
    bpy.data.objects.remove(o, do_unlink=True)


def lay_of(name):
    """layout bounding box (mm) of an existing object"""
    o = bpy.data.objects[name]
    pts = [(Mt.inverted() @ (o.matrix_world @ Vector(c))) / MM + HOME for c in o.bound_box]
    return [min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]


def bx(bm, x0, x1, y0, y1, z0, z1):
    return L["add_box"](bm, x0 - HOME.x, x1 - HOME.x, y0 - HOME.y, y1 - HOME.y, z0 - HOME.z, z1 - HOME.z, Mt)


MY = Mt @ Matrix.Rotation(math.radians(-90), 4, "X")       # local +Z = layout +Y; local (x, y) = layout (x, -z)
MX = Mt @ Matrix.Rotation(math.radians(90), 4, "Y")        # local +Z = layout +X; local (x, y) = layout (-z, y)


def cyl_y(bm, x, z, y0, y1, d, segs=24):
    return L["add_cyl"](bm, x - HOME.x, -(z - HOME.z), y0 - HOME.y, y1 - HOME.y, d, segs, MY)


def cyl_x(bm, y, z, x0, x1, d, segs=24):
    return L["add_cyl"](bm, -(z - HOME.z), y - HOME.y, x0 - HOME.x, x1 - HOME.x, d, segs, MX)


made = []


def part(name, builders, rgba, anchors=None):
    bm = L["new_bm"]()
    for f in builders:
        f(bm)
    ob = L["to_object"](name, bm, coll, L["material"]("bay_" + name.split("_")[1], rgba))
    if anchors:
        ob["anchors"] = {k: list(v) for k, v in anchors.items()}
    mw = ob.matrix_world.copy()
    ob.parent = trunk
    ob.matrix_world = mw
    made.append(name)
    return ob


(bx0, by0, bz0), (bx1, by1, bz1) = lay_of("battery_3s")                    # battery: standing, leads leave the top
bcx = (bx0 + bx1) / 2
EXIT_MAIN = (bcx, -105.0, bz1)                                              # ASSUMED: main leads leave the top near its -y end,
EXIT_BAL = (bcx, -93.0, bz1)                                                # the balance lead near its +y end

# ---- XT30 pair + balance plug OUTSIDE, in a printed cradle on the bay's rear face, right half (user 2026-11-19): no inside spot leaves the
# XT30's wires room to bend; outside, the battery can be unplugged and charged without opening anything. The battery's leads and balance lead
# leave through the open rear-right window and round the corner; the robot's pigtail comes back in through it. (body_v2.py: plug_cradle)
XR_ = -53.0                                                                  # the bay's rear face (body_v2.py XR)
XT = dict(x0=XR_ - 0.6 - 5.2, x1=XR_ - 0.6, y0=-110.0, y1=-100.0, z0=90.0, z1=116.0)    # standing, battery (female) half on top, in the middle of the
#   rear face: the right foot reaches behind the bay up to z 108.9 but only out to y -111.2 (leg back 45, knee -45, toe in; 2026-11-19)
part("bay_xt30_pair", [lambda b: bx(b, XT["x0"], XT["x1"], XT["y0"], XT["y1"], XT["z0"] + 4.0, XT["z1"] - 4.0),          # mated housings
                       lambda b: bx(b, XT["x0"] + 0.6, XT["x1"] - 0.6, XT["y0"] + 0.8, XT["y1"] - 0.8, XT["z1"] - 4.0, XT["z1"]),   # solder cups + shrink (battery)
                       lambda b: bx(b, XT["x0"] + 0.6, XT["x1"] - 0.6, XT["y0"] + 0.8, XT["y1"] - 0.8, XT["z0"], XT["z0"] + 4.0)],  # (pigtail)
     (0.95, 0.8, 0.05, 1),
     {"bat_pos": ((XT["x0"] + XT["x1"]) / 2, XT["y1"] - 2.5, XT["z1"]), "bat_neg": ((XT["x0"] + XT["x1"]) / 2, XT["y0"] + 2.5, XT["z1"]),
      "out_pos": ((XT["x0"] + XT["x1"]) / 2, XT["y1"] - 2.5, XT["z0"]), "out_neg": ((XT["x0"] + XT["x1"]) / 2, XT["y0"] + 2.5, XT["z0"])})
# balance plug (JST-XH 4-pin housing 12.4 x 5.75 x 7.0) standing at the rear right above the bay, wires down, mouth up
_bdy = args.get("balance_dy", 0.0)                                           # 2026-11-25: -2.4 (moved right for the Mini560 box on the hood)
BP = dict(x0=XR_ - 1.4 - 7.0, x1=XR_ - 1.4, y0=-121.0 + _bdy, y1=-115.25 + _bdy, z0=128.5, z1=140.9)    # rear right ABOVE the bay's top, behind the Uno Q
#   cover (0.1 from its rear wall), mouth UP (the charger lead plugs in from above); clear of the cable corner below (z < 124)
part("bay_balance_plug", [lambda b: bx(b, BP["x0"], BP["x1"], BP["y0"], BP["y1"], BP["z0"], BP["z1"])], (0.95, 0.95, 0.92, 1),
     {"wires": ((BP["x0"] + BP["x1"]) / 2, (BP["y0"] + BP["y1"]) / 2, BP["z0"])})

# ---- switch hardware on the body box (power_switch: x -41.4..-33.6, y -103.1..-89.6, z 101.4..114.6; bushing toward +y)
(sx0, sy0, sz0), (sx1, sy1, sz1) = lay_of("power_switch")
scx, scz = (sx0 + sx1) / 2, (sz0 + sz1) / 2
lever = args.get("lever_len", 10.0)
part("bay_switch_hw", [lambda b: cyl_y(b, scx, scz, sy1, sy1 + 9.0, 6.0),                       # bushing M6 x 0.75, 9 long
                       lambda b: cyl_y(b, scx, scz, sy1 + 1.6, sy1 + 3.6, 9.2, 6),              # nut AF 8 (corners 9.2) captive in the recess's hex, no washer
                       lambda b: cyl_y(b, scx, scz, sy1 + 9.0, sy1 + 9.0 + lever, 3.0)],        # lever (straight, at mid-throw)
     (0.75, 0.76, 0.8, 1), {"lever_tip": (scx, sy1 + 9.0 + lever, scz)})
LUG = {k: (scx, sy0 - 1.5, scz + dz) for k, dz in (("lug_a", -4.7), ("lug_com", 0.0), ("lug_b", 4.7))}   # (the box includes the 4 mm lugs)
part("bay_switch_joints", [lambda b, dz=dz: bx(b, scx - 1.3, scx + 1.3, sy0 - 3.0, sy0 + 2.0, scz + dz - 1.3, scz + dz + 1.3) for dz in (-4.7, 0.0, 4.7)],
     (0.15, 0.15, 0.15, 1), LUG)                                            # solder joint + 3 mm heat-shrink on each lug

if not args.get("reg_outside", False):                                      # 2026-11-25: Mini560 on the hood (pins + plugs): no solder pads here
    # ---- Mini560 solder pads: IN at the -y end, OUT at the +y end (wires leave along y, under the board's ends)
    (rx0, ry0, rz0), (rx1, ry1, rz1) = lay_of("ubec")
    rcx = (rx0 + rx1) / 2
    # maker's drawing (user 2026-11-19): 30 x 18, castellated pad pairs (2.54 pitch) at both short ends, pad centres 26 apart along the board;
    # "+" pair centre 3.85 from one long edge, "-" pair centre 2.73 from the other. Here: "+" toward the rear (x0), IN at the -y end (EN side).
    PADS = {"in_pos": (rx0 + 3.85, ry0 + 2.0, rz0 - 1.0), "in_neg": (rx1 - 2.73, ry0 + 2.0, rz0 - 1.0),
            "out_pos": (rx0 + 3.85, ry1 - 2.0, rz0 - 1.0), "out_neg": (rx1 - 2.73, ry1 - 2.0, rz0 - 1.0)}
    part("bay_mini560_joints", [lambda b, p=p: bx(b, p[0] - 1.2, p[0] + 1.2, p[1] - 1.2, p[1] + 1.2, p[2] - 1.2, p[2] + 1.0) for p in PADS.values()],
         (0.8, 0.8, 0.82, 1), PADS)

# ---- Uno Q: right-angle male headers on its female headers (board frame -> layout: x - 52.8, y - 126.67; z from the board)
(ux0, uy0, uz0), (ux1, uy1, uz1) = lay_of("unoq_pcb")
utop = uz1                                                                  # PCB top
HDR = 8.5                                                                   # female header height
PITCH = 2.54


def ra_header(n, bx_first, by_row, out_sign):
    """n pins in a row along board x, starting at board x bx_first, on the header row at board y by_row; bent legs point out_sign*y"""
    x0 = bx_first - 52.8 - PITCH / 2
    x1 = x0 + n * PITCH
    yr = by_row - 126.67
    zt = utop + HDR
    return [lambda b: bx(b, x0, x1, yr - 1.27, yr + 1.27, zt, zt + 2.5),                                    # plastic spacer on the header
            lambda b: bx(b, x0 + 0.3, x1 - 0.3, *sorted((yr, yr + out_sign * 9.0)), zt + 0.6, zt + 2.1)]      # bent legs + solder + shrink


JA = lambda pin: 26.4 + PITCH / 2 + (pin - 1) * PITCH                       # JANALOG pin -> board x (14 pins from x 26.4)


def straight_plugs(n, bx_first, by_row):
    """n Dupont male-ended jumper housings (2.54 x 2.54 x 14) standing on the female header; wires leave their tops"""
    x0 = bx_first - 52.8 - PITCH / 2
    yr = by_row - 126.67
    zt = utop + HDR
    return [lambda b: bx(b, x0, x0 + n * PITCH, yr - 1.27, yr + 1.27, zt, zt + 14.0)]


if args.get("straight_plugs", False):                                         # 2026-11-25: the user's plain jumper wires (no right-angle headers)
    part("bay_uno_power_plug", straight_plugs(3, JA(6), 2.54), (0.1, 0.1, 0.1, 1), {"wires": (JA(7) - 52.8, 2.54 - 126.67, utop + HDR + 14.5)})
    part("bay_uno_uart_plug", straight_plugs(2, 60.9, 50.8), (0.1, 0.1, 0.1, 1), {"wires": (62.2 - 52.8, 50.8 - 126.67, utop + HDR + 14.5)})
else:
    part("bay_uno_power_plug", ra_header(3, JA(6), 2.54, -1), (0.1, 0.1, 0.1, 1),
         {"wires": (JA(7) - 52.8, 2.54 - 126.67 - 9.0, utop + HDR + 1.4)})
    part("bay_uno_uart_plug", ra_header(2, 60.9, 50.8, +1), (0.1, 0.1, 0.1, 1),
         {"wires": (62.2 - 52.8, 50.8 - 126.67 + 9.0, utop + HDR + 1.4)})

# ---- Qwiic: IMU sockets (both short ends, opposite corners) + the plug in its right (-y) socket; plug in the Uno Q's socket
(ix0, iy0, iz0), (ix1, iy1, iz1) = lay_of("imu")
izc = (iz0 + iz1) / 2
# the board stands on the rear wall, parts toward +x; imu box = board + parts (6.0 deep); the side-entry sockets overhang the short edges 0.5
# sockets from the product photo: each 7 long along its short edge, from 5 mm off one corner (beside the corner hole) toward the middle
part("bay_imu_qwiic", [lambda b: bx(b, ix1 - 3.1, ix1, iy0 - 0.5, iy0 + 3.75, izc - 1.6, izc + 5.4),          # right socket (upper)
                       lambda b: bx(b, ix1 - 3.1, ix1, iy1 - 3.75, iy1 + 0.5, izc - 5.4, izc + 1.6),          # left socket (lower, unused)
                       lambda b: bx(b, ix1 - 3.05, ix1 - 0.1, iy0 - 4.3, iy0 - 0.5, izc - 1.0, izc + 5.0)],   # plug in the right socket (3.8 out)
     (0.95, 0.95, 0.92, 1), {"cable": (ix1 - 1.6, iy0 - 4.3, izc + 2.0)})

# ---- camera USB (user 2026-11-19): RIGID right-angle USB-C -> USB-A adapter (user's photo) in the Uno Q's rear socket, its USB-A mouth
# facing DOWN; the camera's USB-A plug goes in from below and lies against the bay's rear face in a printed cradle (body_v2.py: usb_cradle),
# so a knock goes into the bay, not the Uno Q's socket. Sizes ASSUMED until measured: adapter 10 thick (along the plug) x 17 wide x 22 tall
# (its USB-C plug axis 5 below its top), camera plug overmold 16 x 8 x 20 beyond the adapter's mouth (12 of its 32 inside the adapter).
(cx0, cy0, cz0), (cx1, cy1, cz1) = lay_of("unoq_usbc")
ccy, ccz = (cy0 + cy1) / 2, (cz0 + cz1) / 2
AD = args.get("usb_adapter_L", (10.0, 17.0, 22.0))
xl1 = cx0 - 0.3
xl0 = xl1 - AD[0]
zt = ccz + 5.0
zm = zt - AD[2]                                                              # the USB-A mouth (facing down)
PL = args.get("usb_a_plug", (8.0, 16.0, 20.0))                               # overmold thick (x) x wide (y) x beyond the mouth (z)
part("bay_usb_adapter_L", [lambda b: bx(b, xl0, xl1, ccy - AD[1] / 2, ccy + AD[1] / 2, zm, zt)], (0.55, 0.57, 0.62, 1))
part("bay_usb_a_plug", [lambda b: bx(b, (xl0 + xl1) / 2 - PL[0] / 2, (xl0 + xl1) / 2 + PL[0] / 2, ccy - PL[1] / 2, ccy + PL[1] / 2, zm - PL[2], zm)],
     (0.1, 0.1, 0.1, 1), {"cable": ((xl0 + xl1) / 2, ccy, zm - PL[2])})
bpy.context.view_layer.update()
for n in made:
    lo, hi = lay_of(n)
    print(f"{n:22s} x {lo[0]:7.1f}..{hi[0]:7.1f}  y {lo[1]:7.1f}..{hi[1]:7.1f}  z {lo[2]:6.1f}..{hi[2]:6.1f}")
print(f"USB-C adapter stands {cx0 - xl0:.1f} mm behind the socket (x {xl0:.1f}); camera plug down to z {zm - PL[2]:.1f}")
