"""UNO Q shield: board setup + placement (re-runnable). Run with KiCad's Python (pcbnew), e.g. the KiCad MCP server's venv:
    vendor/KiCAD-MCP-Server/venv/bin/python tools/kicad/shield_layout.py projects/xgoduck_scs0009/pcb/unoq_shield/unoq_shield.kicad_pcb

- 4 copper layers, JLCPCB-safe rules (0.15 mm clearance / 0.2 mm track / 0.3 mm via drill).
- UNO R3 outline + header positions + 4 x M3 holes, from KiCad's Module:Arduino_UNO_R3(_WithMountingHoles) footprint
  (frame: header pin 1 = BOOT/NC of the power header; placed at ORIGIN on the board). The four headers sit on B.Cu (the shield
  plugs DOWN into the UNO Q's female headers; every other part on F.Cu, away from the UNO).
- Placement table PLACE: ref -> (x, y, rotation deg CCW, side) in board mm.
Prints every header pin's final position against its target so a wrong orientation shows at once.
"""
import sys

import pcbnew

BOARD = sys.argv[1] if len(sys.argv) > 1 else "projects/xgoduck_scs0009/pcb/unoq_shield/unoq_shield.kicad_pcb"
MM = pcbnew.FromMM
ORIGIN = (100.0, 100.0)                       # board position of the UNO frame's (0, 0) = power header pin 1
FPLIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"

b = pcbnew.LoadBoard(BOARD)


def P(x, y):
    return pcbnew.VECTOR2I(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))


# ---------------------------------------------------------------- stackup and rules
b.SetCopperLayerCount(4)
ds = b.GetDesignSettings()
ds.m_MinClearance = MM(0.15)
ds.m_TrackMinWidth = MM(0.15)
ds.m_ViasMinSize = MM(0.5)
ds.m_MinThroughDrill = MM(0.3)
ds.m_CopperEdgeClearance = MM(0.3)
nc = ds.m_NetSettings.GetDefaultNetclass()
nc.SetClearance(MM(0.15)); nc.SetTrackWidth(MM(0.25)); nc.SetViaDiameter(MM(0.6)); nc.SetViaDrill(MM(0.3))

# ---------------------------------------------------------------- M3 mounting holes (UNO pattern)
# (never b.Remove(): in KiCad 10.0.8's SWIG every later call then gets unwrapped objects - move what exists, add what is missing)
fp = {f.GetReference(): f for f in list(b.Footprints())}
for i, (x, y) in enumerate([(-13.97, 0.0), (-12.7, 48.26), (38.1, 5.08), (38.1, 33.02)], 1):
    h = fp.get(f"H{i}")
    if h is None:
        h = pcbnew.FootprintLoad(FPLIB + "/MountingHole.pretty", "MountingHole_3.2mm_M3")
        h.SetReference(f"H{i}"); b.Add(h); fp[f"H{i}"] = h
    h.SetPosition(P(x, y))

def pads(f):
    return {p.GetNumber(): (pcbnew.ToMM(p.GetPosition().x) - ORIGIN[0], pcbnew.ToMM(p.GetPosition().y) - ORIGIN[1]) for p in f.Pads()}


# ---------------------------------------------------------------- the four UNO headers, on B.Cu, pins on the UNO's positions
HDR = {"J3": [(2.54 * i, 0.0) for i in range(8)],                      # BOOT(NC) IOREF RESET 3V3 5V GND GND VIN
       "J4": [(22.86 + 2.54 * i, 0.0) for i in range(6)],              # A0..A5
       "J5": [(35.56 - 2.54 * i, 48.26) for i in range(8)],            # D0..D7
       "J6": [(13.72 - 2.54 * i, 48.26) for i in range(10)]}           # D8..D13 GND AREF SDA SCL
for ref, tgt in HDR.items():
    f = fp[ref]
    if not f.IsFlipped():
        f.Flip(f.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    best = None
    for ang in (0, 90, 180, 270):
        f.SetOrientationDegrees(ang)
        p1 = pads(f)["1"]
        f.SetPosition(pcbnew.VECTOR2I(f.GetPosition().x + MM(tgt[0][0] - p1[0]), f.GetPosition().y + MM(tgt[0][1] - p1[1])))
        err = max(abs(pads(f)[str(i + 1)][k] - tgt[i][k]) for i in range(len(tgt)) for k in (0, 1))
        if best is None or err < best[0]:
            best = (err, ang)
    f.SetOrientationDegrees(best[1])
    p1 = pads(f)["1"]
    f.SetPosition(pcbnew.VECTOR2I(f.GetPosition().x + MM(tgt[0][0] - p1[0]), f.GetPosition().y + MM(tgt[0][1] - p1[1])))
    print(f"{ref}: B.Cu, {best[1]} deg, worst pin error {best[0]:.3f} mm")

# ---------------------------------------------------------------- everything else (board mm, UNO frame), F.Cu
PLACE = {
    # power in (right edge): XT30 mouth out of the right edge (pad 2 = +, pad 1 = -, per the footprint's silk), the P-FET
    # turned so BAT+ enters its top (source) row next to the XT30 and VBAT_SW leaves its bottom (drain) row toward the buck
    "J1": (28.0, 19.0, 270), "Q1": (22.0, 12.0, 270),
    "R1": (17.0, 6.5, 0), "C1": (20.0, 6.5, 0), "R2": (16.5, 9.5, 90),
    # buck (SY8368A land pattern: LX is boxed in by GND pads 1/3 on the left and IN pins on the right; its bottom edge pairs
    # GND pad 3 with IN pin 4): both input caps straight under it across those two pads (smallest hot loop), the bulk cap
    # beside; LX leaves through the gap between the GND pads to the inductor on the LEFT; BS cap next to the inductor's LX pad
    "U1": (13.0, 20.0, 0), "C2": (13.2, 23.4, 180), "C3": (13.2, 26.3, 180), "C4": (20.5, 27.5, 0),
    "L1": (2.0, 20.4, 180), "C5": (10.0, 16.9, 270), "C6": (14.0, 16.0, 90),
    "R3": (7.5, 12.0, 0), "R4": (10.5, 12.0, 0), "C7": (7.5, 13.6, 0),
    "R5": (12.0, 9.8, 0), "R6": (15.0, 12.0, 90), "R7": (12.0, 8.2, 0),
    # output caps right at the inductor's V_SERVO pad, bulk cap toward the servo connectors
    "C8": (-8.0, 15.0, 180), "C9": (-8.0, 18.2, 180), "C10": (-8.0, 22.6, 180), "C11": (-8.0, 25.8, 180),
    "C12": (-11.0, 31.0, 0),
    # battery sense by A2 (J4 pin 3 at x 27.94)
    "R8": (24.0, 3.5, 0), "R9": (28.0, 3.5, 0), "C13": (31.5, 3.5, 0),
    # 3V3 LDO by the 5V pin (J3 pin 5 at x 10.16)
    "U4": (8.0, 4.0, 0), "C16": (4.5, 4.0, 90), "C17": (11.5, 4.0, 90),
    # servo bus logic by D0..D2 (bottom right), output to the connectors on the left
    "Q2": (22.0, 40.0, 0), "R10": (18.5, 40.0, 90), "R11": (22.0, 36.0, 0), "R12": (25.0, 40.0, 90),
    "U2": (28.5, 39.0, 0), "U3": (33.0, 39.0, 0), "C14": (28.5, 43.0, 0), "C15": (33.0, 43.0, 0),
    "R13": (28.5, 35.5, 0), "R14": (33.0, 35.5, 0),
    # left edge: power slide switch (lever out of the left edge), panel-switch header, Qwiic, four servo connectors
    "SW1": (-22.0, 6.0, 270), "J2": (-15.0, 10.0, 0), "J11": (-5.5, 6.5, 180), "R19": (-8.0, 12.0, 0), "R20": (-4.5, 12.0, 0),
    "J7": (-24.0, 18.0, 0), "J8": (-24.0, 25.0, 0), "J9": (-24.0, 32.0, 0), "J10": (-24.0, 39.0, 0),
}
for ref, (x, y, ang) in PLACE.items():
    f = fp[ref]
    if f.IsFlipped():
        f.Flip(f.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    f.SetOrientationDegrees(ang)
    f.SetPosition(P(x, y))
# ---------------------------------------------------------------- outline (UNO R3, with the beveled corners): reuse segments
OUTLINE = [(-27.94, -2.54), (38.1, -2.54), (38.1, 0.0), (40.64, 2.54), (40.64, 35.31), (38.1, 37.85), (38.1, 49.28),
           (36.58, 50.8), (-27.94, 50.8)]
edges = [d for d in list(b.GetDrawings()) if d.GetLayer() == pcbnew.Edge_Cuts]
for k, (a_, c_) in enumerate(zip(OUTLINE, OUTLINE[1:] + OUTLINE[:1])):
    if k < len(edges):
        sh = edges[k]
    else:
        sh = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_SEGMENT); sh.SetLayer(pcbnew.Edge_Cuts); sh.SetWidth(MM(0.1)); b.Add(sh)
    sh.SetShape(pcbnew.SHAPE_T_SEGMENT); sh.SetStart(P(*a_)); sh.SetEnd(P(*c_))
for sh in edges[len(OUTLINE):]:                                   # extra old edges: park them on the outline (zero-length)
    sh.SetStart(P(*OUTLINE[0])); sh.SetEnd(P(*OUTLINE[0]))

# ---------------------------------------------------------------- net class for the moderate-current nets routed as tracks
ns = ds.m_NetSettings
try:
    pw = pcbnew.NETCLASS("PWR"); pw.SetTrackWidth(MM(0.6)); pw.SetClearance(MM(0.2)); pw.SetViaDiameter(MM(0.8)); pw.SetViaDrill(MM(0.4))
    ns.SetNetclass("PWR", pw)
    for n in ("/VBAT_SW", "/+5V_UNO", "/+3V3", "/U1_VCC"):
        ns.SetNetclassPatternAssignment(n, "PWR")
except Exception as ex:                                             # API differences between KiCad builds: not fatal
    print("netclass PWR not set:", ex)

# ---------------------------------------------------------------- copper zones (the high-current paths), named, updated in place
def net(name):
    return b.FindNet(name)


BOARD_POLY = OUTLINE
ZONES = [  # (name, net, layer, priority, polygon in UNO frame)
    ("GND_top", "/GND", pcbnew.F_Cu, 0, BOARD_POLY),
    ("GND_in1", "/GND", pcbnew.In1_Cu, 0, BOARD_POLY),
    ("GND_in2", "/GND", pcbnew.In2_Cu, 0, BOARD_POLY),
    ("GND_bot", "/GND", pcbnew.B_Cu, 0, BOARD_POLY),
    ("VSERVO_in2", "/V_SERVO", pcbnew.In2_Cu, 1, [(-27.6, -2.2), (8.0, -2.2), (8.0, 50.5), (-27.6, 50.5)]),
    ("BAT_top", "/BAT+", pcbnew.F_Cu, 2, [(20.6, 7.6), (30.6, 7.6), (30.6, 16.4), (25.4, 16.4), (25.4, 10.6), (20.6, 10.6)]),
    ("VBATSW_top", "/VBAT_SW", pcbnew.F_Cu, 2, [(14.6, 13.6), (25.0, 13.6), (25.0, 24.0), (19.4, 24.0), (19.4, 29.4),
                                                  (16.0, 29.4), (16.0, 27.4), (14.6, 27.4)]),
    ("LX_top", "/LX", pcbnew.F_Cu, 2, [(13.6, 19.95), (13.6, 20.85), (10.6, 20.85), (10.6, 23.0), (4.4, 23.0), (4.4, 17.0),
                                        (10.6, 17.0), (10.6, 19.95)]),
    ("VSERVO_top", "/V_SERVO", pcbnew.F_Cu, 2, [(-7.6, 13.4), (-0.4, 13.4), (-0.4, 27.4), (-7.6, 27.4)]),
]
have = {z.GetZoneName(): z for z in list(b.Zones())}
for name, netname, layer, prio, poly in ZONES:
    zz = have.get(name)
    if zz is None:
        zz = pcbnew.ZONE(b); zz.SetZoneName(name); b.Add(zz)
    zz.SetLayer(layer); zz.SetNet(net(netname)); zz.SetAssignedPriority(prio)
    zz.SetLocalClearance(MM(0.2)); zz.SetMinThickness(MM(0.2))
    zz.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)     # no floating copper islands
    zz.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)                # solid: no starved thermal spokes on the small GND pads
    ol = zz.Outline(); ol.RemoveAllContours(); ol.NewOutline()
    for x, y in poly:
        ol.Append(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))
print("zones:", [z.GetZoneName() for z in list(b.Zones())])

# ---------------------------------------------------------------- vias (locked: Freerouting's SES import keeps locked items)
def free(x, y, r_clear):
    """no pad of any net, no edge, no hole within r_clear of (x, y) (UNO frame); zones re-fill round a via by themselves"""
    pt = P(x, y)
    for f in fp.values():
        for pd in f.Pads():
            if pd.HitTest(pt, MM(r_clear)):
                return False
    xs = [p[0] for p in OUTLINE]; ys = [p[1] for p in OUTLINE]
    return min(xs) + 0.8 < x < max(xs) - 2.8 and min(ys) + 0.8 < y < max(ys) - 0.8


def inside(poly, x, y):
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def via(x, y, netname, d=0.6, drill=0.3):
    v = pcbnew.PCB_VIA(b); v.SetPosition(P(x, y)); v.SetWidth(MM(d)); v.SetDrill(MM(drill))
    v.SetNet(net(netname)); v.SetLocked(True); b.Add(v)


POWER_POLYS = [z[4] for z in ZONES if z[3] == 2]
n_gnd = n_vs = 0
# U1's two GND pads are boxed in (LX channel, signal pins): a 0.3 / 0.5 mm via in each, into the In1 GND plane
via(11.7, 19.5, "/GND", 0.5, 0.3); via(11.7, 21.25, "/GND", 0.5, 0.3); n_gnd += 2
# LX: the land pattern boxes the LX pad in; it leaves only through the 0.8 mm gap between GND pads 1 and 3. A locked 0.45 mm
# track through that gap to the inductor's LX pad guarantees the joint (>= 0.165 mm to each GND pad); the LX pour widens it
lx = pcbnew.PCB_TRACK(b); lx.SetStart(P(13.0, 20.39)); lx.SetEnd(P(6.75, 20.39)); lx.SetWidth(MM(0.45)); lx.SetLayer(pcbnew.F_Cu)
lx.SetNet(net("/LX")); lx.SetLocked(True); b.Add(lx)
# V_SERVO: top copper round the inductor / output caps down to the In2 plane, and the bulk cap's pad
for x in (-6.6, -5.4, -1.6, -0.9):
    for y in (14.2, 16.6, 19.0, 21.8, 24.2, 26.6):
        if free(x, y, 0.55):
            via(x, y, "/V_SERVO", 0.8, 0.4); n_vs += 1
for x, y in ((-14.7, 32.9), (-14.7, 29.1), (-16.2, 31.0)):
    if free(x, y, 0.55):
        via(x, y, "/V_SERVO", 0.8, 0.4); n_vs += 1
# GND stitching grid (2.6 mm), not inside the power pours, clear of every pad
xs = [p[0] for p in OUTLINE]; ys = [p[1] for p in OUTLINE]
gx = min(xs) + 1.5
while gx < max(xs) - 1.5:
    gy = min(ys) + 1.5
    while gy < max(ys) - 1.5:
        if not any(inside(pp, gx, gy) for pp in POWER_POLYS) and free(gx, gy, 0.65):
            via(gx, gy, "/GND"); n_gnd += 1
        gy += 2.6
    gx += 2.6
print("vias: GND", n_gnd, "V_SERVO", n_vs)
for i in range(1, 5):                                              # mounting holes: board-only (not in the schematic)
    h = fp[f"H{i}"]; h.SetBoardOnly(True); h.SetExcludedFromBOM(True); h.SetExcludedFromPosFiles(True)
missing = [r for r in fp if r not in PLACE and r not in HDR and not r.startswith("H")]
print("not placed:", missing)
b.Save(BOARD)
print("saved", BOARD)
