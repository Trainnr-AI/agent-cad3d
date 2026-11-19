"""Feetech SCS0009 dimensions (mm). Pure data; exec'd by parts/scs0009.py and the joint libraries.

Datasheet frame: x from the left ear tip, y across the body (0 = centre), z from the case bottom.
Joint frame (J_*): origin on the shaft axis at the tower top (25.25), +Z out of the shaft, +X toward
the connector end. Exact values from Feetech's STEP (tools/cad/scs0009_from_step.py) unless marked.
"""
EAR_TIP_L, EAR_TIP_R = 0.0, 32.5
BODY_X0, BODY_X1 = 4.6, 27.9            # 23.3 long
SHAFT_X = 10.55                         # 5.7 from the body centre (16.25)
HOLE_X = (2.0, 30.5)                    # ear holes 2 x Ø2, 28.5 apart
HOLE_D = 2.0
BODY_W = 12.1
EAR_W = 11.0
EAR_Z0, EAR_Z1 = 16.8, 18.4             # flange 1.6 thick ...
EAR_RECESS, EAR_RIM = 0.6, 0.7          # ... underside recessed 0.6 inside a 0.7 rim (ear hole has a Ø3.6 relief)
BOTTOM_SEAM, MIDDLE_SEAM = 5.8, 15.8    # bottom case | core middle | top case
CASE_TOP = 21.45                        # main case top, flat for x <= ROOF_X0
ROOF_X0, ROOF_SLOPE = 20.84, 0.114      # beyond it the roof slopes down 0.114 mm/mm (6.5 deg) to ~20.75 at x = 26.8
ROOF_PANEL_RECESS, ROOF_END = 0.2, 20.68   # label panel recessed 0.2 inside a rim; end rim 20.68 at x = 27.9
TOWER_X1 = 19.75                        # gear tower: Ø12 round about the shaft, flat end at x = 19.75
TOWER_TOP = 25.25
TOWER_R = 6.0
BOSS_D, BOSS_H = 5.76, 0.3              # raised boss round the shaft on the tower top -> 25.55
BOSS_TONGUE_X1, BOSS_TONGUE_HW = 17.75, 2.88
BOSS_TOP = TOWER_TOP + BOSS_H
SPLINE_OD, SPLINE_ROOT_D, SPLINE_TEETH = 3.95, 3.45, 20
SPLINE_Z0, SPLINE_Z1, SPLINE_CHAMFER = 25.95, 28.45, 0.285
NECK_D, NECK_Z0 = 3.45, 25.15
JOURNAL_D = 4.0                         # Ø4.0 journal in a Ø4.03 plastic bore (no bearing in the servo)
HORN_SCREW_HOLE_D, HORN_SCREW_HOLE_DEPTH = 1.6, 5.0
HORN_SCREW = (2.0, 4.0, 3.5, 1.3)       # kit M2.0x4 self-tapping: thread Ø, length, head Ø, head height (head inferred)
CASE_SCREW_X = (5.85, 26.65)            # 4 screws from the bottom at x = 16.25 ± 10.4, y = ±4.8
CASE_SCREW_Y = (-4.8, 4.8)
CASE_SCREW = (1.0, (20.75, 20.13), 2.0, 0.8, 0.2)
CABLE_EXIT = ((4.6, 6.35), (-1.5, 1.5), (0.0, 2.3))   # slot in the SHAFT-end face, bottom, centred (x, y, z ranges)
EAR_SCREW = (2.0, 3.5, 1.3)             # kit M2 self-tapping into printed pads (thread length chosen per design)
MASS_KG = 0.0132
# ---- Horn: kit lengths from Feetech's listing; section NOMINAL until measured ----
HORN_HUB_D, HORN_HUB_H = 6.5, 4.0       # (NOMINAL)
HORN_PLATE_T = 1.6                      # (NOMINAL)
HORN_DISC_D = 17.76
HORN_CROSS_L = 11.94                    # 23.88 tip to tip; shaft centre -> outermost point (round tip included)
# MEASURED 2026-11-20 (user, caliper): 24.0 tip to tip (r 12.0). Kept 11.94 here so the code matches the built parts: the full-horn
# pockets reach r 12.24, so the real horn has 0.24 at the tips (design clearance 0.20) - no rebuild needed.
HORN_ARM_L = 20.12 - HORN_HUB_D / 2     # single arm 20.12 overall (hub edge -> tip)
HORN_DOUBLE_ARM_L = 34.49 / 2
# arm widths and holes measured off Feetech's kit photo (2026-11-13), scaled by the listing's lengths (X 23.88 tip to tip:
# 7.36 px/mm; double arm 34.49: 7.28 px/mm). Width 4.84 at r 4.1, 4.60 at r 6.8, 4.32 at r 9.6 -> linear 5.2 at the axis,
# 4.3 where the round tip starts (+-0.15). Inner corner between arms 3.76 from the axis (fillet ~0.4). Was 5.6 -> 3.6 (nominal):
# the full-length pockets would have been 0.3 too narrow near the tips. Hub and thicknesses still NOMINAL (not in a top view).
HORN_ARM_W_ROOT, HORN_ARM_W_TIP = 5.2, 4.3
HORN_HOLE_D, HORN_HOLE_R = 1.0, (4.9, 7.4, 9.8)   # row of small holes along each arm (photo; hole size not resolvable)
HORN_CROSS_STRAIGHT = HORN_CROSS_L - HORN_ARM_W_TIP / 2   # 9.79: where the straight arm ends and the round tip starts

# ---- the same in the joint frame (mm) ----
J_BODY_X = (BODY_X0 - SHAFT_X, BODY_X1 - SHAFT_X)        # (-5.95, 17.35)
J_EAR_X = (EAR_TIP_L - SHAFT_X, EAR_TIP_R - SHAFT_X)     # (-10.55, 21.95)
J_HOLE_X = (HOLE_X[0] - SHAFT_X, HOLE_X[1] - SHAFT_X)    # (-8.55, 19.95)
J_EAR_Z = (EAR_Z0 - TOWER_TOP, EAR_Z1 - TOWER_TOP)       # (-8.45, -6.85)
J_BODY_HW, J_EAR_HW = BODY_W / 2, EAR_W / 2              # 6.05, 5.5
J_CASE_TOP = CASE_TOP - TOWER_TOP                        # -3.8
J_ROOF_X0 = ROOF_X0 - SHAFT_X                            # 10.29
J_BOTTOM = -TOWER_TOP                                    # -25.25
J_TOWER_X1 = TOWER_X1 - SHAFT_X                          # 9.2
J_BOSS_TOP = BOSS_H                                      # 0.3
J_SPLINE_TIP = SPLINE_Z1 - TOWER_TOP                     # 3.2
J_HORN_HUB_Z = (BOSS_H, BOSS_H + HORN_HUB_H)             # hub sits on the boss: 0.3 .. 4.3 (nominal)
J_HORN_PLATE_Z = (BOSS_H + HORN_HUB_H - HORN_PLATE_T, BOSS_H + HORN_HUB_H)   # 2.7 .. 4.3 (nominal)
J_CABLE = ((CABLE_EXIT[0][0] - SHAFT_X, CABLE_EXIT[0][1] - SHAFT_X), CABLE_EXIT[1],
           (CABLE_EXIT[2][0] - TOWER_TOP, CABLE_EXIT[2][1] - TOWER_TOP))   # ((-5.95,-4.2), (-1.5,1.5), (-25.25,-22.95))
J_CASE_SCREW_X = (CASE_SCREW_X[0] - SHAFT_X, CASE_SCREW_X[1] - SHAFT_X)   # (-4.7, 16.1)
