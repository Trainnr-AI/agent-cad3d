# XGO-Duck → SCS0009: servo-connection review (2026-11-09)

Layout: uniform 0.92× of the original (`data/layout_uniform_0p92.json`), every joint axis,
direction, range and proportion identical except head yaw. Checked with
`tools/blender/compare_kinematics.py`, `sweep_compare.py`, `rotation_range.py`,
`overlap_volume.py`, `torque_review.py`.

## Per joint

Torque = static gravity torque, worst case. "Walk" = home pose standing on one foot;
"worst" = any of stand / sit on one foot / full-range sweep of neck and head. Original servo limit
1.04 N·m (HLS1910 in the sim model). SCS0009 at 6 V: stall 0.226 N·m, rated 0.074 N·m.

| Joint | Range (orig = new) | Servo held by → gets ear pads + Ø10 boss | Turning part → gets 10×15×3 bearing + X-horn pocket | Walk torque: orig % limit → new % stall | Worst case: new % stall |
|---|---|---|---|---|---|
| hip yaw L/R | −12.6…+25.8° | body / hip-yaw support plate | hip-yaw connector | 0% → 0% | 0% |
| hip roll L/R | ±22° | hip-yaw connector + cover plate | L-bracket | 16% → **48%** | 49% |
| hip pitch L/R | ±90° | thigh | L-bracket | 5% → 17% | 25% |
| knee L/R | ±90° | thigh | shin plates | 11% → 31% | 31% |
| ankle L/R | ±90° | shin plates | foot | 0% → 2% | **92%** (sitting, one foot) |
| neck pitch | −135…+60° | neck plates | body | 3% → 9% | 57% (at −86°) |
| head pitch | ±105° | neck plates | head-pitch link | 1% → 3% | 23% |
| head yaw | **±170° → ±150°** (servo travel) | head-yaw part | neck side | 0% | 1% |
| head roll | ±25° | head | head-yaw part | 0% | 2% |
| mouth | — | head | jaw (horn only, as original) | — | — |

All joints: axis and positive direction identical (dot = 1.00000); servo output on the same
side as the original; no new collisions over 184 swept poses; connector free range ±178°.

## Connection (every joint)

- Horn: X horn in a pocket with 16 tapered crush ribs (0.25 mm interference) → no rotational
  play from the coupling. M2×6 machine screw into the shaft (same thread engagement as stock M2×4).
- Rear: Ø10 boss printed in the same part as the ear pads (coaxial by construction), 10×15×3
  bearing in the turning part, 0.5 mm axial gap to the floor.
- Overlap check: servo↔frame and bearing↔seat/boss touch only (0.000 mm³); horn↔pocket 2.58 mm³
  = the designed rib interference.

## Differences from the original that affect smoothness

| | Original HD-1910 (HLS1910) | SCS0009 | Effect |
|---|---|---|---|
| Torque headroom (walk, worst joint) | 16% of limit | 48% of stall (146% of rated) | Main risk: hip roll / knee sag and heat under dynamic walking loads |
| Position resolution | 4096 steps (runtime centres at 2047) | 1024 steps over 300° = 0.293°/step | ~0.4 mm steps at the foot |
| Position sensor | 12-bit (per runtime) | carbon-film potentiometer | more noise and wear |
| Gear backlash | not published | ≤ 0.5° | — |
| Travel | ≥ 340° used (head yaw) | 300° | head yaw ±150° |
| Actuator model | BAM fit `1910_m6.json` | none yet | the RL policy's actuator model must be re-identified |

## Actions

1. Coaxiality of the two-sided support: assemble each joint with the far-end screws loose,
   rotate the joint through its range so the plates self-align on the shaft + boss, then tighten.
   Far-end holes in the plates are clearance (Ø2.3) to allow this. Boss printed at Ø10.0; check
   bearing slides on with a light press before assembly.
2. Torque headroom: before printing, simulate stand / walk with SCS0009 limits (trainnr), and
   identify the SCS0009's dynamics from recorded telemetry (trainnr `identify_system`) so the
   policy is trained on the real actuator. If hip roll / knee saturate: reduce mass (battery,
   infill) or use a stronger servo in the same 9 g case for those joints.
3. Mass estimate (553 g) assumes everything except servos scales with s³; the battery and boards
   don't, so expect ~580 g and ~5% higher torques.

## Head fix (simulation, 2026-11-09)

Problem: with the 0.92x head (135 g) the SCS0009 neck saturates past about -57° forward, and the
duck tips backward with the neck at +60° (soft joints let the body lean back over the heels).
Fix, verified in MuJoCo against the original (`sim/scene_scs0009_h0.85_e1.xml`, trainnr robot
`xgoduck_scs0009_head085`):

1. Head shell scaled 0.85 about the head-roll joint (on top of the 0.92 robot scale): head
   135 g -> 109 g, total 587 g -> 561 g. Neck load 59-83% across -69°…-135°; holds -135°, which
   the original can't (it tips forward past about -115°).
2. Servo position gain: full torque at 1° of error (kp ≈ 13 N·m/rad) instead of 3°. Needed for
   neck-back +60° (the head size alone doesn't fix it). Feetech's PID is configurable; confirm the
   real SCS0009 can be tuned this stiff without oscillating (trainnr system identification).

| Head scale | Head g | neck +60° | neck -135° | neck load at -92° |
|---|---|---|---|---|
| 0.95 | 126 | falls | holds (89%) | 75% |
| 0.90 | 117 | holds | holds (86%) | 71% |
| **0.85** | **109** | **holds** | **holds (83%)** | **69%** |

Fit: at 0.85 the head-roll servo stays inside the head outline (69 x 44.5 x 97 mm) but with only
1.4 mm to the outline on one side (3.8 mm at full size): the head redesign needs a local bulge or
a ~1 mm shift of the internal mount on that side. The 52 mm camera board fits.

## Legs v1 (2026-11-09) — original-style joints

Joint support follows the original (assembly guide steps 05/09/11/12/14): one moving plate per
link and a **16x22x4 bearing around the output**, its outer ring in the holding link's cap plate
(the plate that sits on the servo's front face; servo hangs from it by its ears, screws from
below). The X horn is **trimmed to 6 mm radius** to fit inside the bearing bore, in a crush-rib
pocket inside the moving link's Ø16 boss; M2x6 machine screw into the shaft. The ankle is two-sided
like the original foot: horn plate outboard + 10x15x3 bearing inboard on a Ø10 pin on the shin.

| Part (original) | Built from | g (solid PLA) |
|---|---|---|
| hip_yaw_support (body plate, shared) | caps for both hip-yaw servos | 7.1 |
| yaw_connector (2x_hip_yaw2rol) | hip-yaw moving plate + hip-roll cap | 4.5 |
| l_bracket (2x_hip_rol_output) | hip-roll + hip-pitch moving plates | 6.5 |
| thigh (thigh plate) | hip-pitch cap + knee cap + 9 mm bridge | 4.4 |
| shin (2x_shank) | knee moving plate + ankle floor, ear pads, Ø10 pin | 6.5 |
| foot | ankle horn plate + 10x15x3 bearing plate + sole (original footprint) | 14.9 |

All parts watertight. Motion: 91 poses over all 10 leg joints, both legs present: no collisions
except knee at full 90° (foot touches the hip-yaw connector), which the original also has.
Bearings for the legs: 8x 16x22x4 + 2x 10x15x3. Built by `tools/blender/parts/leg_orig_style.py`
(features in `parts/joint_orig_style.py`); geometry is functional v1 (flat plates), not yet
styled to the original's rounded shapes; horn dimensions still nominal.

## Neck review (2026-11-09) — 0.85x head, stiff gain (`sim/scene_scs0009_h0.85_e1.xml`)

Chain (body → head): neck pitch ID30 → head pitch ID31 → head yaw ID32 → head roll ID33 (+ mouth ID34).
Axes, directions, parent/child and servo output side identical to the original for all four
(`compare_kinematics.py`). Ranges identical except head yaw ±170° → ±150° (SCS0009 300° travel).

How the original supports each joint (assembly guide steps 02, 16–18, 22 + CAD parts list) and
what the SCS0009 version needs:

| Joint | Original | SCS0009 version |
|---|---|---|
| Neck pitch ID30 | two-sided: main + secondary horn into a U on the body; ID30/ID31 cases joined by two `2x_neck` plates screwed into the case holes | U on the body: X-horn pocket one arm, 10x15x3 bearing in the other on a Ø10 pin. Neck = printed cradle holding both servos by their ears (SCS0009 has no case holes); ears end up 2.1 mm apart |
| Head pitch ID31 | same, U = `head_pitch` | same as neck pitch, U = head-pitch bracket |
| Head yaw ID32 | one horn (secondary optional) | trimmed horn + 16x22x4 bearing cap (leg style); bearing also carries the head's weight |
| Head roll ID33 | two 16x22x4 bearings in the yaw bracket | horn one side; Ø10 pin on the head's servo mount + 10x15x3 the other side |
| Mouth ID34 | horn only | horn only |

Physics (MuJoCo, same commands to both ducks; peak servo load % of limit, max lag):

| Motion | Original | SCS0009 |
|---|---|---|
| neck pitch to −135° (1.5 s) | falls forward | holds, 83%, 1.1° |
| neck pitch to +60° | 18% | 50%, 0.5° |
| head pitch ±105° | 12–15% | 33–46%, ≤0.7° |
| head yaw ±150° | 9% | 33%, 0.6° |
| head roll ±25° | 3% | 7%, 0.1° |
| neck −69° + head up 69° | 21% / 11% | 57% / 27% |
| neck −115° + head up 105° | 24% / 12% | 72% / 35% |
| neck back 46° + head down 69° | falls backward | falls backward (same) |
| fast: head pitch 105° in 0.5 s | 22% | 80%, 1.2° |
| fast: head yaw 150° in 0.5 s | 57% | 87%, 1.4° |
| fast: neck −135° in 1.0 s / 0.75 s | falls / falls | holds 1.5° lag / **can't follow (stalls)** |

Clearance (357 poses incl. combined): one new contact, head pitch −96° (head-roll servo vs hip-yaw
servo, 0.6 mm), which is unreachable on both ducks: the original's head shell hits its body at −62°.

Limits vs the original: head yaw ±150°; big fast neck swings need ≥ 1 s (torque-speed limit),
which the original can't do without falling anyway. Leave cable slack through head yaw (±150°).

## Neck v1 + rear-pivot rework (2026-11-09)

SCS0009 has no rear shaft and a flat bottom. Two-sided joints (neck pitch, head pitch, ankle) use:
- a **Ø10 pin printed on the part that holds the servo** (cradle floor / shin floor) under the
  output shaft: coaxial by construction;
- a **separate bearing plate** with a 10x15x3 bearing that slides over the pin from outside and is
  screwed to the moving part (2x M2 csk). A one-piece U/foot can't be assembled: sliding it onto
  the horn would push its far arm through the servo (verified with `assembly_path_check.py`).

Neck parts (`tools/blender/parts/neck_orig_style.py`, collection `neck_v1`): neck_cradle 5.1 g,
body_neck_mount 4.5 g + body_neck_bearing_plate 1.5 g, head_pitch_u 6.4 g (its top plate is the
head-yaw moving plate) + head_pitch_bearing_plate 1.5 g. Foot split into foot 13.3 g +
foot_bearing_plate 1.8 g.

Fixes found by the new one-piece check (`shells`): cap plates were cut in two by the 16x22x4 seat
(caps now have a 2.2 mm ring around the seat), L-bracket plates only touched at the corner (now
overlap through), knee pads stopped 0.01 mm short of the plate, head-pitch U plate faces were
swapped. Yaw connector trimmed flat at its plate top (the new ring rubbed the body plate on yaw).

Verified: all 18 printed parts one solid, watertight; horn pockets 1.0-1.9 mm³ rib press fit;
servos on cradles touching only; every assembly slide clear (ankle, knee, hip pitch, hip roll,
hip yaw, neck pitch, head pitch); motion: all 13 leg+neck joints x 9 + 9 combined poses clear
except knee at full 90° (foot bearing plate vs yaw connector) - the original also collides there.

## Joints v2 (2026-11-10) — exact SCS0009, knee + ankle prototype

Rethought from the servo's facts (Feetech STEP): no case holes, no output bearing (brass Ø4 journal in a
3 mm plastic bore), no rear shaft, Ø12 half-round tower with a Ø5.76 x 0.3 boss, cable slot at the
shaft end, kit X horn 23.88 tip-to-tip with ~Ø1 holes. Hence: the printed part is the link and the
servo a cartridge in a cradle; every joint has its own bearing coaxial by construction (printed in the
same part as the cradle); torque goes through a form-fit horn pocket + the M2 clamp.
Two joint types (`tools/blender/parts/joint_v2.py`, built by `parts/leg_v2.py`):

| | Type C — cap (hip yaw/roll/pitch, knee, head yaw) | Type F — fork (ankle, neck pitch, head pitch, head roll) |
|---|---|---|
| stator | cap plate on the tower top (16x22x4 seat), ear pads (M2 kit screws from below), **saddle walls** round the body (0.15 clearance, 8 deep) | 4 mm floor under the servo bottom with a **captive M3 nut** (hex pocket from the servo side: the servo itself traps it), ear pads, saddle walls, **cable channel at the shaft end** |
| rotor | Ø15.95 boss in the bearing, horn **trimmed to r 6** (printed jig `horn_trim_jig_r6`), ribs r 0.5, M2x6 counterbored clamp | horn plate with the **full 23.88 cross at 45°** (no trimming), ribs; separate bearing plate: **693ZZ on an M3x8 axle** into the nut, 2x M2 csk |
| why | the only compact coaxial layout for a 16-bore bearing (a fork is >=36 wide: the knees would touch) | metal pivot instead of a printed Ø10 pin; Ø8 seat instead of Ø15; nut trap prints without supports |

Layout changes: all servo axes snapped exactly onto the joint axes (they were 0.35° off from the
original servo fit, `snap_servo_axes.py`); **ankle servos slid 6.37 mm along their axis** toward the
horn so the ankle floor lies in the knee plate's plane -> the **shin is one flat 4 mm plate**
(knee boss + ankle cradle on the outboard face, flat inboard face = print bed; the 693's inner ring
bears on a flush Ø4.5 island inside a Ø16 x 0.5 recess).

Verified (left leg, v2 thigh 7.9 g, shin 9.1 g, foot 15.8 g, bearing plate 1.8 g; hips still v1):
one solid watertight piece each; servo/cradle, bearings, nut, axle, plates: 0.00 mm3 overlap; horn in
pocket 2.0 (knee) / 1.2 (ankle) mm3 = rib interference; assembly: knee servo up into the thigh cradle,
shin onto the knee horn, ankle servo down into the shin cradle, foot onto the ankle horn, bearing plate
from inboard — all clear; sweep over hip yaw/roll/pitch, knee, ankle (9 samples each) clean except
knee **+90.0° only** (ankle-cradle wall vs hip-pitch cap ring, 17.5 mm3; clean at +85°; the original
collides at +90° too) -> knee software limit +85°. Printability (`print_check.py`, 50°): foot 67 mm2,
bearing plate 31, shin 211 (small bridges), thigh 373 (= the two 2 mm bearing-lip rings) unsupported.
Horn section still nominal (plate 1.6, hub Ø6.5 x 4.0); the pockets are parametric on it.
Open: roll out to hips, right leg, neck/head; head yaw/roll fit; ear-screw length.

### Rollout to hips + right leg + neck (2026-11-10, in progress)

Built with `leg_v2.py` (both sides, `with_support` on the left) and `neck_v2.py`; all type-F stators
now use a 4 mm floor with the captive M3 nut and the cable channel at the shaft end (v1 had it at the
far end). Boolean lessons that made the parts come out clean: one combined cutter per pocket (ribs
subtracted from the cutter, not unioned into the part), one combined axle cutter (recess + hole +
hex), no face flush with another (saddle walls 0.1 inside the plate edge, pads 0.05 inside the plate
ends, trim annulus 0.01 into the plate top), no two round cutters on the same vertex-angle grid.

State: left leg 7 parts + neck 5 parts + right leg 5 of 6 parts one solid watertight piece.
Masses (g solid PLA): thigh 7.8, shin 9.0, foot 15.8, foot bearing plate 1.8, L-bracket 6.6, yaw
connector 6.1, body plate 8.3; neck cradle 7.4, body mount 5.8 + plate 1.4, head-pitch U 9.2 + plate 2.1.
Horn pockets 1.2-2.1 mm3 interference everywhere; nuts/bearings/axles/plates 0.00. Assembly: all 11
steps (hips + neck) clear except the hip-roll servo (below). Printability (50°): body mount 0, bearing
plates 0-34, L-bracket 143, head U 148, yaw connector 321, body plate 308, neck cradle 398 mm2.

**Open (next session):**
1. `right_shin` NOT MANIFOLD (10.8 g, one healed sliver; the left with identical plan geometry is
   clean) — bisected: not the saddle walls, not the cable channel. Suspect the pad/plate union; try
   building the cradle plate + pads as one hull or sinking the pads deeper.
2. Hip-roll servo ∩ yaw connector 23.9 mm3 on both sides; insertion from below blocked at -13 mm.
   New in v2 (v1 was 0.00): locate with the posed-overlap centroid script (see HANDOFF lessons),
   probably the roll cap's saddle walls or the yaw rotor plate against the exact case.
3. Re-run the full sweep (10 leg joints + neck pitch/head pitch, combined poses) with the final
   meshes; re-check hip_yaw_support / yaw connector print orientation; the head-pitch U's bearing
   block (BLOCK) still assumes the v1 10x15x3 plate thickness offsets - verify against the 693 plate.
4. Regenerate the sim model (ankle servos moved 6.37 mm; new part meshes) and masses.

### Rollout fixes + element review (2026-11-10, macOS session)

- Hip-roll servo ∩ yaw connector 23.9 -> 0.00 mm3: the servo's shaft-end ear reached 2.6 mm into the yaw
  plate's underside; channel cut along the insertion path (plate pre-cut before the unions), strip 17 wide.
  1.1 mm of plate is left above the channel under the boss. Insertion path clear on both sides.
- right_shin manifold: cradle plate + pads + walls unioned first (`combine_cutters`), then joined.
- Combined poses (new, `leg_motion_check.py` `poses`, `json_out`, `full`): both hips yawed out met at the
  roll bearing rings from ~0.3 rad (original: 7 mm3 at 0.45). Inboard flats on the roll cap (0.8 wall at
  the seat) and the L-bracket ring -> clear to 0.35; at 0.45 + roll 0.384: caps 15, brackets 24 mm3.
  Untraced: thigh x hip-yaw servo 8 mm3 (roll out, pitch 1.2, knee 1.48); L-bracket x body plate 0.3 mm3.
- Collection `duck_v2_exploded` (explode_robot.py + per-item offsets); hardware dummies parented to bodies.
  Ear screws and bearing-plate countersunk screws have no dummies yet.

### Standard hardware: 6702ZZ replaces 16x22x4 (2026-11-10)

The original's 16x22x4 is a niche size ("ET2216"); type-C joints now use the ISO thin-section **6702ZZ
(15x21x4)** (`joint_v2.py` BRG_*, BRG_NAME; lip hole and shoulder derive from it). Boss Ø14.95, wall round
the r 6 horn pocket 1.3 mm (was 1.8). All 18 parts rebuilt: one piece, manifold; servo/part pairs 0.00 mm3,
horn fits 1.6-2.1. Sweep (12 joints x 9 + 18 poses): knee ±90° 0.3 mm3 (was 13.6); both hips out 0.45 +
roll: caps 1.8, L-brackets 9.0 mm3 (was 15 / 24); thigh x hip-yaw servo 7.7 (roll out + pitch 1.2 + knee
1.48, untraced). BOM: 6702ZZ x9, 693ZZ x4, M3x8 + M3 nut x4, M2x6 machine x13 (shaft thread unconfirmed),
M2 csk x8, kit ear screws x28.
Retention: two-sided joints are captured on both sides; the nine one-sided joints hang on ONE M2 screw into
the servo shaft (as the original, M2.5) - not positive retention; bearing seats have 0.05 clearance.
`color_code.py`: colours per component kind + labelled legend on `duck_v2_exploded`.

### Fit audit, one screw type, cross-sections (2026-11-10)

`fit_audit.py` (overlap by ray-parity sampling + smallest surface gap, per fit, against the intended fit:
clear / press / thread) over all 13 joints, 210 fits. First run found three real faults, all fixed:
- foot bearing plate's screws missed the sole by 1 mm (flange overhung it): sole now reaches under the
  flange (flange 4.4 beyond the plate, screws at 2.3); foot 17.1 g.
- M3x8 axle tip touched the servo bottom (693 4.0 + floor 4.0 = 8.0): `PLATE_T = FLOOR_T = 4.5`, tip stops
  0.5 short, 1.9 mm in the nut. Type-C rotor plates are 4.5 too (shin stays one flat plate).
- bearing-plate lip 0.2 -> 0.8 mm (`BP_T = 4.8`).
Model bugs found on the way: `add_annulus` transformed the whole bmesh (ring dummies misplaced) - fixed;
nut dummy not turned like its pocket - fixed. Final: 208/210 as designed; the 2 others are the head-yaw
servo in the OLD yaw block shell (no cap designed). Sweep after the changes: knee ±90° 0.3 mm3; both hips
out 0.45 + roll: caps 1.8, L-brackets 14.6, hip plate x L-bracket 1.1 mm3; thigh x hip-yaw servo 7.7 (untraced).

Fasteners: ONE self-tapper, **M2x6 pan-head self-tapping x36** (28 servo ears: 1.6 ear + 4.4 pad; 8 bearing
plates: pan head on the flange / in a Ø3.9 counterbore leaving 2.0 under the head -> 4.0 bite). Horn clamp:
**M2x6 pan-head machine x13** - datasheet "出力轴螺丝 M2.0x4", thread type not stated; the STEP's Ø1.6 shaft hole
is the M2 tap drill. UNCONFIRMED on a real servo. M3x8 socket + M3 nut x4. 6702ZZ x9, 693ZZ x4.

Views (`section_view.py` align, `color_code.py` auto_labels/title, `explode_leg.py`): collections `sec_<joint>`
(8 labelled cross-sections through shaft + servo length, 3x), `leg_exploded` (every element of the left leg),
`duck_v2_exploded`. Bearing dummies now have inner/outer rings (ring walls NOMINAL); ear and plate screws modelled.

### Views redone; yaw block designed (2026-11-10)

- `explode_all.py` replaces the body-offset explode: every element separate along its joint axis (128+ elements,
  both legs + neck + head), labelled by `color_code.py` auto labels. `sec_<joint>`: nine cut-aways in their REAL
  orientation, laid out to mirror the body (`section_view.py` `in_place: {"at": ...}`), cut face toward the viewer.
- **yaw_block** (`parts/head_v2.py`, collection `head_v2`, 6.7 g): type-C stator for the head-yaw servo (cap +
  6702ZZ seat + pads + saddle) and type-C ROTOR for head roll (boss + trimmed-horn pocket); driver channel through
  the yaw saddle walls for the roll clamp screw. Head roll is now a CAP joint (was planned as a fork): ±25 deg,
  40 g head, no plate behind the head. 22/22 fits as designed; assembly paths clear (yaw servo into block, block
  onto roll horn, U onto yaw horn); head yaw / head pitch / head roll / neck pitch x13 + 6 combined poses: 0
  collisions. Open: the head-side holder of the roll servo (cap in the head core) - with the head.
  BOM: 6702ZZ x10 (was 9), trimmed horns x10, M2x6 machine x14.

### Cable exits (2026-11-10) - first check, cables not designed yet

Datasheet: fixed lead, 15 cm, 3 wires (GND, Vcc, signal); connector type not given. The lead leaves the
shaft-end face at the bottom (`J_CABLE`, 3 x 2.3 slot). Check: an 8 mm straight stub out of each slot
(collection `cables`, `cable_stub_<servo>`) against every printed part and servo, 448 pairs. Blocked:
- head-pitch servo -> head_pitch_u (the MOVING U's top plate passes ~2.5 mm in front of the exit; ±105 deg) - real
- hip-roll servos (L/R) -> hip_yaw_support (exit points up at the hip plate; yaw ±26 deg between them) - tight
- hip-yaw servos -> left: body_neck_mount (static); right: neck_cradle + neck-pitch servo (the cradle MOVES) - real
Free but tight (0.6 mm to the cradle wall): neck pitch, head pitch, both ankles (inside their cable channels).
All other exits clear by >= 1 mm. To do with the body/neck rework: route these three; model lead + connector
once the connector is measured; bus topology (one lead per servo -> hub/board with 15 sockets or splitters).

### Cable kit + wiring plan (2026-11-10)

From the kit photo (scaled by the servo and horns, +-10 %, NOMINAL - `parts/cable_kit.py`, collection `cable_kit`):
servo lead flat 3-wire 3 x 1 mm, 150 mm, 3-pin plug ~6 x 8 x 4.5; hub board ~17 x 20 mm with three 3-pin sockets;
extension cable ~165 mm between two plugs; 2 flanged ear screws + 1 horn screw; three horns.
Run lengths (exit -> joint axes -> hub, +15 mm slack per joint crossed, +12 mm for the two ends):
- leg: hip pitch 46, knee 45, ankle 91 -> hubs on the thigh; hip roll 75 and hip yaw 43 -> body;
  thigh -> body 152 (extension, 165). Without a leg hub: knee 181, ankle 231 = too long.
- neck: neck pitch 93, head pitch 125 -> body. Head yaw 193, head roll 230 to the body = too long; head roll to a
  cradle hub 151 = marginal -> hub on the head-pitch U for head yaw + head roll, extension to the body (~150).
Plan: 2 hubs per thigh (A: hip pitch, link, extension up; B: knee, ankle, link), 1 hub on the U, body takes 9 plugs
(2 leg extensions, 2 hip roll, 2 hip yaw, neck pitch, head pitch, head extension). With kit hubs only the body needs
8 more hubs (13 of 15) - a controller/driver board with enough sockets is the better answer. Mounts not designed.

### Horn detail + electronics notes (2026-11-10)

Horns (`parts/scs0009.py`): hub bored for the spline and the screw, screw-head recess, a row of Ø1 holes along each
arm (`HORN_HOLE_R`, from the kit photo, NOMINAL); trimmed horns keep only the holes inside r 4.8. All 14 servos
refreshed; 23 horn fits still as designed (press 1.0-2.1 mm3). Kit horns: double arm, X, disc - only the X is used.

Electronics (from the original's files, vendor/xgoduck_hardware/PCBA V1.1 BOM; nothing tested):
- Arduino Uno Q + the XGO-Duck expansion board (68.55 x 53.38; STEP in vendor/.../PCBA/Step): half-duplex TTL bus
  driver (SN74LVC1G125/126), QMI8658A IMU, **three** 3-pin 2.0 mm servo ports (TE 292253-3), each fused 3 A,
  XH2.54-2P battery input, DC5521 charge jack, switch, TPS566238 6 A buck, AMS1117-3.3.
- SCS0009: 4-7.4 V, over-voltage protection above 9 V, TTL half-duplex. The original 2S Li-ion pack is 8.4 V full:
  above the servo's rated range. Options: 2S LiFePO4 (6.4 V nominal, ~7.3 V full, no converter) or a 6 V buck on
  the servo rail (needs the schematic: is the servo rail battery-direct?).
- Three ports + single-lead servos -> kit hubs as trees: per leg 4 hubs (2 in the body: hip yaw, hip roll; 2 on the
  thigh: hip pitch, knee, ankle), head 3 hubs (neck pitch; head pitch; head yaw + head roll) = 11 hubs, 11 extension
  cables (15 of each in the kits). Mouth servo would need a 12th.
- Firmware: SCS protocol / 1024 steps over 300 deg, not the HD-1910's 4096 over 360 deg.

### Electronics without the XGO expansion board - proposed off-the-shelf set (2026-11-10, nothing bought or tested)

| Role | Pick | Facts found (web, 2026-11-10) | Open |
|---|---|---|---|
| Controller | Arduino Uno Q | 68.85 x 53.34 mm; power: USB-C 5 V/3 A or VIN 7-24 V; Qwiic (3.3 V I2C); UART on JDIGITAL (3.3 V) | height, mass |
| Servo bus driver | Waveshare Bus Servo Adapter (A) | ST/SC series, UART + USB-C, 42 x 33 mm (x 18.5 high per one retailer), Ø2.5 holes on 37 x 28, ~16 g; input 9-12.6 V, "5-8.4 V can also be input", passes the input to the servos | number/type of servo sockets, UART pinout + level, mode switch |
| IMU | Adafruit BNO085 STEMMA QT/Qwiic (4754) | 25.6 x 22.7 x 4.6 mm, 2.5 g, plugs into the Uno Q's Qwiic | runtime wants gyro + gravity: OK |
| Servo supply | Hobbywing UBEC 5 A adjustable (5 / 6 / 7.4 V, 5 A cont., 15 A peak, 2-8S in) set to 6 V | or UBEC 10 A (2-6S) if 5 A proves short | size, mass; measure real current |
| Battery | 3S LiPo ~1000 mAh (11.1 V) | feeds Uno Q VIN (7-24 V) directly and the UBEC with headroom; a 2S pack sags below VIN's 7 V and below the UBEC's dropout | size/mass to choose; needs a low-voltage alarm or cut-off |
| Wiring | kit hubs + extension cables; switch, fuse, XT30 | adapter port count decides the hub tree | |
Rejected: Feetech FE-URT-1 (USB only on the host side, listed as discontinued by one seller).

### Body v2 (first attempt), wiring, views (2026-11-10) - body NOT accepted yet

`parts/body_v2.py` (collections `body_v2`, `electronics`): body_frame 57.8 g solid (deck with both hip-yaw caps =
old hip_yaw_support; neck pedestal + horn plate straight under the neck axis = old body_neck_mount; rear tower with the
Uno Q in side grooves and a bay 34.6 x 54.8 x 69 for battery / servo adapter / regulator; 4 hubs between the hip-yaw
servos; lid posts) + body_lid 7.0 g, 4x M2x6st. `neck_v2.py`: `web_down`, `mount_in_body`.
Found on the way: the OLD body_neck_mount overlapped both hip-yaw servos (59 / 66 mm3; plate 14, its screws 8) - both
are fixed to the trunk, so the motion sweep (pairs on different bodies) never saw it. Fixed by the new pedestal.
Fit audit 94/97: fails = cable stubs (left hip-yaw lead into the neck horn plate 10 mm3; both hip-roll leads into the
deck 8 mm3). Motion sweep (14 joints x 9 + 24 poses): the body is in the legs' way in SINGLE-joint moves - hip yaw
inward (thigh/shin x frame, battery/adapter), hip roll inward (shin x frame), hip roll outward (L-bracket x deck),
hip pitch back 45-90 deg and knee -90 deg (foot bearing plate x frame, Uno Q), neck -135 deg (cradle x frame 13 mm3),
neck +60 deg (head roll servo x Uno Q; U and yaw block x lid, IMU). Cause: full-width lower box + deck, tail +15 mm,
tower to z 128. The old-shell baseline is not usable as is (the shell has the old servo cavities: 5000 mm3 at rest).
TODO: body envelope = what the legs and head leave free; refit the electronics or change them / the scale.

Wiring (`wiring.py`, `parts/cable_kit.py`; collections `wiring`, `wiring_exploded`): 13 hubs (8 in the body, 2 per
thigh, 1 on the U), 14 leads, 13 extension cables, power + signal wires; adapter assumed to have ONE bus socket.
Longest lead: head pitch 147 mm of 150; neck pitch 110; ankle 91; extension body->U 131, body->thigh 94-97 of 165.
Cables are curves through waypoints (exit, joint axes, hub), not collision-checked; hub mounts not designed.
Views: `explode_all.py` now handles body v2 + electronics; `color_code.py` `view`; `section_view.py` `point`;
`body_exploded`, `body_sec_hips`, `body_sec_bay`, `body_sec_long`.

### Body split: motor body + electronics enclosure (2026-11-10)

`swept_space.py`: voxel map (1.5 mm, +1.5 clearance) of what the moving parts sweep (300 poses: every joint alone over its full
range, walking-range combinations, squat pairs, neck/head sweeps with the head turned); free space = reachable from a seed.
Result: behind the hips only +-18 mm about the centre plane (z 57..96), +-22 above the deck (z 99..113 rear / 125 front of
x = -33), nothing above ~z 126 where the head comes down at neck-back. The old shell has the same proportions (+-20 lower part).
The original's legs never touch its trunk in single-joint sweeps; its head does at neck +60 deg (clean at +36) and head pitch -105.
`body_v2.py` now builds THREE parts (user: leave the body for the SCS motors, only the enclosure changes):
- body_frame 20.1 g = motor body only (deck with both hip-yaw caps, centre strip +-9, neck pedestal narrowed at its base, horn
  plate); + 3 pilots in the deck's rear edge.
- electronics_enclosure 27.4 g: keel 43.8 x 32.8 x 35.6 inside (servo adapter, regulator), pocket 44.8 x 40.8 x 13.8..25.8
  (battery 42 x 30 x 13 ASSUMED, IMU, hub 6); tongue under the deck + 3x M2x6 from behind (access holes in the rear wall).
- body_lid 6.6 g: top plate on the enclosure's posts, 4x M2x6; Uno Q on 3.5 mm stand-offs at z 130.6..142.2 (it fits nowhere
  inside: 69 x 53 mm). Accepted by the user: neck back clean at +27.5 deg, cradle x Uno Q at +44 (range +60); head pitch back
  clean to -70, roll servo x Uno Q at -88.
Checks on the unsplit shape before the last fixes: legs clear of the enclosure in every single-joint move; L-bracket x centre
strip from +15 deg roll-out (54 mm3 at 22) -> strip narrowed to +-9, pedestal base narrowed; neck -135: cradle x body 13 mm3
(not located); layout faults fixed (IMU under the Uno's pins -> pocket; regulator 0.2 too long -> keel +1).
NOT re-run since those fixes and the split: fit audit, motion sweep.
Wiring (13 hubs: 6 body, 2 per thigh, 2 on the neck cradle, 1 on the U): all leads <= 92 mm, extensions <= 95 mm.
