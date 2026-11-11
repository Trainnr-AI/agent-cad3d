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

### Cable routes + hub mounts, and the full verification run (2026-11-10)

Design (no extra printed parts for the hubs):
- hubs (13): 5 as a zip-tied stack between the hip-yaw servos, under a tongue of the top plate (x <= 15.5: further forward the
  neck swings past); 2 flat on the battery in the pocket; per thigh one on the hip-pitch servo's bottom and one on the knee
  servo's (foam tape + zip tie); 1 on the back of the neck cradle (short side along the neck, 2 zip-tie holes through the floor);
  1 on the head-yaw servo's bottom. Head chain: body hub (neck lead) - cradle hub (head-pitch lead) - head hub (yaw, roll).
- `joint_v2.cable_channel`: the lead of a type-F servo is folded in a pocket in the shaft-end pad and leaves SIDEWAYS.
- routes (`wiring.py`): one extension across each hip (thigh hub -> over the hip-pitch servo -> behind the roll servo -> gap
  between the yaw connectors -> deck slot); hip-roll lead by the same gap; hip-yaw leads folded inboard above the stack; neck
  lead loops in front of the pedestal; enclosure wires rise in the 4.5 mm gap in front of the battery through a notch in the new
  battery_shelf (3.3 g, loose plate on the pocket's ledges - without it the battery fell into the keel); 3 cables to the stack
  through a notch in the pocket's front wall. Tie holes in each thigh's bridge.
- IMU now stands in the keel between adapter and regulator; battery assumed 40 x 30 x 13; extension cables modelled as round
  bundles (Ø2.7), leads as flat ribbons (orientation not modelled).
- body_frame: no bearing-seat rings beyond the deck plate (L-brackets hit them at roll-out, 54 mm3); pedestal's front top corner
  cut at 45 deg (cradle x pedestal 13 mm3 at neck -135 -> 0); enclosure screws seated on the keel wall's inner face.

Verification (before the last three fixes unless noted):
- cable reach, 211 poses: 0 short; worst ext cradle->head 155/165, body->thigh 141/165, ankle lead 108/150.
- cables/hubs vs parts, 1495 pairs: 15 contacts, all <= 8.8 mm3 (enclosure wires vs walls 3.7-8.8; ankle leads vs shin 2.6; ...);
  hubs clear of everything at rest.
- fit audit: 416/421. Battery x step wall 69 mm3 (shelf raised it) -> wall raised, NOT re-audited; the other 4 are the old
  straight 8 mm cable stubs (superseded by the routes).
- motion sweep 14 joints x 13 + 24 poses: rest clean. Single-joint contacts: knee +-90 (thigh x shin 0.3; foot x thigh hub 9.2);
  neck +44/+60 x Uno Q (accepted, clean at +27.5); head pitch -105: head-roll servo x right hip-yaw servo 404 (the original also
  collides there). After the last fixes (re-swept neck + head pitch only): neck -135: hub C x body bearing plate 1.6 mm3;
  head pitch +-88: hub C x head-pitch plate 3.5, +-105: 35 -> usable head pitch about +-70 deg with the hub there.
- combined poses (feet folded into the hips, legs crossed / splayed in, extreme roll+pitch): still collide, incl. thigh / shin x
  enclosure in "roll out + pitch", "yaw in + squat", "legs crossed" - the original collides in these poses too.

### Review of the full exploded view; electronics fastening, part 1 (2026-11-10)

- Exploded view: hubs now ride on their servos (they were drawn on the thigh, i.e. in front of the pulled-out servo).
  `screw_audit.py`: 61 screws - direction, what the head clamps, what the thread enters, driver access. All clamp/thread as
  designed except the head-roll ear screws (no head core yet). Exploded placement check: 67 screw/part relations, 0 wrong side.
  Assembly order forced by driver access: ankle ear screws before the foot; hip-yaw horn screw before the roll servo; hip-pitch
  horn screw before the L-bracket goes on the roll horn; head-yaw horn screw before the U goes on the cradle; head-roll horn
  screw before the head-yaw servo; neck / head-pitch ear screws before the cradle is mounted; top plate before the Uno Q.
- Printability (50 deg): enclosure 738, body_frame 485, neck_cradle 409, thigh 340, yaw_block 301, yaw_connector 290 mm2
  unsupported; shin 222, U 157, L-bracket 141; foot 67; plates, lid, shelf ~0.
- Fastening done: Uno Q on 4 posts at the UNO hole pattern (ASSUMED for the Uno Q) with 4x M3x8 into the top plate, which is
  now as long as the board (x -52.8..16.0) with a wire notch at the rear; battery_shelf fixed (tab in a rear-wall slot + 2x
  M2x6 into bosses under the ledges). Audit 28/28; neck re-swept: clean to +27.5 deg, 8.6 mm3 at +44.
- NOT fastened yet: servo adapter, regulator, IMU (keel: 18.5 + 4.6 + 8 = 31.1 of 32.8 mm - no room for mounts as arranged),
  battery (adhesive hook-and-loop on the shelf intended, not modelled), hub stack (trapped, not fixed).
- Found while trying to make every cable end exact: a plug is ~6 mm long and needs ~4 mm more for the wire to bend. The 5-hub
  stack between the hip-yaw servos has 1.5 mm beside it and 3.9 mm above; the two hubs in the pocket have 3.4 mm beside them.
  So the body hubs as placed cannot be plugged in. Needs the hub's real socket orientation and the adapter's socket count.
BOM now: M3x8 x8 (4 axles + 4 Uno Q), M2x6 pan self-tapping x45.

### Hub board and plug, read from the photo (2026-11-10)

8x enlargement of the kit photo, scaled by the sockets' pin pitch = 2.5 mm (5264-type 3-pin, as Feetech's SCS connector):
hub = green PCB ~20 x 17 mm with THREE SIDE-ENTRY sockets (each ~12.3 wide x 9 deep x 6 high) opening in three directions -
A and B opposite each other, C at 90 deg; ~24 x 20 x 7 mm overall. Plug ~10 wide x 9.5 long x 5.5 high. With a plug in and the
wires bent, each used socket needs ~7 mm beyond its mouth -> a fully used hub occupies about 31 x 34 x 7 mm.
Waveshare Bus Servo Adapter (A): its documentation lists ONE "Bus Servo Control Interface".
`parts/cable_kit.py` updated (PLUG, HUB, SOCKET, SOCKETS, socket_mouths); `wiring.py` still ends each cable at the hub centre
and still uses the old 17 x 20 x 6 boxes for the body hubs - the 13-hub layout does NOT fit with the real hub size (body: 7
hubs; servo bottoms are 23 x 12 mm). Decision pending: replace the body hubs by one small bus strip, or fewer / other hubs.

### Body hubs do not fit at their real size (2026-11-10) - DECISION OPEN

- A hub with all three sockets used occupies about 31 x 34 x 9 mm (9.5 cm3): plugs enter from three sides, each needs ~7 mm.
- The bus needs 13 hubs whatever the tree (14 servos + 1 adapter socket, 3-socket hubs -> 15 - 2). 7 of them are in the body
  (2 hip yaw, 2 hip roll, neck, 2 junctions) = ~66 cm3 + cable slack; the enclosure has ~46 cm3 free, in fragments.
- Seats that do exist in the body: 1 flat above the battery; 2 flat under the Uno Q between its posts (needs the Uno 6.5 mm
  higher -> less neck-back range than the accepted 27 deg). Between the hip-yaw servos: gap 20 wide, a hub needs 24 / 31.
- Growing the body is not free: swept-space map redone to x -90 (voxel 2 mm, 300 poses, scratchpad swept2.npz): behind the
  enclosure (x -58..-70) the free half-width is only 8-18 mm at z 98-122 (legs swinging back with yaw); the keel's rear-bottom
  corner (x -52, z 56-68: 8-12 mm free vs 18.4 half-width) is already inside the swept space of the combined walking poses.
- Waveshare adapter: second search again finds ONE bus servo socket.
- Options: (1) one small multi-socket bus board in the body instead of the 7 hubs (kit hubs stay on thighs / cradle / head = 6);
  (2) kit hubs only, body 20+ mm longer, leg range limited; (3) kit hubs only, 3 per thigh + under the hips, exposed.

### Wiring v3: bus board in the body, six kit hubs on the limbs, every cable in a named socket (2026-11-10)

Decision (user): one bus board in the body instead of the 7 body hubs.
- **Bus board** (not in the kit, to be made): 12 x 5-hole 2.54 mm perfboard (30.5 x 12.7), five top-entry 3-pin headers in
  parallel (the mate of the kit plug - type still to be confirmed against a real plug, 2.5 mm pitch assumed), four kit
  extension cables with one plug cut off soldered to it (to left thigh, right thigh, neck cradle: under the middle, leaving
  down the deck slot / forward; to the servo adapter: from above at the rear end). Seat: between the two hip-yaw servos, on
  three 3.5 mm posts + one boss of body_frame, 1x M2x6 self-tapper (`bus_screw_M2x6st`). Why not all plugs: 9 headers do not
  fit a perfboard in the 20 mm gap; why top-entry here: the plugs come in from above, where the leads arrive.
- **Kit hubs: 6** (real size, sockets A/B opposite + C): LTA/RTA on the hip-pitch servo's bottom, LTB/RTB on the knee servo's,
  C on the back of the cradle floor (moved 7 mm down: 0.43 of the neck length), Y on the head-yaw servo's bottom.
  Seat = 1.5 mm foam tape + zip tie (PCB centre 2.3 above the face).
- `wiring.py`: each cable end is a plug in ONE socket (`wire_plug_<cable>_a/b`, 6 mm in, 3.5 mm out, then a 7 mm bend point);
  the hub's turn on its seat (90 deg steps) and the socket per cable are chosen for the shortest runs. Result:
  BUS 1 = L hip roll, 2 = neck pitch, 3 = L hip yaw, 4 = R hip yaw, 5 = R hip roll; LTA A = hip pitch lead, B = to LTB, C = from body;
  LTB A = ankle, B = from LTA, C = knee; RTA A = to RTB, B = hip pitch, C = from body; RTB A = from RTA, B = ankle, C = knee;
  C A = head pitch lead, B = from body, C = to head; Y A = from cradle, B = head yaw, C = head roll.
- Kit parts used: 14 leads, 6 hubs, 8 extension cables (2 thigh A-B, 2 body-thigh, body-cradle, cradle-head, adapter-bus; 4 of
  them soldered at the board end). Spare: 8 hubs, 6 cables.
- Measured at rest (3,135 pairs, 0.3 mm sampling): hubs, sockets, plugs and the bus board against every printed part, servo
  and electronics box: 0 contacts. Flexible cables: 13 grazes, 0.6 - 7.8 mm3 (ribbon model has no controlled twist).
- Cable reach over 211 poses: 0 short. Slack to stow (cable length - run): hip-yaw leads ~120 mm, hip-pitch / knee / head-yaw
  leads ~130 mm, thigh A-B ~135 mm: fold flat and tie under the zip tie that holds the hub. NOT modelled.
- Not verified: plug / header type and sizes (photo estimate +-10 %), mated plug height on the bus board (13 mm assumed, 4.6 mm
  spare under the top plate).

### Electronics fastening, part 2 (2026-11-10)

- Balance sensor: flat on four 3.4 mm bosses on the keel floor (x -29.5, centre), 4x M2x6 self-tappers from above
  (`imu_screws_M2x6st`) - low, central, rigid; goes in first.
- Servo adapter + regulator: stand on two rails across the keel's ends (8.6 mm, above the sensor), a fence post on each rail
  between them; the adapter is captive between rails, wall, posts and the shelf (0.7 mm above it). Regulator: lateral only
  (wall / posts) - add a strip of adhesive foam tape to the wall. Adapter's connector positions unknown: not checked.
- Battery shelf lifted 1.6 mm (ledges thicker); battery seat = two side ribs + two front stops on the shelf, rear wall behind,
  top plate 9.4 mm above: it cannot leave the pocket; a hook-and-loop patch underneath holds it down.
- Bus board: see above. Uno Q: 4x M3x8 on the top plate's posts (earlier).
- Masses: body_frame 19.5 g, enclosure 32.2 g (rails are solid: could be hollowed), top plate 9.2 g, shelf 4.0 g.

### Cradle hub seat at real size (2026-11-10)

Full sweep with all 53 wiring solids on their bodies: thigh hubs, head hub and bus board clear in every single-joint move; the
real-size hub on the BACK of the cradle floor hit the body's bearing plate (neck -119..-135) and the head-pitch plate (head
pitch +-88). Search (scratchpad seat_search.py: hub + three 7 mm plug rooms, 3 faces x 13 heights x 4 turns, BVH overlap at
1.5 deg steps against everything on the trunk and on the head-pitch U): no seat clears the full range.
Chosen: rear side wall, 0.30 of the neck length up, C socket toward the head -> neck -135..+19.5 deg (was +27.5, limited by the
Uno Q), head pitch -76.5..+105 deg. Sockets: A = from the bus board, B = to the head hub, C = head-pitch lead.
Neck-to-head cable re-routed: worst reach 164 of 165 mm (neck +60, head pitch 69, yaw 149 deg - outside the new neck limit).

### Support-free printing, body parts (2026-11-10)

print_check (faces within 50 deg of straight down, first layer excluded), best orientation = as assembled, standing on the bed:
- electronics_enclosure 754 -> ~100 mm2: 55 deg undersides (SLOPE 1.45) under the pocket ledges, the four top-plate posts and
  the tongue; step wall and shelf bosses removed. Left: shelf-tab slot (10.6 mm bridge), tops of round holes, 0.3 mm strips.
- body_frame 485 -> 413 mm2: 55 deg gusset under the pedestal's wide top. Left: the two 6702 seat lips (2 mm annular ledge,
  196 mm2) + 91 mm2 above them - shared cap design (joint_v2.cap_features), same on all ten type-C joints; NOT changed.
- body_lid ~5, battery_shelf 0. Thigh, yaw connector, neck cradle, yaw block: not reworked.
Sweep after the change (53 wiring solids included): no new contact at rest or in any single-joint move; four pairs in already
colliding extreme combined poses got deeper (enclosure ledges, e.g. yaw in + squat right shin 535 -> 644 mm3).
Stale check fixed: top-plate screws thread into the ENCLOSURE's posts (19.7 mm3), not the motor body.

### Retention, step 3: bayonet built and tested on the two knees only (2026-11-10)

`joint_v2.py`: `bayonet_hook(M, phi)` (stator: one L-section hook, 24 deg wide, on the cap plate at r 10.9..12.6, lip z 7.7..9.2 =
below the rotor plate's top, no added height) and `bayonet_rim_cuts(M, phi, lo, hi, assemble, sign)` (rotor: disc turned down to
r 9.0 above a 2 mm flange where the hook passes + one notch in the flange). `leg_v2.py`: knee, hook opposite the shin's strip,
assembly angle -116 deg (left) / +116 deg (right) - probed clear of thigh, servos, L-bracket and hubs; switch off with
`"bayonet": false`.
Test (scratchpad bayonet_test.py, BVH overlap, 121 angles over -90..+90): shin held against pull-off at every angle; 0.3 mm
axial play before the hook engages; no rubbing (only the old 0.3 mm3 thigh x shin touch at the range end); slides on freely at
the assembly angle (0..8 mm path clear) and comes off only within 114..117 deg. Leg fit audits: no fit changed (6 "missing"
rows per leg are stale checks against the removed hip_yaw_support).
COST: the hook stands on the thigh's flat print face -> thigh unsupported area 340 -> 982 mm2 (shin 222 -> 275). Conflicts
with the support-free goal. Alternative not built: a separate screw-on keeper clip (1 small print + 1x M2x6 per joint): all
parts keep printing flat, no assembly angle, also possible on head yaw (+-150 deg leaves no assembly angle for a bayonet).
NOT done: hips (6 joints), head yaw, head roll. Assumes the servo can be turned by hand to 116 deg (SCS0009 travel unverified).

### Retention with a screw-on keeper clip, knees (2026-11-10) - replaces the built-in hook as default

`joint_v2.py`: `keeper_features` / `keeper_seat_cuts` / `keeper_screw`; `leg_v2.py` arg `"retention": "clip" | "bayonet" | "none"`.
Clip = the same 24 deg hook as a 0.27 g print with a foot: 1x M2x6 self-tapper (4.4 mm in the cap plate) + a rib in a slot
through the cap plate (shear, anti-turn). Fitted after the shin is on: no assembly angle. Shin: stepped flange, no notch.
Test (keeper_test.py, 121 angles -90..+90, both knees): held against pull-off at every angle, play 0.3 / 0.35 mm, clip never
touches the shin. Fits 18/18 (clip on thigh gap 0.07, screw thread 4.4 mm3, clear of shin, servo, bearing, ear screws).
Print: thigh 342 mm2 (was 340 before any retention, 982 with the built-in hook), shin 275, clip 10.
Display for the user: collection `keeper_demo` (assembled / exploded / half section) at x +0.62..0.85 beside the robot.
NOT done: hips, head yaw, head roll; motion sweep with the clip (started).

### Retention dropped; bearing-seat lip made printable (2026-11-10)

- User decision: NO keeper clip / hook (more parts and complexity). `leg_v2.py` default `"retention": "none"`; thigh and shin
  are back to their pre-step-3 shape (340 / 222 mm2). Clip and hook code stay in `joint_v2.py`, unused. Build note: thread-
  locker on the 14 horn screws + a pull test on the first printed knee.
- Shared type-C cap (`cap_features`): lip hole is now a SQUARE opening (half-side LIP_SQ 8.6) clipped to the seat circle,
  cut only 0.2 mm past the plate's servo face (was a round 17 mm hole cut 1 mm into the pads). With the seat open to the bed
  the lip prints as four 12 mm wall-to-wall bridges instead of a 2 mm ring in the air. Outer ring carried over 4 arcs.
- Rebuilt: both legs, body, yaw block. Bearing fits 36/36, horn fits 23/23 (three stale check targets pointed at removed
  parts hip_yaw_support / body_neck_mount: now body_frame).
- print_check (counts bridges and one-layer steps as unsupported): body_frame 413 -> 354, thigh 340 -> 264, yaw connector
  290 -> 254, yaw block 301 -> 264. What is left per seat: ~65 mm2 lip bridges + ~65 mm2 where pads / saddle walls start over
  the opening (walls bridge it; the pad edge is a 2.4 mm one-layer cantilever, as before).
- Not external: no outside surface changed, so the motion sweep was not repeated.

### Foot rebuilt on the original pad shape (2026-11-11)

User: "the foot should be same as the original in shape not size ... every aspect ratio of the original duck must be there".
Was: a flat, level 5 mm slab, 49.7 x 46.4 mm. Original (scaled 0.92): rounded pad 49.68 x 37.11, 12.2-15.5 mm thick, top 6.53 mm
under the ankle axis, SOLE CANTED 5 deg about the fore-aft axis (normal (0, 0.087, -0.996) on the left foot: inner edge lower),
side walls square to the sole, servo trough r 13.3.

- `leg_v2.py` foot = the original mesh itself (welded, ears cut off 0.05 under the pad top, closed) + our ears. The original
  mesh has ~80 open edges, all in its ears, so the pad is checked closed after the cut.
- Pad cut only by the swept envelope of shin + ankle servo + cable stub over ankle +-90 deg (computed from the built shin: points
  every 0.4 mm over every face, 1 deg steps, 0.5 mm clearance, 8 axial slices): trough r 14.53 (original 13.3), ~0.9 cm3 removed.
  First version sampled vertices and edges only -> missed flat walls between slice ends -> 80 mm3 shin x foot at +-90; fixed.
- Horn ear (outboard) one piece with the pad through a lug with a 55 deg underside. It stands 3.8 mm OUTSIDE the original
  outline (servo + two plates = 40.95 mm > 37.11 mm pad); it starts 5.5 mm above the ground, so the footprint is the original's.
- Bearing plate (inboard) now sits in a slot in the pad's side (x +-9.15, floor 3.6 above the sole), flush with the original
  outline; 2x M2x6 pan-head self-tappers along the axle direction, heads in 2.3 mm counterbores (2.5 under the head, 3.5 in the pad).
  All three inboard screws (axle + 2) go in from the same side.
- Verified, both feet: one closed solid; sole normal, sole area (1543.5 mm2) and footprint below z 6 identical to the original;
  rest fits 26 pairs clean, plate screws thread 7.5 mm3, horn crush ribs 1.4 mm3; print (sole down) 101 mm2, plate 15 mm2.
- Mass 27.1 g solid PLA per foot (slab: 17.1) + plate 1.9. Sim masses not regenerated.
- Range: shin x foot clear over ankle +-90. Thigh x foot toe corner beyond 86 deg in one direction (left -, right +) at knee 0:
  2.0 mm3 at 87, 21.9 at 90. Not relieved (would bite the toe's front edge). The original's foot clears its thigh there.
  Knee: foot x L-bracket from 70 deg (0.7 mm3; 48.5 at 75, 767 at 90), left +, right -. The ORIGINAL's own foot meets its own
  hip-roll part from the same angle (triangle overlap: 0 at 60, 51 at 70), so this is the original's behaviour, not a loss.
- `explode_all.py`: ankle inboard parts spread 3 / 9 / 17 / 20 mm (they crossed the mid-plane before and swapped sides).

### Thigh: original outline, hubs re-packed on the outer side, optional cable cover (2026-11-11)

User: hubs untidy on the thigh; wants a flush look; "the thigh plate should resemble the original design as much as possible";
"the cover should be an optional add on but not necessary"; "all the faces rear, top, outer should be covered".
- Original thigh (scaled) = rounded-triangle housing, x -28.7..27.6, z 50.3..94.0, walls ~6.35 mm outside the servo faces.
- Tried first and dropped: hubs on the hip-pitch servo's TOP face and the knee servo's REAR face under an L-shaped box cover
  (wiring.py `"thigh_hubs": "faces"` still builds it). A plugged hub stands 8.9 mm off its face -> 3.4 mm outside the original
  outline top and rear, 7 mm overhang outboard, 0.5 mm from the shin's ankle end at full knee bend, box cover 8.9 g.
- Packing search (scratch pack.py / pack3.py: 1 mm grid, 4 turns, PCB + socket + 7 mm plug rooms as rectangles, >= 50 % of the PCB
  on a servo end face): two plugged hubs do NOT fit inside the original outline. One arrangement fits within 2.0 mm (left) /
  2.5 mm (right) of it; that one is now the layout (wiring.py `plane_hub`; offsets differ per side, the hub is not mirror-symmetric).
  Sockets: hub A (on the hip-pitch servo's end) = hip-pitch lead (front), cable from the body (rear), link (down); hub B (on the
  knee servo's end) = link (front), ankle lead (rear), knee lead (top). Link cable now 13-18 mm of path (165 mm cable).
- `leg_v2.py` thigh: + 2 mm web with the original outline (convex hull of the original mesh's side view) on the bed side of the plate;
  + two posts (Ø5 rear top corner, Ø4.2 on the long front edge) out to the hub plane, pilot holes in their tips. Our two bearing
  rings still stand 0.3-1.75 mm outside the outline (6702 seat + 2.2 wall). Thigh 7.7 -> 10.3 g.
- Optional `thigh_cover` (arg `"thigh_cover"`, default on): 1 mm plate = original outline grown 4.1 mm, 0.8 mm walls down to the thigh
  plate along the top and rear, 2 mm lip along the long front edge and round the knee (the shin's ankle end passes 16.65 mm from
  the knee axis); 2x M2x6 pan-head self-tappers into the posts (5 mm of thread). 7.1 g. Thigh with cover 40.5 mm thick (original 30.8).
- Rest: cover, hubs, sockets, plugs vs thigh, shin, foot, L-bracket, yaw connector, servos, body: 0 contacts (only the screws'
  thread, 11.0 mm3). Print: thigh 266 mm2 (was 264), cover 0 (standing on its plate). Cable lengths at rest all inside the kit lengths.
- Motion (full sweep, 14 joints x 13 + 24 poses, cover fitted): rest clean. First build of the web cost knee range: OUR shin carries
  the ankle servo's cradle in the thigh plate's plane (the original's shin is a plate that stays inboard), so from ~68 deg of forward
  knee bend the cradle ran into the web (15.5 mm3 at 75, 88 at 90; squat poses with knee 80 deg collided). Fixed with a relief in the
  web's lower front (swept cradle, 50..90 deg): the plate's long edge now has a bite of about 18 x 11 mm; the original outline is
  complete only on the optional cover. A first relief used the 3-D distance between knee and ankle axes (43.4 instead of 38.6) and
  was 4.8 mm short; fixed. Cover lip shortened to 1.5 mm short of the servos' end faces (the foot's horn ear met it from 72 deg).
- Sweep vs the one before the thigh work: single joints unchanged except knee 90 deg in the forward direction: foot x thigh 4.2 ->
  55 mm3 (foot meets the front cover post; that pose is already blocked by foot x L-bracket from 70 deg, as on the original).
  Squat A-D unchanged. New in already-colliding extreme poses: thigh x foot 15 mm3 (knee 85 deg poses), cover / thigh x top plate
  and Uno Q in "yaw in + squat".
- NOT verified: hub and plug sizes (photo estimate, +-10 %) - the packing margin is 2-2.5 mm, so a hub 10 % bigger does not fit
  under the cover; spare cable (120-150 mm per cable) not modelled, intended in the empty triangle between the two servos; cover
  never test-fitted or printed. Cable reach re-run with the new thigh routes: 0 short over 211 poses.

### Kit hubs only: custom bus board removed (2026-11-11)

User: "avoid any custom perf board and use off the shelf parts"; no hubs in the electronics bay; hubs may sit on motors, with optional covers.
Facts: SCS0009 lead = Molex 5264-3P (datasheet). Waveshare Bus Servo Adapter (A) has TWO servo sockets (product photo; earlier text
searches said one - wrong), 42 x 33 mm, available in India (Robu, Evelta, Amazon.in). 14 servos on 2 sockets need 12 three-socket hubs;
the 14 servo kits bring 14. Only ready-made many-socket hub found: Juxi 8-channel (Amazon.com only) - not used.
- Seat searches (scratch hub_seats.py fixed body, hub_seats_rel2.py per moving body; swept map swept3.npz: 300 poses, 1.5 mm voxels,
  1 mm clearance; plugged hub = PCB + 3 sockets + 3 x 7 mm plug rooms): fixed body 0 seats except a chest pocket in front of the neck
  pedestal (holds ONE hub: no pair of positions is disjoint); thigh outer side has a free second layer; thigh top rear corner free;
  shin (foot motor) 0, hip-yaw link 0, L-bracket 0, neck cradle 0. The gap between the hip-yaw servos is below the search's resolution;
  placed by hand and checked exactly.
- Layout built (wiring.py; body_v2.py; leg_v2.py): each thigh 4 hubs on the outer side in two layers (A, B as before; C over A, D over B,
  taped on the lower hub's sockets, PCB 7.5 mm further out); chest hub N on a new chest plate (22.5 wide, 1.6 thick, standing 11 mm up from the deck underside on the front of the centre strip); hub G standing in the
  gap between the hip-yaw servos, socket C down in the deck's cable slot. Trees: adapter socket 1 -> out of a new port in the keel's
  front wall (y +8.6..+16, z 86.5..93) -> left thigh C; socket 2 -> up the riser and the top notch -> G -> (down the slot) right
  thigh C and (forward, beside the pedestal) N -> cradle hub -> head hub. Per thigh: C = cable from the body + hip-yaw lead + link,
  D = link + hip-roll lead + link, A = link + hip-pitch lead + link, B = link + knee + ankle. 12 hubs, 12 extension cables, 14 leads.
  Bus posts / boss / screw removed from the deck (body_frame 20.2 g with the chest plate; prints as before, 354 mm2). Thigh cover deepened for two layers
  (thigh with cover ~48 mm thick, cover 8.1 g).
- First tries dropped: both legs from a chest junction under the deck (161 of 165 mm); two hubs on the chest (they overlap).
- Verified: rest 0 contacts between bodies; stacked hubs / cover / thigh and G, N vs body, servos, lid: 0 (boolean volumes against the
  multi-shell screw and Uno dummies gave false 664 / 522 mm3 = the whole socket / plug; triangle test: none touching). Cable reach
  0 short over 211 poses (longest at rest: left body-to-thigh 118 of 165, hip-yaw leads 94-95 of 150). Full sweep vs the previous
  one: leg single joints unchanged; neck back beyond the existing +19.5 deg limit now also meets hub G (0.5 mm3 at +44, 12 at +60);
  extreme pose "neck fwd head up" (-135 / +105 deg): head-pitch U and yaw block meet chest hub N (new, up to 296 mm3).
- NOT verified / open: hub and plug sizes (photo estimate); every short hop uses a 165 mm kit cable -> ~150 mm of slack per hop
  (9 hops), not modelled; second-layer hubs are held only by tape on the lower hub's sockets (or by the optional cover); adapter still
  modelled standing as a 42 x 18.5 x 33 block, really flat with top-entry sockets; regulator current for 14 servos; chest cover not
  designed; right-leg seats assumed mirror of the searched left leg (the sweep covers both legs).

### Thigh hubs: no stacking - two on the outer side, one on top, one on the rear (2026-11-11)

User: "stacking two on the side does not make sense"; hubs on the thigh's top and rear; fit into existing volume.
- The coarse seat searches (1.5 mm voxels, 1 mm clearance) had ruled the top and rear faces out. EXACT tests (hub solid vs the real
  meshes, scratch top_hub_test.py / rear_hub_test.py: hip pitch 13 steps x roll 3 x yaw 3, knee 13 x ankle 3, squat pairs = 172
  poses; knee -90..90 in 5 deg steps) show both clear. Lesson: the voxel search is a filter, not a verdict.
- wiring.py: hub C on the thigh's TOP, toward the rear (29.5 mm behind the hip-pitch shaft, 1.2 mm toward the outer side) on a new
  printed shelf (leg_v2.py `top_shelf`: 16 x 28.6 x 1.6, level with the hip-pitch servo's top face, over the knee servo's upper end);
  hub D on the knee servo's REAR face, 13.5 above the knee axis, seat 2.2 (clears the holder's side wall and hub B's board).
  Further forward on top, or more toward the outer side, the top hub fouls hub A's plugs. Chain per thigh: C (hip-yaw + hip-roll
  leads) - D (cable from the body) - A (hip-pitch lead) - B (knee + ankle leads). Second layer on the outer side removed.
- Thigh 9.5 g with shelf, without cover posts. The OPTIONAL COVER IS OFF: hub D's upper plug stands where the rear post was; the
  cover has to be reshaped for this layout (`thigh_posts` / `thigh_cover` args).
- Verified: thigh hubs vs each other, thigh, servos: 0 overlaps; rest 0 contacts; cable reach 0 short over 211 poses (hip-yaw leads
  98-99 of 150, body-to-thigh 103-105 of 165); full sweep: single joints identical to the stacked layout (and knee 90 deg foot x thigh
  back to 4.2 mm3 without the post); three new sub-3 mm3 entries in already-colliding extreme poses.
- Size against the original (scaled): top hub 1.9 mm above the original thigh's top, rear hub 2.5 mm behind its rear, the two
  outer-side hubs 9.5 mm beyond its outer face (as since the first hub layout). Chest hub N: 2.1 mm in front of our own neck mount,
  8.5 mm in front of the original body shell's foremost point. Hub G: inside. No ready seat found inside the body for N.
