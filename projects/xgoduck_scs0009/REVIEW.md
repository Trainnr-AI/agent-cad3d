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

### Servo adapter drawn as the real board (2026-11-11)

Was a 42 x 18.5 x 33 block standing between the keel wall and two fence posts. Product photo: a 42 x 33 x 1.6 board with parts on one
side. `body_v2.py`: the board stands on the rails on a long edge, 2.5 mm off the keel's side wall, parts toward the keel's middle, its
two short ends in guide slots on the keel's end walls (1.5 mm of board in each slot, 0.15 clearance per face); the battery shelf above
keeps it down; no screws. Parts as boxes (ESTIMATED from the photo): two servo sockets top front, UART header top rear, DC jack bottom
front, screw terminal bottom middle, USB-C bottom rear. Deepest part (jack) reaches 12.6 mm from the board's back; 9.3 mm are left to
the regulator. `wiring.py`: both servo cables start 9 mm out of the sockets; 6 V wire to the screw terminal; UART wires to the header.
Checked: adapter touches the enclosure only where its lower edge rests on the rails; clear of regulator, sensor, shelf, battery and
screws; enclosure prints as before (119 mm2), 34.8 g; rest 0 contacts; cable reach 0 short over 211 poses (left body-to-thigh 98 of
165, adapter-to-gap 54). NOT verified: part positions and heights on the board, plug height on the sockets (13 mm assumed), the UART
side's voltage (the Uno Q's pins are 3.3 V), whether the board's standoffs must come off (the guide slots need bare board ends).

### Chest hub moved to the front of the neck (2026-11-11)

User: wanted the chest hub on the neck; "fully forward is not needed".
- Exact tests (hub solid with its three plug rooms vs the real meshes; scratch neck_hub_test.py, neck_front_map2.py): 240 positions on
  the cradle's two side walls; none is clear over the whole allowed neck x head-pitch range. Back wall: 10 fit at rest beside hub C,
  best collides in 88 of 148 poses. Front wall, mid-length, plug toward the head: neck to about -115, head up only to about +45.
  Front wall, mid-length, plug toward the BODY: neck to about -80, head up to about +90 (coarse grid) -> chosen.
- The gap between the two neck servo bodies (11.2 mm) is filled by their ears and the cradle's ear pads: no recess possible.
- `wiring.py`: hub N on the cradle's front side wall at 0.50 of the neck length, mid-depth, socket C toward the neck's pivot;
  cables: G -> N (beside the pedestal, up the front, 54 mm), neck-pitch lead -> N (60), N -> C round the cradle's floor (86).
  `body_v2.py`: chest plate off (`chest_plate` arg); body_frame 19.7 g.
- MEASURED limits with hub N (1 deg steps, hub + its plugs vs body, head parts, hub G): neck forward clear to -91 deg (body at -92;
  was -135); head pitch up clear to +85 deg at every neck angle (head-pitch U at +86; was +105); neck back, head down and head yaw
  +-90 deg: no contact from this hub. Existing limits unchanged: neck back +19.5, head pitch down -76.5. The joint_range properties
  in the scene still hold the original declared ranges.
- Verified: N vs cradle, servos, hub C and its plugs: none touching; rest 0 contacts; cable reach 0 short over 211 poses after
  restoring hub C's socket order (with N-to-C arriving on the head-side socket the cradle-to-head cable was 167 of 165 at one extreme
  head pose). Full sweep (run with the swapped socket order): contacts of hub N only beyond the measured limits (neck -103 and
  further, head pitch +88 and +105); legs unchanged.
- Process note: a failed scan left the rig posed (neck +20, head +105) in the live scene and corrupted the next scan, which took
  that pose as rest. Restored by the inverse rotations and checked against the cradle's rest bounding box; the saved file was never
  affected. Scan scripts now restore the pose in a `finally` block.

### Full fit, screw and assembly audit (2026-11-11)

User: "look each and every servo motor, its horn design, size, all the joints, spaces, clearances and make sure everything fits".
- fit_audit.py on the whole robot (list from audit_all_args.json refreshed against the scene: 325 fits; the 98 dropped were the
  old body hubs): 321 as designed. Per fit type (min..max over all joints):
  horn in its crush-rib pocket 1.04..2.16 mm3 press (14 of 14); horn screw in the shaft 2.46..3.00 mm3 (13: head roll has no holder
  yet); ear screws in the pads 9.2..9.6 mm3 per pair (13); 6702ZZ in seat and on boss: touching, no overlap (9 + 10); 693ZZ on the
  island and in the plate: 0..0.02 gap (4 + 4); M3 axle tip 0.44 short of the servo bottom; M3 nut 0.12 in its pocket; 6702 to the
  servo tower 0.84; ear-screw heads to the moving part >= 1.84; servo body to the moving part >= 0.20 (the two 0.20 mm gaps: head-pitch
  U to the head-yaw servo, head-roll servo body to the yaw block); shin to thigh web 0.42; foot trough to shin 0.81, to ankle servo 0.66;
  adapter to sensor 0.60, to shelf 0.68, to battery 2.48.
  The 4 not as designed (unchanged since the previous audit): the straight 8 mm cable stubs of left hip yaw (11.0 mm3), both hip roll
  (8.3) vs the motor body and head pitch (11.6) vs the U - those leads must bend within 8 mm of the servo.
- screw_audit.py: 71 screws, 61 ok. To check: 4 M3 axles (thread into steel nuts, which the tool does not treat as a part - fine),
  2 head-roll ear screws (thread into nothing: the head's servo holder is not designed), 4 Uno Q screws (tool does not see the 2.5 mm
  thread-forming hole as a thread; driver meets the Uno's modelled top block at 7 mm).
- assembly_path_check.py, 30 steps (every servo into its holder, every moving part onto horn / bearing, every bearing plate): 27
  passed at first. FOUND AND FIXED: both feet could not slide onto the horn - the original-shape pad stopped the horn's arms 4-6 mm
  before home (14-22 mm3). `leg_v2.py` horn_room now runs 7.5 mm inboard: path clear to 1 mm, then the crush fit (1.7 mm3); foot 26.9 g,
  one solid, prints as before (101 mm2), fits unchanged. FOUND, NOT A PART CHANGE: the LEFT hip-yaw servo cannot drop straight in - the
  neck's horn plate overhangs its front ear (12.4 mm3 from 8 mm above home upward; trimming the plate would open the neck horn's
  pocket). It goes in with a slide: 4 mm toward the rear, down to 5 mm above home, forward 4, down (0 contact along the path). That
  needs the space behind the servo free -> hip-yaw servos go in BEFORE the electronics enclosure is attached. The right one drops in.
- Horn caveat: hub Ø6.5 x 4, plate 1.6, arm widths 5.6 -> 3.6 are NOMINAL (listing + photo); every horn fit above is against that
  assumed horn. Servo body: exact (Feetech STEP).

### Head: study of the original (2026-11-11) - no part built yet

Sources: vendor/xgoduck_hardware (STL set V5 "audio/video", Assembly_Guide.pdf p.16-21, BOM, readme), press coverage for Microduck.
- XGO-Duck head = 15th servo for the beak (ID 34, outside the walking policy) + printed parts: head.stl (yellow base tray = upper
  bill, 88.3 x 118.9 x 16), head_shell.stl (white dome with grille slots, 88 x 124.1 x 48.1), jaw.stl (U-frame lower bill: front bar
  + two side arms up to the pivots, 88 x 67.7 x 30.1), head_servo_support.stl (clamps the servos to the base, 4 screws), eye.stl,
  FACE / FACE_CAMERA / FACE_LIGHT / FACE_TOF (front panel of the audio/video version), two TPU bill pads (double-sided tape).
  Guide p.19: servo 34 sits beside servos 32 / 33 on the base; the jaw arm screws to its horn. p.20: support over the servos, base
  has an opening underneath for the yaw servo's bracket. Microduck: same joints + beak, camera, ToF, mics, speaker, head IMU, NFC
  (hardware closed).
- Scene: collection `orig_head_parts` (hidden by default) = original STLs at our head scale (0.782 = 0.92 x 0.85), registered to the
  merged head mesh by a 24-orientation search + refinement: head base mean 0.17 mm, shell 0.10 mm (trustworthy); jaw, servo support,
  eye, FACE and the pads 0.4-1.0 mm and visibly misplaced (the merged sim mesh does not contain them as separate shapes) - NOT usable
  as placed. Collection `orig_head_row` = the same parts side by side at full size, for looking at.
- Our head now: merged old shell x -11.9..85.4 (bill end at +x, wide and blunt; rounded end at the back), y +-34.5, z 190.9..235.4.
  Head-roll servo lies across the head behind the yaw block (x -8.4..17.3, z 198.5..210.7), shaft forward along x at z 204.6;
  base tray z 191.3..203.8; a standard cap ring (r 12.75) about the roll axis spans z 191.85..217.35 - it fits the tray's depth.

## Head fit study: beak servo option A + head electronics (2026-11-11)

Collection `head_fit` (study objects only, nothing printable yet). Layout coordinates in mm.

- **Beak servo** (15th SCS0009, `scs_mouth`): in the front of the head ahead of the yaw block, shaft along +Y toward the
  left side wall, long axis vertical, connector end up. Case x 52.4..64.6, y -101.6..-76.0, z 196.5..229.0.
  Hinge at x 58.5, z 207.0 (original hinge is about x 33, z 210, so ours is ~25 mm further forward and 3 mm lower).
- **Dome**: servo top (z 229.0) is 1.6 mm or more under the original dome's inside skin (230.6+ above the top ear,
  229.3+ elsewhere). The BVH contact with `orig_head_shell` is one internal rib of the original at x 52.5..53.8,
  which our dome will not have. Hinge at z 210 or 208.5 did poke into the skin; 207.0 does not.
- **Tray**: servo bottom (z 196.5) sits inside the original tray's ribs (tray z 191.3..203.8): material to remove.
- **Head roll -25..+25 deg**: no contact between anything in the front of the head and the yaw block, head-yaw and
  head-roll servos, U, hub Y and its plugs.
- **Jaw** (original jaw cut off behind the new hinge): opening about the new hinge lowers the bill tip 6.6 mm at
  30 deg, ~7.8 mm at 40 deg.
- **Electronics stand-ins** (assumed sizes, not real parts yet), all clear of each other, the servo and the dome skin:
  camera 15x30x25 at the face (2.3 mm to dome), depth sensor 3x12x13 beside it (1.0 mm), light 8x5x5,
  speaker 5x20x20 (1.3 mm), USB hub/audio board 6x24x18 (0.8 mm to dome, 0.2 mm to tray rib).
- Not verified: real part sizes, cable/plug room, how the dome/tray/jaw print and screw together (not designed).

## Head v2 built on the original parts (2026-11-11) - supersedes the fit study above

`tools/blender/parts/head_shell_v2.py` (collection `head_shell_v2`), check `tools/blender/head_check.py`, exploded view
`tools/blender/head_explode.py` (collection `head_exploded`, 0.3 m to the robot's right).

What the fit study got wrong (found when checked against the real original tray / dome):
- the jaw was registered back to front (bar at the back); the opening figure was meaningless;
- an upright beak servo pokes ~3 mm through the upper bill plate (plate top z 202.3) -> it now lies flat, shaft to the left wall;
- the head-roll servo's rear corner sticks 3-4.5 mm out of the original dome's round back (the earlier "fits" was a bbox estimate)
  -> user decision: bulge the dome's back;
- hub Y on top of the yaw servo is inside the dome's roof (even at roll 0) -> must move (NOT done yet);
- with the original base, the right side wall meets the head-pitch U from 3 deg of head roll -> user decision: cut the wall.

Parts (solid PLA, one piece each, watertight):
- `head_base` 20.9 g: original tray; rear tub cleared, head-roll holder (cap plate with 6702ZZ seat as a bulkhead on the floor,
  ear pads to the floor, block under the servo); floor opening x 21.9..52.5, y up to -75.6 and right wall open x 23.0..52.5
  (swept reach of yaw block / U over roll +-25 deg + 0.8); beak-servo ear pads (front one a column on the bill plate, rear one
  on a web to the left wall), 2 rails under its body, jaw pivot post, 2 slots in the bill plate for the jaw arms; 4 dome screw seats.
- `head_dome` 23.5 g: original dome; back squared off by a hollow bulge (about 6 mm at the rear corners), right skirt trimmed
  to z 204.3 over x 23..52.5, original ribs removed where the beak servo lies, 4 screw posts. The dome was cut by the base,
  so the two mate with zero overlap (as registered they overlapped by a few tenths).
- `head_jaw` 6.9 g: original lower bill from x 64 forward; arms INSIDE the head through the slots: left = horn plate on the beak
  servo's trimmed horn (r 6, as the cap joints), right = M2x6 screw pivot (free in the post, threaded in the arm).
- Beak servo: hinge x 58.5, z 210.8, tower top y -77.8. Screws: 4 dome (2 rear at x 5, y -100 +-22.2 from below with 2 mm seat;
  2 front through the bill plate at the original's holes, heads proud under the plate), 2 beak ear, 1 jaw pivot, 2 roll ear
  (these now thread into pads - the earlier "2 screws into nothing" is closed).

Checked (exact: boolean common volume / sampled penetration / edge-through-triangle; NOT voxels):
- no part sits inside another by more than 0.03 mm at rest; roll servo to dome 0.54 mm, to base seated; yaw block to base
  0.40 mm; U to base 2.17 mm; jaw to base 0.36 mm (arm / pivot post), to dome 1.07 mm; beak servo to jaw 2.30, to dome 6.3,
  to yaw block 6.8 mm.
- head roll -25..25 deg (1 deg steps), yaw 0: no contact.
- jaw 0..35 deg: no contact with base, dome, servo, yaw block, U. Lowest point of the bill drops 6.1 mm at 20 deg, 8.1 mm at 30 deg.
- head yaw -170..170 at roll 0: no contact.
- **yaw and roll together do NOT work**: with the head yawed more than about 10 deg, any roll beyond about +-5 deg puts the
  head base (and from ~15 deg the jaw / dome) on the head-pitch U or its bearing plate (U is r 31.7 about the yaw axis, the
  base tub is 30 wide). Open - needs a narrower U top / bearing plate or a limit in the controller.
- Tool lesson: triangulate BEFORE changing frame (n-gons with holes triangulate wrongly once rotated -> phantom contacts), and
  BVHTree.overlap alone is not proof of contact.

Not done / not verified: hub Y and the beak servo's lead + 13th hub (wiring.py still has the old hub on the yaw servo);
face panel, eyes, camera / speaker mounts (the dome's front is open, as on the original before its FACE part); TPU bill pads;
assembly path and driver access (planned: jaw up through the slots, pivot screw from the left, beak servo slides onto the horn,
ear screws from the right, horn screw from the left, all with the dome off); print orientation / overhang check (bearing seat
prints with its axis horizontal); head pitch / neck poses with the new shell; sim masses.

### Face panel + camera (2026-11-11)

`head_face` (8.4 g, part "face" of head_shell_v2.py), after the original FACE: D panel in the dome's 1.9 mm front groove
(x 75.6..77.5, found by probing the dome; 0.13 mm to the dome), round camera collar in the middle, one opening each side.
Ours is a hollow box to x 84.1 (beak servo's front ear and the camera board lie in it); the collar is a hood round the lens,
dia 29.8, to x 100.4 = 15 mm past the dome's top front edge, 19 mm past the bill plate (user asked for a walled bulge
instead of a bare lens). Side openings (distance sensor 8 x 5 window right, 5.2 hole for a 5 mm light left) sit ABOVE the camera
row at z 226, y -100 -+ 14. Held by the groove and the base: no screws. Camera stand-in `cam_ov5640_usb`: Waveshare OV5640 5MP USB
Camera (A)/(C), board 18 x 36 at x 78.4, body 18 x 18 x 19.4 at the board's left end, lens centre y -99.6, z 212.0 - the body's
place on the board and the parts on its back are NOMINAL. Board slides into end grooves from below.
Checked: no penetration face/dome/base/servo/camera; face to beak servo 0.55 mm, to jaw 3.3 mm; roll +-25 and jaw 0..35 no contact.
Not done: distance-sensor and speaker/mic mounts (no confirmed board sizes), USB hub, cables, whether the real camera's layout matches.
Whole-duck exploded view (`duck_v2_exploded`) now carries the new head as one unit (explode_all.py updated); `sec_head_roll`
still shows the old shell. Blender crashed twice from a scratch script that iterated a freed bmesh (fixed).

### All-USB rear bay variant, beside the current duck (2026-11-11)

User wants no loose jumper wires. Variant `duck_usb_bay` (tools/blender/parts/rear_bay_usb.py): a copy of the assembled duck
0.30 m behind the current one (world x -0.30) with `electronics_enclosure_usb` instead of the enclosure; the current duck is untouched.
- Idea: Arduino's documented USB arrangement - a USB-C Power Delivery hub powers the Uno Q and carries data; servo adapter on its
  own USB-C (switch B), camera and sound card on the hub; distance / balance sensors on Qwiic. No VIN wires, no header cap.
- Enclosure: same cross-section (keel 36, pocket 44 wide), rear face x -53 -> -97 (+44 mm), old rear wall opened, own lid
  (2x M2x6), 7 sound holes in the floor, 3 driver holes in the new rear wall for the body screws. 53.8 g solid (was 34.8 g).
- Stand-ins, on end, from the old rear wall backward: Waveshare USB TO AUDIO 53 x 23.5 x 14.6 (retailer figures); battery->USB-C
  PD module 35 x 25 x 12 (NOMINAL, no part chosen); bare 4-port USB 2.0 hub board 28 x 18 x 6 above it; 1-port PD dongle
  60 x 28 x 12 (NOMINAL, no part chosen); 28 mm speaker flat on the floor (NOMINAL). No crossings; gaps 0.4..5.6 mm.
- Legs: 1323 poses (hip yaw x roll x pitch x knee x ankle at range ends and steps, both legs, sampled points, 1 mm margin):
  a leg already enters the PRESENT bay's envelope in 538 of them (range-end combinations); the extension adds 8 (0.6 %), all
  at hip roll +-22 deg with the foot folded behind the body.
- Not done: USB plugs and cables (bulky, not modelled), real part choices and sizes, mounts for the boards, extra mass behind
  the hips (balance / simulation), ground clearance when leaning back, wiring.py, whether a USB speaker works on the Uno Q.

### Rear bay restart from the UNO Q datasheet (2026-11-11)

The all-USB variant (+44 mm rear bay) was rejected by the user as too big; its collection and script are removed.
Source now: the user's datasheet ABX00162-ABX00173 (44 pages, modified 11/11/2026). `tools/blender/parts/uno_q.py` draws the
board from it (objects `unoq_*`, the old `uno_q` box hidden).

Mechanical (datasheet section 12): 68.58 x 53.34 mm (body_v2.py has len 68.85 - a typo, 0.27 too long); 4 holes R1.6 at
(13.97, 2.54) (15.24, 50.80) (66.04, 7.62) (66.04, 35.56) from the corner under the USB-C end; two 3.6 x 45 deg chamfers at the
far end; tallest top part 8.5 mm above the PCB; underside parts under 2 mm (our stand-offs are 4.5); USB-C 3 mm high, 1 mm past
the edge; two 60-pin connectors UNDER the board (JMISC, JMEDIA), 3.01 x 38.5. The lid's four M3 stand-offs sit within 0.10 mm
of the exact holes. USB-C end drawn at the REAR: socket at x -53.8..-46.4, y -93.4..-84.4, z 133.2..136.2.
Positions of USB-C / button / JCTL / JSPI / Qwiic and the header ends are scaled from the drawing (+-0.5 mm).

Electrical facts that change the design:
- Power: VIN 7-24 V on JANALOG pin 8 (GND pins 6, 7); or 5 V on the 5V pin (JANALOG 5 / JSPI 2), up to 3 A; or USB-C 5 V / 3 A.
- "A protected P-channel MOSFET (Q2801) can source USB VBUS from 5V_SYS when the board operates as a USB host/OTG" (4.3): the
  board itself can power a USB device while fed from VIN. JCTL pin 10 = VBUS_DISABLE. (An Arduino forum reply said the opposite;
  the host-mode switch is also reported as a software issue on older images - to be tested on the real board.)
- Servo bus: D0 = PB7 USART1_RX, D1 = PB6 USART1_TX, 3.3 V, FT pads (5 V tolerant as inputs) -> servo adapter's UART header.
- Qwiic = I2C4 (PD13 SDA, PD12 SCL), 3.3 V: balance sensor + distance sensor chain.
- Light: any PWM pin, e.g. ~D9 (PB8); not ~D3 (not 5 V tolerant, op-amp output).
- Audio: the board has analog audio (mic, headphone, line out, earpiece) only on JMISC under the board (needs a carrier).
- Do not use A0 / A1 above 3.3 V; JCTL / JMISC MPU pins are 1.8 V.

### Head-chain hubs and UNO Q connections rerouted (2026-11-11/12)

User: keep the present bay; a few header wires are acceptable; hubs may use the head if needed, neck preferred.
- Hub on top of the head-yaw servo: NO seat there clears the dome through head roll (search, 7 and 4 mm plug room) - removed.
- **Hub Y** now on the head-pitch U's LEFT arm, outer face, over the head-pitch horn screw (layout 38.0, -82.2, 168.5, board normal +y):
  takes the head-yaw lead, the cable from cradle hub C and the cable to the head. Needs a ~2 mm printed pad on the U (not built;
  it covers the horn screw, so it goes on after that screw).
- **Hub M** in the head, flat above the beak servo (64.5, -104.0, 219.5): head-roll lead, beak lead, cable from Y. Rests ~1 mm above
  the servo's top (pad / foam, not built).
- Exact check of both hubs with their plugs (edge-through-triangle, as drawn): rest + 117 poses neck -80..+19.5 x head pitch -75..+85
  + 175 poses yaw +-170 x roll +-25: no contact. (The right arm's outer face has no seat. Two plugged hubs do not fit in the head
  together with 7 mm plug room.)
- wiring.py: 13 hubs, 35 cables. New: lead_mouth, ext_cradle_to_U, ext_U_to_head, usb_camera_to_uno, signal_light_to_uno;
  VIN / UART / Qwiic now end on the real pins (VIN+GND = JANALOG 8/7, D0/D1 + GND, Qwiic socket, light on ~D9).
  Reach over 211 poses: 0 short. head yaw lead 82/150, head roll lead 111/150, beak lead 29/150, cradle->U 113/165, U->head
  115/165; camera USB cable needs >= 328 mm, light wires >= 314 mm (routes along the bus cable up the rear of the cradle).
- NOT done: USB-C plug shape at the rear of the UNO Q (a straight plug sticks ~20 mm out behind the duck), which connector the
  camera board has, housings on the header wires, distance sensor (no room in the face: proposed on the body's chest, on Qwiic),
  cable collision checks (only lengths are checked), hub pads on the U and the beak servo, host-mode USB test on the real board.
- Fixed: the beak servo's horn and horn screw had drifted out of place in the scene (re-parenting on re-runs in head_shell_v2.py);
  back on the servo, script corrected; horn to base 2.45 mm, horn screw to dome 3.57 mm.

### Audio tray, block battery, USB hub, switch (2026-11-12, user: "do with the audio parts")

body_v2.py options `audio_tray`, `battery`, `battery_y0` (run with audio_tray true, battery [40.2, 24.3, 24.2], battery_y0 -114.7); also
corrected: Uno Q length 68.58 and its exact hole positions (datasheet). Then uno_q.py and wiring.py re-run.
- `body_lid` 14.9 g: walls round the Uno Q's outline 17 mm high (posts fused into them) = tray under the board; the board's underside
  is now at z 144.1 (was 131.6, +12.5). Openings: rear (USB-C cable down), front (cables to the neck), 18 sound holes left, 5 slots right.
  Window over the battery (the block pack reaches 126.9, into the plate's thickness).
- Battery stand-in: 3S 450 mAh block 40.2 x 24.3 x 24.2 (JSumo listing), to the right between the lid posts; switch stand-in 12 x 6 x 6
  (NOMINAL) beside it at the open rear, where the battery's main and balance plugs are reached for charging (plugs not drawn).
- Tray: `usb_audio` = Waveshare USB TO AUDIO 53 x 23.5 x 14.6 flat (0.3 under the board's underside connectors), `speaker` 25 x 15 x 4
  (NOMINAL) on edge behind the sound holes. `usb_hub` = Adafruit CH334F 2-port 24 x 20.1 x 5 flat above the regulator (0.7 to the adapter).
- Exact crossing check of the new / changed parts against everything: none (screws in holes aside).
- wiring: usb_uno_to_hub 128, usb_hub_to_audio 125, usb_camera_to_hub worst 284, light wires worst 319 mm; 0 kit cables short.
  The shelf's front notch now carries 7 cables incl. three USB cables - NOT checked for room.
- Head over the back (672 poses neck x head pitch x yaw x roll): lowest point over the board's footprint reaches below the board's top
  parts (now z 154) in 84 poses, was 50 with the board at 142; below upright wire plugs on the headers (~z 166) in 181. Needs a combined
  neck / head-pitch limit; low or right-angle housings on the header wires.
- Open: real sound card connector / mic positions, hub's solder-pad outputs, USB speaker support on the Uno Q, which packs can be bought,
  balance charging via the rear plugs (no charger on board), plugs / housings / cable collisions, mass and balance, exploded labels.

### Optional covers: thighs and neck (2026-11-12)

`tools/blender/parts/covers.py` (collection `covers`), built from the hubs / plugs as placed by wiring.py. The duck works without them.
- **Thigh cover** (each 8.1 g): outer plate + top and rear walls down to the thigh plate, short lip elsewhere; outline = the original
  thigh outline grown 4.1 mm, squared out only at the top rear corner where a plug lies beyond it; plate 9.7 mm beyond the servos'
  end faces. 2x M2x6: from outside into a post on the web, from the top into a boss above the hip-pitch servo's rear part.
  The post and boss are now part of the thigh (leg_v2.py `cover_mounts`, thigh 10.4 g; both legs rebuilt, parent_hw.py re-run).
  Checks: no crossing with the thigh, servos, hubs, plugs at rest; over 229 poses per leg (hip yaw x roll x pitch, knee x ankle,
  pitch x knee) the cover is reached only by the foot at knee +-90 with ankle -+45 (1 pose each) - the bare thigh is reached by the
  foot in 30 poses of the same grid.
- **Neck cover** (4.8 g, one C-shaped piece, snaps on from the left, no screws): caps over hubs N (front) and C (rear), strap along
  the servos' left side (z 143..158), two 0.8 mm snap tabs behind the cradle floor's right face, rib in the gap between the servo
  cases, open underneath, notch for the cable to hub Y. No crossing at rest (0.85 mm to the nearest socket).
  It COSTS range (5 deg grid): head pitch forward free to +70 (contact from +75; was +85), neck forward free to -65 (contact from
  -70 with the body's bearing plate; was -80), neck back free to +5 (contact from +10 with the lid / Uno Q, which now sits 12.5 mm
  higher; was +19.5), and head pitch -75 with the head yawed / rolled brings the head base onto it.
- A first attempt edited the thigh meshes in the scene and broke them; restored from the saved file, then done in leg_v2.py.
- Not done: snap-tab flex and print check of the neck cover, cover copies' labels in the exploded view, trimming the neck caps to
  win range back, thigh cover vs cables (lengths only).

### Soles, chest distance sensor, light, mounts, Uno Q cover; neck cover dropped (2026-11-12)

User: no neck cover (removed; covers.py builds it only with "neck": true). Built instead:
- **Soles** (`parts/soft_parts.py`, collection `soft_parts`): the original 2x_foot_bottom_tpu.stl at 0.92, 11.1 g TPU each, plate 2.3 mm
  under the pad, its raised middle up into the pad. Our pad (from the simulation mesh) had a closed underside, so leg_v2.py (`sole_stl`)
  now cuts the sole's own shape out of it: foot 26.9 -> 23.8 g, sole in pocket 0.00 mm. Both legs rebuilt (twice today: cover mounts, pocket).
  The duck stands 2.3 mm higher on them than the model without.
- **Distance sensor** `tof_sensor` (25.5 x 17.7 x 4.6, Adafruit VL53L1X STEMMA QT size, NOMINAL) on a wider, taller chest plate of the
  body frame (body_v2.py `tof`), Qwiic cable from the balance sensor through the keel's port (106 mm path). NOT fastened (board holes unknown).
  Motion: reached only at neck -80 (hub N's plug) and with the head folded onto the chest (neck <= -60 with head pitch +85) -> neck
  forward free to -75.
- **Light** `light_led_5mm` in the face panel's hole (5 mm LED, flange behind the wall, 1 mm proud); hole lowered 1.8 mm so it clears the
  roof. Wires to ~D9 + GND as before.
- **Mounts** (body_v2.py): sound card between two rails and two stops on the tray floor, kept down by the board above (0.3); speaker in
  a three-sided pocket against the left wall; USB hub now ON EDGE on the battery shelf between the battery and the left lid posts (rail +
  rear stop; 0.4 / 0.35 mm side gaps - tight); power switch in a pocket on the shelf at the open rear; battery between its ribs and stops,
  under the lid window, held down only by the sound card above it (foam) - no strap (no room beside it).
- **Uno Q cover** `uno_cover` (10.8 g, optional): skirt over the tray walls, ceiling 4 mm above the headers (so the header wires need
  RIGHT-ANGLE housings), front top edge chamfered 5.5 mm, window over the LED matrix (board x 31.3..58.8, y 4.4..20.7 scaled from the
  datasheet drawing), notches for the USB-C plug, VIN, Qwiic, light and UART wires, 2x M2x6 into the tray wall beside a board post.
  Tray posts trimmed flush with the walls.
- Neck pitch x head pitch map (yaw 0, roll 0, hubs included): WITHOUT the cover the raised board / tray already blocks neck back from
  +15 (all head pitches) and head pitch back (-75..-45) at neck -40..+10; so neck back is now free to +10, was +19.5 before the tray.
  The cover adds: neck +10 blocked (free to +5) and one more 10-deg step of head pitch back at neck -20..+5. Head pitch forward unaffected.
- Exact crossing checks of every new / changed part at rest: none. Cables: 0 short over 211 poses (USB runs 66 / 55 / 263, sensor 106).
- Open: fastening of the distance sensor; real connector positions on the sound card / hub; hub fit is tight; right-angle housings;
  bill pads; eyes; body side shells; yaw + roll; simulation with the new masses; print / assembly checks.

### Simulation of the full v2 assembly (2026-11-12)

Pipeline (all re-runnable): `tools/blender/body_mass.py` (mass / COM / inertia per body from the built parts: printed = solid PLA x 0.75,
servo 13.2 g, bearings 4.0 / 1.0 g ASSUMED, steel by volume, electronics by table, wiring 31 g spread over the nearest bodies) ->
`sim/body_mass_v2.json`; `tools/blender/export_bodies.py` (now decimates a body over 190 k faces - MuJoCo's STL limit is 200 k - and no
longer skips render-hidden servos) -> `sim/assets_v2`; `tools/sim/apply_body_meshes.py` + new `tools/sim/apply_body_inertia.py` (masses,
inertias, sites imu / tof / head_camera moved to the real parts) -> `sim/robot_scs0009_v2.xml`, `scene_scs0009_v2.xml`; fix_key_heights.
Clean copy `exports/xgoduck_scs0009/sim_v2/`; trainnr robot **xgoduck_scs0009_v2@9cbc173ecab9** (project xgoduck_scs0009). Studio
recording `xgoduck_v2_vs_original` (original grey, v1 yellow, v2 blue, 8 tests back to back).
- Mass 682 g (sim model before: 561 g): trunk + bay 233 (was 195), head 89 (was 109), thigh 58 (was 37), foot 33 (was 23), shin 22 (15).
- Bug found by the sim: the TPU soles were laid level in Blender's display pose, but the original pad's underside is the plane that is
  level at the STAND keyframe (5 deg off). Standing on the soles' outer edges, the hip rolls fought each other (93 N mm, 41 % of stall).
  Fixed: soft_parts.py `sole_matrix` puts the sole on that plane (floor-down direction from MuJoCo), the foot pocket in leg_v2.py uses the
  same function; legs and covers rebuilt. Now 4 contacts per foot, hip roll 30 / 21 N mm. Foot 22.2 g.
- Servo tests (servo_tests.py, `sim/v2_tests.json`), worst servo % of stall: stand 33 (knee; before 33), squat 0.3 41 (37), shift 32 (33),
  neck -1.2 rad 56 (59), neck +60 deg 47 (50), head pitch -1.5 37 (40), neck to -135 deg 80 (83). Fast neck -135 in 0.75 s: tilt 2 deg (before
  42 deg; the original falls). Neck back + head pitch back (1.047 / -1.2): v2 stays up, the original and the previous model fall.
  sit_stand and one_leg fall for all three robots (scripted, too abrupt) - not a v2 difference.
- NOT shown by these tests: self-collision is off in the model (neck -135 passes in sim, the real neck now stops at about -75 with the chest
  sensor; head over the back stops on the Uno Q); the masses rest on the assumptions above; no walking policy has been run on v2.

### Sole merged into the foot: one printable piece (2026-11-12, user decision)

leg_v2.py: the original sole's shape (2x_foot_bottom_tpu.stl x 0.92) is now UNIONED into the foot instead of a pocket for a TPU part
(`sole_merge`, default true), plus a skirt that carries the pad's underside outline down to the sole's bottom so nothing overhangs.
Placement fixed on the way: the sole had been centred on the mean of the pad's vertices (4 mm off at the rear, 5 mm proud on one side,
and the left foot's union came out non-manifold); now on the middle of the pad's underside outline. Separate sole objects removed.
- foot 32.4 g solid PLA each, one piece, watertight, footprint = the original pad exactly (x -34.7..15.0), sole face 2.0 mm below the old pad.
- Printed sole-down (on the plane that is level when the duck stands): 133 mm2 of overhang left, largest face 13 mm2 (was 516 mm2 / 202 mm2
  before the skirt).
- Knee x ankle grid (left, 15 x 10 deg): the merged foot collides in exactly the same poses as the foot without a sole (knee forward
  limit 45..75 deg depending on ankle, ankle -90 end stop) - nothing lost.
- No soft TPU under the foot any more: grip is PLA on the floor (sim friction unchanged; the real duck may slip - a TPU sole could
  still be printed separately with sole_merge false).
- Sim rebuilt: 675 g; trainnr robot **xgoduck_scs0009_v2b@2b7873e6e299** (v2@9cbc173ecab9 kept: trainnr never overwrites). Worst servo:
  stand 33 %, squat 42 %, shift 33 %, neck -1.2 rad 56 %, neck +60 deg 47 %, head pitch -1.5 37 %; all tests upright.

### Review of all 15 servos, horns and grooves; strip camera in a brow band (2026-11-12)

- `tools/blender/servo_review.py` (data/servo_review.json): every shaft on its joint axis (0.00 deg, <= 0.03 mm), each case in the
  right holder, each horn only in the part it drives, holder / output bodies = the joint's two bodies. Head yaw range +-170 deg exceeds
  the SCS0009's 300 deg travel (+-150) - to be limited (joint_range props not changed yet).
- `tools/blender/horn_check.py` (data/horn_check.json) found and the fixes removed: (1) beak groove in the jaw turned 45 deg to the
  horn (36 mm3 overlap) -> angle 0; (2) trimmed horns (10 cap joints + beak) had NO end clearance -> joint_v2 pocket_cutters +0.2 at
  the flat end; (3) three different tapers for the same star horn (model: to the tip width at the round tip's start, 10.14; grooves /
  ribs: at 11.94; trimmed model: 11.94) -> one taper (HORN_CROSS_STRAIGHT) in scs0009.py, pocket_cutters, rib_adds; all 15 servo models
  regenerated (refresh_servos.py; scs_mouth tagged servo_joint "mouth"), all grooved parts rebuilt. Now: trimmed + beak side 0.20,
  end 0.20, rib bite 0.22, top 0.13, hole 2.28 / cbore 4.18; full cross side 0.30, end 0.30, ribs 0.19. Hub below the plate sits in
  the arm-shaped void (the shaft centres it). Lid screw heads cleared from the sound card's rails. Horn section is still NOMINAL:
  Feetech's spec sheet lists no accessories (no horn drawing) - real horn to be measured.
- Feetech spec (Seeed PDF): stall 2.3 kg cm / 0.8 A at 6 V, no-load 120 mA, 330 deg pot / 300 deg used, connector 5264-3P, lead 15 cm.
- Camera: Waveshare OV5640 5MP USB Camera (B) (the only one available) per Waveshare's drawing: PCB 62 x 9 x 1.0, camera 8.5 x 8.5 x 5.6 at
  29.5 from the socket end (lens D5.5, FOV 68), socket J1 3.2 high on the camera side at the end, LED ~49.4 (scaled), 3 x D2.1 holes
  (10.2, 7.0) (60, 2.75) (60, 6.75). It cannot sit inside the dome (61 inside < 62), so head_shell_v2.py `camera: ov5640_b` builds a
  BROW BAND across the face in front of the dome's edge (x 83.4..90.5, y -134.3..-66.0 within the dome's outside, z 203.6..216.4):
  lens on the centre line 0.5 proud, LED window (the separate 5 mm LED and its wires are gone, D9 free), socket at the duck's right,
  cable straight back through an opening; 3x M2x6 from behind into bosses in the front wall (5 mm thread, tips 0.1 short). Face 8.8 g,
  watertight; hollow skin otherwise. No penetration with dome / base / board; jaw 0..35 deg and head roll +-25 deg: no contact; camera
  cable 270 of 400 mm. Prints on its side with 409 mm2 of supports (largest face 34 mm2).

## Joint ranges, neck clearance, sim v2c (2026-11-12)

**Problem found.** The original STAND (neck +20°, head +20°) and SIT (neck +29°, head +92°) poses collided. The Uno Q sits
12.5 mm higher for the audio tray, so when the neck leans back the neck-side parts come down on the board, its cover and lid.
Hub C (cradle rear wall) hit at neck +10°. The cradle hit the full cover at +25°. Original neck range: −135…+60°.

| neck-back limit (rigid parts) | cover on | cover off |
|---|---|---|
| board as built (raised) | ~20–24° | ~30–34° |
| board 12.5 mm lower | ~35–39° | ~45–49° |

**Fixes**
- Hub C is now **stacked on hub N** on the cradle's front wall, a quarter turn round (`wiring.py`). No seat on the rear/front wall
  was clear. Its cables were rerouted, and all 37 cables still reach over 211 poses.
- Hub Y stays on the U's left arm, turned 2 (`y_turn`). No turn is clean at neck +30°.
- **Uno Q cover cut** by the space the neck and U sweep (neck 0…+30°, head pitch −45…+90°, 1.5 mm clearance). The cutter is one
  side-profile prism (`parts/neck_sweep.py`, also in `body_v2.py` via `"neck_sweep"`). The piece cut loose at the front is
  dropped, so the cover is now a hood over the board's rear ⅔ (7.1 g solid, one shell, manifold). The board's front ~20 mm,
  including the front of the left header row, is open under the neck's swing.

**Usable ranges** (`sim/joint_ranges_v2.json`; Blender `joint_range`; MJCF):

| joint | new (deg) | original | limit by |
|---|---|---|---|
| knee L / R | −90…+68 / −68…+90 | ±90 | foot reaches the hip L-bracket |
| ankle L / R | −86…+90 / −90…+86 | ±90 | shin |
| neck pitch | −80…+25 | −135…+60 | forward: tof/chest; back: hub Y plug on the header row (>+20), neck on board (+35) |
| head pitch | −35…+65 | ±105 | ≥+70: jaw / yaw block on stacked hub C; back: head on bay |
| head yaw | ±150 | ±170 | leads |
| hips, head roll | unchanged | | |

Coupled, for the controller (MuJoCo has no self-collision here): neck 0 → head ≥ −35; +10 → ≥ −15; +20 → ≥ 0; +25 → 0…+25.
Head yaw beyond ±45° with full roll still meets the U.

Keyframes: STAND unchanged (clear). SIT is now neck 20 / head 65 (was 29 / 92). FOLD can't be reached; it is clamped to
neck 25 / head 25 / knees 68.

**Verified** (exact intersection volumes, `leg_motion_check.py`): each of the 14 joints alone over its whole new range (9 samples):
clean. STAND, coupled-boundary poses, head 65 at yaw ±150: clean. SIT and neck 20 / head 65 show a 0.1 mm³ graze of a cable
plug on the left header. Still touching, as before: the 46°/55° hip/knee crouch combos (foot on the L-bracket), leg folded up
behind (foot on the bay), yaw > ±45° with full roll.

**Sim** (`xgoduck_scs0009_v2c@8425d4c7ff9a`, `exports/xgoduck_scs0009/sim_v2/`): 667.7 g, OV5640 (B) mass 5 g ASSUMED, head
camera site moved to the strip's lens. Servo tests (`v2_tests_c.json`) all upright, no saturation. Worst loads: stand / shift
32% (right knee), squat 41%, neck −69° 55%. The original duck at our scale: 39 / 52 / 58%.

**User decision (2026-11-12):** the limited neck range is accepted. The audio tray stays under the raised Uno Q; FOLD is dropped.

## Camera only: audio dropped, board lowered (2026-11-12)

**User decision:** no speaker and no USB sound card ("mic is already there in the camera"). That also removes the USB hub: the
OV5640 (B) camera's USB cable runs straight from the Uno Q's USB-C to the head (`usb_camera_to_uno`; worst reach 335 mm, so buy a
40 cm cable). **User chose to lower the board** back to its old height (4.5 mm posts, 12.5 mm lower).

- `body_v2.py`: new flags `"audio": false` (no sound card, speaker, hub or their rails, sound holes or mic slots) and
  `"board_stand": 4.5`. The tray walls, Uno Q cover, power-switch pocket and battery window stay. Cover screws sit halfway up
  the low walls. Build arguments are saved in `projects/xgoduck_scs0009/data/build_args/`.
- The board is at z 131.6–133.2. The battery top clears the board's underside connectors by 2.8 mm.
- Cover recut by the swept space of the neck, the U and hub Y with its plugs (neck 0…+40°): one shell, manifold, 9.6 g solid.

**Ranges** (replace the table above): neck **−80…+40°** (was +25°). Head pitch −45…+65°. Hip pitch, leg back: 85° (a straight
leg's foot reaches the lower cover at 88°). Knees, ankles, yaw and roll as before. Coupled: neck 0 → head ≥ −45; +10 → ≥ −35;
+20/+25 → ≥ −15; +30/+35 → ≥ 0; +40 → ≥ +20.

Keyframes: STAND unchanged. SIT is neck 28.6° (original) / head 65°. FOLD is neck 40°, head 57° (original), knees 68°, hips 85°.

Verified (exact volumes): each joint alone over its full range is clean, except neck +40° with head 0°, which is the coupled
limit. STAND, SIT, FOLD and the coupled-boundary poses are clean. The usual combined extremes still touch.

**Sim** `xgoduck_scs0009_v2d@09a2f6a0dd50` (`exports/xgoduck_scs0009/sim_v2/`): 646.4 g. Servo tests (`v2_tests_d.json`) all
upright, no saturation, worst servo load 32–55% (original duck at our scale: 39–58%).

## Battery: Pro-Range 850 mAh 3S 30C, bay lengthened (2026-11-12)

**User decision:** Pro-Range 850/3S-30C (57 × 31 mm, 75 g, Deans T-plug, JST-XH balance; height not listed, so ≤ 22 mm is
assumed, and the lid window takes up to ~24 mm). The battery pocket is **23 mm longer toward the rear** (`"pocket_ext": 23`).
The keel, servo adapter, regulator, IMU, Uno Q and its cover did not move.

- The pocket is 67.8 mm long inside. The battery sits centred, 6.3 mm from the rear wall, which leaves room for its leads.
- The extension has full-height walls up to the lid, two more lid posts and screws in its rear corners, and the shelf runs on
  into it. Its underside starts at **z 99**, so nothing hangs below that behind the keel where the feet pass. That overhang
  **needs print supports** (tree supports under the extension).
- **Power switch:** now under the lid's rear end, in the leads' space, with its slider up through a 3 × 7 mm slot in the lid
  (reachable from outside). Side rails and the rear wall locate it; add a dot of hot glue. It is still NOMINAL 12 × 6 × 6.
- Part weights: enclosure 39.0 g, lid 9.9 g, shelf 5.5 g. Duck total **680.2 g** (was 646.4).
- Fits: 31 bay fits, all as designed. The battery has 0.2 mm side gaps to the posts and walls and 0.4 mm under the lid.

**Motion:** every joint alone over its full range is clean (hip 85° back included); only neck +40° with head 0° touches,
which is the coupled limit. A backward step clears the new rear section up to about 34° of hip pitch with the knee folded and
the hip rolled out. At 40° and beyond the foot reaches it. A 29° step with the hip rolled in touches the **original keel**
(this was already so). The extreme "walk" combinations (46°) still touch, as before.

**Sim** `xgoduck_scs0009_v2e@842feeddc827`: 680.2 g. All servo tests (`v2_tests_e.json`) upright, no saturation. Worst loads:
stand 28%, squat 38%, neck −69° 55% (original at our scale: 39 / 52 / 58%). Runtime estimate ~50–75 min (not measured).

## Battery standing upright, regulator Mini560 5 V (2026-11-12) - replaces the lengthened bay

**User:** the horizontal 850 mAh pack stuck out at the back. Stand it upright and keep the centre of gravity near the middle.
The bay is back to its original length (`pocket_ext` dropped). Section view: `data/bay_upright_battery_section.png`.

- **Battery** (Pro-Range 850 3S, 57 × 31, thickness assumed 21) stands at the keel's front, 0.3 mm from the front wall, between
  side ribs, leads at the top. It runs from the keel floor (z 58.9) to z 115.9. The trunk's centre of mass is 8 mm further forward
  and 5 mm lower than with the lengthened bay.
- **Servo adapter**: the real board, from Waveshare's STEP (42 × 33 × 14.6 incl. pins; `parts/adapter_mesh/`). It stands across
  the keel behind the battery, sockets toward it, its edges in rib slots (ribs at the top and bottom of each edge only: the power
  edge's DC jack, terminal and USB-C, and the socket edge's sockets and header pins, reach the edge). **Power and UART wires are
  soldered on the back**: the screw terminal faces the side wall, 0.4 mm away. There is a 4 mm gap behind the board for those
  wires. The servo cables leave the plugs and bend up in a ~6 mm gap in front of the adapter, cross over the battery and leave
  through the pocket's front notch.
- **Keel** 0.5 mm wider each side (`keel_hw` 18.5) for the 33 mm board. The 3 front screws are sunk flush (M2 countersunk,
  Ø4.0 seats) because the battery stands against that wall.
- **Balance sensor** upright on 4 bosses on the pocket's rear wall, above the adapter (the wall is now full height).
  The software has to know its new orientation.
- **Regulator: Mini560 5 V / 5 A** (29 × 18 × 5.4 mm, ~₹93 at Zbotic/Robodo), clipped under the lid's rear part. No 6 V module that
  small is sold in India. The SCS0009 datasheet allows a 4–7.4 V input, so at 5 V stall torque is about 0.188 N·m instead of
  0.226 (`servo_torque_nm` in `sim/joint_ranges_v2.json`). Listings say "5 A max, about 3 A continuous without heat dissipation",
  so this is to be measured on the robot.
- **Power switch** under the lid's rear part, slider out of the left side opening.
- **Cables:** the adapter → left thigh cable now needs **250 mm** (it runs over the battery and down the deck slot). Distance
  sensor and IMU Qwiic cables are rerouted.
- Fits: all as designed. The tightest is the adapter in its rib slots (0.15 mm); IMU on its bosses is touching (designed).
  Weights: enclosure 30.0 g, lid 11.1 g, no shelf. Duck 660.3 g.
- Motion: same as before, except at neck +40° hub Y's plugs needed the cover recut after rewiring (build order step 4, see
  `data/build_args/README.md`). Every joint alone is clean except neck +40° with head 0° (coupled limit); the extreme walking
  combinations touch as before.
- **Sim** `xgoduck_scs0009_v2f@4642785293d4` with 5 V torque: all servo tests upright, no saturation. Worst loads: stand 40%,
  squat 50%, neck −69° 66% (at 6 V the same body would be ~33 / 42 / 55%).

Buy list changes: Mini560 **5V** 5 A (not the 6 V UBEC); one **250 mm** 3-pin servo extension (adapter → left thigh);
M2 **countersunk** screws ×3 for the bay front wall.

## Distance sensor dropped (2026-11-12)

**User:** remove the distance sensor for now; the camera covers obstacle sensing. The chest plate (its only job was to carry it)
and the IMU → sensor Qwiic cable are gone (`"tof": false`). The body frame is now 19.7 g, and the duck 657.6 g (sim
`xgoduck_scs0009_v2g@76bf53a86cf7`, all servo tests upright, worst 65%). Fix made on the way: `parts/neck_sweep.py` now carries
the hubs and plugs with their anchor part (the U or the neck). The wiring makes them as free objects, so before this the cover
cut missed hub Y's plugs whenever the wiring had just been rebuilt. Neck +40° is clean again (the coupled limit is unchanged).
The neck's forward limit (−80°) was partly set by the sensor; it is unchanged here (hub N / C also limit it) and can be
re-checked if more forward reach is wanted.

## Balance sensor: 7Semi BNO086 Qwiic (2026-11-12)

User's choice (Robocraze, ₹2,294). The 7Semi datasheet ES-12243 gives the board as 27.89 × 20.87 mm; the hole positions are not
given, so they are ESTIMATED (±11.35 × ±7.83 mm, 2.6 mm in from the corners) and need measuring on arrival. It is mounted upright
on the bay's rear wall, long side across, 0.5 mm right of centre (`"imu": [27.89, 20.87, 6.0]`, `"imu_holes"`, `"imu_dy": -0.5`). The
regulator moved 1.5 mm forward to clear its ~6 mm depth. Fits: all as designed (≥ 0.2 mm). I²C address 0x4B (default). The
software must use the board's mounted orientation. Mass change −0.5 g (sim not re-onboarded for that).

## Power switch: MTS-102-A2T mini toggle in a side-wall recess (2026-11-12)

The battery-side current is about 2–3 A at peak (estimate), so the 12 × 6 × 6 slide switch stand-in (SS-12F15 class, 0.3–0.5 A) was
too weak. User's choice: **MTS-102-A2T** (Robu, 6 A 125 VAC, ON-ON: use the middle pin and one outer pin). It is mounted through a
6.3 mm hole in a 1.6 mm plate at the bottom of a 7 × 8 mm recess in the bay's left side wall (x −37.5, z 108). The lever tip is flush
with the outside, so nothing sticks out for a leg to catch. The body sits inside, between the balance sensor and the battery, above
the adapter. The switch clips under the lid are gone. Fits: the switch touches its plate (as designed), 1.0 mm to the IMU, 2.45 mm
to the battery. Left-leg check: nothing reaches the recess.

**New combined limit found** (it was there before this change too): leg swung back ≥ 45° *and* hip rolled fully out (22°) puts
the thigh on the hip-yaw servo and the bay's front sides (x −26…−5). Controller: limit hip roll-out when hip pitch is back beyond
~40°. The other combined extremes are as before.

## Screws: few standard types, servo kit screws at the joints (2026-11-13)

User's choices: machine screws everywhere, all bought from Robu, as few types as possible; each SCS0009 comes with its horn screw
(M2 machine screw, pan head; the output spline is brass) and two self-tapping mounting screws. So the screw set is:

- **Joints (14):** the servo's own horn screw. It is shorter than the M2 × 6 it replaces, so every joint's head seat went down
  (type C 0.6, type F 1.0, in a Ø4.2 counterbore), leaving 1.86 mm of print over the horn and ~1.8 mm of thread in the spline.
- **Servo ears (30):** the servo's own mounting screws (2 per servo).
- **Everything else (30):** M2 × 6 socket head (Robu EasyMech, k 2.0, dk 3.7), threaded into the printed Ø1.6 pilots.
- **Bay front wall (3):** M2 × 10 countersunk (Robu has no M2 × 6 CSK); deck pilots deepened from 5.1 to 8.5 mm.
- **M3 × 8 (8) + M3 nuts (4)** as before (the model already had their real heads).

Head room, measured at every screw head (99) with its real size: before this, the **hip yaw** screws' heads had only 1.09 mm under
the hip-roll servo's ear (the old stand-in already overlapped it) and the head-roll rear ear screw 1.24 mm under the dome. Fixed by
the kit screw + lowered seat (hip yaw: 0.39 mm left) and two Ø4.4 recesses inside the dome (camera upper screw, head-roll ear screw;
≥ 2.2 mm of wall left behind them). Now every head fits at rest; smallest gaps 0.14 (dome screws under the jaw) and 0.15 (mouth ear).
All cut parts are single watertight shells; their volumes dropped by exactly the counterbore volumes.

Code: `joint_v2.py` (SOCKET_HEAD, KIT_SCREW, KIT_SEAT; type-F counterbore; `screw_heads`, `head_room_cut`), joint screws renamed
`*_screw_M2x4kit` (explode_all, head_v2 updated), `head_shell_v2.py` (dome recesses), `body_v2.py` (countersunk M2 × 10).
**To measure on arrival:** the kit horn screw (assumed 4.0 long, head Ø3.5 × 1.3) and the kit mounting screws (if longer than
6 mm, the pads' 4.9 mm pilots must go deeper).

## Hardware review: screws, holes, fits, assembly, margins (2026-11-13)

Full rebuild with these changes, then every check rerun on the rebuilt model.

**Tolerances (FDM):** M2 pilot 1.6 -> 1.7, M2 clearance 2.3 -> 2.4, servo saddle clearance 0.15 -> 0.25 per side, 6702 rotor boss
14.95 -> 14.9, kit-screw counterbore 4.2 -> 4.4. Bearing seats unchanged (6702 Ø21.1, 693 Ø8.1). A tolerance test print of
these features before the full print is advisable (printers differ).

**Fixes found by the review:**
- Servo-ear screw pilots ended exactly at a 6 mm screw's tip: now type C +1.5 (to 0.6 under the bearing lip; screws up to ~7.2),
  type F 8.5 deep. If the kit mounting screws are longer than 7.2: an M2 nut under the head as spacer.
- Camera board: its bosses were cut away again by the room cut in front of the board (held by 1.25 mm of thread; screw tip 0.1
  from the face). Bosses now added last, 5 mm of thread; front wall 1.5 -> 2.2 (pilot end 0.5 inside, camera 0.2 back).
- Uno Q: lid screws in 0.7 counterbores (socket heads clear the board by 0.4); cover-screw pilots were 0.15 from the M3 bore ->
  moved to 1.0, on their own bosses, 1 mm longer (they bottomed out).
- Left hip-yaw servo: only a 0.5 mm window to go in under the neck fork. Saddle walls 8 -> 6 and a 1 mm relief in the pedestal
  over its front ear tip: now in from the rear or either side at 6.5-8.5 mm lift (L-shaped insertion).
- Unused keel cable port removed (upright bay: no cable uses it). Ear holes filled into the dome (left fully, right above the U).
- Tools: joint lookups (wiring reach, neck sweep, swept space, head check) now ignore copies outside the scene - stray copies from
  a library load had frozen the neck in the cover sweep and in the cable reach check (cover uncut; neck/head reach at rest only).

**Verified after the rebuild:** fit audit 309 pairs, 308 as designed (battery vs enclosure 0.27 mm3 = sampling: no surface
intersects). Assembly paths: all 30 leg / neck / head-pitch / head-yaw / head-roll-servo / beak-servo steps OK. Head room: all 99
screw heads fit at real size (smallest 0.14 under the jaw). Cable reach over 211 poses: none short; least slack: left body-to-thigh
extension 224 of 250 (26), usb camera 335 of 400, ext U-to-head 100 of 165. Motion: only the known coupled limits (neck +40 with the
head below +20; head yaw beyond +-45 with roll; legs folded up behind; deep crouch foot on the L-bracket).

**Still open:**
- **Head-roll yaw block** cannot be fitted: alone along the roll axis it meets the head base (9 -> 2.5 mm of travel, up to 1.0 mm
  deep); with its servo, either side, or up through the floor opening also blocked. Needs a relief in the head base along its path
  (~1.2 mm) or a different order.
- **Jaw** cannot be fitted onto the beak servo in the head base (alone either way, or with the servo from above / the front: up to
  4.6 mm). Needs an assembly concept (e.g. a removable part of the base, or the jaw's arm entering from a slot).
- Kit screws (horn and mounting) and horn: measure on arrival.

## Head assembly and driver access (2026-11-13, after the hardware review)

- **Jaw**: no change needed, an ORDER: jaw straight up through the bill-plate slots with the beak servo not yet in, pivot screw,
  then the beak servo lowered ~4.5 mm toward the middle and slid outward onto the jaw arm (all paths clear in the model).
- **Yaw block**: goes onto the roll horn from the front (3.5 mm of travel to clear the horn) - up through the floor opening
  4.3 mm forward, then back. Its front end stood 2.0 mm from the support under the beak servo: relief in the head base = the
  block's front outline (an inverted T) + 0.3, 4.6 mm deep, 0.9 mm under the beak servo. Path clear (0.2 mm touch at the very
  start, 20 mm below the head).
- **Driver access** (new check, a straight tool line ~35 mm along every screw): fixed - Ø4.6 driver holes through the face
  behind the 3 camera screws and through the jaw under the 2 front dome screws; Uno Q cover screws moved to the rear side of
  their posts, one each side (the front one had the neck plate 6.5 mm away; in front the right head sat half over the skirt's
  opening and the left boss met the board's underside parts). The left hip-yaw servo's front ear screw is reachable only at
  20-25 deg (neck fork 8 mm above, a hole would pass 0.66 from the neck horn): an M2 × 6 socket screw + ball-end key there.
  All other blocked lines are assembly order (e.g. lid screws before the Uno Q, joint screws before the next servo) or leave
  ≥ 15 mm for an L-key. The order is written down in **ASSEMBLY.md**.
- Checks after the rebuild: head fits 17/17, body + head fits 153 (only the battery sampling artifact), Uno Q fits 39/39, head
  motion only the known limits, cable reach none short.

## Horn from the kit photo (2026-11-13)

The user's STEP (fs90-horn.step) is an FS90 servo with its single-arm horn: Ø4.8 spline (the SCS0009's is 20T Ø3.95), so not
our horn. Measured instead from Feetech's SCS0009 kit photo: the X horn has 4 equal arms (not long/short pairs). Scale from the
listing's lengths: X 23.88 tip to tip = 175.8 px (7.36 px/mm), double arm 34.49 = 251 px (7.28 px/mm); the servo's ears give
8.15 px/mm at its top face, 12 mm nearer the camera - consistent with a close camera. Arm width 4.84 at r 4.1, 4.60 at r 6.8,
4.32 at r 9.6 -> 5.2 at the axis tapering to 4.3 where the round tip starts (was nominal 5.6 -> 3.6): the full-length pockets
(ankles, neck, head pitch) were 0.3 too narrow near the tips, the r-6 trimmed ones 0.24 tight. Holes at r 4.9 / 7.4 / 9.8.
Hub Ø/height and plate thickness are not visible from above: still nominal (the rotor is located by its bearing, not by the
horn's top). Uncertainty +-0.15 on widths, inside the pocket's 0.20 clearance + 0.25 crush ribs. Rebuilt every servo horn and
all 15 pockets, rechecked.
Rechecked after the horn update: horn in groove on all 15 joints (side gap 0.20 trimmed / 0.30 full, ribs 0.19-0.20 on 14-17
points, top gap 0.13; the jaw's tightest spot went from 0.002 to 0.094), fit audit 308/309 (battery sampling artifact), all
assembly paths (left hip-yaw by its L-path), head room 99/99, cable reach none short, motion: only the 20 known limit poses.

## Full kit horn, uncut: knees done (2026-11-13)

User: use the full horn everywhere (the 9 bearing joints trimmed it to r 6 to fit inside the 6702ZZ). The horn plate is then wider
than the bearing, so the bearing cannot sit round it on the fixed side.

- **Test joint (type C2, `tools/blender/parts/c2_proto.py`)**: bearing in a bridge above a full-horn plate. One-piece fixed part:
  **not assemblable** (the moving plate is trapped between base and bridge; every path blocked) - caught by assembly_path_check
  before showing. Fix chosen by the user: a **bolt-on bearing cap**. Then verified: 3 single-piece parts, 21 fits as designed,
  7-step build path, cap screws 5.0 mm in the post, the post blocks 132 deg for a 10 mm link.
- **Knee: type C2M** (`joint_v2.c2m_*`), because the C2 layout would have made the shin unprintable flat (boss 4.5 mm out of its
  print face) and moved the ankle and foot 2.8 mm. The **6702ZZ sits in the shin** (seat in its flat inboard face, outer ring turns
  with the shin, inner-ring relief in the seat floor); the **cap** carries a fixed tube boss into the bore (3.9 of 4.0 mm), a shoulder
  on the inner ring (0.1 axial play), its own **post down onto the thigh's face** (the thigh prints on that face), **2 × M2 × 6
  socket** in clearance holes (the cap centres on the bearing, then is clamped - countersunk heads would force it onto the pilots).
  Ø5.2 hole through the cap to the horn screw. Thigh: plate thinned to 1.8 under the shin's disc (room r 14.29), relief over the
  servo's boss. Shin, ankle and foot otherwise unchanged.
- **Checks (both legs)**: all leg parts one piece, watertight; 46 knee fits as designed (horn press 2.05 mm³, horn-holder 0.38,
  shin-thigh 0.38, shin-cap 0.10 at the shoulder); 12 build steps clear; cap screws 3.4-3.5 mm in the thigh (wall >= 1.28), ear
  screws 4.4-4.5. **Motion**: the first cap (top 13.8 over the shin's 9.9) hit the electronics enclosure at the **hip-yaw limit
  (-12.6 deg, 58 mm³)** and hip roll -22 deg. Clearance mapped by ray casts (hip-yaw limit: 6.9 mm over the shin face at the cap's
  front edge -> 2.2 at its rear). Cap lowered to 10.2..13.3 and its rear top edge chamfered 45 deg from 4 mm behind the axis ->
  the sweep is back to exactly the 20 poses of before the knee change. Remaining: in the 2 walking-corner poses (yaw -0.11, roll
  +0.19, pitch +-0.8, knee -+0.96), where the shin already hit the enclosure (182 mm³, a known limit), the cap now also does (200 mm³).
- **Fixed on the way**: the horn pocket cutter (pocket minus ribs) came out non-manifold when built in place at the left knee -
  now built in the joint's own frame and moved (all joints). Screw audit: long shanks (> ~8 mm) were skipped (axis from the largest
  face) - fixed; it then flagged the bay's 3 countersunk screws sitting on a flat counterbore -> 90 deg countersink (body_v2;
  **rebuilt and checked in the next full rebuild**).
- **Horn screw** (all joints, unchanged): 1.80-1.85 mm of thread in the brass spline (probed directly; the screw audit reads 1.46
  at the knees - its sampling, not the geometry). Measure the real screw.
- **Hip pitch: stays trimmed (user decision).** Built as C2M first (cap post off-centre, clear of the body-to-thigh cable; disc
  r 9.5): fits 96/96, build steps and screws fine, but the motion sweep showed the **yaw connector hitting the cap at hip roll
  +11..+22 deg and in all 12 walking-corner poses**, and the L-bracket could no longer be fitted to the roll horn with the cap on.
  My first clearance map had swung only the hip pitch; swinging pitch and roll together: the yaw connector passes **0.3-2 mm**
  over the L-bracket's inboard face almost everywhere (5.9 at best; a cap needs >= 3.9). Options shown: trimmed (chosen),
  full horn without a bearing, a 6706 thin-section bearing round the full horn (thigh ring ~Ø41), widening the hips.
- Thigh zip-tie holes: the knee cap's post now stands on the bridge's knee end over one of them -> both holes moved straight
  across the bridge, 0.9 mm before the knee post. Holder base under the moving disc thinned to 1.6 (1.0 mm under the disc): it
  prints as a ~28 mm bridge; print_check counts the new bridges (shin seat floor, thigh room) as unsupported.
- **Hip roll: stays trimmed (user decision).** Space check first (cap_space.py, all left-hip joints + knee swung together): the
  front of the L-bracket's roll plate looked free - but that check held the OTHER leg still. Both legs moved (corrected run -
  the first run started from a bent pose, see below): a test cap (plate 10.2..13.3 on both roll joints) **touches/overlaps the
  other leg with both hips rolled -22 deg (any yaw, already at 0)** and **from 18 deg of outward yaw of both hips**; at the
  walking yaw (12.6) with roll 0 it clears by 2.9 mm. A C2M roll joint would also move the L-bracket's inboard flat from 8.8 to
  11.35 (bearing seat wall). Options shown: trimmed (chosen), widen the hips, full horn without a bearing.
- **Pose corruption (my error, fixed):** a pose script crashed mid-loop and left 4 hip joints rotated (left yaw -12.6, right yaw
  -25.8, both rolls -22); the next checks started from that pose. The figures first shown to the user for the hip roll (0.12 mm,
  overlap from 5 deg) came from it; corrected above (same conclusion, smaller limits). Joints restored and compared with their
  stored rest (`q0` prop): all equal. Pose scripts now restore every joint in a try/finally.
- **New finding about TODAY's parts** (12 both-hips poses added to the motion sweep): with **both hips yawed fully outward
  (+-25.8 deg)** the two L-brackets overlap (0.1 mm3 at roll 0, 14.3 mm3 at roll +-22) and the yaw connectors (1.8 mm3); the gap at
  roll 0 just before is 0.05 mm. Yawed fully inward with roll -22 the legs cross (feet, shins: expected). Yawed inward 12.6 with
  roll +22: thighs vs enclosure 1.2 mm3 (knee caps add 2 mm3). Not caused by the full-horn work; needs a yaw limit or a fix.
- Still to do: hip yaw (same cramped hip cluster: check the space first, with all hip joints of BOTH legs swung together), head
  roll, head yaw (±150 deg: no free sector for a post), then the full rebuild with every check, the exploded view (it still holds
  the old thigh and shin copies) and the screw/bearing counts.

## Joint limits, part 1: the hips (2026-11-13)

Single joints never collide over their full ranges; combinations do. A servo's own angle limits (Feetech SCS/SCSCL memory table:
Min Angle Limit reg 9, Max Angle Limit reg 11, 2 bytes, 0..1023 steps - SCS0009 table not found, confirm on a servo; never 0/0 =
motor mode) act per joint, so the hardware limits are chosen as a **box in which every combination of the coupled joints is clear**.

- **Both hips, yaw x roll** (leg_motion_check exact volumes on a both-legs grid, then min gaps at the box corners, both legs moving):
  outward yaw 25.8 collides at every roll (L-brackets 14.3, yaw connectors 1.8 mm3 - today's parts); roll -22 (feet together)
  collides at every yaw (feet); inward yaw + inward roll: the knee caps reach the electronics enclosure's lower edge first.
  **Safe box: hip yaw -6.3 .. +19 deg (+ = outward), hip roll -12 .. +19.5 deg (- = feet together)**, corners: (-6.3, -10.9)
  0.97 mm, (-6.3, -12) 0.57, (-6.3, +19.5) 2.66, (+19, -12) 1.39, (+19, +19.5) 0.65. Walking (yaw -6.3..+12.6, roll +-10.9) fits.
- **Enclosure chamfer (user: try it)**: 45 deg 2.5 mm on the keel's lower outer edges along x -30..-5 (where the knees pass), 1.2
  inner fillet keeps the corner wall >= 1.3 (battery 0.35 clear). Knee cap at the walking corner 0.50 -> 0.97 mm, roll -12 became
  safe (0.10 -> 0.57). It cannot extend the inward yaw: at -8 deg the cap meets the flat wall higher up.
- Body rebuild found and fixed: the **IMU pilots ended exactly on the rear wall's outer face** (coincident faces: 56 non-manifold
  edges; the earlier builds were clean by luck) -> pilots through the wall. **Bay screws M2x10 countersunk now seat** in a 90 deg
  countersink (screw audit ok, 7.6 mm in the body frame). Body fits 86/86 apart from the known battery sampling artifact (6 scattered
  points mid-battery, not at the walls or the fillet).
- Sitting with the legs straight forward (hip pitch at -90): the heels and the thighs' rear corners touch first, the wire hubs and
  plugs on the thighs are 3.5-3.8 mm above the ground, the body 27 mm. User: sit/stand is wanted but not a glute sit; hardware first,
  MuJoCo sit/stand study after.

## Joint limits, part 2: neck, head; the head-roll rework that did not work (2026-11-13)

- Every joint alone in 5 deg steps: only **neck pitch +40** collides (head base vs Uno Q cover, 71 mm3). Neck x head pitch grid
  (10 deg): clean except neck +10..+40 with the head tilted down. **User: neck -80..+30, head pitch -5..+65** (corners >= 1.36 mm,
  closest pair the U and its own servo; the bearing plate's contact with the cradle at the axle is the joint itself, excluded).
- **Head yaw x roll: roll is free only facing straight** (+-15 within +-2.5 deg of yaw; at yaw +-5 a 5 deg roll touches; up to
  550 mm3 at +-45). Cause: the head base's floor opening was generated from the roll sweep at yaw 0 only; yawed, the U's top-plate
  corners (21-32 mm from the yaw axis, 1.2 mm under the floor) meet the floor as it rolls (r x sin roll).
- Rework tried (user: rework, target +-10 at every yaw, head stays put): U trimmed by the head-side parts' sweep as a solid of
  revolution about the yaw axis (computed in neck_v2, `u_roll_clear_deg`), far corner lowered 5 mm into the bearing plate's old
  block. Built clean (one piece each, screws 4.0 mm bite) - but **the U could no longer be assembled** (it slides along the
  pitch axis over the head-pitch servo; the lowered corner would pass through the cradle and the servo) and the M3 axle head hit
  the moved block. Moving the corner to the bearing plate instead leaves 1.4 mm of height above the cradle to join them. My
  estimate that reshaping the U would do was wrong: it missed the U's assembly path. Reverted (option kept, default off);
  the restored U: fits 56/56, paths 4/4.
- Options shown: raise the head on a taller U centre (+-5: ~2 mm, +-10: ~4.5, +-15: ~7), software interlock, smaller raise.
  **User: software interlock** (servo limits yaw +-150, roll +-15; roll only while facing straight). JOINT_LIMITS.md updated.
- Head yaw / head roll full horn: no room for a cap (the U's plate sits between the floor and the servo top with 1.2 / 1.5 mm)
  -> both stay trimmed.
- **Hip yaw full horn: no room** (cap_space.py, both legs' yaw/roll + left pitch swung): under the yaw connector's plate the
  hip-roll servo (turning with yaw) is 1.6-1.8 mm away over the whole cap footprint, 0.1-0.2 at its edges; a cap needs ~3.9.
  Stays trimmed. **Horns as read from the servos**: full on 6 (knees - new, C2M + bolt-on cap - ankles, neck pitch, head pitch);
  trimmed on 9 (hip yaw / roll / pitch x2, head yaw, head roll, the beak servo). The beak's jaw arm was not part of the rollout
  (not a bearing joint): still to check for a full horn.

## Trimmed horn, jig; full rebuild with every check (2026-11-13)

- **Trimmed horns** (user: the beak stays trimmed too): all 9 trimmed pockets fit the trimmed horn model (sides 0.20, end at the cut
  0.20, ribs 0.20, top 0.13); the beak's "side min 0.094" in horn_check is its sampling at a corner - measured directly, the nearest
  surface is the designed 0.1 top gap. **Trim jig fixed**: its slots had 0.06-0.10 per side at the wall (an FDM slot prints 0.1-0.2
  narrow: the horn would not have dropped in) and its wall sat at r 6.0 exactly (a flush cut = the pocket's end). Now: slots 0.25
  along the arm's real taper, a Ø8.0 recess at the plate (the horn's plate fills the corners between the arms to r 3.76), hub hole
  +0.4, wall r 5.65 -> cut arms end 5.45..6.15 < the pocket's 6.2. Tested with the horn model: seats (0.012 mm3 = contact), arm slots
  0.24, hub 0.20. Lives in collection `print_tools` (in leg_v2 it was 300 mm away and made the duck "487 mm wide" in the labels).
- **Full rebuild** (legs, neck, head joints, head shell, ear fill, body, Uno Q, covers, wiring, cover recut) then every check. Found
  and fixed:
  - **left foot non-manifold** (108 edges, +1.5 g): the horn ear's lug started exactly in the horn plate's outer face (coplanar
    union) - clean until the wiring rebuild changed the boolean order. Lug face 0.05 inside; build_part can now report the
    manifold state per step (`PROBE["steps"]`, leg_v2 `foot_steps`).
  - **yaw block's assembly path** scraped 0.2 mm (also in the earlier runs - not caught then): going up 4.3 forward its roll boss
    stayed 0.18 behind the floor opening's rear edge. Now 4.8 forward (0.3 clear), front relief to 5.2 (0.3 from the beak servo's
    rails). ASSEMBLY.md updated.
  - **camera screws**: tip 0.3 before the pilot's end, pilot end 0.5 inside the face's front skin (audit limits 0.3 / 0.6) -> front
    wall 2.2 -> 2.5, pilot 0.4 past the tip, skin 0.7 (the face's front moves 0.3: duck depth 145.5 -> 145.8). A nut spacer was not
    possible: the heads have 0.3 to the dome.
  - **knee order**: driver access - with the leg on the body the knee cap screws and horn screw face the enclosure at 13-16 mm. The
    leg below the hip is now built off the robot and hung on the hip-roll horn last (path checked for every leg part, both sides).
  - IMU pilots (coincident with the rear wall's outer face, broke the enclosure) and the bay's countersinks: see "Joint limits,
    part 1".
- **Screw audit (94 heads), dispositions** of what it still flags (28): servo horn screws 13 (1.80-1.85 mm in the brass spline,
  probed; measure the kit screw); M3x8 axles 3 (tip 1.8 into the nylon nut by design - snug-only bearing axles); ear screws "bite
  2 parts" 5 (the servo's ear hole is drawn at the screw's 2.0: model artifact); Uno Q M3 posts 4 ("wall 0.15" is sampling; measured
  along the screw: >= 0.98 at the rear posts, 0.83 in the bottom 2 mm of the front posts, 1.35-1.87 above); the C2 test rig 1 (not
  part of the robot; the rig is removed from the scene).
- **Final pass after the fixes**: 27 printed parts one piece, watertight; horns 15/15; fits 333 (only the battery sampling artifact);
  all assembly paths (the left hip-yaw servo by its L-path: lift 7-8.5, from the rear or a side); screws as above, none new; head
  room 77/77; cables 29/29 reach (least slack 16 mm); motion: the 20 known limits + the both-hips poses outside the limit box.
- Printability (50 deg, supports per part in its best orientation): feet 436-438 mm2, head base 1719, dome 1937, thigh 678, shin
  523, face 466, body lid 594 - bridges are counted as unsupported; the head shell parts need supports.
- **Exploded view** regenerated (170 elements, the knee caps and their screws in build order along the knee axis; no stale copies);
  size labels: nanoduck H 237.5 x W 162.5 x D 145.8 mm. **Print files**: `exports/xgoduck_scs0009/print/` - 30 STL (mm) + manifest,
  310 g solid PLA (`tools/blender/export_print.py`, refuses non-closed parts).

## MuJoCo on the rebuilt parts; sit and stand (2026-11-13)

Model `sim/scene_scs0009_v2.xml` re-exported from the rebuilt parts (665.7 g, servo 0.1883 N m at 5 V, limits from
JOINT_LIMITS.md; trainnr bundle `xgoduck_scs0009_v2h`, `_v2i` with the new SIT). Recordings streamed into the trainnr Studio (the Studio's live Simulator
runs only its built-in rigs): `nanoduck_v2h_moves`, `nanoduck_v2h_sit_attempt` (the old SIT), `nanoduck_v2h_sit_stand`.

- **Moves that work**: stand hold (knees ~40 % of stall), crouch 0.3, side shift 0.12 rad, neck / head / head yaw / hip yaw within
  the limits. Crouch >= 0.4 needs the knee past its 68 deg limit.
- **The old SIT cannot work**: hip -30 / knee 60 / ankle 0 with the feet flat needs the trunk pitched ~90 deg; in the sim it falls on
  its face at 0.8 s (neck and head servos at 100 %). No blend of it, and no open-loop search around it, stayed up.
- **New SIT** (found by a quasi-static planner: feet flat, root pose solved from the feet, CoM vs the soles, static per-leg
  torques, MuJoCo self-collision): **hip pitch 70, knee 18, ankle -16** (left; right mirrored), hip roll / yaw, neck and head as
  STAND. The duck leans back ~35 deg and sets the rear of the electronics enclosure on the floor (two points 37 mm apart, 58 % of
  the weight; the next part, the servo adapter, is 6 mm off the floor; no screw head, switch or battery near it).
  - Straight line in joint space from STAND: the CoM stays >= 15.5 mm inside the soles all the way (STAND itself has 16.4).
  - MuJoCo, STAND -> SIT -> STAND: **108 / 108 variations pass** (move 1.5 / 2 / 3 s, friction 0.4 / 0.7 / 1.0, servo stiffness
    x1 / x0.5, trunk CoM -6 / 0 / +6 mm, trunk mass x1 / x1.15): no fall, no saturation, trunk down in every one, back to the
    same standing height. Peak servo load at 3 s per move: 61 % median, 69 % max (2 s: 67 / 74; 1.5 s: 74 / 80). **Use 3 s.**
  - servo_tests `sit_stand:3:2`: upright, peaks hip pitch 57 %, knee 40 %, tracking error <= 0.5 deg.
  - **Exact parts in Blender** (leg_motion_check, 21 poses along the path): no collision; control pose (hip +60, knee -90) does
    collide as expected. Smallest gap that closes along the path: shin to foot-plate screw heads 2.21 mm (2.54 at STAND).
  - Written to `sim/joint_ranges_v2.json` (keyframe override) and the SIT key in the scene (settled pose, `tools/sim/settle_key.py`).
- Not modelled: servo deadband / backlash / gear friction, real print mass (solid-PLA estimate - real parts lighter), floor other
  than flat and rigid. Check on the robot with the servos' overload protection set (JOINT_LIMITS.md part 3) and a hand ready.

### Sit/stand against what the plain model leaves out (2026-11-13, `tools/sim/sit_stand_robust.py`)

Added per trial: gear friction / viscous / rotor inertia (bracketing the BAM fits of the Dynamixel XL320 and XL330 - no SCS0009
fit exists), deadband 0-3 steps, backlash +-0..1 deg (passive hinge in series), command delay 0-20 ms, torque falling with speed,
kp x0.5-1.2, PLA fill 0.45-1.0 (573-743 g; the mass model was already at 0.75, not solid), bought-part mass +-15 %, trunk CoM
+-6 mm, floor friction 0.4-1.0, slopes +-5 deg pitch / roll, soft floor, bumps up to +-1.5 mm.

- **Feet slid during the sit** (old SIT, hip yaw 0 / roll -5): leaning back tilts the hip axes; each foot slid 5.8 mm out and
  twisted 2.5 deg, and where it could not slide back (bumps) the hip rolls fought at their limit for up to 0.8 s while standing
  up. **SIT hip yaw -3 / roll -4**: 2.5 mm, foot twist 0.3, roll 0.1 deg. More toe-in reaches the enclosure with the ankle
  screws (-4/-4: 0.7 mm; -5/-4.5 collides); -3/-4 is clear >= 1.58 mm along the whole path (exact parts, both legs).
- **Open loop** (servos only hold their angles): 66/300 random trials pass, 192 fall - about half of them while just STANDING:
  STAND has its CoM 8.5 mm behind the sole centre (16 mm to the heel edge), and backlash + deadband in the three pitch joints
  (2-3 deg of trunk lean together), an uphill slope (~8 mm at 5 deg) and a bump under a heel use that up. One factor at a time,
  nothing falls.
- **With IMU balance** (ankle correction -1.5 x trunk pitch error vs the planned pitch; the hip-roll term did not help; -2.0
  oscillates): **245/300 pass, 4 fall** (all facing uphill 4.4-4.9 deg with slack servos). Other failures: 13 servos pinned
  0.1-0.54 s (hip pitch / ankle, standing up); 49 feet "slid" >= 5 mm - MuJoCo contact creep on slopes (the noslip solver removed
  it on the pure-slope case, 7.9 -> 1.9 mm, but is too slow on bumps to run for all 300). 86 % of trials touch 100 % load for an
  instant (the feedback through the delay); without balance the nominal peak was 65 %.
- Consequences for the firmware: the sit/stand needs the IMU ankle balance; do not sit facing uphill on more than ~4 deg (the
  IMU sees the slope at STAND - refuse the sit); keep the overload protection on. Still not modelled: real SCS0009 values (measure
  backlash, deadband register, P gain, delay on a servo), sensor noise and IMU filtering.
- **Balance smoothing**: the plain correction chattered (pitch error flipping sign by 0.1-0.4 deg, the stiff servos saturate at
  0.83 deg error) - the nominal run with balance touched 100 % where open loop peaks at 56 %. Tilt error low-passed 25 ms with a
  0.2 deg dead zone: nominal peak back to 56 %; 300 random trials **255 pass, 1 fall** (uphill 4.4 deg, 1 deg backlash, half-stiff
  servos, 11 ms delay, bumps), 6 pinned 0.1-0.36 s (hip pitch / ankle, standing up), 42 "slid" (creep artifact as above); 86/300
  touch 100 % for an instant (unfiltered: 245 pass, 4 fall, 13 pinned, 86 % touch 100 %). Longer filters (50-150 ms) or a rate
  limit lag too much and fall more. **Trade-off**: at the 8 stacked worst-case corners (every servo extreme at once - 20 ms delay,
  1 deg backlash, 3-step deadband, half stiffness - plus +-5 deg slopes) the smoothed version falls in 6/8, the unfiltered in 2/8.
  Tune the filter once the real servo's delay / backlash are measured.

## Build readiness pass (2026-11-14)

Everything re-run on the current files (joints at rest before and after every step):
- **MuJoCo, held in the air**: each of the 14 servos alone over its full servo range: 8-29 % of stall, never at the limit, lag
  <= 0.1 deg. (The beak servo is not in the sim; the jaw is 6.7 g.)
- **MuJoCo, on the floor** (ideal model): stand 40 %, crouch 0.3 50 %, shift +-0.12, neck -57 deg 62 %, neck +28.6, head pitch
  -4.6 / +65, head yaw +-149, head roll +-15, hip yaw in / out both legs, sit/stand 52 %: all upright.
- **Exact parts in Blender** (`leg_motion_check`, new `ranges_deg` option = the servo limits from joint_ranges_v2.json): every joint
  at 9 points over its servo range + 296 poses sampled every 0.25 s from the 17 motions above: **0 collisions**. (Earlier sweeps used
  the joint empties' stored, wider ranges - their "known limits" were outside the servo limits.)
- Printed parts 27/27 one piece, watertight; horns 15/15 (same gaps as before); fits 333 (battery sampling artifact only);
  assembly paths all OK (left hip-yaw servo by its L-path); screws 89, 27 flagged, none new (cam screws and the removed C2 rig no
  longer flagged); printability unchanged.
- **Head / beak**: head roll sweep clear; beak 0..35 deg open clear; closing past 0 hits the head base from 2 deg -> **beak servo
  limit 0..+35** (added to JOINT_LIMITS.md). head_check lists base x dome, base x roll servo, base x beak servo as "not intended"
  rest contacts; the boolean gives slivers 0-0.08 mm thick (mating faces) and a 9-ray majority vote finds no vertex of either part
  inside the other: seated surfaces, not interference.
- Not covered by any of this: real-part measurements, printer tolerance, firmware (limits, interlocks, IMU balance, overload
  settings), walking, getting up (torque, see the comparison with the original), fall loads, cables against moving parts.

## Power switch: the lever was never modelled (2026-11-14)

The MTS-102 stand-in was body + pins only, so "lever tip flush with the outside" (2026-11-12) was never checked. Listings: M6 x 0.75
bushing, 9 mm thread, overall 29-33 mm -> lever tip ~15.5-19.5 mm from the body face, i.e. ~4-8 mm proud of the left wall (the body
face is 11.6 mm in). The left leg passes that spot: knee cap 4.8 mm from the wall face during sit/stand, ankle screw 2.2 mm at hip
pitch +85 -> at most ~2 mm may stand proud there. Recessing the lever fully needs ~18 mm depth and ~22 mm height (lever throw about
+-22 deg); the room between the IMU (x <= -42.4), battery (x >= -28.9), servo adapter (z <= 100.9) and regulator (z >= 119.5) is
13.5 x 18.6 mm. **User (2026-11-14): keep the MTS-102 where it is, sticking out.** Clearance to the lever's real envelope (Ø4,
z 99..117 for the full throw): sit/stand 6.2 / 4.4 / 2.9 / 2.3 mm at 2 / 4 / 6 / 8 mm proud; single-joint hip pitch +85 2.4 / 0.4 / 0 / 0.
So <= 4 mm proud needs nothing; more needs a hip-pitch-back servo limit of ~+60 / -60 (to be set from the measured lever).

Also found while measuring: 4 of 20 hip corners (yaw -6.3 with roll -12 or +19.5 and hip pitch +45 / +85; yaw +19 / roll +19.5 /
pitch +85) collide with the bay / lid / cover inside the servo limits. Roll-out with the leg back is the known interlock (REVIEW
"Power switch" section, 2026-11-12); **roll-in -12 with pitch +45 (foot bearing plate into the adapter / enclosure, 122 mm3) is not
in JOINT_LIMITS.md yet** - the interlock table needs a hip roll x hip pitch rule. The readiness pass covered single joints and the
17 planned motions, not these combinations.

## Hip map: roll x pitch x yaw x knee (2026-11-14)

1,248 left-leg poses on the exact parts (pitch -90..+85, roll -12..+19.5, yaw -6.3 / 0 / +19, knee -90 / -45 / 0 / +68, ankle 0): 245
collide - the knee cap, shin and foot bearing plate on the enclosure / body frame / lid / Uno Q cover with the leg swung back, the
foot on the Uno Q cover with the leg far forward and toed out. Leg forward to -45: nothing collides. Toed in (-6.3) the leg cannot go
back past ~+15 at any roll. The right leg matches the mirrored left in 40/40 sampled poses. Table and firmware rule in
JOINT_LIMITS.md ("Hip map"), data in data/hip_collision_map.json. Not covered: ankle angles, both legs together at large pitch.

## Independent review fixes (2026-11-15)

An independent review of `main` at 91b780c (microduck-nano session, `docs/project/design-review-2026-11-14.md` in that repo, scripts
in its `scripts/design-review/2026-11-14/`) re-ran the checks with its own tools. Most numbers reproduced; it found the items below.
Battery protection / clearances and the regulator's placement are deferred by the user and not touched.

**New tool: `tools/blender/clearance_sweep.py`.** `leg_motion_check.py` judges by intersection VOLUME > 0.05 mm3, which passes thin
touches. The new sweep tests every pair of parts on different bodies for surfaces crossing (depth, from points inside the other
part) and for gaps under a margin (point-to-surface from every vertex, triangle centre and points 0.5 mm apart over large triangles);
pairs already that close at rest (bearings on bosses, servos in holders: 46) are not judged. It runs headless
(`blender -b <blend> --python tools/blender/clearance_sweep.py -- args.json`), ~1.5 s a pose. **Its first version tested only
triangles lying wholly inside the other part's box, so the large flat faces of printed parts were skipped: it missed 20 of the
2026-11-14 map's collisions.** Fixed (triangles whose box meets the other's, plus face sampling) and re-run; it now reproduces all
245 and the review's head finding to the 0.01 mm. Gap readings are upper bounds (two edges passing can read up to ~0.25 mm large):
margins below 0.3 mm are not meaningful.

1. **Head roll x yaw** (review: FAIL). Inside the allowed zone (roll within +-2.5 deg of yaw 0) the head base cut 0.16-0.40 mm into
   the head-pitch U at yaw -2..-2.5 / roll -10..-15. Re-measured on a 0.25 deg x 1 deg grid: +-2.5 -> 28 poses collide; +-1.5 -> none,
   gap 0.07; **+-1.0 -> none, gap >= 0.31 mm**; +-0.5 -> >= 0.5. Interlock narrowed to +-1.0 (keeps the user's software-interlock
   decision; the microduck-nano firmware already uses +-1.0). JOINT_LIMITS.md.
2. **Right leg is not the left's mirror** (review: FAIL, 30 poses). Swept the right leg's own 1,248 poses: 30 collide where the mirrored
   left pose is clear, none the other way - forward 45-90 deg with the knee at its limit, right foot into the neck bearing plate and
   body frame, up to 6.9 mm (the 2026-11-14 spot check of 40 poses missed them). The body is not symmetric there (neck plate, Uno Q
   cover screws). JOINT_LIMITS.md now has one table per leg; data/hip_collision_map.json has `right_poses`.
3. **Left map with a margin.** All 245 reproduce; 8 more collide (0.06-0.70 mm: knee cap into the hip-yaw servo with the leg back
   75-85 and rolled out +10; thin touches on the enclosure), 19 more clear by < 0.3 mm. Table regenerated at a 0.3 mm margin
   (`tools/blender/hip_map_table.py`); cells that changed: toe-in at -90 forward loses +15, at +15 back loses 0, at +30 back loses -8;
   toe-out at +85 back loses +10; a few folded-knee entries are tighter (yaw -6.3 at 0, yaw 0 at +30, yaw +19 at +45).
   Also found: at the hip-roll servo limit +19.5 the hip-yaw bearing is 0.29-0.30 mm from the L-bracket in every pose (both legs) -
   not touching, but under the 0.5 mm JOINT_LIMITS aims for. Recorded in the servo-limit row; limit unchanged.
4. **Both legs together** (review: ankle axles touch 0.2 mm with hips rolled in -12, legs forward). Swept 2,496 two-leg poses (mirror-
   symmetric, and one leg forward / one back): 39 collide leg against leg, up to 8 mm, all symmetric. Toe-out +19 with roll -12..+5
   and both legs forward of -30: the feet / foot bearing plates meet - inside each leg's own map, so a new interlock (never both legs
   forward of -30 while both are toed out). Toe-in with roll out and the legs back also collides, but each leg's map already excludes
   it. One forward / one back: clear. The review's exact touch (yaw 0, roll -12, forward) did not reproduce on this grid (ankle 0).
5. **Uno Q screw vs header** (review: 0.69 mm). Datasheet sec. 12: the JDIGITAL housing starts **2.05 mm** from the (15.24, 50.80)
   hole's centre, so any M3 head (dk 5.5) overlaps it by 0.70 mm - real, not a model error. **User (2026-11-15): one M2 x 6 socket
   (BOM row 19, dk 3.7) at that hole.** `body_v2.py`: that post's pilot Ø2.5 -> PILOT_D 1.7, new dummy `uno_screw_M2x6`; head to header
   0.16 mm in the model (Ø3.8 head; ~0.21 with the real 3.7), 0.94 to the cover. Rebuilt by the documented order (build_args/README:
   body, uno_q, wiring, cover recut) after checking on a copy that the order reproduces today's model exactly (152 objects; uno_cover
   +4 vertices, same volume). Lid one piece, watertight, 11.16 g (was 11.14). BOM rows 19 (35 -> 36) and 22 (8 -> 7).
6. **BOM stale text**: coupon now "designed"; switch lever "stands out of the wall" (was "flush").
7. **Duplicate horn screws** (14 joints had two stand-ins 1.1 mm apart): the servo builder's own `scs_<joint>_horn_screw` sat on the
   horn, the joint library's `<joint>_screw_M2x4kit` 1.1 mm higher on the printed plate (where the real screw is). The 14 servo-builder
   copies were deleted (the beak keeps its only one); `refresh_servos.py` now passes `horn_screw: false` where the joint has its own.
   The mass model counted both (14 x 0.1 g): body_mass not re-run for 1.4 g.

Not redone: exploded views (their copies still show four M3 on the Uno Q; regenerate with build_args/README steps 6-10); STL exports
(re-export the lid); the sim's body masses (sim/body_mass_v2.json, < 2 g change).

## Cables, ankle angles, knee x ankle; a false inside test (2026-11-15, later)

**Tool fix (both new tools).** The inside test (ray parity, 2 of 3 rays) was fooled at single points by the exact servo meshes:
the first cable check reported the left body-to-thigh extension 4.33 mm inside the left hip-roll servo at hip pitch +85 from ONE
sample whose 1 mm neighbours were outside; measured directly in Blender the span stays outside (closest: yaw connector 0.49,
hip-yaw bearing 0.38). A point now also needs its nearest face to face away from it. Re-validated `clearance_sweep.py` on 546
earlier poses (60 left / 58 right / 80 both-legs / 319 head collisions + 30 clear): every collision still found, none new, a few
depths moved by < 1.2 mm - the hip maps and head result above stand.

**Cable reach** (`wiring.py` `"motion": true`, 211 poses): every cable long enough; tightest the left body-to-thigh extension, 224
of 250 mm at hip pitch +85.

**Cables against moving parts** (new `tools/blender/cable_pinch_check.py`, 212 poses inside the servo limits: each joint alone,
STAND / SIT, walking-range combinations, neck + head extremes). Each cable span is taken as the straight line between its two
waypoints (the least cable, as in the reach check); points inside a part deeper than 0.5 mm, beyond what they were at rest, are
reported. A real cable has slack and bends round the part: these are where a part PUSHES a cable (chafe / pinch to look at on the
build), not cuts. 21 found:
- **Walking (repeated every step):** both hip-roll leads and both body-to-thigh extensions pressed 2.3 mm into the body frame's
  edge with the leg back 46, toed in, rolled out. Chafe risk - wants a clip / guide or a radius on that edge.
- **Head turned 75-125 deg:** the U-to-head extension (2.6 mm into the head-pitch U) and the camera USB (2.5 into the yaw block,
  1.2 into the U, 1.1 into the head base) are dragged across the parts. Head pitch up 59: camera USB 1.7 into the neck cradle.
- **Ankle / knee near their limits:** ankle leads 0.6-1.5 mm into the foot / shin. Neck back -80: gap-to-neck extension 1.3 into
  the body frame. Others 0.5-0.7 mm at the extremes.
Not changed (needs a routing decision / clips on the real build).

**Ankle angles.** (a) Knee x ankle at hip rest (13 x 15 poses each leg): with the knee bent past +30 the foot meets the L-bracket and
thigh unless the ankle stays above a limit that rises from -60 (knee 40) to 0 (knee 68) - up to 6.7 mm at knee 68 / ankle -23. Same
for both legs; walking (knee <= 55, ankle +-23) and SIT are inside. New interlock row in JOINT_LIMITS.md. At the ankle's servo limit
-86 the foot is 0.17 mm from the thigh (any knee <= 30). (b) The hip grid again with the ankle at -23 / +23 (2,496 poses each leg):
apart from (a), ankle +23 adds 6 (left) / 7 (right) collisions, 0.3-1.6 mm, foot on the enclosure / lid / Uno Q cover with the leg
back or folded; listed in JOINT_LIMITS.md. Data in `data/hip_collision_map.json` (`ankle_pm23_*`, `knee_ankle_*`).

**Housekeeping:** exploded views regenerated (build_args/README steps 6-10; they now show the Uno Q M2 screw); all 28 printed-part
STLs re-exported (the two optional thigh covers unchanged).

## Final changes and full check; design closed (2026-11-15)

Full check of screws, assembly, fits, motion and wiring on the current model, then the last changes (user's choices):
- **Deck edge for the hip cables** (user: round + sleeve). The walking chafe point is the deck plate's LOWER OUTER edge (not the
  cap's top, as first said): from the yaw connector under the deck the hip-roll lead and the body-to-thigh extension rise to the
  thigh beside the deck and wrap round that edge with the leg back. `body_v2.py` rounds it r 1.5 over x -4.9..11.2 (where the outer
  face reaches the bottom), both sides: one prism cutter (a fan of separate wedges broke the exact boolean - caught on the test
  copy: the frame came out as a 13.7 mm3 fragment). body_frame one piece, watertight, 19.16 g (-15.3 mm3, as computed); the old
  corner is now 0.62 mm from the surface (r 1.5). The pinch check still reports these spans 2.3 mm into the deck: it models the
  cable as a straight line, so it cannot see the round; the cable now bends over r 1.5 instead of 90 deg, inside a sleeve.
- **Head cables** (user: sleeve + slack loop, head yaw keeps +-150): ASSEMBLY.md step 4.
- **Uno Q cover** (user: widen): each M3 head overhangs the board edge by 0.21 mm and sat 0.09 mm from the cover's skirt; a Ø6.5
  relief round each of the 3 M3 heads, from 0.6 under the board's top: gap 0.49 (0.5 less the facets). Cover one piece, 8.61 g.
- BOM row 13b: 6 mm braided expandable sleeve, 1 m (uses ~30 cm).
- Wiring cross-check: 13 kit hubs all 3 sockets used, 12 x 165 + 1 x 250 mm extensions, 15 leads, the adapter's 2 sockets, no
  loops - matches the BOM.

Full pass on the saved model (now `scripts/full_check.sh`, arguments in `data/check_args/`, README there with the expected results):
rest pose OK; 27 printed parts one piece; printability unchanged; horns 15/15; fits 367 (2 known sampling artifacts - battery
0.30 mm3, left hip-pitch servo 0.11 mm3 = 0.002 by exact boolean); assembly 55 paths OK + the left hip-yaw servo's L-path + the 3
segment paths; screws 89, 26 flagged, all previously dispositioned (the M3 that was flagged by the header is now the M2, wall 1.62);
motion sweep unchanged; cable reach 0 short; independent all-pairs check (microduck-nano static.py): no print-print > 0.02 mm, no
servo-print > 0.06 mm.

**Design closed** for the first build, with these open by the user's decision or by needing real parts: battery protection and
clearances, regulator placement (deferred); everything marked "measure on arrival" (horn, kit screws, servo steps/deg and
protection registers, switch lever, battery thickness, IMU holes); walking (one-foot balance fails - gait study) and get-up; fall
loads. Changes after this go through the coupon / first-leg build.

## Final review before the build: limits closed, bearing steps (2026-11-18)

User: "review the design and final assembly and make sure it all works and fits perfectly ... close the design and limits".

- **Full check suite** (`scripts/full_check.sh`, saved .blend of 2026-11-15) - every result as in `data/check_args/README.md`:
  rest pose OK; 27 printed parts one piece; printability unchanged; horns 15/15; fits 367, 2 known (battery stand-in 0.30 mm3,
  left hip-pitch servo 0.11 mm3 sampling); assembly 55 OK + the left hip-yaw servo's L-path (sides from 6.5 mm of lift, rear from
  7.5, to 8.5) + 3 segment paths OK; screws 89 / 26 flagged (dispositioned); motion sweep 28 colliding poses, all outside the servo
  limits (the model's stored wider ranges, walking-envelope corners, both hips at roll +-22 - as before); cable reach 0 short;
  cable pinch 21 (handled: sleeve, rounded deck edge, slack loop).
- **Parts list vs model:** 10 x 6702ZZ, 4 x 693ZZ, 4 M3 x 8 axles + 4 nylon nuts in the model = BOM rows 17 / 18 / 22 / 22b.
- **Limits:** each joint alone at both ends of its servo limit (`clearance_sweep.py`, margin 0.8): two ends were under the 0.5 mm
  the limits promise. Hip roll outward (hip-yaw bearing to L-bracket, the same at any pitch / yaw): 0.68 mm at 18, 0.56 at 18.5,
  0.43 at 19, 0.30 at 19.5 -> limit **+18.5** (right -18.5). Ankle (foot to thigh, knee 0..30): 0.65 at -85, 0.17 at -86 -> limit
  **-85** (right +85). After: every end >= 0.5 (hip roll 0.56, ankle 0.50-0.51, head yaw -150 0.73, the rest nothing within 0.8).
  No motion uses the degree removed (walking roll +-10.9, ankle +-23; SIT ankle -16).
- **Limits as data:** `sim/joint_ranges_v2.json` gets `status` (closed), the beak (0..35) and an `interlocks` block (head roll x
  yaw +-1.0, knee x ankle table, both legs, hip maps, sit / stand path); MJCF v2 ranges updated by `tools/sim/set_joint_ranges.py`
  (its rewrite of the scene's settled SIT key was reverted). The Blender joint empties keep their wider `joint_range`: the cover
  recut builds its swept space from them, and narrowing them would change geometry on the next rebuild.
- **ASSEMBLY.md:** only 5 of the 10 6702ZZ had a step. From the joint library (type C: the bearing sits in the part that holds the
  servo, seat open away from it, lip on the servo side): hip roll -> yaw connector, hip pitch -> thigh, head yaw -> yaw block (plus
  the hip-yaw pair in the body frame, head roll in the head base, knees in the shins; 693ZZ in the bearing plates). New section 0
  item 5: all 14, pressed on the outer ring before any servo goes in. Uno Q screws corrected (3 x M3 x 8 + 1 x M2 x 6, since
  2026-11-15).
- **BOM:** the user's bearing listings (6702ZZ Amazon.in 10-pack Rs 1,731; 693ZZ 4-pack - a spare pack advised).

**Design and limits closed** for the first build. Still open (user's / needs parts): battery protection, regulator placement,
everything "measure on arrival", walking gait and get-up studies, fall loads.

## Battery protection and regulator placement (2026-11-18)

The independent review (2026-11-14) called the power design unsafe for children: no fuse, no protection board, no low-voltage
cut-off; regulator 0.2 mm from PLA and 3.6 mm from the LiPo in a closed bay; a failed regulator puts 12.6 V on the 15 servos.

- **Measured** (free-space map of the bay, 1 mm voxels, all parts and modelled wires): the bay has no room for a 52 mm protection
  board except standing behind the battery (x -33.1..-29.7, y -112..-96, z 67.5..117.5: adapter 0.47, switch 0.50, battery 0.80,
  wall 0.88, regulator 2.0 mm; 4 modelled wires to reroute); an INA219 monitor (25.6 x 20.3 x 4.5) fits nowhere (under the Uno Q
  4.4 mm); an inline blade-fuse holder does not fit. Servo current (SCS0009 datasheet): 0.3-0.4 A loaded, 0.8 A stalled at 4.8 V
  -> ~2-3 A at 5 V standing, the Mini560's continuous rating without cooling; ~1-1.5 W of heat. The servos turn their torque off
  above 9 V (datasheet), so a failed regulator stops them moving but may damage them.
- **User decision:** "keep the most simple and effective assembly", no component that is not essential -> **no fuse, protection
  board, clamp diode or monitor**. The board position above is recorded (ASSEMBLY.md section 5) for later.
- **Regulator (printed parts only):** centred between the walls (1.2 / 1.2 mm, was 0.1 / 2.1) and a 15 x 26 mm **window in the lid
  plate** over it (board less 1.5 mm each side; `body_v2.py` REG_WIN): its component side faces the air under the Uno Q. Mounted
  component side up between the rails. Lid 10.2 g (was 11.1), one piece, watertight. Rebuilt in order (body, Uno Q, wiring, cover
  recut); print STLs re-exported (28, none refused).
- **Uno Q battery plug** (JANALOG 7 GND / 8 VIN, next to A0): marked + hot-glued (ASSEMBLY.md step 1.5) - shifted one pin it
  destroys the MCU or shorts the battery.
- **Rules instead of electronics** (ASSEMBLY.md section 5): switch off after use; short sessions; cell check before every charge
  (BOM row 25 checker + bag now needed, as tools); never charge a cell under 3.0 V; swelling check; first power-up heat test of the
  regulator (finger test, ~60 C).
- **Full check suite after the change:** all as expected (`data/check_args/README.md`): fits 367 (the 2 known; regulator 1.20 mm
  from the enclosure, 0.20 from the lid rim), assembly 55 + paths, screws 89 / 26, motion 28 (outside the limits), cable reach 0
  short, cable pinch 20 (was 21). Exploded views regenerated (build steps 6-10), renders `data/duck_exploded.png` and
  `data/bay_exploded.png` redone; the coupon / trim-jig collections hidden again. **Design closed for the hand-off.**

## The bay with exact parts, screws and wiring (2026-11-19)

User: "look at the bay assembly with exact wiring, headers, cables ... each component designed exact dimensions and shapes and screws,
holes"; then the real parts: battery 60 x 30 x 20 with an XT30 (listing + photo), the Waveshare Bus Servo Adapter (A) (42 x 33, M2.5 at
37 x 28; spec "9-12.6 V for ST servos"), the 7Semi BNO086 (27.89 x 20.87), the Mini560 maker's drawing (30 x 18, pad pairs 26 apart),
a rigid right-angle USB-C (male) -> USB-A (female) adapter, and the camera's own cable (USB-A -> JST-SH).

- **Battery 3 mm taller than designed** (60 vs 57): its top met the regulator and left no room for its 14 AWG leads. No inside spot for
  the regulator and the switch together (free-space maps, all orientations). User: **raise the lid + Uno Q 3 mm** (`body_v2.py`
  `pocket_top` 128; `body_v2.json` battery [60, 30, 20]). Neck x head pitch x head yaw re-swept (390 poses, exact parts): nothing under
  0.5 mm inside the limits; outside them (neck +40) the hub plug on the U now meets the Uno Q's headers (5 poses, all beyond +30).
- **Regulator** in a slot under the lid (rails + 0.8 mm ledges under its long edges), component side up, its parts through a window of
  its full outline (30 x 18 + 0.2: its parts run to the long edges): ~8 mm over the battery for the leads, its hot side in the air under
  the Uno Q. Slid in on the bench. Pads at the drawing's positions ("+" toward the rear, IN at the -y end: check the board's labels).
- **XT30 + balance plug outside, rear-right cradle** (user): inside, no spot left the XT30's wires room to bend (searched all positions
  and orientations with 4 mm wire room at both ends). The battery's leads and balance lead leave through the open rear-right window;
  the robot's 14 AWG pigtail loops up behind the XT30 and back in. Unplug the battery and charge it from behind, nothing to open.
  The battery's leads need ~62 mm (measure; else a 10 cm XT30 extension).
- **Camera USB** (user's rigid right-angle adapter): mouth down, the camera's USB-A plug lying in a **rear-left cradle** (ribs + zip
  tie), so a knock goes into the bay, not the Uno Q's only USB-C socket (a straight adapter + plug stood 36.5 mm off the back).
- **Switch**: its M6 nut + washer did not fit the 7 x 8 recess and no tool reached it; now a **hex recess** holds the nut captive and
  the switch is screwed in from inside. Moved 1.2 mm forward (IMU).
- **Uno Q plugs**: its JDIGITAL / JANALOG headers are FEMALE: right-angle 2.54 mm male header plugs, 3 pins on JANALOG 6-7-8 (UART GND,
  GND, VIN) and 2 on D0 / D1; the cover's notches widened / raised for them.
- **Wiring** (`wiring.py`, through the connectors' anchors from the new `parts/bay_parts.py`): every bay wire at its real size and
  every one checked against every part - all clear (tightest 0.26 mm). Routes: battery leads / balance lead / pigtail through the
  rear-right window; 5 V over the IMU and down the 3 mm gap behind it to the adapter's back pins; UART up behind the IMU and out of
  the left-rear window along the left wall; Uno Q power, UART ground and Qwiic out of the right window, forward under the lid's edge.
  Found on the way: the lid's tray walls (0.5 mm under the board) leave no way out from under the Uno Q; the Uno Q's M3 screw tip pokes
  3 mm under the lid at the right-rear corner; stale `wiring_mesh` objects (an old layout) were hidden. The adapter's 3 front UART
  pins point at the battery: tape them.
- **Screws** (screw audit + driver access, bay): bay -> body 3 x M2 x 10 CSK, 7.6 mm bite, driver through the rear wall before the
  battery; lid 4 x M2 x 6, 4.5-4.7 mm, before the Uno Q; IMU 4 x M2 x 6, 4.4-4.5 mm; Uno Q 3 x M3 x 8 + 1 x M2 x 6; cover 2 x M2 x 6.
- **Legs vs the new outside parts**: 450 poses with a leg swung back (hip pitch 30-85, knee -90..68, yaw, roll): 49 of the 50 poses
  that touch a new part are already forbidden by the hip map; the one allowed (left leg +85, knee 0) leaves the switch lever 0.61 mm
  from the foot's bearing plate (BOM row 9: a lever > 4 mm proud -> hip pitch back ~+60).
- Exploded, labelled bay sheet: `data/bay_assembly_sheet.png` (`tools/blender/sheet_labels.py`, `data/build_args/bay_labels.json`).

### Camera cable in the head; rear cradles re-placed; full re-check (2026-11-19, later)

- **Camera cable in the head** (user: "fix the camera cable in the head"): the old route ran through the U, the head base, the dome
  and the face (up to 2.3 mm). A path search (`tools/blender/cable_path_search.py`: A* on a 1 mm map of the head, clearance 1.7 mm for the Ø3.2 cable,
  the jaw closed AND fully open) gives the round cable a clear way: from the U out beside the floor opening on the right, up the head's
  right side, over the jaw (49 mm in the head). It showed two real faults in the head: (1) the camera's JST plug leaves only 0.3 mm
  before the brow band's end wall - its 4 wires had no room to turn; (2) the plug's wire pocket was walled off from the head (dome edge
  behind it, a 1 mm face strip beside the board's end, the face's D-panel): the camera could not have been cabled. Fixed in
  `head_shell_v2.py` (face only; the dome is unchanged): `plug_room` 1.5 (band 1.2 longer at the right: 1.1 mm past the dome's outline),
  the board's slot on past its end, and a 7 x 4 slot in the D-panel (hidden behind the band). The lead (3.6 x 0.9, its width kept
  vertical) and the cable now clear every head part (lead >= 0.04 mm, cable touching the head base at 0.0). Found on the way: re-running
  the face re-cut the dome's camera-screw recess each time and broke the dome into 4 pieces - restored from main and the cut now runs once.
- **Full suite on the new bay** found: the regulator's parts in the Uno Q's underside envelope (the envelope said 3 mm; the datasheet
  says < 2 mm: envelope corrected, regulator 0.8 mm lower -> 1.25 mm clear); 1 cable "too short" (the battery leads vs the 60 mm assumed);
  the right foot (leg back 45, knee -45, toe in: hip-map ALLOWED) reaching the XT30 cradle (2.05 mm). Fixed: XT30 to the middle of the
  rear face (the foot reaches y < -111.2 up to z 108.9 only), balance plug above the bay at the rear right (mouth up, behind the Uno Q
  cover, on a fin). The battery's main leads must reach ~75 mm (BOM row 4).

## Whole-robot assembly check: what could not be built, fixed (2026-11-20 / 21)

An assembly animation of the whole robot (`robot_assembly_seq.py` -> `assembly_anim.py`) whose every move is path-checked on the real
geometry (`anim_to_paths.py` -> `assembly_path_check.py`, 1 mm steps, hits confirmed by volume). The bay's moves had never been checked
(the 55 "assembly paths" are servos, legs and head). Found and fixed:

- **Servo adapter could not go into the bay**: the switch's mounting block (part of the enclosure, 11.6 mm deep for a flush lever)
  hung 1.1 mm over it; the pocket's floor ledge and the left-rear lid post (+ corbel) also stood over its left edge. Now: the block is a
  separate **switch plate** (built on the bench: nut, switch, wires; in the bay down 9.5 mm in from the wall, then out onto it;
  2x M2x10 countersunk from outside, heads flush; lever still 7.4 mm out), notches in the ledge, the post 2.6 mm forward.
- **Balance-plug fin** ran 0.1 mm into the lid and the Uno Q board and through the cover's rear wall (20.7 mm3), joined to the wall by a
  sliver: reshaped (0.1 / 0.3 / 0.3 clear, joint 1.6 x 5.3 mm above the battery leads). The fit audit had no such pairs: 32 added.
- **Screws from the outside** (user): bay-to-body 3x M2x10 countersunk now go **up from under the robot** (were driven inside the bay
  through access holes in its rear wall); the **IMU sits on 4 pegs** + hot glue (4 screws fewer).
- **Camera could not be fitted into the face** (closed brow band, panel 2 mm behind the board): the band's front wall is a separate
  **visor**; the board goes in from the front; 3x M2x10 countersunk from the front - the right one into a boss behind the board, the
  left pair (the dome's thin wall leaves no room for a boss) into **2 M2 nuts** in a pocket, their heads on a 1 mm raised pad so the
  tips stop short of the dome (`data/face_camera_section.png`).
- **Dome could not come down**: the roll servo's right ear-screw head was recessed into the rear-right post's side. The rear dome
  screws moved x 5.0 -> 1.5 (dome posts, base bosses + webs, holes through the base's rim; `parts/head_rear_screws.py`, in place - the
  full base/dome rebuild from the original meshes is broken, see data/build_args/README.md).
- **Assembly order** (ASSEMBLY.md): the left hip-yaw servo needs a ~10 mm lift (the old "7.5" came from a check that moved the case
  without its spline - `checks/hip_yaw_servo_path.py` now moves the whole servo); **both hip-roll servos before the bay** (the battery
  is in their way after); **all four M3 nylon nuts into their pockets before the servo next to them** (neck, head pitch, ankles).

Checks on the result: full suite - fits 399 / 1 known (+ the face under the dome's edge, 0.23 mm3, fixed after the run: no crossing),
printed parts 29 one piece, bay moves 42/42, screws 87 / 28 flagged (26 known + the visor's left pair: full engagement in metal nuts),
motion 35, cable reach 1 (battery leads ~75 mm, BOM), pinch 21; the whole-robot sequence 458/458 segments OK.
