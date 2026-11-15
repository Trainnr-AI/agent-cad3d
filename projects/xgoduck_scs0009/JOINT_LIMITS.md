# nanoduck (XGO-Duck SCS0009 v2): joint limits

Measured in the model 2026-11-13 (`tools/blender/leg_motion_check.py` exact intersection volumes, then minimum gaps at the limit
corners with every coupled joint moving). Angles in degrees from each joint's zero (the robot's rest pose); left/right signs as
in the MJCF. Decided with the user 2026-11-13. **Updated 2026-11-15** after the independent review of 2026-11-14 (microduck-nano
`docs/project/design-review-2026-11-14.md`): head roll x yaw narrowed, hip map re-measured with a clearance margin for each leg
separately, both-legs interlock added (`tools/blender/clearance_sweep.py`; REVIEW.md "Independent review fixes").

Three layers, because a servo's own angle limit acts on ONE joint and most collisions are combinations:

1. **Servo angle limits** (stored in each servo: Min/Max Angle Limit, SCS/SCSCL registers 9 and 11, 2 bytes, 0..1023 steps).
   Inside them every combination of the coupled joints listed is clear by >= 0.5 mm.
2. **Software interlocks** for combinations a per-joint limit cannot block (table below). The gait / behaviour code must keep to
   them.
3. **Servo overload protection** on every servo, the last line if 1 or 2 fails (a stalled servo overheats, strips its gears or
   breaks a part): Unload Condition (reg 19) bit 5 = load overload, bit 2 = overheat; Overload Torque Threshold, Protection Time,
   Holding Torque; Max Temperature Limit (reg 13, default 70 C). Values to be set and read back on a real SCS0009 - the public
   SCSCL table has inconsistent addresses for two of these.

Converting degrees to servo steps needs each servo's zero (horn fitted at the middle of its range, ASSEMBLY.md step 0.3) and
direction. The SCS protocol works in 1024 steps over 300 deg (docs/HANDOFF.md) = **3.41 steps/deg**: confirm on a servo (read
the position at two known angles) before writing limits. Example: a joint zero at step 512, limit +19 deg -> 512 + 65 = 577.

## 1. Servo angle limits

| Joint | Model range | **Servo limit** | Why | Measured |
|---|---|---|---|---|
| hip yaw L / R | -12.6..+25.8 / -25.8..+12.6 | **L -6.3..+19, R -19..+6.3** (+ = foot outward) | outward 25.8: the two L-brackets and yaw connectors collide (today's parts); inward + roll inward: knee caps vs enclosure | box corners 0.57-2.66 mm |
| hip roll L / R | +-22 | **L -12..+19.5, R -19.5..+12** (- = feet together) | roll -22: the feet collide at any yaw | (with hip yaw, as above); at +19.5 the hip-yaw bearing is **0.29-0.30 mm** from the L-bracket (2026-11-15, point-to-surface; under the 0.5 aimed for, not touching) |
| hip pitch L / R | -90..+85 / -85..+90 | unchanged | alone: clear over the range | 5 deg sweep |
| knee L / R | -90..+68 / -68..+90 | unchanged | alone: clear | 5 deg sweep |
| ankle L / R | -86..+90 / -90..+86 | unchanged | alone: clear | 5 deg sweep; 2026-11-15: at -86 the foot is **0.17 mm** from the thigh, at +90 0.50 from the shin (any knee <= 30; not touching, under the 0.5 aimed for) |
| neck pitch | -80..+40 | **-80..+30** (user: option A) | +40 alone: head base vs Uno Q cover 71 mm3 | corners >= 1.36 mm |
| head pitch | -45..+65 | **-5..+65** (user: option A) | neck +10..+40 with head -45..-5: U / bearing plate vs head base | corners >= 1.36 mm |
| head yaw | -150..+150 | **+-150** | alone: clear | 5 deg sweep |
| head roll | -25..+25 | **+-15** + software interlock (user) | yawed, the U meets the head's floor | grid; 2026-11-15: 0.25 deg x 1 deg grid, interlock narrowed to +-1 deg of yaw |
| beak (mouth servo) | - | **0..+35** (0 = closed, + = open) | closing past 0: jaw on the head base from 2 deg, then face / dome / camera | 1 deg sweep, 2026-11-14 |

Walking (yaw -6.3..+12.6, roll +-10.9, pitch +-45.8, knee +-55, ankle +-23), STAND (neck 20, head 20) and SIT fit inside the servo
limits. SIT was replaced 2026-11-13 (the original hip -30 / knee 60 face-plants): now hip pitch 70, knee 18, ankle -16, hip yaw -3,
hip roll -4 (left; right mirrored), neck 20, head 20, reached from STAND in a straight line over 3 s with the IMU ankle balance on
(REVIEW.md "MuJoCo on the rebuilt parts; sit and stand").

**Neck / head pitch (user: A):** A = neck -80..+30, head -5..+65 (keeps STAND and SIT; the head can no longer tilt below -5 at any
neck angle, although it is clear down to -45 with the neck at 0 or back). B = neck -80..+20, head -25..+65 (more head range;
SIT's neck 28.6 no longer fits).

**Head yaw x head roll: no useful box exists; user: software interlock.** At head yaw 0 the head rolls +-25 freely. Yawed, the U's arm ends sit off
the roll axis just under the head base's floor (12-13 mm from it), and roll dips the floor onto them by about r x sin(roll): at
yaw +-5 even 5 deg of roll touches; at +-45 10-15 deg collide up to 550 mm3. Roll is usable to +-15 only within +-2.5 deg of yaw (2026-11-13; finer grid 2026-11-15: +-1.0, below).
Tried (2026-11-13): trimming the U and lowering its far corner gives the room, but the U then cannot slide over the head-pitch
servo onto its horn (the corner must stay above the servo top), and the bearing plate cannot reach the U above the cradle
(1.4 mm of height). With this U the most roll at every yaw is about +-3 deg; more needs the head raised on a taller U centre
(+-5: ~2 mm, +-10: ~4.5, +-15: ~7). **User: keep the head, software interlock**: servo limits yaw +-150, roll +-15; roll only
while abs(head yaw) <= **1.0** (2026-11-15; was 2.5).

**2026-11-15 re-measure** (yaw -4..+4 in 0.25 deg steps x roll -25..+25 in 1 deg, exact parts): within +-2.5 deg of yaw 0, 28 poses
collide - at yaw -2 to -2.5 with roll -10 to -15 the head base cuts up to 0.40 mm into the head-pitch U (found by the independent
review). Within +-1.5: no collision, gap down to 0.07 mm; within **+-1.0: no collision, gap >= 0.31 mm**; within +-0.5: >= 0.5 mm.
The microduck-nano firmware already uses +-1.0.

## 2. Software interlocks (combinations)

| Combination | Limit | Collides |
|---|---|---|
| head roll with head yaw | roll only while abs(head yaw) <= **1.0** (was 2.5 until 2026-11-15), then abs(roll) <= 15; yaw only while abs(roll) <= 1 | U vs head base / jaw (yaw 2-2.5 with roll 10-15: 0.40 mm) |
| hip pitch + knee + ankle + roll (walking envelope corners) | e.g. yaw -6.3, roll +10.9, pitch +45.8, knee -55: shin vs enclosure 182-200 mm3 (old shin too); pitch -45.8 + knee +55 + ankle -23: L-bracket vs foot 20 mm3 | keep the gait's actual trajectories inside the swept-clear set (to be checked per gait in the sim) |
| hip pitch + knee (fold / sit-up) | left hip +60 with knee -90: foot vs Uno Q cover / body lid | sit / stand only by the checked path: STAND <-> SIT (hip 70, knee 18, ankle -16, hip yaw -3, roll -4) in a straight joint-space line, >= 3 s each way, IMU ankle balance on, not facing uphill > ~4 deg (exact parts clear >= 1.58 mm along it) |
| hip pitch x hip roll x hip yaw x knee (each leg) | the hip maps below, **one per leg** (2026-11-15) | leg swung back: knee cap / shin / foot bearing plate vs enclosure, body frame, lid, Uno Q cover; swung far forward with toe-out: foot / shin vs the Uno Q cover; **right leg only**: forward 45-90 with the knee at its limit, foot vs the neck bearing plate / body frame (up to 6.9 mm) |
| knee x ankle (each leg) | with the knee bent past +30 (left signs; right mirrored): ankle >= -60 at knee 40, >= -45 at 45-50, >= -35 at 55, >= -25 at 60, >= -10 at 64, >= 0 at 68 (between rows: the stricter) | foot vs L-bracket / thigh, up to 6.7 mm (knee 68, ankle -23). 2026-11-15, hip at rest, any hip angle the same. Walking (knee <= 55, ankle +-23) and SIT are inside |
| both legs together | never both legs forward of hip pitch -30 (left signs; right mirrored) while both hips are toed out (yaw > 0); one leg forward and one back is clear | the two feet / foot bearing plates against each other, up to 8 mm (toe-out +19, roll -12..+5, pitch -30..-90). The legs-back cases (toe-in with roll out) are already excluded by each leg's map |

### Hip maps: allowed hip roll by hip pitch and hip yaw, one table per leg

Measured 2026-11-15 on the exact parts with `tools/blender/clearance_sweep.py` (replaces the left-only map of 2026-11-14): for
EACH leg 1,248 poses (hip pitch -90..+85, roll -12..+19.5, yaw -6.3 / 0 / +19, knee -90 / -45 / 0 / +68; ankle 0), plus 2,496 with
both legs moving. A cell allows a roll when no knee of its group collides (surfaces crossing deeper than 0.05 mm) and no gap is
under **0.3 mm** (the tool's accuracy floor; the hip-yaw bearing's 0.3 mm at roll +19.5 belongs to that roll limit, above, and is
left out). Full data, every pose with its contacts and gaps: `data/hip_collision_map.json` (`poses` = left, `right_poses` = right,
`both_legs_poses`), tables printed by `tools/blender/hip_map_table.py`.

Both tables are in the **left leg's signs** (pitch + = leg back toward the bay, roll + = out, yaw + = toe out); for the right leg
flip all four signs before looking up. Knee in -45..+68 (left signs); "folded" = the knee at -90.

Left leg: 253 poses collide (all 245 of the 2026-11-14 map, plus 8 at 0.06-0.70 mm: the knee cap into the hip-yaw servo with the
leg far back and rolled out, and thin touches on the enclosure), 19 more clear by under 0.3 mm.

| hip pitch (left leg) | yaw -6.3 | yaw +0 | yaw +19 |
|---|---|---|---|
| -90 | -12..+10 | -12..+10 | -12..+0 |
| -75 | all | all | -12..+0 |
| -60 | all | all | -12..+15 |
| -45 | all | all | all |
| -30 | all | all | all |
| -15 | all | all | all |
| +0 | all (folded: +10..+19.5) | all | all |
| +15 | -8..-4, +5..+10 (folded: none) | -8..+19.5 | -8..+19.5 |
| +30 | -4..+5 (folded: none) | -8..+10 (folded: -8..+5) | -8..+19.5 |
| +45 | **none** | -12..+10 (folded: -12..-4) | all (folded: -12..+15) |
| +60 | **none** | -12..+0 (folded: -12..-8) | -12..+15 (folded: -12..+10) |
| +75 | **none** | -12..-8 | -12..+10 (folded: -12..+5) |
| +85 | **none** | -12..-8 | -12..+5 |

Right leg: 283 poses collide. It is **not** a mirror of the left: 30 poses collide where the mirrored left pose is clear (none the
other way) - leg forward 45-90 with the knee at its limit, the right foot reaches the neck bearing plate and the body frame (up
to 6.9 mm). The rows -45..-90 differ; the rest is the same as the left.

| hip pitch (right leg, mirrored) | yaw -6.3 | yaw +0 | yaw +19 |
|---|---|---|---|
| -90 | -12..+0 | -12..+0 | **none** |
| -75 | -12..+5 | -12..+0 | **none** |
| -60 | -12..+15 | -12..+15 | -12..-8 |
| -45 | all | all | -12..+5 |
| -30 | all | all | all |
| -15 | all | all | all |
| +0 | all (folded: +10..+19.5) | all | all |
| +15 | -8..-4, +5..+10 (folded: none) | -8..+19.5 | -8..+19.5 |
| +30 | -4..+5 (folded: none) | -8..+10 (folded: -8..+5) | -8..+19.5 |
| +45 | **none** | -12..+10 (folded: -12..-4) | all (folded: -12..+15) |
| +60 | **none** | -12..+0 (folded: -12..-8) | -12..+15 (folded: -12..+10) |
| +75 | **none** | -12..-8 | -12..+10 (folded: -12..+5) |
| +85 | **none** | -12..-8 | -12..+5 |

Firmware rule: between grid values use the stricter neighbour (pitch and yaw); look up each leg in its own table. In words:
**with the hip toed in at all, keep hip pitch back under ~+15 deg**; with yaw 0, roll the leg out no more than the row allows
(at +45 back: <= +10); **the right leg forward of -45 has less room than the left**. The original's walking envelope (yaw
-6.3..+12.6, roll +-10.9, pitch +-45.8, knee +-55, ankle +-23) crosses this map (toe-in with the leg back) - a gait must be checked
against it. **Ankle (2026-11-15):** the same grid with the ankle at -23 and +23 (each leg): apart from the knee x ankle rule (row
above), ankle **+23** adds these collisions (0.3-1.6 mm, foot vs enclosure / lid / Uno Q cover) - hip pitch / roll / yaw / knee
(left signs): -15 / -12 and -8 / -6.3 / -90; +30 / 0 and +5 / 0 / -90; +60 / 0 / 0 / -45; +85 / 0 / 0 / 0; right leg also +45 / -4 /
0 / -90. Treat those cells as not allowed with the ankle raised. Data: `ankle_pm23_left/right` and `knee_ankle_left/right` in the
map file. Not covered: ankle beyond +-23 together with the hip; the review's two-leg
ankle-axle touch (hips rolled in -12, legs forward: 0.2 mm) was not reproduced on this grid (yaw 0 / roll -12 / forward: clear).

## 3. Overload protection (all 15 servos, values to confirm on a servo)

Enable overload + overheat unloading (Unload Condition bits 5 and 2), keep the Overload Torque Threshold at or below the factory
80 %, Protection Time short (factory 2 s), Holding Torque low (factory 20 %), Max Temperature 70 C. Set per servo with the
Feetech tool before the first power-up with the legs fitted; read every value back.

Sources: Feetech SCSCL memory table (aifitlab-wiki.super.site, "FEETECH SCSCL Servo - Memory Table Analysis"); Waveshare
"SC Servo - Memory Table Explanation".
