# nanoduck (XGO-Duck SCS0009 v2): joint limits

Measured in the model 2026-11-13 (`tools/blender/leg_motion_check.py` exact intersection volumes, then minimum gaps at the limit
corners with every coupled joint moving). Angles in degrees from each joint's zero (the robot's rest pose); left/right signs as
in the MJCF. Decided with the user 2026-11-13.

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
| hip roll L / R | +-22 | **L -12..+19.5, R -19.5..+12** (- = feet together) | roll -22: the feet collide at any yaw | (with hip yaw, as above) |
| hip pitch L / R | -90..+85 / -85..+90 | unchanged | alone: clear over the range | 5 deg sweep |
| knee L / R | -90..+68 / -68..+90 | unchanged | alone: clear | 5 deg sweep |
| ankle L / R | -86..+90 / -90..+86 | unchanged | alone: clear | 5 deg sweep |
| neck pitch | -80..+40 | **-80..+30** (user: option A) | +40 alone: head base vs Uno Q cover 71 mm3 | corners >= 1.36 mm |
| head pitch | -45..+65 | **-5..+65** (user: option A) | neck +10..+40 with head -45..-5: U / bearing plate vs head base | corners >= 1.36 mm |
| head yaw | -150..+150 | **+-150** | alone: clear | 5 deg sweep |
| head roll | -25..+25 | **+-15** + software interlock (user) | yawed, the U meets the head's floor | grid |
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
yaw +-5 even 5 deg of roll touches; at +-45 10-15 deg collide up to 550 mm3. Roll is usable to +-15 only within +-2.5 deg of yaw.
Tried (2026-11-13): trimming the U and lowering its far corner gives the room, but the U then cannot slide over the head-pitch
servo onto its horn (the corner must stay above the servo top), and the bearing plate cannot reach the U above the cradle
(1.4 mm of height). With this U the most roll at every yaw is about +-3 deg; more needs the head raised on a taller U centre
(+-5: ~2 mm, +-10: ~4.5, +-15: ~7). **User: keep the head, software interlock**: servo limits yaw +-150, roll +-15; roll only
while abs(head yaw) <= 2.5.

## 2. Software interlocks (combinations)

| Combination | Limit | Collides |
|---|---|---|
| head roll with head yaw | roll only while abs(head yaw) <= 2.5, then abs(roll) <= 15; yaw only while abs(roll) <= 1 | U vs head base / jaw |
| hip pitch + knee + ankle + roll (walking envelope corners) | e.g. yaw -6.3, roll +10.9, pitch +45.8, knee -55: shin vs enclosure 182-200 mm3 (old shin too); pitch -45.8 + knee +55 + ankle -23: L-bracket vs foot 20 mm3 | keep the gait's actual trajectories inside the swept-clear set (to be checked per gait in the sim) |
| hip pitch + knee (fold / sit-up) | left hip +60 with knee -90: foot vs Uno Q cover / body lid | sit / stand only by the checked path: STAND <-> SIT (hip 70, knee 18, ankle -16, hip yaw -3, roll -4) in a straight joint-space line, >= 3 s each way, IMU ankle balance on, not facing uphill > ~4 deg (exact parts clear >= 1.58 mm along it) |

## 3. Overload protection (all 15 servos, values to confirm on a servo)

Enable overload + overheat unloading (Unload Condition bits 5 and 2), keep the Overload Torque Threshold at or below the factory
80 %, Protection Time short (factory 2 s), Holding Torque low (factory 20 %), Max Temperature 70 C. Set per servo with the
Feetech tool before the first power-up with the legs fitted; read every value back.

Sources: Feetech SCSCL memory table (aifitlab-wiki.super.site, "FEETECH SCSCL Servo - Memory Table Analysis"); Waveshare
"SC Servo - Memory Table Explanation".
