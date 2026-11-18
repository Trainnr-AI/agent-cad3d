# XGO-Duck SCS0009 v2: assembly sequence

Derived from the model's checks (2026-11-13): assembly paths (`tools/blender/assembly_path_check.py`, L-paths), driver access
(a straight tool line along each screw, ~35 mm), screw audit (thread bite, pilot depth, walls). ✓ = verified in the model;
"order" = the step only works in this order (a later part blocks the path or the tool).

Parts list and screws: `BOM.md`. Tools: 1.5 mm hex key (plain + **ball end**), 2.5 mm hex key, PH0/PH1 screwdrivers with
shafts ≤ Ø3 mm, caliper.

## 0. Before printing / building

1. **Measure** a servo's horn (hub Ø and height, plate thickness, arm widths, tip-to-tip), its horn screw and its mounting
   screws (length under the head, head Ø and height). The model assumes horn screw M2 × 4 (head Ø3.5 × 1.3) and mounting screws
   ≤ 7.2 mm long. Longer mounting screws: put an M2 nut from the Robu set under the head as a spacer.
2. **Print the tolerance coupon** first: send the folder `projects/xgoduck_scs0009/print/test_batch/` to the printing service (its
   PDF guide + 6 print-ready STLs, about 13 g: the 5 coupon pieces, built by `tools/blender/parts/tolerance_coupon.py` from the
   same joint code as the robot's parts, and the horn trim jig; see `print/README.md`). Same printer, filament and settings
   as for the robot. Test with the real parts:
   - `coupon_knee_rotor` (print the bearing-seat face down): a **6702ZZ** presses into the Ø21.1 seat by hand and sits flat, and
     the outer ring does not rock; a **full X horn** pushes into the pocket underneath (crush ribs take up the play), no wobble.
   - `coupon_knee_cap` (top face down): its Ø14.9 boss slides into the bearing's bore without force, and the bearing turns freely
     once the cap is on (0.1 mm axial play by design). Through the Ø5.2 hole a screwdriver reaches the horn screw.
   - `coupon_trim_plate`: a **horn trimmed with the jig** (step 0.4) pushes in, no wobble.
   - `coupon_bar`: a **693ZZ** presses into the Ø8.1 seat; an **M3 nylon nut** drops into the hex trap and an M3 x 8 passes the
     Ø3.3 hole; an **M2 x 6** threads into the Ø1.7 pilot with a firm but even feel (no splitting) and slips through the Ø2.4 hole;
     an **M3 x 8** forms its thread in the Ø2.5 pilot; the 28 mm bridge underneath is flat (sag < 0.5 mm).
   - `coupon_servo_collar`: slides over the **servo body** with a little play, no force.
   If holes come out tight (FDM holes often print 0.1-0.2 mm small) or the horn is loose, tell the agent the measurements: the
   fits are constants in `tools/blender/parts/joint_v2.py` (SEAT_D, BOSS_D, HORN_CLEAR, RIB_INTERF, PILOT_D, CLEAR_D, BP_SEAT_D,
   NUT_AF, SADDLE_CLEAR) - change them, rebuild, re-check, re-print the coupon.
3. Set every servo to the middle of its range (horn zero) before fitting a horn part on it.
4. **Horns:** 6 servos use the kit X horn **as it comes** (both knees, both ankles, neck pitch, head pitch). The other **9 have
   their horn's arms cut short** (hip yaw, hip roll, hip pitch on both legs, head yaw, head roll, beak): print the **trim jig**
   (`horn_trim_jig`, 0.4 g), drop the horn in hub-first with the arms in the 4 slots, cut each arm flush with the jig's wall with a
   flush cutter, file off the burr. The arms then end 5.5-6.2 mm from the centre (the pockets are 6.2 deep). Don't cut the 6 full
   horns. Check each horn in its pocket before the screw: it should press in on the ribs, flat on the pocket floor.

5. **Press all 14 bearings into their printed parts first**, on the bench, before any servo goes in (a press on a part with a
   servo in it loads the servo's gears). Each one-sided joint's 6702ZZ sits in the part that HOLDS the servo, in a seat that
   opens away from the servo (the lip, on the servo side, carries the outer ring), so it goes in from the outside:

   | Bearing | Into | Seat | Moving part that runs in it |
   |---|---|---|---|
   | 6702ZZ × 2 | body frame (motor body) | hip-yaw seats | yaw connector |
   | 6702ZZ × 2 | yaw connector (each leg) | hip-roll cap | L-bracket |
   | 6702ZZ × 2 | thigh (each leg) | hip-pitch cap | L-bracket |
   | 6702ZZ × 2 | shin (each leg) | knee seat in its flat inboard face | knee cap's boss |
   | 6702ZZ × 1 | head base | head-roll bulkhead | yaw block |
   | 6702ZZ × 1 | yaw block | head-yaw cap | head-pitch U |
   | 693ZZ × 2 | foot bearing plate (each leg) | ankle | on the M3 × 8 axle in the shin |
   | 693ZZ × 1 | body neck bearing plate | neck | on the M3 × 8 axle in the neck cradle |
   | 693ZZ × 1 | head-pitch bearing plate | head pitch | on the M3 × 8 axle in the neck cradle |

   Press on the **outer ring only** (a socket or a flat washer the size of the ring; never on the shield or the inner ring),
   square, until it sits on the lip. It should go in by hand or with a vice and stay put; if it drops in loose or needs
   hammering, the seat size is off: stop and report (section 0, item 2). Spin each one by hand: smooth, no grit.

General rule for the joints: **the joint's clamp screw (the servo's own horn screw) goes in before the next servo is fitted**:
its head faces the next servo (hip yaw: 0.3 mm from the hip-roll servo case; head yaw: 4.7 mm from the head-pitch servo).

## 1. Body

1. The two **6702ZZ** in the hip-yaw seats of the motor body (section 0, item 5; outer ring against the lip).
2. **Right hip-yaw servo**: straight down into its saddle ✓. **Left hip-yaw servo** ✓: lift it ~7.5 mm above its seat, slide it
   in from **either side** (clear from 6.5 mm of lift) **or from the rear** (from 7.5 mm) under the neck fork, then down (the
   window ends at 8.5 mm; re-checked 2026-11-18).
3. Servo ear screws: the kit's mounting screws, except the **left servo's front ear**: an **M2 × 6 socket screw with the
   ball-end key at 20-25°** (the neck fork is 8 mm above it) ✓.
4. **Electronics bay** - exact layout, screws and wires: `data/bay_assembly_sheet.png` (exploded, numbered as below; model
   2026-11-19: every part, connector, screw and wire at its real size, all wires clear of all parts). Bench work first:
   - **Solder** (20 AWG for power, cut female-female jumper wires for signals; heat-shrink every joint):
     servo adapter's back pins: DC+ / DC- (5 V in) and UART TX / RX / GND; **tape over its 3 front UART pins** (they point at the
     battery). Mini560 pads **from below**: IN+ (from the switch), IN- (pigtail black), OUT+ / OUT- (to the adapter). Switch
     lugs: middle = pigtail red, one outer = Mini560 IN+ and the Uno Q's VIN wire. Uno Q plugs: **right-angle 2.54 mm male
     header pins** - 3 pins (JANALOG 6 UART-GND, 7 GND, 8 VIN) and 2 pins (D0 = adapter TX, D1 = adapter RX).
   1. Bay enclosure onto the motor body: **3 x M2 x 10 countersunk** (Phillips) from inside the front wall, screwdriver through the
      3 holes in the rear wall ✓ (before the battery: it stands 0.6 mm in front of them).
   2. **Servo adapter** slides down into its rib slots ✓ (no screws), its two servo cables plugged in first.
   3. **IMU**: Qwiic cable into its right socket, then **4 x M2 x 6 socket** into the rear-wall bosses ✓ (4.5 mm of thread).
   4. **Switch**: drop its M6 nut into the hex recess from outside, then screw the switch in from inside by turning its body until
      snug with the lever moving up / down (before soldering its wires).
   5. **Battery** (60 x 30 x 20): hook-and-loop on the floor, at the front. Its main leads and balance lead go back over its top and
      **out of the rear-right window**. Its main leads must reach **~75 mm** from the pack's top to its XT30 (round the corner to the
      middle of the rear face): **measure before buying / on arrival**.
   6. **Rear cradles**: the battery's XT30 into the robot's pigtail, the pair standing in the **middle of the rear face** (battery half on
      top; the right foot reaches the rear right below z 109), the pigtail looping down and up behind it and back in through the
      window. The **balance plug** stands **above the bay at the rear right**, behind the Uno Q cover, mouth up, against its fin.
      Zip ties on both. **Unplug the battery at the XT30; charge through the balance plug from above** - nothing to open.
   7. **Mini560**: slide it into the lid's slot (rails + ledges) **component side up**, its parts standing up through the window.
   8. **Lid**: **4 x M2 x 6 socket** into the bay posts ✓ (before the Uno Q: it covers them).
   9. **Uno Q**: **3 x M3 x 8** + **1 x M2 x 6** (the hole by the digital header) ✓. Plugs: the 3-pin on **JANALOG 6-7-8** - one pin
      over puts 12 V on A0 and destroys the microcontroller, one the other way shorts the battery: mark it (paint dot on pin 6's
      side), check against the board's labels before the first power-up, **hot-glue it**; the 2-pin on D0 / D1; Qwiic. Power, UART
      ground and Qwiic come up outside the bay's right wall (under the lid's edge), UART along the left wall.
   10. **Camera USB**: the rigid right-angle USB-C -> USB-A adapter into the Uno Q's rear socket, **mouth down**; the camera's
      USB-A plug into it from below, lying in the **rear-left cradle**, 1 zip tie. Its cable loops up the rear-right corner and
      forward along the right side to the neck. In the head (do this when building the head): the camera's JST plug into the strip's socket, its 4 wires
      turned back **right at the plug** into the 1.5 mm pocket past it, behind the board's end, through the face's hollow and the
      slot in the face's D-panel (right side); the cable then runs over the jaw, down the head's right side and out beside the floor
      opening to the head-pitch U. Turn the head through yaw +-150 and pitch, and open the beak, before closing the dome: nothing may pull.
   11. Uno Q cover (optional): **2 x M2 x 6** through its sides ✓.
5. Before the first power-up: switch off, battery plugged, check the Uno Q plug position again; then the heat test (section 5).

## 2. Neck and head

1. **Neck servo** down into the neck cradle ✓ (ear screws before the cradle meets the body: the body is 7.4 mm from them).
   Cradle onto the neck horn plate of the motor body ✓, horn screw, bearing plate from the other side ✓ (2 × M2 × 6), M3 × 8 axle
   + nylon nut (snug only).
2. **Head-pitch servo** down into the cradle ✓ (ear screws before the U goes on).
3. **Head base**: 6702ZZ into the roll bulkhead; **head-roll servo** into its holder ✓; ear screws.
4. **Yaw block** onto the roll servo's horn ✓: **up through the floor opening ~4.8 mm forward of its place, then slide it back**
   onto the horn and bearing (keep its rear boss clear of the opening's back edge on the way up). Horn screw now (order: the head-yaw servo covers it).
5. **Head-yaw servo** up into the yaw block ✓; ear screws.
6. **U** onto the head-yaw horn and bearing ✓; horn screw now (order: the head-pitch servo covers it, 4.7 mm).
7. **Jaw** straight **up through the two bill-plate slots** ✓ (beak servo NOT in yet); **jaw pivot screw** through the post into
   the right arm (order: the beak servo is 17 mm from it once in).
8. **Beak servo** ✓: lower it in **~4.5 mm toward the middle of the head**, then slide it outward so its horn enters the jaw arm's
   pocket. Horn screw, ear screws (with the face still off ✓).
9. **Camera** onto the face first: 3 × M2 × 6 driven **through the face's Ø4.6 holes** from behind ✓. Then the face onto the head.
10. **Dome** last: 2 rear screws from below; **2 front screws up through the jaw's Ø4.6 holes** ✓.
11. Head (with the U) onto the head-pitch servo in the cradle ✓, horn screw, bearing plate ✓, M3 axle + nylon nut.

## 3. Legs (each side)

The knee's cap screws and horn screw point inboard: once the leg hangs on the body the electronics enclosure is 13-16 mm in front of
them (driver access check). So **build the leg below the hip first, off the robot**, then hang it on the hip.

1. Hip-yaw: **yaw connector** onto the hip-yaw horn and bearing ✓, horn screw (order: the hip-roll servo covers it, 0.3 mm).
2. **Hip-roll servo** into the yaw connector ✓; ear screws (>= 22 mm room: an L-key).
3. Off the robot - thigh: **hip-pitch servo** into the thigh ✓, ear screws; **knee servo** into the thigh ✓, ear screws.
4. Off the robot - **knee** (full kit horn, uncut): press the full X horn into the shin's ribbed pocket, **shin onto the spline** ✓,
   **horn screw** ✓; **6702ZZ into the shin's seat** (its flat inboard face) ✓; **knee cap** down: boss into the bearing, post onto
   the thigh ✓; **2 x M2 x 6 socket** ✓ - snug them only after the cap has settled on the bearing (it centres itself; clearance
   holes). The horn screw can be checked later through the cap's Ø5.2 hole.
5. Off the robot - **ankle servo** down into the shin cradle ✓, ear screws (before the foot); **foot** onto the horn ✓ + horn screw;
   bearing plate from the other side ✓ (2 x M2 x 6), M3 x 8 axle + nylon nut (snug only).
6. **L-bracket** onto the hip-pitch horn ✓ + horn screw (order: before step 7, the hip-roll servo then covers it).
7. Hang the leg on the hip: the L-bracket onto the **hip-roll horn** ✓ + horn screw.
8. Thigh covers last.

## 4. Wiring

Route per `tools/blender/wiring.py` (hubs, extensions, the 250 mm extension to the left thigh). Straight-line reach over all
poses leaves ≥ 26 mm of slack on every cable (least: left body-to-thigh extension 224 of 250 mm) - route without tight bends
and tie the slack where the wiring plan shows it.

Where moving parts push on cables (`tools/blender/cable_pinch_check.py`, 2026-11-15; BOM row 13b, 6 mm braided sleeve):
- **Each hip:** the hip-roll servo lead and the body-to-thigh extension leave the yaw connector under the deck and go up to the
  thigh. With the leg swung back (walking) they wrap round the deck's lower outer edge, every step. Sleeve both together for
  ~6 cm from the yaw connector to the thigh; the edge is rounded (r 1.5) for them. Leave them loose enough that the leg can swing
  fully back (hip pitch +85) without pulling.
- **Neck:** sleeve the U-to-head extension and the camera USB together for ~10 cm across the head-pitch U and yaw block, and leave
  a **slack loop** there: when the head turns past ~75 deg they are dragged across those parts. Turn the head by hand to both
  limits (+-150) before closing the dome and check nothing pulls tight or catches.
- Smaller rubs at the joint limits (ankle leads on the foot / shin at full ankle or knee bend, the gap-to-neck extension on the
  body frame at neck -80): route with a little slack and check by hand at the limits.

## 5. Power and battery: rules (no protection electronics on board - user, 2026-11-18)

The robot has no fuse, protection board or battery monitor (user's choice: nothing that is not essential). The battery and the
person charging it are the protection, so these rules are part of the build (the battery's XT30 and balance plug are in the
rear-right cradle: unplug after play, charge there):

- **Switch off right after use.** Left on, the Uno Q drains the battery below 3 V per cell within hours; a LiPo drained that far
  must **not** be recharged (it can catch fire while charging) - it goes to e-waste.
- **Short sessions, then charge.** Expect roughly 20 min of use per charge (measure it on the first runs and write it here).
  If the robot ever goes limp and dark on its own, the battery is flat: before charging, check every cell with a LiPo checker
  (BOM row 25); any cell under 3.0 V -> do not charge, retire the pack.
- **Charging:** only the balance plug on the charger, in the LiPo bag, on a hard surface, never unattended (BOM row 16).
- **Look at the battery every time it comes out:** puffy, dented, hot or smelling -> stop, put it in the bag outside, retire it.
  A pack that no longer slides into the bay freely has swollen.
- **First power-up heat test** (regulator): lid on, the robot holding a squat (or STAND) for 10 min, then touch the regulator
  through the lid's window. If you cannot keep a finger on it for 5 s (about 60 °C), switch off and report - PLA softens near 60 °C.
- If protection is wanted later: a 3S 10 A protection board (50 × 16 × 3.4 mm, e.g. HX-3S-01) fits standing behind the battery
  (x -33.1..-29.7, y -112..-96, z 67.5..117.5: 0.47-0.88 mm from the adapter, switch, battery and wall; measured 2026-11-18)
  without changing any printed part.

## Not yet checked in the model

The horn shape (pockets drawn from the kit photo; hub and plate thickness nominal), the real kit screws, battery thickness (bay
designed for 21 mm), IMU hole spacing (estimated), the SCS0009's steps per degree and protection registers (JOINT_LIMITS.md) - all on
arrival of the parts. Print the tolerance coupon first (section 0, item 2). The cable pinch check models each cable span as a straight
line between its clips: confirm the routing on the first assembled leg.
