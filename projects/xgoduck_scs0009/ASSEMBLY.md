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
2. **Print a tolerance coupon** first (6702 seat Ø21.1 + boss Ø14.9, 693 seat Ø8.1, M2 pilot Ø1.7, clearance Ø2.4, M3 nut
   pocket, a servo saddle at 0.25 clearance) and adjust if your printer prints holes smaller than ~0.1-0.2 mm.
3. Set every servo to the middle of its range (horn zero) before fitting a horn part on it.
4. **Horns:** 6 servos use the kit X horn **as it comes** (both knees, both ankles, neck pitch, head pitch). The other **9 have
   their horn's arms cut short** (hip yaw, hip roll, hip pitch on both legs, head yaw, head roll, beak): print the **trim jig**
   (`horn_trim_jig`, 0.4 g), drop the horn in hub-first with the arms in the 4 slots, cut each arm flush with the jig's wall with a
   flush cutter, file off the burr. The arms then end 5.5-6.2 mm from the centre (the pockets are 6.2 deep). Don't cut the 6 full
   horns. Check each horn in its pocket before the screw: it should press in on the ribs, flat on the pocket floor.

General rule for the joints: **the joint's clamp screw (the servo's own horn screw) goes in before the next servo is fitted**:
its head faces the next servo (hip yaw: 0.3 mm from the hip-roll servo case; head yaw: 4.7 mm from the head-pitch servo).

## 1. Body

1. Press the two **6702ZZ** into the hip-yaw seats of the motor body (outer ring against the lip).
2. **Right hip-yaw servo**: straight down into its saddle ✓. **Left hip-yaw servo** ✓: lift it ~7 mm above its seat, slide it
   in from the **rear or from either side** under the neck fork, then down (the window is 6.5-8.5 mm of lift).
3. Servo ear screws: the kit's mounting screws, except the **left servo's front ear**: an **M2 × 6 socket screw with the
   ball-end key at 20-25°** (the neck fork is 8 mm above it) ✓.
4. Electronics bay (the enclosure): fit it to the motor body with the **3 × M2 × 10 countersunk** from inside the keel (before the
   battery). Then the servo adapter (slides into its rib slots), regulator, IMU (4 × M2 × 6), toggle switch, battery last
   (hook-and-loop under it).
5. Lid: **lid screws (4 × M2 × 6) before the Uno Q** (the board covers them) ✓. Then the Uno Q (4 × M3 × 8), then the cover with
   its **2 side screws** (one each side, behind the rear posts) ✓.

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

## Not yet checked in the model

The horn shape (pockets drawn from the kit photo; hub and plate thickness nominal), the real kit screws, battery thickness (bay
designed for 21 mm), IMU hole spacing (estimated), the SCS0009's steps per degree and protection registers (JOINT_LIMITS.md) - all on
arrival of the parts. Print a tolerance coupon with the knee's 6702 pocket and a 28 mm bridged floor before whole legs.
