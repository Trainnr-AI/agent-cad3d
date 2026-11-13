# XGO-Duck SCS0009 v2: bill of materials

Kept up to date as parts are finalized with the user (India sourcing). Status: **FINAL** = chosen and designed in;
**PENDING** = a stand-in is designed in, the real part is still to be chosen. Quantities are counted from the Blender model.
Last update: 2026-11-13.

## Electronics

| # | Status | Part | Qty | Key specs (as designed) | Where / price | Notes |
|---|---|---|---|---|---|---|
| 1 | FINAL | **Feetech SCS0009** serial bus servo | 15 | 6 V 2.3 kg·cm (4–7.4 V input), 300°, Molex 5264 lead 15 cm | (project basis) | 14 joints + beak. Run at **5 V** (see 5) |
| 2 | FINAL | **Arduino UNO Q** (ABX00162 2 GB / ABX00173 4 GB) | 1 | 68.58 × 53.34 mm, 4 × M3 | Arduino store / authorised resellers | On 4.5 mm posts on the lid, under the hood cover |
| 3 | FINAL | **Waveshare OV5640 5MP USB Camera (B)** | 1 | 9 × 62 mm strip, built-in mic | (user's listing) | In the head's brow band. Its mic replaces the sound card |
| 4 | FINAL | **Pro-Range 850/3S-30C LiPo** | 1 | 11.1 V 850 mAh, 57 × 31 mm, 75 g, Deans T-plug, JST-XH balance | Robu (Pro-Range brand) | Stands upright at the keel front. **Measure its thickness** (designed for 21 mm, 0.3 mm spare) |
| 5 | FINAL | **Mini560 5 V 5 A** buck module | 1 | 29 × 18 × 5.4 mm, 6–32 V in, **5 V out** | [Zbotic](https://zbotic.in/product/mini560-dc-5v-5a-step-down-stabilized-module/) ~₹93 · [Robodo](https://robodo.in/products/mini560-dc-5v-5a-step-down-stabilized-module) | Order the **5 V** version (also sold in 3.3 / 9 / 12 V). ~3 A continuous without cooling: measure current. Upgrade path: UBEC-8A outside the body |
| 6 | FINAL | **Waveshare Bus Servo Adapter (A)**, SKU 25514 | 1 | 42 × 33 × 14.6 mm, UART 3.3 V, 2 × 5264 sockets | [ThinkRobotics](https://thinkrobotics.com/ar/products/serial-bus-servo-driver) ₹500 | Jumper on **A** (UART). Power + UART wires **soldered on the back**. Rated 7(9)–12.6 V in: **test at 5 V on arrival** |
| 8 | FINAL | **7Semi BNO086 9-DOF VR IMU Qwiic Fusion Breakout** | 1 | 27.89 × 20.87 mm (7Semi datasheet ES-12243), ~6 mm with its Qwiic sockets, 2 × Qwiic, I²C 0x4B (0x4A by jumper), 3.3 V | [Robocraze](https://robocraze.com/products/bno086-vr-imu-9-dof-orientation-qwiic-fusion-breakout-7semi) ₹2,294 | Upright on the bay's rear wall on 4 bosses. **Hole spacing ESTIMATED** (±11.35 × ±7.83 mm, not in the datasheet): measure on arrival. Set its orientation in software. Same software as BNO085/BNO080 |
| 9 | FINAL | **Mini toggle switch MTS-102-A2T** (ON-ON, 3 pins; Daier MTS series) | 1 | 6 A @125 V AC, body 13.2 × 7.8 × 9.5, 6 mm bushing | [Robu](https://robu.in/product/mts-102-a2t-miniature-toggle-switch-3-pinon-on/) | In a recess in the bay's left side wall, lever tip flush with the outside. Wire **battery + → middle pin → one outer pin** (the other outer pin unused) = plain on/off. Do not use the tiny SS-12F15 slide switch (0.5 A) |

## Cables and connectors

| # | Status | Part | Qty | Notes |
|---|---|---|---|---|
| 10 | FINAL (with the servos) | Kit 3-way servo hubs (5264) | 13 | Come in the SCS0009 packaging (user). Designed for: board 20 × 17, ~24 × 20 × 7 with sockets, plug 9.5 × 10 × 5.5. **Check the real hub against these on arrival** and count: 13 needed |
| 11 | FINAL (with the servos) | 3-pin servo extension, 165 mm (kit) | 12 | Come with the servos (user). Count on arrival: 12 needed, plus the separate 250 mm one (row 12) |
| 12 | FINAL | 3-pin servo extension (5264), **250 mm** | 1 | Adapter → left thigh (runs over the battery, down the deck slot) |
| 13 | FINAL | USB cable, Uno Q USB-C → camera, **≥ 40 cm** | 1 | Worst-case run 335 mm. Connector at the camera end: per the camera's listing |
| 14 | FINAL | **Adafruit STEMMA QT / Qwiic JST-SH 4-pin cable, 100 mm** (Adafruit 4210) | 1 (buy 2) | IMU → Uno Q Qwiic (through the lid's rear notch). [Robu](https://robu.in/product/adafruit-stemma-qt-qwiic-jst-sh-4-pin-cable-100mm-long/) ₹129 (alt. [Zbotic](https://zbotic.in/product/adafruit-stemma-qt-qwiic-jst-sh-4-pin-cable-100mm-long/) ₹145). Symmetric, either end goes either way. Keep the heat-shrink sleeve in the middle out of the lid notch. Female JST-SH both ends; red 3.3 V, black GND, blue SDA, yellow SCL. Buy 2 (cheap; the tiny latches break). Check whether the 7Semi board ships with one. The plug at the IMU end has ~6 mm beside the board: use the IMU's Qwiic socket on the right (−y) side |
| 15 | FINAL | **SafeConnect nylon T-connector MALE pigtail, 14 AWG silicone wire, 10 cm** ([Robu](https://robu.in/product/safeconnect-nylon-t-connector-male-pigtail-with-14awg-silicon-wire-10cm/)) | 1 | Mates with the battery's red T-plug (female); duck side → switch → Mini560 + Uno Q VIN. Plus ~30 cm 20–22 AWG red/black wire for the regulator and Uno Q VIN leads |
| 16 | FINAL | **IMAX B3AC compact balance charger, 2S–3S LiPo** ([Robu](https://robu.in/product/b3ac-compact-balance-charger-2s-3s-lipo/)) | 1 | Charges through the battery's **white balance plug only** (3 × 800 mA, about 1–1.5 h for the 850 mAh pack). Basic and unregulated by display: Robu itself recommends it only for light use. **Never charge unattended; charge in a LiPo bag** (row 25). Upgrade path: SkyRC IMAX B6AC V2 |

## Mechanical

| # | Status | Part | Qty | Notes |
|---|---|---|---|---|
| 17 | FINAL | **Ball bearing 6702ZZ** (= 61702-2Z / 61702ZZ; 6702-2RS rubber-sealed also fits) | 10 | **bore d = 15 mm, outer D = 21 mm, width B = 4 mm**, deep groove, double metal shield; ~4 g each | [cart2india uxcell 10-pack](https://www.cart2india.com/track-runner-bearings/uxcell-6702zz-deepgroove-ball-bearings-for-rollers-15mm-x-21mm-x-4mm-seal-bearings-with-fiber-gloves-double-metal-sealed-bearing-steel-skateboard-bearings-10pcs/00000000004390703266) ~₹2,600; bicycle shops (6702-2RS) | Cap joints: knees, hip pitch / roll / yaw (×2 legs), head yaw, head roll. Printed seat Ø21.1 with a 1 mm lip on the outer ring; rotor boss Ø14.95 in the bore, shoulder Ø16.6 on the inner ring. A spare or two is advisable |
| 18 | FINAL | **Ball bearing 693ZZ** (= 693-2Z; 693-2RS also fits) | 4 | **bore d = 3 mm, outer D = 8 mm, width B = 4 mm**, deep groove, double metal shield; ~1 g each | [Quartz Components](https://quartzcomponents.com/products/693zz-ball-bearings-miniature-carbon-steel-bearings-3mm-x-8mm-x-4-mm-deep-groove-bearing); any 3D-printer shop | Fork joints: both ankles, neck, head pitch. Seated in the printed bearing plate, running on an M3 × 8 screw through the island (row 22) |
| 19 | FINAL | **M2 × 6 machine screw, socket head** (Robu datasheet: dk 3.7, k 2.0, 1.5 mm Allen) | 31 + 4 = 35 (+ 2 per further full-horn joint, see below) | Everything that is not a servo, plus ONE servo ear (left hip-yaw servo, front ear: under the neck fork, reachable only at 20-25 deg: socket head + ball-end key): lid 4, dome 4, camera 3, IMU 4, bearing plates (neck 2, head pitch 2, feet 4) 8, thigh covers 4, Uno Q cover 2, jaw pivot 1, **full-horn bearing caps, 2 per joint** (knees done 2026-11-13; all the other bearing joints stay trimmed (no room for a cap); plain heads in clearance holes so the cap centres on its bearing before it is clamped) - machine screws threaded straight into the printed Ø1.6 pilots (user 2026-11-13: few screw types, all standard, all from Robu). Buy **4 × [Robu EasyMech M2 × 6 socket head + nut, 12-pc set](https://robu.in/product/easymech-m2-x-6mm-socket-head-cap-allen-bolt-nut/)** = 48 (3 bought = 36 covers the 35 now with 1 spare: **buy 1 more** for spares; 1-2 more if further joints get caps; the nuts are spare). All heads checked in the model at full size (see below) |
| 20 | FINAL | **M2 × 10 countersunk machine screw, Phillips, SS304** | 3 | Bay front wall (3): heads 0.3 mm under the face (the battery stands 0.3 mm from it) in a **90° countersink** (2026-11-13: was a flat Ø4.0 counterbore, on which a countersunk head only bears on the hole's edge; found when the screw audit's axis bug was fixed). [Robu EasyMech SS304 CSK Phillips M2 × 10, 25 pcs](https://robu.in/?p=337115) ₹99 (Robu has no M2 × 6 CSK): 1 pack, 22 spare. (Not used for the bearing caps: a countersunk head would pull the cap onto the printed pilot instead of letting it centre on the bearing.) PH1 screwdriver |
| 21 | FINAL (with the servos) | **Servo's own horn screw** (M2 × ~4 machine screw, pan head; 1 per servo) | 14 | Joint clamp screw: through each joint's moving part into the servo's **brass** output spline (user's photo 2026-11-13). Seats lowered so 1.86 mm of print stays over the horn and the screw has ~1.8 mm (4½ threads) in the spline. **Measure one**: length under the head, head Ø and height (model assumes 4.0, Ø3.5 × 1.3) |
| 21b | FINAL (with the servos) | **Servo's own mounting screws** (2 per servo, self-tapping, washer-style pan head) | 29 | The servo ears into the printed pads (15 servos × 2 = 30; 29 used, the left hip-yaw servo's front ear takes an M2 × 6 socket screw, row 19: 1 spare). **Measure one**: length under the head, thread Ø, head Ø and height - the pads' pilots are 4.9 deep for a 6 mm screw through the 1.6 mm ear; a longer screw needs deeper pilots |
| 22 | FINAL | **M3 × 8 socket head cap screw, SS304** | 8 | 4 × Uno Q (into printed pilots), 4 × axles of the 693 bearings. [Robu EasyMech SS304 M3 × 8, 25 pcs](https://robu.in/product/easymech-ss304-m3-x-8-mm-socket-head-cap-allen-bolt-25-pcs/) ₹89 (user's pick 2026-11-13). 2.5 mm Allen key |
| 22b | FINAL | **M3 hex nut, white nylon** (AF 5.5, 2.4 thick: exactly the captive pocket) | 4 | On the 693 bearing axles (ankles, neck, head pitch): light axial clamp only, the steel screw carries the bearing's load. [Robu white nylon nut M3, 25 pcs](https://robu.in/product/white-nylon-nut-m3/) (user's pick; the steel nut pack is out of stock). **Tighten snug only** (nylon threads strip) |
| 23 | ESTIMATED | Servo horns: the kit **X (4-arm) horn**, 1 per servo | 15 | Come with the servos. Pockets drawn from Feetech's kit photo scaled by the listing (23.88 tip to tip): arms 5.2 -> 4.3 wide (+-0.15), holes at r 4.9 / 7.4 / 9.8; hub Ø6.5 x 4.0 and plate 1.6 still nominal. **Full horn, uncut** (user 2026-11-13): ankles, neck pitch, head pitch (as before) + **knees (done)**. **Hip pitch and hip roll stay trimmed to r 6** (trim jig, 4 servos; no room for a cap - user 2026-11-13). Head yaw, head roll and hip yaw stay trimmed (no room for a cap, 2026-11-13). **In the model (read from the servos): full horn on 6 (knees ×2, ankles ×2, neck pitch, head pitch), trimmed to r 6 with the jig on 9 (hip yaw / roll / pitch ×2, head yaw, head roll, and the beak servo's jaw arm - the beak not yet checked for a full horn).** Check on arrival (caliper or a printed pocket test) |
| 23b | FINAL | **Tools**: 1.5 mm hex key **with a ball end** (+ a plain one), 2.5 mm hex key, PH0 + PH1 screwdrivers (shaft ≤ Ø3), caliper | 1 set | Ball end: the left hip-yaw front ear screw is reached at 20-25 deg. Shaft ≤ Ø3: the camera and front dome screws are driven through Ø4.6 holes |
| 24 | - | Hot glue, foam tape, zip ties | - | Switch, hubs, cable slack |
| 25 | SUGGESTED | LiPo voltage alarm (1–8S buzzer, plugs on the balance lead) + LiPo fire-safe bag | 1 each | Warns before the pack drops below ~3.5 V/cell (over-discharge ruins it); store / charge in the bag |

## Printed parts

PLA (75% fill assumed in the mass model). Also print the **horn trim jig** (0.4 g, `joint_v2.trim_jig`) for the 9 horns that are cut short (ASSEMBLY.md step 0.4). Feet print as single pieces with the sole merged. Thigh covers are optional. The
enclosure needs **supports** only if the old lengthened bay is used (not the current upright layout). Mass with
everything: **658 g** (sim `xgoduck_scs0009_v2g`).

**Full-horn joints (decided 2026-11-13, rollout in progress):** the bearing joints that take the full horn get a small bolt-on **bearing cap**, 2 × M2 × 6 socket (row 19): **knees: 2 more prints**.
**Hip pitch: stays trimmed** - with the hip rolled, the yaw connector passes 0.3-2 mm over the L-bracket's inboard face, no room for a cap.
**Hip roll: stays trimmed** - a cap on the L-bracket's front face hits the other leg from 5 deg of outward yaw of both hips (walking uses 12.6).
**Knees done (2026-11-13), type C2M:** the 6702ZZ sits in the shin (seat in its flat face, so the shin still prints flat and the
ankle and foot did not move); the cap (`<side>_knee_cap`, 2.9 g solid PLA; rear top edge chamfered 45° to clear the electronics enclosure at the hip-yaw limit) carries a fixed boss into the bearing's bore and its own post
down onto the thigh's face (the thigh prints on that face too). A Ø5.2 hole through the cap reaches the horn screw. Prints top face down.

## Screw head room (resolved 2026-11-13)

Every screw head in the model now has its real size (M2 socket Ø3.8 × 2.0; kit horn screw Ø3.5 × 1.3) and fits at rest: 99 heads checked, smallest gaps 0.14 (dome screws under the jaw), 0.15 (mouth ear screw), 0.30 (camera, head roll ear), 0.39 (hip yaw). Changes made:

- **All 14 joints:** the servo's own horn screw instead of a bought M2 × 6; head seat lowered 0.6 (type C) / 1.0 (type F) in a Ø4.2 counterbore. This also fixed the **hip yaw** screws, whose head the hip-roll servo's ear blocked at 1.09 mm (no socket head fits there).
- **Head dome:** two Ø4.4 recesses inside, over the upper camera screw and the head-roll servo's rear ear screw (wall left behind them ≥ 2.2 mm).
- **Bay front wall:** 3 × M2 × 10 countersunk (row 20), deck pilots deepened to 8.5 mm.

## Not used any more

Distance sensor Adafruit VL53L1X + its chest plate and cable (dropped 2026-11-12: the camera does obstacle sensing) ·
USB sound card, speaker, USB hub (audio dropped 2026-11-12) · Hobbywing UBEC 5A (does not fit; Mini560 instead) ·
UBEC-8A (better, does not fit inside) · lengthened battery bay.
