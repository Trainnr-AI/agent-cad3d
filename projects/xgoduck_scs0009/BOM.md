# XGO-Duck SCS0009 v2: bill of materials

Kept up to date as parts are finalized with the user (India sourcing). Status: **FINAL** = chosen and designed in;
**PENDING** = a stand-in is designed in, the real part is still to be chosen. Quantities are counted from the Blender model.
Last update: 2026-11-28 (**planned: UNO Q shield PCB** - one JLCPCB-assembled board on the UNO Q's headers would replace the Bus Servo Adapter (A), the UBEC-8A, the switch wiring and the splices; IMU stays the 7Semi breakout on the shield's Qwiic socket; design, parts and LCSC numbers in `pcb/SHIELD_SPEC.md`, not yet ordered - the rows below stay the current build. Earlier 2026-11-28: **UBEC-8A replaces the Mini560** - row 5, 14b, 15, printed parts: in a vented pod on the bay's rear wall, lid extended over it; 2026-11-27: **camera vents**: 18 + 18 slots, 1.5 mm, through the head face's brow band below and above the camera board - the camera runs hot; `tools/blender/camera_vents.py`, head_face 7.4 -> 7.1 g. **bay v2**: one container + flat lid replace the enclosure, body lid, Uno Q cover, switch plate and Mini560 back plate - `tools/blender/parts/bay_v2.py`; every screw hole +0.4 after the user's printed test (M2 2.8 / 2.0, M3 3.7 / 2.9; print the **hole gauge** first); hip pitch limit 80 (leg back); printer **Bambu Lab A1, 0.12 mm High Quality** - rows 8, 13, 14, 15, 19, 20, 21, 22, 24, printed parts). Earlier: 2026-11-25 (all-Dupont bay, no soldering - rows 5, 14b, 15, 24; REVIEW.md 2026-11-25). Previous: 2026-11-20 (head: camera now held by a front **visor** + 3 × M2 × 10 countersunk from the front, rear dome screws moved back - rows 19 / 20, printed parts 31; screws from the outside: bay-to-body screws now from below, IMU on pegs + hot glue - row 19 now 32; switch on a separate bolt-in **switch plate** - the enclosure's switch block stopped the servo adapter going in; balance-plug fin reshaped - it ran into the lid, the Uno Q and its cover: row 20 now 5, printed parts 30; enclosure / lid / cover masses re-read). 2026-11-19 (bay redesigned with exact parts: battery 60 × 30 × 20 + XT30, rows 4 / 13 / 14b / 15 / 24; bay raised 3 mm). 2026-11-18 (bearing listings rows 17 / 18 and cost; regulator under a lid window, lid 10.2 g; no battery protection electronics by the user's choice - checker + bag now needed, row 25). 2026-11-15 (cable sleeve row 13b; independent-review fixes: Uno Q hole by the header M3 -> M2 × 6 (row 19 36, row 22 7), coupon designed, switch lever text; before that, 2026-11-14: every bought-part count re-read from the Blender model: servos, horns, horn screws, ear screws,
M2 / M3 screws, bearings, nuts, hubs and extension leads all match).

## Electronics

| # | Status | Part | Qty | Key specs (as designed) | Where / price | Notes |
|---|---|---|---|---|---|---|
| 1 | FINAL | **Feetech SCS0009** serial bus servo | 15 | 6 V 2.3 kg·cm (4–7.4 V input), 300°, Molex 5264 lead 15 cm | (project basis) | 14 joints + beak. Run at **5 V** (see 5) |
| 2 | FINAL | **Arduino UNO Q** (ABX00162 2 GB / ABX00173 4 GB) | 1 | 68.58 × 53.34 mm, 4 × M3 | Arduino store / authorised resellers | On 4.5 mm posts on the lid, under the hood cover |
| 3 | FINAL | **Waveshare OV5640 5MP USB Camera (B)** | 1 | 9 × 62 mm strip, built-in mic | (user's listing) | In the head's brow band. Its mic replaces the sound card |
| 4 | FINAL | **3S 850 mAh 30C LiPo** (Pro-Range / 'Orange', Robu) | 1 | 11.1 V 850 mAh, **60 × 30 × 20 mm** (listing), **XT30** plug (yellow, from the photo; confirm), JST-XH balance, 14 AWG leads | Robu (Pro-Range brand) | Stands upright at the keel front (bay raised 3 mm for its 60 mm, 2026-11-19). Leads + balance lead leave its top, go out of the rear-right window to the **rear cradles**: its main leads must reach **~75 mm** from the pack's top to the XT30's face (balance lead ~70 mm) - **check before buying**: a pack with 8-10 cm leads. **Measure the pack** (thickness ≤ 20.5, height ≤ 60.5) |
| 5 | FINAL (2026-11-28) | **UBEC-8A** switching regulator (user's listing; replaces the Mini560) | 1 | **55 × 28 × 10 mm, 36 g** (listing - **measure**: body, lead lengths, the jumper header), 6–36 V in (2–8S), **8 A continuous / 15-16 A burst**, output 5.2 / 6.0 / 7.4 / 8.4 V by a jumper, **two output leads in parallel** (servo plugs), an output switch on a lead | user's listing (not priced) | **Set the jumper to 5.2 V** (the servos' torque / power model is at 5 V; 6.0 V is within the SCS0009's range and gives ~20 % more torque - user's call, never 7.4 / 8.4). Only the servos run from it (the Uno Q takes the switched battery on VIN, as before). Sits in a **pod on the outside of the bay's rear wall** (printed with the container; vents in its back, floor and the lid's extension over it; the lid holds it down, no screws): the only place clear of every in-limit leg / neck / head motion (>= 16 mm) and 23.5 mm off the floor in SIT. Wires: IN+/- (bare ends) -> splices SW+ / BAT- (**~110 mm** needed), OUT: **both** output leads, plugs cut off, stripped, twisted per pole into the servo adapter's power terminal (**~155 mm** needed). Its own output switch: see REVIEW.md (decision open) |
| 6 | FINAL | **Waveshare Bus Servo Adapter (A)**, SKU 25514 | 1 | 42 × 33 × 14.6 mm, UART 3.3 V, 2 × 5264 sockets | [ThinkRobotics](https://thinkrobotics.com/ar/products/serial-bus-servo-driver) ₹500 | Jumper on **A** (UART). Power + UART wires **soldered on the back**. Rated 7(9)–12.6 V in: **test at 5 V on arrival** |
| 8 | FINAL | **7Semi BNO086 9-DOF VR IMU Qwiic Fusion Breakout** | 1 | 27.89 × 20.87 mm (7Semi datasheet ES-12243), ~6 mm with its Qwiic sockets, 2 × Qwiic, I²C 0x4B (0x4A by jumper), 3.3 V | [Robocraze](https://robocraze.com/products/bno086-vr-imu-9-dof-orientation-qwiic-fusion-breakout-7semi) ₹2,294 | **Bay v2 (2026-11-27):** stands on its end (long side vertical) in two grooves on the column's rear wall, slid down until it wedges - no screws, pegs or glue; the Qwiic socket used is the top one. **Check the board is 1.6 mm thick** (the wedge is sized for it). Set its orientation in software. Same software as BNO085/BNO080 |
| 9 | FINAL (lever protrudes, user OK 2026-11-14) | **Mini toggle switch MTS-102-A2T** - its lever sticks out of the left wall (listings: ~4-8 mm; the model had the body only). Leg clearance to the lever (exact parts): sit/stand 6.2 / 4.4 / 2.9 / 2.3 mm at 2 / 4 / 6 / 8 mm proud; with the left leg swung fully back (hip pitch +85) the foot bearing plate reaches it from ~4.5 mm proud. **Measure the lever from the bushing face to the tip on arrival**: if it stands > 4 mm proud, limit hip pitch back to ~+60 / -60 (not used by SIT +70 with the knee tucked, nor walking +-46). (ON-ON, 3 pins; Daier MTS series) | 1 | 6 A @125 V AC, body 13.2 × 7.8 × 9.5, 6 mm bushing | [Robu](https://robu.in/product/mts-102-a2t-miniature-toggle-switch-3-pinon-on/) | In a recess in the bay's left side wall; the body sits in the recess and the **lever stands out of the wall** (user OK 2026-11-14). Wire **battery + → middle pin → one outer pin** (the other outer pin unused) = plain on/off. Do not use the tiny SS-12F15 slide switch (0.5 A) |

## Cables and connectors

| # | Status | Part | Qty | Notes |
|---|---|---|---|---|
| 10 | FINAL (with the servos) | Kit 3-way servo hubs (5264) | 13 | Come in the SCS0009 packaging (user). Designed for: board 20 × 17, ~24 × 20 × 7 with sockets, plug 9.5 × 10 × 5.5. **Check the real hub against these on arrival** and count: 13 needed |
| 11 | FINAL (with the servos) | 3-pin servo extension, 165 mm (kit) | 12 | Come with the servos (user). Count on arrival: 12 needed, plus the separate 250 mm one (row 12) |
| 12 | FINAL | 3-pin servo extension (5264), **250 mm** | 1 | Adapter → left thigh (runs over the battery, down the deck slot) |
| 13 | FINAL | **Camera's own USB cable** (USB-A male → JST-SH, with the camera) + **rigid right-angle USB-C (male) → USB-A (female) OTG adapter** (user's part, photo 2026-11-19) | 1 + 1 | **Bay v2 (2026-11-27):** the Uno Q's USB-C is at the bay's FRONT; the adapter turned sideways (socket to the right), the camera's USB-A plug inside the bay, its cable out of the notch in the right wall to the neck. **Plug the cable into the adapter and the adapter into the Uno BEFORE the Uno goes into the bay** (no room to plug them in place). **Measure** the adapter (assumed 10 × 17 × 22) and the plug overmold (assumed 16 × 8, 20 beyond the adapter) |
| 13b | FINAL (2026-11-15) | **Braided expandable cable sleeve, PET, 6 mm** (3-9 mm range) | 1 m (uses ~30 cm) | Over the cables where moving parts rub them (cable pinch check, ASSEMBLY.md step 4): each hip (hip-roll lead + body-to-thigh extension, ~6 cm from the yaw connector to the thigh, round the deck's rounded lower edge) and the neck (U-to-head extension + camera USB, ~10 cm across the head-pitch U and yaw block). Seal the cut ends with heat or a turn of tape. Not priced |
| 14b | FINAL (2026-11-25) | **Female-female jumper wires 2.54 mm, 20 cm** (plain Dupont housings, the user's) | 8 + spares (2026-11-28: the UBEC's own leads replace the 4 Mini560 jumpers) | switch COM + ON (2), Uno Q JANALOG 6-7-8 (3) and D0 / D1 (2), adapter UART (3: GND, TX, RX - header type open). The ends into the adapter's screw terminal and into the 3 splices are cut and stripped. Replaces the right-angle header of 2026-11-19: the plain housings stand 14 mm over the Uno Q's headers (hood tunnel / open front) (the Mini560 they also served is replaced by the UBEC, row 5) |
| 14 | FINAL | **Adafruit STEMMA QT / Qwiic JST-SH 4-pin cable, 100 mm** (Adafruit 4210) | 1 (buy 2) | IMU → Uno Q Qwiic (bay v2: from the Uno's socket at the rear, along the right wall, down onto the IMU's top socket - 92 mm route). [Robu](https://robu.in/product/adafruit-stemma-qt-qwiic-jst-sh-4-pin-cable-100mm-long/) ₹129 (alt. [Zbotic](https://zbotic.in/product/adafruit-stemma-qt-qwiic-jst-sh-4-pin-cable-100mm-long/) ₹145). Symmetric, either end goes either way. Keep the heat-shrink sleeve in the middle out of the lid notch. Female JST-SH both ends; red 3.3 V, black GND, blue SDA, yellow SCL. Buy 2 (cheap; the tiny latches break). Check whether the 7Semi board ships with one. The plug at the IMU end has ~6 mm beside the board: use the IMU's Qwiic socket on the right (−y) side |
| 15 | FINAL | **XT30 MALE pigtail, 14 AWG silicone, ~15 cm** | 1 | Mates the battery's XT30, which lies along the outside of the bay's left wall (bay v2, 2026-11-27); the pigtail comes back in through the left-wall slot: red -> splice -> jumper to the switch's COM lug; black -> splice -> UBEC IN- + Uno Q GND. 3 heat-shrink splices (14 AWG to 2-3 jumper wires each); no 20 / 22 AWG wire needed any more (jumpers throughout) |
| 16 | FINAL | **IMAX B3AC compact balance charger, 2S–3S LiPo** ([Robu](https://robu.in/product/b3ac-compact-balance-charger-2s-3s-lipo/)) | 1 | Charges through the battery's **white balance plug only** (3 × 800 mA, about 1–1.5 h for the 850 mAh pack). Basic and unregulated by display: Robu itself recommends it only for light use. **Never charge unattended; charge in a LiPo bag** (row 25). Upgrade path: SkyRC IMAX B6AC V2 |

## Mechanical

| # | Status | Part | Qty | Notes |
|---|---|---|---|---|
| 17 | FINAL | **Ball bearing 6702ZZ** (= 61702-2Z / 61702ZZ; 6702-2RS rubber-sealed also fits) | 10 | **bore d = 15 mm, outer D = 21 mm, width B = 4 mm**, deep groove, double metal shield; ~4 g each | **Amazon.in 10-pack, ₹1,731** (₹173 each; the "6702ZZ 15x21x4mm" option of a 6700-6704ZZ listing, user's pick 2026-11-17); [cart2india uxcell 10-pack](https://www.cart2india.com/track-runner-bearings/uxcell-6702zz-deepgroove-ball-bearings-for-rollers-15mm-x-21mm-x-4mm-seal-bearings-with-fiber-gloves-double-metal-sealed-bearing-steel-skateboard-bearings-10pcs/00000000004390703266) ~₹2,600; bicycle shops (6702-2RS) | Cap joints: knees, hip pitch / roll / yaw (×2 legs), head yaw, head roll. Printed seat Ø21.1 with a 1 mm lip on the outer ring; rotor boss Ø14.95 in the bore, shoulder Ø16.6 on the inner ring. A spare or two is advisable |
| 18 | FINAL | **Ball bearing 693ZZ** (= 693-2Z; 693-2RS also fits) | 4 | **bore d = 3 mm, outer D = 8 mm, width B = 4 mm**, deep groove, double metal shield; ~1 g each | 4-pack listing (user's pick 2026-11-17, 3 x 8 x 4 mm, "693ZZ"): exactly 4, no spare - a second pack or a 10-pack advised (small, easy to drop or damage when pressed); [Quartz Components](https://quartzcomponents.com/products/693zz-ball-bearings-miniature-carbon-steel-bearings-3mm-x-8mm-x-4-mm-deep-groove-bearing); any 3D-printer shop | Fork joints: both ankles, neck, head pitch. Seated in the printed bearing plate, running on an M3 × 8 screw through the island (row 22) |
| 19 | FINAL | **M2 × 6 machine screw, socket head** (Robu datasheet: dk 3.7, k 2.0, 1.5 mm Allen) | **26** (2026-11-27, bay v2: the Uno Q cover's 2 and the Uno Q's 1 are gone; was 29) (+ 2 per further full-horn joint, see below) | Everything that is not a servo, plus ONE servo ear (left hip-yaw servo, front ear: under the neck fork, reachable only at 20-25 deg: socket head + ball-end key): lid 4, dome 4, camera 3, (IMU: none since 2026-11-20 - pegs + hot glue), bearing plates (neck 2, head pitch 2, feet 4) 8, thigh covers 4, Uno Q cover 2, **Uno Q 1** (the hole next to the digital header: an M3 head overlaps the header housing by 0.7 mm, this head clears it by ~0.2 mm; 2026-11-15), jaw pivot 1, **full-horn bearing caps, 2 per joint** (knees done 2026-11-13; all the other bearing joints stay trimmed (no room for a cap); plain heads in clearance holes so the cap centres on its bearing before it is clamped) - machine screws threaded straight into the printed Ø2.0 pilots (2026-11-27: holes +0.4 after the user's test; were 1.6 / 1.7) (user 2026-11-13: few screw types, all standard, all from Robu). Buy **4 × [Robu EasyMech M2 × 6 socket head + nut, 12-pc set](https://robu.in/product/easymech-m2-x-6mm-socket-head-cap-allen-bolt-nut/)** = 48 (3 bought = 36 covers the 29 now with 7 spare, and their nuts give the camera visor's 2 M2 nuts; 1 more pack if further joints get caps; the nuts are spare). All heads checked in the model at full size (see below) |
| 20 | FINAL | **M2 × 10 countersunk machine screw, Phillips, SS304** | 3 + 3 = **6** (2026-11-27: the switch plate's 2 are gone with the v1 bay) | **Camera visor (3, 2026-11-20)**: from the front through the visor and the board, heads flush (2 into M2 nuts behind the board's left end, 1 into a boss). **Switch plate (2, 2026-11-20)**: from outside through the bay's left wall into the plate, heads flush with the wall (8.3 mm of thread in the plate). **Bay to body (3), up from under the robot** through the tongue into the deck (2026-11-20; were driven inside the bay through its rear wall), heads flush in a **90° countersink** (2026-11-13: was a flat Ø4.0 counterbore, on which a countersunk head only bears on the hole's edge; found when the screw audit's axis bug was fixed). [Robu EasyMech SS304 CSK Phillips M2 × 10, 25 pcs](https://robu.in/?p=337115) ₹99 (Robu has no M2 × 6 CSK): 1 pack, 17 spare. (Not used for the bearing caps: a countersunk head would pull the cap onto the printed pilot instead of letting it centre on the bearing.) PH1 screwdriver |
| 21 | **CHANGED 2026-11-25** | **M2 × 6 button head machine screw** (ISO 7380, A2 stainless; head **Ø3.5 × 1.3 max** - check the listing; hex key 1.3 mm) - replaces BOTH kit screws (rows 21a / 21b) | **44** (15 horn + 29 ear) - buy 50 | **Horns** (14 joints + beak): every seat is 2.2 mm from the head seat to the spline tip and the shaft hole is 5.0 deep: 3.8 mm of thread (audit 3.1-3.3), 1.2 short of the bottom; fits all 15 seats incl. the hip yaws (0.38 mm over the head there - the row-19 socket head does NOT fit those two). **Ears**: into the Ø2.0 printed pilots (2026-11-27; were 1.7) like every other M2 here (1.6 through the ear + 4.4 into the pad); same head envelope the model always used. Shaft thread: **M2 machine thread, confirmed 2026-11-25** (the user threaded an M2 × 6 into a servo's shaft by hand). Model stand-ins rebuilt to it (`parts/horn_screw_standins.py`) |
| 21a / 21b | NOT USED (2026-11-25) | Servo's own screws: 1 horn screw (machine, ~3.5 mm under the head from the user's photo) + 2 mounting screws (self-tapping, washer head, ~7.5-8 mm) per servo | 0 | Horn screw: ~1.3 mm of thread in the shaft - the coupon's screw did not reach. Mounting screws: 21 of the 30 ear pads take at most 7.45-7.55 mm (pilots end 0.75 mm before the bearing lip in half of them: forcing a longer one can crack through), and the washer head is bigger than the Ø3.5 x 1.3 room at the head-roll (0.28 mm) and beak (0.13 mm) ears. Keep them as spares |
| 22 | FINAL | **M3 × 8 socket head cap screw, SS304** | **4** (2026-11-27, bay v2: the Uno Q has no screws; was 7) | 4 × axles of the 693 bearings (through Ø3.7 holes). [Robu EasyMech SS304 M3 × 8, 25 pcs](https://robu.in/product/easymech-ss304-m3-x-8-mm-socket-head-cap-allen-bolt-25-pcs/) ₹89 (user's pick 2026-11-13). 2.5 mm Allen key |
| 22b | FINAL | **M3 hex nut, white nylon** (AF 5.5, 2.4 thick: exactly the captive pocket) | 4 | On the 693 bearing axles (ankles, neck, head pitch): light axial clamp only, the steel screw carries the bearing's load. [Robu white nylon nut M3, 25 pcs](https://robu.in/product/white-nylon-nut-m3/) (user's pick; the steel nut pack is out of stock). **Tighten snug only** (nylon threads strip) |
| 23 | ESTIMATED | Servo horns: the kit **X (4-arm) horn**, 1 per servo | 15 | Come with the servos. Pockets drawn from Feetech's kit photo scaled by the listing (23.88 tip to tip; **measured 24.0** by the user 2026-11-20 -> full-horn pockets keep 0.24 at the tips, fits): arms 5.2 -> 4.3 wide (+-0.15), holes at r 4.9 / 7.4 / 9.8; hub Ø6.5 x 4.0 and plate 1.6 still nominal. **Full horn, uncut** (user 2026-11-13): ankles, neck pitch, head pitch (as before) + **knees (done)**. **Hip pitch and hip roll stay trimmed to r 6** (trim jig, 4 servos; no room for a cap - user 2026-11-13). Head yaw, head roll and hip yaw stay trimmed (no room for a cap, 2026-11-13). **In the model (read from the servos): full horn on 6 (knees ×2, ankles ×2, neck pitch, head pitch), trimmed to r 6 with the jig on 9 (hip yaw / roll / pitch ×2, head yaw, head roll, and the beak servo's jaw arm - beak stays trimmed, user 2026-11-13).** All 15 pockets checked against the horn model 2026-11-14 (full: 0.30 at the tips, 0.19 sides; trimmed: 0.20). Check on arrival (caliper or a printed pocket test) |
| 23b | FINAL | **Tools**: 1.5 mm hex key **with a ball end** (+ a plain one), 2.5 mm hex key, PH0 + PH1 screwdrivers (shaft ≤ Ø3), caliper | 1 set | Ball end: the left hip-yaw front ear screw is reached at 20-25 deg. Shaft ≤ Ø3: the camera and front dome screws are driven through Ø4.6 holes |
| 24 | - | Hot glue, foam tape, **zip ties 2.5 mm (bay v2: 1-2 to tie the battery lead, pigtail and balance lead at the left-wall slot; + spares)**, a **foam pad** (servo adapter's top front against the battery), heat-shrink 2-6 mm, Kapton tape | - | Cradles, Uno Q power plug, switch, hubs, cable slack; tape over the adapter's front UART pins |
| 25 | **NEEDED (tools, not in the robot)** | LiPo cell checker / alarm (1–8S, shows each cell's voltage, plugs on the balance lead) + LiPo fire-safe bag | 1 each | No protection electronics on board (user, 2026-11-18): check the cells before every charge (any cell < 3.0 V -> do not charge, retire the pack); store / charge in the bag. Rules: ASSEMBLY.md section 5 |

## Printed parts

PLA (75% fill assumed in the mass model). Also print the **horn trim jig** (0.4 g, `joint_v2.trim_jig`) for the 9 horns that are cut short (ASSEMBLY.md step 0.4). Feet print as single pieces with the sole merged. Thigh covers are optional. **Print the
hole gauge first** (same printer and profile; tools/blender/parts/hole_gauge.py: each hole size with -0.2 / +0.2). Printer (2026-11-27): **Bambu Lab
A1, 0.4 nozzle, "0.12mm High Quality @BBL A1", Bambu PLA Basic**, 3 walls, 7 top / 7 bottom layers, 40 % gyroid, tree supports, slicer hole
compensation 0 (it is in the model) - projects/xgoduck_scs0009/data/build_args/slicing.json. The bay v2 container prints upright (supports under the
top box's overhang, outside only); its lid prints top face down (no supports). Robot mass with
everything: **665.7 g** before the UBEC (2026-11-28: UBEC 36 g + pod / lid ~7 g as printed - Mini560 ~4 g = **~705 g**, not yet in the sim; the trunk's centre of mass moves ~3 mm back) (sim `xgoduck_scs0009_v2h`, `sim/body_mass_v2.json`; the dome's stored mass is 0.8 g low - its ear fill
came later - so the model is ~0.6 g light).

STL files (mm, as modelled; one closed piece each, checked on export): `exports/xgoduck_scs0009/print/` + `manifest.json`
(`tools/blender/export_print.py`). Solid-PLA masses (real prints are lighter by the infill):

| Part | Qty | g solid | Part | Qty | g solid |
|---|---|---|---|---|---|
| body_frame | 1 | 19.1 | left / right_foot | 1 + 1 | 32.2 each |
| bay_v2_container (2026-11-28: + UBEC pod) | 1 | 75.1 | left / right_foot_bearing_plate | 1 + 1 | 1.9 each |
| bay_v2_lid (2026-11-28: + pod cover) | 1 | 17.8 | left / right_knee_cap | 1 + 1 | 2.9 each |
| hole_gauge (test piece, print first) | 1 | 14.5 | left / right_l_bracket | 1 + 1 | 6.4 each |
| body_neck_bearing_plate | 1 | 1.6 | left / right_shin | 1 + 1 | 9.4 each |
| neck_cradle | 1 | 7.7 | left / right_thigh | 1 + 1 | 9.9 each |
| head_pitch_u | 1 | 9.8 | left / right_yaw_connector | 1 + 1 | 5.4 each |
| head_pitch_bearing_plate | 1 | 2.2 | left / right_thigh_cover (optional) | 1 + 1 | 8.1 each |
| yaw_block | 1 | 6.6 | horn_trim_jig_r6 (tool) | 1 | 0.4 |
| head_base | 1 | 20.3 | | | |
| head_dome | 1 | 24.4 | | | |
| head_face (2026-11-27: camera vents) | 1 | 7.1 | | | |
| head_jaw | 1 | 6.7 | | | |
| head_visor (2026-11-20) | 1 | 2.5 | **Total** (29 parts incl. the optional covers, + jig, without the gauge; re-read from the model 2026-11-27) | | **354 g solid** (2026-11-28 UBEC pod +9.7; was 317 before bay v2) |

**Tolerance coupon (designed 2026-11-14, print first):** 7 small pieces cut by the same joint-library code as the robot -
knee rotor (6702 seat, full horn pocket + crush ribs), knee cap centre, trimmed-horn plate, a bar (693 seat, M3 nut trap, M2 pilot /
clearance, M3 pilot, 28 mm bridge), a servo collar, and (2026-11-22) the lying-down boss twice (with / without supports); 19.6 g solid PLA in all (`tools/blender/parts/tolerance_coupon.py`,
collection `coupon`). Test procedure and the constants to adjust: ASSEMBLY.md step 2. Not counted in the total above.

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

## Cost estimate (India, checked 2026-11-14)

From the links in this BOM and web-search snapshots of the listed shops (Robu blocks automated page reads, so some Robu prices
are from cached copies): **check the live page before ordering.** Incl. GST unless noted; shipping not included.

| Row | Item | Qty bought | Price | Line total | Source |
|---|---|---|---|---|---|
| 1 | SCS0009 | 15 | ₹864 (Evelta; Hubtronics ₹799 as "SC-0090-C001") | **₹12,960** | evelta.com/scs0009-6v-2-3kg-300deg-serial-bus-servo-motor |
| 2 | Arduino UNO Q 2 GB | 1 | ₹4,652 (Robocraze, may be pre-order); MakerBazar ₹4,899 | ₹4,652 | robocraze.com/products/official-arduino-uno-q-sbc |
| 3 | OV5640 USB Camera (B) | 1 | ₹2,530 (Hubtronics); Evelta ₹2,602 | ₹2,530 | hubtronics.in/ov5460-usb-camera-b |
| 4 | Pro-Range 850 mAh 3S 30C | 1 | ₹1,107 (Robu product page; category page ₹1,159) | ₹1,107 | robu.in/product/orange-850mah-3s-30c-60c-lithium-polymer-battery-pack-lipo/ |
| 5 | UBEC-8A (user's listing; replaces the Mini560 ₹93) | 1 | - | - | user (row 5) |
| 6 | Bus Servo Adapter (A) | 1 | ₹500 | ₹500 | ThinkRobotics (row 6) |
| 8 | 7Semi BNO086 | 1 | ₹2,294 | ₹2,294 | Robocraze (row 8) |
| 14 | Qwiic cable 100 mm | 2 | ₹129 | ₹258 | Robu (row 14) |
| 16 | IMAX B3AC charger | 1 | ₹353 (cached) | ₹353 | robu.in/?p=7364 |
| 17 | 6702ZZ 10-pack | 1 | ₹1,731 (Amazon.in, 2026-11-17; cart2india ₹2,600) | ₹1,731 | Amazon.in (row 17) |
| 20 | M2 × 10 CSK, 25 pcs | 1 | ₹99 | ₹99 | Robu (row 20) |
| 22 | M3 × 8 SHCS, 25 pcs | 1 | ₹89 | ₹89 | Robu (row 22) |
| 22b | M3 white nylon nut, 25 pcs | 1 | ₹49 | ₹49 | robu.in/product/white-nylon-nut-m3 |
| | **Priced subtotal** | | | **₹26,715** | |

Not priced (no listing price found): M2 × 6 socket sets × 4 (row 19), 693ZZ × 4 (row 18), MTS-102 switch (row 9; Robu's ₹30
**SMTS-102** is the sub-miniature size - not the 13.2 × 7.8 body the recess is drawn for), T-plug male pigtail (row 15; the female
version is ~₹92 elsewhere), 250 mm 5264 servo extension (row 12; only JR/Futaba-plug extensions found, ₹55-85 - not the same
plug), USB-C cable ≥ 40 cm (row 13), hook-up wire, LiPo alarm + bag (row 25), PLA (~250 g at 75 % fill). Rough allowance
**₹1,000-1,500** -> **about ₹27,700-28,200 for one robot**, before shipping and spares (2026-11-17: the 6702ZZ pack ₹1,731 instead of ₹2,600).

Suggested spares (not decided): 2 × SCS0009 (~₹1,730) - the servos are what a first build strips or burns; 2-4 × 6702ZZ (the
10-pack is exactly the 10 needed; singles not found in India, a second 10-pack is ₹1,731 on Amazon.in); a second 693ZZ pack.

## Not used any more

Distance sensor Adafruit VL53L1X + its chest plate and cable (dropped 2026-11-12: the camera does obstacle sensing) ·
USB sound card, speaker, USB hub (audio dropped 2026-11-12) · Hobbywing UBEC 5A (does not fit; Mini560 instead) · Mini560 5 V (2026-11-28: replaced by the UBEC-8A, row 5) ·
UBEC-8A (better, does not fit inside) · lengthened battery bay.
