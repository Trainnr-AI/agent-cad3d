# Handoff — where the project stands (updated 2026-11-18)

> **Design CLOSED for the first build (2026-11-18).** Parts, limits, assembly and power are final; next is physical: send
> `projects/xgoduck_scs0009/print/test_batch/` to the printer, measure the coupon with the real bearings / screws / servo, then
> one leg, then the whole robot. Order: BOM.md (bearings: Amazon.in picks rows 17-18). Build: ASSEMBLY.md (incl. section 5,
> battery rules - the robot has no protection electronics by choice). Firmware limits: `sim/joint_ranges_v2.json` (+ hip map).
> Re-check anything you change with `scripts/full_check.sh`.

Read this first when continuing on another machine. It replaces the agent's local notes, which
don't travel with the repo.

## What this repo is

An agentic 3D-design workspace: Claude Code drives a live Blender session (MCP for Blender add-on,
or `scripts/blender_doctor.py` over the same socket) while you watch and take over. First project:
**XGO-Duck redesigned for Feetech SCS0009 servos** (`projects/xgoduck_scs0009/`).

## Resume on WSL2 (Ubuntu)

```bash
git clone git@github.com:Trainnr-AI/agent-cad3d.git && cd agent-cad3d
./scripts/setup.sh            # uv, Blender 5.2.3 (tarball -> ~/.local/opt), add-on, vendor/ sources
WAYLAND_DISPLAY= blender projects/xgoduck_scs0009/xgoduck_scs0009.blend &   # WSLg; X11 — native Wayland dies (EGL_BAD_MATCH)
python3 scripts/blender_doctor.py                              # should print "Blender 5.2.3 LTS" + objects
claude                                                          # approve the 'blender' MCP server
```

- **Port 9877.** The add-on's server port is saved in the project .blend (9877, because the trainnr
  Studio viewer needs 9876). A new/other .blend defaults to 9876: set 9877 in the N-panel → MCP for
  Blender, or `BLENDER_PORT=9876 python3 scripts/blender_doctor.py`.
- **Blender on the Windows side instead of in WSL:** start it there, then
  `BLENDER_HOST=$(ip route show default | awk '{print $3}') python3 scripts/blender_doctor.py`
  (or enable WSL mirrored networking so `localhost` works). Set the same host in `.mcp.json` env.
- **Long Blender jobs** (booleans, sweeps): `BLENDER_TIMEOUT=1800 python3 scripts/blender_doctor.py ...`.
- **Simulation** needs only `uv`: `uv run --with mujoco python -I tools/sim/servo_tests.py ...`.
  Native viewer: `uv run --with mujoco python tools/sim/mujoco_showcase.py <scenes>` (on Linux plain
  python works; `mjpython` is only needed on macOS).
- **trainnr** (separate plugin, local projects under `~/trainnr/projects`): on the new machine
  `create_project xgoduck_scs0009`, then `onboard_robot` each scene below. Studio: `launch_studio`.
  Stream test replays into it with `tools/sim/rerun_replay.py --showcase --url rerun+http://127.0.0.1:9876/proxy`.

## Decisions (with reasons)

| Decision | Why |
|---|---|
| All 15 joints SCS0009 | user choice; SCS0009 is ~4.4x weaker than the original HD-1910 (2.3 vs 10 kg·cm stall) |
| **Uniform 0.92x** scale (~240 mm tall) | motion must match the original exactly (same axes, directions, ranges, proportions). 0.917 is the smallest uniform scale where the servos fit (hip-roll/yaw horns touch below it) |
| Head shell 0.85x + servo gain "full torque at 1° error" (kp≈13 N·m/rad) | with the full head the neck saturates and the duck falls; both changes needed (sim-verified). Confirm the real SCS0009 can be tuned this stiff |
| Head yaw ±150° (original ±170°) | SCS0009 travel is 300° |
| Horn: kit X horn **trimmed to r = 6 mm**, in a pocket with 16 tapered **crush ribs** (0.25 mm interference), one M2x6 machine screw into the shaft | a printed pocket alone leaves 2-3° play; the disc horn's ~1 mm holes can't take screws; trimmed so it fits inside the 16x22x4 bearing bore |
| One-sided joints (hip yaw/roll/pitch, knee, head yaw): **original style** — 16x22x4 bearing around the output, outer ring in the holding part's cap plate | matches the original (assembly guide steps 05/09/12) |
| Two-sided joints (neck pitch, head pitch, ankle): **Ø10 pin printed on the part that holds the servo** (coaxial by construction) + **separate bearing plate** with a 10x15x3 bearing, screwed to the moving part | SCS0009 has no rear shaft, flat bottom. A one-piece U can't be assembled (its far arm would pass through the servo) — verified with `assembly_path_check.py` |
| Servos mounted only by their two ears (pads + M2 self-tap) | SCS0009 has no case screw holes |
| Power: decide later; leave room for a 6 V buck | SCS0009 max 7.4 V, original pack is 8.4 V |

## Servo model (2026-11-10): exact SCS0009 from Feetech's CAD

`tools/blender/parts/scs0009.py` now builds the servo from Feetech's own STEP (assembly
SCS009-20201118-S, `vendor/feetech/`, fetched by `fetch_vendor.sh`; converted by
`tools/cad/scs0009_from_step.py` into `parts/scs0009_mesh/*.stl`, 5 µm tolerance, committed). Object
names are unchanged (`<name>_case`, `<name>_spline`, `<name>_horn_<type>`); new: `_case_screws`,
`_horn_screw`, optional `_ear_screws`. Facts that differ from the old box model (all read from the B-rep):

- Spline: 20 T, OD 3.95, **root Ø3.45**, teeth 25.95–28.45 above the bottom (0.29 tip chamfer); Ø3.45 neck below.
- **Boss Ø5.76 × 0.3 on the tower top** (25.55) with a tongue toward the far end; horn hub sits on it.
- Roof: flat 21.45 only to x = 20.84 (datasheet x), then slopes 6.5° down to ~20.7 at the connector end.
- Ear underside recessed 0.6 inside a 0.7 rim → pads touch only the rim.
- **Cable leaves the shaft-end face** (bottom, centred, ~3 × 2.3 slot) — `neck_orig_style.py` cuts its
  cable notch at the far end: wrong, fix when the neck is regenerated.
- Case screws: 4 from the bottom at x = 5.85 / 26.65, y = ±4.8 (Ø2.2 counterbores; ~Ø1.0 × 20).
- Kit horn lengths: cross 23.88 tip-to-tip, disc Ø17.76, double arm 34.49, single arm 20.12.
  Horn thickness/hub still NOMINAL (`horn["nominal"]`). Not in the STEP: screws (sizes inferred), horns, motor/PCB.

## Joints v2 (2026-11-10)

Knee (type C: cap + 16x22x4 + trimmed horn) and ankle (type F: captive M3 nut + 693ZZ axle + full
horn) redesigned on the exact servo, verified (REVIEW.md "Joints v2"). New tools: `parts/joint_v2.py`,
`parts/leg_v2.py`, `print_check.py` (overhang area per orientation), `refresh_servos.py` (rebuild the
placed servos from the current model), `snap_servo_axes.py` (axes exactly on the joints; ankle servos
slid 6.37 mm so the shin is one flat plate). Collection `leg_v2` (left: thigh, shin, foot, bearing
plate, hardware dummies, trim jig); `leg_v1` still holds the hip parts and the superseded v1 leg.
BOM v2: **6702ZZ (15x21x4) x9** (replaces the niche 16x22x4), **693ZZ x4, M3x8 + M3 nut x4** (replace the 10x15x3), kit M2 self-tappers for
ears, M2x6 machine screws for horns. Knee usable range +85° (collision at exactly +90°, as the original).
Rolled out to both legs' hips and the neck (`parts/neck_v2.py`, collection `neck_v2`); two defects
still open (right shin manifoldness, hip-roll servo vs yaw connector 24 mm3) — see REVIEW.md
"Rollout". Branch `joints-v2`.

Boolean hygiene (what finally made every part clean): combine simple cutters first and cut once
(`combine_cutters`, `pocket_cutter`); never leave a face flush with another part face; keep round
cutters on different vertex-angle grids; `print_check.py` before calling a part printable. To locate a
collision: pose the body (`matrix_basis @ Quaternion(axis, angle)`), BVH `overlap`, print the hit
triangles' centres in the servo frames.

## Current state

Blender file `projects/xgoduck_scs0009/xgoduck_scs0009.blend` shows four labelled groups:
original duck, new duck (assembled), new duck exploded, neck exploded. Old experiments are in the
hidden collection `zz_archive (hidden)`.

**Designed (v1, functional flat-plate geometry, all one solid watertight piece):**
- legs (`leg_v1`, `tools/blender/parts/leg_orig_style.py`): hip_yaw_support (shared), yaw_connector,
  l_bracket, thigh, shin, foot + foot_bearing_plate — per side
- neck (`neck_v1`, `tools/blender/parts/neck_orig_style.py`): neck_cradle, body_neck_mount +
  bearing plate, head_pitch_u (its top plate is the head-yaw moving plate) + bearing plate

**Verified:** horn pockets 1.0-1.9 mm³ rib press fit; servos touch cradles/holders without overlap;
every assembly slide clear; all 13 leg + neck joints through full range + 9 combined poses clear
except knee at full 90° (the original collides there too). Details: `projects/xgoduck_scs0009/REVIEW.md`.

**Simulation** (`projects/xgoduck_scs0009/sim/`):
- `scene_original.xml` — original duck, same servo-model structure (for like-for-like tests)
- `scene_scs0009_h0.85_e1.xml` — 0.92x, 0.85 head, stiff gain, original shells scaled
- `scene_scs0009_v1.xml` — **the redesigned duck**: per-body meshes exported from Blender (new parts,
  servos, bearings) for visual + collision. Holds every tested pose, incl. neck −135° where the
  original falls. Masses/inertias still come from the scaled model, not from the new parts.

## State after the macOS session of 2026-11-10 (branch `joints-v2`) - read REVIEW.md from "Rollout fixes" down for the detail

**Hardware decided (one screw type, standard parts):** 6702ZZ (15x21x4) x10 replaces the niche 16x22x4; 693ZZ x4;
M3x8 socket + M3 nut x4; **M2x6 pan-head self-tapping x43** (28 servo ears, 8 bearing plates, 4 top plate, 3 enclosure);
M2x6 pan-head machine x14 for the horns (shaft thread UNCONFIRMED: datasheet says "M2.0x4", type not given).
Plates and servo floors are 4.5 mm (the M3x8 axle then stops 0.5 short of the servo). Bearing-plate lip 0.8 mm.

**Printed parts (all one solid, watertight):** per leg yaw_connector, l_bracket, thigh, shin, foot, foot_bearing_plate;
neck_cradle, body_neck_bearing_plate, head_pitch_u, head_pitch_bearing_plate (`neck_v2.py`); yaw_block (`head_v2.py`:
holds the head-yaw servo, is the head-roll output; head roll is a CAP joint now); body_frame = motor body (deck with both
hip-yaw caps, neck pedestal + horn plate), electronics_enclosure, body_lid (`body_v2.py`). `hip_yaw_support` and
`body_neck_mount` are gone (absorbed into body_frame; the old mount cut into both hip-yaw servos).

**Electronics (off the shelf, nothing bought or tested):** Arduino Uno Q on top of the enclosure (fits nowhere inside),
Waveshare Bus Servo Adapter (A), Adafruit BNO085 (Qwiic), 6 V UBEC (Hobbywing 5 A), 3S LiPo (size ASSUMED 42x30x13),
kit hubs + extension cables: 13 hubs (6 body, 2 per thigh, 2 on the neck cradle, 1 on the U), assuming ONE adapter socket.
Accepted by the user: neck back limited to ~27 deg (cradle meets the Uno Q at 44; the original is clean to 36, hits at 60).

**Tools added:** `fit_audit.py` (overlap + gap per fit vs intended fit), `swept_space.py` (free-space map around the body),
`section_view.py` (cut-aways: aligned, in place, or through a point), `color_code.py` (colours + labels, any view side),
`explode_all.py` / `explode_list.py`, `wiring.py` + `parts/cable_kit.py` (cables in place, assembled or exploded).
Blender collections: `duck_v2_exploded` + `wiring_exploded` + `duck_v2_labels`, `sec_<joint>` (9), `body_exploded`,
`body_sec_*` (from the FIRST body attempt - stale), `cable_kit`, `cables` (8 mm exit stubs), `electronics`, `wiring`.

## State after the session of 2026-11-13 (branch `kit-hubs-original-shapes`) - REVIEW.md from "Full kit horn, uncut" down

- **Horns**: full kit X horn on 6 servos (knees - new type C2M: 6702ZZ in the shin + bolt-on cap with 2 x M2x6 socket - ankles,
  neck pitch, head pitch); trimmed to r 6 with the printed jig on 9 (hips, head yaw / roll, beak): no room for a cap there
  (measured with all coupled joints and both legs moving). Trim jig redesigned and tested against the horn model.
- **Joint limits**: `projects/xgoduck_scs0009/JOINT_LIMITS.md` - servo angle limits (hip yaw -6.3..+19, hip roll -12..+18.5, ankle -85..+90,
  neck -80..+30, head pitch -5..+65, head roll +-15), software interlocks (head roll only facing straight; walking-envelope
  corners), overload protection settings. Values in steps to be confirmed on a servo.
- **Body**: keel's lower outer edges chamfered for the knee caps; IMU pilots and bay countersinks fixed.
- **Head**: roll is free only facing straight (the U's corners under the floor) - user chose a software interlock; raising the
  head ~4.5 mm on a taller U centre would give +-10 at every yaw (not done). Yaw block goes in 4.8 mm forward; face front 2.5 mm.
- **Verified after the full rebuild**: 27 printed parts one piece; horns 15/15; fits 333; all assembly paths (leg built off the
  robot, hung on the hip-roll horn last); screws dispositioned; head room 77/77; cables 29/29; motion = known limits only.
- **Files**: `exports/xgoduck_scs0009/print/` 30 STL + manifest (git-ignored); ASSEMBLY.md (build order, horn trimming), BOM.md.
- **Sim** (re-exported from these parts, trainnr bundle `xgoduck_scs0009_v2i` = v2h + the new SIT; recordings in the trainnr Studio): stand, crouch
  0.3, shift, neck / head moves work. **Sit/stand**: new SIT hip pitch 70 / knee 18 / ankle -16 / hip yaw -3 / hip roll -4
  (leans back onto the enclosure's rear edge; the yaw/roll keep the feet from sliding), straight path from STAND over 3 s,
  exact parts clear >= 1.58 mm. Ideal model: passes. Against servo slack / delay, print mass and floors
  (`tools/sim/sit_stand_robust.py`): open loop 192/300 fall (STAND has only 16 mm to the heel edge) - **the firmware needs the
  IMU ankle balance** (ankle += -1.5 x trunk pitch error, error low-passed 25 ms, 0.2 deg dead zone) and must not sit facing
  uphill on > ~4 deg. REVIEW.md "MuJoCo on the rebuilt parts; sit and stand" and the section after it. The old SIT face-plants.
- **Next**: measure the real parts (horn, kit screws, battery, IMU holes, servo steps/deg and protection registers); print a
  tolerance coupon (6702 pocket, 28 mm bridge); then the gait inside the limits (no walking policy exists for this robot yet -
  trainnr's walk families are fixed rigs).

## State after the session of 2026-11-14 (main + PR "coupon") - REVIEW.md from "Build readiness pass" down

- **Build readiness pass** (all re-run on the current files): each of the 14 servos over its full servo range held in the air
  (8-29 % of stall); 17 floor motions upright (peak 62 %); every joint within its servo limits + 296 poses from those motions: 0
  collisions on the exact parts; parts 27/27 watertight, horns 15/15, fits 333, assembly paths, screws, printability unchanged.
  Beak servo limit **0..+35** added (closing past 0 hits the head base). Sweep the servo limits with
  `leg_motion_check.py` `ranges_deg` (from `sim/joint_ranges_v2.json`) - the Blender joint empties still store wider ranges.
- **Hip map** (`JOINT_LIMITS.md` "Hip map", `data/hip_collision_map.json`): 1,248 poses, hip pitch x roll x yaw x knee. The hip
  yaw/roll servo box holds only with the leg near straight down; toed in, a leg cannot swing back past ~+15. Right leg = mirror
  (40/40). A walking gait must stay inside this map (the original's walking envelope crosses it). Not covered: ankle angles,
  both legs at large pitch together.
- **Getting up after a fall: very likely not possible** with SCS0009 at 5 V (torque / (weight x leg length) 0.35 vs the
  original HLS1910's 1.47); the original's get-up is an RL policy (runtime repo), not vendored. Study (5 / 6 / 7.4 V) not run.
- **BOM** checked against the model (every bought-part count matches; horn screws 15), printed-parts table, cost estimate
  ~Rs 28,500-29,000 per robot (search snapshots; Robu blocks automated fetches - use web search). User adds shop links.
- **Power switch MTS-102**: the model had the body only; the real lever stands ~4-8 mm proud. **User: keep it protruding.**
  Leg clearance to the lever: sit/stand >= 2.3 mm at 8 mm proud; hip pitch +85 hits it from ~4.5 mm proud -> if the measured
  lever is > 4 mm proud, limit hip pitch back to ~+60 / -60. Add the real lever to the model once measured.
- **Tolerance coupon** (`tools/blender/parts/tolerance_coupon.py`, collection `coupon` in the .blend, STLs via
  `tools/blender/export_print.py` `{"collections":["coupon"], ...}` -> `exports/xgoduck_scs0009/print_coupon/`): knee rotor
  (full horn + 6702 seat), knee cap boss, trimmed-horn plate, bar (693 seat, M3 nut trap, M2 / M3 pilots, M2 clearance, 28 mm
  bridge), servo collar - 13 g, built from the same joint_v2 code as the robot. Test procedure: ASSEMBLY.md step 2.

## State after the session of 2026-11-15 (branch `review-fixes-2026-11-15`) - REVIEW.md "Independent review fixes"

An independent review (sister repo microduck-nano, `docs/project/design-review-2026-11-14.md`) re-ran our checks. Applied:
- **Head roll only within +-1.0 deg of yaw 0** (was +-2.5: 0.40 mm into the U at yaw -2.5 / roll -15).
- **Hip map per leg** (the right leg is NOT the left's mirror: forward 45-90 with the knee at its limit, the right foot hits the
  neck bearing plate, 6.9 mm), re-measured with a 0.3 mm margin; **both-legs interlock**: never both legs forward of -30 while
  both are toed out (feet collide, 8 mm). `data/hip_collision_map.json` (left, right, both), `JOINT_LIMITS.md`.
- **Uno Q**: the hole next to the digital header takes an **M2 x 6 socket** (any M3 head overlaps the header by 0.7 mm; user
  choice); lid pilot changed; BOM M2x6 36, M3x8 7.
- 14 duplicate horn-screw stand-ins removed; BOM stale text fixed.
- New tools: `tools/blender/clearance_sweep.py` (contacts + gaps under a margin, headless, ~1.5 s/pose; use it instead of
  `leg_motion_check.py` volumes for anything thin), `tools/blender/hip_map_table.py` (map JSON + tables).
- Found, not changed then: at the hip-roll limit +19.5 the hip-yaw bearing is 0.3 mm from the L-bracket (fixed 2026-11-18: limit +18.5).
- Deferred by the user then, decided 2026-11-18 (no protection electronics; regulator under a lid window): battery protection (fuse, protection board, low-voltage cut-off) and clearances, regulator
  placement. The review also found the robot can't stand on one foot (CoM 9 mm outside the foot at the roll limit, 74-77 % of
  stall on the hip roll): walking needs a gait study before it is counted on.
- Later the same day (REVIEW.md "Cables, ankle angles, knee x ankle"): cable reach OK; **cable pinch check** (new
  `tools/blender/cable_pinch_check.py`): 21 places where a moving part pushes a cable - the worst repeated one is in walking (hip-roll
  leads / body-to-thigh extensions on the body frame's edge, 2.3 mm), then the head cables at large head yaw. Not changed: needs
  clips / routing decisions. **Knee x ankle interlock** added (knee past +30 limits the ankle, 0 at knee 68); ankle +-23 hip
  sweeps added to the map file. Exploded views and STLs regenerated.
- Tool lesson: an inside test by ray parity alone gives false single-point hits on the exact servo meshes - both new tools now
  also require the nearest face to face away. Scripts run in the LIVE Blender must take paths from ARGS only: one that wrote to
  `sys.argv[-1]` overwrote the Blender executable (restored from the official 5.2.3 dmg, checksum verified).

## Design closed (2026-11-15, end of session) - REVIEW.md "Final changes and full check; design closed"

Last changes: deck's lower outer edge rounded r 1.5 for the hip cables (both sides), braided sleeve (BOM 13b) on the hip and neck
cables + a slack loop at the neck (ASSEMBLY.md step 4), Uno Q cover relieved round the M3 heads (0.49 mm). Full check pass clean
except the known, dispositioned items. **Re-run everything with `scripts/full_check.sh`** (headless, ~15-20 min; expected results:
`projects/xgoduck_scs0009/data/check_args/README.md`). Next is the physical build: coupon -> one leg -> full robot.
The test print package for the printing service (PDF guide + oriented STLs) is `projects/xgoduck_scs0009/print/test_batch/`.

## Final review (2026-11-18) - design and limits CLOSED for the first build

Full check suite re-run on the saved .blend: every result as expected (`data/check_args/README.md`). Changes:
- **Limits:** hip roll outward +19.5 -> **+18.5** and ankle -86 -> **-85** (left; right mirrored), so every servo-limit end clears
  by >= 0.5 mm (all 28 ends measured with `clearance_sweep.py`). `sim/joint_ranges_v2.json` is now the closed, machine-readable
  source: `joints` (beak 0..35 added) + `interlocks` (head roll x yaw, knee x ankle, both legs, hip maps, sit / stand); the v2 MJCF
  ranges updated (scene keyframes left as settled). **Firmware:** microduck-nano's `tools/import_design.py` reads this file and
  `data/hip_collision_map.json` - its import of the right-leg map / both-legs / knee x ankle (deferred by the owner "until the
  robot is final") can start; the `interlocks` block is new and not read by it yet.
- **ASSEMBLY.md:** new section 0 item 5 - where each of the 14 bearings goes (5 of the 10 6702ZZ had no step: hip roll ->
  yaw connector, hip pitch -> thigh, head yaw -> yaw block) and how to press them; Uno Q screws corrected (3 x M3 x 8 + 1 x M2 x 6).
- **BOM:** 6702ZZ Amazon.in 10-pack Rs 1,731 and a 693ZZ 4-pack (user's picks); estimate ~Rs 27,700-28,200.
- **Power (same day, user: "keep the most simple and effective assembly", nothing that is not essential):** no fuse, protection
  board or battery monitor; the regulator got a window in the lid (component side up, 1.2 mm from the walls); battery rules +
  a cell checker as a tool (ASSEMBLY.md section 5). Where a protection board would fit is recorded there. REVIEW.md
  "Battery protection and regulator placement".

## Bay with exact parts + camera cable in the head (2026-11-19, branch bay-exact-2026-11-19)

Real parts in: battery 60 x 30 x 20 + XT30 (bay raised 3 mm: `body_v2.json` pocket_top 128), Mini560 30 x 18 (maker's drawing; in a slot
under the lid, parts up through a window), the user's rigid right-angle USB-C -> USB-A adapter (mouth down, rear-left cradle), XT30 in the
middle of the rear face + balance plug above the bay at the rear right (unplug / charge without opening anything), switch nut captive in a
hex recess, right-angle header plugs on the Uno Q. New: `tools/blender/parts/bay_parts.py` (connectors + anchors), the exact bay harness in
`wiring.py`, `tools/blender/sheet_labels.py` (labelled sheet `data/bay_assembly_sheet.png`), `tools/blender/cable_path_search.py`.
Camera cable re-routed in the head; the face got room for the plug's wires and a slot in its D-panel (the camera could not be cabled
before). Every bay / head cable clears every part. To measure: the battery's leads (~75 mm needed), the USB adapter / plug, the Mini560's
IN end, the switch lever. REVIEW.md "The bay with exact parts..." and "Camera cable in the head...".

## Open items / next steps

1. **Staged build** (agreed): print the coupon -> adjust fits (constants in `tools/blender/parts/joint_v2.py`: SEAT_D, BOSS_D,
   HORN_CLEAR, RIB_INTERF, PILOT_D, CLEAR_D, BP_SEAT_D, NUT_AF, SADDLE_CLEAR) -> rebuild + re-check -> print one leg -> full build.
2. **Measure on arrival**: switch lever (bushing face to tip); one SCS0009 (steps/deg, backlash, deadband, delay, protection
   registers); horn and kit screws; battery thickness; IMU hole spacing (estimated). Feed the servo numbers into
   `tools/sim/sit_stand_robust.py` and re-run.
3. **Firmware** (none yet): servo limits incl. the beak, interlocks (head roll x yaw, hip map, sit path), IMU ankle balance
   for sit/stand, overload protection, fall detection -> torque off.
4. **Walking**: no gait for this robot; must respect the hip map. **Get-up** study (optional, decides whether hardware must change).
5. Not checked: fall loads on horns / one-sided joints (no retention clip, user decision); ankle beyond +-23 combined with the
   hip; both-legs check with the ankle moved.
7. Walking feasibility: the independent review's single-support check (one foot) fails - see the 2026-11-15 state above.
8. Cable routing: decide clips / guides for the chafe points of the cable pinch check (walking: body frame edge; head yaw: U,
   yaw block). The check models straight spans: confirm on the first assembled leg.
6. The trainnr robot `xgoduck_scs0009_v2i` has the earlier SIT angles (re-onboard as a new name if needed: onboarding never
   overwrites). `projects/xgoduck_scs0009/sim/assets_v2/` is an uncommitted duplicate of `sim/assets/` (delete freely).

## How the agent should work here (lessons)

- Verify numerically against the original, not by eye: `compare_kinematics.py`, `sweep_compare.py`,
  `leg_motion_check.py` (mesh-level sweeps + combined poses).
- Every printed part must be **one piece** (`L["shells"]`), not just watertight.
- Every joint must be **assemblable**: `assembly_path_check.py` slides each part along its axis.
- Overlap volume: use `overlap_sample.py` (ray-parity sampling) when a mesh is built from several
  overlapping pieces (servo horns) — the exact boolean INTERSECT gives garbage there.
- Booleans: avoid coplanar/coincident faces (offset cutters slightly); `build_part` heals tiny slivers.
- Screenshots: `tools/blender/closeup.py` renders from its own camera (viewport screenshots go stale
  when Blender is in the background).
- In zsh never name a loop variable `path`.

## Licensing

The duck's mesh-derived files (scaled copies of XGO-Duck / Microduck STLs in `sim/assets/`, the
.blend) come from models published non-commercial / share-alike (see vendor/xgoduck_rl and
vendor/xgoduck_hardware LICENSING). Keep this repo private; get permission before any commercial use.


2026-11-21: whole-robot assembly animation + path check (458 moves OK) - see projects/xgoduck_scs0009/REVIEW.md, last section:
switch plate, IMU on pegs, bay screws from below, camera visor + nuts, rear dome screws moved, assembly-order fixes (ASSEMBLY.md).
Head base / dome: do not rebuild from the original meshes (data/build_args/README.md, 4b).
