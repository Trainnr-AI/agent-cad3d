# Handoff — where the project stands (2026-11-09)

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
- **Joint limits**: `projects/xgoduck_scs0009/JOINT_LIMITS.md` - servo angle limits (hip yaw -6.3..+19, hip roll -12..+19.5,
  neck -80..+30, head pitch -5..+65, head roll +-15), software interlocks (head roll only facing straight; walking-envelope
  corners), overload protection settings. Values in steps to be confirmed on a servo.
- **Body**: keel's lower outer edges chamfered for the knee caps; IMU pilots and bay countersinks fixed.
- **Head**: roll is free only facing straight (the U's corners under the floor) - user chose a software interlock; raising the
  head ~4.5 mm on a taller U centre would give +-10 at every yaw (not done). Yaw block goes in 4.8 mm forward; face front 2.5 mm.
- **Verified after the full rebuild**: 27 printed parts one piece; horns 15/15; fits 333; all assembly paths (leg built off the
  robot, hung on the hip-roll horn last); screws dispositioned; head room 77/77; cables 29/29; motion = known limits only.
- **Files**: `exports/xgoduck_scs0009/print/` 30 STL + manifest (git-ignored); ASSEMBLY.md (build order, horn trimming), BOM.md.
- **Sim** (re-exported from these parts, trainnr bundle `xgoduck_scs0009_v2i` = v2h + the new SIT; recordings in the trainnr Studio): stand, crouch
  0.3, shift, neck / head moves work. **Sit/stand solved**: new SIT hip 70 / knee 18 / ankle -16 (leans back onto the
  enclosure's rear edge), straight path from STAND over 3 s - balanced all the way, 108/108 robustness variations, peak load
  <= 69 %, exact parts clear >= 2.2 mm (REVIEW.md "MuJoCo on the rebuilt parts; sit and stand"). The old SIT face-plants.
- **Next**: measure the real parts (horn, kit screws, battery, IMU holes, servo steps/deg and protection registers); print a
  tolerance coupon (6702 pocket, 28 mm bridge); then the gait inside the limits (no walking policy exists for this robot yet -
  trainnr's walk families are fixed rigs).

## Open items / next steps

1. **Re-run the checks**: fit audit + full motion sweep have NOT been run since the last body fixes and the split into
   motor body + enclosure (see REVIEW.md "Body split"). Known before the fixes: L-bracket x centre strip at roll-out
   (strip since narrowed), neck -135 deg cradle x motor body 13 mm3 (not located).
2. **Cables**: blocked exits - head-pitch lead (the moving U passes 2.5 mm in front of the slot), hip-yaw leads (neck
   horn plate / cradle), hip-roll leads (deck). Cables are curves through waypoints, not collision-checked. Hub mounts on
   the thighs, cradle and U are not designed. How battery / adapter / regulator are held down is not designed.
3. **Measure on real parts**: X horn (plate, hub, arm widths, holes), plug and hub board, the adapter's socket count and
   UART pinout, the Uno Q's height and plug positions, a real 3S pack and UBEC; try an M2 machine screw in the shaft.
4. **Head**: head core holding the roll servo (cap), mouth servo, 0.85x shell, camera/speaker if wanted.
5. **Positive retention** of the ten one-sided joints (one M2 screw into the shaft each, as the original): user's call
   (bayonet hooks proposed). Hips: both yawed fully out still touch (L-brackets 14.6 mm3 at 0.45 rad + roll).
6. Recompute masses/inertias, regenerate the sim model (`scene_scs0009_v1` is the OLD v1 geometry), firmware for the SCS
   protocol (1024 steps / 300 deg), walking policy.
7. Test-print one knee joint; styling; STL export.

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
