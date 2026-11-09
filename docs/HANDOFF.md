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
BOM v2: 16x22x4 x5, **693ZZ x4, M3x8 + M3 nut x4** (replace the 10x15x3), kit M2 self-tappers for
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

## Open items / next steps

1. **Measure the X horn** (plate thickness, hub Ø and height below/above the plate, screw recess,
   arm width at the hub and at r = 6 mm) and the kit ear-screw length. Then regenerate legs + neck
   against the exact servo (boss, roof, cable slot at the shaft end, horn 11.94) and re-run all checks;
   then **test-print one knee joint** to tune crush ribs and bearing fits.
2. **Head:** head-yaw bearing cap in the yaw block, head-roll mount (pin + bearing plate), mouth
   servo, 0.85x head shell around the camera board (head-roll servo has only 1.4 mm to the outline
   on one side). The body/trunk shell still carries the original's neck arms — cut them.
3. Recompute masses/inertias from the new parts (+ battery/boards) and update the sim model.
4. **Walking:** train a walk policy for the new duck with `xgoduck_rl` (mjlab, CUDA GPU), identify the
   real SCS0009 dynamics from telemetry first (trainnr `identify_system`). Only a trained policy can
   be shown in trainnr's embedded MuJoCo view (as `walk` or `deploy:<name>`).
5. Styling: reshape the flat plates toward the original's rounded parts; STL export for printing.

Parts list deltas vs the original BOM: 16x22x4 bearings 8 (legs) + yaw/neck as designed; 10x15x3
bearings 2 (ankles) + 2 (neck); M2x6 machine screws 1 per horn; M2x6/M2x8 countersunk for bearing plates.

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
