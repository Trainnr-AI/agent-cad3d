# Design check suite (nanoduck)

`scripts/full_check.sh [out_dir]` runs every check below headless, in parallel, on the SAVED .blend (the live Blender is not
touched; save first). ~15-20 min. Each JSON here is a tool's ARGS; `<repo>` and `<out>` are filled in by
`tools/blender/run_headless.py`. Any one check: `blender -b projects/xgoduck_scs0009/xgoduck_scs0009.blend --python
tools/blender/run_headless.py -- <tool.py> <args.json> <out_dir>`.

| Check | Tool | Args | Expected on the closed design (2026-11-15) |
|---|---|---|---|
| Robot at rest | `checks/rest_pose_check.py` | - | joints differing from rest q0: none |
| Printed parts one piece | `checks/printed_parts_check.py` | - | 27 checked, no problems |
| Printability | `print_check.py` | `print.json` | feet 436 / 438 mm2 unsupported, head base 1719, dome 1937, jaw 186, face 466 |
| Horns in their pockets | `horn_check.py` | `horn.json` | 15/15: full 0.30 at the tips / 0.19 sides, trimmed 0.20 |
| Fits | `fit_audit.py` | `fit.json` | 367 fits, 2 "not as designed", both known: battery vs enclosure 0.30 mm3 (sampling; battery deferred), left hip-pitch servo vs thigh 0.11 mm3 (sampling: exact boolean 0.002 mm3, touching) |
| Assembly paths | `assembly_path_check.py` | `asm.json` | 55 OK + left hip-yaw servo "BLOCKED" straight up (it goes in by an L-path: next two rows) |
| L-path of the left hip-yaw servo | `checks/hip_yaw_servo_path.py` | - | lift 8-8.5 mm then slide from the rear or a side: clear for 40 mm |
| Segment paths (jaw, beak servo, yaw block) | `checks/segment_path_check.py` | `segment_paths.json` | all OK |
| Screws | `screw_audit.py` | `screws.json` | 89 screws, 26 flagged - all dispositioned in REVIEW.md ("Screw audit ... dispositions"): kit horn screws bite 1.5-1.9 mm in the spline, M3 bearing axles into nylon nuts, ear screws "bite 2 parts" (model artifact), Uno Q M3 posts "wall 0.15" (sampling) |
| Screwdriver access | `checks/driver_access.py` | `driver.json` | nearest obstruction per screw; shortest 17 mm (jaw pivot) |
| Motion sweep | `leg_motion_check.py` | `motion.json` | the known walking-envelope corners and both-hips poses outside the servo limits (JOINT_LIMITS.md) |
| Cable reach | `wiring.py` (motion) | `cable_reach.json` | 0 cables too short (least slack: left body-to-thigh 224 of 250) |
| Cable pinch | `cable_pinch_check.py` | `cable_pinch.json` | 21 spans pushed by a part (straight-span model): handled by sleeve / rounded edge / slack loop, ASSEMBLY.md step 4 |

Not in the script (long sweeps, run when joints or parts change): `clearance_sweep.py` + `hip_map_table.py` (hip maps, both legs,
head yaw x roll, knee x ankle - the pose lists are in data/hip_collision_map.json's grid). An independent all-pairs interference
check is in the sister repo microduck-nano (`scripts/design-review/2026-11-14/static.py`): on the closed design no printed part
cuts another by > 0.02 mm and no servo cuts a print by > 0.06 mm.
