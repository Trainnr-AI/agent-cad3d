# Design check suite (nanoduck)

`scripts/full_check.sh [out_dir]` runs every check below headless, in parallel, on the SAVED .blend (the live Blender is not
touched; save first). ~15-20 min. Each JSON here is a tool's ARGS; `<repo>` and `<out>` are filled in by
`tools/blender/run_headless.py`. Any one check: `blender -b projects/xgoduck_scs0009/xgoduck_scs0009.blend --python
tools/blender/run_headless.py -- <tool.py> <args.json> <out_dir>`.

| Check | Tool | Args | Expected (2026-11-25, all-Dupont bay after the deep review; was 2026-11-19) |
|---|---|---|---|
| Robot at rest | `checks/rest_pose_check.py` | - | joints differing from rest q0: none |
| Printed parts one piece | `checks/printed_parts_check.py` | - | 30 checked, no problems (2026-11-25: + mini560_back_plate) |
| Printability | `print_check.py` | `print.json` | feet 436 / 438 mm2 unsupported, head base 1719, dome 1937, jaw 186, face 466 |
| Horns in their pockets | `horn_check.py` | `horn.json` | 15/15: full 0.30 at the tips / 0.19 sides, trimmed 0.20 |
| Fits | `fit_audit.py` | `fit.json` | 489 fits (2026-11-25: 13 pairs with removed parts dropped, 103 for the Mini560 / splices / switch housings added), 1 "not as designed", known: left hip-pitch servo vs thigh 0.11 mm3 (sampling: exact boolean 0.002 mm3, touching). (2026-11-19: the battery stand-in flag is gone - the 20 mm pack has room; the Uno Q's underside envelope is the datasheet's 2 mm) | 2026-11-20: +32 pairs (bay plate / fin / cover / board, camera visor / nuts, dome on the base) that had been missing.
| Assembly paths | `assembly_path_check.py` | `asm.json` | 55 OK + left hip-yaw servo "BLOCKED" straight up (it goes in by an L-path: next two rows) |
| L-path of the left hip-yaw servo | `checks/hip_yaw_servo_path.py` | - | the whole servo (case + spline + case screws): lift 9.5-10.5 mm, then slide from either side: clear for 40 mm (below 9.5 the spline hits the body, above 10.5 the case) |
| Bay assembly moves | `assembly_path_check.py` (from `build_args/bay_assembly_anim.json` via `anim_to_paths.py`) | - | 44 OK |
| Segment paths (jaw, beak servo, yaw block) | `checks/segment_path_check.py` | `segment_paths.json` | all OK |
| Screws | `screw_audit.py` | `screws.json` | 87 screws, 23 flagged - all dispositioned in REVIEW.md ("Screw audit ... dispositions"; 2026-11-25: horn screws M2 x 6, 3.1-3.3 mm in the spline, "wall 0.13" = the spline teeth's sampling, 3 "case" bites = 1 sample in 720), M3 bearing axles into nylon nuts, ear screws "bite 2 parts" (model artifact), Uno Q M3 posts "wall 0.15" (sampling); + the camera visor's left pair (bite 1.5 = the whole M2 nut, metal: full engagement) |
| Screwdriver access | `checks/driver_access.py` | `driver.json` | nearest obstruction per screw; shortest 17 mm (jaw pivot) |
| Motion sweep | `leg_motion_check.py` | `motion.json` | 1 + 14 x 9 + 69 poses (2026-11-25: + 22 bay parts, the current SIT, the neck interlock's 4 corners), 42 with collisions, none inside the limits + interlock (JOINT_LIMITS.md; neck 20-40 with the head low / high: the hub on the U vs the D0/D1 housing). Before: the walking-envelope corners, both hips at roll +-22, the model's wider stored ranges, and neck +40 (beyond the +30 limit: the hub plug on the U meets the 3 mm higher Uno Q's headers) |
| Cable reach | `wiring.py` (motion) | `cable_reach.json` | 1 "SHORT": `power_battery_leads` 74-78 of the 60 mm assumed - a BUYING requirement (the battery's own leads must reach ~75 mm to its XT30 in the rear cradle, BOM row 4), not a motion problem; all other cables ok |
| Cable pinch | `cable_pinch_check.py` | `cable_pinch.json` | 2026-11-25: 25 spans inside a part at rest, 18 pushed > 0.5 mm in motion (was 38 / 19). Earlier: 21 spans pushed by a part (straight-span model). The bay pigtails / camera cable into the right foot occur only toe-in with the leg back (hip yaw -6.3, pitch +45.8: forbidden by the hip map); camera cable at the U (1.8 mm, neck +30 / head pitch +65 / yaw 150) and head base (0.95, roll -12): the neck slack loop (ASSEMBLY.md step 4) |

Not in the script (long sweeps, run when joints or parts change): `clearance_sweep.py` + `hip_map_table.py` (hip maps, both legs,
head yaw x roll, knee x ankle - the pose lists are in data/hip_collision_map.json's grid). An independent all-pairs interference
check is in the sister repo microduck-nano (`scripts/design-review/2026-11-14/static.py`): on the closed design no printed part
cuts another by > 0.02 mm and no servo cuts a print by > 0.06 mm.


Walking corners vs the bay (2026-11-19, `clearance_sweep.py`, 216 poses: both legs, hip yaw -6.3 / 0 / +12.6, roll 0 / +-10.9,
pitch +-30 / +-45.8, knee as the gait, ankle 0 / +-23): every hit on the bay or its rear cradles is toe-in with the leg back (yaw -6.3) or
roll 10.9 at pitch 45.8 - both forbidden by the hip map (max roll +10 at +45). Behind the bay, walking feet reach z 129.8; only
y -108.5..-91.5 is never swept.
