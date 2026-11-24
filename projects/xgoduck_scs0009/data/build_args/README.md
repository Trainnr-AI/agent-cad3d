# Build order for the body (run each with scripts/blender_doctor.py <script> --args "$(cat <file>)", <repo> = repo root)

1. `tools/blender/parts/body_v2.py`  with `body_v2.json` (clears the body_v2 and electronics collections)
2. `tools/blender/parts/uno_q.py`    (exact Uno Q; args: robot, collection "electronics", usb_end "rear", joint_lib/base_lib/dims)
2b. `tools/blender/parts/bay_parts.py` (args: robot, joint_lib/base_lib/dims): the bay's connectors and small hardware at real size -
   XT30 pair + balance plug (rear-right cradle), switch nut / lever / lugs, Mini560 solder joints, the Uno Q's right-angle plugs,
   Qwiic plugs, the right-angle USB-C -> USB-A adapter + camera plug (rear-left cradle). Each carries `anchors` for wiring.py.
3. `tools/blender/wiring.py`         with `wiring.json` (+ `"motion": true` for the cable reach check)
4. `tools/blender/parts/neck_sweep.py` with `cover_recut.json`: recut the Uno Q cover for hub Y's plugs AS WIRED
   (step 3 can move them to other sockets; the cut made in step 1 used the old ones)

Head (after `head_shell_v2.py` builds the dome):

4d. 2026-11-25 (review): `tools/blender/parts/horn_screw_standins.py` - the 15 horn-screw stand-ins as the M2 x 6 button head, seated on
   each part's counterbore floor (names kept); `tools/blender/parts/hood_box_fixes.py` - five cuts in the Mini560 box on the hood
   (battery-lead slot, window skin, pigtail notch, two PCB guide grooves), also in body_v2.py for a rebuild. Both idempotent.
4c. 2026-11-25: `tools/blender/parts/horn_ring_recess.py` (no args) cut the horn top-face ring recess (Ø5.2 x 0.35) IN PLACE into the
   12 parts holding the 15 horn seats; parts rebuilt from now on get it from joint_v2 (`top_ring_recess`). Idempotent; safe to re-run.

4b. 2026-11-20: DO NOT rebuild `head_shell_v2.py` parts ["base", "dome"] - from the original source meshes it now gives broken parts
   (masses in the billions, stray geometry, dropped posts) and overwrites jaw_soft["front_screws"] (must stay 73.06, -130.9 / 73.06, -69.12).
   The base and dome are edited in place: `tools/blender/parts/head_rear_screws.py` (once; flags on head_dome: rear_screw_x, base_giveway)
   moves the two rear dome screws to x 1.5 (new dome posts + pilots, slot in the old right post, base bosses + webs, holes through the
   base's rim, post bottoms trimmed to z 199.25). The FACE rebuilds fine: `head_shell_v2.py` parts ["face"], camera "ov5640_b" (visor,
   nut block, rear boss, 3x M2x10 countersunk, 2 M2 nuts).

5. `tools/blender/parts/dome_ear_fill.py` with `ear_fill.json`: fills the dome's two side "ear" arches (left fully; right
   except where the head-pitch U swings in head roll, + 1.5 mm). Then rerun the `cam` part's head-room cut if the dome
   was rebuilt (head_shell_v2 does it when the camera screws exist).

Exploded views (after any rebuild: their '.x' / '.hx' copies keep the old meshes until regenerated):

6. `tools/blender/explode_all.py` with `{"collection":"duck_v2_exploded","offset":[0,-0.45,0],"scale":1.5,"gap":30}` - the whole
   duck: legs and neck opened along their joint axes, the head taken apart, the bay behind the body in build order (189 elements on 2026-11-25; the bay laid out as bay_explode.json, incl. the bay_* connectors - explode_all.py keeps its OWN copy
   of that layout: keep them in sync; 2 "(missing)" names - hip_yaw_support, body_neck_mount - are old names, expected).
7. `tools/blender/wiring.py` with `wiring_exploded.json` - the cables drawn on that exploded copy (collection wiring_exploded).
8. `tools/blender/explode_list.py` with `bay_explode.json` + `wiring.py` with `wiring_bay_exploded.json` - the bay sheet.
8b. `tools/blender/sheet_labels.py` with `bay_labels.json`: numbered labels + leader lines on the bay sheet; render with closeup.py
   (collection bay_exploded, also wiring_bay_exploded + bay_labels, isolate, hide_prefix uno_q.x, elev 82, azim 0) -> data/bay_assembly_sheet.png.
9. `tools/blender/head_explode.py` with `{"offset":[0,-0.3,0.1]}` - the head sheet.
10. Hide the Uno Q's envelope box copies (`uno_q.x*`): the board's parts (`unoq_*`) are shown instead. Rerunning step 3
    (assembled wiring) and step 4 (cover recut) is not needed: the copies share the parts' meshes (checked 2026-11-19: cover,
    dome, face, enclosure and all cables unchanged). Renders: `tools/blender/closeup.py` -> data/duck_exploded.png (elev 72, azim
    135), duck_exploded_side.png, bay_exploded.png, head_exploded.png.

Whole-robot assembly animation (2026-11-20): `tools/blender/robot_assembly_seq.py` (in Blender: reads asm.json, segment_paths.json, the
screw audit's screws.json with axes, bay_assembly_anim.json; writes robot_assembly_anim.json) -> `tools/blender/assembly_anim.py` with that
JSON (scene robot_assembly, 42 steps, 199 s on 2026-11-25 - regenerate it after any bay_assembly_anim.json change). Its moves are path-checked with `tools/blender/anim_to_paths.py` (+ bay_assembly_allow.json,
`fast`, step 1 mm) -> `assembly_path_check.py`: 458 segments, all OK on 2026-11-21 (~25 min in 4 parallel Blenders).
