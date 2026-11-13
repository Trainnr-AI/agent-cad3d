# Build order for the body (run each with scripts/blender_doctor.py <script> --args "$(cat <file>)", <repo> = repo root)

1. `tools/blender/parts/body_v2.py`  with `body_v2.json` (clears the body_v2 and electronics collections)
2. `tools/blender/parts/uno_q.py`    (exact Uno Q; args: robot, collection "electronics", usb_end "rear", joint_lib/base_lib/dims)
3. `tools/blender/wiring.py`         with `wiring.json` (+ `"motion": true` for the cable reach check)
4. `tools/blender/parts/neck_sweep.py` with `cover_recut.json`: recut the Uno Q cover for hub Y's plugs AS WIRED
   (step 3 can move them to other sockets; the cut made in step 1 used the old ones)

Head (after `head_shell_v2.py` builds the dome):

5. `tools/blender/parts/dome_ear_fill.py` with `ear_fill.json`: fills the dome's two side "ear" arches (left fully; right
   except where the head-pitch U swings in head roll, + 1.5 mm). Then rerun the `cam` part's head-room cut if the dome
   was rebuilt (head_shell_v2 does it when the camera screws exist).

Exploded views (after any rebuild: their '.x' / '.hx' copies keep the old meshes until regenerated):

6. `tools/blender/explode_all.py` with `{"collection":"duck_v2_exploded","offset":[0,-0.45,0],"scale":1.5,"gap":30}` - the whole
   duck: legs and neck opened along their joint axes, the head taken apart, the bay behind the body in build order (166 elements).
7. `tools/blender/wiring.py` with `wiring_exploded.json` - the cables drawn on that exploded copy (collection wiring_exploded).
8. `tools/blender/explode_list.py` with `bay_explode.json` + `wiring.py` with `wiring_bay_exploded.json` - the bay sheet.
9. `tools/blender/head_explode.py` with `{"offset":[0,-0.3,0.1]}` - the head sheet.
10. Hide the Uno Q's envelope box copies (`uno_q.x*`): the board's parts (`unoq_*`) are shown instead. Then rerun step 3
    (assembled wiring) and step 4 (cover recut). Renders: `tools/blender/closeup.py` -> data/duck_exploded.png (elev 72, azim
    135), duck_exploded_side.png, bay_exploded.png, head_exploded.png.
