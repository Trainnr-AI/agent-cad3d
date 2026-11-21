# agent-cad3d — operating guide for the 3D agent

This repo is an agentic 3D design workspace. The user watches and controls Blender; the agent
drives the same live Blender session through the `blender` MCP server (MCP for Blender).
Primary domain right now: **robotics** (robot links, grippers, fixtures, task scenes).
Other backends (FreeCAD, SolidWorks) may be added later — keep work backend-agnostic where cheap.

## Ground rules

- **The user's Blender session is shared.** Never wipe the scene, delete objects you didn't create,
  or open/replace the .blend unless asked. Save (`bpy.ops.wm.save_mainfile`) before risky edits.
- **Small steps, then look.** After every meaningful change, verify with `look`
  (use `angles` for geometry, `wireframe`/`xray` for internals) and `get_scene_info` for numbers.
  Don't claim a dimension is right without reading it back from the scene.
- **Numbers over eyeballing.** Read back `dimensions`, `location`, `rotation_euler`, vertex bounds
  after creating geometry and report them when accuracy matters.
- **Idempotent scripts.** Code sent through `execute_blender_code` should reuse-or-create by name
  (so re-running doesn't duplicate objects), and should not depend on the current selection or mode.

## Units and conventions (robotics)

- Scene units: metric, `scale_length = 1.0`, lengths in **metres**, angles in **radians** in code.
- **Z-up, X-forward, right-handed** — matches Blender, MuJoCo and ROS (REP-103).
- One Blender object = one rigid link. Name links `<robot>_<link>` (e.g. `arm_link2`).
- Put each link's **origin at its parent joint frame**, with the joint axis along a local axis
  (prefer Z for revolute). Apply scale (`Ctrl+A` / `transform_apply(scale=True)`) before export.
- Keep a separate low-poly `<name>_collision` mesh (convex or primitive) per link; visual meshes can be detailed.
- Record joint metadata as custom properties on the child link: `joint_type`, `joint_axis`,
  `joint_lower`, `joint_upper` (rad or m), `parent`. Mass/inertia as `mass_kg` when known.

## Exports

- Write exports under `exports/<project>/` — meshes as STL/OBJ (per link, local frame), whole
  scenes as glTF (`.glb`). Never write outside the repo unless the user asks.
- Robot descriptions (MJCF/URDF) go in `exports/<project>/` next to their meshes and reference them
  relatively. The trainnr tools (`onboard_robot`, etc.) can take an MJCF from there into simulation.

## Layout

- **Start here: `docs/HANDOFF.md`** — project state, decisions, open items, how to resume on WSL.
- `scripts/setup.sh` — installs uv, Blender (macOS: brew; Linux/WSL: official tarball), the add-on,
  and fetches `vendor/` at pinned commits (`scripts/fetch_vendor.sh`). Idempotent.
- `scripts/blender_doctor.py` — talk to the add-on directly (port 9877; `BLENDER_PORT`, `BLENDER_HOST`,
  `BLENDER_TIMEOUT` env). Run a script in Blender with `--args '<json>'` (script sees `ARGS`).
- `.mcp.json` — pins `mcp-for-blender`, port 9877 (9876 is left for the trainnr Studio viewer).
- `tools/blender/` — in-Blender tools: import (`import_mjcf`), parts (`parts/`), checks
  (`compare_kinematics`, `sweep_compare`, `leg_motion_check`, `assembly_path_check`,
  `overlap_sample`, `overlap_volume`, `fit_audit`, `swept_space`, `print_check`), views (`closeup`, `color_code`,
  `section_view`, `explode_all`, `explode_list`, `wiring`, `organize_scene`), export (`export_bodies`).
- `tools/cad/` — STEP → STL + feature report with cadquery (`uv run --with cadquery`), e.g. the exact
  SCS0009 meshes in `tools/blender/parts/scs0009_mesh/`.
- `tools/sim/` — MuJoCo: `build_scaled_mjcf`, `apply_body_meshes`, `fix_key_heights`, `servo_tests`,
  `rerun_replay` (into trainnr Studio), `mujoco_showcase` (native viewer).
- `tools/slicer/` — OrcaSlicer CLI (`scripts/setup_orca.sh`): `slice_project.sh <slicing.json>` slices every printed part in
  6 orientations on a copy of the .blend, maps supports back onto the model (bearing seats, servo / horn pockets), picks each
  part's orientation, gives hours / grams / cm3 and a sliced .3mf. Skill: `.claude/skills/slice-print/`.
- `projects/<project>/` — .blend, `REVIEW.md`, `data/`, `sim/`. `exports/`, `vendor/` — git-ignored.

## Checks before calling a part done

One solid piece (`shells == 1`) and watertight; fits measured (overlap volumes); the joint can be
assembled (`assembly_path_check.py`); full-range motion sweep incl. combined poses, compared with
the original. Report what was verified and what wasn't.
