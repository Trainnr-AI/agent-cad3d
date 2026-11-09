# agent-cad3d

Agentic 3D design: describe what you want, and an AI agent builds it in a live Blender session
you can watch and take over at any time. Focused first on robotics: robot links, grippers,
fixtures and simulation scenes, exported to STL/OBJ/glTF and MJCF/URDF.

Built on [MCP for Blender](https://github.com/ahujasid/mcp-for-blender). Other CAD backends
(FreeCAD, SolidWorks) are planned.

**Continuing work? Read [docs/HANDOFF.md](docs/HANDOFF.md).**

## Setup (macOS / Linux / WSL2)

```bash
./scripts/setup.sh
```

This installs `uv` and Blender if missing, installs the MCP for Blender addon and enables it.
Then:

1. Open Blender. The addon's server auto-starts; this repo uses `localhost:9877` (9876 is the trainnr
   Studio viewer's port). Set the port in the addon panel and save it in your .blend. Check it in the 3D viewport:
   press `N` → **MCP for Blender** tab (click **Start MCP Server** if needed).
2. Run `claude` in this repo and approve the `blender` MCP server from `.mcp.json`.
3. Ask for something: *"Make a 2-link planar arm, 0.3 m and 0.25 m links, revolute joints about Z."*

Telemetry is disabled in `.mcp.json`. The addon only listens on localhost.

## How it works

```
you ──▶ Claude Code ──MCP (stdio)──▶ mcp-for-blender ──TCP :9877──▶ Blender addon ──▶ your scene
                ◀────────── screenshots, scene info, results ◀──────────
```

The agent writes Blender Python for modelling, reads numbers back from the scene, and takes
viewport screenshots to check its own work. Conventions it follows are in [CLAUDE.md](CLAUDE.md).

## Security note

The MCP server can run any Python inside Blender. Save your work, and only connect it to Blender
instances you trust. Set `BLENDER_MCP_SAFE_MODE=1` in `.mcp.json`'s `env` to block file, network
and subprocess access from agent scripts (normal modelling and import/export still work).
