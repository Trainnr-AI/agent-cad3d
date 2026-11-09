"""Enable the MCP for Blender addon and persist it in user preferences.

Run headless:  blender --background --python scripts/enable_addon.py
"""
import addon_utils
import bpy

MODULE = "blender_mcp"

addon_utils.modules_refresh()
mod = addon_utils.enable(MODULE, default_set=True, persistent=True)
if mod is None:
    raise SystemExit(f"Could not enable addon '{MODULE}'. Is blender_mcp.py in the user addons folder?")

bpy.context.preferences.use_preferences_save = True
bpy.ops.wm.save_userpref()
print(f"[agent-cad3d] Enabled '{MODULE}' and saved preferences.")
