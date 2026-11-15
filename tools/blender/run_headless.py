"""Run a Blender tool headless with its ARGS from a JSON file:

  blender -b projects/xgoduck_scs0009/xgoduck_scs0009.blend --python tools/blender/run_headless.py -- <tool.py> <args.json> [<out_dir>]

In the JSON, "<repo>" is replaced by the repository root and "<out>" by out_dir (default: the system temp dir), so the stored
arguments in projects/<project>/data/check_args/ work on any machine. The tool sees ARGS exactly as when sent to the live Blender
with scripts/blender_doctor.py. (Paths come ONLY from after "--": never from bare sys.argv - inside a live Blender that is the
Blender executable itself.)
"""
import json
import os
import sys
import tempfile

argv = sys.argv[sys.argv.index("--") + 1:]
repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
out = os.path.abspath(argv[2]) if len(argv) > 2 else tempfile.gettempdir()
os.makedirs(out, exist_ok=True)
text = open(argv[1]).read().replace("<repo>", repo).replace("<out>", out)
G = {"ARGS": json.loads(text), "__name__": "__main__"}
tool = argv[0] if os.path.isabs(argv[0]) else os.path.join(repo, argv[0])
exec(compile(open(tool).read(), tool, "exec"), G)
