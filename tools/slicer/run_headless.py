"""Run an in-Blender tool script headless, with ARGS like blender_doctor.py gives it (for when the live Blender is closed, or to
work on a COPY of the .blend so the user's session is never touched).

Blender -b <copy.blend> --python tools/slicer/run_headless.py -- <script.py> '<json ARGS>'
"""
import json
import sys

i = sys.argv.index("--")
script, args = sys.argv[i + 1], json.loads(sys.argv[i + 2])
g = {"ARGS": args, "__name__": "__main__"}
exec(compile(open(script).read(), script, "exec"), g)
