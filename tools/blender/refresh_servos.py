"""Rebuild every placed servo (scs_* roots) with the current parts/scs0009.py (exact meshes), keeping
the roots, their parents and poses. Horn trim per joint: type-C joints get the r = 6 trimmed cross,
type-F joints the full cross. Runs inside Blender.
ARGS = {"servo_script": "/abs/parts/scs0009.py", "parts_dir": "/abs/tools/blender/parts",
        "full_horn": ["ankle", "neck_pitch", "head_pitch", "head_roll"], "full_angle_deg": 45.0, "trim_r": 6.0, "only": [...]}
"""
import bpy

args = globals()["ARGS"]
code = open(args["servo_script"]).read()
full = args.get("full_horn", ["ankle", "neck_pitch", "head_pitch", "head_roll"])
roots = [o for o in bpy.data.objects if o.type == "EMPTY" and o.name.startswith("scs_") and "servo_joint" in o]
if args.get("only"):
    roots = [o for o in roots if o.name in args["only"]]
done = []
for root in roots:
    j = root["servo_joint"]
    trim = None if any(j.endswith(f) for f in full) else args.get("trim_r", 6.0)
    # the joint library places the real horn screw (<joint>_screw_M2x4kit, on the printed plate 1.1 mm above the horn);
    # the servo's own stand-in would duplicate it 1.1 mm lower (review 2026-11-14) -> only where no joint screw exists (mouth)
    joint_screw = bpy.data.objects.get(f"{'neck' if j == 'neck_pitch' else j}_screw_M2x4kit")
    ns = {"ARGS": {"name": root.name, "collection": root.users_collection[0].name, "location": tuple(root.location),
                   "horn": "cross", "horn_trim_r": trim, "horn_angle_deg": 0.0 if trim else args.get("full_angle_deg", 45.0), "exact": True, "screws": True,
                   "horn_screw": joint_screw is None, "parts_dir": args.get("parts_dir")}}
    exec(code, ns)
    root["horn_trim_r"] = trim if trim else 0.0
    done.append(f"{root.name}({'r6' if trim else 'full'})")
bpy.context.view_layer.update()
print(f"refreshed {len(done)} servos: " + ", ".join(done))
