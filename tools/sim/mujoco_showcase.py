#!/usr/bin/env python3
"""Play the servo tests live in MuJoCo's native viewer, several robots side by side in ONE scene.

The first scene provides the floor/lights; every robot (scene's robot) is attached with its own
name prefix, offset along -Y. Each robot runs the same scripted commands (servo_tests.script),
looping through the test list. Status prints to the terminal.

macOS needs mjpython for the passive viewer:
  uv run --with mujoco mjpython tools/sim/mujoco_showcase.py \
      projects/xgoduck_scs0009/sim/scene_scs0009_v1.xml projects/xgoduck_scs0009/sim/scene_original.xml
"""
import argparse
import importlib.util
import math
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec_ = importlib.util.spec_from_file_location("servo_tests", os.path.join(HERE, "servo_tests.py"))
st = importlib.util.module_from_spec(spec_)
spec_.loader.exec_module(st)

ap = argparse.ArgumentParser()
ap.add_argument("scenes", nargs="+", help="scene XMLs; the first one's floor/lights are used")
ap.add_argument("--tests", default="stand_hold,squat_0.3,shift_0.12,set:head_yaw:2.618,set:head_roll:0.436,"
                                   "pose:neck_pitch=-1.2;head_pitch=1.2,set:neck_pitch:-2.356")
ap.add_argument("--gap", type=float, default=0.22)
ap.add_argument("--loop", action="store_true", default=True)
ap.add_argument("--check", action="store_true", help="build the scene, run 2 s headless, exit")
a = ap.parse_args()

names = [os.path.basename(s).replace("scene_", "").replace(".xml", "") for s in a.scenes]
singles = [mujoco.MjModel.from_xml_path(s) for s in a.scenes]

# ---- combined scene: base = first scene's world without its robot; every robot attached with a prefix
base = mujoco.MjSpec.from_file(a.scenes[0])
for b in list(base.worldbody.bodies):
    base.delete(b)                        # remove the first scene's own robot (re-attached below)
for k in list(base.keys):
    base.delete(k)
for s in list(base.sensors):
    base.delete(s)
for i, (scene, name) in enumerate(zip(a.scenes, names)):
    child = mujoco.MjSpec.from_file(scene)
    for g in list(child.worldbody.geoms):
        child.delete(g)                   # its floor
    for l in list(child.worldbody.lights):
        child.delete(l)
    for k in list(child.keys):
        child.delete(k)
    frame = base.worldbody.add_frame(pos=[0, -i * a.gap, 0])
    frame.attach_body(child.worldbody.first_body(), f"{name}/", "")
m = base.compile()
d = mujoco.MjData(m)


def place(i):
    """set robot i's qpos from its own STAND keyframe, offset along -Y"""
    sm = singles[i]
    key = sm.key("STAND")
    for j in range(sm.njnt):
        jn = f"{names[i]}/{sm.joint(j).name}"
        cj = m.joint(jn)
        qa, sa = m.jnt_qposadr[cj.id], sm.jnt_qposadr[j]
        n = 7 if sm.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE else 1
        d.qpos[qa:qa + n] = key.qpos[sa:sa + n]
        if n == 7:
            d.qpos[qa + 1] -= i * a.gap
    mujoco.mj_forward(m, d)


act = [[m.actuator(f"{names[i]}/{singles[i].actuator(k).name}").id for k in range(singles[i].nu)] for i in range(len(names))]
tests = a.tests.split(",")
print("robots (left to right along -Y):", ", ".join(names))

if a.check:
    for i in range(len(names)):
        place(i)
        d.ctrl[act[i]] = singles[i].key("STAND").ctrl
    for _ in range(1000):
        mujoco.mj_step(m, d)
    for n in names:
        bid = m.body(f"{n}/trunk_base").id
        print(f"check: {n} trunk z {d.xpos[bid][2] * 1000:.1f} mm, upright {d.xmat[bid][8]:.3f}")
    print(f"check ok: {m.nbody} bodies, {m.nu} actuators")
    raise SystemExit

with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.azimuth, v.cam.elevation, v.cam.distance = 150, -18, 0.75
    v.cam.lookat[:] = [0.0, -(len(names) - 1) * a.gap / 2, 0.11]
    ti = 0
    while v.is_running():
        test = tests[ti % len(tests)]
        mujoco.mj_resetData(m, d)
        for i in range(len(names)):
            place(i)
        scripts = [st.script(singles[i], test) for i in range(len(names))]
        dur = max(s[0] for s in scripts) + 0.8
        print(f"\n▶ {test}")
        t, fell = 0.0, set()
        n_sub = max(1, int(round(st.DT_CTRL / m.opt.timestep)))
        while t < dur and v.is_running():
            t0 = time.time()
            for i, (_, f) in enumerate(scripts):
                d.ctrl[act[i]] = f(min(t, scripts[i][0]))
            for _ in range(n_sub):
                mujoco.mj_step(m, d)
            t += st.DT_CTRL
            for i, n in enumerate(names):
                bid = m.body(f"{n}/trunk_base").id
                if n not in fell and d.xmat[bid][8] < math.cos(math.radians(50)):
                    fell.add(n)
                    print(f"   {n} FELL at {t:.1f} s")
            v.sync()
            time.sleep(max(0.0, st.DT_CTRL - (time.time() - t0)))
        for n in names:
            if n not in fell:
                print(f"   {n} stayed upright")
        ti += 1
        if not a.loop and ti >= len(tests):
            break
