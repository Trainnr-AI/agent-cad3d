#!/usr/bin/env python3
"""Write a keyframe's qpos as the robot actually rests in it: start at FROM (default STAND), drive the servos to the key's
ctrl over --move s, hold --hold s, then store the settled free-joint pose (x, y reset to 0) and joint angles in the key.
Use for poses that are not upright (e.g. a sit leaning back onto the trunk), where fix_key_heights.py (height only) is not enough.
  python3 tools/sim/settle_key.py scene.xml SIT [--from STAND] [--move 3] [--hold 2]      (edits the scene file in place)"""
import argparse
import math
import re

import mujoco
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("scene")
ap.add_argument("key")
ap.add_argument("--from", dest="start", default="STAND")
ap.add_argument("--move", type=float, default=3.0)
ap.add_argument("--hold", type=float, default=2.0)
a = ap.parse_args()
m = mujoco.MjModel.from_xml_path(a.scene)
d = mujoco.MjData(m)
k0, k1 = m.key(a.start).id, m.key(a.key).id
c0, c1 = m.key_ctrl[k0].copy(), m.key_ctrl[k1].copy()
mujoco.mj_resetDataKeyframe(m, d, k0)
n = int(a.move / m.opt.timestep)
for i in range(n + int(a.hold / m.opt.timestep)):
    s = min(i / n, 1.0)
    d.ctrl[:] = c0 + (c1 - c0) * (1 - math.cos(math.pi * s)) / 2
    mujoco.mj_step(m, d)
q = d.qpos.copy()
q[0:2] = 0
floor = []
for c in d.contact[:d.ncon]:
    g = [m.geom(x).name for x in (c.geom1, c.geom2)]
    if "floor" in g:
        floor.append([x for x in g if x != "floor"][0])
R = d.xmat[1].reshape(3, 3)
print(f"{a.key}: settled root z {q[2] * 1000:.1f} mm, trunk pitch {math.degrees(math.atan2(R[0, 2], R[2, 2])):+.1f} deg, "
      f"on the floor: {sorted(set(floor))}, max joint error {math.degrees(np.max(np.abs(q[7:] - c1))):.2f} deg")
txt = open(a.scene).read()
pat = re.compile(r'(<key name="%s" qpos=")([^"]*)(")' % re.escape(a.key))
txt, nsub = pat.subn(lambda mt: mt.group(1) + " ".join(f"{v:.9g}" for v in q) + mt.group(3), txt, count=1)
assert nsub == 1, f"key {a.key} not found"
open(a.scene, "w").write(txt)
