#!/usr/bin/env python3
"""Set each keyframe's root height so the lowest collision vertex sits CLEARANCE above the floor.
  python3 tools/sim/fix_key_heights.py scene.xml [--clearance-mm 1.0]   (edits the scene file in place)"""
import argparse
import re

import mujoco
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("scene")
ap.add_argument("--clearance-mm", type=float, default=1.0)
a = ap.parse_args()
m = mujoco.MjModel.from_xml_path(a.scene)
d = mujoco.MjData(m)
floor = m.geom("floor").id
new_z = {}
for k in range(m.nkey):
    d.qpos[:] = m.key_qpos[k]
    mujoco.mj_kinematics(m, d)
    low = np.inf
    for g in range(m.ngeom):
        if g == floor or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 3:
            continue
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        low = min(low, w[:, 2].min())
    new_z[m.key(k).name] = m.key_qpos[k][2] - low + a.clearance_mm / 1000
    print(f"{m.key(k).name:6} lowest point {low * 1000:+.2f} mm -> root z {new_z[m.key(k).name] * 1000:.2f} mm")
txt = open(a.scene).read()
for name, z in new_z.items():
    pat = re.compile(r'(<key name="%s" qpos=")\s*([^\s"]+)\s+([^\s"]+)\s+([^\s"]+)' % name, re.S)
    txt = pat.sub(lambda mt: f'{mt.group(1)}{mt.group(2)} {mt.group(3)} {z:.6f}', txt, count=1)
open(a.scene, "w").write(txt)
