#!/usr/bin/env python3
"""Stream servo tests (servo_tests.py) into a Rerun viewer, e.g. the trainnr Studio viewer.

For each robot scene x test: simulates it and logs, on the `sim_time` timeline,
  world/<robot>/<body>/<geom>   meshes (static) moved by per-frame body transforms
  plots/<test>/<joint>/load_pct   |torque| as % of the servo's limit
  plots/<test>/<joint>/err_deg    |target - actual| in degrees
Robots are placed side by side (x offset) in one recording per test, so they can be compared.

  uv run --with rerun-sdk==0.36.2 --with mujoco python -I tools/sim/rerun_replay.py \
      --url rerun+http://127.0.0.1:9876/proxy --tests neck_to_-1.1,squat_0.3 \
      projects/xgoduck_scs0009/sim/scene_original.xml projects/xgoduck_scs0009/sim/scene_scs0009.xml
"""
import argparse
import importlib.util
import math
import os

import mujoco
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("servo_tests", os.path.join(HERE, "servo_tests.py"))
st = importlib.util.module_from_spec(spec)
spec.loader.exec_module(st)

COLORS = {"original": [200, 200, 200], "scs0009": [240, 190, 60], "scs0009_h1.0": [240, 190, 60],
          "scs0009_h0.85_e1": [90, 200, 120], "scs0009_v1": [240, 175, 40], "scs0009_v2": [80, 160, 240]}
LABELS = {"original": "original (grey)", "scs0009": "SCS0009 (yellow)", "scs0009_h1.0": "SCS0009 first version (yellow)",
          "scs0009_h0.85_e1": "SCS0009 fixed: 0.85x head + stiffer gain (green)",
          "scs0009_v1": "NEW duck v1: redesigned legs + neck (yellow)",
          "scs0009_v2": "NEW duck v2: full assembly - head, bay, electronics, soles, covers (blue)"}
CENTER_Y = [-0.11]


def mesh_of(m, mid):
    va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
    return m.mesh_vert[va:va + vn].copy(), m.mesh_face[fa:fa + fn].copy()


def log_robot_static(m, prefix, color):
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 2:
            continue                                   # visual meshes only
        b = m.geom_bodyid[g]
        v, f = mesh_of(m, m.geom_dataid[g])
        path = f"{prefix}/{m.body(b).name}/g{g}"
        q = m.geom_quat[g]
        rr.log(path, rr.Transform3D(translation=m.geom_pos[g], quaternion=rr.Quaternion(xyzw=[q[1], q[2], q[3], q[0]])), static=True)
        rr.log(path, rr.Mesh3D(vertex_positions=v, triangle_indices=f, albedo_factor=color), static=True)


def run(scene, test, name, offset, sign=1, every=2, t0=0.0):
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("STAND").id)
    mujoco.mj_forward(m, d)
    dur, f = st.script(m, test, sign)
    prefix = f"world/{name}"
    rr.log(prefix, rr.Transform3D(translation=offset), static=True)
    log_robot_static(m, prefix, COLORS.get(name, [180, 180, 255]))
    limit = np.array([m.actuator_forcerange[i][1] for i in range(m.nu)])
    jq = np.array([m.jnt_qposadr[m.actuator_trnid[i][0]] for i in range(m.nu)])
    names = [m.actuator(i).name for i in range(m.nu)]
    n_sub = max(1, int(round(st.DT_CTRL / m.opt.timestep)))
    t, k = 0.0, 0
    fell_at = None
    z0 = d.xpos[m.body("trunk_base").id][2]
    while t < dur:
        d.ctrl[:] = f(t)
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        t += st.DT_CTRL
        k += 1
        if k % every:
            continue
        rr.set_time("sim_time", duration=t0 + t)
        up = d.xmat[m.body("trunk_base").id].reshape(3, 3)[2, 2]
        if fell_at is None and (up < math.cos(math.radians(50)) or d.xpos[m.body("trunk_base").id][2] < 0.55 * z0):
            fell_at = t
            rr.log("captions", rr.TextLog(f"{test}: {name} FELL at {t:.1f} s", level="WARN"))
        for b in range(1, m.nbody):
            q = d.xquat[b]
            rr.log(f"{prefix}/{m.body(b).name}",
                   rr.Transform3D(translation=d.xpos[b], quaternion=rr.Quaternion(xyzw=[q[1], q[2], q[3], q[0]])))
        load = 100 * np.abs(d.actuator_force) / limit
        err = np.degrees(np.abs(d.ctrl - d.qpos[jq]))
        for i, n in enumerate(names):
            if n.startswith("right_"):
                continue                               # the right side mirrors the left
            rr.log(f"plots/{n}/load_pct/{name}", rr.Scalars(load[i]))
            rr.log(f"plots/{n}/err_deg/{name}", rr.Scalars(err[i]))
    return dur, fell_at


CAPTIONS = {"stand": "hold the standing pose", "squat": "squat down and stand up (feet flat, IK)",
            "shift": "lean sideways onto one leg (both hip rolls)", "neck": "bend the neck forward, hold, return"}
SHOWN = []
KEY_JOINTS = ["neck_pitch", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "head_pitch"]


def spatial_view():
    kw = dict(origin="world", name=" vs ".join(LABELS.get(n, n) for n in SHOWN) if SHOWN else "robots")
    try:   # fixed camera in front of both robots (they face +X, side by side along -Y)
        c = CENTER_Y[0]
        kw["eye_controls"] = rrb.EyeControls3D(position=[0.42, c - 0.36, 0.24], look_target=[0.02, c, 0.10])   # 3/4 view: shows neck and legs
    except AttributeError:
        pass
    return rrb.Spatial3DView(**kw)


def blueprint(test):
    plots = [rrb.TimeSeriesView(origin=f"plots/{j}/load_pct", name=f"{j}: servo load % of limit") for j in KEY_JOINTS]
    errs = [rrb.TimeSeriesView(origin=f"plots/{j}/err_deg", name=f"{j}: tracking error °") for j in ("neck_pitch", "left_knee")]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(spatial_view(), rrb.TextLogView(origin="captions", name="what's happening"), row_shares=[5, 1]),
            rrb.Grid(*(plots + errs)),
            column_shares=[3, 2]),
        collapse_panels=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--tests", default="stand_hold,squat_0.3,shift_0.12,neck_to_-1.0,neck_to_-1.1")
    ap.add_argument("--url", default="rerun+http://127.0.0.1:9876/proxy")
    ap.add_argument("--gap", type=float, default=0.22)
    ap.add_argument("--showcase", action="store_true", help="all tests back to back in ONE recording")
    ap.add_argument("--app", default="xgoduck_sim_showcase")
    a = ap.parse_args()
    SHOWN[:] = [os.path.basename(sc).replace("scene_", "").replace(".xml", "") for sc in a.scenes]
    CENTER_Y[0] = -(len(a.scenes) - 1) * a.gap / 2
    if a.showcase:
        rec = rr.RecordingStream(application_id=a.app, recording_id="showcase")
        rec.connect_grpc(a.url)
        rec.send_recording_name("sim showcase: all tests back to back")
        rec.send_blueprint(blueprint("showcase"))
        with rec:
            rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            t0 = 0.0
            for test in a.tests.split(","):
                rr.set_time("sim_time", duration=t0)
                rr.log("captions", rr.TextLog(f"▶ {test}: {CAPTIONS.get(test.split('_to_')[0] if '_to_' in test else test.split('_')[0], test)}"))
                durs = []
                for i, sc in enumerate(a.scenes):
                    name = os.path.basename(sc).replace("scene_", "").replace(".xml", "")
                    dur, fell = run(sc, test, name, [0.0, -i * a.gap, 0.0], t0=t0)
                    durs.append(dur)
                    if fell is None:
                        rr.set_time("sim_time", duration=t0 + dur)
                        rr.log("captions", rr.TextLog(f"{test}: {name} stayed upright"))
                t0 += max(durs) + 0.5
        rec.flush()
        print(f"streamed showcase, {t0:.1f} s")
        return
    for test in a.tests.split(","):
        rec = rr.RecordingStream(application_id="xgoduck_servo_tests", recording_id=f"test-{test}")
        rec.connect_grpc(a.url)
        rec.send_recording_name(f"servo test: {test}")
        rec.send_blueprint(blueprint(test))
        with rec:
            rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            for i, sc in enumerate(a.scenes):
                name = os.path.basename(sc).replace("scene_", "").replace(".xml", "")
                run(sc, test, name, [0.0, -i * a.gap, 0.0])
        rec.flush()
        print("streamed", test)


if __name__ == "__main__":
    main()
