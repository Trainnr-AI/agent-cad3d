#!/usr/bin/env python3
"""Stream one sit_stand_robust.py trial (an imperfect robot / floor) into a Rerun viewer, e.g. the trainnr Studio:
the same conditions twice side by side - servos holding angles only (open loop) and with the IMU ankle balance.
The floor slope is shown by tilting each robot's frame with its floor (simulated with gravity tilted).

  uv run --with rerun-sdk==0.36.2 --with mujoco python -I tools/sim/rerun_robust.py projects/xgoduck_scs0009/sim/scene_scs0009_v2.xml \
      --mass projects/xgoduck_scs0009/sim/body_mass_v2.json --params '{"slope_pitch_deg": 2.7, ...}' --app nanoduck_sit_robust
"""
import argparse
import importlib.util
import json
import math
import os

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, f"{name}.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ssr = load("sit_stand_robust")
rp = load("rerun_replay")


def slope_quat(p):
    """rotation that turns the simulated gravity (tilted) into straight down: shows the floor tilted"""
    pr, rr_ = math.radians(p["slope_pitch_deg"]), math.radians(p["slope_roll_deg"])
    g = np.array([-math.sin(pr), math.sin(rr_) * math.cos(pr), -math.cos(pr) * math.cos(rr_)])
    a = np.cross(g, [0, 0, -1.0])
    s, c = np.linalg.norm(a), float(np.dot(g, [0, 0, -1.0]))
    if s < 1e-9:
        return [0, 0, 0, 1]
    ang = math.atan2(s, c)
    a = a / s * math.sin(ang / 2)
    return [a[0], a[1], a[2], math.cos(ang / 2)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--mass", required=True)
    ap.add_argument("--params", required=True, help="JSON: sit_stand_robust parameters (missing ones = nominal)")
    ap.add_argument("--balance-k", type=float, default=-1.5)
    ap.add_argument("--url", default="rerun+http://127.0.0.1:9876/proxy")
    ap.add_argument("--app", default="nanoduck_sit_robust")
    ap.add_argument("--gap", type=float, default=0.26)
    a = ap.parse_args()
    p = dict(ssr.NOMINAL, **json.loads(a.params))
    runs = [("open_loop", 0.0, [235, 120, 90]), ("imu_balance", a.balance_k, [80, 160, 240])]
    rp.LABELS.update(open_loop="open loop: servos only hold their angles (orange)",
                     imu_balance=f"IMU ankle balance, k {a.balance_k} (blue)")
    rec = rr.RecordingStream(application_id=a.app, recording_id=f"robust-{int(__import__('time').time())}")
    rec.connect_grpc(a.url)
    rec.send_recording_name("sit/stand on an imperfect robot: open loop vs IMU balance")
    rp.SHOWN[:] = [r[0] for r in runs]
    rp.CENTER_Y[0] = -a.gap / 2
    rec.send_blueprint(rp.blueprint("robust"))
    q = slope_quat(p)
    desc = ", ".join(f"{k} {v:g}" for k, v in json.loads(a.params).items())
    with rec:
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.set_time("sim_time", duration=0.0)
        rr.log("captions", rr.TextLog(f"▶ same robot and floor twice: {desc}"))
        for k, (name, bk, color) in enumerate(runs):
            prefix = f"world/{name}"
            rr.log(prefix, rr.Transform3D(translation=[0, -k * a.gap, 0], quaternion=rr.Quaternion(xyzw=q)), static=True)
            rr.log(f"{prefix}/floor", rr.Boxes3D(centers=[[0, 0, -0.002]], half_sizes=[[0.12, 0.09, 0.002]], colors=[[120, 120, 120]]), static=True)
            state = {"static": False, "fell": False}

            def on_step(m, d, t, acts, cmd, prefix=prefix, name=name, color=color, state=state):
                if not state["static"]:
                    rp.log_robot_static(m, prefix, color)
                    state["static"] = True
                rr.set_time("sim_time", duration=t)
                for b in range(1, m.nbody):
                    qq = d.xquat[b]
                    rr.log(f"{prefix}/{m.body(b).name}", rr.Transform3D(translation=d.xpos[b], quaternion=rr.Quaternion(xyzw=[qq[1], qq[2], qq[3], qq[0]])))
                load_ = 100 * np.abs(d.actuator_force) / 0.1883
                jq = [m.jnt_qposadr[m.joint(m.actuator_trnid[i][0]).id] for i in range(m.nu)]
                for i, n in enumerate(acts):
                    if n.startswith("right_"):
                        continue
                    rr.log(f"plots/{n}/load_pct/{name}", rr.Scalars(load_[i]))
                    rr.log(f"plots/{n}/err_deg/{name}", rr.Scalars(math.degrees(abs(cmd[i] - d.qpos[jq[i]]))))
                if not state["fell"] and d.xmat[1].reshape(3, 3)[2, 2] < math.cos(math.radians(50)):
                    state["fell"] = True
                    rr.log("captions", rr.TextLog(f"{name}: FELL at {t:.1f} s", level="WARN"))

            res = ssr.trial(a.scene, a.mass, dict(p, balance_k=bk, balance_k_roll=0.0), on_step=on_step, every=10)
            rr.set_time("sim_time", duration=12.0)
            rr.log("captions", rr.TextLog(f"{name}: {'PASS' if res['ok'] else 'FAIL'} - fell on {res['fell_on'] or 'nothing'}, "
                                          f"peak load {100 * res['peak_load']:.0f} %, longest at limit {res['pinned_s']} s",
                                          level="INFO" if res["ok"] else "WARN"))
            print(name, res)
    rec.flush()


if __name__ == "__main__":
    main()
