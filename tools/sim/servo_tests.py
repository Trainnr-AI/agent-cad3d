#!/usr/bin/env python3
"""Like-for-like servo tests on two (or more) MuJoCo scenes built by build_scaled_mjcf.py.

Every robot gets the same scripted commands (joint targets) and is judged the same way:
fell?, trunk sag, how closely each joint follows its target, and how hard each servo works
(peak |torque| as % of its limit, % of time saturated).

  uv run --with mujoco python -I tools/sim/servo_tests.py projects/xgoduck_scs0009/sim/scene_original.xml \
      projects/xgoduck_scs0009/sim/scene_scs0009.xml --json out.json
"""
import argparse
import json
import math

import mujoco
import numpy as np

DT_CTRL = 0.01


def smooth(a, b, t):                      # cosine blend 0..1
    t = min(max(t, 0.0), 1.0)
    return a + (b - a) * (1 - math.cos(math.pi * t)) / 2


def key_ctrl(m, name):
    return np.array(m.key(name).ctrl)


def squat_pose(m, stand, frac):
    """IK: for each leg, move the foot site up (toward the hip) by frac x hip-to-foot height while
    keeping its orientation and fore-aft position; solved on hip_pitch/knee/ankle by Newton."""
    d = mujoco.MjData(m)
    aid = {m.actuator(i).name: i for i in range(m.nu)}
    jq = {n: m.jnt_qposadr[m.actuator_trnid[i][0]] for n, i in aid.items()}
    q = stand.copy()

    def foot(qq, side):
        d.qpos[:] = 0
        d.qpos[3] = 1
        for n, i in aid.items():
            d.qpos[jq[n]] = qq[i]
        mujoco.mj_kinematics(m, d)
        sid = m.site(f"{side}_foot").id
        hip = d.xanchor[m.joint(f"{side}_hip_pitch").id]
        return d.site_xpos[sid].copy(), d.site_xmat[sid].reshape(3, 3).copy(), hip.copy()

    for side in ("left", "right"):
        p0, R0, hip = foot(stand, side)
        target = p0 + np.array([0, 0, frac * (hip[2] - p0[2])])
        names = [f"{side}_hip_pitch", f"{side}_knee", f"{side}_ankle"]

        def resid(qq):
            p, R, _ = foot(qq, side)
            dR = R @ R0.T                                   # orientation error (pitch about world y)
            return np.array([p[0] - target[0], p[2] - target[2], (dR[0, 2] - dR[2, 0]) / 2])
        for _ in range(40):
            r = resid(q)
            if np.linalg.norm(r) < 1e-7:
                break
            J = np.zeros((3, 3))
            for k, n in enumerate(names):
                dq = q.copy()
                dq[aid[n]] += 1e-5
                J[:, k] = (resid(dq) - r) / 1e-5
            step = np.linalg.lstsq(J, -r, rcond=None)[0]
            for k, n in enumerate(names):
                q[aid[n]] += step[k]
    return q


def script(m, test, sign=1):
    """Returns (duration_s, fn(t) -> ctrl) for a named test."""
    stand, sit = key_ctrl(m, "STAND"), key_ctrl(m, "SIT")
    aid = {m.actuator(i).name: i for i in range(m.nu)}
    if test == "stand_hold":
        return 4.0, lambda t: stand
    if test == "sit_stand":
        def f(t):
            if t < 1.5:
                return smooth(stand, sit, t / 1.5)
            if t < 2.5:
                return sit
            if t < 4.0:
                return smooth(sit, stand, (t - 2.5) / 1.5)
            return stand
        return 5.0, f
    if test.startswith("pose:"):               # pose:neck_pitch=-1.2;head_pitch=1.2 - several joints together
        lo = stand.copy()
        for kv in test[5:].split(";"):
            jn, ang = kv.split("=")
            lo[aid[jn]] = float(ang)
        return 4.0, lambda t: smooth(stand, lo, t / 1.5) if t < 1.5 else (lo if t < 2.5 else smooth(lo, stand, (t - 2.5) / 1.5))
    if test.startswith("set:"):                # e.g. set:head_pitch:-1.5 - move one joint there, hold, return
        parts = test.split(":")                # set:<joint>:<angle>[:<move seconds>]
        jn, ang = parts[1], float(parts[2])
        mv = float(parts[3]) if len(parts) > 3 else 1.5
        lo = stand.copy()
        lo[aid[jn]] = ang
        return 2 * mv + 1.0, lambda t: smooth(stand, lo, t / mv) if t < mv else (lo if t < mv + 1.0 else smooth(lo, stand, (t - mv - 1.0) / mv))
    if test.startswith("neck_to_"):            # e.g. neck_to_-0.8: bend the neck to that angle, hold, return
        lo = stand.copy()
        lo[aid["neck_pitch"]] = float(test[len("neck_to_"):])
        return 4.0, lambda t: smooth(stand, lo, t / 1.5) if t < 1.5 else (lo if t < 2.5 else smooth(lo, stand, (t - 2.5) / 1.5))
    if test.startswith("squat_"):              # e.g. squat_0.25: lower the trunk by that fraction of leg length,
        frac = float(test[len("squat_"):])     # feet flat and in place (IK on hip pitch, knee, ankle)
        sq = squat_pose(m, stand, frac)

        def f(t):
            if t < 1.5:
                return smooth(stand, sq, t / 1.5)
            if t < 2.5:
                return sq
            if t < 4.0:
                return smooth(sq, stand, (t - 2.5) / 1.5)
            return stand
        return 5.0, f
    if test.startswith("shift_"):              # e.g. shift_0.12: both hip rolls lean the body sideways, hold, return
        r = float(test[len("shift_"):])
        sh = stand.copy()
        for j in ("left_hip_roll", "right_hip_roll"):
            sh[aid[j]] += r
        return 4.0, lambda t: smooth(stand, sh, t) if t < 1.0 else (sh if t < 3.0 else smooth(sh, stand, t - 3.0))
    if test == "one_leg":
        # shift hips sideways with both hip rolls, then lift the unloaded foot (hip pitch + knee)
        shift = stand.copy()
        for j in ("left_hip_roll", "right_hip_roll"):
            shift[aid[j]] += sign * 0.17
        lift = shift.copy()
        free = "right" if sign > 0 else "left"
        s_hp = 1 if free == "right" else -1
        lift[aid[f"{free}_hip_pitch"]] += s_hp * 0.45
        lift[aid[f"{free}_knee"]] += s_hp * 0.9
        lift[aid[f"{free}_ankle"]] += -s_hp * 0.45

        def f(t):
            if t < 1.0:
                return smooth(stand, shift, t)
            if t < 1.8:
                return smooth(shift, lift, (t - 1.0) / 0.8)
            if t < 3.8:
                return lift
            return smooth(lift, shift, (t - 3.8) / 0.8)
        return 5.0, f
    raise ValueError(test)


def run(scene, test, sign=1):
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("STAND").id)
    mujoco.mj_forward(m, d)
    dur, f = script(m, test, sign)
    limit = np.array([m.actuator_forcerange[i][1] for i in range(m.nu)])
    jq = np.array([m.jnt_qposadr[m.actuator_trnid[i][0]] for i in range(m.nu)])
    trunk = m.body("trunk_base").id
    z0 = d.xpos[trunk][2]
    n_sub = max(1, int(round(DT_CTRL / m.opt.timestep)))
    peak, sat, err, steps, fell, min_up = np.zeros(m.nu), np.zeros(m.nu), np.zeros(m.nu), 0, False, 1.0
    t = 0.0
    while t < dur:
        d.ctrl[:] = f(t)
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        t += DT_CTRL
        steps += 1
        tau = np.abs(d.actuator_force)
        peak = np.maximum(peak, tau / limit)
        sat += tau >= 0.98 * limit
        err = np.maximum(err, np.abs(d.ctrl - d.qpos[jq]))
        up = d.xmat[trunk].reshape(3, 3)[2, 2]
        min_up = min(min_up, up)
        if d.xpos[trunk][2] < 0.55 * z0 or up < math.cos(math.radians(50)):
            fell = True
            break
    names = [m.actuator(i).name for i in range(m.nu)]
    return {"fell": fell, "t_end": round(t, 2), "sag_mm": round((z0 - d.xpos[trunk][2]) * 1000, 1),
            "max_tilt_deg": round(math.degrees(math.acos(max(-1, min(1, min_up)))), 1),
            "joints": {n: {"peak_pct": round(100 * peak[i], 0), "sat_pct": round(100 * sat[i] / steps, 0),
                           "max_err_deg": round(math.degrees(err[i]), 1)} for i, n in enumerate(names)}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--tests", default="stand_hold,squat_0.9,shift_0.12,neck_to_-0.4,neck_to_-0.8,neck_to_-1.2")
    ap.add_argument("--json")
    a = ap.parse_args()
    results = {}
    for test in a.tests.split(","):
        sign = 1
        if test == "one_leg":                 # pick the side direction that works for the FIRST (reference) robot
            r = [run(a.scenes[0], test, s) for s in (1, -1)]
            sign = 1 if not r[0]["fell"] or r[1]["fell"] else -1
        for sc in a.scenes:
            results.setdefault(test, {})[sc.split("scene_")[-1].replace(".xml", "")] = run(sc, test, sign)
    for test, by in results.items():
        print(f"\n=== {test}")
        robots = list(by)
        for r in robots:
            x = by[r]
            print(f"  {r:10} {'FELL at ' + str(x['t_end']) + ' s' if x['fell'] else 'upright'}; sag {x['sag_mm']} mm; max tilt {x['max_tilt_deg']}°")
        print(f"  {'joint':16}" + "".join(f" | {r[:10]:>10} peak% sat% err°" for r in robots))
        for j in by[robots[0]]["joints"]:
            row = f"  {j:16}"
            for r in robots:
                v = by[r]["joints"][j]
                row += f" | {v['peak_pct']:>16.0f} {v['sat_pct']:>4.0f} {v['max_err_deg']:>4.1f}"
            print(row)
    if a.json:
        json.dump(results, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
