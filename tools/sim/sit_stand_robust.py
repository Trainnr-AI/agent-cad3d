#!/usr/bin/env python3
"""STAND -> SIT -> STAND under what the plain model leaves out: servo non-idealities, real print mass, floors.

Per trial, drawn at random within the ranges below (plus fixed worst-case corners):
  servo   - gear friction (Coulomb frictionloss, viscous damping), rotor inertia (armature), position deadband (steps of
            300/1024 deg), output backlash (a passive hinge in series with each servo, +-b), command delay, torque falling
            with speed (stall torque x (1 - w/w0) while driving), servo stiffness (kp) spread.
            Proxy references (no SCS0009 fit exists): BAM m1 fits of the Dynamixel XL320 (Coulomb 0.0275 N m, viscous
            0.0033, armature 0.0011) and XL330 (0.0191, 0.0039, 0.0016, delay 6 ms) - Duclusaud et al., ICRA 2025.
  mass    - PLA fill factor of the printed parts (body_mass_v2.json lists them; built at fill 0.75), bought-part mass of the
            trunk (battery / boards are partly ASSUMED), trunk CoM shift.
  floor   - friction, slope (gravity tilted: pitch + = facing uphill, roll), soft floor (contact time constant: carpet),
            bumps (random heightfield, +-h).
Pass = no part but the feet / trunk touches the floor, back at standing height (+-5 mm) and upright to the floor (< 4 deg),
no servo pinned at its torque limit > 0.1 s, feet slid < 5 mm.

  uv run --with mujoco python -I tools/sim/sit_stand_robust.py projects/xgoduck_scs0009/sim/scene_scs0009_v2.xml \
      --mass projects/xgoduck_scs0009/sim/body_mass_v2.json --n 300 --json out.json
"""
import argparse
import json
import math

import mujoco
import numpy as np

STEP = math.radians(300 / 1024)
RANGES = {   # name: (lo, hi)   nominal model in brackets
    "frictionloss": (0.005, 0.04),        # N m        [0.01]
    "damping": (0.004, 0.03),             # N m s/rad  [0.0216]
    "armature": (0.0005, 0.003),          # kg m2      [0.0015]
    "deadband_steps": (0, 3),             # steps      [0]
    "backlash_deg": (0.0, 1.0),           # +- deg     [0]
    "delay_ms": (0, 20),                  # ms         [0]
    "w0": (4.4, 20.0),                    # rad/s no-load speed at 5 V (datasheet 0.1 s/60 deg at 6 V -> ~8.7)  [inf]
    "kp_scale": (0.5, 1.2),               #            [1]
    "pla_fill": (0.45, 1.0),              #            [0.75]
    "trunk_bought_scale": (0.85, 1.15),   #            [1]
    "trunk_com_dx_mm": (-6.0, 6.0),       #            [0]
    "floor_friction": (0.4, 1.0),         #            [1]
    "slope_pitch_deg": (-5.0, 5.0),       #            [0]
    "slope_roll_deg": (-5.0, 5.0),        #            [0]
    "floor_timeconst": (0.02, 0.1),       # s          [0.02]
    "bump_mm": (0.0, 1.5),                #            [0]
    "bump_wavelength_mm": (10.0, 60.0),   # spacing of the bumps (5 = independent 5 mm cells, gravel-like)
}
NOMINAL = dict(frictionloss=0.01, damping=0.0215855, armature=0.0015, deadband_steps=0, backlash_deg=0.0, delay_ms=0, w0=1e9,
               kp_scale=1.0, pla_fill=0.75, trunk_bought_scale=1.0, trunk_com_dx_mm=0.0, floor_friction=1.0, slope_pitch_deg=0.0,
               slope_roll_deg=0.0, floor_timeconst=0.02, bump_mm=0.0, bump_wavelength_mm=30.0, step_mm=0.0, hip_roll_kp_scale=1.0,
               balance_k=0.0, balance_k_roll=0.0, noslip_iterations=20,
               balance_tau_ms=0.0, balance_dead_deg=0.0, balance_rate_dps=0.0)


def build(scene, p, seed):
    spec = mujoco.MjSpec.from_file(scene)
    base = mujoco.MjModel.from_xml_path(scene)
    acts = [base.actuator(i).name for i in range(base.nu)]
    jnames = [base.joint(base.actuator_trnid[i][0]).name for i in range(base.nu)]
    key = {k: (base.key(k).qpos.copy(), base.key(k).ctrl.copy()) for k in ("STAND", "SIT")}
    for k in list(spec.keys):
        spec.delete(k)
    b = math.radians(p["backlash_deg"])
    for jn in jnames:
        j = spec.joint(jn)
        j.frictionloss, j.armature = p["frictionloss"], p["armature"]
        j.damping = [p["damping"], 0.0, 0.0]          # MuJoCo 4: damping is a 3-vector (linear term first)
        if b > 0:                                           # passive hinge in series: the output can move +-b without the motor
            bl = j.parent.add_joint(name=jn + "_backlash", type=mujoco.mjtJoint.mjJNT_HINGE, axis=j.axis, pos=j.pos,
                                    range=[-b, b], limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=[0.0005, 0.0, 0.0], armature=1e-5)
    floor = spec.geom("floor")
    if p["bump_mm"] > 0:
        rng = np.random.default_rng(seed)
        n = 120                                             # 0.6 m square, 5 mm cells
        h = rng.normal(0, 1, (n, n))
        sig = p.get("bump_wavelength_mm", 5.0) / 4 / 5.0    # Gaussian smoothing (cells) - a bump spacing of ~wavelength
        if sig > 0.3:
            k = np.fft.fftfreq(n)
            h = np.real(np.fft.ifft2(np.fft.fft2(h) * np.exp(-2 * (math.pi * sig) ** 2 * (k[:, None] ** 2 + k[None, :] ** 2))))
        h = (h - h.min()) / (h.max() - h.min())             # 0..1 -> heights 0..2 x bump
        hf = spec.add_hfield(name="bumps", size=[0.3, 0.3, 2 * p["bump_mm"] / 1000, 0.005], nrow=n, ncol=n, userdata=h.flatten().tolist())
        g = spec.worldbody.add_geom(name="floor_bumps", type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname="bumps",
                                    pos=[0, 0, -p["bump_mm"] / 1000], contype=floor.contype, conaffinity=floor.conaffinity)
        floor.name = "floor_plane"
        floor.contype = floor.conaffinity = 0
        g.name = "floor"
    if p.get("step_mm", 0) > 0:                             # the LEFT foot on a step (rug / tile edge) between the feet, along x
        st = p["step_mm"] / 1000
        spec.worldbody.add_geom(name="floor_step", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.15, 0.1, st / 2], pos=[0, 0.1, st / 2],
                                contype=floor.contype, conaffinity=floor.conaffinity)
    spec.memory = 128 * 1024 * 1024                         # the noslip pass over a bumpy floor (300+ contacts) needs a bigger arena
    m = spec.compile()
    # mass: printed parts at another fill, trunk bought parts, trunk CoM
    return m, acts, jnames, key


def apply_mass(m, p, mass_json):
    info = json.load(open(mass_json))
    fill0 = info["fill"]
    for bn, bd in info["bodies"].items():
        bid = m.body(bn).id
        pla = sum(x[1] for x in bd["parts"] if len(x) > 2 and "PLA" in str(x[2])) / 1000
        bought = bd["mass"] - pla
        new = pla * p["pla_fill"] / fill0 + bought * (p["trunk_bought_scale"] if bn == "trunk_base" else 1.0)
        m.body_inertia[bid] *= new / m.body_mass[bid]
        m.body_mass[bid] = new
    m.body_ipos[m.body("trunk_base").id][0] += p["trunk_com_dx_mm"] / 1000


def imu_tilt(d, up):
    """trunk pitch (nose up +) and roll (left side up +) relative to gravity, rad"""
    R = d.xmat[1].reshape(3, 3)
    return math.asin(max(-1, min(1, float(R[:, 0] @ up)))), math.asin(max(-1, min(1, float(R[:, 1] @ up))))


_REF = {}


def reference_tilt(scene, mass_json, move, hold):
    """the planned trunk tilt over time: the same plan on the ideal model, flat floor"""
    key = (scene, move, hold)
    if key not in _REF:
        m, acts, jnames, k = build(scene, NOMINAL, 0)
        apply_mass(m, NOMINAL, mass_json)
        d = mujoco.MjData(m)
        jq = np.array([m.jnt_qposadr[m.joint(j).id] for j in jnames])
        stand, sit = k["STAND"][1], k["SIT"][1]
        d.qpos[:7] = k["STAND"][0][:7]
        d.qpos[2] += 0.001
        d.qpos[jq] = stand
        mujoco.mj_forward(m, d)
        up = np.array([0, 0, 1.0])
        out = []
        for a, b_, T in [(stand, stand, 1.5), (stand, sit, move), (sit, sit, hold), (sit, stand, move), (stand, stand, 1.5)]:
            n = int(T / m.opt.timestep)
            for i in range(n):
                d.ctrl[:] = a + (b_ - a) * (1 - math.cos(math.pi * i / max(1, n - 1))) / 2
                mujoco.mj_step(m, d)
                out.append(imu_tilt(d, up))
        _REF[key] = out
    return _REF[key]


def trial(scene, mass_json, p, move=3.0, hold=2.0, seed=0, on_step=None, every=10):
    m, acts, jnames, key = build(scene, p, seed)
    apply_mass(m, p, mass_json)
    d = mujoco.MjData(m)
    tau = 0.1883
    m.actuator_gainprm[:, 0] *= p["kp_scale"]
    m.actuator_biasprm[:, 1] *= p["kp_scale"]
    for side in ("left", "right"):                         # softer hip roll servos (SCS P gain) let the legs follow an uneven floor
        i = m.actuator(f"{side}_hip_roll").id
        m.actuator_gainprm[i, 0] *= p.get("hip_roll_kp_scale", 1.0)
        m.actuator_biasprm[i, 1] *= p.get("hip_roll_kp_scale", 1.0)
    m.geom_friction[:, 0] = p["floor_friction"]
    m.opt.noslip_iterations = int(p.get("noslip_iterations", 20))   # MuJoCo's soft contacts creep under a steady side load (a 5 deg
                                                                     # slope at friction 1: 8 mm in 10 s); the noslip pass removes that
    fl = m.geom("floor").id
    floors = {fl} | ({m.geom("floor_step").id} if p.get("step_mm", 0) > 0 else set())
    m.geom_solref[fl] = [p["floor_timeconst"], 1.0]
    pr, rr = math.radians(p["slope_pitch_deg"]), math.radians(p["slope_roll_deg"])
    m.opt.gravity[:] = 9.81 * np.array([-math.sin(pr), math.sin(rr) * math.cos(pr), -math.cos(pr) * math.cos(rr)])
    jq = np.array([m.jnt_qposadr[m.joint(j).id] for j in jnames])
    jv = np.array([m.jnt_dofadr[m.joint(j).id] for j in jnames])
    stand, sit = key["STAND"][1], key["SIT"][1]
    d.qpos[:7] = key["STAND"][0][:7]
    d.qpos[2] += 0.001 + p["bump_mm"] / 1000 + p.get("step_mm", 0) / 1000              # set down (1 mm above the highest floor)
    d.qpos[jq] = stand
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    delay = int(round(p["delay_ms"] / 1000 / dt))
    db = p["deadband_steps"] * STEP
    plan = [(stand, stand, 1.5), (stand, sit, move), (sit, sit, hold), (sit, stand, move), (stand, stand, 1.5)]
    targets = []
    for a, b_, T in plan:
        n = int(T / dt)
        for i in range(n):
            s = i / max(1, n - 1)
            targets.append(a + (b_ - a) * (1 - math.cos(math.pi * s)) / 2)
    foot = [m.site("left_foot").id, m.site("right_foot").id]
    n0 = int(1.5 / dt)
    pinned = np.zeros(m.nu)
    run = np.zeros(m.nu)
    pin_at = None
    peak = 0.0
    bad, trunk_down = set(), False
    sit_lo, sit_hi = n0 + int(move / dt), n0 + int((move + hold) / dt)
    ref = reference_tilt(scene, mass_json, move, hold) if (p.get("balance_k", 0) or p.get("balance_k_roll", 0)) else None
    A = {n: i for i, n in enumerate(acts)}
    buf = []
    up = -m.opt.gravity / np.linalg.norm(m.opt.gravity)
    ef, corr = [0.0, 0.0], [0.0, 0.0]                       # filtered tilt error, applied correction (pitch, roll)
    tau_f = p.get("balance_tau_ms", 0.0) / 1000
    dead = math.radians(p.get("balance_dead_deg", 0.0))
    rate = math.radians(p.get("balance_rate_dps", 0.0)) * m.opt.timestep
    for i, tgt in enumerate(targets):
        tgt = tgt.copy()
        if ref is not None:                                 # IMU balance: the trunk's tilt to gravity vs the planned tilt
            pg, rg = imu_tilt(d, up)
            e = (pg - ref[min(i, len(ref) - 1)][0], rg - ref[min(i, len(ref) - 1)][1])
            for j in (0, 1):                                # low-pass, dead zone, rate limit (firmware-style smoothing)
                ef[j] += (e[j] - ef[j]) * (min(1.0, m.opt.timestep / tau_f) if tau_f > 0 else 1.0)
                want = (p["balance_k"], p.get("balance_k_roll", 0.0))[j] * math.copysign(max(abs(ef[j]) - dead, 0.0), ef[j])
                corr[j] += max(-rate, min(rate, want - corr[j])) if rate > 0 else want - corr[j]
            tgt[A["left_ankle"]] += corr[0]
            tgt[A["right_ankle"]] -= corr[0]
            tgt[A["left_hip_roll"]] += corr[1]
            tgt[A["right_hip_roll"]] += corr[1]
        buf.append(tgt)
        cmd = buf[max(0, i - delay)]
        q, w = d.qpos[jq], d.qvel[jv]
        err = cmd - q
        d.ctrl[:] = q + np.sign(err) * np.maximum(np.abs(err) - db, 0.0)
        lim = tau * np.clip(1 - np.abs(w) / p["w0"], 0.05, 1.0)      # driving direction loses torque with speed
        m.actuator_forcerange[:, 0] = np.where(w < 0, -lim, -tau)
        m.actuator_forcerange[:, 1] = np.where(w > 0, lim, tau)
        mujoco.mj_step(m, d)
        if on_step is not None and i % every == 0:          # e.g. stream to a viewer: on_step(m, d, t, actuator names, command)
            on_step(m, d, i * dt, acts, cmd)
        if i == n0:
            f0 = [d.site_xpos[s][:2].copy() for s in foot]
            z0 = d.qpos[2]
        f = np.abs(d.actuator_force)
        atlim = f >= np.minimum(np.abs(m.actuator_forcerange[:, 0]), m.actuator_forcerange[:, 1]) * 0.98
        run = np.where(atlim, run + dt, 0.0)                # longest continuous time each servo sits at its limit
        if run.max() > pinned.max():
            pin_at = i * dt
        pinned = np.maximum(pinned, run)
        peak = max(peak, float((f / tau).max()))
        for c in d.contact[:d.ncon]:
            g = (c.geom1, c.geom2)
            if g[0] in floors or g[1] in floors:
                o = m.geom(g[1] if g[0] in floors else g[0]).name
                if o == "trunk_collision" and sit_lo <= i < sit_hi:
                    trunk_down = True
                if "foot" not in o and o != "trunk_collision":
                    bad.add(o)
    R = d.xmat[1].reshape(3, 3)
    tilt = math.degrees(math.acos(max(-1, min(1, R[2, 2]))))
    slide = max(np.linalg.norm(d.site_xpos[s][:2] - f0[k]) for k, s in enumerate(foot)) * 1000
    dz = (d.qpos[2] - z0) * 1000
    pinned_s = float(pinned.max())
    ok = not bad and abs(dz) < 5 and tilt < 4 and pinned_s < 0.1 and slide < 5
    return dict(ok=bool(ok), fell_on=sorted(bad), dz_mm=round(dz, 1), tilt=round(tilt, 1), slide_mm=round(slide, 1),
                peak_load=round(peak, 2), pinned_s=round(pinned_s, 2), trunk_down=trunk_down,
                worst_pinned=acts[int(np.argmax(pinned))],
                pinned_phase=None if pin_at is None else ("stand" if pin_at < 1.5 else "sitting down" if pin_at < 1.5 + move else
                                                          "sitting" if pin_at < 1.5 + move + hold else "standing up" if pin_at < 1.5 + 2 * move + hold else "stand after"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--mass", required=True)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--move", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json")
    ap.add_argument("--balance-k", type=float, default=0.0, help="IMU balance: ankle correction per rad of trunk pitch error (-1.5 tuned)")
    ap.add_argument("--balance-tau-ms", type=float, default=0.0, help="low-pass on the tilt error (25 tuned)")
    ap.add_argument("--balance-dead-deg", type=float, default=0.0, help="dead zone on the tilt error (0.2 tuned)")
    ap.add_argument("--noslip", type=int, default=0, help="noslip iterations (removes contact creep on slopes; slow on bumps)")
    a = ap.parse_args()
    ctl = dict(balance_k=a.balance_k, balance_k_roll=0.0, noslip_iterations=a.noslip, balance_tau_ms=a.balance_tau_ms,
               balance_dead_deg=a.balance_dead_deg)
    rng = np.random.default_rng(a.seed)
    out = {"nominal": trial(a.scene, a.mass, dict(NOMINAL, **ctl), a.move)}
    print("nominal:", out["nominal"])
    worst = dict(frictionloss=0.04, damping=0.03, armature=0.003, deadband_steps=3, backlash_deg=1.0, delay_ms=20, w0=4.4,
                 kp_scale=0.5, pla_fill=1.0, trunk_bought_scale=1.15, floor_friction=0.4, floor_timeconst=0.1, bump_mm=1.5)
    corners = {}
    for dx in (-6.0, 6.0):
        for sp in (-5.0, 5.0):
            for sr in (0.0, 5.0):
                corners[f"worst, CoM {dx:+.0f} mm, slope pitch {sp:+.0f} roll {sr:+.0f}"] = dict(worst, trunk_com_dx_mm=dx, slope_pitch_deg=sp, slope_roll_deg=sr)
    out["corners"] = {}
    for lab, p in corners.items():
        r = trial(a.scene, a.mass, dict(p, **ctl), a.move, seed=7)
        out["corners"][lab] = r
        print(("PASS " if r["ok"] else "FAIL ") + lab, r)
    trials = []
    for t in range(a.n):
        p = {k: float(rng.uniform(*v)) for k, v in RANGES.items()}
        p["deadband_steps"] = int(round(p["deadband_steps"]))
        r = trial(a.scene, a.mass, dict(p, **ctl), a.move, seed=t)
        if (t + 1) % 25 == 0:
            print(f"  {t + 1}/{a.n} done", flush=True)
        trials.append({"params": p, **r})
    ok = sum(t["ok"] for t in trials)
    pk = [t["peak_load"] for t in trials]
    print(f"random: {ok}/{a.n} pass; trunk down {sum(t['trunk_down'] for t in trials)}/{a.n}; peak load median "
          f"{100 * np.median(pk):.0f}% p95 {100 * np.percentile(pk, 95):.0f}% max {100 * max(pk):.0f}%")
    why = {"fell": 0, "pinned >= 0.1 s": 0, "tilted >= 4 deg": 0, "slid >= 5 mm": 0, "height off": 0}
    for t in trials:
        if t["ok"]:
            continue
        if t["fell_on"]:
            why["fell"] += 1
        else:
            why["pinned >= 0.1 s"] += t["pinned_s"] >= 0.1
            why["tilted >= 4 deg"] += t["tilt"] >= 4
            why["slid >= 5 mm"] += t["slide_mm"] >= 5
            why["height off"] += abs(t["dz_mm"]) >= 5
    print("failures by reason (a trial can have several):", why, flush=True)
    for t in [t for t in trials if not t["ok"]][:15]:
        print("  FAIL", {k: round(v, 3) for k, v in t["params"].items()}, {k: v for k, v in t.items() if k not in ("params", "ok")})
    out["random"] = trials
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
