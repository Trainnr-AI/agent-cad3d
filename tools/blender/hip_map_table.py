#!/usr/bin/env python3
"""Hip map: merge clearance_sweep.py results into data/hip_collision_map.json and print the JOINT_LIMITS.md tables.
Plain Python (no Blender).

  python3 tools/blender/hip_map_table.py --left a.json [b.json ...] --right ... --both ... \
      --out projects/xgoduck_scs0009/data/hip_collision_map.json --margin 0.5

Sweep poses carry joint angles in each leg's own MJCF signs; the map stores them per leg, and the tables use the LEFT leg's
signs for both legs (right leg mirrored: hip pitch + = leg back, roll + = out, yaw + = toe out). A cell allows a hip roll when
every knee in the group is clear: no collision and no gap under the margin (default 0.3 mm, the sweep's accuracy floor; pairs already that close at rest - bearings on their
bosses, servos in their holders - are not judged; see the sweep's rest_pairs; and
--ignore pairs whose gap is a single joint's own limit, e.g. the hip-yaw bearing 0.3 mm from the L-bracket at roll +19.5).
"""
import argparse
import json

J = ("hip_pitch", "hip_roll", "hip_yaw", "knee")
KNEE_MAIN, KNEE_FOLD = (-45.0, 0.0, 68.0), (-90.0,)


def load(files):
    poses, rest = [], set()
    for f in files:
        d = json.load(open(f))
        poses += d["poses"]
        rest |= set(d.get("rest_pairs", []))
    return poses, sorted(rest)


def leg_pose(x, side):
    s = 1 if side == "left" else -1
    q = {j: s * x["q"][f"{side}_{j}"] for j in J}               # left-leg signs
    return q, {j: x["q"][f"{side}_{j}"] for j in J}             # (mirrored to left signs, own MJCF signs)


def close_gap(x, margin, ignore):
    """smallest gap (mm) under the margin among the pose's pairs, leaving out the ignored pairs; None if none"""
    g = [float(c.split("gap ")[1].split()[0]) for c in x["pairs"]
         if "gap " in c and not any(all(w in c.split(":")[0] for w in i.split("*")) for i in ignore)]
    g = [v for v in g if v < margin]
    return min(g) if g else None


def entry(own, x, margin, ignore):
    gap = close_gap(x, margin, ignore)
    return {**own, "collides": x["collides"], "under_margin": (not x["collides"]) and gap is not None,
            "min_gap_mm": gap, "max_depth_mm": x["max_depth_mm"], "contacts": x["pairs"]}


def fmt(rolls, allowed):
    ok = [r for r in rolls if allowed[r]]
    if len(ok) == len(rolls):
        return "all"
    if not ok:
        return "**none**"
    runs, cur = [], [ok[0]]
    for r in ok[1:]:
        if rolls.index(r) == rolls.index(cur[-1]) + 1:
            cur.append(r)
        else:
            runs.append(cur)
            cur = [r]
    runs.append(cur)
    return ", ".join(f"{c[0]:+g}..{c[-1]:+g}" if len(c) > 1 else f"{c[0]:+g}" for c in runs)


def table(cells, grid, label):
    rolls, yaws = grid["hip_roll"], grid["hip_yaw"]
    out = [f"| hip pitch ({label}) | " + " | ".join(f"yaw {y:+g}" for y in yaws) + " |", "|---|" + "---|" * len(yaws)]
    for p in grid["hip_pitch"]:
        row = []
        for y in yaws:
            main = {r: all(cells[(p, r, y, k)] for k in KNEE_MAIN) for r in rolls}
            fold = {r: main[r] and all(cells[(p, r, y, k)] for k in KNEE_FOLD) for r in rolls}
            a, b = fmt(rolls, main), fmt(rolls, fold)
            row.append(a if a == b else f"{a} (folded: {b.replace('**', '')})")
        out.append(f"| {p:+g} | " + " | ".join(row) + " |")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", nargs="+", required=True)
    ap.add_argument("--right", nargs="+", required=True)
    ap.add_argument("--both", nargs="+")
    ap.add_argument("--grid", required=True, help="json with the grid (the old map)")
    ap.add_argument("--out")
    ap.add_argument("--margin", type=float, default=0.3)
    ap.add_argument("--ignore", nargs="*", default=["hip_yaw_brg_6702ZZ x *_l_bracket"],
                    help="pairs whose small gap belongs to one joint's own limit (JOINT_LIMITS layer 1), not to the map; '*' = any text")
    a = ap.parse_args()
    grid = json.load(open(a.grid))["grid"]
    Lp, rest = load(a.left)
    Rp, _ = load(a.right)
    left, right, cells = [], [], {"left": {}, "right": {}}
    for side, P, store in (("left", Lp, left), ("right", Rp, right)):
        for x in P:
            mir, own = leg_pose(x, side)
            e = entry(own, x, a.margin, a.ignore)
            store.append(e)
            cells[side][tuple(mir[j] for j in J)] = not e["collides"] and not e["under_margin"]
    both = []
    if a.both:
        Bp, _ = load(a.both)
        for x in Bp:
            l2l = [c for c in x["pairs"] if "left_" in c.split(":")[0] and "right_" in c.split(":")[0]]
            both.append({"mode": "same" if " sym " in x["label"] else "opposite", "left": leg_pose(x, "left")[1],
                         "right": leg_pose(x, "right")[1], "collides": x["collides"],
                         "legs_collide": any("depth" in c and float(c.split("depth ")[1].split()[0]) > 0.05 for c in l2l),
                         "min_gap_mm": x["min_gap_mm"], "max_depth_mm": x["max_depth_mm"], "contacts": x["pairs"]})
    if a.out:
        doc = {"what": "each leg vs the rest of the robot, and both legs together, on the exact parts "
                           "(tools/blender/clearance_sweep.py, 2026-11-15; replaces the 2026-11-14 left-only volume map). Degrees. "
                           "'poses' = LEFT leg, 'right_poses' = RIGHT leg, each in its own MJCF signs (left: hip pitch + = leg back, "
                           "roll + = out, yaw + = toe out; right: all four signs flipped - the right leg is NOT an exact mirror: see "
                           "JOINT_LIMITS.md). Ankle 0, other joints at rest. 'both_legs_poses': both legs moved together, 'same' = "
                           "mirror-symmetric, 'opposite' = one leg's hip pitch mirrored the other way. collides = surfaces cross deeper "
                           f"than 0.05 mm; under_margin = clear but closer than {a.margin} mm.",
                   "margin_mm": a.margin, "margin_ignores": a.ignore, "grid": grid, "rest_pairs_not_judged": rest,
                   "poses": left, "right_poses": right, "both_legs_poses": both}
        try:                                              # keep extra sections added by other runs (ankle_pm23_*, knee_ankle_*)
            old = json.load(open(a.out))
            doc.update({k: v for k, v in old.items() if k not in doc})
            if "ankle_pm23_left" in old:
                doc["what"] += old["what"][old["what"].find(" 'ankle_pm23_left/right'"):]
        except (OSError, ValueError):
            pass
        with open(a.out, "w") as f:                       # one pose per line: readable diffs
            f.write("{\n")
            keys = list(doc)
            for n, k in enumerate(keys):
                v = doc[k]
                end = ",\n" if n < len(keys) - 1 else "\n"
                if isinstance(v, list) and v and isinstance(v[0], dict):
                    f.write(json.dumps(k) + ": [\n" + ",\n".join(json.dumps(x, separators=(",", ":")) for x in v) + "\n]" + end)
                else:
                    f.write(json.dumps(k) + ": " + json.dumps(v) + end)
            f.write("}\n")
    print(table(cells["left"], grid, "left leg"))
    print()
    print(table(cells["right"], grid, "right leg, mirrored"))
    print()
    print("left collide", sum(p["collides"] for p in left), "under margin", sum(p["under_margin"] for p in left),
          "| right collide", sum(p["collides"] for p in right), "under margin", sum(p["under_margin"] for p in right),
          "| both: legs collide", sum(p["legs_collide"] for p in both))


if __name__ == "__main__":
    main()
