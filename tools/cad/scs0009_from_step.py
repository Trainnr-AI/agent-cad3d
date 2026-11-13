"""Feetech SCS0009: STEP -> per-solid STL (for Blender) + exact feature report.

    uv run --python 3.12 --with cadquery --with numpy python -I tools/cad/scs0009_from_step.py \
        vendor/feetech/SCS009-20230110-S.stp tools/blender/parts/scs0009_mesh [--report]

The STEP (Feetech assembly SCS009-20201118-S, Pro/E, exported 2023-01-10) holds four solids: top case,
core middle, bottom case, output gear; the motor/PCB sub-assembly is empty and there are no screws or
horns. STLs are written in the STEP frame in mm (x = width, y = shaft axis, z = body length; shaft axis
at x = 0, z = -5.7; case bottom at y = -1.45). parts/scs0009.py maps them to the joint frame.
--report lists every cylindrical face (holes, bosses, spline, fillets) and every horizontal plane per
solid — the numbers behind the constants in parts/scs0009.py.
"""
import json
import os
import sys

import numpy as np
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface

VOL_NAMES = {904.09: "topcase", 579.56: "coremiddle", 448.59: "bottomcase", 220.37: "outputgear"}


def load(path):
    solids = cq.importers.importStep(path).solids().vals()
    if len(solids) != 4:
        raise SystemExit(f"expected 4 solids, got {len(solids)}")
    return {VOL_NAMES[min(VOL_NAMES, key=lambda k: abs(k - s.Volume()))]: s for s in solids}


def cylinders(solid):
    rows = {}
    for f in solid.Faces():
        if f.geomType() != "CYLINDER":
            continue
        c = BRepAdaptor_Surface(f.wrapped).Cylinder()
        d, p = c.Axis().Direction(), c.Axis().Location()
        dv, pv = np.array([d.X(), d.Y(), d.Z()]), np.array([p.X(), p.Y(), p.Z()])
        k = int(np.argmax(np.abs(dv)))
        bb = f.BoundingBox()
        lo, hi = [bb.xmin, bb.ymin, bb.zmin][k], [bb.xmax, bb.ymax, bb.zmax][k]
        centre = [None if i == k else round(float(pv[i]), 3) for i in range(3)]
        key = ("xyz"[k], round(c.Radius(), 3), tuple(None if v is None else round(v, 2) for v in centre))
        r = rows.setdefault(key, dict(axis="xyz"[k], r=round(c.Radius(), 3), centre=centre, span=[lo, hi], faces=0))
        r["span"] = [round(min(r["span"][0], lo), 3), round(max(r["span"][1], hi), 3)]
        r["faces"] += 1
    return sorted(rows.values(), key=lambda r: (r["axis"], r["r"], str(r["centre"])))


def planes(solid):
    levels = {}
    for f in solid.Faces():
        if f.geomType() != "PLANE" or abs(f.normalAt().y) < 0.999:
            continue
        bb = f.BoundingBox()
        key = (round(bb.ymin, 3), "+y" if f.normalAt().y > 0 else "-y")
        L = levels.setdefault(key, dict(y=key[0], normal=key[1], area=0.0, x=[bb.xmin, bb.xmax], z=[bb.zmin, bb.zmax]))
        L["area"] = round(L["area"] + f.Area(), 2)
        L["x"] = [round(min(L["x"][0], bb.xmin), 3), round(max(L["x"][1], bb.xmax), 3)]
        L["z"] = [round(min(L["z"][0], bb.zmin), 3), round(max(L["z"][1], bb.zmax), 3)]
    return [L for _, L in sorted(levels.items()) if L["area"] >= 0.5]


def main():
    path, out = sys.argv[1], sys.argv[2]
    report = "--report" in sys.argv
    os.makedirs(out, exist_ok=True)
    parts = load(path)
    bb = cq.Compound.makeCompound(list(parts.values())).BoundingBox()
    print(f"assembly bbox x[{bb.xmin:.3f},{bb.xmax:.3f}] y[{bb.ymin:.3f},{bb.ymax:.3f}] z[{bb.zmin:.3f},{bb.zmax:.3f}]"
          f"  (datasheet 12.1 wide, 28.45 tall incl. spline, 32.5 over the ears)")
    dims = {"frame": "STEP: x width, y shaft axis (case bottom y=-1.45, tower top 23.8, spline tip 27.0), z length (shaft z=-5.7)",
            "bbox": [[bb.xmin, bb.ymin, bb.zmin], [bb.xmax, bb.ymax, bb.zmax]], "parts": {}}
    for name, s in parts.items():
        fn = os.path.join(out, f"scs0009_{name}.stl")
        cq.exporters.export(cq.Workplane().add(s), fn, tolerance=0.005, angularTolerance=0.35)
        pb = s.BoundingBox()
        dims["parts"][name] = dict(volume_mm3=round(s.Volume(), 3), bbox=[[pb.xmin, pb.ymin, pb.zmin], [pb.xmax, pb.ymax, pb.zmax]],
                                   cylinders=cylinders(s), planes=planes(s))
        print(f"  {name:11s} vol={s.Volume():8.2f} mm3  -> {fn} ({os.path.getsize(fn)/1e6:.2f} MB)")
        if report:
            print(f"    horizontal planes (y, normal, area, x-extent, z-extent):")
            for L in dims["parts"][name]["planes"]:
                print(f"      y={L['y']:7.3f} {L['normal']} area={L['area']:8.2f} x{L['x']} z{L['z']}")
            print(f"    cylinders (axis, r, centre, span):")
            for r in dims["parts"][name]["cylinders"]:
                if r["r"] >= 0.3:                      # skip the sub-0.3 mm fillets
                    print(f"      {r['axis']} r={r['r']:6.3f} centre={r['centre']} span={r['span']} faces={r['faces']}")
    json.dump(dims, open(os.path.join(out, "scs0009_dims.json"), "w"), indent=1)
    print(f"wrote {os.path.join(out, 'scs0009_dims.json')}")


if __name__ == "__main__":
    main()
