"""Report which mesh objects intersect each other (triangle-level, BVH). Runs inside Blender.

ARGS = {"collection": "duck_scs0009", "include": ["scs_"], "exclude": ["_tmp"],
        "pairs_with": ["duck_scs0009:"] (optional: also test included objects against these)}
Prints every intersecting pair with the number of intersecting triangle pairs. Coincident
touching faces count as intersections, so treat tiny counts (<5) as contact, not collision.
"""
import bpy
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
coll = bpy.data.collections[args["collection"]]
inc = args.get("include", [""])
exc = args.get("exclude", [])
dg = bpy.context.evaluated_depsgraph_get()


def pick(prefixes):
    return [o for o in coll.all_objects if o.type == "MESH" and any(p in o.name for p in prefixes)
            and not any(x in o.name for x in exc)]


def tree(o):
    me = o.evaluated_get(dg).to_mesh()
    mw = o.matrix_world
    t = BVHTree.FromPolygons([mw @ v.co for v in me.vertices], [p.vertices[:] for p in me.polygons])
    o.evaluated_get(dg).to_mesh_clear()
    return t


A = pick(inc)
B = pick(args["pairs_with"]) if args.get("pairs_with") else []
trees = {o.name: tree(o) for o in set(A) | set(B)}
hits = []
for i, a in enumerate(A):
    others = A[i + 1:] + [b for b in B if b not in A]
    for b in others:
        # skip parts of the same servo (case/spline/horn share a root)
        if a.parent is not None and a.parent == b.parent:
            continue
        n = len(trees[a.name].overlap(trees[b.name]))
        if n:
            hits.append((n, a.name, b.name))
for n, a, b in sorted(hits, reverse=True):
    print(f"  {n:5d}  {a}  x  {b}")
print(f"{len(hits)} intersecting pairs among {len(A)} objects" + (f" (+{len(B)} others)" if B else ""))
