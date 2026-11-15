"""Assembly paths that are not straight: each part is moved along segments (e.g. down, then sideways) in order, from the given
amount back to 0, and must not enter the listed parts by more than 0.1 mm. Runs inside Blender.
ARGS = {"paths": [{"name": "...", "parts": [...], "against": [...], "segments": [[[dx, dy, dz], amount_mm], ...]}]}"""
import bpy, bmesh
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
def wt(o):
    bm=bmesh.new(); bm.from_mesh(o.data); bm.transform(o.matrix_world); t=BVHTree.FromBMesh(bm); bm.free(); return t
def depth(vs, t):
    d0=0.0; d=Vector((0.137,0.311,0.94)).normalized()
    for v in vs[::2]:
        c=0; q=v.copy()
        for _ in range(60):
            h=t.ray_cast(q,d)
            if h[0] is None: break
            c+=1; q=h[0]+d*1e-7
        if c%2==1: d0=max(d0,t.find_nearest(v)[3]*1000)
    return d0
out=[]
for st in ARGS["paths"]:
    parts=[bpy.data.objects[n] for n in st["parts"]]
    against={n:wt(bpy.data.objects[n]) for n in st["against"]}
    segs=[(Vector(s[0]).normalized(), s[1]) for s in st["segments"]]   # executed in order; each from its amount to 0
    worst=[]
    for i,(d,a) in enumerate(segs):
        rest=sum((dd*aa for dd,aa in segs[i+1:]), Vector())
        n=max(1,int(abs(a)/0.5))
        for s in range(n+1):
            k=a*(1-s/n)
            off=(d*k+rest)/1000; M=Matrix.Translation(off)
            for p in parts:
                vs=[M@p.matrix_world@v.co for v in p.data.vertices]; tp=BVHTree.FromPolygons(vs,[q.vertices[:] for q in p.data.polygons])
                for nm,ta in against.items():
                    if tp.overlap(ta):
                        dd=depth(vs,ta)
                        if dd>0.1: worst.append((i,round(k,1),p.name,nm,round(dd,2)))
    if worst:
        out.append(f"BLOCKED {st['name']}: deepest {max(w[4] for w in worst)} mm; " + "; ".join(f"seg{w[0]} at {w[1]}: {w[2]} x {w[3]} {w[4]}" for w in sorted(worst,key=lambda w:-w[4])[:5]))
    else: out.append(f"OK      {st['name']}")
print("\n".join(out))
