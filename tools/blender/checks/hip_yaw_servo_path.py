"""The left hip-yaw servo can only go in by an L-shaped path (lifted over its pads, then slid in from the rear or a side):
reports for each lift height and slide direction how far it slides clear. Runs inside Blender. No ARGS."""
import bpy, bmesh
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
out=[]
def wt(o):
    bm=bmesh.new(); bm.from_mesh(o.data); bm.transform(o.matrix_world); t=BVHTree.FromBMesh(bm); bm.free(); return t
obst=[o for o in bpy.data.objects if o.type=='MESH' and o.visible_get() and ".x" not in o.name and not o.name.startswith(("chip_","scs_left_hip_yaw","wire_","wm_","left_hip_yaw_","body_lid","uno","imu","electronics","battery","servo_adapter","ubec","power_switch","enclosure","cable_stub","head","cam","neck","scs_neck","scs_head","yaw_block","jaw","dome","mouth","scs_mouth","left_","right_","scs_right","scs_left_hip_roll","scs_left_hip_pitch","scs_left_knee","scs_left_ankle")) and not any("explode" in c.name for c in o.users_collection)]
case=bpy.data.objects["scs_left_hip_yaw_case"]
cb=[case.matrix_world@Vector(c) for c in case.bound_box]
lo=Vector([min(v[i] for v in cb) for i in range(3)])-Vector((0.06,0.06,0.03)); hi=Vector([max(v[i] for v in cb) for i in range(3)])+Vector((0.06,0.06,0.03))
near=[]
for o in obst:
    ob=[o.matrix_world@Vector(c) for c in o.bound_box]
    if all(min(v[i] for v in ob)<hi[i] and max(v[i] for v in ob)>lo[i] for i in range(3)): near.append((o.name,wt(o)))
def hits(M):
    vs=[M@v.co for v in case.data.vertices]; tc=BVHTree.FromPolygons(vs,[p.vertices[:] for p in case.data.polygons])
    r=[]
    for n,t in near:
        pr=tc.overlap(t)
        if not pr: continue
        # depth: max distance of case vertices inside the obstacle (ray parity) - cheap volume proxy
        dep=0.0
        for v in vs[::3]:
            loc,nor,i,dist=t.find_nearest(v)
            if loc is None: continue
            d=Vector((0.137,0.311,0.94)).normalized(); c=0; q=v.copy()
            for _ in range(60):
                hh=t.ray_cast(q,d)
                if hh[0] is None: break
                c+=1; q=hh[0]+d*1e-7
            if c%2==1: dep=max(dep,dist*1000)
        if dep>0.05: r.append(f"{n} ({dep:.2f} mm deep)")
    return r
for h in (5.5,6.0,6.5,7.0,7.5,8.0,8.5):
    for d,name in ((Vector((1,0,0)),"+x front"),(Vector((-1,0,0)),"-x rear"),(Vector((0,1,0)),"+y left/out"),(Vector((0,-1,0)),"-y in")):
        first=None
        for k in range(0,41,2):
            M=Matrix.Translation((Vector((0,0,h))+d*k)*0.001)@case.matrix_world
            hs=hits(M)
            if hs: first=(k,hs); break
        out.append(f"lift {h:4.1f} mm, slide {name:12}: " + ("clear for 40 mm" if first is None else f"hits at {first[0]} mm: {first[1]}"))
print("\n".join(out))
