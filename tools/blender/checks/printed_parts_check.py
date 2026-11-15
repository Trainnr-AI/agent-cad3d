"""Every printed part (mesh with mass_g_solid_PLA in the robot collections) is ONE watertight piece. Runs inside Blender. No ARGS."""
import bpy, bmesh
R="duck_scs0009"
def body_of(o):
    while o is not None and not o.name.startswith(R+":"): o=o.parent
    return o
bad=[]; n=0
for c in ("leg_v2","neck_v2","head_v2","head_shell_v2","body_v2"):
    col=bpy.data.collections.get(c)
    if not col: bad.append(f"collection {c} missing"); continue
    for o in col.objects:
        if o.type!="MESH" or "mass_g_solid_PLA" not in o: continue
        n+=1; b=bmesh.new(); b.from_mesh(o.data); nm=sum(not e.is_manifold for e in b.edges)
        seen=set(); sh=0
        for v in b.verts:
            if v.index in seen: continue
            sh+=1; st=[v]
            while st:
                w=st.pop()
                if w.index in seen: continue
                seen.add(w.index); st+=[e.other_vert(w) for e in w.link_edges]
        b.free()
        if nm or sh!=1: bad.append(f"{o.name}: {nm} non-manifold edges, {sh} pieces")
print(f"printed parts checked: {n}; problems: {bad if bad else 'none'}")
