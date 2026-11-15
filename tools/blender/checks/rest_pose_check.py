"""Rest pose: every robot joint frame is at its stored rest rotation (custom prop q0). Runs inside Blender. No ARGS."""
import bpy
R="duck_scs0009"; bad=[]
for e in [e for e in bpy.data.collections[R].objects if e.type=="EMPTY" and "joint" in e and e.name.startswith(R+":")]:
    if "q0" not in e: bad.append((e["joint"],"no q0 prop")); continue
    q=e.rotation_quaternion; s=list(e["q0"]); d=min(sum((a-b)**2 for a,b in zip(q,s)), sum((a+b)**2 for a,b in zip(q,s)))**0.5
    if d>1e-4: bad.append((e["joint"], round(d,5)))
print("joints differing from their stored rest q0:", bad if bad else "none")
