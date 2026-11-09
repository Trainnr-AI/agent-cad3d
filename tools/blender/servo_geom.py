"""Shared collision geometry for servo layout tools (exec'd by other scripts; needs bpy/mathutils).

Shapes, all in world space:
  ("box", centre, [u0, u1, u2], half_extents)      oriented box
  ("cyl", centre, axis, radius, half_height)       cylinder (a servo horn's swept disc)
overlap(A, B) -> penetration depth estimate (m) or None. Box-box is exact (separating axes);
anything with a cylinder is tested by dense surface sampling, accurate to ~0.1 mm here.

SCS0009 shapes in its own frame (origin on the shaft at the case top, +Z out of the shaft,
+X toward the far end): body, ears, and the X-horn swept disc.
"""
import math

from mathutils import Vector

MM = 0.001
SCS_BODY = (Vector((-5.95, -6.05, -25.25)) * MM, Vector((17.35, 6.05, 0.0)) * MM)
SCS_EARS = (Vector((-10.55, -5.5, -8.45)) * MM, Vector((21.95, 5.5, -6.85)) * MM)
SCS_HORN_Z = (0.3 * MM, 4.3 * MM)


def scs_shapes(W, horn_r_mm=12.3, clearance_mm=0.0):
    """World shapes of an SCS0009 whose frame is the 4x4 matrix W."""
    c = clearance_mm * MM / 2
    R = W.to_3x3().normalized()
    axes = [R.col[0].copy(), R.col[1].copy(), R.col[2].copy()]
    out = []
    for tag, (lo, hi) in (("body", SCS_BODY), ("ears", SCS_EARS)):
        out.append((tag, ("box", W @ ((lo + hi) / 2), axes, (hi - lo) / 2 + Vector((c, c, c)))))
    z0, z1 = SCS_HORN_Z
    out.append(("horn", ("cyl", W @ Vector((0, 0, (z0 + z1) / 2)), axes[2], horn_r_mm * MM + c, (z1 - z0) / 2 + c)))
    return out


def _box_box(A, B):
    _, ca, ua, ha = A
    _, cb, ub, hb = B
    d = cb - ca
    best = None
    for ax in ua + ub + [x.cross(y) for x in ua for y in ub]:
        if ax.length < 1e-9:
            continue
        ax = ax.normalized()
        ov = sum(abs(ax.dot(u)) * h for u, h in zip(ua, ha)) + sum(abs(ax.dot(u)) * h for u, h in zip(ub, hb)) - abs(d.dot(ax))
        if ov <= 1e-6:
            return None
        best = ov if best is None else min(best, ov)
    return best


def _in_box(p, B):
    _, c, u, h = B
    d = p - c
    return min(h[i] - abs(d.dot(u[i])) for i in range(3))      # >0 inside (depth to nearest face)


def _in_cyl(p, C):
    _, c, a, r, hh = C
    d = p - c
    z = d.dot(a)
    rad = (d - a * z).length
    return min(r - rad, hh - abs(z))


def _cyl_points(C, n=48, rings=3):
    _, c, a, r, hh = C
    ref = Vector((1, 0, 0)) if abs(a.x) < 0.9 else Vector((0, 1, 0))
    u = a.cross(ref).normalized()
    v = a.cross(u)
    pts = [c + a * hh, c - a * hh]
    for zf in (-1, 0, 1):
        for k in range(n):
            t = 2 * math.pi * k / n
            for rf in [i / rings for i in range(1, rings + 1)]:
                pts.append(c + a * (zf * hh) + (u * math.cos(t) + v * math.sin(t)) * r * rf)
    return pts


def _box_points(B, n=6):
    _, c, u, h = B
    pts = []
    rng = [i / n * 2 - 1 for i in range(n + 1)]
    for i, j, k in ((0, 1, 2), (1, 2, 0), (2, 0, 1)):        # sample all six faces
        for s in (-1, 1):
            for a in rng:
                for b in rng:
                    pts.append(c + u[i] * (s * h[i]) + u[j] * (a * h[j]) + u[k] * (b * h[k]))
    return pts


def _bound(S):
    return S[3].length if S[0] == "box" else math.hypot(S[3], S[4])


def overlap(A, B):
    if (A[1] - B[1]).length > _bound(A) + _bound(B):        # bounding spheres apart: cheap reject
        return None
    if A[0] == "box" and B[0] == "box":
        return _box_box(A, B)
    # anything involving a cylinder: sample each surface inside the other
    depth = 0.0
    for P, Q in ((A, B), (B, A)):
        pts = _cyl_points(P) if P[0] == "cyl" else _box_points(P)
        inside = _in_cyl if Q[0] == "cyl" else _in_box
        for p in pts:
            d = inside(p, Q)
            if d > 1e-6:
                depth = max(depth, d)
    return depth or None
