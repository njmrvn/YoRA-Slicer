"""Printing strategy 'curve': every layer lies on a plane perpendicular to a curve through the part (its spine), so a
bent part (an arm, a bent tube) prints along its own bend instead of in flat layers that would overhang.

    curve:   a smooth curve (centripetal Catmull-Rom) through the control points, `curve_points`, or through points
             found from the part (find_curve: the middle of its cross-sections, marching up). It starts straight up
             from the bed so the first layers lie flat on it, and carries straight on past its last point.
    layer k: the plane through C(s_k) with normal T(s_k), the curve's direction there; s_k is the arc length of the
             layer top along the curve, so layer heights are measured along the curve. The in-plane axes U, V turn
             with the curve as little as possible (rotation-minimising frame): seams and infill stay steady.
    slicing: each layer is cut from the part in its own frame, half a layer below its top as the flat slicer does,
             then walls, infill and seams are made by the flat slicer and planner in the 'unrolled' frame (u, v, s).
             Every move is mapped back: p = C_k + R_k (u, v, z - s_k), with R_k = [U V T].
    flow:    where the curve bends a layer is thicker on the outside of the bend than on the inside; the flow
             follows the thickness at each move (its distance to the plane below, along the layer normal).
An optional horizontal base is printed first (the part below Base height, in flat layers); the curve then starts
from there. The nozzle tilts with each layer (or stays vertical), which the robot check, simulation and export follow.
"""
import math

import numpy as np

from .slicer import MAX_PIXELS, Grid, Slicer, clip_loop, section_loops, seg_directions, signed_area
from .planner import EXTRUDING, Planner, toolpath_from_rows


def is_curve(s):
    return s.get('print_strategy', 'horizontal') == 'curve'


def parse_points(text):
    """'x,y,z; x,y,z; ...' -> (n, 3) array in the order given, or None (fewer than two points)."""
    pts = []
    for part in str(text or '').replace('\n', ';').split(';'):
        v = [float(t) for t in part.replace(' ', ',').split(',') if t.strip()]
        if len(v) >= 3:
            pts.append(v[:3])
    return np.array(pts, dtype=float) if len(pts) >= 2 else None


def format_points(P):
    return '; '.join('%.1f,%.1f,%.1f' % tuple(p) for p in P)


# ---------------- geometry helpers ----------------
def frame_from(d, u_ref):
    """3x3 rotation [U V T] with T along d and U as close as possible to u_ref."""
    t = np.asarray(d, dtype=float)
    t = t / max(np.linalg.norm(t), 1e-12)
    u = np.asarray(u_ref, dtype=float) - np.dot(u_ref, t) * t
    if np.linalg.norm(u) < 1e-6:
        u = np.array([1.0, 0, 0]) - t[0] * t
        if np.linalg.norm(u) < 1e-6:
            u = np.array([0, 1.0, 0]) - t[1] * t
    u /= np.linalg.norm(u)
    return np.column_stack([u, np.cross(t, u), t])


def plane_section(T, origin, R):
    """Oriented loops (in the plane's u, v) where the triangles T cut the plane through origin with axes R = [U V N]."""
    Q = (np.asarray(T, dtype=float) - origin) @ R
    z = Q[:, :, 2]
    sel = (z.min(axis=1) <= 0) & (z.max(axis=1) >= 0)
    return section_loops(Q[sel], seg_directions(Q[sel]), 0.0)


def _inside(pt, loop):
    x, y = loop[:, 0], loop[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    cross = (y > pt[1]) != (y2 > pt[1])
    with np.errstate(divide='ignore', invalid='ignore'):
        xi = x + (pt[1] - y) * (x2 - x) / (y2 - y)
    return bool((np.count_nonzero(cross & (xi > pt[0])) % 2) == 1)


def _centroid(loop):
    x, y = loop[:, 0], loop[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    c = x * y2 - x2 * y
    a = c.sum() / 2
    if abs(a) < 1e-12:
        return loop.mean(axis=0), 0.0
    return np.array([((x + x2) * c).sum(), ((y + y2) * c).sum()]) / (6 * a), a


def region_centroid(loops, near=(0.0, 0.0)):
    """Middle (area centroid, holes taken off) of the piece of the cross-section around `near`: the outer outline that
    contains it, or else the closest one. None if there is no outline."""
    if not loops:
        return None
    areas = np.array([signed_area(l) for l in loops])
    if areas.sum() < 0:          # the whole mesh is inside out
        areas = -areas
    outer = [i for i, a in enumerate(areas) if a > 0]
    if not outer:
        return None
    near = np.asarray(near, dtype=float)
    cents = {i: _centroid(loops[i])[0] for i in outer}
    hit = [i for i in outer if _inside(near, loops[i])]
    pick = min(hit, key=lambda i: abs(areas[i])) if hit else min(outer, key=lambda i: np.hypot(*(cents[i] - near)))
    members = [pick] + [j for j, a in enumerate(areas) if a < 0 and _inside(loops[j][0], loops[pick])]
    tot, acc = 0.0, np.zeros(2)
    for j in members:
        c, _ = _centroid(loops[j])
        acc += c * areas[j]
        tot += areas[j]
    return acc / tot if abs(tot) > 1e-12 else cents[pick]


# ---------------- finding the curve ----------------
def find_curve(tris, s, max_points=8):
    """Control points of the part's middle line: start at the middle of its foot on the bed, then march up, cutting the
    part square to the current direction and stepping to the middle of each cut; the direction follows the middles."""
    T = np.asarray(tris, dtype=float)
    v = T.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    size = float(np.linalg.norm(hi - lo))
    h = float(s['layer_height'])
    step = max(2 * h, min(size / 40.0, 10 * h), 1.0)
    z0 = float(lo[2])
    eps = min(max(h / 2, 0.2), max((hi[2] - lo[2]) / 10, 0.05))
    c = region_centroid(plane_section(T, np.array([0.0, 0.0, z0 + eps]), np.eye(3)),
                        near=((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2))
    if c is None:
        return np.array([[(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, z0], [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]]])
    p = np.array([c[0], c[1], z0])
    pts = [p.copy()]
    d = np.array([0.0, 0.0, 1.0])
    u_ref = np.array([1.0, 0.0, 0.0])
    for _ in range(int(4 * size / step) + 10):
        R = frame_from(d, u_ref)
        q = p + d * step
        m = region_centroid(plane_section(T, q, R))
        if m is None:
            break
        c = q + R[:, 0] * m[0] + R[:, 1] * m[1]
        dn = c - p
        if np.linalg.norm(dn) < 1e-6:
            break
        d = d + dn / np.linalg.norm(dn)          # half way to the new direction: follows the bend, ignores wobble
        d /= np.linalg.norm(d)
        if d[2] < 0.05:                          # turning back down: a layer could not be printed on the one below
            break
        pts.append(c)
        p = c
        u_ref = R[:, 0]
    P = np.array(pts)
    if len(P) < 2:
        P = np.vstack([P, P[0] + [0, 0, hi[2] - z0]])
    if len(P) > 3:                               # smooth out the step-to-step wobble (ends stay)
        Q = P.copy()
        Q[1:-1] = (P[:-2] + 2 * P[1:-1] + P[2:]) / 4
        P = Q
    # a few control points, evenly along the curve (first and last kept)
    L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    k = min(max_points, len(P))
    idx = np.unique(np.searchsorted(L, np.linspace(0, L[-1], k)).clip(0, len(P) - 1))
    idx[-1] = len(P) - 1
    return np.round(P[np.unique(idx)], 1)


# ---------------- other sources: an edge of the part, a drawn line, another part ----------------
def simplify_points(P, max_points=16):
    """Fewest points (first and last kept) that stay close to the polyline P (Douglas-Peucker in 3D); the tolerance
    grows until there are at most max_points."""
    P = np.asarray(P, dtype=float)
    if len(P) <= 2:
        return P
    size = float(np.linalg.norm(P.max(axis=0) - P.min(axis=0)))
    tol = max(0.1, size * 0.002)

    def dp(a, b, keep):
        if b <= a + 1:
            return
        A, B = P[a], P[b]
        AB = B - A
        L2 = float(AB @ AB)
        Q = P[a + 1:b] - A
        t = np.clip(Q @ AB / L2, 0, 1) if L2 > 1e-12 else np.zeros(len(Q))
        d = np.linalg.norm(Q - t[:, None] * AB, axis=1)
        i = int(np.argmax(d))
        if d[i] > tol:
            keep.add(a + 1 + i)
            dp(a, a + 1 + i, keep)
            dp(a + 1 + i, b, keep)

    while True:
        keep = {0, len(P) - 1}
        dp(0, len(P) - 1, keep)
        if len(keep) <= max_points:
            return P[sorted(keep)]
        tol *= 1.5


def bottom_up(P):
    """The polyline running from its lower end to its higher end."""
    P = np.asarray(P, dtype=float)
    return P[::-1] if len(P) > 1 and P[-1, 2] < P[0, 2] else P


def sharp_edges(T, cad=None, angle=30.0):
    """(vertices, edges): the welded vertices of the triangles T and the edges (k, 2) that are sharper than `angle`
    degrees, open, or (for a STEP part, `cad` = CAD face of each triangle) between two CAD faces."""
    T = np.asarray(T, dtype=float)
    V = T.reshape(-1, 3)
    _, first, inv = np.unique(np.round(V / 1e-4).astype(np.int64), axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    Vu = V[first]
    F = inv.reshape(-1, 3)
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    tri_of = np.concatenate([np.arange(len(F))] * 3)
    ue, eid, counts = np.unique(E, axis=0, return_inverse=True, return_counts=True)
    eid = eid.reshape(-1)
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    order = np.argsort(eid, kind='stable')
    starts = np.concatenate([[0], np.cumsum(counts)])
    sharp = counts != 2                                          # open borders and non-manifold edges count as edges
    two = np.nonzero(counts == 2)[0]
    ta, tb = tri_of[order[starts[two]]], tri_of[order[starts[two] + 1]]
    s2 = (nrm[ta] * nrm[tb]).sum(1) < math.cos(math.radians(angle))
    if cad is not None:
        cad = np.asarray(cad)
        s2 |= cad[ta] != cad[tb]
    sharp[two] = s2
    return Vu, ue[sharp]


def edge_chain(T, point, cad=None, angle=30.0, turn=60.0, reach=None):
    """The run of sharp edges of the triangles T (and borders between the CAD faces `cad`, for a STEP part) through the
    edge closest to `point`, as an ordered polyline (bottom to top). It goes on in both directions while it can carry
    on straight enough (turns under `turn` degrees) and stops at a corner. None if no sharp edge is within `reach` mm
    of the point (default: 5 % of the part's size, at least 5 mm)."""
    T = np.asarray(T, dtype=float)
    V = T.reshape(-1, 3)
    Vu, se = sharp_edges(T, cad, angle)
    if not len(se):
        return None
    A, B = Vu[se[:, 0]], Vu[se[:, 1]]
    p = np.asarray(point, dtype=float)
    AB = B - A
    t = np.clip(((p - A) * AB).sum(1) / np.maximum((AB ** 2).sum(1), 1e-12), 0, 1)
    dist = np.linalg.norm(A + t[:, None] * AB - p, axis=1)
    k = int(np.argmin(dist))
    if reach is None:
        reach = max(5.0, 0.05 * float(np.linalg.norm(V.max(axis=0) - V.min(axis=0))))
    if dist[k] > reach:
        return None
    nbr = {}
    for a, b in se:
        nbr.setdefault(int(a), []).append(int(b))
        nbr.setdefault(int(b), []).append(int(a))
    lim = math.cos(math.radians(turn))

    def walk(prev, cur, used):
        out = []
        while True:
            d = Vu[cur] - Vu[prev]
            d /= max(np.linalg.norm(d), 1e-12)
            best, bc = None, lim
            for w in nbr.get(cur, []):
                if w == prev or w in used:
                    continue
                e = Vu[w] - Vu[cur]
                c = float(e @ d) / max(np.linalg.norm(e), 1e-12)
                if c > bc:
                    best, bc = w, c
            if best is None:
                return out
            used.add(best)
            out.append(best)
            prev, cur = cur, best

    a, b = int(se[k, 0]), int(se[k, 1])
    used = {a, b}
    fwd = walk(a, b, used)
    back = walk(b, a, used)
    chain = back[::-1] + [a, b] + fwd
    return bottom_up(Vu[chain])


# ---------------- the curve and its frames ----------------
def catmull_rom(P, per_segment=24):
    """Centripetal Catmull-Rom curve through the points (n, 3)."""
    P = np.asarray(P, dtype=float)
    if len(P) == 2:
        return P[0] + (P[1] - P[0]) * np.linspace(0, 1, per_segment + 1)[:, None]
    ext = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = [P[:1]]
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        t0 = 0.0
        t1 = t0 + max(np.linalg.norm(p1 - p0), 1e-9) ** 0.5
        t2 = t1 + max(np.linalg.norm(p2 - p1), 1e-9) ** 0.5
        t3 = t2 + max(np.linalg.norm(p3 - p2), 1e-9) ** 0.5
        t = np.linspace(t1, t2, per_segment + 1)[1:, None]
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.vstack(out)


class Spine:
    """The curve sampled every `ds` mm by arc length, with a rotation-minimising frame at each sample."""

    def __init__(self, P, zbed=0.0, extend=0.0, ds=0.5):
        P = np.asarray(P, dtype=float)
        if P[0, 2] > zbed + 1e-6:                        # start on the bed, straight below the first point
            P = np.vstack([[P[0, 0], P[0, 1], zbed], P])
        self.control = P
        dense = catmull_rom(P)
        seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
        L = np.concatenate([[0], np.cumsum(seg)])
        self.length = float(L[-1])                       # to the last control point
        n = max(int(math.ceil((self.length + extend) / ds)) + 1, 2)
        s = np.linspace(0, n - 1, n) * ds
        S = np.column_stack([np.interp(np.minimum(s, L[-1]), L, dense[:, k]) for k in range(3)])
        end_dir = dense[-1] - dense[-min(len(dense), 4)]
        end_dir /= max(np.linalg.norm(end_dir), 1e-12)
        past = s > L[-1]
        S[past] = dense[-1] + (s[past] - L[-1])[:, None] * end_dir           # straight on past the end
        # tangents, smoothed over ~3 mm so the planes turn evenly
        Tn = np.gradient(S, axis=0)
        k = max(1, int(round(1.5 / ds)))
        if len(Tn) > 2 * k + 1:
            ker = np.ones(2 * k + 1) / (2 * k + 1)
            Tn = np.column_stack([np.convolve(np.pad(Tn[:, j], k, mode='edge'), ker, mode='valid') for j in range(3)])
        Tn /= np.maximum(np.linalg.norm(Tn, axis=1, keepdims=True), 1e-12)
        U = np.zeros_like(Tn)
        U[0] = frame_from(Tn[0], [1.0, 0, 0])[:, 0]
        for i in range(1, len(Tn)):                      # parallel transport: the least turn about the curve
            u = U[i - 1] - np.dot(U[i - 1], Tn[i]) * Tn[i]
            nu = np.linalg.norm(u)
            U[i] = u / nu if nu > 1e-9 else frame_from(Tn[i], U[i - 1])[:, 0]
        self.s, self.S, self.T, self.U, self.ds = s, S, Tn, U, ds
        self.start_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Tn[0, 2])))))

    def frame(self, sv):
        """(point C, rotation R = [U V T]) at arc length sv (clamped to the sampled range)."""
        x = min(max(sv / self.ds, 0.0), len(self.s) - 1.0)
        i = min(int(x), len(self.s) - 2)
        f = x - i
        C = self.S[i] * (1 - f) + self.S[i + 1] * f
        t = self.T[i] * (1 - f) + self.T[i + 1] * f
        u = self.U[i] * (1 - f) + self.U[i + 1] * f
        return C, frame_from(t, u)

    def nearest(self, X, chunk=4000):
        """For points X (N, 3): arc length of the closest point of the curve, and the distance to it."""
        X = np.asarray(X, dtype=float)
        step = max(1, int(round(2.0 / self.ds)))                   # segments of ~2 mm
        A, B = self.S[:-step:step], self.S[step::step]
        sa = self.s[:-step:step]
        AB = B - A
        L2 = np.maximum((AB ** 2).sum(1), 1e-12)
        par, dist = np.zeros(len(X)), np.zeros(len(X))
        for c in range(0, len(X), chunk):
            x = X[c:c + chunk]
            t = np.clip(np.einsum('nmk,mk->nm', x[:, None, :] - A[None], AB) / L2, 0, 1)
            d = np.linalg.norm(x[:, None, :] - (A[None] + t[..., None] * AB[None]), axis=2)
            j = np.argmin(d, axis=1)
            r = np.arange(len(x))
            par[c:c + chunk] = sa[j] + t[r, j] * np.sqrt(L2[j])
            dist[c:c + chunk] = d[r, j]
        return par, dist


# ---------------- slicing ----------------
class CurveSlicer(Slicer):
    """The flat slicer, with every layer cut from the part on its own plane (see the module doc)."""

    def __init__(self, tris, s, spine, layers, base=None, progress=None):
        self.s = s
        self.clip = None
        self.progress = progress or (lambda stage, frac: None)
        self.warnings = []
        self._masks, self._outlines, self._index = {}, {}, {}
        self.T = np.asarray(tris, dtype=np.float64)
        self.spine, self.frames, self.base = spine, layers, base
        self.layers_z = [(st, h) for st, h, _, _ in layers]
        par, dist = spine.nearest(self.T.reshape(-1, 3))
        par = par.reshape(-1, 3)
        self.tri_lo, self.tri_hi = par.min(axis=1), par.max(axis=1)
        w = s['line_width']
        reach = float(dist.max()) if len(dist) else 1.0
        margin = (s['brim_loops'] + 3) * w * max(1.0, s['first_layer_width_pct'] / 100.0) + 2.0
        res = s['resolution'] if s['resolution'] > 0 else min(max(w / 10.0, 0.05), 1.0)
        span = (2 * reach + 2 * margin) ** 2
        if span / res ** 2 > MAX_PIXELS:
            res = math.sqrt(span / MAX_PIXELS)
            self.warnings.append('Slicing resolution raised to %.2f mm to fit memory.' % res)
        self.grid = Grid(-reach - margin, -reach - margin, reach + margin, reach + margin, res)
        self.zmodel = float(self.tri_hi.max()) if len(self.tri_hi) else 0.0

    def layer_loops(self, i):
        st, h, C, R = self.frames[i]
        M = C - R[:, 2] * (h / 2)                                     # half a layer below the top, as the flat slicer
        W = 2 * h + 2.0
        sel = (self.tri_lo <= st + W) & (self.tri_hi >= st - h - W)   # only the part of the model near this layer
        loops = plane_section(self.T[sel], M, R)
        if self.base is not None:                                     # nothing below the flat base
            loops = [q for q in (clip_loop(l, R[2, 0], R[2, 1], self.base - M[2]) for l in loops) if q is not None]
        return loops

    def mask(self, i):
        if i < 0 or i >= len(self.layers_z):
            return None
        if i not in self._masks:
            loops = self.layer_loops(i)
            self._masks[i] = self.rasterize(loops)
            self._outlines[i] = loops
        return self._masks[i]


def curve_settings(s, nb, warnings):
    """Settings for the curved layers: like the angled ones (no spiral, raft or brim on top of a base); seams placed by
    a point or a drawn line can't be followed on curved layers and become aligned."""
    s2 = dict(s)
    s2['spiral_vase'] = False
    if nb:
        s2['brim_loops'] = 0
        s2['first_layer_height'] = s['layer_height']
        s2['first_layer_width_pct'] = 100.0
        s2['first_layer_speed'] = s['outer_wall_speed']
    if s.get('spiral_vase'):
        warnings.append('Spiral vase is not used with curved layers.')
    if s.get('raft_type', 'none') != 'none':
        warnings.append('Raft is not used with curved layers yet: use a horizontal base (Base height) instead.')
    return s2


def seam_targets(s, frames):
    """Where each curved layer's seam goes, in the layer's own (u, v), for a picked seam point or a drawn seam line;
    None for the other seam modes.
      point: the picked point stands for the vertical line through it (as on flat layers). It is placed on the layer
             where that line meets the part's surface nearest the curve, and every layer keeps the same side of the
             curve there, so the seam runs along the bent part.
      line:  each layer starts where the drawn line crosses its plane (the crossing nearest the curve), or at the
             line's point closest to the plane where it doesn't cross."""
    from .planner import parse_seam_path
    mode = s['seam']
    if mode not in ('custom', 'path') or not frames:
        return None
    P = parse_points(s.get('seam_path', '')) if mode == 'path' else None
    if mode == 'path' and P is None:
        one = parse_seam_path(s.get('seam_path', ''))
        if one is None:
            return None
        mode, sx, sy = 'custom', float(one[0, 0]), float(one[0, 1])
    elif mode == 'custom':
        sx, sy = float(s['seam_x']), float(s['seam_y'])
    out = {}
    if mode == 'custom':
        best = None
        for k, (_, _, C, R) in enumerate(frames):
            n = R[:, 2]
            if abs(n[2]) < 0.05:
                continue
            p = np.array([sx, sy, (n @ C - n[0] * sx - n[1] * sy) / n[2]])
            q = (p - C) @ R
            if best is None or np.hypot(q[0], q[1]) < best[0]:
                best = (float(np.hypot(q[0], q[1])), q[:2])
        if best is None:
            return None
        return {k: best[1].copy() for k in range(len(frames))}
    for k, (_, _, C, R) in enumerate(frames):
        n = R[:, 2]
        d = (P - C) @ n
        cand = []
        for i in np.nonzero(np.sign(d[:-1]) != np.sign(d[1:]))[0]:
            t = d[i] / (d[i] - d[i + 1])
            cand.append(P[i] + t * (P[i + 1] - P[i]))
        if cand:
            x = min(cand, key=lambda c: np.linalg.norm(c - C))
        else:
            x = P[int(np.argmin(np.abs(d)))]
        out[k] = ((x - C) @ R)[:2]
    return out


def build_spine(tris, s):
    """(Spine, control points, found automatically?) for the part."""
    tris = np.asarray(tris, dtype=np.float64)
    P = parse_points(s.get('curve_points', ''))
    auto = P is None
    if auto:
        P = find_curve(tris, s)
    v = tris.reshape(-1, 3)
    zbed = float(v[:, 2].min())
    size = float(np.linalg.norm(v.max(axis=0) - v.min(axis=0)))
    return Spine(P, zbed, extend=size), P, auto


def plan_layers_frames(sp, tris, s, base, nb):
    """The curved layers: (top arc length, height, C, R) from the start (bed, or the top of the base) to the top."""
    h = float(s['layer_height'])
    par, _ = sp.nearest(np.asarray(tris, dtype=float).reshape(-1, 3))
    top = float(par.max())
    s0 = 0.0
    if nb:
        above = np.nonzero(sp.S[:, 2] >= base)[0]
        s0 = float(sp.s[above[0]]) if len(above) else top
    hh = h if nb else float(s['first_layer_height'])
    st = s0 + hh
    out = []
    while st - hh / 2 < top - 1e-6:
        C, R = sp.frame(st)
        out.append((st, hh, C, R))
        hh = h
        st += h
    return out


def slice_curve(tris, s, progress=None):
    """Toolpath for the curve strategy (optional horizontal base first)."""
    progress = progress or (lambda stage, frac: None)
    tris = np.asarray(tris, dtype=np.float64)
    warnings, rows, layer_z = [], [], []
    sp, P, auto = build_spine(tris, s)
    base = max(float(s.get('base_height', 0.0)), 0.0)
    planner = Planner(s)
    nb = 0
    if base > 0:
        sl = Slicer(tris, s, lambda st, f: progress(st, 0.3 * f), z_limit=base)
        res = sl.run()
        warnings += res.warnings
        nb = len(res.layers)
        if nb:
            rows += planner.plan(res.layers)
            layer_z += [l.z for l in res.layers]
    if sp.start_tilt > 10 and not nb:
        warnings.append('The curve leaves the bed at %.0f° from vertical, so the first layers are not flat on the bed. '
                        'Start it straight up, or print a horizontal base first (Base height).' % sp.start_tilt)
    s2 = curve_settings(s, nb, warnings)
    frames = plan_layers_frames(sp, tris, s2, base, nb)
    normals = [[0.0, 0.0, 1.0]] * nb
    thick, rows2, uv, t_rows = None, [], None, None
    by = s.get('curve_thickness_by', 'flow')
    if frames:
        cs = CurveSlicer(tris, s2, sp, frames, base if nb else None,
                         lambda st, f: progress(st, 0.3 * bool(nb) + (1 - 0.3 * bool(nb)) * f))
        res2 = cs.run()
        warnings += [w for w in res2.warnings if w not in warnings]
        p2 = Planner(s2)
        targets = seam_targets(s, frames)
        if targets is not None:          # the planner gets each layer's seam point in its own frame
            p2.layer_seams = targets
            p2.s = dict(s2, seam='custom')
        p2.e = planner.e
        st0, _, C0, R0 = frames[0]
        if planner.pos is not None:
            q = (np.asarray(planner.pos, dtype=float) - C0) @ R0
            p2.pos = (float(q[0]), float(q[1]), float(st0 + q[2]))
        start = len(p2.rows)
        rows2 = p2.plan(res2.layers)[start:]
        if rows2:
            a = np.array(rows2, dtype=np.float64)
            uv = a[:, :2].copy()
            a, thick, t_rows = _map_back(a, frames, planner.e, base if nb else None, by)
            a[:, 6] += nb
            rows += [tuple(r) for r in a]
        layer_z += [st for st, _, _, _ in frames]
        normals += [R[:, 2].round(5).tolist() for _, _, _, R in frames]
    else:
        warnings.append('Nothing to print with curved layers above the base.')
    if thick is not None:
        h = float(s['layer_height'])
        lo, hi = thick
        if lo <= 0.05 * h:
            warnings.append('The curve bends too tightly for the width of the part: on the inside of the bend the layers '
                            'would cross. Bend it less (move or remove curve points) or use a thinner part.')
        elif lo < 0.5 * h or hi > 1.5 * h:
            warnings.append('Where the curve bends, layers range from %.2f to %.2f mm thick (layer height %.2f mm); the '
                            '%s follows the thickness. A gentler bend keeps them more even.' % (lo, hi, h, by))
    if s.get('nozzle_orientation', 'normal') == 'normal':
        warnings.append('Curved layers with a tilted nozzle: the Borunte and KUKA programs carry the tilt; G-code '
                        'carries positions only (vertical nozzle).')
    warnings.append('Robot check on curved layers: collisions with the printed part are not checked yet (the bed and '
                    'reach are).')
    tp = toolpath_from_rows(rows, layer_z, s, warnings)
    tp.flat_layers = nb
    tp.part_R = None
    n = len(tp)
    tp.uv = np.column_stack([tp.x, tp.y]).astype(np.float64)      # moves in their layer's own frame (overhang)
    if uv is not None and len(uv) <= n:
        tp.uv[n - len(uv):] = uv
    if t_rows is not None and len(t_rows) <= n:             # bead thickness of every move (preview, colours)
        lz = np.asarray(layer_z, dtype=float)
        lh = np.diff(np.concatenate([[0.0], lz])) if len(lz) else np.ones(1)
        tp.thick = lh[np.clip(tp.layer, 0, len(lh) - 1)].astype(np.float64)
        tp.thick[n - len(t_rows):] = t_rows
    if s.get('nozzle_orientation', 'normal') == 'normal' and frames:
        tilts = np.zeros((len(frames), 4, 4))
        tilts[:, 3, 3] = 1.0
        tilts[:, :3, :3] = np.stack([R for _, _, _, R in frames])
        tp.tilts = tilts
        tp.orient = np.where(tp.layer >= nb, tp.layer - nb + 1, 0).astype(np.int32)
    else:
        tp.orient = np.zeros(n, np.int32)
    tp.stats['strategy'] = {'curve': True, 'flat_layers': nb, 'nozzle': s.get('nozzle_orientation', 'normal'),
                            'normals': normals, 'curve_points': np.round(P, 2).tolist(), 'auto': auto,
                            'thickness': [round(thick[0], 3), round(thick[1], 3)] if thick is not None else None,
                            'thickness_by': by}
    return tp


def _map_back(a, frames, e0, base, by='flow'):
    """Rows in the unrolled frame (u, v, s) -> bed frame. The material per mm follows the local layer thickness; by
    'flow' the speed stays (more or less material per second), by 'speed' the material per second stays and the speed
    changes instead (slower where thicker). Returns (rows, (thinnest, thickest), thickness of every row)."""
    k = a[:, 6].astype(np.int64).clip(0, len(frames) - 1)
    st = np.array([f[0] for f in frames])
    hs = np.array([f[1] for f in frames])
    Cs = np.stack([f[2] for f in frames])
    Rs = np.stack([f[3] for f in frames])
    local = np.column_stack([a[:, 0], a[:, 1], a[:, 2] - st[k]])
    xyz = Cs[k] + np.einsum('nij,nj->ni', Rs[k], local)
    # the plane below each layer: the previous layer's, or the base top / the bed for the first one
    Cp = np.vstack([[[0.0, 0.0, base if base is not None else Cs[0][2] - hs[0]]], Cs[:-1]])
    Np = np.vstack([[[0.0, 0.0, 1.0]], Rs[:-1, :, 2]])
    if base is None:
        Np[0] = Rs[0, :, 2]                          # the first layer lies on the bed: as thick as it is high
        Cp[0] = Cs[0] - Rs[0, :, 2] * hs[0]
    n_k = Rs[k, :, 2]
    mid = (xyz + np.vstack([xyz[:1], xyz[:-1]])) / 2
    with np.errstate(divide='ignore', invalid='ignore'):
        t = ((mid - Cp[k]) * Np[k]).sum(1) / np.maximum((Np[k] * n_k).sum(1), 1e-6)
    de = np.diff(np.concatenate([[e0], a[:, 3]]))
    ext = np.isin(a[:, 5].astype(np.int64), EXTRUDING) & (de > 0)
    factor = np.clip(t / hs[k], 0.2, 3.0)
    de = np.where(ext, de * factor, de)
    out = a.copy()
    out[:, :3] = xyz
    out[:, 3] = e0 + np.cumsum(de)
    if by == 'speed':
        out[:, 4] = np.where(ext, a[:, 4] / factor, a[:, 4])
    thick = (float(t[ext].min()), float(t[ext].max())) if ext.any() else None
    return out, thick, np.where(ext, t, hs[k])


def from_part(tris, s):
    """Control points of another part's middle line (a guide part: a bent rod or tube modelled where the print goes)."""
    return find_curve(tris, s, max_points=12)


def preview(tris, s, n_planes=14):
    """The curve and a few of its layer planes, for drawing on the part in Prepare."""
    sp, P, auto = build_spine(tris, s)
    v = np.asarray(tris, dtype=float).reshape(-1, 3)
    par, dist = sp.nearest(v)
    top = float(par.max()) if len(par) else sp.length
    reach = float(dist.max()) if len(dist) else 10.0
    step = max(1, int(round(4.0 / sp.ds)))
    keep = sp.s <= top + 1e-6
    line = sp.S[keep][::step]
    planes = []
    for sv in np.linspace(0, top, n_planes + 2)[1:-1]:
        C, R = sp.frame(sv)
        planes.append({'c': C.round(3).tolist(), 'n': R[:, 2].round(5).tolist(), 'u': R[:, 0].round(5).tolist()})
    return {'points': np.round(P, 2).tolist(), 'auto': auto, 'line': line.round(2).tolist(), 'planes': planes,
            'reach': round(reach, 2), 'start_tilt': round(sp.start_tilt, 1)}
