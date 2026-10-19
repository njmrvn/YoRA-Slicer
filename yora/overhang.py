"""Overhang: how far each wall line sticks out past the layer below, as an angle from vertical.

For every wall move (outer and inner walls, spiral vase), at its middle, in the layer plane:
  outside the outline of the layer below (sticking out into the air): d = distance to the nearest outer wall below;
  inside it: supported (d = 0) when it rests on any bead below (within half a line width of its centre), otherwise
             d = distance to the nearest bead below (e.g. a thin hollow wall leaning inwards).
With the layer height h the overhang angle is atan(d / h): 0 = right on top, 45 deg = out by one layer height,
towards 90 deg = printed in the air. The outline is the even-odd area of the outer wall loops below (holes are
outside). Angled layers are
measured in their own (tilted) layer plane. The distances are exact (point to segment); a grid of cells finds the
nearby segments, so a layer costs about the same whatever the size of the part.
"""
import math

import numpy as np

from .slicer import KIND_INNER, KIND_OUTER, KIND_RETRACT, KIND_TRAVEL

WALLS = (KIND_OUTER, KIND_INNER)


def nearest_distance(Q, A, B, reach):
    """Distance from each point Q (m, 2) to the nearest segment A-B (k, 2), looked at only within `reach`; farther
    points get `reach`."""
    g = 2.0 * reach                                # cell size: pieces <= reach/2 long, so 3 x 3 cells cover the reach
    L = np.hypot(*(B - A).T)
    n = np.maximum(np.ceil(L / (reach / 2)), 1).astype(int)
    seg = np.repeat(np.arange(len(A)), n)
    k = np.arange(len(seg)) - np.repeat(np.cumsum(n) - n, n)
    t0, t1 = (k / n[seg])[:, None], ((k + 1) / n[seg])[:, None]
    PA = A[seg] + t0 * (B[seg] - A[seg])
    PB = A[seg] + t1 * (B[seg] - A[seg])
    lo = np.minimum(PA.min(0), Q.min(0)) - g
    cell = lambda P: np.floor((P - lo) / g).astype(np.int64)   # noqa: E731
    pc = cell((PA + PB) / 2)
    BIG = np.int64(1) << 31
    pkey = pc[:, 0] * BIG + pc[:, 1]
    order = np.argsort(pkey, kind='stable')
    skey = pkey[order]
    qc = cell(Q)
    best = np.full(len(Q), reach)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            qk = (qc[:, 0] + dx) * BIG + (qc[:, 1] + dy)
            s0 = np.searchsorted(skey, qk, 'left')
            cnt = np.searchsorted(skey, qk, 'right') - s0
            tot = int(cnt.sum())
            if not tot:
                continue
            qi = np.repeat(np.arange(len(Q)), cnt)
            pi = order[np.repeat(s0, cnt) + np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt)]
            a, ab = PA[pi], PB[pi] - PA[pi]
            u = np.clip(((Q[qi] - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-18), 0, 1)
            d = np.hypot(*(Q[qi] - a - u[:, None] * ab).T)
            np.minimum.at(best, qi, d)
    return best


def inside(Q, A, B):
    """Even-odd test: is each point Q inside the area bounded by the segments A-B (closed loops)?"""
    out = np.zeros(len(Q), dtype=bool)
    if not len(A):
        return out
    dy = B[:, 1] - A[:, 1]
    dy = np.where(np.abs(dy) < 1e-12, 1e-12, dy)
    step = max(1, 4_000_000 // len(A))
    for i in range(0, len(Q), step):
        q = Q[i:i + step]
        cross = (A[None, :, 1] > q[:, None, 1]) != (B[None, :, 1] > q[:, None, 1])
        xi = A[None, :, 0] + (q[:, None, 1] - A[None, :, 1]) * (B[None, :, 0] - A[None, :, 0]) / dy[None]
        out[i:i + step] = (np.count_nonzero(cross & (xi > q[:, None, 0]), axis=1) % 2) == 1
    return out


def angles(tp, s):
    """Overhang angle (deg) of every move; 0 for moves that are not walls and on the first layer of each block of
    flat or angled layers; 90 where nothing of the layer below is near."""
    n = len(tp)
    out = np.zeros(n, dtype=np.float32)
    if n < 2:
        return out
    x, y = tp.x.astype(np.float64), tp.y.astype(np.float64)
    angled_from = tp.flat_layers if tp.part_R is not None and tp.flat_layers is not None else None
    if angled_from is not None:   # angled layers: measure in the slice frame (p_s = R^T p)
        Ps = np.column_stack([x, y, tp.z.astype(np.float64)]) @ tp.part_R
        sel = tp.layer >= angled_from
        x, y = np.where(sel, Ps[:, 0], x), np.where(sel, Ps[:, 1], y)
    de = np.diff(tp.e, prepend=tp.e[0])
    seg = np.hypot(np.diff(x, prepend=x[0]), np.diff(y, prepend=y[0]))
    extruding = (de > 1e-6) & (seg > 1e-6) & (tp.kind != KIND_TRAVEL) & (tp.kind != KIND_RETRACT)
    extruding[0] = False
    walls = extruding & np.isin(tp.kind, WALLS)
    lz = np.asarray(tp.layer_z, dtype=float)
    w = float(s['line_width'])
    layers = tp.layer
    order = np.argsort(layers, kind='stable')
    bounds = np.searchsorted(layers[order], np.arange(len(lz) + 1))
    for L in range(1, len(lz)):
        if angled_from is not None and L == angled_from:
            continue                      # the first angled layer sits on the flat base: not comparable
        cur = order[bounds[L]:bounds[L + 1]]
        cur = cur[walls[cur]]
        if not len(cur):
            continue
        prev = order[bounds[L - 1]:bounds[L]]
        prev = prev[extruding[prev]]
        if not len(prev):
            out[cur] = 90.0
            continue
        h = max(lz[L] - lz[L - 1], 1e-3)
        reach = 6.0 * h                   # farther than this is ~80 deg: treated as printed in the air
        Q = np.column_stack([(x[cur] + x[cur - 1]) / 2, (y[cur] + y[cur - 1]) / 2])
        seg_ab = lambda i: (np.column_stack([x[i - 1], y[i - 1]]), np.column_stack([x[i], y[i]]))   # noqa: E731
        po = prev[tp.kind[prev] == KIND_OUTER]
        d_any = nearest_distance(Q, *seg_ab(prev), reach)
        if len(po):
            ins = inside(Q, *seg_ab(po))
            d_out = nearest_distance(Q, *seg_ab(po), reach)
        else:
            ins, d_out = np.ones(len(Q), dtype=bool), d_any
        d = np.where(ins, np.where(d_any <= 0.5 * w, 0.0, d_any), d_out)
        a = np.degrees(np.arctan2(d, h))
        a[d >= reach] = 90.0
        out[cur] = a.astype(np.float32)
    return out


def summary(tp, s, ang):
    """Layers with walls steeper than the limit: their count, height range and a warning text."""
    limit = float(s.get('overhang_limit', 45.0))
    w = float(s['line_width'])
    if not len(tp):
        return {'limit': limit, 'max': 0.0, 'layers': []}, None
    seg = np.hypot(np.diff(tp.x, prepend=tp.x[0]), np.diff(tp.y, prepend=tp.y[0]))
    bad = ang > limit
    lengths = np.bincount(tp.layer[bad], weights=seg[bad], minlength=len(tp.layer_z)) if bad.any() else np.zeros(len(tp.layer_z))
    layers = np.nonzero(lengths >= w)[0]          # ignore single specks shorter than a bead
    info = {'limit': limit, 'max': float(ang.max()) if len(ang) else 0.0, 'layers': layers.tolist()}
    if not len(layers):
        return info, None
    z = np.asarray(tp.layer_z, dtype=float)
    zs = np.asarray(tp.z, dtype=float)
    lo = float(min(zs[tp.layer == layers[0]])) if (tp.layer == layers[0]).any() else z[layers[0]]
    hi = float(max(zs[tp.layer == layers[-1]])) if (tp.layer == layers[-1]).any() else z[layers[-1]]
    text = (f'Overhang steeper than {limit:g}° on {len(layers)} layer{"s" if len(layers) > 1 else ""} '
            f'(layers {layers[0] + 1}–{layers[-1] + 1}, Z {lo:.0f}–{hi:.0f} mm, up to {info["max"]:.0f}°). '
            'Colour the preview by Overhang angle to see where; the angled layer strategy or a lower angle helps.')
    return info, text


def annotate(tp, s):
    """Measure the overhang of a sliced toolpath: tp.overhang (deg per move), stats['overhang'] and a warning."""
    tp.overhang = angles(tp, s)
    info, text = summary(tp, s, tp.overhang)
    tp.stats['overhang'] = info
    if text:
        tp.warnings.append(text)
    return tp
