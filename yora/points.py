"""Point control: how many points a path has and where they sit (settings group 'Point control').

    simplify  fewest points: the slicer's simplification only (Douglas-Peucker to the curve tolerance). Spacing
              follows the shape, so it changes from place to place and from layer to layer.
    uniform   points at an even spacing along the path. Closed loops start at the seam, so with an aligned seam the
              points of consecutive layers line up. Corners sharper than the corner angle are always kept.
    adaptive  as few points as the curve tolerance allows, but never further apart than the max spacing and never
              closer than the min spacing: long even steps on straight parts, short even steps in curves.

All modes then: optional smoothing (corners pinned), points closer than the min spacing merged, segments longer than
the max spacing split. Paths are (n, 2) or (n, 3) arrays (x, y[, z]); spacing is measured in XY, Z is interpolated.
"""
import math

import numpy as np

MODES = ('simplify', 'uniform', 'adaptive')


def active(s, kind_is_wall):
    """Does point control change paths of this kind?"""
    if s.get('point_apply', 'walls') == 'walls' and not kind_is_wall:
        return False
    return (s.get('point_mode', 'simplify') != 'simplify' or s.get('point_smooth', 0) > 0
            or s.get('point_min_spacing', 0) > 0 or s.get('point_max_spacing', 0) > 0)


def source_tolerance(s):
    """Tolerance for the slicer's own simplification: finer when points are placed afterwards, so they land on the
    true curve and not on the chords of a coarse polygon."""
    tol = max(s['simplify'], 0.01)
    return tol if s.get('point_mode', 'simplify') == 'simplify' else max(min(tol / 4, 0.05), 0.01)


def control(pts, s, closed):
    """Apply the point control settings to one path. Closed paths have the first point repeated at the end and keep
    their start (the seam); open paths keep both ends."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        return pts
    mode = s.get('point_mode', 'simplify')
    tol = max(s['simplify'], 0.005)
    lo = max(s.get('point_min_spacing', 0.0), 0.0)
    hi = max(s.get('point_max_spacing', 0.0), 0.0)
    corner = math.radians(s.get('point_corner_angle', 30.0)) if s.get('point_keep_corners', True) else math.inf
    passes = int(s.get('point_smooth', 0))
    pieces = []
    for sec in split_at_corners(pts, corner, closed):
        if passes > 0:
            sec = smooth(sec, passes)
        if mode == 'uniform':
            sec = resample(sec, max(s.get('point_spacing', 2.0), 0.05), fit=s.get('point_fit', True))
        elif mode == 'adaptive':
            sec = adaptive(sec, tol, max(lo, 0.05), hi if hi > 0 else math.inf)
        pieces.append(sec)
    out = np.vstack([pieces[0]] + [p[1:] for p in pieces[1:]])
    if lo > 0:
        out = merge_close(out, lo, keep=corner_mask(out, corner, closed))
    if hi > 0 and mode != 'uniform':
        out = split_long(out, hi)
    return out


# ---------------------------------------------------------------- building blocks
def _xy_len(p):
    return np.hypot(*np.diff(p[:, :2], axis=0).T)


def turn_angles(pts, closed):
    """Direction change at each vertex (rad); 0 at the ends of an open path."""
    p = pts[:, :2]
    a = np.zeros(len(p))
    if len(p) < 3:
        return a
    if closed:   # first point == last point
        prev = p[:-1] - np.roll(p[:-1], 1, axis=0)
        nxt = np.roll(p[:-1], -1, axis=0) - p[:-1]
        t = np.abs((np.arctan2(nxt[:, 1], nxt[:, 0]) - np.arctan2(prev[:, 1], prev[:, 0]) + np.pi) % (2 * np.pi) - np.pi)
        a[:-1] = t
        a[-1] = t[0]
    else:
        prev = p[1:-1] - p[:-2]
        nxt = p[2:] - p[1:-1]
        a[1:-1] = np.abs((np.arctan2(nxt[:, 1], nxt[:, 0]) - np.arctan2(prev[:, 1], prev[:, 0]) + np.pi) % (2 * np.pi) - np.pi)
    zero = np.hypot(*(p - np.roll(p, 1, axis=0)).T) < 1e-9
    a[zero] = 0
    return a


def corner_mask(pts, corner, closed):
    """Vertices that must stay: path ends and corners sharper than `corner` (rad)."""
    m = turn_angles(pts, closed) >= corner
    m[0] = m[-1] = True
    return m


def split_at_corners(pts, corner, closed):
    """Sections between kept vertices (each section shares its end points with its neighbours)."""
    idx = np.nonzero(corner_mask(pts, corner, closed))[0]
    return [pts[a:b + 1] for a, b in zip(idx[:-1], idx[1:]) if b > a]


def resample(sec, spacing, fit=True):
    """Points every `spacing` mm along the section (XY arc length), both ends kept. fit: adjust the spacing so the
    section divides into equal steps (no short last step)."""
    seg = _xy_len(sec)
    cum = np.concatenate([[0], np.cumsum(seg)])
    L = cum[-1]
    if L <= 1e-9:
        return sec[[0, -1]]
    if fit:
        n = max(int(round(L / spacing)), 1)
        sv = np.linspace(0, L, n + 1)
    else:
        sv = np.append(np.arange(0, L - 0.25 * spacing, spacing), L)
    return np.column_stack([np.interp(sv, cum, sec[:, k]) for k in range(sec.shape[1])])


def adaptive(sec, tol, lo, hi):
    """Greedy chord fit on a dense, even resampling of the section: from each point go as far as possible while
    every skipped point stays within `tol` of the chord and the chord is no longer than `hi`."""
    dense = resample(sec, lo, fit=True)
    if len(dense) <= 2:
        return dense
    p = dense[:, :2]
    kmax = len(dense) - 1 if math.isinf(hi) else max(int(hi / lo), 1)
    keep, i, n = [0], 0, len(dense)
    while i < n - 1:
        m = min(kmax, n - 1 - i)
        block = p[i:i + m + 1]
        ok = 1
        # try the longest chord first, shrink until every skipped point is within tol
        for j in range(m, 1, -1):
            a, b = block[0], block[j]
            ab = b - a
            ln = math.hypot(*ab)
            if ln < 1e-12:
                continue
            mid = block[1:j]
            dev = np.abs(ab[0] * (mid[:, 1] - a[1]) - ab[1] * (mid[:, 0] - a[0])) / ln
            if dev.max() <= tol:
                ok = j
                break
        i += ok
        keep.append(i)
    return dense[keep]


def smooth(sec, passes):
    """Laplacian smoothing with fixed end points (takes out the small zig-zag of traced outlines)."""
    out = sec.copy()
    for _ in range(passes):
        if len(out) < 3:
            break
        out[1:-1, :2] = 0.25 * out[:-2, :2] + 0.5 * out[1:-1, :2] + 0.25 * out[2:, :2]
    return out


def merge_close(pts, lo, keep=None):
    """Drop points closer than `lo` (XY) to the last kept point; ends and `keep` points always stay."""
    keep = np.zeros(len(pts), bool) if keep is None else keep
    out, last = [0], pts[0, :2]
    for i in range(1, len(pts) - 1):
        if keep[i] or math.hypot(pts[i, 0] - last[0], pts[i, 1] - last[1]) >= lo:
            out.append(i)
            last = pts[i, :2]
    if len(pts) > 1:
        if len(out) > 1 and not keep[out[-1]] and math.hypot(*(pts[-1, :2] - pts[out[-1], :2])) < lo:
            out.pop()
        out.append(len(pts) - 1)
    return pts[out]


def split_long(pts, hi):
    """Insert points so no segment is longer than `hi` (XY)."""
    rows = [pts[:1]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / hi - 1e-9))
        if n > 1:
            t = np.linspace(0, 1, n + 1)[1:, None]
            rows.append(a + t * (b - a))
        else:
            rows.append(b[None])
    return np.vstack(rows)
