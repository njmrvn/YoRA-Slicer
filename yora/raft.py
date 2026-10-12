"""Raft: a few layers printed under the part, and the part lifted onto them.

The raft area comes from the part's footprint (its first layer) and the raft type:
    outline           the footprint, holes filled
    hull              the closed area of the footprint (convex hull): an arc-shaped part gets a half disc
    rectangle         the bounding rectangle (sides along X / Y)
    fitted_rectangle  the smallest rectangle around the part, turned to fit it
    circle            the smallest circle around the part
grown by the raft offset. Each raft layer is an optional outline loop plus lines at the raft angle (turned by the
angle step on every layer), joined into one continuous zig-zag: from the end of a line the path follows the raft
edge to the start of the next one. The lower layers use the raft density, the top layer can be solid so the part's
first layer sits on a smooth surface. The part's layers are lifted by the raft thickness + the gap.
"""
import math

import cv2
import numpy as np

from .slicer import KIND_RAFT, Layer, Path

TYPES = ['none', 'outline', 'hull', 'rectangle', 'fitted_rectangle', 'circle']


def enabled(s):
    return s.get('raft_type', 'none') != 'none' and int(s.get('raft_layers', 0)) > 0


def raft_height(s):
    return s['raft_layer_height'] if s['raft_layer_height'] > 0 else s['first_layer_height']


def thickness(s):
    """How far the part is lifted: raft layers + gap."""
    return int(s['raft_layers']) * raft_height(s) + s['raft_gap'] if enabled(s) else 0.0


def raft_region(footprint, s, res):
    """Raft area (uint8 mask on the slicer grid) for the footprint mask of the part's first layer."""
    kind = s['raft_type']
    H, W = footprint.shape
    off_px = s['raft_offset'] / res
    region = np.zeros((H, W), np.uint8)
    pts = cv2.findNonZero(footprint)
    if pts is None:
        return region
    if kind == 'rectangle':
        x, y, w, h = cv2.boundingRect(pts)
        x0, y0 = int(round(x - off_px)), int(round(y - off_px))
        x1, y1 = int(round(x + w - 1 + off_px)), int(round(y + h - 1 + off_px))
        region[max(y0, 0):min(y1, H - 1) + 1, max(x0, 0):min(x1, W - 1) + 1] = 1
        return region
    if kind == 'fitted_rectangle':
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
        box = cv2.boxPoints(((cx, cy), (rw + 2 * off_px, rh + 2 * off_px), ang))
        cv2.fillPoly(region, [np.round(box * 16).astype(np.int32)], 1, cv2.LINE_8, 4)
        return region
    if kind == 'circle':
        (cx, cy), r = cv2.minEnclosingCircle(pts)
        cv2.circle(region, (int(round(cx * 16)), int(round(cy * 16))), int(round((r + off_px) * 16)), 1, -1, cv2.LINE_8, 4)
        return region
    if kind == 'hull':
        cv2.fillPoly(region, [cv2.convexHull(pts)], 1)
    else:                                                   # outline: the footprint, holes filled
        cs, _ = cv2.findContours(footprint.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(region, cs, -1, 1, -1)
    if off_px > 0:                                          # grow by the offset (rounded corners)
        dist = cv2.distanceTransform(1 - region, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        region = (dist <= off_px).astype(np.uint8)
    return region


def _edge_loops(mask, origin, res):
    """Boundary loops of a mask in mm, every boundary pixel (so each line end has an edge point next to it)."""
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    return [c.reshape(-1, 2).astype(np.float64) * res + origin for c in cs if len(c) >= 3]


def _along_edge(loops, p, q, tol, res):
    """Points along the edge loop from near p to near q (the shorter way round), or None if p and q are not both
    on the same loop."""
    best = None
    for L in loops:
        dp, dq = np.hypot(*(L - p).T), np.hypot(*(L - q).T)
        i, j = int(np.argmin(dp)), int(np.argmin(dq))
        if dp[i] <= tol and dq[j] <= tol and (best is None or dp[i] + dq[j] < best[0]):
            best = (dp[i] + dq[j], L, i, j)
    if best is None:
        return None
    _, L, i, j = best
    n = len(L)
    fwd = [(i + k) % n for k in range((j - i) % n + 1)]
    bwd = [(i - k) % n for k in range((i - j) % n + 1)]
    seg = lambda idx: float(np.hypot(*np.diff(L[idx], axis=0).T).sum()) if len(idx) > 1 else 0.0
    idx = fwd if seg(fwd) <= seg(bwd) else bwd
    pts = np.vstack([p, L[idx], q]).astype(np.float32)
    pts = cv2.approxPolyDP(pts.reshape(-1, 1, 2), 0.7 * res, False).reshape(-1, 2).astype(np.float64)   # pixel steps out
    return pts[1:-1]


def zigzag(segs, fill, origin, res, spacing):
    """Raft lines joined into continuous paths: nearest line next, joined along the raft edge. A new path starts
    only where the next line is not reachable along the edge (separate raft islands)."""
    if not segs:
        return []
    loops = _edge_loops(fill, origin, res)
    tol = 3 * res + 0.5
    reach = 4 * spacing + 2.0                                 # longest join along the edge
    A = np.array([s[0] for s in segs])
    B = np.array([s[1] for s in segs])
    d = (B[0] - A[0]) / max(float(np.hypot(*(B[0] - A[0]))), 1e-9)
    row = np.rint(((A + B) / 2) @ np.array([-d[1], d[0]]) / spacing).astype(int)   # which line of the pattern
    first = int(np.argmin(row))                                                    # start at one side
    left = np.ones(len(segs), dtype=bool)
    left[first] = False
    chains, cur, cur_row = [], [A[first], B[first]], row[first]
    for _ in range(len(segs) - 1):
        end = cur[-1]
        da = np.where(left, np.hypot(*(A - end).T), np.inf)
        db = np.where(left, np.hypot(*(B - end).T), np.inf)
        near = left & (np.abs(row - cur_row) <= 1)          # the zig-zag goes row by row: neighbours first
        if near.any():
            da, db = np.where(near, da, np.inf), np.where(near, db, np.inf)
        ia, ib = int(np.argmin(da)), int(np.argmin(db))
        k, start, stop = (ia, A[ia], B[ia]) if da[ia] <= db[ib] else (ib, B[ib], A[ib])
        left[k] = False
        straight = float(np.hypot(*(start - end)))
        neighbour = abs(row[k] - cur_row) <= 1
        path = _along_edge(loops, end, start, tol, res) if (neighbour or straight <= reach) else None
        if path is not None:
            along = float(np.hypot(*np.diff(np.vstack([end, path, start]), axis=0).T).sum())
            # a join to the next row may be long where the lines run nearly along the edge, but it must not go the
            # long way round the raft; other joins stay short
            ok = along <= 1.5 * straight + 2 * spacing if neighbour else along <= 2 * reach
        else:
            ok = False
        cur_row = row[k]
        if ok:
            cur += list(path) + [start, stop]
        else:
            chains.append(np.array(cur))
            cur = [start, stop]
    chains.append(np.array(cur))
    return chains


def add_raft(slicer, result):
    """Put raft layers under the sliced part (in place) and lift the part's layers onto them."""
    s, g = slicer.s, slicer.grid
    footprint = slicer.mask(0)
    if footprint is None or not footprint.any():
        result.warnings.append('Raft: the part has no first layer to build a raft under.')
        return result
    res = g.res
    region = raft_region(footprint, s, res)
    if (region[0, :].any() or region[-1, :].any() or region[:, 0].any() or region[:, -1].any()):
        result.warnings.append('Raft: the raft reaches the edge of the slicing area; reduce the raft offset.')
    w = s['line_width']
    rh = raft_height(s)
    n = int(s['raft_layers'])
    min_area = (0.5 * w) ** 2
    dist_in = cv2.distanceTransform(region, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
    raft = []
    for r in range(n):
        layer = Layer(r, (r + 1) * rh, rh, w)
        stages = []
        if s['raft_outline']:
            loops = slicer.contours(dist_in > w / 2, g.origin, min_area, external=True)
            if loops:
                stages.append([Path(KIND_RAFT, l, True) for l in loops])
        inset = w / 2 + (w * 0.9 if s['raft_outline'] else 0.0)
        fill = (dist_in > inset).astype(np.uint8)
        top = r == n - 1
        solid_top = top and s['raft_top_solid'] and n > 1        # a one-layer raft uses the density
        density = 100.0 if solid_top else min(max(s['raft_density'], 1.0), 100.0)
        spacing = w * 100.0 / density
        angle = s['raft_angle'] + r * s['raft_angle_step']
        segs = slicer.line_infill(fill, g.origin, spacing, angle, w * 0.5)
        lines = [Path(KIND_RAFT, c, False) for c in zigzag(segs, fill, g.origin, res, spacing)]
        if lines:
            stages.append(lines)
        if stages:
            layer.stages.append(stages)
        raft.append(layer)
    shift = n * rh + s['raft_gap']
    for layer in result.layers:                             # lift the part onto the raft
        layer.index += n
        layer.z += shift
        layer.shift = shift
        if layer.spiral is not None:
            p = layer.spiral.pts.copy()
            p[:, 2] += shift
            layer.spiral = Path(layer.spiral.kind, p, layer.spiral.closed)
    result.layers[:0] = raft
    result.raft = {'layers': n, 'shift': round(shift, 4), 'type': s['raft_type'],
                   'area_mm2': round(float(region.sum()) * res * res, 1)}
    return result
