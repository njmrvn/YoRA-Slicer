"""Planar slicer for large-nozzle (FGF) printing.

Pipeline per layer:
  mesh section at mid-layer -> oriented loops -> raster mask (non-zero winding, so overlapping
  shells merge) -> Euclidean distance transform -> walls are iso-contours of the distance field,
  infill lines are clipped against thresholded regions. Working on a raster makes offsets robust
  to self-intersections and holes without a polygon clipping library.
"""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .points import source_tolerance

KIND_TRAVEL, KIND_OUTER, KIND_INNER, KIND_INFILL, KIND_SOLID, KIND_BRIM, KIND_RETRACT, KIND_RAFT = range(8)
KIND_NAMES = {KIND_TRAVEL: 'travel', KIND_OUTER: 'outer wall', KIND_INNER: 'inner wall', KIND_INFILL: 'sparse infill',
              KIND_SOLID: 'solid infill', KIND_BRIM: 'brim', KIND_RETRACT: 'retract', KIND_RAFT: 'raft'}
MAX_PIXELS = 25e6


@dataclass
class Path:
    kind: int
    pts: np.ndarray       # (k, 2), or (k, 3) for spiral paths with their own Z
    closed: bool


@dataclass
class Layer:
    index: int
    z: float              # top of the layer (nozzle height)
    height: float
    width: float
    stages: list = field(default_factory=list)   # list of island groups: [ [stage paths], [stage paths], ... ]
    brim: list = field(default_factory=list)     # Paths
    spiral: Path = None
    shift: float = 0.0    # how far the layer was lifted (onto a raft): its z in the model is z - shift


@dataclass
class SliceResult:
    layers: list
    resolution: float
    warnings: list
    raft: dict = None     # raft.add_raft(): layers, shift, type, area


def clip_loop(loop, a, b, c):
    """Part of a closed polygon where a*x + b*y >= c (Sutherland-Hodgman, one half-plane)."""
    p = np.asarray(loop, dtype=float)
    if len(p) > 1 and np.allclose(p[0], p[-1]):
        p = p[:-1]
    v = p[:, 0] * a + p[:, 1] * b - c
    if (v >= 0).all():
        return p
    if (v < 0).all():
        return None
    out = []
    n = len(p)
    for i in range(n):
        j = (i + 1) % n
        if v[i] >= 0:
            out.append(p[i])
        if (v[i] >= 0) != (v[j] >= 0):
            t = v[i] / (v[i] - v[j])
            out.append(p[i] + t * (p[j] - p[i]))
    return np.array(out) if len(out) >= 3 else None


def signed_area(pts):
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


class Grid:
    def __init__(self, xmin, ymin, xmax, ymax, res):
        self.res = res
        self.x0, self.y0 = xmin, ymin
        self.w = int(math.ceil((xmax - xmin) / res)) + 1
        self.h = int(math.ceil((ymax - ymin) / res)) + 1

    @property
    def origin(self):
        return np.array([self.x0, self.y0])


class Slicer:
    def __init__(self, tris, s, progress=None, clip=None, z_limit=None, z_top=None):
        """clip = (a, b, r, z0, base): keep only a*x + b*y + r*(z + z0) >= base in each layer (the angled layers above
        a horizontal base, see strategy.py); z_limit: only layers up to this height; z_top: model top (default: mesh)."""
        self.s = s
        self.clip = clip
        self.progress = progress or (lambda stage, frac: None)
        self.T = np.asarray(tris, dtype=np.float64)
        self.zmin = self.T[:, :, 2].min(axis=1)
        self.zmax = self.T[:, :, 2].max(axis=1)
        n = np.cross(self.T[:, 1] - self.T[:, 0], self.T[:, 2] - self.T[:, 0])
        # Direction of the section segment with material on its left (CCW outer loops): z x n
        self.seg_dir = np.stack([-n[:, 1], n[:, 0]], axis=1)
        self.warnings = []
        self._masks = {}
        self._outlines = {}     # layer -> exact cross-section loops (for smooth walls)
        self._index = {}

        w = s['line_width']
        lo, hi = self.T.reshape(-1, 3).min(axis=0), self.T.reshape(-1, 3).max(axis=0)
        margin = (s['brim_loops'] + 3) * w * max(1.0, s['first_layer_width_pct'] / 100.0) + 2.0
        if s.get('raft_type', 'none') != 'none':                  # room for the raft around the part
            reach = s['raft_offset']
            if s['raft_type'] in ('circle', 'fitted_rectangle'):     # these can reach past the part's box
                bw, bh = hi[0] - lo[0], hi[1] - lo[1]
                reach += math.hypot(bw, bh) / 2 - min(bw, bh) / 2
            margin = max(margin, reach + 3 * w + 2.0)
        res = s['resolution'] if s['resolution'] > 0 else min(max(w / 10.0, 0.05), 1.0)
        span = (hi[0] - lo[0] + 2 * margin) * (hi[1] - lo[1] + 2 * margin)
        if span / res ** 2 > MAX_PIXELS:
            res = math.sqrt(span / MAX_PIXELS)
            self.warnings.append('Slicing resolution raised to %.2f mm to fit memory.' % res)
        self.grid = Grid(lo[0] - margin, lo[1] - margin, hi[0] + margin, hi[1] + margin, res)
        self.zmodel = hi[2] if z_top is None else z_top
        self.layers_z = self._layer_heights()
        if z_limit is not None:
            self.layers_z = [(z, h) for z, h in self.layers_z if z <= z_limit + 1e-6]

    # ---------------- geometry ----------------
    def _layer_heights(self):
        s = self.s
        out = []
        z, h = s['first_layer_height'], s['first_layer_height']
        while z - h / 2 < self.zmodel - 1e-6:
            out.append((z, h))
            h = s['layer_height']
            z += h
        return out

    def section(self, z):
        """Oriented closed loops of the mesh cut at height z."""
        sel = np.nonzero((self.zmin <= z) & (self.zmax >= z))[0]
        if len(sel) == 0:
            return []
        T = self.T[sel]
        d = T[:, :, 2] - z
        d[d == 0] = 1e-9
        m = len(sel)
        E = np.empty((m, 3, 2))
        C = np.empty((m, 3), dtype=bool)
        with np.errstate(divide='ignore', invalid='ignore'):
            for j, (a, b) in enumerate(((0, 1), (1, 2), (2, 0))):
                da, db = d[:, a], d[:, b]
                C[:, j] = (da > 0) != (db > 0)
                t = np.where(C[:, j], da / (da - db), 0.0)
                E[:, j] = T[:, a, :2] + (T[:, b, :2] - T[:, a, :2]) * t[:, None]
        ok = C.sum(axis=1) == 2
        if not ok.any():
            return []
        E, C, dirv = E[ok], C[ok], self.seg_dir[sel][ok]
        order = np.argsort(~C, axis=1, kind='stable')[:, :2]
        rows = np.arange(len(E))
        p0, p1 = E[rows, order[:, 0]].copy(), E[rows, order[:, 1]].copy()
        flip = ((p1 - p0) * dirv).sum(axis=1) < 0
        tmp = p0[flip].copy()
        p0[flip] = p1[flip]
        p1[flip] = tmp
        keep = np.hypot(*(p1 - p0).T) > 1e-9
        loops = chain_segments(p0[keep], p1[keep])
        if self.clip is not None:
            a, b, r, z0, base = self.clip
            c = base - r * (z + z0)
            loops = [q for q in (clip_loop(l, a, b, c) for l in loops) if q is not None]
        return loops

    def rasterize(self, loops):
        g = self.grid
        acc = np.zeros((g.h, g.w), dtype=np.int16)
        areas = [signed_area(l) if len(l) >= 3 else 0.0 for l in loops]
        sign = -1.0 if sum(areas) < 0 else 1.0   # whole mesh inverted: flip
        for loop, a in zip(loops, areas):
            if abs(a) < 1e-9:
                continue
            px = (loop - g.origin) / g.res
            lo = np.maximum(np.floor(px.min(axis=0)).astype(int) - 1, 0)
            hi = np.minimum(np.ceil(px.max(axis=0)).astype(int) + 2, [g.w, g.h])
            if (hi <= lo).any():
                continue
            sub = np.zeros((hi[1] - lo[1], hi[0] - lo[0]), dtype=np.uint8)
            pts = np.round((px - lo) * 16).astype(np.int32).reshape(-1, 1, 2)
            cv2.fillPoly(sub, [pts], 1, cv2.LINE_8, 4)
            if a * sign > 0:
                acc[lo[1]:hi[1], lo[0]:hi[0]] += sub
            else:
                acc[lo[1]:hi[1], lo[0]:hi[0]] -= sub
        return (acc > 0).astype(np.uint8)

    def mask(self, i):
        if i < 0 or i >= len(self.layers_z):
            return None
        if i not in self._masks:
            z, h = self.layers_z[i]
            loops = self.section(z - h / 2)
            self._masks[i] = self.rasterize(loops)
            self._outlines[i] = loops
        return self._masks[i]

    def outline_index(self, i, reach):
        """Exact cross-section (the mesh cut) of layer i, indexed for nearest-point queries up to ~reach mm.
        One index per layer (the last one is kept); it is rebuilt only if a larger reach is asked for."""
        cur = self._index.get(i)
        if cur is None or cur.reach < reach:
            self.mask(i)
            self._index = {i: SegmentIndex(self._outlines.get(i) or [], reach + 3 * self.grid.res)}
            self._index[i].reach = reach
        return self._index[i]

    def contours(self, region, origin, min_area, closed_only=True, external=False, offset=None):
        """Closed loops around region (CCW). With offset=(layer, distance mm) the loops are iso-distance lines of
        the layer's outline: every traced pixel is moved onto the exact offset of the mesh cross-section, so walls
        follow the model instead of the pixel grid (no staircase), then simplified to the 'simplify' tolerance."""
        mode = cv2.RETR_EXTERNAL if external else cv2.RETR_LIST
        cs, _ = cv2.findContours(region.astype(np.uint8), mode, cv2.CHAIN_APPROX_NONE)
        res = self.grid.res
        eps = max(self.s['simplify'] / res, 0.5)
        index = self.outline_index(offset[0], offset[1]) if offset is not None else None
        out = []
        for c in cs:
            if len(c) < 3:
                continue
            if index is not None and len(c) >= 8:
                raw = c.reshape(-1, 2).astype(np.float64) * res + origin
                pts = forward_only(index.snap(raw, offset[1], tol=2.0 * res), raw)
                tol = source_tolerance(self.s)
                pts = cv2.approxPolyDP(pts.astype(np.float32).reshape(-1, 1, 2), tol, True).reshape(-1, 2).astype(np.float64)
                pts = remove_hooks(pts, 3 * res)
            else:
                c = cv2.approxPolyDP(c, eps, True)
                pts = c.reshape(-1, 2).astype(np.float64) * res + origin
            if len(pts) < 3:
                continue
            a = signed_area(pts)
            if abs(a) < min_area:
                continue
            out.append(pts if a > 0 else pts[::-1].copy())
        return out

    # ---------------- infill ----------------
    def line_infill(self, region, origin, spacing, angle_deg, min_len):
        if spacing <= 0 or not region.any():
            return []
        res = self.grid.res
        H, W = region.shape
        th = math.radians(angle_deg)
        u = np.array([math.cos(th), math.sin(th)])
        n = np.array([-math.sin(th), math.cos(th)])
        corners = origin + np.array([[0, 0], [W * res, 0], [0, H * res], [W * res, H * res]])
        cn, cu = corners @ n, corners @ u
        ks = np.arange(math.ceil(cn.min() / spacing), math.floor(cn.max() / spacing) + 1)
        if len(ks) == 0:
            return []
        t = np.arange(cu.min(), cu.max() + res, res * 0.7)
        P = ks[:, None, None] * spacing * n + t[None, :, None] * u
        idx = np.rint((P - origin) / res).astype(np.int64)
        ix, iy = idx[..., 0], idx[..., 1]
        inside = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        vals = np.zeros(ix.shape, dtype=np.int8)
        vals[inside] = region[iy[inside], ix[inside]]
        padded = np.pad(vals, ((0, 0), (1, 1)))
        dv = np.diff(padded, axis=1)
        sr, sc = np.nonzero(dv == 1)
        er, ec = np.nonzero(dv == -1)
        segs = []
        for r, a, b in zip(sr, sc, ec - 1):   # nonzero returns row-major order, so starts/ends pair up
            p, q = P[r, a], P[r, b]
            if np.hypot(*(q - p)) >= min_len:
                segs.append((p, q))
        return segs

    def chain_lines(self, segs, region, origin, max_gap):
        """Greedy nearest-neighbour ordering; joins line ends with an extruded connector when it stays inside."""
        if not segs:
            return []
        res = self.grid.res
        allowed = cv2.dilate(region.astype(np.uint8), np.ones((3, 3), np.uint8))
        A = np.array([s[0] for s in segs])
        B = np.array([s[1] for s in segs])
        left = np.ones(len(segs), dtype=bool)
        left[0] = False
        chains, cur = [], [A[0], B[0]]
        for _ in range(len(segs) - 1):
            end = cur[-1]
            da = np.where(left, np.hypot(*(A - end).T), np.inf)
            db = np.where(left, np.hypot(*(B - end).T), np.inf)
            ia, ib = int(np.argmin(da)), int(np.argmin(db))
            if da[ia] <= db[ib]:
                k, start, stop, gap = ia, A[ia], B[ia], da[ia]
            else:
                k, start, stop, gap = ib, B[ib], A[ib], db[ib]
            left[k] = False
            if max_gap > 0 and gap <= max_gap and self._inside(end, start, allowed, origin, res):
                cur += [start, stop]
            else:
                chains.append(np.array(cur))
                cur = [start, stop]
        chains.append(np.array(cur))
        return chains

    @staticmethod
    def _inside(p, q, allowed, origin, res):
        n = max(2, int(math.ceil(np.hypot(*(q - p)) / (res * 0.7))) + 1)
        pts = p + (q - p) * np.linspace(0, 1, n)[:, None]
        idx = np.rint((pts - origin) / res).astype(np.int64)
        H, W = allowed.shape
        if (idx[:, 0] < 0).any() or (idx[:, 1] < 0).any() or (idx[:, 0] >= W).any() or (idx[:, 1] >= H).any():
            return False
        return bool(allowed[idx[:, 1], idx[:, 0]].all())

    # ---------------- per layer ----------------
    def cover(self, i):
        """Pixels that have material in every layer within the top/bottom solid window."""
        s = self.s
        out = None
        for j in list(range(i - s['bottom_layers'], i)) + list(range(i + 1, i + s['top_layers'] + 1)):
            m = self.mask(j)
            if m is None:
                return np.zeros((self.grid.h, self.grid.w), dtype=np.uint8)
            out = m.copy() if out is None else (out & m)
        return out if out is not None else np.ones((self.grid.h, self.grid.w), dtype=np.uint8)

    def build_layer(self, i, prev_spiral_end):
        s, g = self.s, self.grid
        z, h = self.layers_z[i]
        w = s['line_width'] * (s['first_layer_width_pct'] / 100.0 if i == 0 else 1.0)
        layer = Layer(i, z, h, w)
        mask = self.mask(i)
        if mask is None or not mask.any():
            return layer
        res = g.res
        dist = cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
        min_area = (0.5 * w) ** 2
        nwalls = s['walls']
        spiral = s['spiral_vase'] and i >= s['bottom_layers'] and nwalls > 0

        if i == 0 and s['brim_loops'] > 0 and s.get('raft_type', 'none') == 'none':   # with a raft: no brim
            dout = cv2.distanceTransform(1 - mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
            for k in reversed(range(s['brim_loops'])):
                for loop in self.contours(dout <= (w / 2 + k * w), g.origin, min_area, external=True, offset=(i, w / 2 + k * w)):
                    layer.brim.append(Path(KIND_BRIM, loop, True))

        if spiral:
            loops = self.contours(dist > w / 2, g.origin, min_area, external=True, offset=(i, w / 2))
            if not loops:
                return layer
            loop = max(loops, key=lambda l: abs(signed_area(l)))
            if len(loops) > 1:
                self._warn_once('spiral', 'Spiral vase uses only the largest outline; other islands are skipped.')
            target = prev_spiral_end if prev_spiral_end is not None else loop[np.argmax(loop[:, 1])]
            k = int(np.argmin(np.hypot(*(loop - target[:2]).T)))
            loop = np.vstack([np.roll(loop, -k, axis=0), np.roll(loop, -k, axis=0)[:1]])
            seg = np.hypot(*np.diff(loop, axis=0).T)
            frac = np.concatenate([[0], np.cumsum(seg)]) / max(seg.sum(), 1e-9)
            z0 = z - h
            zs = z0 + frac * h
            layer.spiral = Path(KIND_OUTER, np.column_stack([loop, zs]), False)
            return layer

        cover = self.cover(i)
        solid_all = mask & (1 - cover)
        n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        overlap = s['infill_overlap'] / 100.0
        infill_d = (nwalls * w + w / 2 - overlap * w) if nwalls > 0 else w / 2
        density = s['infill_density'] / 100.0
        angle = s['infill_angle'] + (90.0 if i % 2 else 0.0)
        thin = 0
        for lab in range(1, n_lab):
            x, y, ww, hh, area = stats[lab]
            ys = slice(max(y - 1, 0), min(y + hh + 1, g.h))
            xs = slice(max(x - 1, 0), min(x + ww + 1, g.w))
            origin = g.origin + np.array([xs.start, ys.start]) * res
            isl = labels[ys, xs] == lab
            d = np.where(isl, dist[ys, xs], 0)
            stages = []
            walls = []
            for k in range(nwalls):
                loops = self.contours(d > w / 2 + k * w, origin, min_area, offset=(i, w / 2 + k * w))
                if not loops:
                    break
                walls.append([Path(KIND_OUTER if k == 0 else KIND_INNER, l, True) for l in loops])
            if nwalls > 0 and not walls:
                if area * res * res > min_area:
                    thin += 1
                continue
            order = list(reversed(walls)) if s['wall_order'] == 'inner_first' else walls
            stages.extend(order)

            region = d > infill_d
            solid = region & (solid_all[ys, xs] > 0)
            sparse = region & ~solid
            infill = []
            max_gap_solid = 3 * w if s['connect_infill'] else 0
            segs = self.line_infill(solid, origin, w, angle, w * 0.5)
            infill += [Path(KIND_SOLID, c, False) for c in self.chain_lines(segs, solid, origin, max_gap_solid)]
            if density > 0 and sparse.any():
                spacing = w / min(density, 1.0)
                if s['infill_pattern'] == 'concentric':
                    k = 0
                    while True:
                        loops = self.contours(sparse & (d > infill_d + k * spacing), origin, min_area)
                        if not loops:
                            break
                        infill += [Path(KIND_INFILL, l, True) for l in loops]
                        k += 1
                else:
                    segs = self.line_infill(sparse, origin, spacing, angle, w * 0.5)
                    max_gap = 3 * spacing if s['connect_infill'] else 0
                    infill += [Path(KIND_INFILL, c, False) for c in self.chain_lines(segs, sparse, origin, max_gap)]
            if infill:
                stages.append(infill)
            if stages:
                layer.stages.append(stages)
        if thin:
            self._warn_once('thin', 'Some regions are thinner than one line width and were skipped (first at layer %i).' % (i + 1))
        return layer

    def _warn_once(self, key, text):
        if key not in getattr(self, '_warned', set()):
            self._warned = getattr(self, '_warned', set()) | {key}
            self.warnings.append(text)

    def run(self):
        layers = []
        spiral_end = None
        n = len(self.layers_z)
        for i in range(n):
            layer = self.build_layer(i, spiral_end)
            if layer.spiral is not None:
                spiral_end = layer.spiral.pts[-1]
            layers.append(layer)
            for j in [j for j in self._masks if j < i - self.s['bottom_layers']]:
                del self._masks[j]
                self._outlines.pop(j, None)
            self.progress('slicing', (i + 1) / max(n, 1))
        return SliceResult(layers, self.grid.res, self.warnings)


def forward_only(pts, raw):
    """Drop snapped points that step backwards along the traced contour (several pixels can snap to nearly the
    same spot at corners; out-of-order points would make the path double back on itself, a visible hook)."""
    n = len(pts)
    if n < 4:
        return pts
    tan = np.roll(raw, -1, axis=0) - np.roll(raw, 1, axis=0)      # contour direction at each traced pixel
    keep = [0]
    for i in range(1, n):
        if np.dot(pts[i] - pts[keep[-1]], tan[i]) > 1e-9:
            keep.append(i)
    while len(keep) > 3 and np.dot(pts[keep[0]] - pts[keep[-1]], tan[keep[-1]]) <= 1e-9:
        keep.pop()                                                   # last points already past the start
    return pts[keep]


def remove_hooks(pts, max_len):
    """Remove vertices of a closed polygon where the path doubles back (turn > 120 deg) over a segment shorter
    than max_len: snapping artefacts, not geometry."""
    pts = np.asarray(pts, dtype=float)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        a = pts - np.roll(pts, 1, axis=0)           # incoming segment
        b = np.roll(pts, -1, axis=0) - pts          # outgoing segment
        la, lb = np.hypot(*a.T), np.hypot(*b.T)
        cos = (a * b).sum(1) / np.maximum(la * lb, 1e-12)
        bad = (cos < -0.5) & (np.minimum(la, lb) < max_len)
        if bad.any():
            pts = np.delete(pts, int(np.argmax(bad)), axis=0)
            changed = True
    return pts


class SegmentIndex:
    """Line segments of the cross-section in a bucket grid, for nearest-point queries near the outline."""

    def __init__(self, loops, cell):
        self.cell = max(cell, 1e-3)
        loops = [l for l in loops if len(l) >= 2]
        if not loops:
            self.A = self.B = np.zeros((0, 2))
            self.buckets = {}
            return
        self.A = np.vstack(loops)
        self.B = np.vstack([np.roll(l, -1, axis=0) for l in loops])
        # long edges are split for bucketing (each piece keeps its segment id)
        L = np.hypot(*(self.B - self.A).T)
        n = np.maximum(1, np.ceil(L / (self.cell / 2)).astype(int))
        seg = np.repeat(np.arange(len(L)), n)
        t = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 0.5) / np.repeat(n, n)
        mid = self.A[seg] + (self.B[seg] - self.A[seg]) * t[:, None]
        keys = np.floor(mid / self.cell).astype(np.int64)
        order = np.lexsort((keys[:, 1], keys[:, 0]))
        keys, seg = keys[order], seg[order]
        brk = np.nonzero(np.any(np.diff(keys, axis=0) != 0, axis=1))[0] + 1
        self.buckets = {tuple(keys[s]): np.unique(g) for s, g in zip(np.r_[0, brk], np.split(seg, brk))}

    def _candidates(self, key):
        """Segment ids in the 5 x 5 cells around a cell (cached)."""
        ids = self._cand.get(key)
        if ids is None:
            kx, ky = key
            cand = [self.buckets[(kx + dx, ky + dy)] for dx in (-2, -1, 0, 1, 2) for dy in (-2, -1, 0, 1, 2)
                    if (kx + dx, ky + dy) in self.buckets]
            ids = np.unique(np.concatenate(cand)) if cand else np.zeros(0, dtype=np.int64)
            self._cand[key] = ids
        return ids

    def nearest(self, P):
        """Closest outline point C and distance D for each point of P (inf where nothing is within reach)."""
        P = np.asarray(P, dtype=float)
        C = P.copy()
        D = np.full(len(P), np.inf)
        if not self.buckets or len(P) == 0:
            return C, D
        if not hasattr(self, '_cand'):
            self._cand = {}
        keys = np.floor(P / self.cell).astype(np.int64)
        uk, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        lists = [self._candidates((int(kx), int(ky))) for kx, ky in uk]
        K = max((len(l) for l in lists), default=0)
        if K == 0:
            return C, D
        table = np.full((len(uk), K), -1, dtype=np.int64)     # candidate ids per cell, padded with -1
        for r, l in enumerate(lists):
            table[r, :len(l)] = l
        for s in range(0, len(P), max(1, 4_000_000 // K)):      # bounded memory: points x candidates per chunk
            pi = np.arange(s, min(s + max(1, 4_000_000 // K), len(P)))
            ids = table[inv[pi]]
            valid = ids >= 0
            a, b = self.A[np.where(valid, ids, 0)], self.B[np.where(valid, ids, 0)]
            p = P[pi][:, None, :]
            ab = b - a
            t = np.clip(((p - a) * ab).sum(2) / np.maximum((ab ** 2).sum(2), 1e-18), 0.0, 1.0)
            q = a + t[..., None] * ab
            d = np.where(valid, np.hypot(q[..., 0] - p[..., 0], q[..., 1] - p[..., 1]), np.inf)
            j = np.argmin(d, axis=1)
            r = np.arange(len(pi))
            D[pi] = d[r, j]
            C[pi] = q[r, j]
        return C, D

    def snap(self, P, dist, tol):
        """Move points onto the line at exactly `dist` from the outline (along the direction to the closest
        outline point). Points whose exact distance disagrees with the raster by more than tol stay put (e.g.
        overlapping shells, where the nearest mesh edge is inside the merged part)."""
        P = np.asarray(P, dtype=float)
        out = P.copy()
        for _ in range(2):
            C, D = self.nearest(out)
            ok = np.isfinite(D) & (np.abs(D - dist) <= tol) & (D > 1e-9)
            out[ok] = C[ok] + (out[ok] - C[ok]) * (dist / D[ok])[:, None]
        return out


def chain_segments(p0, p1, q=1e-4):
    """Join directed segments end-to-start into loops (open chains are kept and closed implicitly)."""
    if len(p0) == 0:
        return []
    k0 = np.round(p0 / q).astype(np.int64)
    k1 = np.round(p1 / q).astype(np.int64)
    starts = {}
    for i, k in enumerate(map(tuple, k0.tolist())):
        starts.setdefault(k, []).append(i)
    k1t = list(map(tuple, k1.tolist()))
    k0t = list(map(tuple, k0.tolist()))
    used = np.zeros(len(p0), dtype=bool)

    def take(key):
        for dx in (0, -1, 1):
            for dy in (0, -1, 1):
                lst = starts.get((key[0] + dx, key[1] + dy))
                while lst:
                    j = lst.pop()
                    if not used[j]:
                        return j
        return None

    loops = []
    for i in range(len(p0)):
        if used[i]:
            continue
        used[i] = True
        pts, cur = [p0[i]], i
        while True:
            ke = k1t[cur]
            if abs(ke[0] - k0t[i][0]) <= 1 and abs(ke[1] - k0t[i][1]) <= 1 and len(pts) > 1:
                break
            j = take(ke)
            if j is None:
                pts.append(p1[cur])
                break
            used[j] = True
            pts.append(p0[j])
            cur = j
        loops.extend(l for l in split_at_repeats(np.array(pts), q) if len(l) >= 3)
    return loops


def split_at_repeats(pts, q=1e-4):
    """Split a chained loop that passes through the same vertex twice (shells touching at a point
    or edge) into simple loops. A figure-8 would otherwise be filled even-odd and lose the overlap."""
    keys = list(map(tuple, np.round(pts / q).astype(np.int64).tolist()))
    stack, seen, out = [], {}, []
    for p, k in zip(pts, keys):
        if k in seen:
            i = seen[k]
            out.append(np.array(stack[i:]))
            for kk in keys_of(stack[i:], q):
                seen.pop(kk, None)
            del stack[i:]
        seen[k] = len(stack)
        stack.append(p)
    out.append(np.array(stack))
    return out


def keys_of(pts, q):
    return [tuple(v) for v in np.round(np.array(pts) / q).astype(np.int64).tolist()]


def slice_mesh(tris, s, progress=None):
    from . import raft
    sl = Slicer(tris, s, progress)
    result = sl.run()
    if raft.enabled(s):
        if s['brim_loops'] > 0:
            result.warnings.append('Raft: the brim is left out (the raft holds the part).')
        raft.add_raft(sl, result)
    return result
