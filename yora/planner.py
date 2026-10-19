"""Turn sliced layers into an ordered list of moves with absolute E, speeds, travel and retraction."""
import math
from dataclasses import dataclass, field

import numpy as np

from .slicer import (KIND_BRIM, KIND_INFILL, KIND_INNER, KIND_OUTER, KIND_RAFT, KIND_RETRACT, KIND_SOLID,
                     KIND_TRAVEL, KIND_NAMES)
from .settings import parse_int_list
from . import points as point_control
from . import pattern as surface_pattern

EXTRUDING = (KIND_OUTER, KIND_INNER, KIND_INFILL, KIND_SOLID, KIND_BRIM, KIND_RAFT)


@dataclass
class Toolpath:
    """One row per move: the machine goes from the previous row to this one."""
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    e: np.ndarray          # absolute E after the move
    f: np.ndarray          # speed, mm/s
    kind: np.ndarray
    layer: np.ndarray
    layer_z: list
    stats: dict = field(default_factory=dict)
    level: np.ndarray = None   # extruder flow level the move runs at (preview of the DO outputs)
    warnings: list = field(default_factory=list)
    # printing strategy (strategy.py): layers >= flat_layers lie on planes tilted by part_R (slice frame -> bed),
    # orient[i] = 1 where the nozzle tilts by `tilt` (4x4). None / defaults = all horizontal, nozzle vertical.
    flat_layers: int = None
    part_R: np.ndarray = None
    part_z0: float = 0.0
    tilt: np.ndarray = None
    orient: np.ndarray = None
    overhang: np.ndarray = None   # overhang angle (deg) of every move, walls only (overhang.py)

    def __len__(self):
        return len(self.x)


def e_per_mm(width, height, s):
    if width > height:
        area = (width - height) * height + math.pi * (height / 2) ** 2   # stadium bead
    else:
        area = math.pi * (width / 2) * (height / 2)
    area *= s['flow'] / 100.0
    if s['e_mode'] == 'filament':
        return area / (math.pi * (s['filament_diameter'] / 2) ** 2)
    return area


class Planner:
    def __init__(self, s):
        self.s = s
        self.rows = []
        self.pos = None          # (x, y, z)
        self.e = 0.0
        self.seams = {}          # wall kind -> seam points of this layer's loops
        self.prev_seams = {}     # ... and of the previous layer (aligned seams follow them)
        self.ramps = {}          # wall kind -> seam points of loops that ramped on this layer
        self.prev_ramps = {}
        self.next_layer = None
        self._spiral_u = 0.0     # spiral vase: path length printed so far (surface pattern phase)
        self._outer = (None, [])  # (layer, its outer wall loops): tells holes from outer boundaries
        self.pattern_ctx = None   # surface pattern: part centre, picture, paint (pattern.context)

    def _air_sign(self, path, layer):
        """+1 if the air is outside this CCW outer wall loop (an outer boundary), -1 if inside it (a hole)."""
        if self._outer[0] is not layer:
            self._outer = (layer, [p.pts for st in layer.stages for stage in st for p in stage
                                   if p.kind == KIND_OUTER and p.closed])
        others = [o for o in self._outer[1] if o is not path.pts]
        return -1.0 if surface_pattern.is_hole(path.pts, others) else 1.0

    def add(self, x, y, z, f, kind, layer):
        self.rows.append((x, y, z, self.e, f, kind, layer))
        self.pos = (x, y, z)

    # ---- travel ----
    def travel_to(self, p, z, layer):
        s = self.s
        x, y = float(p[0]), float(p[1])
        if self.pos is None:
            self.add(x, y, z + s['safe_z'], s['travel_speed'], KIND_TRAVEL, layer)
            self.add(x, y, z, s['travel_speed'], KIND_TRAVEL, layer)
            return
        cx, cy, cz = self.pos
        dist = math.hypot(x - cx, y - cy)
        if dist < 1e-6 and abs(z - cz) < 1e-6:
            return
        long_travel = dist > s['retract_min_travel']
        retract = long_travel and s['retract_length'] > 0
        if retract:
            self.e -= s['retract_length']
            self.add(cx, cy, cz, s['travel_speed'], KIND_RETRACT, layer)
        if long_travel and s['z_hop'] > 0:
            hz = max(cz, z) + s['z_hop']
            self.add(cx, cy, hz, s['travel_speed'], KIND_TRAVEL, layer)
            self.add(x, y, hz, s['travel_speed'], KIND_TRAVEL, layer)
        self.add(x, y, z, s['travel_speed'], KIND_TRAVEL, layer)
        if retract:
            self.e += s['retract_length']
            self.add(x, y, z, s['travel_speed'], KIND_RETRACT, layer)

    # ---- extrusion ----
    def speed_for(self, kind, layer_index):
        s = self.s
        if layer_index == 0:
            return s['first_layer_speed']
        return {KIND_OUTER: s['outer_wall_speed'], KIND_INNER: s['inner_wall_speed'],
                KIND_INFILL: s['infill_speed'], KIND_SOLID: s['solid_infill_speed'],
                KIND_BRIM: s['first_layer_speed'], KIND_RAFT: s['raft_speed']}[kind]

    # ---- seam ----
    def seam_target(self, pts, layer, kind=KIND_OUTER):
        """Point the loop should start closest to, for the chosen seam position."""
        s = self.s
        mode = s['seam']
        here = np.array(self.pos[:2]) if self.pos else None
        if mode == 'nearest' and here is not None:
            return here
        if mode == 'custom':
            return np.array([s['seam_x'], s['seam_y']], dtype=float)
        if mode == 'direction':
            a = math.radians(s['seam_angle'])
            return pts[:, :2].mean(axis=0) + 1e6 * np.array([math.cos(a), math.sin(a)])
        if mode == 'path':
            P = self.seam_path()
            if P is not None:
                return seam_nearest_path(pts[:, :2], layer.z - layer.height / 2 - layer.shift, P)   # model height
        if mode == 'random':
            self._rng_count = getattr(self, '_rng_count', 0) + 1
            ang = np.random.default_rng(layer.index * 7919 + self._rng_count).uniform(0, 2 * math.pi)
            c = pts[:, :2].mean(axis=0)
            return c + 1e6 * np.array([math.cos(ang), math.sin(ang)])
        if mode == 'aligned':
            # each loop follows the closest seam of the same kind of wall on the previous layer
            cands = self.prev_seams.get(kind) or self.prev_seams.get(KIND_OUTER) or []
            if cands:
                c = np.array(cands)
                d = np.hypot(*(c[:, None, :] - pts[None, :, :2]).transpose(2, 0, 1)).min(axis=1)
                return c[int(np.argmin(d))]
        # rear (and the first aligned loop): the back of the part, +Y, nearest the current x
        cx = here[0] if here is not None else pts[:, 0].mean()
        return np.array([cx, pts[:, 1].max() + 1e6])

    def seam_path(self):
        """The drawn seam line (n, 3), parsed once; None if empty."""
        if not hasattr(self, '_seam_path'):
            self._seam_path = parse_seam_path(self.s.get('seam_path', ''))
        return self._seam_path

    def loop_from_seam(self, pts, layer, kind=KIND_OUTER):
        """The closed loop (first point repeated at the end) starting exactly at the seam: the closest point of the
        loop to the seam target (a new vertex if it falls on a segment), or the sharpest corner near it."""
        loop = pts[:, :2]
        n = len(loop)
        target = self.seam_target(loop, layer, kind)
        a, b = loop, np.roll(loop, -1, axis=0)
        ab = b - a
        t = np.clip(((target - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-18), 0, 1)
        q = a + t[:, None] * ab
        i = int(np.argmin(np.hypot(*(q - target).T)))
        radius = self.s['seam_corner']
        if radius > 0 and n >= 3:
            prev, nxt = loop - np.roll(loop, 1, axis=0), np.roll(loop, -1, axis=0) - loop
            turn = np.abs((np.arctan2(nxt[:, 1], nxt[:, 0]) - np.arctan2(prev[:, 1], prev[:, 0]) + np.pi) % (2 * np.pi) - np.pi)
            seg = np.hypot(*ab.T)
            cum = np.concatenate([[0], np.cumsum(seg)])
            per = cum[-1]
            s0 = cum[i] + t[i] * seg[i]                                  # arc position of the seam point
            dist = np.abs((cum[:n] - s0 + per / 2) % per - per / 2)       # arc distance of each vertex from it
            cand = np.nonzero((dist <= radius) & (turn >= math.radians(20)))[0]
            if len(cand):
                # sharpest corner; corners within 1 deg of it count as equal and the closest one wins
                sharp = cand[turn[cand] >= turn[cand].max() - math.radians(1)]
                k = int(sharp[np.argmin(dist[sharp])])
                out = np.roll(loop, -k, axis=0)
                return np.vstack([out, out[:1]])
        if t[i] <= 1e-6:
            out = np.roll(loop, -i, axis=0)
        elif t[i] >= 1 - 1e-6:
            out = np.roll(loop, -((i + 1) % n), axis=0)
        else:
            out = np.vstack([q[i][None], np.roll(loop, -(i + 1), axis=0)])   # new vertex on segment i
        return np.vstack([out, out[:1]])

    # ---- extrusion with per-point Z and flow ----
    def _emit(self, pts, zs, factors, f, kind, layer_index, epm):
        """Extrude along pts (n,2) with heights zs (n,) and per-segment flow factors (n-1,). The bead is laid along
        the bed, so flow follows the horizontal length (a ramp's slope adds no material)."""
        for p, z, k in zip(pts[1:], zs[1:], factors):
            seg = math.hypot(p[0] - self.pos[0], p[1] - self.pos[1])
            if seg < 1e-9 and abs(z - self.pos[2]) < 1e-9:
                continue
            self.e += seg * epm * max(float(k), 0.0)
            self.add(float(p[0]), float(p[1]), float(z), f, kind, layer_index)

    @staticmethod
    def _cut(path, s0, s1, step=None):
        """Points of polyline `path` between arc lengths s0 and s1 (s1 may exceed the length: wraps around a closed
        loop), with vertices at both ends and extra points every `step` mm (for smooth ramps)."""
        seg = np.hypot(*np.diff(path, axis=0).T)
        cum = np.concatenate([[0], np.cumsum(seg)])
        per = cum[-1]
        marks = list(cum[(cum > s0) & (cum < s1)])
        if s1 > per:                                                   # wrap: second lap
            marks += list(cum[(cum > 0) & (cum < s1 - per)] + per)
        if step:
            marks += list(np.arange(s0 + step, s1, step))
        sv = np.unique(np.clip(np.array([s0] + marks + [s1]), s0, s1))
        u = np.where(sv > per, sv - per, sv)
        x = np.interp(u, cum, path[:, 0])
        y = np.interp(u, cum, path[:, 1])
        return np.column_stack([x, y]), sv

    def _wall_near(self, layer, kind, p, dist):
        """Does `layer` have a wall loop of this kind passing within `dist` of point p?"""
        if layer is None:
            return False
        for st in layer.stages:
            for stage in st:
                for q in stage:
                    if q.kind != kind or not q.closed:
                        continue
                    a = q.pts[:, :2]
                    ab = np.roll(a, -1, axis=0) - a
                    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-18), 0, 1)
                    if np.hypot(*(a + t[:, None] * ab - p).T).min() <= dist:
                        return True
        return False

    @staticmethod
    def _at(path, sv):
        """Points of a closed polyline (first point repeated at the end) at arc lengths sv (wrapping)."""
        cum = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
        u = np.mod(sv, cum[-1]) if cum[-1] > 0 else np.zeros_like(sv)
        return np.column_stack([np.interp(u, cum, path[:, 0]), np.interp(u, cum, path[:, 1])])

    def extrude_loop(self, path, layer, epm):
        """A closed wall loop: seam, then step / ramp / scarf layer change, seam gap / overlap and wipe.

        ramp  (continuous): the loop starts one layer down at the seam and rises over the ramp length, goes round and
              ends exactly at the seam; the next layer's ramp starts there and lies on this one, so the bead keeps a
              full layer of thickness and the path carries straight on. The first ramp of a wall (on a flat layer)
              grows from the start height, and where the wall ends (top of the part) the wedge is filled once.
        scarf: every loop is self-contained: rise over the ramp, go round, then pass over the ramp again filling the
              wedge (ends one ramp length past the seam). Use it with random / nearest seams."""
        s = self.s
        loop = self.loop_from_seam(path.pts, layer, path.kind)
        uneven = s['point_mode'] == 'simplify'
        if point_control.active(s, path.kind in (KIND_OUTER, KIND_INNER, KIND_BRIM)):
            loop = point_control.control(loop, s, closed=True)       # points start at the seam
        seam0 = loop[0].copy()                                         # the seam on the plain wall (pattern moves it)
        if path.kind == KIND_OUTER and surface_pattern.active(s):
            loop = surface_pattern.apply(loop, s, True, layer.index, self._air_sign(path, layer),
                                         model_z=layer.z - layer.height / 2 - layer.shift, ctx=self.pattern_ctx)
        per = float(np.hypot(*np.diff(loop, axis=0).T).sum())
        z = layer.z
        f = self.speed_for(path.kind, layer.index)
        walls = (KIND_OUTER, KIND_INNER) if s['ramp_walls'] == 'all_walls' else (KIND_OUTER,)
        mode = s['layer_change']
        ramp = (mode in ('ramp', 'scarf') and layer.index > 0 and path.kind in walls and s['ramp_length'] > 0 and per > 1e-6)
        stacked = mode == 'ramp' and s['seam'] in ('aligned', 'custom', 'rear', 'direction', 'path')   # seams of consecutive layers coincide
        w = s['line_width']
        below = [pl for sp, pl in self.prev_ramps.get(path.kind, []) if np.hypot(*(sp - seam0)) <= w]
        ramped_below = bool(below)
        self.seams.setdefault(path.kind, []).append(seam0)
        if ramp:
            self.ramps.setdefault(path.kind, []).append((seam0, loop))
            L = min(s['ramp_length'], 0.9 * per)
            h = layer.height
            continuing = stacked and ramped_below                      # on top of the previous layer's ramp
            r0 = 0.0 if continuing else s['ramp_start_height'] / 100.0
            z_low = z - h * (1.0 - r0)
            step = min(2.0, L / 4) if uneven else None              # even point modes already place the points
            up, sv = self._cut(loop, 0.0, L, step)                     # 1. ramp: Z rises by one layer over L
            zr = z_low + (z - z_low) * sv / L
            # bead thickness / layer height: constant on top of the previous ramp, growing on a flat layer
            thick = np.ones_like(zr) if continuing else (zr - (z - h)) / h
            ramp_xy = up
            if continuing:
                # start exactly where the loop below ended and move across onto this layer's wall while climbing,
                # so consecutive layers join without a sideways step (the wall can shift between layers)
                prev = self._at(below[0], sv)
                t = (sv / L)[:, None]
                ramp_xy = (1.0 - t) * prev + t * up
            self.travel_to(ramp_xy[0], z_low, layer.index)
            self._emit(ramp_xy, zr, (thick[:-1] + thick[1:]) / 2, f, path.kind, layer.index, epm)
            rest, _ = self._cut(loop, L, per)                           # 2. rest of the loop at full height
            self._emit(rest, np.full(len(rest), z), np.ones(len(rest) - 1), f, path.kind, layer.index, epm)
            # 3. fill the wedge under the ramp (top surface there is still sloped), unless the next layer's ramp will
            fill = not stacked or not self._wall_near(self.next_layer, path.kind, loop[0], w)
            if fill:
                gap = (z - zr) / h                                      # what is missing up to the layer top
                self._emit(up, np.full(len(up), z), (gap[:-1] + gap[1:]) / 2, f, path.kind, layer.index, epm)
                end_s = L
            else:
                end_s = per
        else:
            gap = s['seam_gap'] if path.kind in (KIND_OUTER, KIND_INNER) else 0.0
            end_s = max(per - gap, 0.5 * per) if gap > 0 else per + min(-gap, per)
            pts, _ = self._cut(loop, 0.0, end_s)
            self.travel_to(pts[0], z, layer.index)
            self._emit(pts, np.full(len(pts), z), np.ones(len(pts) - 1), f, path.kind, layer.index, epm)
        if s['wipe_length'] > 0 and path.kind in (KIND_OUTER, KIND_INNER):   # wipe: keep moving along the loop, extruder off
            w0 = end_s - per if end_s > per else end_s
            wipe, _ = self._cut(loop, w0, w0 + min(s['wipe_length'], per))
            for p in wipe[1:]:
                self.add(float(p[0]), float(p[1]), z, s['travel_speed'], KIND_TRAVEL, layer.index)

    def extrude_path(self, path, layer, epm):
        pts = path.pts
        if path.closed and pts.shape[1] == 2:
            return self.extrude_loop(path, layer, epm)
        if path.closed:
            pts = np.vstack([pts, pts[:1]])
        elif self.pos is not None:
            here = np.array(self.pos[:2])
            if np.hypot(*(pts[-1, :2] - here)) < np.hypot(*(pts[0, :2] - here)):
                pts = pts[::-1]
        if point_control.active(self.s, path.kind in (KIND_OUTER, KIND_INNER, KIND_BRIM) or pts.shape[1] == 3):
            pts = point_control.control(pts, self.s, closed=path.closed)
        if path is layer.spiral and surface_pattern.active(self.s):   # one spiral vase turn
            plain = pts
            pts = surface_pattern.apply(pts, self.s, False, layer.index, 1.0, spiral=True, u0=self._spiral_u,
                                        z_shift=layer.shift, ctx=self.pattern_ctx)
            self._spiral_u += float(np.hypot(*np.diff(plain[:, :2], axis=0).T).sum())
        z = layer.z
        has_z = pts.shape[1] == 3
        f = self.speed_for(path.kind, layer.index)
        if has_z and self.pos is not None and math.dist(self.pos, pts[0]) < 2 * self.s['line_width']:
            first = 0   # spiral continues from the previous turn: keep extruding, no travel
        else:
            self.travel_to(pts[0], float(pts[0, 2]) if has_z else z, layer.index)
            first = 1
        for p in pts[first:]:
            pz = float(p[2]) if has_z else z
            seg = math.sqrt((p[0] - self.pos[0]) ** 2 + (p[1] - self.pos[1]) ** 2 + (pz - self.pos[2]) ** 2)
            if seg < 1e-9:
                continue
            self.e += seg * epm
            self.add(float(p[0]), float(p[1]), pz, f, path.kind, layer.index)

    @staticmethod
    def _dist_to(path, here):
        pts = path.pts[:, :2]
        if path.closed:
            return float(np.min(np.hypot(*(pts - here).T)))
        return float(min(np.hypot(*(pts[0] - here)), np.hypot(*(pts[-1] - here))))

    def run_stage(self, paths, layer, epm):
        """Print paths in nearest-neighbour order from the current position."""
        left = list(paths)
        while left:
            if self.pos is None:
                idx = 0
            else:
                here = np.array(self.pos[:2])
                idx = min(range(len(left)), key=lambda i: self._dist_to(left[i], here))
            self.extrude_path(left.pop(idx), layer, epm)

    def plan(self, layers):
        if surface_pattern.active(self.s):
            self.pattern_ctx = surface_pattern.context(self.s, layers)
        for li, layer in enumerate(layers):
            if self.seams:
                self.prev_seams, self.seams = self.seams, {}
            self.prev_ramps, self.ramps = self.ramps, {}
            self.next_layer = layers[li + 1] if li + 1 < len(layers) else None
            epm = e_per_mm(layer.width, layer.height, self.s)
            if layer.brim:
                self.run_stage(layer.brim, layer, epm)
            if layer.spiral is not None:
                self.extrude_path(layer.spiral, layer, epm)
            islands = list(layer.stages)
            while islands:
                here = np.array(self.pos[:2]) if self.pos else None
                if here is None:
                    idx = 0
                else:
                    idx = min(range(len(islands)),
                              key=lambda i: min(self._dist_to(p, here) for p in islands[i][0]))
                for stage in islands.pop(idx):
                    self.run_stage(stage, layer, epm)
        if self.pos is not None:
            x, y, z = self.pos
            self.add(x, y, z + self.s['safe_z'], self.s['travel_speed'], KIND_TRAVEL, max(len(layers) - 1, 0))
        return self.rows


def parse_seam_path(text):
    """'x,y,z; x,y,z; ...' -> (n, 3) array sorted by z, or None."""
    pts = []
    for part in str(text or '').replace('\n', ';').split(';'):
        v = [float(t) for t in part.replace(' ', ',').split(',') if t.strip()]
        if len(v) >= 3:
            pts.append(v[:3])
        elif len(v) == 2:
            pts.append([v[0], v[1], 0.0])
    if not pts:
        return None
    P = np.array(pts, dtype=float)
    return P[np.argsort(P[:, 2], kind='stable')]


def seam_on_path(P, z):
    """XY of the seam line at height z: interpolated between its points (sorted by z), the end points beyond."""
    if len(P) == 1 or z <= P[0, 2]:
        return P[0, :2].copy()
    if z >= P[-1, 2]:
        return P[-1, :2].copy()
    k = int(np.searchsorted(P[:, 2], z)) - 1
    a, b = P[k], P[k + 1]
    t = (z - a[2]) / (b[2] - a[2]) if b[2] > a[2] else 0.0
    return a[:2] + t * (b[:2] - a[:2])


def seam_nearest_path(loop, z, P, step=1.0):
    """Point of the closed loop (at height z) closest in 3D to the polyline P (n, 3), sampled every `step` mm.
    Where the line crosses the layer this is the crossing; elsewhere the part of the loop nearest the line."""
    a = np.vstack([loop, loop[:1]])
    seg = np.hypot(*np.diff(a, axis=0).T)
    cum = np.concatenate([[0], np.cumsum(seg)])
    sv = np.arange(0, cum[-1], step) if cum[-1] > step else cum[:-1]
    Q = np.column_stack([np.interp(sv, cum, a[:, 0]), np.interp(sv, cum, a[:, 1]), np.full(len(sv), z)])
    if len(P) == 1:
        d = np.linalg.norm(Q - P[0], axis=1)
    else:
        A, B = P[:-1], P[1:]
        AB = B - A
        L2 = np.maximum((AB ** 2).sum(1), 1e-12)
        t = np.clip(np.einsum('qmk,mk->qm', Q[:, None, :] - A[None], AB) / L2, 0, 1)
        C = A[None] + t[..., None] * AB[None]
        d = np.linalg.norm(Q[:, None, :] - C, axis=2).min(axis=1)
    return Q[int(np.argmin(d)), :2]


def toolpath_from_rows(rows, layer_z, s, warnings=()):
    a = np.array(rows, dtype=np.float64).reshape(-1, 7)
    tp = Toolpath(a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5].astype(np.int8), a[:, 6].astype(np.int32),
                  list(layer_z), warnings=list(warnings) + surface_pattern.warnings(s))
    analyze(tp, s)
    return tp


def plan_layers(slice_result, s):
    rows = Planner(s).plan(slice_result.layers)
    tp = toolpath_from_rows(rows, [l.z for l in slice_result.layers], s, slice_result.warnings)
    if slice_result.raft:
        tp.stats['raft'] = slice_result.raft
    return tp


def flow_levels(tp, s):
    """Same rule as posts/Borunte_FGF.py: level = ceil(value / full_scale * max_level), clamped to 1..max."""
    n = len(tp)
    dx = np.diff(np.concatenate([[tp.x[0]], tp.x])) if n else np.zeros(0)
    dy = np.diff(np.concatenate([[tp.y[0]], tp.y])) if n else np.zeros(0)
    dz = np.diff(np.concatenate([[tp.z[0]], tp.z])) if n else np.zeros(0)
    de = np.diff(np.concatenate([[0.0], tp.e])) if n else np.zeros(0)
    seg = np.sqrt(dx * dx + dy * dy + dz * dz)
    ext = (de > 1e-4) & (seg > 1e-3)
    per_mm = np.where(ext, de / np.maximum(seg, 1e-9), 0.0)
    rate = per_mm * tp.f
    value = rate if s['flow_mode'] == 'rate' else per_mm
    full = s['flow_full_scale'] if s['flow_full_scale'] > 0 else (float(value.max()) if ext.any() else 1.0)
    dos = parse_int_list(s['flow_level_dos'])
    max_level = (1 << len(dos)) - 1 if dos else 1
    level = np.where(ext, np.clip(np.ceil(value / full * max_level - 1e-9), 1, max_level), 0).astype(np.int8)
    return level, full, max_level, seg, de, per_mm, rate, ext


def analyze(tp, s):
    n = len(tp)
    if n == 0:
        tp.stats = {'moves': 0, 'layers': 0}
        tp.level = np.zeros(0, dtype=np.int8)
        return
    level, full, max_level, seg, de, per_mm, rate, ext = flow_levels(tp, s)
    tp.level = level
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(seg > 0, seg / np.maximum(tp.f, 1e-9), 0.0)
    e_total = float(de[ext].sum())
    volume = e_total if s['e_mode'] == 'volumetric' else e_total * math.pi * (s['filament_diameter'] / 2) ** 2
    nl = int(tp.layer.max()) + 1
    layer_time = np.bincount(tp.layer, weights=t, minlength=nl)
    # Extruder on/off and level switches, as the post would write them
    switches = int(np.count_nonzero(np.diff(level[(level > 0) | (tp.kind == KIND_TRAVEL)]) != 0))
    bbox = [float(tp.x[ext].min()), float(tp.y[ext].min()), float(tp.z[ext].min()),
            float(tp.x[ext].max()), float(tp.y[ext].max()), float(tp.z[ext].max())] if ext.any() else None
    tp.stats = {
        'moves': n,
        'layers': nl,
        'time_s': float(t.sum()),
        'extrude_mm': float(seg[ext].sum()),
        'travel_mm': float(seg[tp.kind == KIND_TRAVEL].sum()),
        'e_total': e_total,
        'volume_cm3': volume / 1000.0,
        'mass_g': volume / 1000.0 * s['density'],
        'max_rate': float(rate.max()),
        'max_per_mm': float(per_mm.max()),
        'max_volumetric_mm3s': float(rate.max()) * (1.0 if s['e_mode'] == 'volumetric' else math.pi * (s['filament_diameter'] / 2) ** 2),
        'flow_full_scale': full,
        'flow_max_level': max_level,
        'flow_level_hist': np.bincount(level, minlength=max_level + 1).tolist(),
        'io_switches_est': switches,
        'layer_time_min': float(layer_time.min()),
        'layer_time_max': float(layer_time.max()),
        'bbox': bbox,
        'by_kind_mm': {KIND_NAMES[k]: float(seg[(tp.kind == k) & ext].sum()) for k in EXTRUDING},
    }
