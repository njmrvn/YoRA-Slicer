"""Solve robot joints for a toolpath and check it (reach, limits, singularity, configuration, collisions).

For every move the nozzle points straight down at the bed (tool Z normal to the bed). Because the
nozzle is round, its rotation about tool Z is free: the solver keeps it constant while it can and
otherwise searches within +-rotz_range for a rotation that keeps the robot inside its joint limits,
away from the wrist singularity, without a configuration change ("minimum tool orientation change").

Error bits per move:
  1 unreachable, 2 joint limit, 4 wrist singularity, 8 configuration change / joint jump, 32 collision with bed,
  64 collision with the printed part

Collisions: with a robot loaded from a .robot file the check uses the surface of the link meshes (links 2-6,
see RobotFile.hulls): a link collides with the bed when any of its points is
over the print area and below the bed + floor_margin, and with the part when it is inside the part's bounding box
below the layers already printed. Skeleton-only models check the joint origins against the bed plane.
A tool head model (tool.py) is checked too: below the bed surface over the print area, or inside the part.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import kinematics as K
from .settings import parse_float_list
from .slicer import KIND_RETRACT

ERR_REACH, ERR_LIMIT, ERR_SING, ERR_CONFIG, ERR_COLLISION, ERR_PART = 1, 2, 4, 8, 32, 64
BATCH = True   # solve problem-free stretches with vectorised IK (False = move by move; same results, slower)
ERR_NAMES = {ERR_REACH: 'unreachable', ERR_LIMIT: 'joint limit', ERR_SING: 'wrist singularity',
             ERR_CONFIG: 'configuration change', ERR_COLLISION: 'collision with bed', ERR_PART: 'collision with part'}


@dataclass
class RobotPath:
    joints: np.ndarray        # (N, 6) deg, one per toolpath move
    rz: np.ndarray            # (N,) tool rotation about Z used for the move (deg)
    err: np.ndarray           # (N,) error bits
    config: tuple
    summary: dict = field(default_factory=dict)
    issues: list = field(default_factory=list)


def cell_frames(s):
    bed = K.xyzrpw_to_pose(parse_float_list(s['bed_frame']) + [0] * (6 - len(parse_float_list(s['bed_frame']))))
    tcp = K.xyzrpw_to_pose(parse_float_list(s['tcp']) + [0] * (6 - len(parse_float_list(s['tcp']))))
    return bed, tcp


def tcp_pose_bed(x, y, z, rz_deg, tilt=None):
    """Nozzle pose in the bed frame: at (x, y, z), tool Z pointing down (or tilted by `tilt`, 4x4, for angled
    layers), rotated rz about tool Z."""
    T = K.transl(x, y, z) if tilt is None else K.transl(x, y, z) @ tilt
    return T @ K.rotx(math.pi) @ K.rotz(math.radians(rz_deg))


class Solver:
    def __init__(self, robot, s):
        self.robot, self.s = robot, s
        self.bed, tcp = cell_frames(s)
        self.inv_tcp = K.inv_pose(tcp)
        self.inv_bed = K.inv_pose(self.bed)
        rng, step = s['rotz_range'], max(s['rotz_step'], 0.1)
        offs = [0.0]
        k = 1
        while k * step <= rng + 1e-9:
            offs += [k * step, -k * step]
            k += 1
        self.offsets = offs
        self.rz0 = s['tool_rz']
        self.part_box = None      # (xmin, ymin, xmax, ymax) of the flat layers, set by solve_robot_path
        self.part_box_a = None    # ... of the angled layers, in the slice frame (part_R, part_z0)
        self.part_R, self.part_z0 = np.eye(3), 0.0
        self.orient = None        # per move: 0 = vertical nozzle, 1 = tilted (set by solve_robot_path)
        self.tilts = [np.eye(4), np.eye(4)]   # nozzle rotation for orient 0 (vertical) and 1 (tilted with the layers)
        from . import tool
        self.tool_pts = tool.hull_flange(s)   # tool head model surface points in the flange frame (None = no model)

    def flange(self, x, y, z, rz, o=0):
        return self.bed @ tcp_pose_bed(x, y, z, rz, self.tilts[o] if o else None) @ self.inv_tcp

    def part_hit(self, p, printed, margin=0.0):
        """p (..., M, 3) bed-frame points; printed (..., 2) = top of the flat layers printed so far and top of the
        angled layers (slice frame), -inf = none. True where any point is inside the printed part's box below them."""
        printed = np.asarray(printed, dtype=float)
        if printed.ndim == 0:                     # a single height: flat layers only
            printed = np.array([float(printed), -np.inf])
        hit =np.zeros(p.shape[:-2], dtype=bool)
        x, y, z = p[..., 0], p[..., 1], p[..., 2]
        if self.part_box is not None:
            x0, y0, x1, y1 = self.part_box
            hit |= ((x >= x0) & (x <= x1) & (y >= y0) & (y <= y1) & (z < printed[..., 0, None] - margin)).any(axis=-1)
        if self.part_box_a is not None:
            q = p @ self.part_R
            x0, y0, x1, y1 = self.part_box_a
            hit |= ((q[..., 0] >= x0) & (q[..., 0] <= x1) & (q[..., 1] >= y0) & (q[..., 1] <= y1) &
                    (q[..., 2] - self.part_z0 < printed[..., 1, None] - margin)).any(axis=-1)
        return hit

    def problems(self, q, q_prev=None):
        """Error bits for a solved joint vector (0 = fine)."""
        if q is None:
            return ERR_REACH
        r, s = self.robot, self.s
        bits = 0
        if not r.within_limits(q):
            bits |= ERR_LIMIT
        if abs(q[4]) < s['sing_margin']:
            bits |= ERR_SING
        if q_prev is not None and np.abs(q - q_prev).max() > s['joint_jump_max']:
            bits |= ERR_CONFIG
        return bits

    def collision(self, q, printed_z=None):
        """Collision bits for joints q; printed_z = top of the layers already printed (None = nothing yet)."""
        mesh_pts = self.robot.link_points(q)
        tool_bits = self._tool_collision(q, printed_z) if self.tool_pts is not None else 0
        if mesh_pts is None:
            pts = self.robot.link_origins(q)[2:]                     # shoulder onwards (base sits on the floor)
            z_bed = (self.inv_bed[:3, :3] @ pts.T).T[:, 2] + self.inv_bed[2, 3]
            return (ERR_COLLISION if (z_bed < self.s['floor_margin']).any() else 0) | tool_bits
        p = (self.inv_bed[:3, :3] @ mesh_pts.T).T + self.inv_bed[:3, 3]
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        bits = 0
        over_bed = (x >= 0) & (x <= self.s['bed_x']) & (y >= 0) & (y <= self.s['bed_y'])
        if (over_bed & (z < self.s['floor_margin'])).any():
            bits |= ERR_COLLISION
        if printed_z is not None and self.part_hit(p, printed_z):
            bits |= ERR_PART
        return bits | tool_bits

    def _tool_collision(self, q, printed_z):
        """Tool head model vs the bed (any point below the bed surface over the print area) and the printed part."""
        T = self.inv_bed @ self.robot.fk(q)
        p = (T[:3, :3] @ self.tool_pts.T).T + T[:3, 3]
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        bits = 0
        if ((x >= 0) & (x <= self.s['bed_x']) & (y >= 0) & (y <= self.s['bed_y']) & (z < -0.5)).any():
            bits |= ERR_COLLISION
        if printed_z is not None and self.part_hit(p, printed_z, 0.5):
            bits |= ERR_PART
        return bits

    def solve_point(self, x, y, z, rz_pref, q_prev, config, o=0):
        """Best joints for one point: keep rz_pref if it works, else the nearest rz that does."""
        best = None
        cands = sorted(self.offsets, key=lambda o: abs(self.rz0 + o - rz_pref))
        for k, off in enumerate([None] + cands):
            rz = rz_pref if off is None else self.rz0 + off
            q = self.robot.ik_near(self.flange(x, y, z, rz, o), q_prev, config)
            bits = self.problems(q, q_prev)
            if bits == 0:
                return q, rz, 0
            if q is not None and (best is None or _severity(bits) < _severity(best[2])):
                best = (q, rz, bits)
        if best is None:
            return None, rz_pref, ERR_REACH
        return best

    def choose_start(self, x, y, z, q_pref, o=0):
        """Configuration and rotation for the first point: closest to the preferred joints."""
        best = None
        for off in self.offsets:
            rz = self.rz0 + off
            sols = self.robot.ik_all(self.flange(x, y, z, rz, o)) or []
            for q, flags in sols:
                q = self.robot.ik_near(self.flange(x, y, z, rz, o), q_pref, flags)
                if q is None:
                    continue
                bits = self.problems(q)
                margin = float(np.min(np.minimum(q - self.robot.lower, self.robot.upper - q)))
                score = (_severity(bits), float(np.abs(q - q_pref).sum()) - 0.2 * margin)
                if best is None or score < best[0]:
                    best = (score, q, rz, tuple(flags))
        return best


def _batch_moves(sv, tp, i0, i1, q_prev, rz, lock, step, printed_z_of):
    """Solve moves i0..i1-1 at once at tool rotation rz in configuration lock. Returns (joints of the leading moves
    that have no problem at all, the last joint vector reached), stopping before the first move with a problem."""
    robot, s = sv.robot, sv.s
    # check points in order: intermediate points on long moves (every `step` mm), then the move's end point
    xs, ys, zs, owner = [], [], [], []
    for i in range(i0, i1):
        x, y, z = float(tp.x[i]), float(tp.y[i]), float(tp.z[i])
        if i > 0 and int(tp.kind[i]) != KIND_RETRACT:
            px, py, pz = float(tp.x[i - 1]), float(tp.y[i - 1]), float(tp.z[i - 1])
            L = math.dist((px, py, pz), (x, y, z))
            if L > step:
                t = np.arange(step, L, step) / L
                xs.extend(px + (x - px) * t); ys.extend(py + (y - py) * t); zs.extend(pz + (z - pz) * t)
                owner.extend([i] * len(t))
        xs.append(x); ys.append(y); zs.append(z); owner.append(i)
    P = np.column_stack([xs, ys, zs])
    owner = np.array(owner)
    N = len(P)
    # flange poses: bed @ transl(p) @ tilt @ rotx(pi) @ rotz(rz) @ inv(tcp); one rotation per nozzle orientation
    o = sv.orient[owner] if sv.orient is not None else np.zeros(N, dtype=int)
    Ms = np.stack([sv.tilts[k] @ K.rotx(math.pi) @ K.rotz(math.radians(rz)) @ sv.inv_tcp for k in (0, 1)])[o]
    bed = sv.bed
    T = np.zeros((N, 4, 4))
    T[:, :3, :3] = bed[:3, :3] @ Ms[:, :3, :3]
    T[:, :3, 3] = (P + Ms[:, :3, 3]) @ bed[:3, :3].T + bed[:3, 3]
    T[:, 3, 3] = 1.0
    q_raw, ok = robot.ik_config_batch(T, lock)
    # nearest turn (+-360) to the previous joints, as ik_near does: k_i = k_(i-1) + round((raw_(i-1) - raw_i) / 360)
    k = np.vstack([np.round((q_prev - q_raw[0]) / 360.0)[None], np.round((q_raw[:-1] - q_raw[1:]) / 360.0)])
    Q = q_raw + 360.0 * np.cumsum(k, axis=0)
    bad = ~ok
    bad |= ~np.all((Q >= robot.lower) & (Q <= robot.upper), axis=1)     # out of limits: the exact path decides
    bad |= np.abs(Q[:, 4]) < s['sing_margin']
    prev = np.vstack([np.asarray(q_prev, dtype=float)[None], Q[:-1]])
    bad |= np.abs(Q - prev).max(axis=1) > s['joint_jump_max']
    # collisions (bed, printed part), only where nothing else failed yet
    good = np.nonzero(~bad)[0]
    if len(good):
        pz = np.array([(-np.inf, -np.inf) if printed_z_of(int(m)) is None else printed_z_of(int(m)) for m in owner[good]]).reshape(-1, 2)
        bad[good] |= _collides_batch(sv, Q[good], pz)
    first_bad = int(np.argmax(bad)) if bad.any() else N
    stop_move = int(owner[first_bad]) if first_bad < N else i1
    ends = np.nonzero(np.r_[owner[1:] != owner[:-1], True])[0]                     # end point of each move
    ends = ends[owner[ends] < stop_move]
    last = Q[ends[-1]] if len(ends) else np.asarray(q_prev, dtype=float)
    return Q[ends], last


def _collides_batch(sv, Q, printed_z):
    """Same tests as Solver.collision for many joint vectors; printed_z (N, 2), -inf where nothing is printed yet."""
    s, robot = sv.s, sv.robot
    inv = sv.inv_bed
    hit = np.zeros(len(Q), dtype=bool)
    pts = robot.link_points_batch(Q, frame=inv)          # already in the bed frame
    if pts is None:
        chain = robot.chain_batch(Q)
        org = np.stack([(chain[i] @ robot.L[i])[:, :3, 3] for i in range(1, robot.n)], axis=1)   # links 2..6 origins
        zb = org @ inv[:3, :3].T[:, 2] + inv[2, 3]
        hit |= (zb < s['floor_margin']).any(axis=1)
    else:
        x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
        over = (x >= 0) & (x <= s['bed_x']) & (y >= 0) & (y <= s['bed_y'])
        hit |= (over & (z < s['floor_margin'])).any(axis=1)
        hit |= sv.part_hit(pts, printed_z)
    if sv.tool_pts is not None:
        F = inv[None] @ robot.fk_batch(Q)
        p = np.matmul(sv.tool_pts[None], np.transpose(F[:, :3, :3], (0, 2, 1))) + F[:, None, :3, 3]
        x, y, z = p[..., 0], p[..., 1], p[..., 2]
        hit |= ((x >= 0) & (x <= s['bed_x']) & (y >= 0) & (y <= s['bed_y']) & (z < -0.5)).any(axis=1)
        hit |= sv.part_hit(p, printed_z, 0.5)
    return hit


def _severity(bits):
    return (1000 if bits & ERR_REACH else 0) + (100 if bits & ERR_LIMIT else 0) + (50 if bits & (ERR_COLLISION | ERR_PART) else 0) + \
           (10 if bits & ERR_SING else 0) + (5 if bits & ERR_CONFIG else 0)


def solve_robot_path(tp, s, robot, progress=None):
    progress = progress or (lambda stage, frac: None)
    sv = Solver(robot, s)
    n = len(tp)
    sv.orient = None
    if getattr(tp, 'orient', None) is not None and tp.orient.any():
        sv.orient = tp.orient.astype(int)
        sv.tilts = [np.eye(4), np.asarray(tp.tilt, dtype=float)]
    orient_of = (lambda i: int(sv.orient[i])) if sv.orient is not None else (lambda i: 0)
    joints = np.zeros((n, 6))
    rzs = np.zeros(n)
    err = np.zeros(n, dtype=np.int32)
    pref = parse_float_list(s['start_joints']) if s['start_joints'].strip() else None
    q_pref = np.array(pref if pref and len(pref) == 6 else robot.home, dtype=float)
    start = None
    for i0 in range(0, n, max(1, n // 300)):          # first point the robot can reach seeds the solver
        start = sv.choose_start(tp.x[i0], tp.y[i0], tp.z[i0], q_pref, orient_of(i0))
        if start is not None:
            break
    if start is None:
        raise RuntimeError('The robot cannot reach any point of this print. Check the bed frame and tool TCP.')
    _, q_prev, rz_cur, config = start
    if pref and s['lock_config']:
        cfg_pref = robot.config_of(q_pref)
        if cfg_pref is not None and cfg_pref != config:
            config = cfg_pref
            q_try = robot.ik_near(sv.flange(tp.x[0], tp.y[0], tp.z[0], rz_cur, orient_of(0)), q_pref, config)
            if q_try is not None:
                q_prev = q_try
    lock = config if s['lock_config'] else None
    step = s['check_step']
    ext = tp.e > np.concatenate([[0.0], tp.e[:-1]])
    nl = len(tp.layer_z)
    nb = nl if getattr(tp, 'flat_layers', None) is None else int(tp.flat_layers)   # flat layers come first
    flat = ext & (tp.layer < nb)
    if flat.any():
        sv.part_box = (float(tp.x[flat].min()), float(tp.y[flat].min()), float(tp.x[flat].max()), float(tp.y[flat].max()))
    ang = ext & (tp.layer >= nb)
    if ang.any() and getattr(tp, 'part_R', None) is not None:
        sv.part_R, sv.part_z0 = np.asarray(tp.part_R, dtype=float), float(tp.part_z0)
        q = np.column_stack([tp.x[ang], tp.y[ang], tp.z[ang]]) @ sv.part_R
        sv.part_box_a = (float(q[:, 0].min()), float(q[:, 1].min()), float(q[:, 0].max()), float(q[:, 1].max()))
    layer_top = [float(z) for z in tp.layer_z]
    base_top = layer_top[nb - 1] if nb > 0 else -np.inf

    def printed_z_of(i):
        """(top of the flat layers printed, top of the angled layers printed), or None before anything is printed."""
        li = int(tp.layer[i])
        if li < nb:
            return (layer_top[li - 1], -np.inf) if 0 < li <= nl else None
        if li == nb:
            return (base_top, -np.inf) if nb > 0 else None
        return (base_top, layer_top[li - 1]) if li <= nl else None

    def one_move(i, q_prev, rz_cur):
        """The exact per-move solver: intermediate checks, rotation search, problems, collisions."""
        x, y, z = float(tp.x[i]), float(tp.y[i]), float(tp.z[i])
        printed_z = printed_z_of(i)
        bits = 0
        if i > 0 and int(tp.kind[i]) != KIND_RETRACT:
            # intermediate checks on long moves (the robot moves linearly between points), so the
            # joint-jump test always compares poses at most check_step apart
            px, py, pz = float(tp.x[i - 1]), float(tp.y[i - 1]), float(tp.z[i - 1])
            L = math.dist((px, py, pz), (x, y, z))
            if L > step:
                for t in np.arange(step, L, step) / L:
                    qm = robot.ik_near(sv.flange(px + (x - px) * t, py + (y - py) * t, pz + (z - pz) * t, rz_cur, orient_of(i)), q_prev, lock)
                    b = sv.problems(qm, q_prev)
                    if qm is not None:
                        b |= sv.collision(qm, printed_z)
                        q_prev = qm
                    bits |= b
        q, rz, b = sv.solve_point(x, y, z, rz_cur, q_prev, lock, orient_of(i))
        bits |= b
        if q is None:
            q = q_prev
        else:
            bits |= sv.collision(q, printed_z)
            rz_cur = rz
        return q, rz_cur, bits

    # Fast path: whole stretches of moves are solved at once (vectorised closed-form IK in the locked configuration
    # at the current tool rotation). A stretch is accepted up to the first move with any problem; that move goes
    # through one_move, which may search other rotations, so results are the same as solving move by move.
    i = 0
    chunk = 256
    while i < n:
        if lock is not None and BATCH:
            k_ok, q_last = _batch_moves(sv, tp, i, min(n, i + chunk), q_prev, rz_cur, lock, step, printed_z_of)
            if len(k_ok):
                joints[i:i + len(k_ok)] = k_ok
                rzs[i:i + len(k_ok)] = rz_cur
                q_prev = q_last
                i += len(k_ok)
                chunk = min(chunk * 2, 8192) if i < n and len(k_ok) == chunk else 64
            else:
                chunk = 64
            if i >= n:
                break
        q, rz_cur, bits = one_move(i, q_prev, rz_cur)
        joints[i], rzs[i], err[i] = q, rz_cur, bits
        q_prev = q
        i += 1
        progress('robot', i / max(n, 1))
    progress('robot', 1.0)
    rp = RobotPath(joints, rzs, err, tuple(int(v) for v in config))
    summarize(rp, tp, s, robot, sv)
    return rp


def summarize(rp, tp, s, robot, sv):
    n = len(rp.err)
    bad = rp.err != 0
    counts = {name: int(np.count_nonzero(rp.err & bit)) for bit, name in ERR_NAMES.items()}
    used_lo, used_hi = rp.joints.min(axis=0), rp.joints.max(axis=0)
    margin_lo = used_lo - robot.lower
    margin_hi = robot.upper - used_hi
    issues = []
    last_key = None
    for i in np.nonzero(bad)[0]:
        key = (int(tp.layer[i]), int(rp.err[i]))
        if key == last_key:
            continue
        last_key = key
        issues.append({'move': int(i), 'layer': int(tp.layer[i]) + 1, 'x': round(float(tp.x[i]), 1), 'y': round(float(tp.y[i]), 1),
                       'z': round(float(tp.z[i]), 1), 'problems': [nm for bit, nm in ERR_NAMES.items() if rp.err[i] & bit]})
        if len(issues) >= 200:
            break
    rp.issues = issues
    rp.summary = {
        'ok': bool(not bad.any()),
        'moves': n,
        'valid_ratio': float(1 - bad.mean()) if n else 1.0,
        'counts': counts,
        'config': {'rear': rp.config[0], 'lower_elbow': rp.config[1], 'wrist_flip': rp.config[2]},
        'locked': bool(s['lock_config']),
        'joint_min': used_lo.round(2).tolist(), 'joint_max': used_hi.round(2).tolist(),
        'limit_lower': robot.lower.tolist(), 'limit_upper': robot.upper.tolist(),
        'limit_margin_min': np.minimum(margin_lo, margin_hi).round(2).tolist(),
        'j5_min_abs': float(np.abs(rp.joints[:, 4]).min()) if n else 0.0,
        'j6_travel': float(used_hi[5] - used_lo[5]),
        'rz_min': float(rp.rz.min()) if n else 0.0, 'rz_max': float(rp.rz.max()) if n else 0.0,
        'rz_changes': int(np.count_nonzero(np.abs(np.diff(rp.rz)) > 1e-9)),
        'start_joints': rp.joints[0].round(3).tolist() if n else [],
        'time_s': float(tp.stats.get('time_s', 0.0)),
        'issues_total': int(np.count_nonzero(bad)),
    }


def display_samples(rp, tp, s, robot, max_samples=3000):
    """Robot skeleton (base, link origins, TCP) in bed coordinates for playback, plus joint traces."""
    n = len(rp.err)
    if n == 0:
        return np.zeros((0, 9, 3), dtype=np.float32), np.zeros(0, dtype=np.int32)
    idx = np.unique(np.linspace(0, n - 1, min(n, max_samples)).astype(np.int64))
    bed, tcp = cell_frames(s)
    inv_bed = K.inv_pose(bed)
    out = np.zeros((len(idx), 9, 3), dtype=np.float32)
    for k, i in enumerate(idx):
        q = rp.joints[i]
        pts = robot.link_origins(q)                          # base + 6 link origins (robot base frame)
        tcp_pt = (robot.fk(q) @ tcp)[:3, 3]
        allp = np.vstack([pts, tcp_pt[None, :], np.array([[pts[0][0], pts[0][1], pts[0][2] - 0.0]])])
        allp_bed = (inv_bed[:3, :3] @ allp.T).T + inv_bed[:3, 3]
        out[k] = allp_bed[:9]
    return out, idx.astype(np.int32)


def sim_samples(rp, tp, max_samples=20000):
    """Time-based simulation track (the simulation slider): rows of
    [time (s) at which the robot reaches the move, J1..J6 (deg), error bits, move index] as float32.
    Every move is kept for small jobs; big jobs are sampled evenly in time and in move count."""
    n = len(rp.err)
    if n == 0:
        return np.zeros((0, 9), dtype=np.float32)
    seg = np.zeros(n)
    seg[1:] = np.sqrt(np.diff(tp.x) ** 2 + np.diff(tp.y) ** 2 + np.diff(tp.z) ** 2)
    t = np.cumsum(seg / np.maximum(tp.f, 1e-9))
    if n <= max_samples:
        idx = np.arange(n)
    else:
        by_time = np.searchsorted(t, np.linspace(0, t[-1], max_samples // 2))
        by_move = np.linspace(0, n - 1, max_samples // 2).astype(np.int64)
        idx = np.unique(np.concatenate([by_time.clip(0, n - 1), by_move, np.nonzero(rp.err)[0][:2000]]))
    out = np.zeros((len(idx), 9), dtype=np.float32)
    out[:, 0] = t[idx]
    out[:, 1:7] = rp.joints[idx]
    out[:, 7] = rp.err[idx]
    out[:, 8] = idx
    return out


def jog_ik(robot, s, q_now, pose_bed):
    """Joints that put the TCP at pose_bed ([x, y, z, rx, ry, rz] in the bed frame), staying in the
    configuration of q_now (nearest solution). None if unreachable."""
    bed, tcp = cell_frames(s)
    T = bed @ K.xyzrpw_to_pose(pose_bed) @ K.inv_pose(tcp)
    q_now = np.asarray(q_now, dtype=float)
    cfg = robot.config_of(q_now)
    q = robot.ik_near(T, q_now, cfg)
    if q is None:
        q = robot.ik_near(T, q_now)
    return q


def tcp_bed(robot, s, q):
    """TCP pose [x, y, z, rx, ry, rz] in the bed frame for joints q."""
    bed, tcp = cell_frames(s)
    return K.pose_to_xyzrpw(K.inv_pose(bed) @ robot.fk(np.asarray(q, dtype=float)) @ tcp)


def robot_points_bed(robot, s, q):
    """Robot skeleton for one joint vector, in bed coordinates."""
    bed, tcp = cell_frames(s)
    inv_bed = K.inv_pose(bed)
    pts = np.vstack([robot.link_origins(q), (robot.fk(q) @ tcp)[:3, 3][None, :]])
    return ((inv_bed[:3, :3] @ pts.T).T + inv_bed[:3, 3]).tolist()


def reach_map(robot, s, nx=25, ny=25, z=0.0):
    """Which bed points the nozzle can reach (any configuration, rotation searched), for a bed overlay."""
    sv = Solver(robot, s)
    xs = np.linspace(0, s['bed_x'], nx)
    ys = np.linspace(0, s['bed_y'], ny)
    grid = np.zeros((ny, nx), dtype=np.int8)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            ok = 0
            for off in sv.offsets[::2]:
                for q, _ in robot.ik_all(sv.flange(x, y, z, sv.rz0 + off)) or []:
                    if robot.within_limits(q) and abs(q[4]) >= s['sing_margin']:
                        ok = 1
                        break
                if ok:
                    break
            grid[j, i] = ok
    return {'nx': nx, 'ny': ny, 'x': xs.tolist(), 'y': ys.tolist(), 'ok': grid.tolist(), 'z': z}
