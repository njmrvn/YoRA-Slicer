"""Robot kinematics: a product-of-exponentials (PoE) model of a 6-axis serial robot.

The model is read from a .robot robot model file (yora/robot_file.py: kinematics and 3D link meshes) and saved as
JSON in profiles/robots/. Forward / inverse kinematics, limits, singularity and configuration checks all run here.

Model:  T_flange(q) = exp([S1] q1) ... exp([S6] q6) M        (all in the robot base frame, mm / rad)
        link frame i(q) = exp([S1] q1) ... exp([Si] qi) L_i(0)
"""
import json
import math
import os

import numpy as np

from .settings import PROJECT_DIR

ROBOTS_DIR = os.path.join(PROJECT_DIR, 'profiles', 'robots')


# ---------------- small SE(3) helpers ----------------
def rot_axis(w, theta):
    w = np.asarray(w, dtype=float)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def exp_twist(w, p, theta):
    """Rotation by theta about the axis with direction w through point p."""
    R = rot_axis(w, theta)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (np.eye(3) - R) @ p
    return T


def transl(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def xyzrpw_to_pose(v):
    """[x, y, z, rx, ry, rz] (deg): T = transl * rotx * roty * rotz."""
    x, y, z, a, b, c = (float(t) for t in v)
    return transl(x, y, z) @ rotx(math.radians(a)) @ roty(math.radians(b)) @ rotz(math.radians(c))


def pose_to_xyzrpw(T):
    """Inverse of xyzrpw_to_pose."""
    R = T[:3, :3]
    b = math.asin(max(-1.0, min(1.0, R[0, 2])))
    if abs(R[0, 2]) < 0.999999:
        a = math.atan2(-R[1, 2], R[2, 2])
        c = math.atan2(-R[0, 1], R[0, 0])
    else:
        a = math.atan2(R[2, 1], R[1, 1])
        c = 0.0
    return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), math.degrees(a), math.degrees(b), math.degrees(c)]


def inv_pose(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def pose_error(T, Tt):
    """6-vector (position mm, orientation rad) from T to target Tt, in the base frame."""
    dp = Tt[:3, 3] - T[:3, 3]
    Re = Tt[:3, :3] @ T[:3, :3].T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2)))
    if angle < 1e-9:
        dw = np.zeros(3)
    else:
        dw = angle / (2 * math.sin(angle)) * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return np.concatenate([dp, dw])


# ---------------- robot model ----------------
class Robot:
    def __init__(self, data):
        self.data = data
        self.name = data['name']
        self.w = np.array(data['axes'], dtype=float)          # (6, 3) joint axis directions at q = 0
        self.p = np.array(data['points'], dtype=float)        # (6, 3) a point on each axis at q = 0
        self.M = np.array(data['flange_home'], dtype=float)   # flange pose at q = 0
        self.L = [np.array(l, dtype=float) for l in data['links_home']]   # link frames 1..6 at q = 0
        self.lower = np.array(data['lower'], dtype=float)
        self.upper = np.array(data['upper'], dtype=float)
        self.home = np.array(data.get('home', [0] * 6), dtype=float)
        self.n = len(self.w)
        # link surface points in each link frame (index 0 = base), from a .robot file; None for skeleton-only models
        self.hull = [np.array(h, dtype=float).reshape(-1, 3) for h in data['hull']] if data.get('hull') else None

    def to_json(self):
        d = dict(self.data)
        d['lower'], d['upper'] = self.lower.tolist(), self.upper.tolist()
        return d

    # q in degrees everywhere in the public API
    def _chain(self, q):
        out, T = [], np.eye(4)
        for i in range(self.n):
            T = T @ exp_twist(self.w[i], self.p[i], math.radians(q[i]))
            out.append(T)
        return out

    def fk(self, q):
        """Flange pose in the robot base frame."""
        return self._chain(q)[-1] @ self.M

    def link_frames(self, q):
        """Link frames 1..n (each link's mesh frame) in the robot base frame."""
        chain = self._chain(q)
        return [chain[i] @ self.L[i] for i in range(self.n)]

    def link_points(self, q, first=2):
        """Hull points of links first..n in the robot base frame (None without meshes)."""
        if self.hull is None:
            return None
        F = self.link_frames(q)
        pts = [(F[i - 1][:3, :3] @ self.hull[i].T).T + F[i - 1][:3, 3] for i in range(first, self.n + 1) if len(self.hull[i])]
        return np.vstack(pts) if pts else None

    def link_origins(self, q):
        """Base origin + link frame origins (for drawing the arm)."""
        chain = self._chain(q)
        pts = [np.zeros(3)] + [(chain[i] @ self.L[i])[:3, 3] for i in range(self.n)]
        return np.array(pts)

    def jacobian(self, q):
        """Space Jacobian rows (v; w) for twists, columns per joint (per radian)."""
        J = np.zeros((6, self.n))
        T = np.eye(4)
        for i in range(self.n):
            R, t = T[:3, :3], T[:3, 3]
            w = R @ self.w[i]
            pnt = R @ self.p[i] + t
            J[:3, i] = np.cross(pnt, w)   # v = -w x p = p x w
            J[3:, i] = w
            T = T @ exp_twist(self.w[i], self.p[i], math.radians(q[i]))
        return J

    def ik(self, T_target, q_seed, iters=60, tol_mm=0.01, tol_rad=1e-5):
        """Damped least squares from q_seed (deg). Returns q (deg) or None. Keeps the seed's configuration."""
        q = np.array(q_seed, dtype=float)
        lam = 0.5
        for _ in range(iters):
            T = self.fk(q)
            e = pose_error(T, T_target)
            if np.linalg.norm(e[:3]) < tol_mm and np.linalg.norm(e[3:]) < tol_rad:
                return q
            J = self.jacobian(q)
            # Error as a spatial twist (velocity of the point at the base origin): v = dp - w x p
            twist = np.concatenate([e[:3] - np.cross(e[3:], T[:3, 3]), e[3:]])
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + (lam ** 2) * np.eye(6), twist)
            step = np.degrees(dq)
            m = np.abs(step).max()
            if m > 20:
                step *= 20 / m
            q = q + step
        return None

    # ---------- closed-form IK (J1 vertical, J2 || J3, spherical wrist) ----------
    def _prepare_analytic(self):
        if getattr(self, '_ana', None) is not None:
            return self._ana
        w, p = self.w, self.p
        ok = (abs(abs(w[0] @ np.array([0, 0, 1.0])) - 1) < 1e-6 and abs(abs(w[1] @ w[2]) - 1) < 1e-6
              and abs(abs(w[3] @ w[5]) - 1) < 1e-6 and abs(w[3] @ w[4]) < 1e-6)
        wc = None
        if ok:
            # wrist centre: intersection of the J4 and J5 axes
            A = np.column_stack([w[3], -w[4]])
            t, *_ = np.linalg.lstsq(A, p[4] - p[3], rcond=None)
            wc = p[3] + w[3] * t[0]
            ok = np.linalg.norm(wc - (p[4] + w[4] * t[1])) < 1e-6 and np.linalg.norm(np.cross(w[5], wc - p[5])) < 1e-6
        if not ok:
            self._ana = False
            return False
        c_f = (inv_pose(self.M) @ np.append(wc, 1.0))[:3]       # wrist centre in the flange frame (constant)
        P2 = p[1]
        P3 = P2 + (p[2] - P2) - w[1] * (w[1] @ (p[2] - P2))     # J3 axis point in the arm plane
        B = np.column_stack([w[3], w[4], np.cross(w[3], w[4])])   # wrist basis: x = J4 axis, y = J5 axis
        self._ana = {'wc': wc, 'c_f': c_f, 'P2': P2, 'P3': P3, 'B': B, 'sign6': float(np.sign(w[5] @ w[3])),
                     'sign1': float(np.sign(w[0][2]))}
        return self._ana

    @staticmethod
    def _signed_angle(a, b, axis):
        a = a - axis * (axis @ a)
        b = b - axis * (axis @ b)
        return math.atan2(axis @ np.cross(a, b), a @ b)

    def fk_ok(self, q, T_flange):
        Tq = self.fk(q)
        return np.linalg.norm(Tq[:3, 3] - T_flange[:3, 3]) < 0.01 and np.abs(Tq[:3, :3] - T_flange[:3, :3]).max() < 1e-5

    def ik_all(self, T_flange, validate=True):
        """All closed-form solutions (deg, unwrapped to (-180, 180]) as a list of (q, (rear, lower, flip))."""
        a = self._prepare_analytic()
        if not a:
            return None
        pc = (T_flange @ np.append(a['c_f'], 1.0))[:3]
        wc0, P2, P3 = a['wc'], a['P2'], a['P3']
        sols = []
        base_ang = math.atan2(pc[1], pc[0]) - math.atan2(wc0[1], wc0[0])
        for rear in (0, 1):
            q1 = (base_ang + (math.pi if rear else 0.0)) * a['sign1']
            R1 = rot_axis(self.w[0], q1)
            pcl = R1.T @ pc                           # target wrist centre with J1 undone
            d23 = np.linalg.norm(P3 - P2)
            v3w0 = wc0 - P3
            d3w = np.linalg.norm(v3w0)
            D = np.linalg.norm(pcl - P2)
            c = (d23 ** 2 + d3w ** 2 - D ** 2) / (2 * d23 * d3w)
            if abs(c) > 1 + 1e-9:
                continue
            gamma = math.acos(max(-1.0, min(1.0, c)))           # elbow angle needed
            gamma0 = self._signed_angle(P2 - P3, v3w0, self.w[2])  # signed elbow angle at q3 = 0
            seen = []
            for q3 in (gamma - gamma0, -gamma - gamma0):
                q3 = (q3 + math.pi) % (2 * math.pi) - math.pi
                if any(abs(q3 - s) < 1e-9 for s in seen):
                    continue
                seen.append(q3)
                W = P3 + rot_axis(self.w[2], q3) @ v3w0      # wrist centre with q2 = 0
                if abs(np.linalg.norm(W - P2) - D) > 1e-6 * max(D, 1):
                    continue
                lower = 1 if self._signed_angle(P2 - P3, W - P3, self.w[2]) < 0 else 0
                q2 = self._signed_angle(W - P2, pcl - P2, self.w[1])
                R03 = R1 @ rot_axis(self.w[1], q2) @ rot_axis(self.w[2], q3)
                Rw = a['B'].T @ R03.T @ T_flange[:3, :3] @ self.M[:3, :3].T @ a['B']   # = Rx(q4) Ry(q5) Rx(+-q6)
                b = math.acos(max(-1.0, min(1.0, Rw[0, 0])))
                for flip in (0, 1):
                    q5 = b if flip == 0 else -b
                    if abs(math.sin(q5)) < 1e-9:
                        q4 = 0.0
                        q6 = math.atan2(Rw[2, 1], Rw[1, 1])
                    else:
                        s5 = math.sin(q5)
                        q4 = math.atan2(Rw[1, 0] / s5, -Rw[2, 0] / s5)
                        q6 = math.atan2(Rw[0, 1] / s5, Rw[0, 2] / s5)
                    q6 *= a['sign6']
                    q = np.degrees([q1, q2, q3, q4, q5, q6])
                    q = (q + 180.0) % 360.0 - 180.0
                    sols.append((q, (rear, lower, 1 if q[4] < 0 else 0)))
        if not validate:
            return sols
        # keep only solutions that reproduce the target (guards every sign convention above)
        return [(q, flags) for q, flags in sols if self.fk_ok(q, T_flange)]

    def ik_near(self, T_flange, q_ref, config=None):
        """Closed-form solution closest to q_ref (J4/J6 unwrapped by +-360 toward q_ref and kept within limits).
        With config=(rear, lower, flip) only that configuration is accepted. Returns q (deg) or None."""
        sols = self.ik_all(T_flange, validate=False)
        if sols is None:
            return self.ik(T_flange, q_ref)
        q_ref = np.asarray(q_ref, dtype=float)
        ranked = []
        for q, flags in sols:
            if config is not None and tuple(flags) != tuple(config):
                continue
            q = q.copy()
            for j in range(6):   # pick the turn (+-360) closest to the reference, inside the limits if possible
                k = round((q_ref[j] - q[j]) / 360.0)
                for cand in (q[j] + 360.0 * k, q[j] + 360.0 * (k - 1), q[j] + 360.0 * (k + 1)):
                    if self.lower[j] - 1e-6 <= cand <= self.upper[j] + 1e-6 and abs(cand - q_ref[j]) <= abs(q[j] - q_ref[j]) + 1e-9:
                        q[j] = cand
            ranked.append((float(np.abs(q - q_ref).max()), q))
        for _, q in sorted(ranked, key=lambda t: t[0]):
            if self.fk_ok(q, T_flange):
                return q
        return None

    # ---------- batched versions (numpy over many poses at once) ----------
    @staticmethod
    def _rot_batch(w, th):
        """Rotations about the fixed axis w by angles th (N,) -> (N,3,3)."""
        K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        s, c = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
        return np.eye(3)[None] + s * K[None] + (1 - c) * (K @ K)[None]

    def chain_batch(self, Q):
        """Q (N,6) deg -> list of 6 cumulative transforms (N,4,4), like _chain."""
        Q = np.radians(np.asarray(Q, dtype=float))
        N = len(Q)
        T = np.broadcast_to(np.eye(4), (N, 4, 4)).copy()
        out = []
        for i in range(self.n):
            R = self._rot_batch(self.w[i], Q[:, i])
            E = np.zeros((N, 4, 4))
            E[:, :3, :3] = R
            E[:, :3, 3] = self.p[i] - R @ self.p[i]
            E[:, 3, 3] = 1.0
            T = T @ E
            out.append(T)
        return out

    def fk_batch(self, Q):
        return self.chain_batch(Q)[-1] @ self.M

    def link_points_batch(self, Q, first=2, frame=None):
        """Hull points of links first..n for many joint vectors: (N, P, 3), or None without meshes. With frame (4x4)
        the points are returned in that frame (frame @ base coordinates)."""
        if self.hull is None:
            return None
        chain = self.chain_batch(Q)
        parts = []
        for i in range(first, self.n + 1):
            H = self.hull[i]
            if not len(H):
                continue
            F = chain[i - 1] @ self.L[i - 1]
            if frame is not None:
                F = frame[None] @ F
            parts.append(np.matmul(H[None], np.transpose(F[:, :3, :3], (0, 2, 1))) + F[:, None, :3, 3])
        return np.concatenate(parts, axis=1) if parts else None

    def ik_config_batch(self, T_flange, flags):
        """Closed-form IK of many flange poses (N,4,4) in one configuration flags = (rear, lower, flip), the same
        maths as ik_all. Returns q (N,6) deg in (-180, 180] and ok (N,) = reachable in that configuration and
        reproducing the pose (the fk_ok test)."""
        a = self._prepare_analytic()
        T = np.asarray(T_flange, dtype=float)
        N = len(T)
        q = np.zeros((N, 6))
        if not a or N == 0:
            return q, np.zeros(N, dtype=bool)
        rear, lower_want, flip = (int(v) for v in flags)
        w = self.w
        pc = T[:, :3, :3] @ a['c_f'] + T[:, :3, 3]
        wc0, P2, P3 = a['wc'], a['P2'], a['P3']
        q1 = (np.arctan2(pc[:, 1], pc[:, 0]) - math.atan2(wc0[1], wc0[0]) + (math.pi if rear else 0.0)) * a['sign1']
        R1 = self._rot_batch(w[0], q1)
        pcl = np.einsum('nji,nj->ni', R1, pc)                       # R1^T pc
        d23 = np.linalg.norm(P3 - P2)
        v3w0 = wc0 - P3
        d3w = np.linalg.norm(v3w0)
        D = np.linalg.norm(pcl - P2, axis=1)
        c = (d23 ** 2 + d3w ** 2 - D ** 2) / (2 * d23 * d3w)
        ok = np.abs(c) <= 1 + 1e-9
        gamma = np.arccos(np.clip(c, -1.0, 1.0))
        gamma0 = self._signed_angle(P2 - P3, v3w0, w[2])

        def signed(u, v, axis):                                    # batched _signed_angle
            u = u - np.outer(u @ axis, axis)
            v = v - np.outer(v @ axis, axis)
            return np.arctan2(np.cross(u, v) @ axis, (u * v).sum(1))

        best_q3 = np.full(N, np.nan)
        for sgn in (1.0, -1.0):                                     # the two elbow solutions; keep the wanted one
            q3 = (sgn * gamma - gamma0 + math.pi) % (2 * math.pi) - math.pi
            W = P3 + self._rot_batch(w[2], q3) @ v3w0
            lower = (signed(np.broadcast_to(P2 - P3, W.shape), W - P3, w[2]) < 0).astype(int)
            take = ok & (lower == lower_want) & np.isnan(best_q3)
            best_q3[take] = q3[take]
        ok &= ~np.isnan(best_q3)
        q3 = np.nan_to_num(best_q3)
        W = P3 + self._rot_batch(w[2], q3) @ v3w0
        q2 = signed(W - P2, pcl - P2, w[1])
        R03 = R1 @ self._rot_batch(w[1], q2) @ self._rot_batch(w[2], q3)
        B = a['B']
        Rw = B.T[None] @ np.transpose(R03, (0, 2, 1)) @ T[:, :3, :3] @ (self.M[:3, :3].T @ B)[None]
        b = np.arccos(np.clip(Rw[:, 0, 0], -1.0, 1.0))
        q5 = b if flip == 0 else -b
        s5 = np.sin(q5)
        tiny = np.abs(s5) < 1e-9
        s5s = np.where(tiny, 1.0, s5)
        q4 = np.where(tiny, 0.0, np.arctan2(Rw[:, 1, 0] / s5s, -Rw[:, 2, 0] / s5s))
        q6 = np.where(tiny, np.arctan2(Rw[:, 2, 1], Rw[:, 1, 1]), np.arctan2(Rw[:, 0, 1] / s5s, Rw[:, 0, 2] / s5s)) * a['sign6']
        q = np.degrees(np.column_stack([q1, q2, q3, q4, q5, q6]))
        q = (q + 180.0) % 360.0 - 180.0
        ok &= ((q[:, 4] < 0).astype(int) == flip)                   # flip flag as ik_all defines it (sign of q5)
        F = self.fk_batch(q)
        ok &= (np.linalg.norm(F[:, :3, 3] - T[:, :3, 3], axis=1) < 0.01) & (np.abs(F[:, :3, :3] - T[:, :3, :3]).max(axis=(1, 2)) < 1e-5)
        return q, ok

    def config_of(self, q):
        """(rear, lower, flip) flags of a joint vector, matching ik_all."""
        T = self.fk(q)
        for s, flags in self.ik_all(T, validate=False) or []:
            d = (np.asarray(q) - s + 180.0) % 360.0 - 180.0
            if np.abs(d).max() < 1e-3:
                return flags
        return None

    def within_limits(self, q, margin=0.0):
        return bool(np.all(q >= self.lower + margin) and np.all(q <= self.upper - margin))

    def manipulability(self, q):
        J = self.jacobian(q)
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


# ---------------- storage ----------------
def list_robots():
    if not os.path.isdir(ROBOTS_DIR):
        return []
    return sorted(os.path.splitext(f)[0] for f in os.listdir(ROBOTS_DIR) if f.endswith('.json'))


def load_robot(name):
    with open(os.path.join(ROBOTS_DIR, name + '.json'), 'r', encoding='utf-8') as f:
        return Robot(json.load(f))


def save_robot(robot):
    os.makedirs(ROBOTS_DIR, exist_ok=True)
    path = os.path.join(ROBOTS_DIR, robot.name + '.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(robot.to_json(), f, indent=1)
    return path


def mesh_path(name):
    return os.path.join(ROBOTS_DIR, name + '.mesh.bin')


def import_robot_file(path):
    """Load a .robot robot model file: kinematics (DHM table) and 3D link meshes.
    Joint limits already edited for a robot of the same name are kept. Returns the Robot."""
    from .robot_file import RobotFile
    rf = RobotFile(path)
    if rf.n != 6:
        raise ValueError('Only 6-axis robots are supported ("%s" has %i axes)' % (rf.name, rf.n))
    data = rf.to_model()
    old = os.path.join(ROBOTS_DIR, data['name'] + '.json')
    if os.path.isfile(old):
        with open(old, 'r', encoding='utf-8') as f:
            prev = json.load(f)
        data['lower'], data['upper'] = prev['lower'], prev['upper']
        if any(abs(v) > 1e-9 for v in prev.get('home', [])):            # keep a home that was set; else the file's
            data['home'] = prev['home']
    robot = Robot(data)
    save_robot(robot)
    with open(mesh_path(robot.name), 'wb') as f:
        f.write(rf.mesh_blob())
    return robot
