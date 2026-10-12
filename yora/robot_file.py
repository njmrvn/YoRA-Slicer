"""Read .robot robot model files (robot kinematics + 3D link meshes).

File layout (tests/test_robot_file.py checks it):

    file   = uint32 big-endian size  +  zlib stream                (Qt qCompress)
    body   = header: 4000 little-endian float64 (32000 bytes)
             int64 n_links                                          (base + one per joint)
             per link:  int64 n_meshes
                        per mesh: int64 n_tri, float32 rgba[4], n_tri*3 vertices of (float32 x,y,z + 2 bytes)
                        int64 0, int64 0                            (two empty lists)

    header (double index):   21 n_axes          600.. 6x20 DHM rows [alpha(rad), a, theta(rad), d] (stride 20)
                             64.. joint senses (-1 = inverted, 0/1 = normal)
                             600.. lower limits (deg), 620.. upper limits (deg)
                             1800.. robot name, one char per double;  3800.. post processor name

Kinematics are Craig's modified DH ("DHM"): T(i-1, i) = Rx(alpha) Tx(a) Rz(theta + sense*q) Tz(d).
Link meshes are in each link's own DHM frame (mm); link 0 is the base.
"""
import json
import math
import os
import struct
import zlib

import numpy as np

HEADER_DOUBLES = 4000
I_NAXES = 21
I_SENSE = 64
I_DHM = 200
DHM_STRIDE = 20
I_LOWER = 600
I_UPPER = 620
I_NAME = 1800
I_POST = 3800
I_HOME = 680         # home joints (deg), e.g. 0,-90,90,0,0,0 on KUKA arms (700.. = the pose when the file was saved)


def q_uncompress(raw):
    n = struct.unpack('>I', raw[:4])[0]
    body = zlib.decompress(raw[4:])
    if len(body) != n:
        raise ValueError('Not a .robot robot model file (size %i != %i)' % (len(body), n))
    return body


def _text(h, i, n=80):
    out = []
    for v in h[i:i + n]:
        if v <= 0 or v > 0x10FFFF or v != int(v):
            break
        out.append(chr(int(v)))
    return ''.join(out)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _tr(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


class RobotFile:
    """A parsed .robot file: name, post, DHM table, joint senses and limits, and per-link meshes."""

    def __init__(self, path):
        self.path = path
        with open(path, 'rb') as f:
            body = q_uncompress(f.read())
        h = struct.unpack_from('<%id' % HEADER_DOUBLES, body, 0)
        self.n = int(h[I_NAXES])
        self.name = _text(h, I_NAME) or os.path.splitext(os.path.basename(path))[0]
        self.post = _text(h, I_POST)
        self.dhm = np.array([h[I_DHM + DHM_STRIDE * i: I_DHM + DHM_STRIDE * i + 4] for i in range(self.n)], dtype=float)
        self.sense = np.array([-1.0 if h[I_SENSE + i] < 0 else 1.0 for i in range(self.n)])
        self.lower = np.array(h[I_LOWER:I_LOWER + self.n], dtype=float)
        self.upper = np.array(h[I_UPPER:I_UPPER + self.n], dtype=float)
        self.home = np.array(h[I_HOME:I_HOME + self.n], dtype=float) + 0.0
        self.links = self._read_meshes(body, HEADER_DOUBLES * 8)

    @staticmethod
    def _read_meshes(body, off):
        """List per link of [(rgba, float32 array (n_tri*3, 3))]."""
        n_links = struct.unpack_from('<q', body, off)[0]
        off += 8
        vdt = np.dtype([('p', '<f4', 3), ('x', 'V2')])
        links = []
        for _ in range(n_links):
            n_mesh = struct.unpack_from('<q', body, off)[0]
            off += 8
            meshes = []
            for _ in range(n_mesh):
                n_tri = struct.unpack_from('<q', body, off)[0]
                rgba = struct.unpack_from('<4f', body, off + 8)
                off += 24
                v = np.frombuffer(body, vdt, n_tri * 3, off)['p']
                off += n_tri * 42
                meshes.append((tuple(round(c, 4) for c in rgba), v))
            off += 16   # two empty lists
            links.append(meshes)
        if off != len(body):
            raise ValueError('Unexpected .robot layout (%i bytes left)' % (len(body) - off))
        return links

    # ---------------- kinematics ----------------
    def frames(self, q_deg):
        """Base-relative DHM frames 1..n for joints in degrees."""
        T, out = np.eye(4), []
        for i in range(self.n):
            alpha, a, theta, d = self.dhm[i]
            T = T @ _rx(alpha) @ _tr(a, 0, 0) @ _rz(theta + self.sense[i] * math.radians(q_deg[i])) @ _tr(0, 0, d)
            out.append(T)
        return out

    def to_model(self):
        """Data for kinematics.Robot (product of exponentials), so the existing IK and checks run unchanged."""
        L0 = self.frames([0.0] * self.n)
        axes = [(self.sense[i] * L0[i][:3, 2]).tolist() for i in range(self.n)]
        points = [L0[i][:3, 3].tolist() for i in range(self.n)]
        return {'name': safe_name(self.name), 'source': 'Robot file: ' + os.path.basename(self.path),
                'axes': axes, 'points': points, 'flange_home': L0[-1].tolist(),
                'links_home': [L.tolist() for L in L0], 'lower': self.lower.tolist(), 'upper': self.upper.tolist(),
                'home': self.home.tolist(), 'post': self.post, 'hull': self.hulls()}

    def hulls(self, n_dirs=150):
        """Per link, a few hundred surface points that bound the link (its support points in n_dirs
        directions, so the lowest point of the link in any pose is one of them). Used for collision checks."""
        g = math.pi * (3 - math.sqrt(5))
        k = np.arange(n_dirs)
        z = 1 - 2 * (k + 0.5) / n_dirs
        r = np.sqrt(1 - z * z)
        dirs = np.stack([r * np.cos(g * k), r * np.sin(g * k), z], axis=1)
        out = []
        for meshes in self.links:
            if not meshes:
                out.append([])
                continue
            v = np.concatenate([m for _, m in meshes]).astype(float)
            idx = np.unique(np.argmax(v @ dirs.T, axis=0))
            out.append(np.round(v[idx], 2).tolist())
        return out

    # ---------------- meshes for the 3D view ----------------
    def mesh_blob(self, max_tris=1_500_000):
        """Binary for web/robot.js: uint32 json length, JSON header, float32 vertices.

        Meshes of one link with the same colour are merged, so a link draws in a few calls. Robots with more than
        max_tris triangles are simplified by vertex clustering (nearby points merged, surfaces stay closed), never
        by dropping triangles, which would leave the model full of holes."""
        total = sum(len(v) for meshes in self.links for _, v in meshes) // 3
        cell = 0.0
        if total > max_tris:
            cell = 0.25
            while sum(len(cluster_triangles(v, cell)) for meshes in self.links for _, v in meshes) > max_tris and cell < 50:
                cell *= 1.6
        head, chunks, start = {'name': self.name, 'links': [], 'simplified_mm': round(cell, 3), 'decimated': 1}, [], 0
        for meshes in self.links:
            by_color = {}
            for rgba, v in meshes:
                by_color.setdefault(rgba, []).append(v)
            groups = []
            for rgba, vs in by_color.items():
                v = np.concatenate(vs).reshape(-1, 3, 3)
                if cell > 0:
                    v = cluster_triangles(v.reshape(-1, 3), cell)
                v = np.ascontiguousarray(v.reshape(-1, 3), dtype='<f4')
                groups.append({'color': rgba, 'start': start, 'count': len(v)})
                chunks.append(v.tobytes())
                start += len(v)
            head['links'].append(groups)
        js = json.dumps(head).encode()
        js += b' ' * (-len(js) % 4)
        return struct.pack('<I', len(js)) + js + b''.join(chunks)


def cluster_triangles(verts, cell):
    """Vertex-clustering simplification of a triangle soup (verts (n*3, 3)): points are snapped to the centre of
    their `cell` mm grid cell, triangles that collapse are dropped. Returns (m, 3, 3) triangles."""
    v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    if cell <= 0 or len(v) == 0:
        return v.reshape(-1, 3, 3)
    q = np.floor(v / cell).astype(np.int64)
    _, inv, cnt = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    centre = np.zeros((len(cnt), 3))
    np.add.at(centre, inv, v)
    centre /= cnt[:, None]                          # mean of the points in each cell (keeps the surface in place)
    t = inv.reshape(-1, 3)
    keep = (t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])
    t = t[keep]
    t = np.unique(np.sort(t, axis=1), axis=0, return_index=True)[1]
    t = inv.reshape(-1, 3)[keep][np.sort(t)]        # drop duplicate triangles, keep their original winding
    return centre[t]


def safe_name(name):
    return ''.join(c if c.isalnum() or c in ' _-' else '_' for c in name).strip()


def peek(path):
    """Name and axis count from the header only (for the robot library list)."""
    with open(path, 'rb') as f:
        raw = f.read(4)
        d = zlib.decompressobj()
        head = b''
        while len(head) < HEADER_DOUBLES * 8:
            chunk = f.read(65536)
            if not chunk:
                break
            head += d.decompress(chunk)
    h = struct.unpack_from('<%id' % HEADER_DOUBLES, head, 0)
    return {'name': _text(h, I_NAME) or os.path.splitext(os.path.basename(path))[0], 'axes': int(h[I_NAXES]), 'post': _text(h, I_POST)}


def find_robot_files(extra_dirs=()):
    """.robot files in the user's Downloads/Documents/Desktop and extra_dirs."""
    home = os.path.expanduser('~')
    roots = [(os.path.join(home, d), 3) for d in ('Downloads', 'Desktop', 'Documents')] + \
            [(d, 2) for d in extra_dirs]
    seen, out = set(), []
    for root, depth in roots:
        if not os.path.isdir(root):
            continue
        base = root.rstrip('\\/').count(os.sep)
        for dirpath, dirnames, files in os.walk(root):
            if dirpath.count(os.sep) - base >= depth:
                dirnames[:] = []
            for fn in files:
                if fn.lower().endswith('.robot'):
                    p = os.path.normpath(os.path.join(dirpath, fn))
                    if p.lower() in seen:
                        continue
                    seen.add(p.lower())
                    try:
                        out.append(dict(peek(p), path=p))
                    except Exception:   # not a robot model file
                        pass
    return sorted(out, key=lambda r: (r['axes'] != 6, r['name'].lower()))
