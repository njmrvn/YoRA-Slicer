"""Load STL / OBJ / 3MF into an (n, 3, 3) float64 triangle array, and place it on the bed."""
import io
import math
import os
import re
import struct
import zipfile
import xml.etree.ElementTree as ET

import numpy as np


def load_mesh(data, filename):
    ext = os.path.splitext(filename)[1].lower()
    if ext == '.stl':
        tris = _load_stl(data)
    elif ext == '.obj':
        tris = _load_obj(data.decode('utf-8', errors='replace'))
    elif ext == '.3mf':
        tris = _load_3mf(data)
    else:
        raise ValueError('Unsupported model format: %s (use STL, OBJ or 3MF)' % ext)
    tris = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    tris = tris[np.isfinite(tris).all(axis=(1, 2))]
    if len(tris) == 0:
        raise ValueError('The model has no triangles')
    return tris


def _load_stl(data):
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
            return rec['v'].astype(np.float64)
    text = data.decode('utf-8', errors='replace')
    nums = re.findall(r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', text)
    if not nums:
        raise ValueError('Could not read STL (neither binary nor ASCII)')
    return np.array(nums, dtype=np.float64).reshape(-1, 3, 3)


def _load_obj(text):
    verts, tris = [], []
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'v' and len(parts) >= 4:
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == 'f' and len(parts) >= 4:
            idx = []
            for p in parts[1:]:
                i = int(p.split('/')[0])
                idx.append(i - 1 if i > 0 else len(verts) + i)
            for k in range(1, len(idx) - 1):  # fan triangulation
                tris.append((idx[0], idx[k], idx[k + 1]))
    v = np.array(verts, dtype=np.float64)
    return v[np.array(tris, dtype=np.int64)]


def _load_3mf(data):
    z = zipfile.ZipFile(io.BytesIO(data))
    model_names = [n for n in z.namelist() if n.lower().endswith('.model')]
    if not model_names:
        raise ValueError('No 3D model inside the 3MF file')
    meshes = {}   # object id -> (tris or None, components)
    build = []
    for name in model_names:
        root = ET.fromstring(z.read(name))
        for el in root.iter():
            tag = el.tag.split('}')[-1]
            if tag == 'object':
                oid = el.get('id')
                tris, comps = None, []
                for sub in el.iter():
                    st = sub.tag.split('}')[-1]
                    if st == 'mesh':
                        tris = _3mf_mesh(sub)
                    elif st == 'component':
                        comps.append((sub.get('objectid'), _3mf_matrix(sub.get('transform'))))
                meshes[oid] = (tris, comps)
            elif tag == 'item':
                build.append((el.get('objectid'), _3mf_matrix(el.get('transform'))))

    def resolve(oid, m, depth=0):
        tris, comps = meshes.get(oid, (None, []))
        out = []
        if tris is not None:
            out.append(_apply(tris, m))
        if depth < 8:
            for cid, cm in comps:
                out.extend(resolve(cid, m @ cm, depth + 1))
        return out

    parts = []
    for oid, m in (build or [(k, np.eye(4)) for k in meshes]):
        parts.extend(resolve(oid, m))
    if not parts:
        raise ValueError('The 3MF file has no mesh')
    return np.concatenate(parts)


def _3mf_mesh(mesh_el):
    verts, tris = [], []
    for el in mesh_el.iter():
        tag = el.tag.split('}')[-1]
        if tag == 'vertex':
            verts.append([float(el.get('x')), float(el.get('y')), float(el.get('z'))])
        elif tag == 'triangle':
            tris.append([int(el.get('v1')), int(el.get('v2')), int(el.get('v3'))])
    return np.array(verts)[np.array(tris, dtype=np.int64)]


def _3mf_matrix(text):
    m = np.eye(4)
    if text:
        v = [float(x) for x in text.split()]
        if len(v) == 12:  # 3MF stores the 3x4 matrix column-major by rows of 3
            m[:3, :3] = np.array(v[:9]).reshape(3, 3).T
            m[:3, 3] = v[9:12]
    return m


def _apply(tris, m):
    flat = tris.reshape(-1, 3)
    return (flat @ m[:3, :3].T + m[:3, 3]).reshape(-1, 3, 3)


def rotation_matrix(rx, ry, rz):
    """Rotation about X, then Y, then Z (degrees, fixed axes)."""
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def place(tris, scale=(100.0, 100.0, 100.0), rotation=(0.0, 0.0, 0.0), center_xy=(0.0, 0.0)):
    """Scale (%) and rotate about the model centre, then drop onto z=0 centred at center_xy."""
    flat = tris.reshape(-1, 3)
    c = (flat.min(axis=0) + flat.max(axis=0)) / 2
    m = rotation_matrix(*rotation) @ np.diag(np.asarray(scale, dtype=np.float64) / 100.0)
    out = (flat - c) @ m.T
    lo, hi = out.min(axis=0), out.max(axis=0)
    out[:, 0] += center_xy[0] - (lo[0] + hi[0]) / 2
    out[:, 1] += center_xy[1] - (lo[1] + hi[1]) / 2
    out[:, 2] -= lo[2]
    if np.linalg.det(m) < 0:  # mirrored: keep outward normals
        out = out.reshape(-1, 3, 3)[:, ::-1, :].reshape(-1, 3)
    return out.reshape(-1, 3, 3)


def bounds(tris):
    flat = tris.reshape(-1, 3)
    return flat.min(axis=0), flat.max(axis=0)


def box_mesh(sx, sy, sz):
    """Closed box with outward normals, corner at the origin (used by tests and samples)."""
    v = np.array([[0, 0, 0], [sx, 0, 0], [sx, sy, 0], [0, sy, 0],
                  [0, 0, sz], [sx, 0, sz], [sx, sy, sz], [0, sy, sz]], dtype=np.float64)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def tube_mesh(r_out, r_in, height, segments=96):
    """Closed tube (cylinder with a hole), centred on the origin, outward normals."""
    a = np.linspace(0, 2 * math.pi, segments, endpoint=False)
    ca, sa = np.cos(a), np.sin(a)
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        o0b, o1b = (r_out * ca[i], r_out * sa[i], 0), (r_out * ca[j], r_out * sa[j], 0)
        o0t, o1t = (o0b[0], o0b[1], height), (o1b[0], o1b[1], height)
        i0b, i1b = (r_in * ca[i], r_in * sa[i], 0), (r_in * ca[j], r_in * sa[j], 0)
        i0t, i1t = (i0b[0], i0b[1], height), (i1b[0], i1b[1], height)
        tris += [(o0b, o1b, o1t), (o0b, o1t, o0t)]          # outer wall
        tris += [(i1b, i0b, i0t), (i1b, i0t, i1t)]          # inner wall (faces the hole)
        tris += [(o0t, o1t, i1t), (o0t, i1t, i0t)]          # top ring
        tris += [(o1b, o0b, i0b), (o1b, i0b, i1b)]          # bottom ring
    return np.array(tris, dtype=np.float64)


def to_stl_bytes(tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'] = n
    rec['v'] = tris
    return b'YoRA binary STL'.ljust(80, b' ') + struct.pack('<I', len(tris)) + rec.tobytes()
