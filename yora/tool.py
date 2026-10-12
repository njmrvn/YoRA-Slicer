"""Tool head 3D model (STL / OBJ / 3MF) on the robot flange.

The model is stored in profiles/tools/<name>.stl. Settings place it on the flange:
    tool_model        name of the stored model ('' = none: the view draws a simple cone to the TCP)
    tool_model_pose   X,Y,Z,Rx,Ry,Rz of the model's origin in the flange frame (0 if the CAD was exported
                      with the flange centre as origin and Z pointing out of the flange)
    tool_model_scale  % (1000 for a model exported in cm, 100000 for metres)

find_nozzle_tip() proposes the TCP: the tip of the narrow end of the model's long axis, with tool Z pointing out
of the nozzle. The model's surface points are used in the robot collision check (robotpath.Solver).
"""
import functools
import math
import os
import re

import numpy as np

from . import kinematics as K
from . import mesh
from .settings import PROFILE_DIR, parse_float_list

TOOLS_DIR = os.path.join(PROFILE_DIR, 'tools')


def safe_name(name):
    return re.sub(r'[^A-Za-z0-9 _.-]', '_', os.path.splitext(os.path.basename(name))[0]).strip() or 'tool'


def tool_path(name):
    return os.path.join(TOOLS_DIR, safe_name(name) + '.stl')


def list_tools():
    if not os.path.isdir(TOOLS_DIR):
        return []
    return sorted(os.path.splitext(f)[0] for f in os.listdir(TOOLS_DIR) if f.lower().endswith('.stl'))


def save_tool(data, filename):
    """Store an uploaded model as binary STL. Returns (name, info)."""
    tris = mesh.load_mesh(data, filename)
    os.makedirs(TOOLS_DIR, exist_ok=True)
    name = safe_name(filename)
    with open(tool_path(name), 'wb') as f:
        f.write(mesh.to_stl_bytes(tris))
    _load.cache_clear()
    size = np.ptp(tris.reshape(-1, 3), axis=0)
    hint = ''
    if size.max() < 5:
        hint = 'The model is only %.3g units long: it was probably exported in metres. Set the scale to 100000%%.' % size.max()
    elif size.max() < 60:
        hint = 'The model is only %.3g units long: if it was exported in cm, set the scale to 1000%%.' % size.max()
    return name, {'triangles': int(len(tris)), 'size': size.round(2).tolist(), 'hint': hint}


@functools.lru_cache(maxsize=8)
def _load(name):
    with open(tool_path(name), 'rb') as f:
        return mesh.load_mesh(f.read(), 'tool.stl')


def triangles_flange(s, name=None):
    """Model triangles (n, 3, 3) in the flange frame for the current settings, or None."""
    name = name if name is not None else s.get('tool_model', '')
    if not name or not os.path.isfile(tool_path(name)):
        return None
    pose = K.xyzrpw_to_pose((parse_float_list(s['tool_model_pose']) + [0] * 6)[:6])
    flat = _load(name).reshape(-1, 3) * (s['tool_model_scale'] / 100.0)
    return (flat @ pose[:3, :3].T + pose[:3, 3]).reshape(-1, 3, 3)


def support_points(v, n_dirs=150):
    """Points of v that are extreme in n_dirs evenly spread directions (bounds the shape in any pose)."""
    g = math.pi * (3 - math.sqrt(5))
    k = np.arange(n_dirs)
    z = 1 - 2 * (k + 0.5) / n_dirs
    r = np.sqrt(1 - z * z)
    dirs = np.stack([r * np.cos(g * k), r * np.sin(g * k), z], axis=1)
    return v[np.unique(np.argmax(v @ dirs.T, axis=0))]


def hull_flange(s):
    """Collision points of the tool model in the flange frame, or None."""
    key = (s.get('tool_model', ''), s['tool_model_pose'], s['tool_model_scale'])
    return _hull(key) if key[0] else None


@functools.lru_cache(maxsize=8)
def _hull(key):
    name, pose, scale = key
    t = triangles_flange({'tool_model': name, 'tool_model_pose': pose, 'tool_model_scale': scale})
    return None if t is None else support_points(t.reshape(-1, 3))


def find_nozzle_tip(s):
    """TCP proposal [x, y, z, rx, ry, rz] (flange frame): the narrow end of the model's long axis, tool Z
    pointing out of the nozzle, tool X as close as possible to flange X."""
    t = triangles_flange(s)
    if t is None:
        raise ValueError('Load a tool model first')
    v = t.reshape(-1, 3)
    # long axis = main axis of the surface's second moment (exact per triangle, so tessellation does not bias it)
    w = 0.5 * np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1) + 1e-12
    mu = (t.mean(axis=1) * w[:, None]).sum(0) / w.sum()
    p = t - mu
    sm = p.sum(axis=1)
    cov = (np.einsum('n,nki,nkj->ij', w, p, p) + np.einsum('n,ni,nj->ij', w, sm, sm)) / (12 * w.sum())
    axis = np.linalg.eigh(cov)[1][:, -1]
    k = int(np.argmax(np.abs(axis)))
    if abs(axis[k]) > math.cos(math.radians(5)):     # CAD models are usually square to the flange: snap
        axis = np.eye(3)[k] * np.sign(axis[k])
    d = v @ axis
    lo, hi = d.min(), d.max()
    band = max(0.03 * (hi - lo), 1.0)

    def spread(end):
        sel = v[(d <= lo + band)] if end < 0 else v[(d >= hi - band)]
        p = sel - sel.mean(0)
        p -= np.outer(p @ axis, axis)
        return float(np.sqrt((p ** 2).sum(1).max())) if len(sel) else 1e9

    end = -1 if spread(-1) < spread(1) else 1          # the nozzle is the narrower end
    zt = axis * end
    tip_band = v[(d * end) >= (hi if end > 0 else -lo) - max(0.002 * (hi - lo), 0.2)]
    tip = (tip_band.min(0) + tip_band.max(0)) / 2            # centre of the nozzle end
    tip += zt * ((v @ zt).max() - tip @ zt)           # exactly at the extreme along the nozzle axis
    x = np.array([1.0, 0, 0]) - zt * zt[0]
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0, 1.0, 0]) - zt * zt[1]
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(zt, x), zt, tip
    return [round(float(a), 3) for a in K.pose_to_xyzrpw(T)]


def mesh_bytes(name):
    """Float32 triangles of a stored model (its own coordinates, mm before scale) for the 3D view."""
    t = _load(name)
    if len(t) > 300_000:
        t = t[:: math.ceil(len(t) / 300_000)]
    return np.ascontiguousarray(t, dtype='<f4').tobytes()
