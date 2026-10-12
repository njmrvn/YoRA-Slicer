"""Printing strategy: horizontal layers, or layers on planes tilted by an angle (e.g. 45 deg), for robot printing.

Angled slicing rotates the model so the tilted planes become horizontal, slices and plans it with the normal slicer
(walls, infill, seams, point control all work the same), then rotates the toolpath back into the bed frame.

    slice frame:  R = Rz(direction) @ Ry(angle);  bed point p = R @ (x, y, z + z0),  layer plane normal n = R[:, 2]
                  (n leans from vertical by `angle`, towards `direction` in the bed XY plane: layers grow that way)

An optional horizontal base is printed first (the part below `base height`, flat layers); the angled layers then
cover only the part above it (each layer clipped at the base top). The nozzle either stays vertical or tilts with
the layers (tool Z along -n, perpendicular to the layer), which the robot check, simulation and export follow.
"""
import math

import numpy as np

from .slicer import Slicer, clip_loop  # noqa: F401  (clip_loop re-exported)
from .planner import Planner, toolpath_from_rows


def is_angled(s):
    return s.get('print_strategy', 'horizontal') == 'angled' and abs(s.get('slice_angle', 0.0)) > 1e-6


def frame(s):
    """3x3 rotation from the slice frame to the bed frame; column 2 is the layer normal."""
    t, p = math.radians(s.get('slice_angle', 0.0)), math.radians(s.get('slice_direction', 0.0))
    ct, st, cp, sp = math.cos(t), math.sin(t), math.cos(p), math.sin(p)
    rz = np.array([[cp, -sp, 0], [sp, cp, 0], [0, 0, 1.0]])
    ry = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    return rz @ ry


def tool_tilt(s):
    """4x4 rotation applied before the 'nozzle down' rotation on angled layers (identity = vertical nozzle)."""
    T = np.eye(4)
    if is_angled(s) and s.get('nozzle_orientation', 'normal') == 'normal':
        T[:3, :3] = frame(s)
    return T


def _clipped_range(tris_s, zb, base):
    """Min / max slice-frame height of the part of the mesh at bed height >= base."""
    v = tris_s.reshape(-1, 3)
    zv = zb.reshape(-1)
    pts = [v[zv >= base]]
    for a, b in ((0, 1), (1, 2), (2, 0)):                      # where the triangle edges cross the base plane
        za, zb_ = zb[:, a], zb[:, b]
        cross = (za - base) * (zb_ - base) < 0
        if cross.any():
            t = ((base - za[cross]) / (zb_[cross] - za[cross]))[:, None]
            pts.append(tris_s[cross, a] + t * (tris_s[cross, b] - tris_s[cross, a]))
    allp = np.concatenate(pts) if pts else np.zeros((0, 3))
    if not len(allp):
        return None
    return float(allp[:, 2].min()), float(allp[:, 2].max())


def slice_angled(tris, s, progress=None):
    """Toolpath for the angled strategy (optional horizontal base first)."""
    progress = progress or (lambda stage, frac: None)
    tris = np.asarray(tris, dtype=np.float64)
    R = frame(s)
    base = max(float(s.get('base_height', 0.0)), 0.0)
    warnings, rows, layer_z = [], [], []
    planner = Planner(s)
    nb = 0
    if base > 0:
        sl = Slicer(tris, s, lambda st, f: progress(st, 0.3 * f), z_limit=base)
        res = sl.run()
        warnings += res.warnings
        nb = len(res.layers)
        if nb:
            rows += planner.plan(res.layers)
            layer_z += [l.z for l in res.layers]
    # the angled part: slice the model rotated into the slice frame
    s2 = dict(s)
    s2['brim_loops'] = 0 if (nb or s['brim_loops'] == 0) else s['brim_loops']
    s2['spiral_vase'] = False
    if nb:
        s2['first_layer_height'] = s['layer_height']
        s2['first_layer_width_pct'] = 100.0
        s2['first_layer_speed'] = s['outer_wall_speed']
    if s['brim_loops'] and not nb:
        warnings.append('Angled layers: the brim is printed only with a horizontal base (Base height > 0).')
        s2['brim_loops'] = 0
    if s.get('spiral_vase'):
        warnings.append('Spiral vase is not used with angled layers.')
    if s.get('raft_type', 'none') != 'none':
        warnings.append('Raft is not used with angled layers yet: use a horizontal base (Base height) instead.')
    tris_s = tris @ R                                        # slice-frame coordinates (p_s = R^T p)
    zmax_bed = float(tris[:, :, 2].max())
    zbed = tris[:, :, 2]
    rng = _clipped_range(tris_s, zbed, base) if nb else (float(tris_s[:, :, 2].min()), float(tris_s[:, :, 2].max()))
    if rng is not None and rng[1] - rng[0] > 1e-6:
        z0 = rng[0]
        tris_s = tris_s.copy()
        tris_s[:, :, 2] -= z0
        clip = (R[2, 0], R[2, 1], R[2, 2], z0, base) if nb else None
        seam_to_slice_frame(s, s2, R, z0, zmax_bed)
        sl2 = Slicer(tris_s, s2, lambda st, f: progress(st, 0.3 * bool(nb) + (1 - 0.3 * bool(nb)) * f),
                     clip=clip, z_top=rng[1] - z0)
        res2 = sl2.run()
        warnings += [w for w in res2.warnings if w not in warnings]
        p2 = Planner(s2)
        p2.e = planner.e
        if planner.pos is not None:
            p2.pos = tuple((np.asarray(planner.pos) @ R) - np.array([0, 0, z0]))
        start = len(p2.rows)
        rows2 = p2.plan(res2.layers)[start:]
        if rows2:
            a = np.array(rows2, dtype=np.float64)
            xyz = (a[:, :3] + np.array([0, 0, z0])) @ R.T      # back to the bed frame
            a[:, :3] = xyz
            a[:, 6] += nb
            rows += [tuple(r) for r in a]
        layer_z += [l.z for l in res2.layers]
    else:
        z0 = 0.0
        warnings.append('Nothing to print with angled layers above the base.')
    if s.get('nozzle_orientation', 'normal') == 'normal':
        warnings.append('Angled layers with a tilted nozzle: the Borunte and KUKA programs carry the tilt; G-code '
                        'carries positions only (vertical nozzle).')
    tp = toolpath_from_rows(rows, layer_z, s, warnings)
    tp.flat_layers = nb
    tp.part_R, tp.part_z0 = R, z0
    tp.tilt = tool_tilt(s)
    tp.orient = (tp.layer >= nb).astype(np.int8) if not np.allclose(tp.tilt, np.eye(4)) else np.zeros(len(tp), np.int8)
    tp.stats['strategy'] = {'angled': True, 'angle': float(s['slice_angle']), 'direction': float(s['slice_direction']),
                            'normal': R[:, 2].tolist(), 'flat_layers': nb, 'nozzle': s.get('nozzle_orientation', 'normal')}
    return tp


def seam_to_slice_frame(s, s2, R, z0, zmax):
    """Seam settings (bed frame) for the planner working in the slice frame: a seam point becomes a vertical line
    through it, a seam line is rotated, a direction (or 'rear' = +Y) is turned into the slice frame."""
    from .planner import parse_seam_path
    mode = s['seam']
    if mode in ('custom', 'path'):
        P = parse_seam_path(s['seam_path']) if mode == 'path' else None
        if P is None:
            P = np.array([[s['seam_x'], s['seam_y'], -10.0], [s['seam_x'], s['seam_y'], zmax + 10.0]])
        elif len(P) == 1:
            P = np.array([[P[0, 0], P[0, 1], -10.0], [P[0, 0], P[0, 1], zmax + 10.0]])
        Ps = P @ R - np.array([0, 0, z0])
        s2['seam'] = 'path'
        s2['seam_path'] = ';'.join('%.4f,%.4f,%.4f' % tuple(p) for p in Ps)
    elif mode in ('direction', 'rear'):
        a = math.radians(s['seam_angle'] if mode == 'direction' else 90.0)
        d = np.array([math.cos(a), math.sin(a), 0.0]) @ R
        s2['seam'] = 'direction'
        s2['seam_angle'] = math.degrees(math.atan2(d[1], d[0]))


def tool_rotation(tp, i):
    """4x4 rotation for move i of the toolpath, inserted before the 'nozzle down' rotation."""
    orient = getattr(tp, 'orient', None)
    if orient is None or not orient[i]:
        return np.eye(4)
    return tp.tilt
