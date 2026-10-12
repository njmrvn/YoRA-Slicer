"""Write sample STL models into samples/ (run from the project folder):
    python tools/make_samples.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from yora import mesh  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'samples')


def vase(r_bottom=90, r_top=120, height=300, wave=8, lobes=6, segments=144, rings=60):
    """Closed, lobed vase body (solid): good for spiral vase mode."""
    a = np.linspace(0, 2 * math.pi, segments, endpoint=False)
    zs = np.linspace(0, height, rings + 1)
    rings_pts = []
    for z in zs:
        t = z / height
        r = r_bottom + (r_top - r_bottom) * math.sin(t * math.pi / 2) + wave * np.cos(lobes * a + t * math.pi)
        rings_pts.append(np.column_stack([r * np.cos(a), r * np.sin(a), np.full(segments, z)]))
    tris = []
    for k in range(rings):
        lo, hi = rings_pts[k], rings_pts[k + 1]
        for i in range(segments):
            j = (i + 1) % segments
            tris += [(lo[i], lo[j], hi[j]), (lo[i], hi[j], hi[i])]
    for ring, z, up in ((rings_pts[0], 0, False), (rings_pts[-1], height, True)):
        c = np.array([0, 0, z])
        for i in range(segments):
            j = (i + 1) % segments
            tris.append((c, ring[i], ring[j]) if up else (c, ring[j], ring[i]))
    return np.array(tris, dtype=np.float64)


def bracket():
    """L-shaped block with a round boss: two overlapping boxes plus a tube."""
    a = mesh.box_mesh(400, 120, 60)
    b = mesh.box_mesh(120, 400, 60)
    t = mesh.tube_mesh(90, 50, 60)
    t[:, :, 0] += 260
    t[:, :, 1] += 260
    return np.concatenate([a, b, t])


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for name, tris in (('vase.stl', vase()), ('bracket.stl', bracket()), ('tube.stl', mesh.tube_mesh(150, 110, 200, 128))):
        path = os.path.join(OUT, name)
        with open(path, 'wb') as f:
            f.write(mesh.to_stl_bytes(tris))
        print('wrote', os.path.normpath(path), len(tris), 'triangles')
