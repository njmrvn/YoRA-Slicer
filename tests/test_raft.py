# Raft (yora/raft.py): shapes, offset, line angle per layer, the part lifted onto the raft.
#   python -m unittest discover -s tests -v
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import mesh, settings  # noqa: E402
from yora.slicer import KIND_BRIM, KIND_OUTER, KIND_RAFT  # noqa: E402


def box(x0, x1, y0, y1, z0, z1):
    b = mesh.box_mesh(x1 - x0, y1 - y0, z1 - z0).reshape(-1, 3)
    return (b - b.min(0) + [x0, y0, z0]).reshape(-1, 3, 3)


L_PART = np.concatenate([box(400, 600, 400, 440, 0, 20), box(400, 440, 400, 600, 0, 20)])   # an L, 200 x 200


def job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4, 'raft_offset': 10, 'raft_layers': 3}, **kw))
    return yora.slice_to_toolpath(L_PART, s)


class RaftTest(unittest.TestCase):

    def test_shapes_and_offset(self):
        area = {t: job(raft_type=t).stats['raft']['area_mm2'] for t in ('outline', 'hull', 'rectangle', 'circle')}
        self.assertLess(area['outline'], area['hull'])                     # the closed area of the L is larger
        self.assertLess(area['hull'], area['rectangle'])
        self.assertAlmostEqual(area['rectangle'], 220 * 220, delta=600)     # box + 10 mm offset on every side
        self.assertAlmostEqual(area['circle'], math.pi * (math.hypot(200, 200) / 2 + 10) ** 2, delta=1500)
        self.assertGreater(job(raft_type='outline', raft_offset=20).stats['raft']['area_mm2'], area['outline'] + 5000)

    def test_part_lifted_onto_the_raft(self):
        tp = job(raft_type='hull', raft_gap=0.5, brim_loops=2)
        raft = tp.kind == KIND_RAFT
        self.assertEqual(sorted(set(np.round(tp.z[raft], 3))), [4.0, 8.0, 12.0])
        self.assertAlmostEqual(tp.z[tp.kind == KIND_OUTER].min(), 12 + 0.5 + 4)       # raft + gap + first layer
        self.assertTrue((tp.layer[raft] < 3).all() and (tp.layer[~raft & (tp.kind == KIND_OUTER)] >= 3).all())
        self.assertFalse((tp.kind == KIND_BRIM).any())                               # no brim with a raft
        self.assertGreater(tp.stats['by_kind_mm']['raft'], 0)

    def test_line_angle_per_layer(self):
        tp = job(raft_type='rectangle', raft_layers=2, raft_angle=30, raft_angle_step=90)
        for layer, want in ((0, 30), (1, 120)):
            i = np.nonzero((tp.kind == KIND_RAFT) & (tp.layer == layer))[0]
            d = np.column_stack([tp.x[i] - tp.x[i - 1], tp.y[i] - tp.y[i - 1]])
            d = d[np.hypot(*d.T) > 30]
            self.assertAlmostEqual(float(np.median(np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 180)), want, delta=1)

    def test_one_continuous_path_per_layer_and_density(self):
        def raft_paths_and_length(**kw):
            tp = job(raft_type='hull', raft_layers=2, raft_top_solid=False, **kw)
            paths = []
            for layer in (0, 1):
                k = tp.kind[tp.layer == layer]
                paths.append(int(((k == KIND_RAFT) & np.r_[True, k[:-1] != KIND_RAFT]).sum()))
            r = np.nonzero(tp.kind == KIND_RAFT)[0]
            return paths, float(np.hypot(tp.x[r] - tp.x[r - 1], tp.y[r] - tp.y[r - 1]).sum())
        paths, half = raft_paths_and_length(raft_density=40)
        self.assertEqual(paths, [1, 1])                                   # no travel inside a raft layer
        paths, full = raft_paths_and_length(raft_density=80)
        self.assertEqual(paths, [1, 1])
        self.assertAlmostEqual(full / half, 2.0, delta=0.15)                # line length follows the density

    def test_no_raft_by_default(self):
        tp = job()
        self.assertFalse((tp.kind == KIND_RAFT).any())
        self.assertNotIn('raft', tp.stats)


if __name__ == '__main__':
    unittest.main()
