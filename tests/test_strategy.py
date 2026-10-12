# Printing strategy (yora/strategy.py): angled layers, horizontal base, tilted nozzle in the robot program.
#   python -m unittest discover -s tests -v
import math
import os
import re
import sys
import tempfile
import unittest
import zipfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import kinematics as K, mesh, post, settings, strategy  # noqa: E402
from yora.planner import EXTRUDING  # noqa: E402


def job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4, 'print_strategy': 'angled'}, **kw))
    tp = yora.slice_to_toolpath(mesh.place(mesh.box_mesh(120, 100, 60), center_xy=(500, 500)), s)
    return tp, s


class AngledTest(unittest.TestCase):

    def test_frame_normal(self):
        R = strategy.frame(settings.normalize({'slice_angle': 45, 'slice_direction': 90}))
        np.testing.assert_allclose(R[:, 2], [0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)

    def test_layers_lie_on_tilted_planes(self):
        tp, s = job(nozzle_orientation='vertical')
        ext = np.isin(tp.kind, EXTRUDING)
        h = np.column_stack([tp.x, tp.y, tp.z]) @ tp.part_R[:, 2]
        for L in np.unique(tp.layer[ext]):
            self.assertLess(np.ptp(h[ext & (tp.layer == L)]), 1e-6)
        steps = np.diff([h[ext & (tp.layer == L)][0] for L in np.unique(tp.layer[ext])])
        np.testing.assert_allclose(steps[1:], 4.0, atol=1e-6)      # one layer height apart, along the normal
        self.assertGreater(tp.z[ext].min(), 0)                       # nothing below the bed
        self.assertLessEqual(tp.z[ext].max(), 60 + 1e-6)
        self.assertFalse(tp.orient.any())                            # vertical nozzle: no tilt
        # the angled block holds about the same material as the flat one
        flat = yora.slice_to_toolpath(mesh.place(mesh.box_mesh(120, 100, 60), center_xy=(500, 500)),
                                        settings.normalize({'layer_height': 4, 'first_layer_height': 4}))
        self.assertAlmostEqual(tp.stats['mass_g'] / flat.stats['mass_g'], 1.0, delta=0.15)

    def test_base_then_angled(self):
        tp, s = job(base_height=20)
        self.assertEqual(tp.flat_layers, 5)
        ext = np.isin(tp.kind, EXTRUDING)
        flat = ext & (tp.layer < 5)
        self.assertLessEqual(tp.z[flat].max(), 20 + 1e-6)
        ang = ext & (tp.layer >= 5)
        self.assertGreaterEqual(tp.z[ang].min(), 20 - 1e-6)            # angled layers start on top of the base
        self.assertTrue((tp.orient[tp.layer >= 5] == 1).all() and not tp.orient[tp.layer < 5].any())
        self.assertTrue(np.all(np.diff(tp.layer) >= 0))                # base first

    def test_clip_loop(self):
        sq = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], float)
        out = strategy.clip_loop(sq, 1, 0, 4)                          # keep x >= 4
        self.assertAlmostEqual(abs(0.5 * np.sum(out[:, 0] * np.roll(out[:, 1], -1) - out[:, 1] * np.roll(out[:, 0], -1))), 60)
        self.assertIsNone(strategy.clip_loop(sq, 1, 0, 20))

    def test_kuka_program_tilts_the_nozzle(self):
        tp, s = job(slice_direction=180)
        path, info = post.export_program(tp, s, tempfile.mkdtemp(), 'Tilt', None, post.load_post('KUKA KRL (KR C4)'))
        src = zipfile.ZipFile(path).read('Tilt.src').decode()
        lins = re.findall(r'LIN \{X ([-\d.]+),Y ([-\d.]+),Z ([-\d.]+),A ([-\d.]+),B ([-\d.]+),C ([-\d.]+)\}', src)
        self.assertGreater(len(lins), 10)
        n = tp.part_R[:, 2]
        for row in lins[5:15]:
            a, b, c = (math.radians(float(v)) for v in row[3:])
            R = (K.rotz(a) @ K.roty(b) @ K.rotx(c))[:3, :3]
            np.testing.assert_allclose(R[:, 2], -n, atol=1e-3)          # tool Z perpendicular to the layer


if __name__ == '__main__':
    unittest.main()
