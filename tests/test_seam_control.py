# Seam position control: direction, picked point, drawn seam line (also with angled layers).
#   python -m unittest discover -s tests -v
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import mesh, settings  # noqa: E402
from yora.planner import parse_seam_path, seam_on_path  # noqa: E402
from yora.slicer import KIND_OUTER  # noqa: E402

BOX = mesh.place(mesh.box_mesh(100, 100, 40), center_xy=(500, 500))   # x, y in 450..550, z 0..40


def outer_starts(tp):
    """(layer, x, y, z) of the first point of every outer wall loop."""
    k = tp.kind
    start = (k == KIND_OUTER) & np.r_[True, k[:-1] != KIND_OUTER]
    i = np.nonzero(start)[0] - 1          # the loop starts where the move into its first point begins
    return np.column_stack([tp.layer[i + 1], tp.x[i], tp.y[i], tp.z[i]])


def job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4, 'walls': 1}, **kw))
    return yora.slice_to_toolpath(BOX, s)


class SeamControlTest(unittest.TestCase):

    def test_path_helpers(self):
        P = parse_seam_path('0,0,10; 10,0,0 ; 5 5 20')
        np.testing.assert_allclose(P[:, 2], [0, 10, 20])
        np.testing.assert_allclose(seam_on_path(P, 5), [5, 0])
        np.testing.assert_allclose(seam_on_path(P, 99), [5, 5])
        self.assertIsNone(parse_seam_path(''))

    def test_direction(self):
        st = outer_starts(job(seam='direction', seam_angle=0))
        self.assertTrue((st[:, 1] > 540).all())                 # every loop starts on the +X side
        st = outer_starts(job(seam='direction', seam_angle=-90))
        self.assertTrue((st[:, 2] < 460).all())                 # ... or the -Y side

    def test_custom_point(self):
        st = outer_starts(job(seam='custom', seam_x=530, seam_y=600))
        self.assertTrue((np.abs(st[:, 1] - 530) < 3).all() and (st[:, 2] > 540).all())

    def test_drawn_line_moves_with_height(self):
        st = outer_starts(job(seam='path', seam_path='451,470,0; 451,530,40'))
        self.assertTrue((st[:, 1] < 460).all())                 # on the -X face all the way up
        np.testing.assert_allclose(st[:, 2], 470 + 60 * (st[:, 3] - 2) / 40, atol=4)   # rising along Y with Z

    def test_angled_layers_follow_the_bed_point(self):
        st = outer_starts(job(seam='custom', seam_x=500, seam_y=600, print_strategy='angled',
                              nozzle_orientation='vertical'))
        self.assertGreater(len(st), 10)
        self.assertTrue((st[:, 2] > 540).all())                 # still at the back (+Y) of the part

    def test_angled_layers_follow_a_drawn_line(self):
        # a short line on the +Y face, low down: tilted layers that do not cross it still start next to it
        st = outer_starts(job(seam='path', seam_path='530,551,5; 530,551,15', print_strategy='angled',
                              nozzle_orientation='vertical'))
        self.assertGreater(len(st), 10)
        self.assertGreater((st[:, 2] > 540).mean(), 0.9)


if __name__ == '__main__':
    unittest.main()
