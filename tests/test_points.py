"""Point control (yora/points.py): even spacing, corners kept, seams kept, min / max spacing."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yora import points  # noqa: E402
from yora.settings import normalize  # noqa: E402


def circle(r=50, n=400, start=0.3):
    a = start + np.linspace(0, 2 * math.pi, n + 1)
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def square(side=100):
    return np.array([[0, 0], [side, 0], [side, side], [0, side], [0, 0]], float)


def seg_lengths(p):
    return np.hypot(*np.diff(p[:, :2], axis=0).T)


class PointControlTest(unittest.TestCase):
    def test_uniform_even_on_circle_and_keeps_seam(self):
        c = circle()
        out = points.control(c, normalize({'point_mode': 'uniform', 'point_spacing': 3}), closed=True)
        d = seg_lengths(out)
        self.assertLess(d.max() - d.min(), 0.05)
        self.assertAlmostEqual(d.mean(), 3.0, delta=0.1)
        np.testing.assert_allclose(out[0], c[0])
        np.testing.assert_allclose(out[-1], c[-1])

    def test_uniform_keeps_corners(self):
        out = points.control(square(), normalize({'point_mode': 'uniform', 'point_spacing': 7}), closed=True)
        for corner in ([100, 0], [100, 100], [0, 100]):
            self.assertLess(np.hypot(*(out - corner).T).min(), 1e-9)
        d = seg_lengths(out)
        self.assertLess(d.max() - d.min(), 1e-9)          # 100 / 14 everywhere, fitted per side

    def test_adaptive_within_tolerance_and_spacing(self):
        c = circle(r=40, n=2000)
        s = normalize({'point_mode': 'adaptive', 'simplify': 0.05, 'point_min_spacing': 0.5, 'point_max_spacing': 8})
        out = points.control(c, s, closed=True)
        d = seg_lengths(out)
        self.assertLessEqual(d.max(), 8 + 1e-6)
        self.assertGreaterEqual(d[:-1].min(), 0.5 - 1e-6)
        mid = (out[1:] + out[:-1]) / 2                     # chord sag stays within the tolerance (+ dense sampling)
        self.assertLess((40 - np.hypot(*mid.T)).max(), 0.06)

    def test_max_spacing_splits_simplify_paths(self):
        out = points.control(square(), normalize({'point_max_spacing': 10}), closed=True)
        self.assertLessEqual(seg_lengths(out).max(), 10 + 1e-9)
        self.assertEqual(len(out), 41)

    def test_min_spacing_merges(self):
        p = np.array([[0, 0], [0.1, 0], [0.2, 0], [5, 0], [5.1, 0.0], [10, 0]], float)
        out = points.control(p, normalize({'point_min_spacing': 1, 'point_keep_corners': False}), closed=False)
        np.testing.assert_allclose(out, [[0, 0], [5, 0], [10, 0]])

    def test_z_is_interpolated(self):
        p = np.array([[0, 0, 0], [10, 0, 1], [20, 0, 2]], float)
        out = points.control(p, normalize({'point_mode': 'uniform', 'point_spacing': 2.5}), closed=False)
        np.testing.assert_allclose(out[:, 2], out[:, 0] / 10)

    def test_default_changes_nothing(self):
        s = normalize({})
        self.assertFalse(points.active(s, True))


if __name__ == '__main__':
    unittest.main()
