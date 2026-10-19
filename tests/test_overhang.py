# Overhang angle of the walls (yora/overhang.py): cones of known angle, the warning and the limit setting.
#   python -m unittest discover -s tests -v
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import settings  # noqa: E402
from yora.slicer import KIND_OUTER  # noqa: E402

C = (500.0, 500.0)


def frustum(r0, r1, h, n=180):
    """Closed solid of revolution: radius r0 at the bed, r1 at height h, centred on C."""
    a = np.linspace(0, 2 * math.pi, n + 1)
    p = lambda r, t, z: [C[0] + r * math.cos(t), C[1] + r * math.sin(t), z]   # noqa: E731
    tris = []
    for t0, t1 in zip(a[:-1], a[1:]):
        b0, b1, u0, u1 = p(r0, t0, 0), p(r0, t1, 0), p(r1, t0, h), p(r1, t1, h)
        tris += [[b0, b1, u1], [b0, u1, u0]]                       # side
        tris += [[p(0, 0, h), u0, u1], [p(0, 0, 0), b1, b0]]       # top and bottom caps
    return np.array(tris, dtype=float)


def job(model, **kw):
    s = settings.normalize(dict({'layer_height': 2, 'first_layer_height': 2, 'line_width': 5, 'walls': 1,
                                 'infill_density': 0, 'top_layers': 0, 'bottom_layers': 0}, **kw))
    return yora.slice_to_toolpath(model, s)


def outer_angles(tp):
    m = (tp.kind == KIND_OUTER) & (tp.layer >= 3) & (tp.layer < len(tp.layer_z) - 2)
    return tp.overhang[m]


class OverhangTest(unittest.TestCase):

    def test_straight_wall_has_none(self):
        tp = job(frustum(100, 100, 40))
        self.assertLess(np.percentile(outer_angles(tp), 95), 10)
        self.assertEqual(tp.stats['overhang']['layers'], [])
        self.assertFalse(any('Overhang' in w for w in tp.warnings))

    def test_cone_angles(self):
        for deg in (30, 60):
            tp = job(frustum(60, 60 + 40 * math.tan(math.radians(deg)), 40))
            med = float(np.median(outer_angles(tp)))
            self.assertAlmostEqual(med, deg, delta=6, msg=deg)

    def test_inward_slope_is_supported_by_infill(self):
        # a narrowing cone: a hollow single wall hangs inwards, a filled part does not
        self.assertGreater(np.median(outer_angles(job(frustum(120, 60, 40)))), 45)
        self.assertLess(np.median(outer_angles(job(frustum(120, 60, 40), infill_density=100))), 10)
        self.assertLess(np.percentile(outer_angles(job(frustum(120, 60, 40), infill_density=100, walls=2)), 95), 10)

    def test_warning_follows_the_limit(self):
        model = frustum(60, 60 + 40 * math.tan(math.radians(55)), 40)
        tp = job(model, overhang_limit=45)
        self.assertGreater(len(tp.stats['overhang']['layers']), 10)
        self.assertTrue(any('Overhang steeper than 45' in w for w in tp.warnings))
        tp = job(model, overhang_limit=70)
        self.assertEqual(tp.stats['overhang']['layers'], [])

    def test_angled_layers_take_the_overhang(self):
        # a 50 deg overhang printed on 45 deg planes tilted towards it: what is left is small
        model = frustum(60, 60 + 40 * math.tan(math.radians(50)), 40)
        flat = job(model)
        tilted = job(model, print_strategy='angled', slice_angle=45, slice_direction=0)
        m = tilted.kind == KIND_OUTER
        self.assertTrue(np.isfinite(tilted.overhang[m]).all())
        self.assertLess(np.percentile(tilted.overhang[m], 50), np.percentile(flat.overhang[flat.kind == KIND_OUTER], 50))


if __name__ == '__main__':
    unittest.main()
