# Surface pattern (yora/pattern.py): waves on the outer wall, layer shift, holes, spiral vase, height range.
#   python -m unittest discover -s tests -v
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import pattern, settings  # noqa: E402
from yora.slicer import KIND_OUTER  # noqa: E402

C = np.array([500.0, 500.0])


def tube(r_out, r_in, h, n=180):
    """Closed tube (r_in = 0: a solid cylinder) around C, standing on the bed, outward normals."""
    a = np.linspace(0, 2 * math.pi, n + 1)
    tris = []

    def p(r, t, z):
        return [C[0] + r * math.cos(t), C[1] + r * math.sin(t), z]
    for a0, a1 in zip(a[:-1], a[1:]):
        o00, o10, o11, o01 = p(r_out, a0, 0), p(r_out, a1, 0), p(r_out, a1, h), p(r_out, a0, h)
        tris += [[o00, o10, o11], [o00, o11, o01]]
        i00, i10, i11, i01 = p(r_in, a0, 0), p(r_in, a1, 0), p(r_in, a1, h), p(r_in, a0, h)
        if r_in > 0:
            tris += [[i00, i11, i10], [i00, i01, i11]]
            tris += [[i01, o01, o11], [i01, o11, i11]]              # top annulus
            tris += [[i00, o10, o00], [i00, i10, o10]]              # bottom annulus
        else:
            tris += [[i01, o01, o11], [i00, o10, o00]]
    return np.array(tris, dtype=float)


def job(model, **kw):
    base = {'layer_height': 4, 'first_layer_height': 4, 'line_width': 5, 'walls': 1, 'infill_density': 0,
            'top_layers': 0, 'bottom_layers': 0, 'pattern_amplitude': 4, 'pattern_wavelength': 20}
    return yora.slice_to_toolpath(model, settings.normalize(dict(base, **kw)))


def outer(tp, layer=None):
    m = (tp.kind == KIND_OUTER)
    if layer is not None:
        m &= tp.layer == layer
    xy = np.column_stack([tp.x[m], tp.y[m]])
    return xy, np.hypot(*(xy - C).T)


class PatternTest(unittest.TestCase):

    def test_profiles(self):
        f = np.array([0, 0.25, 0.5, 0.75])
        np.testing.assert_allclose(pattern.profile('sine', f), [0, 0.5, 1, 0.5], atol=1e-12)
        np.testing.assert_allclose(pattern.profile('zigzag', f), [0, 0.5, 1, 0.5], atol=1e-12)
        np.testing.assert_allclose(pattern.profile('spikes', f, 0.5), [0, 0, 1, 0], atol=1e-12)

    def test_off_by_default(self):
        self.assertFalse(pattern.active(settings.normalize({})))
        _, r = outer(job(tube(100, 0, 20)), 2)
        self.assertLess(r.max() - r.min(), 0.3)                         # plain round wall

    def test_sine_outward_on_a_cylinder(self):
        tp = job(tube(100, 0, 20), pattern='sine')
        _, r = outer(tp, 2)
        self.assertAlmostEqual(r.min(), 97.5, delta=0.3)                # wall centre: 100 - w/2
        self.assertAlmostEqual(r.max(), 101.5, delta=0.3)               # + 4 mm depth, outward only
        # whole waves: round(2 pi 97.5 / 20) = 31 peaks round the loop
        ang = np.arctan2(*(outer(tp, 2)[0] - C).T[::-1])
        peaks = np.unique(np.round(ang[r > 101.3], 1))
        self.assertEqual(len(_clusters(np.sort(peaks), 0.05)), 31)

    def test_centered_and_inward(self):
        _, r = outer(job(tube(100, 0, 20), pattern='zigzag', pattern_direction='centered'), 2)
        self.assertAlmostEqual(r.min(), 95.5, delta=0.3)
        self.assertAlmostEqual(r.max(), 99.5, delta=0.3)
        _, r = outer(job(tube(100, 0, 20), pattern='zigzag', pattern_direction='inward'), 2)
        self.assertAlmostEqual(r.max(), 97.5, delta=0.3)

    def test_layer_shift_alternates(self):
        tp = job(tube(100, 0, 20), pattern='sine', pattern_shift=50, pattern_amplitude=2)
        r_at = []
        for L in (2, 3):
            xy, r = outer(tp, L)
            ang = np.mod(np.arctan2(*(xy - C).T[::-1]), 2 * math.pi)
            order = np.argsort(ang)
            r_at.append(np.interp(np.linspace(0.1, 6.1, 400), ang[order], r[order]))
        d = r_at[0] - r_at[1]
        self.assertGreater(np.abs(d).max(), 1.5)                         # crest over trough
        self.assertAlmostEqual(float(np.mean(r_at[0] + r_at[1])) / 2, 97.5 + 1.0, delta=0.15)

    def test_hole_waves_go_into_the_hole(self):
        tp = job(tube(100, 60, 20), pattern='sine')
        _, r = outer(tp, 2)
        hole, rim = r[r < 80], r[r > 80]
        self.assertAlmostEqual(rim.max(), 101.5, delta=0.3)
        self.assertAlmostEqual(hole.max(), 62.5, delta=0.3)              # hole wall centre: 60 + w/2
        self.assertAlmostEqual(hole.min(), 58.5, delta=0.3)              # waves grow into the hole (the air)

    def test_spiral_vase_stays_continuous(self):
        tp = job(tube(100, 0, 24), pattern='sine', spiral_vase=True, pattern_shift=50)
        m = tp.kind == KIND_OUTER
        x, y = tp.x[m], tp.y[m]
        r = np.hypot(x - C[0], y - C[1])
        self.assertAlmostEqual(r.max(), 101.5, delta=0.3)
        dr = np.abs(np.diff(r))                                         # radius changes smoothly, also where
        self.assertLess(dr.max(), 1.2)                                  # one turn meets the next (no phase jump)

    def test_height_range(self):
        tp = job(tube(100, 0, 40), pattern='sine', pattern_z_min=20)
        _, low = outer(tp, 1)                                           # layer top at 8 mm
        _, high = outer(tp, 7)                                          # 32 mm
        self.assertLess(low.max() - low.min(), 0.3)
        self.assertGreater(high.max() - high.min(), 3.5)

    def test_pattern_change_reuses_the_slice(self):
        from unittest import mock
        model, cache = tube(100, 0, 20), {}
        base = {'layer_height': 4, 'first_layer_height': 4, 'walls': 1, 'infill_density': 0, 'top_layers': 0,
                'bottom_layers': 0, 'pattern': 'sine'}
        yora.slice_to_toolpath(model, settings.normalize(base), cache=cache)
        with mock.patch('yora.slice_mesh', side_effect=AssertionError('sliced again')):
            tp = yora.slice_to_toolpath(model, settings.normalize(dict(base, pattern_amplitude=6)), cache=cache)
        self.assertAlmostEqual(outer(tp, 2)[1].max(), 103.5, delta=0.3)            # the new depth is used
        with mock.patch('yora.slice_mesh', wraps=yora.slice_mesh) as sm:            # other settings: slice again
            yora.slice_to_toolpath(model, settings.normalize(dict(base, walls=2)), cache=cache)
        self.assertEqual(sm.call_count, 1)

    def test_bumps_rows_and_stagger(self):
        tp = job(tube(100, 0, 40), pattern='bumps', pattern_row_height=16, pattern_shift=50, pattern_spike_width=80)
        _, r = outer(tp, 1)                                             # mid-layer 6 mm: row 0, near its middle (8)
        self.assertAlmostEqual(r.max(), 101.5, delta=0.6)
        self.assertAlmostEqual(r.min(), 97.5, delta=0.3)                # plain wall between the domes
        xy0, r0 = outer(tp, 1)
        xy1, r1 = outer(tp, 5)                                          # 22 mm: next row, moved half a bump
        a0 = np.arctan2(*(xy0[r0 > 101] - C).T[::-1])
        a1 = np.arctan2(*(xy1[r1 > 101] - C).T[::-1])
        gap = np.abs((a0[:, None] - a1[None, :] + math.pi) % (2 * math.pi) - math.pi).min()
        self.assertGreater(gap, 0.05)                                   # peaks of the two rows do not line up

    def test_noise_closes_and_varies(self):
        tp = job(tube(100, 0, 20), pattern='noise', pattern_seed=3)
        xy, r = outer(tp, 2)
        self.assertGreater(r.max() - r.min(), 2.0)
        x, y = np.linspace(0, 3, 50), np.linspace(0, 2, 50)              # repeats round the loop: no step at the seam
        np.testing.assert_allclose(pattern.value_noise(x, y, 31, 3), pattern.value_noise(x + 31, y, 31, 3))
        _, r2 = outer(job(tube(100, 0, 20), pattern='noise', pattern_seed=4), 2)
        self.assertNotEqual(len(r), 0)
        self.assertGreater(abs(np.sort(r)[len(r) // 2] - np.sort(r2)[len(r2) // 2]) + np.abs(r.mean() - r2.mean()), 1e-3)

    def test_image_wraps_round_the_part(self):
        import tempfile
        from unittest import mock
        import cv2
        with tempfile.TemporaryDirectory() as d, mock.patch.object(pattern, 'PATTERN_DIR', d):
            img = np.full((20, 40), 255, np.uint8)
            img[:, :20] = 0                                             # left half black
            ok, png = cv2.imencode('.png', img)
            name = pattern.save_image(png.tobytes(), 'half black.png')
            self.assertEqual(pattern.list_images(), [name])
            with self.assertRaises(ValueError):
                pattern.save_image(b'not a picture', 'x.png')
            # picture centred at -Y (front), 180 deg wide: the black half covers -180..-90 deg
            tp = job(tube(100, 0, 20), pattern='image', pattern_image=name, pattern_image_span=180)
            xy, r = outer(tp, 2)
            ang = np.degrees(np.arctan2(*(xy - C).T[::-1]))
            raised = r > 100
            self.assertTrue(raised.any())
            self.assertTrue(((ang[raised] > -182) & (ang[raised] < -88)).all())
            self.assertLess(r[(ang > 10) & (ang < 170)].max(), 97.8)    # back of the part: no picture

    def test_paint_limits_the_pattern(self):
        side = '; '.join(f'{500 + 100 * math.cos(t):.1f},{500 + 100 * math.sin(t):.1f},{z}'
                         for t in np.linspace(-0.3, 0.3, 7) for z in (5, 10, 15))      # a patch at +X
        tp = job(tube(100, 0, 20), pattern='sine', pattern_paint=side, pattern_paint_mode='painted', pattern_brush=15)
        xy, r = outer(tp, 2)
        ang = np.arctan2(*(xy - C).T[::-1])
        self.assertGreater(r[np.abs(ang) < 0.25].max(), 101)
        self.assertLess(r[np.abs(ang) > 0.8].max(), 97.8)
        tp = job(tube(100, 0, 20), pattern='sine', pattern_paint=side, pattern_paint_mode='unpainted', pattern_brush=15)
        xy, r = outer(tp, 2)
        ang = np.arctan2(*(xy - C).T[::-1])
        self.assertLess(r[np.abs(ang) < 0.2].max(), 97.8)
        self.assertGreater(r[np.abs(ang) > 0.8].max(), 101)

    def test_warnings(self):
        s = settings.normalize({'pattern': 'sine', 'pattern_amplitude': 4, 'pattern_shift': 50, 'line_width': 5})
        self.assertTrue(any('hanging' in w for w in pattern.warnings(s)))


def _clusters(v, gap):
    out = [[v[0]]]
    for a in v[1:]:
        if a - out[-1][-1] > gap:
            out.append([a])
        else:
            out[-1].append(a)
    if len(out) > 1 and v[0] + 2 * math.pi - v[-1] <= gap:              # wraps round at +-pi
        out[0] += out.pop()
    return out


if __name__ == '__main__':
    unittest.main()


def barrel(h=400.0, r_end=200.5, r_belly=233.0, n=360, rows=80):
    """The barrel table of the report: Ø 401 mm at foot and lip, Ø 466 mm at the belly, 400 mm tall."""
    zs = np.linspace(0, h, rows + 1)
    rs = r_end + (r_belly - r_end) * np.sin(math.pi * zs / h)
    a = np.linspace(0, 2 * math.pi, n + 1)
    P = lambda r, t, z: [C[0] + r * math.cos(t), C[1] + r * math.sin(t), z]   # noqa: E731
    tris = []
    for k in range(rows):
        for t0, t1 in zip(a[:-1], a[1:]):
            b0, b1 = P(rs[k], t0, zs[k]), P(rs[k], t1, zs[k])
            u0, u1 = P(rs[k + 1], t0, zs[k + 1]), P(rs[k + 1], t1, zs[k + 1])
            tris += [[b0, b1, u1], [b0, u1, u0]]
    for t0, t1 in zip(a[:-1], a[1:]):
        tris += [[P(0, 0, h), P(rs[-1], t0, h), P(rs[-1], t1, h)], [P(0, 0, 0), P(rs[0], t1, 0), P(rs[0], t0, 0)]]
    return np.array(tris, dtype=float)


def wave_phase(tp, n):
    """Per layer: where the waves sit round the part, in waves (0..1): the phase of the N-th harmonic of the radius."""
    m = tp.kind == KIND_OUTER
    x, y, L = tp.x[m] - C[0], tp.y[m] - C[1], tp.layer[m]
    r, th = np.hypot(x, y), np.arctan2(y, x)
    out = {}
    for layer in np.unique(L):
        k = L == layer
        if k.sum() < 4 * n:
            continue
        d = r[k] - r[k].mean()
        z = np.sum(d * np.exp(-1j * n * th[k]))
        out[int(layer)] = (np.angle(z) / (2 * math.pi)) % 1.0
    return out


def wrap(v):
    return (np.asarray(v) + 0.5) % 1.0 - 0.5


class PhaseTest(unittest.TestCase):
    """One wave count per lap for the whole part: the waves stay in phase up a shape whose diameter changes."""

    @classmethod
    def setUpClass(cls):
        cls.model = barrel()

    def job(self, **kw):
        base = {'layer_height': 3, 'first_layer_height': 3, 'line_width': 5, 'walls': 1, 'infill_density': 0,
                'top_layers': 0, 'bottom_layers': 2, 'spiral_vase': True, 'pattern': 'sine', 'pattern_amplitude': 2,
                'pattern_wavelength': 10, 'bed_x': 1000, 'bed_y': 1000}
        return yora.slice_to_toolpath(self.model, settings.normalize(dict(base, **kw)))

    def test_vertical_ribs_on_the_barrel(self):
        tp = self.job(pattern_waves=36, pattern_shift=0)
        self.assertEqual(tp.stats['pattern'][0]['n'], 36)
        ph = wave_phase(tp, 36)
        self.assertGreater(len(ph), 100)
        spread = np.abs(wrap(np.array(list(ph.values())) - list(ph.values())[0]))
        self.assertLess(spread.max(), 0.05)                                  # same place at foot, belly and lip

    def test_even_diagonal_on_the_barrel(self):
        tp = self.job(pattern_waves=136, pattern_shift=100 / 6)
        ph = wave_phase(tp, 136)
        layers = [k for k in sorted(ph) if k >= 2]                          # the spiral (above the 2 bottom layers)
        steps = wrap(np.diff([ph[k] for k in layers]) - 1 / 6)              # each layer moves on by 1/6 wave
        self.assertLess(np.abs(steps).max(), 0.01)
        m = (tp.kind == KIND_OUTER) & (tp.layer >= 2)
        r = np.hypot(tp.x[m] - C[0], tp.y[m] - C[1])
        self.assertLess(np.abs(np.diff(r)).max(), 1.0)                      # no step where the spiral turns join

    def test_wave_count_from_the_wavelength_and_the_report(self):
        tp = self.job(pattern_wavelength=10)
        rep = tp.stats['pattern'][0]
        self.assertTrue(130 <= rep['n'] <= 140)                              # mean perimeter ~1380 mm / 10
        self.assertLess(rep['wavelength_min'], 10)
        self.assertGreater(rep['wavelength_max'], 10)

    def test_cylinder_whose_perimeter_is_not_a_multiple(self):
        # closed loops (no spiral), perimeter 2 pi 97.5 = 612.6 mm, target 40 mm: N = 15, ribs line up
        tp = job(tube(100, 0, 60), pattern='sine', pattern_wavelength=40, pattern_shift=0)
        self.assertEqual(tp.stats['pattern'][0]['n'], 15)
        ph = wave_phase(tp, 15)
        vals = np.array([ph[k] for k in sorted(ph) if k >= 2])
        self.assertLess(np.abs(wrap(vals - vals[0])).max(), 0.03)
