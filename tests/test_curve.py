# Curve strategy (yora/curve.py), wall placement and print direction (slicer / planner), per-layer numbers.
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
from yora import curve, kinematics as K, mesh, post, settings, strategy  # noqa: E402
from yora.planner import EXTRUDING  # noqa: E402
from yora.slicer import KIND_OUTER, signed_area  # noqa: E402

BEND_R, LEAD, BEND = 250.0, 60.0, 60.0


def bent_bar(r=40.0, n_ring=48, n_path=60, cx=500.0, cy=500.0):
    """Round bar: straight up for LEAD mm, then bending BEND degrees towards +X with radius BEND_R."""
    path, tang = [], []
    for z in np.linspace(0, LEAD, 8):
        path.append([0, 0, z]); tang.append([0, 0, 1])
    for th in np.linspace(0, math.radians(BEND), n_path)[1:]:
        path.append([BEND_R * (1 - math.cos(th)), 0, LEAD + BEND_R * math.sin(th)])
        tang.append([math.sin(th), 0, math.cos(th)])
    path, tang = np.array(path), np.array(tang)
    a = np.linspace(0, 2 * math.pi, n_ring, endpoint=False)
    rings = [p + r * (np.cos(a)[:, None] * np.array([t[2], 0, -t[0]]) + np.sin(a)[:, None] * np.array([0, 1.0, 0]))
             for p, t in zip(path, tang)]
    R = np.array(rings)
    tris = []
    for i in range(len(R) - 1):
        for j in range(n_ring):
            k = (j + 1) % n_ring
            tris += [[R[i, j], R[i, k], R[i + 1, k]], [R[i, j], R[i + 1, k], R[i + 1, j]]]
    for j in range(n_ring):
        k = (j + 1) % n_ring
        tris += [[path[0], R[0, k], R[0, j]], [path[-1], R[-1, j], R[-1, k]]]
    T = np.array(tris)
    T[:, :, 0] += cx
    T[:, :, 1] += cy
    return T


def centreline_distance(p, cx=500.0, cy=500.0):
    """Distance of points from the bar's middle line (sampled every 0.25 mm)."""
    th = np.linspace(0, math.radians(BEND), 1200)
    line = np.vstack([np.column_stack([np.zeros(240), np.zeros(240), np.linspace(0, LEAD, 240)]),
                      np.column_stack([BEND_R * (1 - np.cos(th)), np.zeros(len(th)), LEAD + BEND_R * np.sin(th)])])
    line[:, 0] += cx
    line[:, 1] += cy
    return np.array([np.linalg.norm(line - q, axis=1).min() for q in p])


def curve_job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4, 'line_width': 6, 'print_strategy': 'curve'}, **kw))
    return yora.slice_to_toolpath(bent_bar(), s), s


class CurveTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tp, cls.s = curve_job()

    def test_found_curve_follows_the_middle(self):
        P = curve.find_curve(bent_bar(), self.s)
        self.assertGreaterEqual(len(P), 4)
        self.assertLess(abs(P[0, 2]), 1e-6)                     # starts on the bed
        self.assertLess(centreline_distance(P).max(), 6.0)      # bar radius 40
        self.assertGreater(P[-1, 2], LEAD + 0.8 * BEND_R * math.sin(math.radians(BEND)))

    def test_layers_lie_flat_on_their_planes(self):
        tp = self.tp
        nrm = np.array(tp.stats['strategy']['normals'])
        ext = np.isin(tp.kind, EXTRUDING)
        P = np.column_stack([tp.x, tp.y, tp.z])
        for L in np.unique(tp.layer[ext]):
            h = P[ext & (tp.layer == L)] @ nrm[L]
            self.assertLess(np.ptp(h), 0.01)

    def test_planes_turn_with_the_bend(self):
        tilt = np.degrees(np.arccos(np.clip(np.array(self.tp.stats['strategy']['normals'])[:, 2], -1, 1)))
        self.assertLess(tilt[0], 1.0)                           # flat on the bed
        self.assertGreater(tilt[-1], 0.8 * BEND)
        self.assertLess(np.abs(np.diff(tilt)).max(), 3.0)       # no jumps between layers
        q = [tilt[int(f * (len(tilt) - 1))] for f in (0.25, 0.5, 0.75)]
        self.assertTrue(q[0] < q[1] < q[2])                     # keeps leaning further

    def test_layers_stay_on_the_part(self):
        tp = self.tp
        ext = np.isin(tp.kind, EXTRUDING)
        P = np.column_stack([tp.x, tp.y, tp.z])[ext][::3]
        d = centreline_distance(P)
        self.assertLess(np.percentile(d, 99), 40.5)             # inside the bar (outer bead edge on its surface)
        self.assertLess(d.max(), 43.0)                          # the top layer may reach a little past the end cap

    def test_flow_follows_the_thickness(self):
        # the bend's centre is on the +X side: there the layers are thinner, so less flow per mm than on the -X side
        tp = self.tp
        L = len(tp.layer_z) * 2 // 3
        m = np.nonzero((tp.layer == L) & (tp.kind == KIND_OUTER))[0]
        P = np.column_stack([tp.x, tp.y, tp.z])
        epm = (tp.e[m] - tp.e[m - 1]) / np.maximum(np.linalg.norm(P[m] - P[m - 1], axis=1), 1e-9)
        x = (P[m, 0] + P[m - 1, 0]) / 2
        inside, outside = epm[x > np.percentile(x, 80)].mean(), epm[x < np.percentile(x, 20)].mean()
        self.assertGreater(outside / inside, 1.2)
        lo, hi = tp.stats['strategy']['thickness']
        self.assertTrue(0 < lo < 4 < hi)

    def _outer_wall(self, tp):
        L = len(tp.layer_z) * 2 // 3
        m = np.nonzero((tp.layer == L) & (tp.kind == KIND_OUTER))[0]
        m = m[np.diff(tp.e, prepend=0)[m] > 0]
        P = np.column_stack([tp.x, tp.y, tp.z])
        seg = np.linalg.norm(P[m] - P[m - 1], axis=1)
        keep = seg > 0.5
        m, seg = m[keep], seg[keep]
        de = tp.e[m] - tp.e[m - 1]
        x = (P[m, 0] + P[m - 1, 0]) / 2
        return m, seg, de, x

    def test_thickness_by_speed(self):
        # by speed: the extruder runs steady (material per second the same) and the robot slows down where the layer
        # is thicker (the outside of the bend, -X); by flow the speed stays and the material per second changes
        tp, _ = curve_job(curve_thickness_by='speed')
        m, seg, de, x = self._outer_wall(tp)
        rate = de / (seg / tp.f[m])
        self.assertLess(rate.max() / rate.min(), 1.05)
        outside, inside = tp.f[m][x < np.percentile(x, 20)].mean(), tp.f[m][x > np.percentile(x, 80)].mean()
        self.assertLess(outside / inside, 0.85)
        m, seg, de, x = self._outer_wall(self.tp)               # by flow (the default)
        self.assertLess(np.ptp(self.tp.f[m]), 1e-9)
        rate = de / (seg / self.tp.f[m])
        self.assertGreater(rate.max() / rate.min(), 1.2)
        self.assertAlmostEqual(tp.stats['e_total'], self.tp.stats['e_total'], delta=1e-6 * self.tp.stats['e_total'])

    def test_bead_thickness_shown(self):
        from yora import server
        tp = self.tp
        self.assertEqual(len(tp.thick), len(tp))
        lo, hi = tp.stats['strategy']['thickness']
        ext = np.isin(tp.kind, EXTRUDING) & (np.diff(tp.e, prepend=0) > 0)
        self.assertAlmostEqual(float(tp.thick[ext].min()), lo, places=3)
        self.assertAlmostEqual(float(tp.thick[ext].max()), hi, places=3)
        buf = np.frombuffer(server.preview_buffer(tp, 'extrude'), dtype=np.float32).reshape(-1, 13)
        self.assertGreater(np.ptp(buf[:, 11]), 1.0)              # the preview draws each bead at its own thickness

    def test_nozzle_tilts_with_each_layer(self):
        tp = self.tp
        self.assertEqual(tp.tilts.shape[0], len(tp.layer_z))
        i = int(np.nonzero(tp.layer == len(tp.layer_z) - 1)[0][0])
        T = strategy.tool_rotation(tp, i)
        np.testing.assert_allclose(T[:3, 2], tp.stats['strategy']['normals'][-1], atol=1e-4)
        self.assertLess(float(tp.overhang.max()), 30.0)        # each layer sits on the one below

    def test_kuka_program_carries_the_tilt(self):
        tp, s = self.tp, self.s
        d = post.normalize(dict(post.BUILTIN['KUKA KRL (KR C4)'], name='KUKA KRL (KR C4)'))
        path, _ = post.export_program(tp, s, tempfile.mkdtemp(), 'Curve Test', None, d)
        with zipfile.ZipFile(path) as z:
            src = z.read('Curve_Test.src').decode('utf-8')
        num = r'\{X ([-\d.]+),Y ([-\d.]+),Z ([-\d.]+),A ([-\d.]+),B ([-\d.]+),C ([-\d.]+)\}'
        lins = [[float(v) for v in m] for m in re.findall('LIN ' + num, src)]
        tool_x = [K.rotz(math.radians(a)) @ K.roty(math.radians(b)) @ K.rotx(math.radians(c)) for _, _, _, a, b, c in lins]
        down = [-T[2, 0] for T in tool_x]                       # tool X out of the nozzle: how much it points down
        self.assertGreater(max(down), 0.999)                    # straight down at the start
        self.assertLess(min(down), math.cos(math.radians(0.8 * BEND)))   # tilted near the top

    @unittest.skipUnless('BORUNTE BRTIRUS 18020A' in K.list_robots(), 'robot model not imported yet')
    def test_robot_holds_each_layers_tilt(self):
        from yora import robotpath as RP
        tp, s = self.tp, self.s
        robot = K.load_robot('BORUNTE BRTIRUS 18020A')
        rp = RP.solve_robot_path(tp, s, robot)
        bed, tcp = RP.cell_frames(s)
        for i in (len(tp) // 3, len(tp) - 50):
            if rp.err[i] & RP.ERR_REACH:
                continue
            T = np.linalg.inv(bed) @ robot.fk(rp.joints[i]) @ tcp            # TCP in the bed frame
            np.testing.assert_allclose(T[:3, 2], -strategy.tool_rotation(tp, i)[:3, 2], atol=1e-3)   # nozzle: -normal
            np.testing.assert_allclose(T[:3, 3], [tp.x[i], tp.y[i], tp.z[i]], atol=0.05)

    def test_straight_part_is_like_flat_layers(self):
        box = mesh.place(mesh.box_mesh(120, 100, 60), center_xy=(500, 500))
        s = settings.normalize({'layer_height': 4, 'first_layer_height': 4, 'print_strategy': 'curve'})
        tp = yora.slice_to_toolpath(box, s)
        flat = yora.slice_to_toolpath(box, settings.normalize({'layer_height': 4, 'first_layer_height': 4}))
        self.assertEqual(len(tp.layer_z), len(flat.layer_z))
        np.testing.assert_allclose(np.array(tp.stats['strategy']['normals']), [[0, 0, 1]] * len(tp.layer_z), atol=1e-6)
        self.assertAlmostEqual(tp.stats['volume_cm3'], flat.stats['volume_cm3'], delta=0.03 * flat.stats['volume_cm3'])

    def test_base_then_curve(self):
        tp, _ = curve_job(base_height=40)
        nb = tp.stats['strategy']['flat_layers']
        self.assertEqual(nb, 10)
        ext = np.isin(tp.kind, EXTRUDING)
        self.assertLessEqual(tp.z[ext & (tp.layer < nb)].max(), 40 + 1e-6)
        self.assertTrue((tp.orient[tp.layer < nb] == 0).all())

    def test_given_points(self):
        pts = '500,500,0; 500,500,60; 560,500,200'
        tp, s = curve_job(curve_points=pts)
        self.assertFalse(tp.stats['strategy']['auto'])
        np.testing.assert_allclose(tp.stats['strategy']['curve_points'], curve.parse_points(pts))

    def test_preview(self):
        out = curve.preview(bent_bar(), self.s)
        self.assertTrue(out['auto'])
        self.assertGreater(len(out['planes']), 5)
        self.assertGreater(len(out['line']), 10)


def outer_starts(tp):
    """Where each layer's outer wall loop starts (its seam)."""
    P = np.column_stack([tp.x, tp.y, tp.z])
    out = []
    for L in range(len(tp.layer_z)):
        m = np.nonzero((tp.layer == L) & (tp.kind == KIND_OUTER))[0]
        if len(m):
            out.append(P[m[0] - 1])
    return np.array(out)


class CurveSourceTest(unittest.TestCase):

    def test_follow_an_edge(self):
        T = bent_bar(n_ring=4)                                  # square bar: four sharp edges along the bend
        th = math.radians(25)                                   # a click on the corner on the +X side, part way up the bend
        click = [500 + BEND_R * (1 - math.cos(th)) + 40 * math.cos(th) + 1.0, 500, LEAD + (BEND_R - 40) * math.sin(th)]
        P = curve.edge_chain(T, click)
        self.assertLess(P[0, 2], 1e-6)                          # from the bed ...
        self.assertGreater(P[-1, 2], LEAD + 0.8 * BEND_R * math.sin(math.radians(BEND)))   # ... to the top
        np.testing.assert_allclose(centreline_distance(P), 40.0, atol=0.05)              # all along that corner
        Q = curve.simplify_points(P)
        self.assertLessEqual(len(Q), 16)
        self.assertLess(np.abs(centreline_distance(Q) - 40.0).max(), 0.05)

    def test_round_part_has_no_edge_along_it(self):
        self.assertIsNone(curve.edge_chain(bent_bar(), [540, 500, 150]))   # only the end circles are edges, far away
        P = curve.edge_chain(bent_bar(), [500, 540, 0])                     # on the foot's circle: that circle
        self.assertLess(np.ptp(P[:, 2]), 1e-6)

    def test_simplify_keeps_the_line(self):
        th = np.linspace(0, math.pi / 2, 400)
        P = np.column_stack([100 * np.sin(th), np.zeros(400), 100 * (1 - np.cos(th))])[::-1]
        Q = curve.simplify_points(curve.bottom_up(P), max_points=12)
        self.assertLessEqual(len(Q), 12)
        self.assertLess(Q[0, 2], Q[-1, 2])                      # bottom to top
        r = np.hypot(Q[:, 0], Q[:, 2] - 100)
        np.testing.assert_allclose(r, 100, atol=1e-6)

    def test_guide_part(self):
        s = settings.normalize({'layer_height': 4})
        P = curve.from_part(bent_bar(r=8.0), s)                 # a thin rod with the same bend
        self.assertLess(centreline_distance(P).max(), 3.0)


class CurveSeamTest(unittest.TestCase):

    def test_drawn_seam_line(self):
        th = np.linspace(0, math.radians(BEND), 30)
        line = np.vstack([[[460, 500, z] for z in np.linspace(0, LEAD, 5)],
                          np.column_stack([500 + BEND_R - (BEND_R + 40) * np.cos(th), np.full(30, 500),
                                           LEAD + (BEND_R + 40) * np.sin(th)])])     # along the outside of the bend
        tp, _ = curve_job(seam='path', seam_path='; '.join('%.2f,%.2f,%.2f' % tuple(p) for p in line))
        d = np.array([np.linalg.norm(line - p, axis=1).min() for p in outer_starts(tp)])
        self.assertLess(np.median(d), 6.0)                      # the wall runs half a bead inside the surface
        self.assertLess(d.max(), 10.0)
        self.assertFalse(any('aligned' in w for w in tp.warnings))

    def test_picked_seam_point(self):
        tp, _ = curve_job(seam='custom', seam_x=460, seam_y=500)    # outside of the bend, at the foot
        st = outer_starts(tp)
        self.assertLess(np.ptp(st[:, 1]), 2.0)                  # stays on that side all the way up the bend
        self.assertLess(st[0, 0], 470)
        self.assertGreater(st[-1, 0], 560)


def box_job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4, 'line_width': 6, 'walls': 1,
                                 'infill_density': 0, 'top_layers': 0, 'bottom_layers': 0}, **kw))
    return yora.slice_to_toolpath(mesh.place(mesh.box_mesh(100, 80, 24), center_xy=(500, 500)), s), s


def outer_loops(tp):
    """Signed area and x extent of the outer wall loop of each layer."""
    out = []
    for L in range(len(tp.layer_z)):
        m = (tp.layer == L) & (tp.kind == KIND_OUTER)
        p = np.column_stack([tp.x[m], tp.y[m]])
        out.append((signed_area(p), np.ptp(p[:, 0])))
    return out


class WallTest(unittest.TestCase):

    def test_placement(self):
        inside = outer_loops(box_job()[0])
        centered = outer_loops(box_job(wall_placement='centered')[0])
        self.assertAlmostEqual(inside[2][1], 100 - 6, delta=0.3)      # bead inside: its middle half a bead in
        self.assertAlmostEqual(centered[2][1], 100, delta=0.3)       # middle of the bead on the surface

    def test_direction(self):
        ccw = [a for a, _ in outer_loops(box_job()[0])]
        cw = [a for a, _ in outer_loops(box_job(wall_direction='cw')[0])]
        alt = [a for a, _ in outer_loops(box_job(wall_direction='alternate')[0])]
        self.assertTrue(all(a > 0 for a in ccw))
        self.assertTrue(all(a < 0 for a in cw))
        self.assertEqual([a > 0 for a in alt], [L % 2 == 0 for L in range(len(alt))])

    def test_per_layer_numbers(self):
        tp, _ = box_job()
        pl = tp.stats['per_layer']
        n = len(tp.layer_z)
        for k in ('time_s', 'path_mm', 'travel_mm', 'height_mm', 'mass_g', 'flow_mm3s'):
            self.assertEqual(len(pl[k]), n, k)
        self.assertAlmostEqual(sum(pl['time_s']), tp.stats['time_s'], delta=0.01 * tp.stats['time_s'])
        self.assertAlmostEqual(sum(pl['path_mm']), tp.stats['extrude_mm'], delta=1.0)
        self.assertAlmostEqual(sum(pl['mass_g']), tp.stats['mass_g'], delta=0.01 * tp.stats['mass_g'] + 0.1)
        np.testing.assert_allclose(pl['height_mm'], 4.0)


if __name__ == '__main__':
    unittest.main()
