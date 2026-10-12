# Run from the project folder:
#   python -m unittest discover -s tests -v
import io
import json
import math
import os
import sys
import tempfile
import threading
import unittest
import urllib.request
import zipfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import gcode, mesh, robot, settings  # noqa: E402
from yora.planner import EXTRUDING  # noqa: E402
from yora.slicer import (KIND_BRIM, KIND_INFILL, KIND_INNER, KIND_OUTER, KIND_RETRACT, KIND_SOLID, KIND_TRAVEL,  # noqa: E402
                           Slicer, signed_area)


def S(**kw):
    return settings.normalize(kw)


def slice_box(sx=100, sy=60, sz=20, **kw):
    s = S(**kw)
    tris = mesh.place(mesh.box_mesh(sx, sy, sz), center_xy=(500, 500))
    return yora.slice_to_toolpath(tris, s), s


class MeshTest(unittest.TestCase):

    def test_stl_binary_and_ascii(self):
        box = mesh.box_mesh(10, 20, 30)
        self.assertEqual(mesh.load_mesh(mesh.to_stl_bytes(box), 'a.stl').shape, (12, 3, 3))
        ascii_stl = 'solid t\n' + ''.join(
            'facet normal 0 0 0\nouter loop\n' + ''.join('vertex %f %f %f\n' % tuple(v) for v in t) + 'endloop\nendfacet\n'
            for t in box) + 'endsolid t\n'
        np.testing.assert_allclose(mesh.load_mesh(ascii_stl.encode(), 'a.stl'), box)

    def test_obj_and_3mf(self):
        obj = 'v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n'
        self.assertEqual(mesh.load_mesh(obj.encode(), 'q.obj').shape, (2, 3, 3))
        model = ('<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
                 '<object id="1" type="model"><mesh><vertices>'
                 '<vertex x="0" y="0" z="0"/><vertex x="10" y="0" z="0"/><vertex x="0" y="10" z="0"/></vertices>'
                 '<triangles><triangle v1="0" v2="1" v3="2"/></triangles></mesh></object></resources>'
                 '<build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 5 6 7"/></build></model>')
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w') as z:
            z.writestr('3D/3dmodel.model', model)
        t = mesh.load_mesh(buf.getvalue(), 'm.3mf')
        np.testing.assert_allclose(t[0, 1], [15, 6, 7])

    def test_place_on_bed(self):
        t = mesh.place(mesh.box_mesh(10, 20, 30), (200, 200, 200), (0, 0, 90), (100, 50))
        lo, hi = mesh.bounds(t)
        np.testing.assert_allclose(lo, [80, 40, 0], atol=1e-9)
        np.testing.assert_allclose(hi, [120, 60, 60], atol=1e-9)


def walls_vs_outline(tris, s, layer=2):
    """Distances of points along the outer/inner walls of one layer to the exact cross-section, and the number of
    zig-zags (turn direction flips > 2 deg) of the outer wall."""
    sl = Slicer(tris, s)
    z, h = sl.layers_z[layer]
    loops = sl.section(z - h / 2)
    A = np.vstack(loops)
    B = np.vstack([np.roll(l, -1, axis=0) for l in loops])
    lay = sl.build_layer(layer, None)
    out = {}
    flips = 0
    for st in lay.stages:
        for stage in st:
            for p in stage:
                if p.kind not in (KIND_OUTER, KIND_INNER):
                    continue
                q = np.vstack([p.pts[:, :2], p.pts[:1, :2]])
                seg = np.diff(q, axis=0)
                pts = np.vstack([q[i] + seg[i] * np.linspace(0, 1, max(2, int(np.hypot(*seg[i]) / 0.2)), endpoint=False)[:, None]
                                 for i in range(len(seg))])
                AB = B - A
                t = np.clip(((pts[:, None] - A) * AB).sum(2) / ((AB ** 2).sum(1) + 1e-12), 0, 1)
                d = np.hypot(*(pts[:, None] - (A + t[..., None] * AB)).transpose(2, 0, 1)).min(1)
                out.setdefault(p.kind, []).append(d)
                if p.kind == KIND_OUTER:
                    ang = np.arctan2(seg[:, 1], seg[:, 0])
                    dd = (np.diff(np.r_[ang, ang[:1]]) + np.pi) % (2 * np.pi) - np.pi
                    big = dd[np.abs(dd) > math.radians(2)]
                    flips += int(np.count_nonzero(np.diff(np.sign(big)) != 0)) if len(big) > 1 else 0
    return {k: np.concatenate(v) for k, v in out.items()}, flips


class SmoothWallsTest(unittest.TestCase):
    """Walls follow the model's exact cross-section, not the pixel grid of the slicing raster."""

    def check(self, tris, max_flips):
        s = S(line_width=5, walls=2, simplify=0.2)
        d, flips = walls_vs_outline(tris, s)
        np.testing.assert_allclose(np.median(d[KIND_OUTER]), 2.5, atol=0.05)
        np.testing.assert_allclose(np.median(d[KIND_INNER]), 7.5, atol=0.05)
        self.assertLess(np.abs(d[KIND_OUTER] - 2.5).max(), 0.25)          # within the simplify tolerance (+ margin)
        self.assertLessEqual(flips, max_flips)

    def test_rotated_box_has_straight_walls(self):
        # a box at 30 deg is the worst case for a pixel grid (staircase along every edge)
        tris = mesh.place(mesh.box_mesh(200, 120, 10), rotation=(0, 0, 30), center_xy=(500, 500))
        self.check(tris, max_flips=0)

    def test_cylinder_wall_is_round(self):
        tris = mesh.place(mesh.tube_mesh(80, 40, 10), center_xy=(500, 500))
        self.check(tris, max_flips=0)


class SlicerTest(unittest.TestCase):

    def test_layer_count_and_heights(self):
        tp, s = slice_box(sz=20, layer_height=2, first_layer_height=3)
        self.assertEqual(tp.layer_z[:3], [3, 5, 7])
        self.assertAlmostEqual(tp.layer_z[-1], 19)   # a layer is only made if its mid-plane is inside the part
        self.assertEqual(len(tp.layer_z), 9)

    def test_outer_wall_is_half_a_line_inside(self):
        tp, s = slice_box(sx=100, sy=60, walls=2, line_width=5)
        sel = (tp.kind == KIND_OUTER) & (tp.layer == 3)
        x, y = tp.x[sel], tp.y[sel]
        res = 0.5
        self.assertAlmostEqual(x.min(), 450 + 2.5, delta=res)
        self.assertAlmostEqual(x.max(), 550 - 2.5, delta=res)
        self.assertAlmostEqual(y.min(), 470 + 2.5, delta=res)
        inner = (tp.kind == KIND_INNER) & (tp.layer == 3)
        self.assertAlmostEqual(tp.x[inner].min(), 450 + 7.5, delta=res)

    def test_hole_gets_walls_on_both_sides(self):
        s = S(walls=1, infill_density=0, top_layers=0, bottom_layers=0)
        tp = yora.slice_to_toolpath(mesh.place(mesh.tube_mesh(50, 30, 10), center_xy=(0, 0)), s)
        sel = (tp.kind == KIND_OUTER) & (tp.layer == 2)
        r = np.hypot(tp.x[sel], tp.y[sel])
        self.assertAlmostEqual(r.max(), 47.5, delta=0.6)
        self.assertAlmostEqual(r.min(), 32.5, delta=0.6)

    def test_overlapping_shells_merge(self):
        # Two boxes sharing a corner: the overlap must be solid, not a hole (non-zero winding)
        tris = np.concatenate([mesh.box_mesh(200, 60, 30), mesh.box_mesh(60, 200, 30)])
        sl = Slicer(tris, S())
        loops = sl.section(15)
        self.assertEqual(sorted(round(signed_area(l)) for l in loops), [12000, 12000])
        m, g = sl.rasterize(loops), sl.grid
        at = lambda x, y: m[int(round((y - g.y0) / g.res)), int(round((x - g.x0) / g.res))]
        self.assertEqual((at(30, 30), at(150, 30), at(30, 150), at(150, 150)), (1, 1, 1, 0))

    def test_inverted_mesh_still_slices(self):
        box = mesh.box_mesh(50, 50, 10)[:, ::-1, :]   # all normals inward
        tp = yora.slice_to_toolpath(mesh.place(box, center_xy=(100, 100)), S())
        self.assertGreater(tp.stats['extrude_mm'], 0)

    def test_solid_top_and_bottom_sparse_middle(self):
        tp, s = slice_box(sz=30, layer_height=2, first_layer_height=2, top_layers=2, bottom_layers=2, infill_density=20)
        solid_layers = sorted(set(tp.layer[tp.kind == KIND_SOLID].tolist()))
        sparse_layers = sorted(set(tp.layer[tp.kind == KIND_INFILL].tolist()))
        n = len(tp.layer_z)
        self.assertEqual(solid_layers, [0, 1, n - 2, n - 1])
        self.assertEqual(sparse_layers, list(range(2, n - 2)))

    def test_infill_density_scales_length(self):
        a, _ = slice_box(sz=10, top_layers=0, bottom_layers=0, infill_density=20)
        b, _ = slice_box(sz=10, top_layers=0, bottom_layers=0, infill_density=40)
        ratio = b.stats['by_kind_mm']['sparse infill'] / a.stats['by_kind_mm']['sparse infill']
        self.assertGreater(ratio, 1.6)
        self.assertLess(ratio, 2.4)

    def test_connected_infill_has_fewer_paths(self):
        a, _ = slice_box(sz=6, connect_infill=True)
        b, _ = slice_box(sz=6, connect_infill=False)
        travels = lambda tp: int(np.count_nonzero(tp.kind == KIND_TRAVEL))
        self.assertLess(travels(a), travels(b))

    def test_concentric_infill(self):
        tp, _ = slice_box(sz=10, top_layers=0, bottom_layers=0, infill_pattern='concentric', infill_density=30)
        self.assertGreater(tp.stats['by_kind_mm']['sparse infill'], 0)

    def test_brim_only_on_first_layer(self):
        tp, _ = slice_box(sz=10, brim_loops=3)
        self.assertEqual(set(tp.layer[tp.kind == KIND_BRIM].tolist()), {0})
        self.assertAlmostEqual(tp.x[tp.kind == KIND_BRIM].min(), 450 - (3 + 2 * 6), delta=0.6)   # first layer width 6 mm

    def test_spiral_vase_rises_continuously(self):
        s = S(spiral_vase=True, bottom_layers=2, walls=1, layer_height=2, first_layer_height=2)
        tp = yora.slice_to_toolpath(mesh.place(mesh.tube_mesh(80, 0.0001, 40, 64), center_xy=(0, 0)), s)
        sel = (tp.layer >= 3) & (tp.kind == KIND_OUTER)
        z = tp.z[sel]
        self.assertTrue((np.diff(z) >= -1e-9).all(), 'Z must never go down along the spiral')
        travel = (tp.kind == KIND_TRAVEL) & (tp.layer >= 3)
        travel[-1] = False   # final retreat move
        self.assertEqual(int(np.count_nonzero(travel)), 0)

    def test_thin_region_warning(self):
        tp, _ = slice_box(sx=3, sy=100, sz=6, line_width=5)
        self.assertTrue(any('thinner' in w for w in tp.warnings))


class PlannerTest(unittest.TestCase):

    def test_e_is_monotonic_without_retraction(self):
        tp, _ = slice_box(sz=10)
        self.assertTrue((np.diff(tp.e) >= -1e-9).all())

    def test_e_per_mm_volumetric(self):
        tp, s = slice_box(sz=10, flow=100, e_mode='volumetric', line_width=5, layer_height=2)
        de, seg = np.diff(tp.e), np.sqrt(np.diff(tp.x) ** 2 + np.diff(tp.y) ** 2 + np.diff(tp.z) ** 2)
        k = (tp.kind[1:] == KIND_OUTER) & (tp.layer[1:] == 3) & (seg > 1)
        area = (5 - 2) * 2 + math.pi * 1.0 ** 2
        np.testing.assert_allclose(de[k] / seg[k], area, rtol=1e-6)

    def test_retract_and_hop_on_long_travel(self):
        tp, s = slice_box(sz=6, retract_length=2.0, retract_min_travel=5, z_hop=4)
        self.assertGreater(int(np.count_nonzero(np.diff(tp.e) < -1.9)), 0)
        self.assertGreater(tp.z.max(), tp.layer_z[-1] + 3.9)

    def test_travel_never_extrudes(self):
        tp, _ = slice_box(sz=10)
        de = np.diff(tp.e)
        self.assertTrue((de[tp.kind[1:] == KIND_TRAVEL] == 0).all())

    def test_flow_levels_rate_mode(self):
        tp, s = slice_box(sz=20, flow_mode='rate', outer_wall_speed=25, infill_speed=50, flow_full_scale=0)
        lv_outer = set(tp.level[(tp.kind == KIND_OUTER) & (tp.layer == 4)].tolist())
        lv_infill = set(tp.level[(tp.kind == KIND_INFILL) & (tp.layer == 4)].tolist())
        self.assertEqual(lv_infill, {7})
        self.assertEqual(lv_outer, {4})   # half the speed -> ceil(3.5)

    def test_stats(self):
        tp, s = slice_box(sx=100, sy=100, sz=10, walls=1, infill_density=100, top_layers=0, bottom_layers=0)
        vol = 100 * 100 * 10 / 1000.0
        self.assertGreater(tp.stats['volume_cm3'], vol * 0.85)
        self.assertLess(tp.stats['volume_cm3'], vol * 1.3)
        self.assertGreater(tp.stats['time_s'], 0)


class GcodeTest(unittest.TestCase):

    def test_roundtrip(self):
        tp, s = slice_box(sz=10, retract_length=1.0)
        tp2, info = gcode.read_gcode(gcode.write_gcode(tp, s), s)
        self.assertEqual(len(tp2.layer_z), len(tp.layer_z))
        self.assertAlmostEqual(tp2.e[-1], tp.e[-1], places=3)
        ext = lambda t: np.isin(t.kind, EXTRUDING).sum()
        self.assertEqual(ext(tp2), ext(tp))

    def test_relative_e_g92_and_types(self):
        g = '\n'.join(['M83', 'G92 E0', ';LAYER_CHANGE', ';TYPE:External perimeter',
                       'G1 X0 Y0 Z0.2 F600', 'G1 X10 E1', 'G1 Y10 E1', ';TYPE:Internal infill', 'G1 X0 E1',
                       'G1 E-0.8', 'G0 X20 Y20', 'G1 E0.8', ';LAYER_CHANGE', 'G1 Z0.4', 'G1 X30 E1'])
        tp, info = gcode.read_gcode(g, S())
        self.assertEqual(len(tp.layer_z), 2)
        self.assertAlmostEqual(tp.e[-1], 4.0)
        self.assertEqual(list(tp.kind[:5]), [KIND_TRAVEL, KIND_OUTER, KIND_OUTER, KIND_INFILL, KIND_RETRACT])
        self.assertEqual(tp.f[1], 10.0)

    def test_absolute_e_with_resets_is_monotonic(self):
        g = 'M82\nG1 X0 Y0 Z0.2\nG1 X10 E2\nG92 E0\nG1 X20 E2\n'
        tp, _ = gcode.read_gcode(g, S())
        self.assertAlmostEqual(tp.e[-1], 4.0)

    def test_arcs(self):
        g = 'G90\nM82\nG1 X10 Y0 Z0.2\nG3 X-10 Y0 I-10 J0 E5\n'   # CCW half circle through +Y
        tp, _ = gcode.read_gcode(g, S())
        self.assertGreater(len(tp), 5)
        self.assertAlmostEqual(tp.y.max(), 10, delta=0.05)
        self.assertAlmostEqual(tp.e[-1], 5)


class RobotTest(unittest.TestCase):

    def test_act_export(self):
        tp, s = slice_box(sz=6, start_joints='0,0,0,0,90,0', user_frame_id=3, tool_id=2, flow_level_dos='5,6')
        out = tempfile.mkdtemp()
        path, info = robot.export_act(tp, s, out, 'box test')
        self.assertTrue(path.endswith('box_test.zip'))
        z = zipfile.ZipFile(path)
        prog = json.loads(z.read('box_test.act').decode().splitlines()[0])
        acts = [i['action'] for i in prog]
        self.assertEqual(acts[:2], [800, 801])
        self.assertEqual(prog[0]['coordID'], 3)
        self.assertEqual(prog[1]['toolID'], 2)
        self.assertEqual(acts[2], 4)                      # MoveJ to start joints
        self.assertEqual(acts[-1], 60000)
        dos = {i['point'] for i in prog if i['action'] == 200}
        self.assertEqual(dos, {1, 5, 6})
        moves = [i for i in prog if i['action'] == 10]
        self.assertEqual(moves[0]['points'][0]['pos']['m3'], '180.000')   # tool Z points down
        self.assertEqual(info['post_settings']['FLOW_LEVEL_DOS'], [5, 6])

    def test_post_settings_file(self):
        RobotPost = robot.load_post_class()
        r = RobotPost('Borunte_FGF', 'BRTIRUS1820A', 6)
        d = tempfile.mkdtemp()
        p = os.path.join(d, 'Borunte_FGF.json')
        with open(p, 'w') as f:
            json.dump({'EXTRUDER_ENABLE_DO': 9, 'FLOW_MODE': 'rate', 'not_a_setting': 1, 'PG': []}, f)
        applied = r.load_settings_file(p)
        self.assertEqual(applied, {'EXTRUDER_ENABLE_DO': 9, 'FLOW_MODE': 'rate'})
        self.assertEqual(r.EXTRUDER_ENABLE_DO, 9)


class ServerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from http.server import ThreadingHTTPServer
        from yora import server
        cls.server_mod = server
        cls.httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        cls.base = 'http://127.0.0.1:%i' % cls.httpd.server_address[1]
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()

    def req(self, path, body=None, raw=False):
        data = None if body is None else (body if raw else json.dumps(body).encode())
        r = urllib.request.urlopen(urllib.request.Request(self.base + path, data=data, method='POST' if data is not None else 'GET'), timeout=120)
        return r

    def test_full_flow(self):
        spec = json.load(self.req('/api/spec'))
        self.assertIn('layer_height', spec['defaults'])
        s = dict(spec['defaults'], layer_height=3, first_layer_height=3)
        q = '/api/model?name=box.stl&settings=' + urllib.request.quote(json.dumps(s))
        info = json.load(self.req(q, mesh.to_stl_bytes(mesh.box_mesh(120, 80, 12)), raw=True))['model']
        self.assertTrue(info['fits'])
        self.assertEqual(len(self.req('/api/model.bin').read()), 12 * 9 * 4)
        self.req('/api/slice', {'settings': s})
        import time
        for _ in range(200):
            st = json.load(self.req('/api/status'))
            if st['state'] != 'running':
                break
            time.sleep(0.05)
        self.assertEqual(st['state'], 'done', st.get('error'))
        res = st['result']
        n_ext = res['extrude_offsets'][-1]
        self.assertEqual(len(self.req("/api/toolpath.bin?kind=extrude").read()), n_ext * 13 * 4)
        r = self.req('/api/export/act', {'settings': s})
        self.assertEqual(r.read()[:2], b'PK')
        r = self.req('/api/export/gcode', {'settings': s})
        self.assertIn(b'M82', r.read())

    def test_static_is_confined(self):
        with self.assertRaises(urllib.error.HTTPError):
            self.req('/../yora/server.py')


if __name__ == '__main__':
    unittest.main()



class SeamTest(unittest.TestCase):
    """Seam position, corner snapping, ramp (scarf) layer change, seam gap / overlap and wipe."""

    @staticmethod
    def job(tris=None, **kw):
        s = S(**dict({'layer_height': 4, 'first_layer_height': 4, 'walls': 1, 'infill_density': 0, 'top_layers': 0,
                      'bottom_layers': 0, 'retract_length': 0, 'z_hop': 0}, **kw))
        tris = tris if tris is not None else mesh.place(mesh.box_mesh(100, 60, 20), center_xy=(500, 500))
        return yora.slice_to_toolpath(tris, s), s

    @staticmethod
    def outer_loops(tp):
        """Per layer: indices of the outer-wall extrusion moves, and the index of the move before the first one."""
        ext = np.r_[False, np.diff(tp.e) > 1e-9]
        out = {}
        for i in np.nonzero(ext & (tp.kind == KIND_OUTER))[0]:
            out.setdefault(int(tp.layer[i]), []).append(int(i))
        return out

    def test_aligned_seam_is_on_one_line(self):
        tp, s = self.job(mesh.place(mesh.tube_mesh(80, 60, 30), center_xy=(500, 500)))
        # the tube has two outer walls per layer (outside and around the hole): check the loop starts on the outside
        ext = np.r_[False, np.diff(tp.e) > 1e-9]
        first = np.nonzero(ext & (tp.kind == KIND_OUTER) & ~np.r_[False, ext[:-1]])[0] - 1
        r = np.hypot(tp.x[first] - 500, tp.y[first] - 500)
        sel = first[(r > 70) & (tp.layer[first] > 0)]
        starts = np.column_stack([tp.x[sel], tp.y[sel]])
        self.assertGreater(len(starts), 5)
        self.assertLess(np.hypot(*np.diff(starts, axis=0).T).max(), 0.05)

    def test_custom_seam_point(self):
        tp, s = self.job(seam='custom', seam_x=700, seam_y=505)
        for L, m in self.outer_loops(tp).items():          # (layer 1 is wider: 120 % first layer width)
            if L > 0:
                np.testing.assert_allclose([tp.x[m[0] - 1], tp.y[m[0] - 1]], [547.5, 505.0], atol=0.05)   # right edge, closest point

    def test_seam_moves_to_corner(self):
        tp, s = self.job(seam='custom', seam_x=700, seam_y=505, seam_corner=40)
        for L, m in self.outer_loops(tp).items():
            if L > 0:
                np.testing.assert_allclose([tp.x[m[0] - 1], tp.y[m[0] - 1]], [547.5, 527.5], atol=0.05)   # nearest corner

    def test_ramp_continues_into_the_next_layer(self):
        step, s = self.job(seam='custom', seam_x=700, seam_y=500)
        tp, s = self.job(seam='custom', seam_x=700, seam_y=500, layer_change='ramp', ramp_length=30)
        self.assertAlmostEqual(tp.e[-1], step.e[-1], delta=1e-6 * step.e[-1])            # same material as a step
        loops = self.outer_loops(tp)
        top = max(loops)
        for L in range(1, top):
            m, nxt = loops[L], loops[L + 1]
            end = (tp.x[m[-1]], tp.y[m[-1]], tp.z[m[-1]])
            np.testing.assert_allclose(end, (547.5, 500.0, tp.layer_z[L]), atol=1e-6)     # loop ends at the seam ...
            self.assertEqual(nxt[0] - 1, m[-1])                                           # ... and the next layer starts
            #                                                                               right there (no travel)
        m = loops[2]
        de = np.diff(tp.e[[m[0] - 1] + m])
        dl = np.hypot(np.diff(tp.x[[m[0] - 1] + m]), np.diff(tp.y[[m[0] - 1] + m]))
        np.testing.assert_allclose(de / dl, de[-1] / dl[-1], rtol=1e-6)                  # full flow on the stacked ramp
        x, y = tp.x[[loops[top][0] - 1] + loops[top]], tp.y[[loops[top][0] - 1] + loops[top]]
        self.assertAlmostEqual(np.hypot(np.diff(x), np.diff(y)).sum(), 300 + 30, delta=1e-6)   # top: wedge filled once

    def test_ramp_joins_layers_when_the_wall_shifts(self):
        # cone: every layer's wall is smaller than the one below, so the ramp must also move sideways onto it
        n, r0, r1, H = 96, 80.0, 50.0, 40.0
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        bot = np.column_stack([r0 * np.cos(a), r0 * np.sin(a), np.zeros(n)])
        top = np.column_stack([r1 * np.cos(a), r1 * np.sin(a), np.full(n, H)])
        tris = []
        for i in range(n):
            j = (i + 1) % n
            tris += [[bot[i], bot[j], top[j]], [bot[i], top[j], top[i]], [[0, 0, 0], bot[j], bot[i]], [[0, 0, H], top[i], top[j]]]
        tris = mesh.place(np.array(tris), center_xy=(500, 500))
        tp, s = self.job(tris, layer_change='ramp', ramp_length=30)
        loops = self.outer_loops(tp)
        for L in range(1, max(loops)):
            m, nxt = loops[L], loops[L + 1]
            self.assertEqual(nxt[0] - 1, m[-1])                     # next layer starts where this loop ended ...
            p_end = np.array([tp.x[m[-1]], tp.y[m[-1]]])
            p_on = np.array([tp.x[nxt[-1]], tp.y[nxt[-1]]])          # ... and ends on its own, smaller wall
            r_end, r_on = np.hypot(*(p_end - 500)), np.hypot(*(p_on - 500))
            self.assertGreater(r_end - r_on, 0.5)

    def test_scarf_rises_over_its_length_and_keeps_volume(self):
        step, s = self.job(seam='custom', seam_x=700, seam_y=500)
        tp, s = self.job(seam='custom', seam_x=700, seam_y=500, layer_change='scarf', ramp_length=30)
        self.assertAlmostEqual(tp.e[-1], step.e[-1], delta=1e-6 * step.e[-1])                    # wedge fill = ramp deficit
        m = self.outer_loops(tp)[2]
        z_layer = tp.layer_z[2]
        z = tp.z[[m[0] - 1] + m]
        self.assertAlmostEqual(z[0], z_layer - 4, places=6)                                          # starts one layer down
        x, y = tp.x[[m[0] - 1] + m], tp.y[[m[0] - 1] + m]
        arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
        k = int(np.argmax(z >= z_layer - 1e-9))
        self.assertAlmostEqual(arc[k], 30, delta=1e-6)                                               # full height after 30 mm
        self.assertTrue(np.all(np.diff(z[:k + 1]) > 0))
        self.assertAlmostEqual(arc[-1], 320 - 2 * 5 * 2 + 30, delta=1e-6)                            # loop + wedge fill
        first = self.job(layer_change='scarf')[0]
        self.assertTrue(np.all(first.z[self.outer_loops(first)[0]] == 4))                           # no ramp on layer 1

    def test_seam_gap_and_overlap(self):
        per = 2 * (95 + 55)
        for gap, length in ((5, per - 5), (-8, per + 8)):
            tp, s = self.job(seam_gap=gap)
            m = self.outer_loops(tp)[1]
            x, y = tp.x[[m[0] - 1] + m], tp.y[[m[0] - 1] + m]
            self.assertAlmostEqual(np.hypot(np.diff(x), np.diff(y)).sum(), length, delta=1e-6)

    def test_wipe_after_loop(self):
        tp, s = self.job(wipe_length=12)
        m = self.outer_loops(tp)[1]
        j = m[-1] + 1
        wipe = [j]
        while j + 1 < len(tp) and tp.kind[j + 1] == KIND_TRAVEL and tp.z[j + 1] == tp.z[m[-1]] and \
                np.hypot(tp.x[j + 1] - tp.x[j], tp.y[j + 1] - tp.y[j]) > 0 and len(wipe) < 10:
            j += 1
            wipe.append(j)
        self.assertEqual(tp.kind[wipe[0]], KIND_TRAVEL)
        self.assertEqual(tp.e[wipe[0]], tp.e[m[-1]])                                              # no extrusion
        d = np.hypot(tp.x[m[-1]] - tp.x[wipe[0]], tp.y[m[-1]] - tp.y[wipe[0]])
        self.assertGreater(d, 0)
