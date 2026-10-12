# Robot model files (.robot): header, kinematics, 3D link meshes, collision hulls, import.
# Uses the robot files kept in profiles/robots/.  Run from the project folder:
#   python -m unittest discover -s tests -v
import os
import shutil
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from yora import kinematics as K, robotpath as RP, settings  # noqa: E402
from yora.robot_file import RobotFile, find_robot_files, peek  # noqa: E402

ROBOTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'profiles', 'robots')
BORUNTE = os.path.join(ROBOTS, 'BORUNTE-BRTIRUS-18020A.robot')
KR120 = os.path.join(ROBOTS, 'KUKA-KR-120-R3100-2.robot')

# Reference kinematics of the BRTIRUS 18020A (measured 2026-10-08): flange and link-frame origins at q = 0, joint axes.
BORUNTE_REF = {
    'flange': [1126.9, 0.0, 1324.6],
    'links': [[0, 0, 494.6], [170, 0, 494.6], [170, 0, 1224.6], [975.5, 0, 1324.6], [975.5, 0, 1324.6], [1126.9, 0, 1324.6]],
    'axes': [[0, 0, 1], [0, -1, 0], [0, -1, 0], [1, 0, 0], [0, -1, 0], [1, 0, 0]],
    'lower': [-150, -90, -60, -180, -115, -360], 'upper': [150, 50, 105, 180, 115, 360],
}


def blob_triangles(blob):
    n = struct.unpack_from('<I', blob)[0]
    return np.frombuffer(blob, '<f4', offset=4 + n).reshape(-1, 3, 3)


def open_edges(tri):
    q = np.round(tri.reshape(-1, 3), 3)
    _, idx = np.unique(q, axis=0, return_inverse=True)
    idx = idx.reshape(-1, 3)
    e = np.sort(np.concatenate([idx[:, [0, 1]], idx[:, [1, 2]], idx[:, [2, 0]]]), axis=1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return (c == 1).sum() / len(c)


@unittest.skipUnless(os.path.isfile(BORUNTE), 'BORUNTE-BRTIRUS-18020A.robot not in profiles/robots')
class RobotFileTest(unittest.TestCase):

    def test_header_and_reference_kinematics(self):
        rf = RobotFile(BORUNTE)
        self.assertEqual((rf.name, rf.n), ('BORUNTE BRTIRUS 18020A', 6))
        self.assertEqual(len(rf.links), 7)                       # base + 6 links
        self.assertEqual(peek(BORUNTE)['name'], rf.name)
        np.testing.assert_allclose(rf.lower, BORUNTE_REF['lower'])
        np.testing.assert_allclose(rf.upper, BORUNTE_REF['upper'])
        m = rf.to_model()
        np.testing.assert_allclose(np.array(m['flange_home'])[:3, 3], BORUNTE_REF['flange'], atol=1e-9)
        np.testing.assert_allclose([np.array(L)[:3, 3] for L in m['links_home']], BORUNTE_REF['links'], atol=1e-9)
        np.testing.assert_allclose(m['axes'], BORUNTE_REF['axes'], atol=1e-9)

    def test_model_matches_its_own_dhm_chain(self):
        rf = RobotFile(BORUNTE)
        r = K.Robot(rf.to_model())
        for q in np.random.default_rng(4).uniform(rf.lower, rf.upper, (50, 6)):
            F = rf.frames(q)
            np.testing.assert_allclose(r.fk(q), F[-1], atol=1e-9)
            for a, b in zip(r.link_frames(q), F):
                np.testing.assert_allclose(a, b, atol=1e-9)

    def test_mesh_blob_and_simplification(self):
        rf = RobotFile(BORUNTE)
        total = sum(len(v) for m in rf.links for _, v in m) // 3
        blob = rf.mesh_blob()
        import json
        n = struct.unpack_from('<I', blob)[0]
        head = json.loads(blob[4:4 + n])
        self.assertEqual(len(head['links']), 7)
        self.assertEqual(len(blob_triangles(blob)), total)              # small enough: the full model is kept
        tri = blob_triangles(rf.mesh_blob(max_tris=total // 3))
        self.assertLessEqual(len(tri), total // 3)
        orig = np.concatenate([v for m in rf.links for _, v in m]).reshape(-1, 3, 3)
        self.assertLess(open_edges(tri), open_edges(orig) + 0.05)        # surfaces stay closed

    def test_hull_bounds_link(self):
        rf = RobotFile(BORUNTE)
        for meshes, hull in zip(rf.links, rf.hulls()):
            if not meshes:
                continue
            v = np.concatenate([m for _, m in meshes])
            h = np.array(hull)
            for d in np.random.default_rng(5).normal(size=(20, 3)):     # lowest point in any direction is close
                self.assertLess(float((v @ d).min() - (h @ d).min()), 0.02 * np.linalg.norm(d))   # hull points are rounded to 0.01 mm
                self.assertGreater(float((v @ d).min() - (h @ d).min()), -0.08 * np.linalg.norm(d) * np.ptp(v, axis=0).max())

    def test_import_and_mesh_collision(self):
        old = K.ROBOTS_DIR
        K.ROBOTS_DIR = tempfile.mkdtemp()
        try:
            r = K.import_robot_file(BORUNTE)
            self.assertTrue(os.path.isfile(K.mesh_path(r.name)))
            r = K.load_robot(r.name)
            self.assertIsNotNone(r.hull)
            s = settings.normalize({})
            sv = RP.Solver(r, s)
            q_ok = r.ik_near(sv.flange(500, 500, 100, s['tool_rz']), r.home)
            self.assertEqual(sv.collision(q_ok), 0)
            q_low = r.ik_near(sv.flange(500, 500, -400, s['tool_rz']), r.home)   # nozzle 400 mm into the bed
            self.assertTrue(sv.collision(q_low) & RP.ERR_COLLISION)
            sv.part_box = (400, 400, 600, 600)
            q_in = r.ik_near(sv.flange(500, 500, 50, s['tool_rz']), r.home)
            self.assertTrue(sv.collision(q_in, printed_z=600) & RP.ERR_PART)  # flange inside a 600 mm part
            self.assertFalse(sv.collision(q_in, printed_z=10) & RP.ERR_PART)
        finally:
            shutil.rmtree(K.ROBOTS_DIR, ignore_errors=True)
            K.ROBOTS_DIR = old

    def test_finds_robot_files(self):
        found = find_robot_files([ROBOTS])
        self.assertIn(os.path.normcase(os.path.normpath(BORUNTE)), [os.path.normcase(f['path']) for f in found])


@unittest.skipUnless(os.path.isfile(KR120), 'KUKA-KR-120-R3100-2.robot not in profiles/robots')
class LargeRobotTest(unittest.TestCase):

    def test_home_joints(self):
        np.testing.assert_allclose(RobotFile(KR120).home, [0, -90, 90, 0, 0, 0], atol=1e-9)
        np.testing.assert_allclose(RobotFile(BORUNTE).home, [0] * 6, atol=1e-9)


if __name__ == '__main__':
    unittest.main()
