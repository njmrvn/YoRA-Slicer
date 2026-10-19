# Run from the project folder:
#   python -m unittest discover -s tests -v
import json
import os
import sys
import tempfile
import unittest
import zipfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import kinematics as K, mesh, robot, robotpath as RP, settings  # noqa: E402

MODEL = 'BORUNTE BRTIRUS 18020A'


def have_model():
    return MODEL in K.list_robots()


def box_job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4}, **kw))
    tp = yora.slice_to_toolpath(mesh.place(mesh.box_mesh(200, 150, 16), center_xy=(500, 500)), s)
    return tp, s


@unittest.skipUnless(have_model(), 'robot model not imported yet')
class KinematicsTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.r = K.load_robot(MODEL)

    def test_pose_convention(self):
        # [x, y, z, rx, ry, rz] (deg) = transl(x, y, z) * rotx(rx) * roty(ry) * rotz(rz)
        v = [100, -50, 300, 20, -35, 70]
        mine = K.xyzrpw_to_pose(v)
        ref = K.transl(*v[:3]) @ K.rotx(np.radians(v[3])) @ K.roty(np.radians(v[4])) @ K.rotz(np.radians(v[5]))
        np.testing.assert_allclose(mine, ref, atol=1e-9)
        np.testing.assert_allclose(K.pose_to_xyzrpw(mine), v, atol=1e-9)

    def test_ik_recovers_random_joints(self):
        rng = np.random.default_rng(7)
        for _ in range(300):
            q = rng.uniform(self.r.lower, self.r.upper)
            q[3] = (q[3] + 180) % 360 - 180
            q[5] = (q[5] + 180) % 360 - 180
            sols = self.r.ik_all(self.r.fk(q))
            self.assertTrue(any(np.abs((s - q + 180) % 360 - 180).max() < 1e-3 for s, _ in sols))

    def test_ik_near_keeps_configuration(self):
        q = np.array([10, -30, 20, 5, 40, -20.0])
        cfg = self.r.config_of(q)
        for dx in (0, 5, 20, 50):
            T = K.transl(dx, 0, 0) @ self.r.fk(q)
            s = self.r.ik_near(T, q, cfg)
            self.assertIsNotNone(s)
            self.assertEqual(self.r.config_of(s), cfg)


@unittest.skipUnless(have_model(), 'robot model not imported yet')
class RobotPathTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.r = K.load_robot(MODEL)

    def test_default_cell_prints_box(self):
        tp, s = box_job()
        rp = RP.solve_robot_path(tp, s, self.r)
        self.assertTrue(rp.summary['ok'], rp.issues[:3])
        # tool really is at each toolpath point, pointing down
        bed, tcp = RP.cell_frames(s)
        inv_bed = K.inv_pose(bed)
        for i in (0, len(tp) // 2, len(tp) - 1):
            T = inv_bed @ self.r.fk(rp.joints[i]) @ tcp
            np.testing.assert_allclose(T[:3, 3], [tp.x[i], tp.y[i], tp.z[i]], atol=0.02)
            self.assertAlmostEqual(T[2, 2], -1.0, places=6)
        # locked configuration: every solution has the same flags
        cfgs = {self.r.config_of(rp.joints[i]) for i in range(0, len(tp), max(1, len(tp) // 40))}
        self.assertEqual(len(cfgs), 1)

    def test_unreachable_bed_is_reported(self):
        tp, s = box_job(bed_frame='2500,-500,0,0,0,0')
        with self.assertRaises(RuntimeError):
            RP.solve_robot_path(tp, s, self.r)

    def test_partly_out_of_reach(self):
        tp, s = box_job(bed_frame='1150,-500,0,0,0,0')
        rp = RP.solve_robot_path(tp, s, self.r)
        self.assertFalse(rp.summary['ok'])
        self.assertGreater(rp.summary['counts']['unreachable'] + rp.summary['counts']['joint limit'], 0)

    def test_singularity_margin(self):
        tp, s = box_job(sing_margin=60)
        rp = RP.solve_robot_path(tp, s, self.r)
        self.assertGreater(rp.summary['counts']['wrist singularity'], 0)

    def test_free_rotation_used_when_needed(self):
        # With no free rotation the solver must stay at tool_rz; with rotation allowed it may move
        tp, s = box_job(rotz_range=0, tool_rz=0)
        rp = RP.solve_robot_path(tp, s, self.r)
        self.assertEqual((rp.summary['rz_min'], rp.summary['rz_max']), (0.0, 0.0))

    def test_collision_with_bed(self):
        tp, s = box_job(floor_margin=2000)   # demand links 2 m above the bed: impossible
        rp = RP.solve_robot_path(tp, s, self.r)
        self.assertGreater(rp.summary['counts']['collision with bed'], 0)

    def test_act_export_uses_solved_path(self):
        tp, s = box_job()
        rp = RP.solve_robot_path(tp, s, self.r)
        path, info = robot.export_act(tp, s, tempfile.mkdtemp(), 'robot_box', rp)
        prog = json.loads(zipfile.ZipFile(path).read('robot_box.act').decode().splitlines()[0])
        movej = [i for i in prog if i['action'] == 4]
        self.assertEqual(len(movej), 1)
        pos = movej[0]['points'][0]['pos']
        np.testing.assert_allclose([float(pos['m%i' % j]) for j in range(6)], rp.joints[0], atol=0.01)
        self.assertTrue(info['robot_checked'])

    def test_reach_map(self):
        s = settings.normalize({})
        m = RP.reach_map(self.r, s, 6, 6)
        self.assertEqual(len(m['ok']), 6)
        self.assertGreater(sum(map(sum, m['ok'])), 0)


if __name__ == '__main__':
    unittest.main()


@unittest.skipUnless(have_model(), 'robot model not imported yet')
class FastPathTest(unittest.TestCase):
    """The vectorised robot check gives the same joints, rotations and problems as solving move by move."""

    @classmethod
    def setUpClass(cls):
        cls.r = K.load_robot(MODEL)

    def same(self, tp, s):
        RP.BATCH = False
        try:
            slow = RP.solve_robot_path(tp, s, self.r)
        finally:
            RP.BATCH = True
        fast = RP.solve_robot_path(tp, s, self.r)
        np.testing.assert_allclose(fast.joints, slow.joints, atol=1e-9)
        np.testing.assert_array_equal(fast.rz, slow.rz)
        np.testing.assert_array_equal(fast.err, slow.err)
        return fast

    def test_problem_free(self):
        self.assertTrue(self.same(*box_job()).summary['ok'])

    def test_out_of_reach(self):
        rp = self.same(*box_job(bed_frame='1150,-500,0,0,0,0'))
        self.assertGreater(rp.summary['counts']['unreachable'] + rp.summary['counts']['joint limit'], 0)

    def test_singularity_and_rotation_search(self):
        rp = self.same(*box_job(sing_margin=60))
        self.assertGreater(rp.summary['counts']['wrist singularity'], 0)

    def test_collisions(self):
        rp = self.same(*box_job(floor_margin=2000))
        self.assertGreater(rp.summary['counts']['collision with bed'], 0)

    def test_unlocked_configuration(self):
        self.same(*box_job(lock_config=False))

    def test_tool_model(self):
        from yora import tool
        old = tool.TOOLS_DIR
        tool.TOOLS_DIR = tempfile.mkdtemp()
        try:
            tool.save_tool(mesh.to_stl_bytes(mesh.box_mesh(60, 60, 200)), 'Box tool.stl')
            tp, s = box_job(tool_model='Box tool', tool_model_pose='0,0,-20,0,0,0', tcp='0,0,190,0,0,0')
            self.same(tp, s)
        finally:
            tool.TOOLS_DIR = old
            tool._load.cache_clear()
            tool._hull.cache_clear()


@unittest.skipUnless(have_model(), 'robot model not imported yet')
class CornerCheckTest(unittest.TestCase):
    """Setup: the bed corners at the bed and at full height, nozzle down."""

    def test_small_bed_near_the_robot_is_reachable_and_far_bed_is_not(self):
        r = K.load_robot(MODEL)
        near = settings.normalize({'robot_model': MODEL, 'bed_frame': '900,-200,0,0,0,0', 'bed_x': 400, 'bed_y': 400, 'bed_z': 200})
        res = RP.corner_check(r, near)
        self.assertEqual(len(res), 8)
        self.assertEqual([c['point'][2] for c in res], [0.0] * 4 + [200.0] * 4)
        self.assertTrue(all(c['ok'] for c in res), [c['problems'] for c in res])
        far = dict(near, bed_frame='6000,0,0,0,0,0')
        res = RP.corner_check(r, far)
        self.assertTrue(all(not c['ok'] and 'unreachable' in c['problems'] for c in res))
