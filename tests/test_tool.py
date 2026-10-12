# Run from the project folder:
#   python -m unittest discover -s tests -v
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from yora import kinematics as K, mesh, robotpath as RP, settings, tool  # noqa: E402


def box(x0, x1, y0, y1, z0, z1):
    b = mesh.box_mesh(x1 - x0, y1 - y0, z1 - z0).reshape(-1, 3)
    return (b - b.min(0) + [x0, y0, z0]).reshape(-1, 3, 3)


# Side-mounted extruder: bracket on the flange, barrel along flange +X 165 mm out in Z, motor at -X, nozzle at +X
EXTRUDER = np.concatenate([box(-60, 60, -60, 60, 0, 40), box(-40, 40, -40, 40, 40, 110), box(-180, -60, -55, 55, 110, 220),
                           box(-60, 330, -45, 45, 120, 210), box(330, 370, -12, 12, 153, 177)])


class ToolTest(unittest.TestCase):

    def setUp(self):
        self.old = tool.TOOLS_DIR
        tool.TOOLS_DIR = tempfile.mkdtemp()
        self.name, self.info = tool.save_tool(mesh.to_stl_bytes(EXTRUDER), 'Side extruder.stl')

    def tearDown(self):
        shutil.rmtree(tool.TOOLS_DIR, ignore_errors=True)
        tool.TOOLS_DIR = self.old
        tool._load.cache_clear()
        tool._hull.cache_clear()

    def test_saved(self):
        self.assertEqual(self.name, 'Side extruder')
        self.assertEqual(tool.list_tools(), ['Side extruder'])
        self.assertEqual(self.info['size'], [550.0, 120.0, 220.0])

    def test_nozzle_tip(self):
        tcp = tool.find_nozzle_tip(settings.normalize({'tool_model': self.name}))
        T = K.xyzrpw_to_pose(tcp)
        np.testing.assert_allclose(T[:3, 3], [370, 0, 165], atol=1e-6)
        np.testing.assert_allclose(T[:3, 2], [1, 0, 0], atol=1e-9)          # tool Z out of the nozzle

    def test_nozzle_tip_follows_model_pose_and_scale(self):
        s = settings.normalize({'tool_model': self.name, 'tool_model_pose': '10,0,50,0,-90,0', 'tool_model_scale': 50})
        T = K.xyzrpw_to_pose(tool.find_nozzle_tip(s))
        np.testing.assert_allclose(T[:3, 3], [10 - 82.5, 0, 50 + 185], atol=1e-6)
        np.testing.assert_allclose(T[:3, 2], [0, 0, 1], atol=1e-9)

    @unittest.skipUnless('BORUNTE BRTIRUS 18020A' in K.list_robots(), 'robot model not imported yet')
    def test_tool_collision(self):
        r = K.load_robot('BORUNTE BRTIRUS 18020A')
        s = settings.normalize({'tool_model': self.name})
        s['tcp'] = ','.join(map(str, tool.find_nozzle_tip(s)))
        sv = RP.Solver(r, s)
        self.assertIsNotNone(sv.tool_pts)
        q = r.ik_near(sv.flange(500, 500, 5, 0), r.home)
        self.assertEqual(sv.collision(q) & RP.ERR_COLLISION, 0)
        q = r.ik_near(sv.flange(500, 500, -60, 0), r.home)              # nozzle 60 mm into the bed
        self.assertTrue(sv.collision(q) & RP.ERR_COLLISION)
        sv.part_box = (400, 400, 600, 600)
        q = r.ik_near(sv.flange(500, 500, 100, 0), r.home)
        self.assertTrue(sv.collision(q, printed_z=300) & RP.ERR_PART)  # extruder body inside a 300 mm part
        self.assertFalse(sv.collision(q, printed_z=96) & RP.ERR_PART)


if __name__ == '__main__':
    unittest.main()
