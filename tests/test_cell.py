# Robot cells (yora/cell.py): save / load / export / import the whole robot setup.
# Run from the project folder:  python -m unittest discover -s tests -v
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from yora import cell, kinematics as K, mesh, settings, tool  # noqa: E402

MODEL = 'BORUNTE BRTIRUS 18020A'


@unittest.skipUnless(MODEL in K.list_robots(), 'robot model not imported yet')
class CellTest(unittest.TestCase):

    def setUp(self):
        # work on copies of the profile folders
        self.dirs = (K.ROBOTS_DIR, tool.TOOLS_DIR, cell.CELLS_DIR)
        tmp = tempfile.mkdtemp()
        K.ROBOTS_DIR = os.path.join(tmp, 'robots')
        shutil.copytree(self.dirs[0], K.ROBOTS_DIR)
        tool.TOOLS_DIR = os.path.join(tmp, 'tools')
        cell.CELLS_DIR = os.path.join(tmp, 'cells')
        tool.save_tool(mesh.to_stl_bytes(mesh.box_mesh(80, 80, 300)), 'Extruder.stl')
        self.values = settings.normalize({'robot_model': MODEL, 'tool_model': 'Extruder', 'tool_model_pose': '0,0,10,0,90,0',
                                          'tcp': '300,0,160,0,90,0', 'bed_frame': '700,-400,-50,0,0,15', 'bed_x': 1200,
                                          'user_frame_id': 3, 'tool_id': 2, 'rotz_range': 45, 'lock_config': False,
                                          'layer_height': 7})
        r = K.load_robot(MODEL)
        r.lower[0], r.upper[0] = -120.0, 130.0
        K.save_robot(r)

    def tearDown(self):
        K.ROBOTS_DIR, tool.TOOLS_DIR, cell.CELLS_DIR = self.dirs
        tool._load.cache_clear()
        tool._hull.cache_clear()

    def test_save_and_load(self):
        c = cell.save_cell('Shop floor', self.values, [10, -20, 30, 0, 45, 90])
        self.assertEqual(cell.list_cells(), ['Shop floor'])
        self.assertNotIn('layer_height', c['settings'])            # print settings are not part of the cell
        r = K.load_robot(MODEL)                                      # limits changed after saving ...
        r.lower[0] = -150.0
        K.save_robot(r)
        back = cell.load_cell('Shop floor')                          # ... loading the cell restores them
        for k in ('robot_model', 'tool_model', 'tool_model_pose', 'tcp', 'bed_frame', 'bed_x', 'user_frame_id', 'tool_id',
                  'rotz_range', 'lock_config'):
            self.assertEqual(back['settings'][k], self.values[k], k)
        self.assertEqual(back['joints'], [10, -20, 30, 0, 45, 90])
        self.assertEqual(back['missing'], [])
        self.assertEqual(K.load_robot(MODEL).lower[0], -120.0)

    def test_bundle_moves_to_another_pc(self):
        cell.save_cell('Shop floor', self.values, [0, 0, 0, 0, 90, 0])
        name, data = cell.export_bundle('Shop floor')
        # "another PC": empty profile folders
        tmp = tempfile.mkdtemp()
        K.ROBOTS_DIR, tool.TOOLS_DIR, cell.CELLS_DIR = (os.path.join(tmp, d) for d in ('robots', 'tools', 'cells'))
        tool._load.cache_clear()
        back = cell.import_bundle(data)
        self.assertEqual(back['missing'], [])
        self.assertIn(MODEL, K.list_robots())
        self.assertTrue(os.path.isfile(K.mesh_path(MODEL)))
        self.assertEqual(tool.list_tools(), ['Extruder'])
        self.assertEqual(K.load_robot(MODEL).upper[0], 130.0)
        self.assertEqual(cell.list_cells(), ['Shop floor'])
        self.assertEqual(back['settings']['tcp'], '300,0,160,0,90,0')

    def test_missing_parts_are_reported(self):
        v = dict(self.values, robot_model='No Such Robot', tool_model='No Such Tool')
        cell.save_cell('Broken', v)
        self.assertEqual(len(cell.load_cell('Broken')['missing']), 2)

    def test_rejects_other_files(self):
        import io, zipfile
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w') as z:
            z.writestr('readme.txt', 'hi')
        with self.assertRaises(ValueError):
            cell.import_bundle(buf.getvalue())


if __name__ == '__main__':
    unittest.main()
