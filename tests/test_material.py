# Material library (yora/material.py): save / rename / delete, built-in examples, heating zones, placeholders.
#   python -m unittest discover -s tests -v
import os
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import material, mesh, post, settings  # noqa: E402

PETG = {'name': 'My PETG', 'brand': 'Acme', 'type': 'PETG', 'color': '#112233', 'density': 1.27, 'bed_temp': 70,
        'zones': [{'name': 'Feed', 'temp': 200}, {'name': '', 'temp': 230}, {'name': 'Nozzle', 'temp': 240}]}


class MaterialTest(unittest.TestCase):

    def setUp(self):
        self.old = material.MATERIALS_DIR
        material.MATERIALS_DIR = tempfile.mkdtemp()

    def tearDown(self):
        material.MATERIALS_DIR = self.old

    def test_save_load_rename_delete(self):
        m = material.save_material(PETG)
        self.assertEqual([z['name'] for z in m['zones']], ['Feed', 'Zone 2', 'Nozzle'])   # unnamed zone gets a name
        self.assertEqual(material.load_material('My PETG')['zones'][2]['temp'], 240)
        material.save_material(dict(PETG, name='My PETG 2'), old_name='My PETG')       # rename
        names = [x['name'] for x in material.list_materials() if not x['builtin']]
        self.assertEqual(names, ['My PETG 2'])
        material.delete_material('My PETG 2')
        self.assertFalse([x for x in material.list_materials() if not x['builtin']])

    def test_validation_and_builtins(self):
        with self.assertRaises(ValueError):
            material.normalize(dict(PETG, zones=[]))                                     # needs a zone
        with self.assertRaises(ValueError):
            material.normalize(dict(PETG, name=' '))
        one = material.normalize(dict(PETG, zones=[{'name': 'Heater', 'temp': 9999}], color='red'))
        self.assertEqual(one['zones'][0]['temp'], 600)                                   # clamped
        self.assertEqual(one['color'], '')
        builtin = next(iter(material.BUILTIN))
        self.assertTrue(material.load_material(builtin)['builtin'])
        with self.assertRaises(ValueError):
            material.save_material(dict(PETG, name=builtin))
        with self.assertRaises(ValueError):
            material.delete_material(builtin)

    def test_placeholders_reach_the_program(self):
        material.save_material(PETG)
        s = settings.normalize({'material': 'My PETG', 'layer_height': 4, 'first_layer_height': 4})
        ph = material.placeholders(s)
        self.assertEqual((ph['nozzle_temp'], ph['temp_1'], ph['zone_count']), ('240', '200', 3))
        self.assertEqual(material.placeholders(settings.normalize({}))['material'], '')
        tp = yora.slice_to_toolpath(mesh.place(mesh.box_mesh(120, 100, 8), center_xy=(500, 500)), s)
        d = post.load_post('KUKA KRL (KR C4)')
        d['events']['program_start'] = 'COMMENT {material} ({material_brand}): {zone_temps}'
        path, _ = post.export_program(tp, s, tempfile.mkdtemp(), 'Mat', None, d)
        src = zipfile.ZipFile(path).read('Mat.src').decode()
        self.assertIn('My PETG (Acme): Feed 200C, Zone 2 230C, Nozzle 240C', src)


if __name__ == '__main__':
    unittest.main()
