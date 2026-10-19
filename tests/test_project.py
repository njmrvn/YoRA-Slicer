# Several parts on the bed (yora/plate.py) and project files (yora/project.py).
#   python -m unittest discover -s tests -v
import io
import json
import os
import sys
import tempfile
import unittest
import zipfile
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import mesh, project, settings  # noqa: E402
from yora.plate import Plate, free_spot  # noqa: E402

S = settings.normalize({'bed_x': 1000, 'bed_y': 1000})


def overlap(a, b):
    (alo, ahi), (blo, bhi) = a, b
    return (alo[:2] < bhi[:2]).all() and (blo[:2] < ahi[:2]).all()


class PlateTest(unittest.TestCase):

    def test_parts_do_not_overlap(self):
        plate = Plate()
        box = mesh.box_mesh(200, 150, 50)
        ids = [plate.add('box.stl', box, S)['id'] for _ in range(4)]
        self.assertEqual(len(set(ids)), 4)
        boxes = [mesh.bounds(p['placed']) for p in plate.parts]
        lo, hi = boxes[0]
        np.testing.assert_allclose((lo[:2] + hi[:2]) / 2, [500, 500])                # the first one in the middle
        for i in range(4):
            for j in range(i + 1, 4):
                self.assertFalse(overlap(boxes[i], boxes[j]), (i, j))
        self.assertEqual(len(plate.triangles()), 4 * len(box))
        info = plate.info(S)
        self.assertEqual([p['start'] for p in info['parts']], [0, 12, 24, 36])
        self.assertTrue(info['fits'])

    def test_update_duplicate_remove(self):
        plate = Plate()
        a = plate.add('a.stl', mesh.box_mesh(100, 100, 20), S)
        plate.update(a['id'], {'scale': [200, 100, 100], 'rotation': [0, 0, 90], 'center': [300, 300]}, S)
        lo, hi = mesh.bounds(a['placed'])
        np.testing.assert_allclose(hi - lo, [100, 200, 20], atol=1e-6)                 # scaled in X, then turned
        b = plate.duplicate(a['id'], S)
        lo2, hi2 = mesh.bounds(b['placed'])
        np.testing.assert_allclose(hi2 - lo2, hi - lo, atol=1e-6)
        self.assertFalse(overlap((lo, hi), (lo2, hi2)))
        plate.remove(a['id'])
        self.assertEqual([p['id'] for p in plate.parts], [b['id']])
        with self.assertRaises(KeyError):
            plate.remove(a['id'])

    def test_full_bed_puts_the_part_beside_the_others(self):
        self.assertEqual(free_spot(np.array([900, 900]), [], S), [500, 500])
        self.assertEqual(free_spot(np.array([900, 900]), [(np.array([0, 0, 0]), np.array([1000, 1000, 10]))], S), [1460, 500])

    def test_two_parts_slice_together(self):
        plate = Plate()
        for _ in range(2):
            plate.add('box.stl', mesh.box_mesh(100, 100, 20), S)
        tp = yora.slice_to_toolpath(plate.triangles(), settings.normalize(dict(S, layer_height=4, first_layer_height=4)))
        m = tp.kind == 1
        span = max(np.ptp(tp.x[m]), np.ptp(tp.y[m]))
        self.assertGreater(span, 200)                                                 # walls round both boxes


class ProjectTest(unittest.TestCase):

    def test_round_trip_with_linked_files(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(project, 'PROFILE_DIR', d), \
                mock.patch('yora.pattern.PATTERN_DIR', os.path.join(d, 'patterns')):
            os.makedirs(os.path.join(d, 'patterns'))
            with open(os.path.join(d, 'patterns', 'logo.png'), 'wb') as f:
                f.write(b'picture bytes')
            s = settings.normalize(dict(S, layer_height=3, pattern='image', pattern_image='logo.png', walls=1))
            plate = Plate()
            plate.add('cube.stl', mesh.box_mesh(50, 50, 50), s)
            plate.add('flat.stl', mesh.box_mesh(120, 40, 10), s, {'scale': [100, 100, 150], 'rotation': [0, 0, 30], 'center': [200, 700]})
            data = project.save(plate, s, 'My job')
            names = zipfile.ZipFile(io.BytesIO(data)).namelist()
            self.assertIn('profiles/patterns/logo.png', names)
            os.remove(os.path.join(d, 'patterns', 'logo.png'))                      # opened on another computer
            p = project.load(data)
            self.assertEqual(p['name'], 'My job')
            self.assertEqual(p['settings']['layer_height'], 3)
            self.assertEqual([n for n, _, _ in p['parts']], ['cube.stl', 'flat.stl'])
            self.assertEqual(p['parts'][1][2], {'scale': [100, 100, 150], 'rotation': [0, 0, 30], 'center': [200, 700]})
            np.testing.assert_allclose(p['parts'][0][1], mesh.box_mesh(50, 50, 50), atol=1e-5)
            self.assertTrue(os.path.isfile(os.path.join(d, 'patterns', 'logo.png')))
            self.assertIn('Added patterns/logo.png from the project.', p['notes'])
            with open(os.path.join(d, 'patterns', 'logo.png'), 'wb') as f:          # the user's own, different copy
                f.write(b'mine')
            p = project.load(data)
            with open(os.path.join(d, 'patterns', 'logo.png'), 'rb') as f:
                self.assertEqual(f.read(), b'mine')
            self.assertTrue(any('Kept your own patterns/logo.png' in n for n in p['notes']))

    def test_rejects_other_files_and_unsafe_paths(self):
        with self.assertRaises(ValueError):
            project.load(b'not a zip')
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w') as z:
            z.writestr('project.json', json.dumps({'format': 'something-else'}))
        with self.assertRaises(ValueError):
            project.load(buf.getvalue())
        with tempfile.TemporaryDirectory() as d, mock.patch.object(project, 'PROFILE_DIR', d):
            buf = io.BytesIO()
            with zipfile.ZipFile(buf, 'w') as z:
                z.writestr('project.json', json.dumps({'format': project.FORMAT, 'settings': {}, 'parts': []}))
                z.writestr('profiles/../../evil.txt', b'x')
                z.writestr('profiles/robots/../../evil.txt', b'x')
                z.writestr('profiles/other/evil.txt', b'x')
            p = project.load(buf.getvalue())
            self.assertEqual(p['notes'], [])
            self.assertEqual(os.listdir(d), [])


if __name__ == '__main__':
    unittest.main()
