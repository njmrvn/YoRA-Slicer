# The slicer is self-contained: slicing, robot model, robot check, simulation and the Borunte / KUKA exports run in a
# separate Python process that may import only the standard library, numpy, OpenCV and the project's own modules.
# Run from the project folder:  python -m unittest discover -s tests -v
import os
import subprocess
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

SCRIPT = r'''
import importlib.abc, os, sys
ALLOWED = set(sys.stdlib_module_names) | {'numpy', 'cv2', 'yora', 'borunte_act', 'Borunte_FGF', 'kuka_krl'}
refused = []

class OnlyAllowed(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        top = name.split('.')[0]
        if top not in ALLOWED and not top.startswith('_'):
            refused.append(name)
            raise ImportError('not allowed for this test: ' + name)
        return None

sys.meta_path.insert(0, OnlyAllowed())
sys.path.insert(0, ROOT)
import tempfile, zipfile, json
import numpy as np
import yora
from yora import mesh, post, robot, robotpath, kinematics, settings, tool

s = settings.normalize({'layer_height': 4, 'first_layer_height': 4})
tp = yora.slice_to_toolpath(mesh.place(mesh.box_mesh(200, 150, 16), center_xy=(500, 500)), s)
out = {'moves': len(tp)}
names = kinematics.list_robots()
if names:
    r = kinematics.load_robot(names[0])
    rp = robotpath.solve_robot_path(tp, s, r)
    out['robot_ok'] = bool(rp.summary['ok'])
    out['sim_samples'] = int(len(robotpath.sim_samples(rp, tp)))
else:
    rp = None
path, info = robot.export_act(tp, s, tempfile.mkdtemp(), 'independent', rp)
prog = json.loads(zipfile.ZipFile(path).read('independent.act').decode().splitlines()[0])
out['act_instructions'] = len(prog)
kpath, _ = post.export_program(tp, s, tempfile.mkdtemp(), 'independent', rp, post.load_post('KUKA KRL (KR C4)'))
out['krl'] = sorted(zipfile.ZipFile(kpath).namelist())
out['refused'] = refused
out['third_party'] = sorted({m.split('.')[0] for m in sys.modules} - ALLOWED - {m for m in sys.modules if m.startswith('_')})
print('RESULT ' + json.dumps(out), flush=True)
'''.replace('ROOT', repr(ROOT))


class IndependentTest(unittest.TestCase):

    def test_runs_on_numpy_and_opencv_only(self):
        p = subprocess.run([sys.executable, '-c', SCRIPT], capture_output=True, text=True, timeout=600, cwd=ROOT)
        line = [l for l in p.stdout.splitlines() if l.startswith('RESULT ')]
        self.assertTrue(line, p.stdout[-3000:] + p.stderr[-3000:])
        import json
        out = json.loads(line[0][7:])
        self.assertGreater(out['moves'], 100)
        self.assertGreater(out['act_instructions'], out['moves'] * 0.9)
        self.assertEqual(out['krl'], ['independent.dat', 'independent.src'])
        self.assertEqual(out['refused'], [])                  # nothing else was even tried
        self.assertEqual(out['third_party'], [])
        if 'robot_ok' in out:
            self.assertGreater(out['sim_samples'], 0)


if __name__ == '__main__':
    unittest.main()
