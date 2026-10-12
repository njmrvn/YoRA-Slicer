# Borunte .act writer (posts/borunte_act.py): package layout, speed, and byte-exact output.
# The reference fingerprints (tests/data/borunte_act_reference.json) are SHA-256 hashes of every file the writer makes
# for fixed programs; the writer was checked byte for byte against reference controller programs when they were
# recorded. Any change to the bytes written fails ReferenceTest.  Run from the project folder:
#   python -m unittest discover -s tests -v
# After an intended format change:  python tests/test_borunte_act.py --update-reference
import hashlib
import json
import math
import os
import random
import sys
import tempfile
import time
import unittest
import zipfile

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'posts'))
REFERENCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'borunte_act_reference.json')

from borunte_act import BorunteActPost, rotx, roty, rotz, transl  # noqa: E402


def zip_files(path):
    z = zipfile.ZipFile(path)
    return [(i.filename, i.compress_type, z.read(i)) for i in z.infolist()]


# ---------------- fixed programs ----------------
DOWN = rotx(math.pi)


def _movel(r):
    r.setSpeed(40)
    for x, y in [(0, 0), (100, 0), (100, 50), (0, 50), (0, 0)]:
        r.MoveL(transl(600 + x, -200 + y, 300) @ DOWN, None)


def _movej(r):
    r.MoveJ(None, [0, -10, 20, 0, -60, 0])
    r.MoveJ(None, [12.5, -33.25, 41, -170, 88.8, 359.9])


def _movej_pose(r):
    r.MoveJ(transl(700, 100, 400) @ DOWN @ rotz(0.3), [5, -20, 30, 0, -70, 10])


def _movec(r):
    r.MoveC(transl(800, 0, 200) @ DOWN, None, transl(900, 100, 200) @ DOWN, None)


def _speed_zone(r):
    for v in (0.05, 1, 30.5, 999, 2000):
        r.setSpeed(v)
        r.MoveL(transl(600, 0, 300 + v / 100) @ DOWN, None)
    for z in (-1, 0, 2.9, 9, 25):
        r.setZoneData(z)
        r.MoveL(transl(650, 10, 300) @ DOWN, None)
    r.setSpeedJoints(45)
    r.MoveJ(None, [0, 0, 0, 0, -90, 0])


def _io(r):
    r.setDO(2, 1)
    r.setDO(3, 0)
    r.waitDI(4, 1, -1)
    r.waitDI(5, 0, 2500)
    r.Pause(1500)
    r.RunCode('Ignored()', True)
    r.RunMessage('ignored message')


def _frames(r):
    r.setFrame(transl(600, 0, 0), 3, 'Bed')
    r.setTool(transl(0, 0, 180), 2, 'Nozzle')
    r.MoveL(transl(10, 20, 30) @ DOWN, None)
    r.setTool(transl(0, 0, 0), 0, 'bad id')
    r.MoveL(transl(10, 20, 40) @ roty(math.pi / 2), None)


def _random(seed):
    rnd = random.Random(seed)

    def fn(r):
        for _ in range(rnd.randint(1, 60)):
            k = rnd.random()
            pose = transl(rnd.uniform(-2000, 2000), rnd.uniform(-2000, 2000), rnd.uniform(-500, 2000)) @ \
                rotz(rnd.uniform(-4, 4)) @ roty(rnd.uniform(-1.6, 1.6)) @ rotx(rnd.uniform(-4, 4))
            if k < 0.45:
                r.MoveL(pose, None)
            elif k < 0.55:
                r.MoveJ(None, [rnd.uniform(-360, 360) for _ in range(6)])
            elif k < 0.6:
                r.MoveC(pose, None, pose @ transl(10, 5, 0), None)
            elif k < 0.68:
                r.setSpeed(rnd.choice([rnd.uniform(0, 1200), rnd.randint(1, 1000)]))
            elif k < 0.72:
                r.setSpeedJoints(rnd.uniform(0, 300))
            elif k < 0.77:
                r.setZoneData(rnd.choice([rnd.uniform(-2, 12), rnd.randint(-1, 20)]))
            elif k < 0.85:
                r.setDO(rnd.randint(1, 16), rnd.choice([0, 1, True, False]))
            elif k < 0.88:
                r.waitDI(rnd.randint(1, 16), rnd.choice([0, 1]), rnd.choice([-1, rnd.uniform(0, 9000)]))
            elif k < 0.92:
                r.Pause(rnd.uniform(-10, 20000))
            elif k < 0.96:
                r.setFrame(transl(0, 0, 0), rnd.randint(-1, 9), 'F%i' % rnd.randint(0, 9))
            else:
                r.setTool(transl(0, 0, 0), rnd.randint(-1, 9), 'T%i' % rnd.randint(0, 9))
    return fn


CASES = {'movel': _movel, 'movej': _movej, 'movej_pose': _movej_pose, 'movec': _movec, 'speed_zone': _speed_zone,
         'io': _io, 'frames': _frames}
CASES.update({'random_%02i' % k: _random(k) for k in range(60)})


def fingerprint(name, fn):
    r = BorunteActPost('Borunte', 'BRTIRUS1820A', 6)
    r.ProgStart(name)
    fn(r)
    r.ProgFinish(name)
    files = zip_files(r.ProgSave(tempfile.mkdtemp(), name))
    return [[f, c, hashlib.sha256(data).hexdigest()] for f, c, data in files]


class WriterTest(unittest.TestCase):

    def test_package_layout(self):
        r = BorunteActPost('Borunte', 'BRTIRUS1820A', 6)
        r.ProgStart('P')
        r.setSpeed(40)
        r.MoveJ(None, [0, -10, 20, 0, -60, 0])
        r.setDO(1, 1)
        r.ProgFinish('P')
        files = zip_files(r.ProgSave(tempfile.mkdtemp(), 'P'))
        self.assertEqual([f[0] for f in files], ['P.act', 'P.counters', 'P.fnc', 'P.palletStyle', 'P.timers', 'P.variables'])
        self.assertTrue(all(c == zipfile.ZIP_STORED for _, c, _ in files))
        lines = files[0][2].decode().split('\r\n')
        self.assertEqual(len(lines), 19)
        prog = json.loads(lines[0])
        self.assertEqual([i['action'] for i in prog], [4, 200, 60000])
        self.assertEqual(prog[0]['speed'], '4.0')
        self.assertEqual([i['insertedIndex'] for i in prog], [1, 2, 0])

    def test_instances_do_not_share_programs(self):
        a, b = BorunteActPost(), BorunteActPost()
        a.MoveJ(None, [0] * 6)
        self.assertEqual(len(b.PG), 0)

    def test_linear_time(self):
        import numpy as np
        r = BorunteActPost()
        T = np.eye(4)
        T[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        t = time.time()
        for i in range(40000):
            T[0, 3] = i % 500
            r.MoveL(T, None)
        r.ProgFinish()
        r.ProgSave(tempfile.mkdtemp(), 'big')
        self.assertLess(time.time() - t, 10)


class ReferenceTest(unittest.TestCase):

    def test_bytes_match_the_reference(self):
        with open(REFERENCE, 'r', encoding='utf-8') as f:
            ref = json.load(f)
        self.assertEqual(sorted(ref), sorted(CASES))
        for name, fn in CASES.items():
            with self.subTest(case=name):
                self.assertEqual(fingerprint(name, fn), ref[name])


if __name__ == '__main__':
    if '--update-reference' in sys.argv:
        os.makedirs(os.path.dirname(REFERENCE), exist_ok=True)
        with open(REFERENCE, 'w', encoding='utf-8') as f:
            json.dump({name: fingerprint(name, fn) for name, fn in CASES.items()}, f, indent=1)
        print('reference written:', REFERENCE)
    else:
        unittest.main()
