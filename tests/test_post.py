# Post-processors (yora/post.py): languages, events, KUKA KRL writer. Run from the project folder:
#   python -m unittest discover -s tests -v
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile
import unittest
import zipfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yora  # noqa: E402
from yora import gcode, kinematics as K, mesh, post, robot, robotpath as RP, settings  # noqa: E402

MODEL = 'BORUNTE BRTIRUS 18020A'


def job(**kw):
    s = settings.normalize(dict({'layer_height': 4, 'first_layer_height': 4}, **kw))
    tp = yora.slice_to_toolpath(mesh.place(mesh.box_mesh(160, 120, 12), center_xy=(500, 500)), s)
    return tp, s


def files(path):
    z = zipfile.ZipFile(path)
    return {n: z.read(n).decode('utf-8') for n in z.namelist()}


def with_events(name, **events):
    d = post.load_post(name)
    d['events'].update(events)
    return d


class DefinitionTest(unittest.TestCase):

    def test_builtins(self):
        names = [p['name'] for p in post.list_posts()]
        for n in ('Borunte .act', 'KUKA KRL (KR C4)', 'G-code'):
            self.assertIn(n, names)
            self.assertTrue(post.load_post(n)['builtin'])

    def test_parse_event(self):
        cmds = post.parse_event('DO 5 ON\nwait 0.25\nWAIT_DI 3 off\nCOMMENT layer {layer}\n$OUT[7]=TRUE\n\n')
        self.assertEqual(cmds, [('do', 5, True), ('wait', 0.25), ('wait_di', 3, False), ('comment', 'layer {layer}'),
                                ('raw', '$OUT[7]=TRUE')])
        with self.assertRaises(ValueError):
            post.parse_event('DO five ON')
        with self.assertRaises(ValueError):
            post.parse_event('WAIT soon')

    def test_raw_code_only_where_possible(self):
        self.assertTrue(post.check_post(with_events('Borunte .act', layer_start='$OUT[1]=TRUE')))
        self.assertFalse(post.check_post(with_events('KUKA KRL (KR C4)', layer_start='$OUT[1]=TRUE')))

    def test_save_and_delete(self):
        old = post.POSTS_DIR
        post.POSTS_DIR = tempfile.mkdtemp()
        try:
            d = with_events('KUKA KRL (KR C4)', layer_start='DO 9 ON')
            d['name'] = 'My KUKA'
            post.save_post(d)
            self.assertEqual(post.load_post('My KUKA')['events']['layer_start'], 'DO 9 ON')
            self.assertIn('My KUKA', [p['name'] for p in post.list_posts()])
            with self.assertRaises(ValueError):
                post.save_post(dict(d, name='Borunte .act'))         # built-ins are read only
            post.delete_post('My KUKA')
            self.assertNotIn('My KUKA', [p['name'] for p in post.list_posts()])
        finally:
            shutil.rmtree(post.POSTS_DIR, ignore_errors=True)
            post.POSTS_DIR = old


class BorunteTest(unittest.TestCase):

    def test_same_as_classic_export(self):
        tp, s = job()
        a, _ = robot.export_act(tp, s, tempfile.mkdtemp(), 'same')
        b, _ = post.export_program(tp, s, tempfile.mkdtemp(), 'same', None, post.load_post('Borunte .act'))
        self.assertEqual(files(a), files(b))

    def test_events_become_actions(self):
        tp, s = job()
        d = with_events('Borunte .act', layer_start='DO 9 ON\nWAIT 0.5', program_end='DO 9 OFF')
        path, info = post.export_program(tp, s, tempfile.mkdtemp(), 'ev', None, d)
        prog = json.loads(files(path)['ev.act'].splitlines()[0])
        on9 = [i for i in prog if i['action'] == 200 and i['point'] == 9 and i['pointStatus']]
        waits = [i for i in prog if i['action'] == 100 and i['type'] == 100 and i['limit'] == '0.50']
        self.assertEqual(len(on9), len(tp.layer_z))
        self.assertEqual(len(waits), len(tp.layer_z))
        self.assertTrue(prog[-2]['action'] == 200 and prog[-2]['point'] == 9 and not prog[-2]['pointStatus'])
        self.assertEqual(info['events_run']['layer_start'], len(tp.layer_z))


class KukaTest(unittest.TestCase):

    def export(self, tp, s, rp=None, **events):
        path, info = post.export_program(tp, s, tempfile.mkdtemp(), 'Krl Test', rp, with_events('KUKA KRL (KR C4)', **events))
        return files(path), info

    def test_program_structure(self):
        tp, s = job()
        f, info = self.export(tp, s)
        self.assertEqual(sorted(f), ['Krl_Test.dat', 'Krl_Test.src'])
        src = f['Krl_Test.src'].splitlines()
        self.assertEqual(src[:3], ['&ACCESS RVP', '&REL 1', 'DEF Krl_Test( )'])
        self.assertEqual(src[-1], 'END')
        body = [l.strip() for l in src]
        motions = [l for l in body if l.startswith(('PTP ', 'LIN '))]
        self.assertTrue(motions[0].startswith('PTP {X'))                                   # first motion is PTP
        self.assertEqual(len(motions), info['moves'])
        self.assertIn('$BASE=BASE_DATA[%i] ; Bed' % s['user_frame_id'], body)
        self.assertIn('$TOOL=TOOL_DATA[%i] ; Nozzle' % s['tool_id'], body)
        self.assertEqual(sum(1 for l in body if l.startswith('; Layer ')), len(tp.layer_z))   # layer_start event
        self.assertIn('DEFDAT Krl_Test', f['Krl_Test.dat'])

    def test_outputs_switch_with_motion_and_end_off(self):
        tp, s = job()
        f, info = self.export(tp, s)
        body = [l.strip() for l in f['Krl_Test.src'].splitlines()]
        trig = [k for k, l in enumerate(body) if l.startswith('TRIGGER WHEN DISTANCE=0 DELAY=0 DO $OUT[')]
        self.assertGreater(len(trig), 2)
        for k in trig:                                     # every trigger is followed by more triggers or a motion
            nxt = next(l for l in body[k + 1:] if not l.startswith('TRIGGER'))
            self.assertTrue(nxt.startswith(('LIN ', 'PTP ', '$VEL')), nxt)
        do = s['extruder_enable_do']
        changes = [l for l in body if ('$OUT[%i]=' % do) in l]
        self.assertTrue(changes[-1].endswith('$OUT[%i]=FALSE' % do) or ('$OUT[%i]=FALSE PRIO' % do) in changes[-1], changes[-1])

    def test_orientation_is_kuka_abc(self):
        tp, s = job(tool_rz=30)
        f, _ = self.export(tp, s)
        m = re.search(r'LIN \{X ([-\d.]+),Y ([-\d.]+),Z ([-\d.]+),A ([-\d.]+),B ([-\d.]+),C ([-\d.]+)\}', f['Krl_Test.src'])
        a, b, c = (math.radians(float(m.group(k))) for k in (4, 5, 6))
        R = K.rotz(a)[:3, :3] @ K.roty(b)[:3, :3] @ K.rotx(c)[:3, :3]                      # KUKA: Rz(A) Ry(B) Rx(C)
        np.testing.assert_allclose(R, (K.rotx(math.pi) @ K.rotz(math.radians(30)))[:3, :3], atol=1e-6)

    @unittest.skipUnless(MODEL in K.list_robots(), 'robot model not imported yet')
    def test_starts_with_ptp_to_checked_joints(self):
        tp, s = job()
        rp = RP.solve_robot_path(tp, s, K.load_robot(MODEL))
        f, info = self.export(tp, s, rp)
        first = next(l.strip() for l in f['Krl_Test.src'].splitlines() if l.strip().startswith(('PTP', 'LIN')))
        vals = [float(v) for v in re.findall(r'A\d ([-\d.]+)', first)]
        np.testing.assert_allclose(vals, rp.joints[0], atol=1e-3)

    def test_split_into_sub_programs(self):
        tp, s = job()
        d = post.load_post('KUKA KRL (KR C4)')
        d['options']['max_moves_per_file'] = 100
        path, info = post.export_program(tp, s, tempfile.mkdtemp(), 'Big', None, d)
        f = files(path)
        parts = sorted(n for n in f if n.startswith('Big_') and n.endswith('.src'))
        self.assertGreater(len(parts), 2)
        main = f['Big.src']
        for p in parts:
            self.assertIn('  %s()' % p[:-4], main)
        n_moves = sum(sum(1 for l in f[p].splitlines() if l.strip().startswith(('LIN ', 'PTP '))) for p in parts)
        self.assertEqual(n_moves, info['moves'])

    def test_wait_stops_advance_before_outputs(self):
        tp, s = job()
        f, _ = self.export(tp, s, extruder_on='WAIT 0.3')
        body = [l.strip() for l in f['Krl_Test.src'].splitlines()]
        k = body.index('WAIT SEC 0.300')
        self.assertEqual(body[k - 1].startswith('$OUT['), True)            # outputs set directly ...
        self.assertIn('WAIT SEC 0', body[k - 6:k])                         # ... after stopping the advance run


class GcodeTest(unittest.TestCase):

    def test_events_and_unchanged_without(self):
        tp, s = job()
        plain = gcode.write_gcode(tp, s, 'g')
        path, _ = post.export_program(tp, s, tempfile.mkdtemp(), 'g', None, with_events('G-code', layer_start=''))
        self.assertEqual(open(path, encoding='utf-8').read(), plain)
        path, _ = post.export_program(tp, s, tempfile.mkdtemp(), 'g', None,
                                      with_events('G-code', layer_start='; L{layer}', extruder_on='DO 4 ON\nWAIT 0.2'))
        text = open(path, encoding='utf-8').read()
        self.assertEqual(text.count('; L'), len(tp.layer_z))
        self.assertIn('M42 P4 S255\nG4 P200', text)


if __name__ == '__main__':
    unittest.main()
