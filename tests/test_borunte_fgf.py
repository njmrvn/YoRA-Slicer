# Run from the project folder:
#   python -m unittest discover -s tests -v
import glob
import json
import math
import os
import sys
import tempfile
import unittest
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'posts'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))

from borunte_act import transl, rotx  # noqa: E402
from Borunte_FGF import RobotPost, flow_level  # noqa: E402
import gcode_replay  # noqa: E402

DOWN = rotx(math.pi)
ACT_MOVEJ, ACT_MOVEL, ACT_DO, ACT_END = 4, 10, 200, 60000


def new_post():
    r = RobotPost('Borunte_FGF', 'BRTIRUS1820A', 6)
    r.SHOW_PROGRAM_ZIP = False
    r.SHOW_PROGRAM_TXT = False
    return r


def at(x, y=0.0, z=0.3):
    return transl(x, y, z) @ DOWN


def events(robot):
    """Compact view of the program: ('L', x) for linear moves, ('DO', io, value) for outputs."""
    out = []
    for ins in robot.PG:
        if ins['action'] == ACT_MOVEL:
            out.append(('L', float(ins['points'][0]['pos']['m0'])))
        elif ins['action'] == ACT_DO:
            out.append(('DO', ins['point'], bool(ins['pointStatus'])))
    return out


def extruder_state(robot):
    """Replay DO actions and return, for each linear move, (enable, level) when it starts."""
    io, states = {}, []
    for ins in robot.PG:
        if ins['action'] == ACT_DO:
            io[ins['point']] = bool(ins['pointStatus'])
        elif ins['action'] == ACT_MOVEL:
            level = sum(1 << b for b, d in enumerate(robot.FLOW_LEVEL_DOS) if io.get(d))
            states.append((io.get(robot.EXTRUDER_ENABLE_DO, False), level))
    return states


class FlowLevelTest(unittest.TestCase):

    def test_mapping(self):
        self.assertEqual(flow_level(None, 10, 0.05, 7), 0)
        self.assertEqual(flow_level(-1.0, 10, 0.05, 7), 0)       # retract
        self.assertIsNone(flow_level(1.0, 0.0, 0.05, 7))         # prime in place
        self.assertEqual(flow_level(0.0001, 100, 0.05, 7), 0)    # below E_MIN_DELTA
        self.assertEqual(flow_level(0.001, 100, 0.05, 7), 1)     # tiny but real flow
        self.assertEqual(flow_level(2.0, 50, 0.05, 7), 6)        # 0.04/0.05*7 = 5.6
        self.assertEqual(flow_level(2.5, 50, 0.05, 7), 7)        # exactly full scale
        self.assertEqual(flow_level(9.0, 50, 0.05, 7), 7)        # clamped


class PostTest(unittest.TestCase):

    def test_extruder_call_becomes_outputs(self):
        r = new_post()
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(2.0)', True)
        r.MoveL(at(50), None)
        r.ProgFinish('P')
        self.assertEqual(events(r), [
            ('L', 0.0),
            ('DO', 2, False), ('DO', 3, True), ('DO', 4, True),  # level 6 = 0b110
            ('DO', 1, True),
            ('L', 50.0),
            ('DO', 1, False),                                    # off at program end
        ])

    def test_travel_and_retract_switch_off(self):
        r = new_post()
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(2.0)', True)
        r.MoveL(at(50), None)       # print
        r.RunCode('Extruder(1.0)', True)
        r.MoveL(at(50, 0, 5), None)  # retract + lift
        r.MoveL(at(80, 0, 5), None)  # travel, no Extruder call
        r.RunCode('Extruder(2.0)', True)
        r.MoveL(at(80, 0, 0.3), None)  # unretract while lowering: 1.0 E over 4.7 mm
        r.RunCode('Extruder(4.0)', True)
        r.MoveL(at(130, 0, 0.3), None)  # print again
        r.ProgFinish('P')
        st = extruder_state(r)
        self.assertEqual([on for on, _ in st], [False, True, False, False, True, True])
        self.assertEqual(st[5], (True, 6))

    def test_e_only_lines_do_not_leak_into_next_segment(self):
        r = new_post()
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(5.0)', True)   # E-only lines, no motion in between
        r.RunCode('Extruder(6.0)', True)
        r.RunCode('Extruder(7.0)', True)   # real segment: 1.0 E over 50 mm
        r.MoveL(at(50), None)
        r.ProgFinish('P')
        # 5.0 is consumed when 6.0 arrives, 6.0 when 7.0 arrives: the move sees dE = 1.0
        self.assertEqual(extruder_state(r)[1], (True, 3))

    def test_outputs_only_written_on_change(self):
        r = new_post()
        r.ProgStart('P')
        r.MoveL(at(0), None)
        for i in range(1, 6):
            r.RunCode('Extruder(%.3f)' % (i * 2.0), True)
            r.MoveL(at(i * 50), None)
        r.ProgFinish('P')
        dos = [e for e in events(r) if e[0] == 'DO']
        self.assertEqual(len(dos), 5)  # 3 bits + enable on + enable off

    def test_absolute_e_reset(self):
        r = new_post()
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(500.0)', True)
        r.MoveL(at(50), None)
        r.RunCode('Extruder(1.0)', True)  # G92 E0 then E1.0: a reset, not a 499 mm retract
        r.MoveL(at(100), None)
        r.ProgFinish('P')
        self.assertEqual(extruder_state(r)[2], (True, 3))  # 1.0/50 = 0.02 -> 2.8 -> 3

    def test_relative_e(self):
        r = new_post()
        r.E_ABSOLUTE = False
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(1.0)', True)
        r.MoveL(at(50), None)
        r.RunCode('Extruder(1.0)', True)
        r.MoveL(at(100), None)
        r.ProgFinish('P')
        self.assertEqual(extruder_state(r)[1:], [(True, 3), (True, 3)])

    def test_on_off_only(self):
        r = new_post()
        r.FLOW_LEVEL_DOS = []
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(0.1)', True)
        r.MoveL(at(50), None)
        r.ProgFinish('P')
        self.assertEqual([e for e in events(r) if e[0] == 'DO'], [('DO', 1, True), ('DO', 1, False)])

    def test_start_delay(self):
        r = new_post()
        r.START_DELAY_S = 0.4
        r.ProgStart('P')
        r.MoveL(at(0), None)
        r.RunCode('Extruder(2.0)', True)
        r.MoveL(at(50), None)
        r.ProgFinish('P')
        delays = [i for i in r.PG if i['action'] == 100 and i['type'] == 100]
        self.assertEqual([d['limit'] for d in delays], ['0.40'])

    def test_other_calls_are_not_intercepted(self):
        r = new_post()
        r.ProgStart('P')
        r.RunCode('ExtruderHeat(200)', True)  # not an Extruder(E) call: must not be intercepted
        self.assertIsNone(r._e_pending)


class PackageTest(unittest.TestCase):

    def test_gcode_to_act_package(self):
        r = new_post()
        sample = os.path.join(HERE, '..', 'samples', 'square.gcode')
        with open(sample) as f:
            stats = gcode_replay.replay(f, r, (600.0, 0.0, 0.0), 'square')
        self.assertEqual(stats['ignored'], {'G28': 1, 'M104': 1, 'M107': 1})
        self.assertEqual(stats['moves'], 11)
        # Flow level at the start of each move (0 = extruder off):
        # approach, layer 1 (1.6 E / 40 mm), travel, layer 2 (0.6, 1.8, 0.6, 0.6 E / 30 mm), lift
        self.assertEqual([lv if on else 0 for on, lv in extruder_state(r)],
                         [0, 6, 6, 6, 6, 0, 3, 7, 3, 3, 0])

        out = tempfile.mkdtemp()
        r.ProgSave(out, 'square', False, False)
        z = zipfile.ZipFile(glob.glob(os.path.join(out, '*.zip'))[0])
        self.assertIn('square.act', z.namelist())
        main = json.loads(z.read('square.act').decode().splitlines()[0])
        self.assertEqual(main[-1]['action'], ACT_END)
        self.assertTrue(any(i['action'] == ACT_DO for i in main))


if __name__ == '__main__':
    unittest.main()
