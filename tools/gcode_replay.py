# YoRA - replay slicer G-code through the Borunte_FGF post
#
# Hands the post what the slicer's own export does:
#   for each G0/G1 point: Extruder(E) (absolute E, only when the line has an E word),
#   setSpeed(F/60) when F changes, then MoveL(pose) with the tool Z normal to the bed.
# It does no inverse kinematics, reachability or collision checks: the app's robot check does those.
# Use it to check extruder I/O, flow levels and the .act package from Cura/PrusaSlicer/Orca output.
#
# Usage (any Python 3.10+ with numpy):
#   python tools/gcode_replay.py samples/square.gcode --out out
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'posts'))

from borunte_act import transl, rotx  # noqa: E402
from Borunte_FGF import RobotPost  # noqa: E402

WORD = re.compile(r'([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))')


def parse_line(line):
    """Return (command, {letter: value}) for one G-code line, or (None, {})."""
    line = line.split(';', 1)[0]
    line = re.sub(r'\(.*?\)', '', line).strip().upper()
    if not line:
        return None, {}
    words = WORD.findall(line)
    if not words:
        return None, {}
    cmd = None
    params = {}
    for letter, value in words:
        if letter in 'GM' and cmd is None:
            cmd = '%s%i' % (letter, int(float(value)))
        else:
            params[letter] = float(value)
    return cmd, params


def replay(lines, robot, bed_xyz=(0.0, 0.0, 0.0), progname='Print'):
    """Drive a RobotPost with the calls the export makes for this G-code. Returns stats."""
    down = rotx(math.pi)
    pos = [0.0, 0.0, 0.0]
    abs_xyz = True
    abs_e = True
    e_offset = 0.0   # converts relative / G92-reset E into one absolute E stream
    e_raw = 0.0
    e_abs = 0.0
    feed = None
    stats = {'moves': 0, 'extruder_calls': 0, 'ignored': {}}

    robot.ProgStart(progname)
    robot.setFrame(transl(*bed_xyz), 1, 'Bed')
    for line in lines:
        cmd, p = parse_line(line)
        if cmd is None and p:
            cmd = 'G1'  # modal continuation
        if cmd in ('G0', 'G1'):
            for i, axis in enumerate('XYZ'):
                if axis in p:
                    pos[i] = p[axis] if abs_xyz else pos[i] + p[axis]
            if 'F' in p and p['F'] != feed:
                feed = p['F']
                robot.setSpeed(feed / 60.0)
            if 'E' in p:
                if abs_e:
                    e_raw = p['E']
                    e_abs = e_offset + e_raw
                else:
                    e_abs += p['E']
                robot.RunCode('Extruder(%.5f)' % e_abs, True)
                stats['extruder_calls'] += 1
            if any(a in p for a in 'XYZ'):
                robot.MoveL(transl(*pos) @ down, None)
                stats['moves'] += 1
        elif cmd == 'G90':
            abs_xyz = True
        elif cmd == 'G91':
            abs_xyz = False
        elif cmd == 'M82':
            abs_e = True
        elif cmd == 'M83':
            abs_e = False
        elif cmd == 'G92':
            if 'E' in p and abs_e:
                e_offset = e_abs - p['E']
                e_raw = p['E']
            for i, axis in enumerate('XYZ'):
                if axis in p:
                    pos[i] = p[axis]
        elif cmd is not None:
            stats['ignored'][cmd] = stats['ignored'].get(cmd, 0) + 1
    robot.ProgFinish(progname)
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('gcode')
    ap.add_argument('--out', default='out', help='folder for the Borunte .zip package')
    ap.add_argument('--bed', type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=('X', 'Y', 'Z'),
                    help='bed origin offset written as the user frame (mm)')
    ap.add_argument('--flow-full', type=float, default=None, help='override FLOW_RATIO_FULL (E per mm)')
    args = ap.parse_args()

    robot = RobotPost('Borunte_FGF', 'BRTIRUS1820A', 6)
    robot.SHOW_PROGRAM_ZIP = False
    robot.SHOW_PROGRAM_TXT = False
    if args.flow_full:
        robot.FLOW_RATIO_FULL = args.flow_full

    name = os.path.splitext(os.path.basename(args.gcode))[0]
    with open(args.gcode, 'r', encoding='utf-8', errors='replace') as f:
        stats = replay(f, robot, tuple(args.bed), name)
    os.makedirs(args.out, exist_ok=True)
    robot.ProgSave(os.path.abspath(args.out), name, False, False)
    print('Moves: %i  Extruder calls: %i  Ignored: %s' % (stats['moves'], stats['extruder_calls'], stats['ignored']))
    print(robot.LOG.strip())


if __name__ == '__main__':
    main()
