# -*- coding: UTF-8 -*-
# YoRA - Borunte post processor for robotic FGF (pellet) / FDM 3D printing
#
# The slicer's export (yora/robot.py, yora/post.py) calls this post with "Extruder(E)" program calls (absolute E)
# between the moves. This post converts the extrusion of each segment into a flow level and writes Borunte
# digital-output actions (action 200) into the .act program; the program itself is written by borunte_act.py.
#
# Flow mapping (per linear segment)
#   dE   = E - E_previous                (absolute E)
#   dist = |XYZ - XYZ_previous|          (mm, in the active reference frame)
#   ratio = dE / dist                    (mm of E per mm of path)
#   level = 0 (off)                       if dE <= E_MIN_DELTA (travel, retract, no Extruder call)
#         = ceil(ratio / FLOW_RATIO_FULL * MAX_LEVEL), clamped to 1..MAX_LEVEL otherwise
#   The level is written as: EXTRUDER_ENABLE_DO (on/off) + FLOW_LEVEL_DOS (binary, LSB first).
#   Outputs are only written when the level changes, before the move that needs it.
# ----------------------------------------------------

import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from borunte_act import BorunteActPost as BorunteBasePost  # noqa: E402  (the .act writer underneath)
from borunte_act import pose_xyz  # noqa: E402

EXTRUDER_CALL = re.compile(r'^\s*Extruder\s*\(\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*\)\s*;?\s*$')


def flow_level(delta_e, dist, flow_ratio_full, max_level, e_min_delta=1e-4, dist_min=1e-3):
    """Map the extrusion of one segment to an integer flow level.
    Returns 0 for no extrusion, None when the level can't be judged (prime/unretract in place)."""
    if delta_e is None or delta_e <= e_min_delta:
        return 0
    if dist < dist_min:
        return None
    ratio = delta_e / dist
    level = int(math.ceil(ratio / flow_ratio_full * max_level - 1e-9))
    return max(1, min(max_level, level))


class FGFMixin:
    """Extruder handling for any program writer with this post interface: Extruder(E) program calls become
    digital-output flow levels. Used with the Borunte .act writer (RobotPost below) and the KUKA KRL writer.
    event_hook, if set, is called with 'extruder_on' / 'extruder_off' / 'program_end' at those moments."""

    event_hook = None

    # ------------------------ Post processor settings ------------------------

    # Stock Borunte settings (speed is written as a percentage of DEF_SPD_MMS, 0.1 % resolution)
    DEF_SPD_MMS = 1000.0
    DEF_SPD_DEG = 200.0
    SHOW_PROGRAM_ZIP = True
    SHOW_PROGRAM_TXT = True
    SKIP_COMMENTS = True

    # Digital output that switches the extruder (screw motor) on/off
    EXTRUDER_ENABLE_DO = 1

    # Digital outputs that binary-encode the flow level, least significant bit first.
    # Example: [2, 3, 4] gives 7 levels. Use [] for plain on/off.
    FLOW_LEVEL_DOS = [2, 3, 4]

    # How the flow level is computed:
    #   'per_mm': E per mm of path (deposition per length), full scale = FLOW_RATIO_FULL
    #   'rate'  : E per second (E/mm x robot speed), full scale = FLOW_RATE_FULL. Matches a screw
    #             extruder whose RPM sets throughput, so it stays right when feature speeds differ.
    FLOW_MODE = 'per_mm'

    # E per mm of path that corresponds to the top flow level (FLOW_MODE = 'per_mm').
    # FDM 1.75 mm filament, 0.4 mm nozzle, 0.2 mm layer: ~0.033. Tune for your pellet extruder.
    FLOW_RATIO_FULL = 0.05

    # E per second that corresponds to the top flow level (FLOW_MODE = 'rate')
    FLOW_RATE_FULL = 2.0

    # Speed assumed in 'rate' mode until the program sets one (mm/s)
    FALLBACK_SPEED_MMS = 30.0

    # True: Extruder(E) receives absolute E. False: E is per segment.
    E_ABSOLUTE = True

    # A drop in absolute E larger than this is treated as a G92 E0 reset, not a retraction (mm)
    E_RESET_JUMP = 50.0

    # Ignore extrusion smaller than this (mm of E)
    E_MIN_DELTA = 1e-4

    # Seconds to wait after switching the extruder on (lets pressure build). 0 = no wait.
    START_DELAY_S = 0.0

    # --------------------------------------------------------------------------------------------------------------------

    # Settings that Borunte_FGF.json may override
    SETTINGS_KEYS = ('DEF_SPD_MMS', 'DEF_SPD_DEG', 'SHOW_PROGRAM_ZIP', 'SHOW_PROGRAM_TXT', 'SKIP_COMMENTS',
                     'EXTRUDER_ENABLE_DO', 'FLOW_LEVEL_DOS', 'FLOW_MODE', 'FLOW_RATIO_FULL', 'FLOW_RATE_FULL',
                     'FALLBACK_SPEED_MMS', 'E_ABSOLUTE', 'E_RESET_JUMP', 'E_MIN_DELTA', 'START_DELAY_S')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # every instance gets its own instruction list
        self.PG = []
        self._e_last = 0.0
        self._e_pending = None
        self._pos_last = None
        self._level = 0
        self._io_state = {}
        self._io_switches = 0
        self._speed_mms = None
        self.load_settings_file()

    def load_settings_file(self, path=None):
        """Apply overrides from Borunte_FGF.json next to this file (written by the YoRA slicer).
        Only existing UPPER_CASE settings are accepted."""
        import json
        import os
        path = path or os.path.splitext(os.path.abspath(__file__))[0] + '.json'
        if not os.path.isfile(path):
            return {}
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        applied = {}
        for key, value in data.items():
            if key in self.SETTINGS_KEYS:
                setattr(self, key, value)
                applied[key] = value
        return applied

    @property
    def MAX_LEVEL(self):
        return (1 << len(self.FLOW_LEVEL_DOS)) - 1 if self.FLOW_LEVEL_DOS else 1

    # ---------------- Extruder interception ----------------
    def RunCode(self, code, is_function_call=False):
        m = EXTRUDER_CALL.match(str(code))
        if m:
            if self._e_pending is not None:
                # Two Extruder() calls with no motion in between (E-only G-code line such as an
                # unretract): that extrusion happens in place, so it must not count toward the next segment
                self._consume_delta_e()
            self._e_pending = float(m.group(1))
            return
        super().RunCode(code, is_function_call)

    def _consume_delta_e(self):
        """Return dE for the segment about to run, and update the E tracker."""
        e = self._e_pending
        self._e_pending = None
        if e is None:
            return None  # no Extruder call: E unchanged, treat as travel
        if not self.E_ABSOLUTE:
            return e
        delta = e - self._e_last
        if delta < -self.E_RESET_JUMP:
            delta = e  # E was reset to 0 (G92 E0)
        self._e_last = e
        return delta

    def _write_do(self, io, value):
        if self._io_state.get(io) == value:
            return
        self._io_state[io] = value
        self._io_switches += 1
        super().setDO(io, value)

    def _apply_level(self, level):
        if level is None or level == self._level:
            return
        if level > 0:
            # Set the flow bits first, then enable, so the extruder starts at the right speed
            for bit, io in enumerate(self.FLOW_LEVEL_DOS):
                self._write_do(io, (level >> bit) & 1)
            self._write_do(self.EXTRUDER_ENABLE_DO, 1)
            was_off = self._level == 0
            if was_off and self.START_DELAY_S > 0:
                super().Pause(self.START_DELAY_S * 1000.0)
            self._level = level
            if was_off:
                self._event('extruder_on')
        else:
            self._write_do(self.EXTRUDER_ENABLE_DO, 0)
            self._level = level
            self._event('extruder_off')

    def _event(self, name):
        if self.event_hook is not None:
            self.event_hook(name)

    def _before_motion(self, path_len):
        delta_e = self._consume_delta_e()
        if self.FLOW_MODE == 'rate':
            if delta_e is None or delta_e <= self.E_MIN_DELTA:
                level = 0
            else:
                speed = self._speed_mms or self.FALLBACK_SPEED_MMS
                level = flow_level(delta_e * speed, path_len, self.FLOW_RATE_FULL, self.MAX_LEVEL, 0.0)
        else:
            level = flow_level(delta_e, path_len, self.FLOW_RATIO_FULL, self.MAX_LEVEL, self.E_MIN_DELTA)
        self._apply_level(level)

    def setSpeed(self, speed_mms):
        self._speed_mms = speed_mms
        super().setSpeed(speed_mms)

    def _dist_to(self, pose):
        if pose is None or self._pos_last is None:
            return 0.0
        return math.dist(pose_xyz(pose), self._pos_last)

    # ---------------- Motion ----------------
    def MoveJ(self, pose, joints, conf_RLF=None):
        self._before_motion(self._dist_to(pose))
        super().MoveJ(pose, joints, conf_RLF)
        self._pos_last = pose_xyz(pose) if pose is not None else None

    def MoveL(self, pose, joints, conf_RLF=None):
        self._before_motion(self._dist_to(pose))
        super().MoveL(pose, joints, conf_RLF)
        self._pos_last = pose_xyz(pose) if pose is not None else None

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        # Arc length approximated by the two chords (start->via->end)
        dist = self._dist_to(pose1)
        if pose1 is not None and pose2 is not None:
            dist += math.dist(pose_xyz(pose2), pose_xyz(pose1))
        self._before_motion(dist)
        super().MoveC(pose1, joints1, pose2, joints2, conf_RLF_1, conf_RLF_2)
        self._pos_last = pose_xyz(pose2) if pose2 is not None else None

    def setFrame(self, pose, frame_id=None, frame_name=None):
        # Positions are compared in the active frame: a frame change breaks the distance chain
        self._pos_last = None
        super().setFrame(pose, frame_id, frame_name)

    def ProgStart(self, progname):
        self._e_last = 0.0
        self._e_pending = None
        self._pos_last = None
        self._level = 0
        self._io_state = {}
        super().ProgStart(progname)

    def ProgFinish(self, progname):
        self._e_pending = None
        self._apply_level(0)  # never leave the extruder running
        self._event('program_end')
        super().ProgFinish(progname)
        self.addlog('Borunte_FGF: %i extruder output changes (DO%i enable, flow bits %s, %i levels)' % (
            self._io_switches, self.EXTRUDER_ENABLE_DO, self.FLOW_LEVEL_DOS, self.MAX_LEVEL))


class RobotPost(FGFMixin, BorunteBasePost):
    """Borunte_FGF: Borunte .act package with extruder outputs."""
    pass


def test_post():
    """Test the post with a short printed square"""
    from borunte_act import transl, rotx

    robot = RobotPost('Borunte_FGF', 'BRTIRUS1820A', 6)
    robot.SHOW_PROGRAM_ZIP = False
    robot.SHOW_PROGRAM_TXT = False
    down = rotx(math.pi)  # tool Z normal to the bed, pointing down

    robot.ProgStart('Print1')
    robot.setFrame(transl(600, 0, 0), 1, 'Bed')
    robot.setTool(transl(0, 0, 180), 1, 'Nozzle')
    robot.setSpeed(40)
    robot.MoveJ(transl(0, 0, 50) @ down, [0, 0, 0, 0, 90, 0])
    robot.MoveL(transl(0, 0, 0.3) @ down, None)
    e = 0.0
    for x, y in [(50, 0), (50, 50), (0, 50), (0, 0)]:
        e += 50 * 0.04
        robot.RunCode('Extruder(%.5f)' % e, True)
        robot.MoveL(transl(x, y, 0.3) @ down, None)
    robot.RunCode('Extruder(%.5f)' % (e - 2), True)  # retract
    robot.MoveL(transl(0, 0, 10) @ down, None)
    robot.ProgFinish('Print1')
    for ins in robot.PG:
        print(ins)
    print(robot.LOG)


if __name__ == "__main__":
    test_post()
