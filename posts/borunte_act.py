# -*- coding: UTF-8 -*-
# YoRA - independent writer for Borunte .act program packages (BRTIRUS controllers)
#
# Writes the program package a BRTIRUS controller loads. Methods: ProgStart, MoveJ, MoveL, MoveC, setSpeed, setDO,
# ... ProgSave. The output is checked byte for byte against reference programs (tests/test_borunte_act.py). It is
# linear in the program size and every instance has its own program list.
#
# Format rules:
#   program      one JSON array of instructions + the end marker, compact JSON, keys in the order shown
#   speed        "%.1f" % clamp(100 * v / DEF_SPD_MMS, 0.1, 100); joint speed uses DEF_SPD_DEG; starts at "80"
#   smooth       int(min(zone_mm, 9))  (truncated, upper limit 9, no lower limit)
#   pose         x, y, z, r, p, w with  T = transl(x,y,z) * rotz(w) * roty(p) * rotx(r),  "%.3f"
#   toolCoord    tool_id * 65536 + 1 (1 before any tool is set); ids <= 0 become 1 with a log message
#   pause        "%.2f" s;  wait for input with a timeout: "%.1f" s, isUnlimit false
#   ignored      RunCode, RunMessage, setAO, setAcceleration(Joints)  (the controller format has no action)
#   package      <prog>.zip, stored (no compression): .act .counters .fnc .palletStyle .timers .variables
# ----------------------------------------------------
import json
import math
import os
import zipfile

ACT_MOVEJ, ACT_MOVEL, ACT_MOVEC = 4, 10, 17
ACT_WAIT, ACT_OUT, ACT_FRAME, ACT_TOOL, ACT_END = 100, 200, 800, 801, 60000

# Lines after the main program in the .act file (empty sub-program slots), as the controller expects them
ACT_TAIL = ['[{"action":60000,"insertedIndex":0}]'] * 8 + ['{}', '""'] + ['[{"action":60000,"insertedIndex":0}]'] * 8

FNC_IDS = ([214, 215, 216, 217, 218, 219] + [357] + list(range(358, 394)) + [429, 427] +
           [443, 444, 445, 465, 485, 449, 469, 489, 453, 473, 493, 457, 477, 497, 461, 481, 501,
            446, 466, 486, 450, 470, 490, 454, 474, 494, 458, 478, 498, 462, 482, 502,
            447, 467, 487, 451, 471, 491, 455, 475, 495, 459, 479, 499, 463, 483, 503,
            448, 468, 488, 452, 472, 492, 456, 476, 496, 460, 480, 500, 464, 484, 504])


def _fnc_text():
    """Controller function-parameter file written with every program (constant)."""
    return ''.join('%i, %i\r\n' % (k, 511 if k == 357 else 0) for k in FNC_IDS)


def _raw(pose, i, j):
    """Element (i, j) of a 4x4 pose given as a matrix object, a numpy array or nested lists, as stored."""
    try:
        return pose[i, j]
    except (TypeError, IndexError, KeyError):
        rows = pose.rows if hasattr(pose, 'rows') else pose
        return rows[i][j]


def _m(pose, i, j):
    return float(_raw(pose, i, j))


def pose_to_xyzrpw_zyx(pose):
    """[x, y, z, r, p, w] (mm, deg) with pose = transl(x,y,z) * rotz(w) * roty(p) * rotx(r)."""
    x, y, z = _m(pose, 0, 3), _m(pose, 1, 3), _m(pose, 2, 3)
    r20 = _m(pose, 2, 0)
    if r20 > 1.0 - 1e-10:
        p, r, w = -math.pi / 2, 0.0, math.atan2(float(-_raw(pose, 1, 2)), _m(pose, 1, 1))
    elif r20 < -1.0 + 1e-10:
        p, r, w = math.pi / 2, 0.0, math.atan2(_m(pose, 1, 2), _m(pose, 1, 1))
    else:
        r00, r10 = _m(pose, 0, 0), _m(pose, 1, 0)
        # negate the stored value before converting to float, like the reference programs: exact integer zeros give
        # -0 -> +0.0 -> "0.000", a float zero gives -0.0 ("-0.000")
        p = math.atan2(float(-_raw(pose, 2, 0)), math.sqrt(r00 * r00 + r10 * r10))
        w = math.atan2(r10, r00)
        r = math.atan2(_m(pose, 2, 1), _m(pose, 2, 2))
    return [x, y, z, r * 180 / math.pi, p * 180 / math.pi, w * 180 / math.pi]


def pose_xyz(pose):
    """Position [x, y, z] of a pose (matrix object, numpy array or nested lists)."""
    if hasattr(pose, 'Pos'):
        return list(pose.Pos())
    return [_m(pose, 0, 3), _m(pose, 1, 3), _m(pose, 2, 3)]


# ---------------- 4x4 poses (numpy; combine with @) ----------------
def eye():
    import numpy as np
    return np.eye(4)


def transl(x, y, z):
    import numpy as np
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def rotx(a):
    import numpy as np
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def roty(a):
    import numpy as np
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])


def rotz(a):
    import numpy as np
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _pos(values):
    v = list(values) + [0.0] * 8
    return {'m%i' % k: '%.3f' % float(v[k]) for k in range(8)}


class BorunteActPost:
    """Borunte .act program writer (post processor interface: ProgStart, MoveL, setDO ... ProgSave)."""

    DEF_SPD_MMS = 1000.0
    DEF_SPD_DEG = 200.0
    SHOW_PROGRAM_ZIP = False     # accepted for compatibility; nothing is opened
    SHOW_PROGRAM_TXT = False
    SKIP_COMMENTS = True

    def __init__(self, robotpost=None, robotname=None, robot_axes=6, *args, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.ROBOT_AXES = robot_axes
        self.PG = []
        self.LOG = ''
        self.PROG_FILES = []
        self.PROG_NAME = 'progname'
        self.CUR_SPD = '80'
        self.INS_BLND = 0
        self.TOOL_COORD = 1

    # ---------------- program ----------------
    def ProgStart(self, progname):
        self.PROG_NAME = progname

    def ProgFinish(self, progname=''):
        self.PG.append({'action': ACT_END, 'insertedIndex': 0})

    def addlog(self, newline):
        self.LOG += newline + '\n'

    def add_ins(self, ins):
        ins['insertedIndex'] = len(self.PG) + 1
        self.PG.append(ins)

    def program_text(self):
        """The .act file content."""
        pg = self.PG if self.PG and self.PG[-1].get('action') == ACT_END else self.PG + [{'action': ACT_END, 'insertedIndex': 0}]
        prog = json.dumps(pg, separators=(',', ':'))
        return '\r\n'.join([prog] + ACT_TAIL)

    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        """Write <folder>/<progname>.zip. Returns its path."""
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, progname + '.zip')
        files = [('.act', self.program_text()), ('.counters', ''), ('.fnc', _fnc_text()),
                 ('.palletStyle', ''), ('.timers', ''), ('.variables', '')]
        with zipfile.ZipFile(path, 'w', zipfile.ZIP_STORED) as z:
            for ext, text in files:
                z.writestr(progname + ext, text.encode('utf-8'))
        self.PROG_FILES = [path]
        print('SAVED: %s' % path)
        return path

    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        self.addlog('Sending to the robot is not supported: copy the zip to the controller.')

    # ---------------- motion ----------------
    def _move(self, action, points):
        self.add_ins({'action': action, 'ckStatus': '63', 'delay': '0.00',
                      'points': [{'pointName': '', 'pos': p} for p in points],
                      'smooth': self.INS_BLND, 'speed': self.CUR_SPD, 'toolCoord': self.TOOL_COORD})

    def MoveJ(self, pose, joints, conf_RLF=None):
        if joints is None:
            raise ValueError('MoveJ needs joint values')
        self._move(ACT_MOVEJ, [_pos(list(joints)[:6])])

    def MoveL(self, pose, joints, conf_RLF=None):
        if pose is None:
            raise ValueError('MoveL needs a pose (the controller program stores X Y Z Rx Ry Rz)')
        self._move(ACT_MOVEL, [_pos(pose_to_xyzrpw_zyx(pose))])

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        self._move(ACT_MOVEC, [_pos(pose_to_xyzrpw_zyx(pose1)), _pos(pose_to_xyzrpw_zyx(pose2))])

    # ---------------- settings ----------------
    def _set_speed_pct(self, pct):
        self.CUR_SPD = '%.1f' % max(0.1, min(100.0, pct))

    def setSpeed(self, speed_mms):
        self._set_speed_pct(100.0 * speed_mms / self.DEF_SPD_MMS)

    def setSpeedJoints(self, speed_degs):
        self._set_speed_pct(100.0 * speed_degs / self.DEF_SPD_DEG)

    def setAcceleration(self, accel_mmss):
        pass

    def setAccelerationJoints(self, accel_degss):
        pass

    def setZoneData(self, zone_mm):
        self.INS_BLND = int(min(zone_mm, 9))

    def setFrame(self, pose, frame_id=0, frame_name=''):
        if frame_id is None or frame_id <= 0:
            self.addlog("Frame does not have a frame ID, defaulting to #1. Rename your frame to '%s 1' to get rid of this message." % frame_name)
            frame_id = 1
        self.PG.append({'action': ACT_FRAME, 'coordID': frame_id, 'coordName': frame_name,
                        'insertedIndex': len(self.PG) + 1, 'customName': frame_name})

    def setTool(self, pose, tool_id=0, tool_name=''):
        if tool_id is None or tool_id <= 0:
            self.addlog("Tool does not have a tool ID, defaulting to #1. Rename your tool to '%s 1' to get rid of this message." % tool_name)
            tool_id = 1
        self.TOOL_COORD = tool_id * 65536 + 1
        self.PG.append({'action': ACT_TOOL, 'toolID': tool_id, 'toolName': tool_name,
                        'insertedIndex': len(self.PG) + 1, 'customName': tool_name})

    # ---------------- I/O, waits, code ----------------
    def setDO(self, io_var, io_value):
        self.PG.append({'action': ACT_OUT, 'type': 0, 'point': io_var, 'pointStatus': _truthy(io_value), 'valveID': 0,
                        'delay': '0.000', 'isWaitInput': False, 'insertedIndex': len(self.PG) + 1, 'customName': ''})

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        timed = timeout_ms is not None and timeout_ms >= 0
        self.PG.append({'action': ACT_WAIT, 'type': 0, 'point': io_var, 'pointStatus': _truthy(io_value),
                        'limit': ('%.1f' % (timeout_ms / 1000.0)) if timed else '0.000', 'isUnlimit': not timed,
                        'insertedIndex': len(self.PG) + 1})

    def Pause(self, time_ms):
        self.PG.append({'action': ACT_WAIT, 'type': 100, 'point': 8, 'pointStatus': 1, 'limit': '%.2f' % (time_ms / 1000.0),
                        'isUnlimit': True, 'insertedIndex': len(self.PG) + 1})

    def RunCode(self, code, is_function_call=False):
        pass

    def RunMessage(self, message, iscomment=False):
        pass

    def setAO(self, io_var, io_value):
        pass


def _truthy(v):
    if isinstance(v, str):
        return v.strip().lower() not in ('', '0', 'false', 'off')
    return bool(v)
