# -*- coding: UTF-8 -*-
# YoRA - KUKA KRL program writer for KR C4 controllers (KSS 8.x)
#
# Same post processor interface as borunte_act.py (ProgStart, MoveJ, MoveL, MoveC, setSpeed, setZoneData, setFrame,
# setTool, setDO, waitDI, Pause, RunCode, RunMessage, ProgFinish, ProgSave), so the extruder logic of
# Borunte_FGF.FGFMixin works on top of it unchanged.
#
# Output: <prog>.src + <prog>.dat in a zip (copy both to KRC:\R1\Program\ on the controller).
#   - Positions are in the active BASE (the bed frame taught on the robot) with the active TOOL (nozzle TCP):
#       LIN {X x, Y y, Z z, A a, B b, C c}   A, B, C = rotation about Z, Y, X (KUKA convention: Rz(A) Ry(B) Rx(C))
#   - Joint moves: PTP {A1 .., A2 .., A3 .., A4 .., A5 .., A6 ..} (joint values as in the robot model).
#   - Speed: $VEL.CP in m/s for LIN/CIRC, BAS(#VEL_PTP, %) for PTP.
#   - Corner blending: $APO.CDIS (mm) and C_DIS on the moves (0 = exact stop at every point).
#   - Digital outputs are switched in sync with the motion (TRIGGER WHEN DISTANCE=0 ... DO $OUT[n]), so the
#     controller's advance run doesn't switch them early. Before a WAIT or at the end of the program they are
#     set directly after "WAIT SEC 0" (stops the advance run), which is the KRL idiom for that.
# ----------------------------------------------------
import math
import os
import re
import zipfile

from borunte_act import pose_to_xyzrpw_zyx


def krl_name(name):
    """A valid KRL module name: letters, digits, underscore, starts with a letter, at most 24 characters."""
    n = re.sub(r'[^A-Za-z0-9_]', '_', str(name or 'Print'))
    if not n or not n[0].isalpha():
        n = 'P_' + n
    return n[:24]


def _num(v, d=3):
    s = ('%.' + str(d) + 'f') % float(v)
    return '0.' + '0' * d if s in ('-0.' + '0' * d,) else s


class KukaKrlPost:
    """KUKA KRL (.src / .dat) writer for KR C4."""

    PROG_EXT = 'src'
    SHOW_PROGRAM_ZIP = False
    SHOW_PROGRAM_TXT = False
    # Options (set by the slicer's post-processor settings)
    OUTPUT_MODE = 'trigger'     # 'trigger' (synchronised with motion), 'out_c' ($OUT_C at the next motion), 'direct'
    PTP_SPEED_PCT = 20.0        # % of the maximum joint speed for PTP moves
    ADVANCE = 3                 # $ADVANCE: motion blocks planned ahead (needed for blending)
    MAX_MOVES_PER_FILE = 0      # >0: split into sub-programs called from the main program
    DECIMALS = 3

    def __init__(self, robotpost=None, robotname=None, robot_axes=6, *args, **kwargs):
        self.ROBOT_POST, self.ROBOT_NAME, self.ROBOT_AXES = robotpost, robotname, robot_axes
        self.PG = []                # program body lines (in order)
        self.LOG = ''
        self.PROG_NAME = 'Print'
        self.PROG_FILES = []
        self._pending = []          # outputs waiting for the next motion: (io, value)
        self._zone = 0.0
        self._speed = None
        self.moves = 0

    # ---------------- program ----------------
    def ProgStart(self, progname):
        self.PROG_NAME = krl_name(progname)

    def ProgFinish(self, progname=''):
        self._flush_direct()

    def addlog(self, newline):
        self.LOG += newline + '\n'

    def _line(self, text):
        self.PG.append(text)

    # ---------------- outputs ----------------
    def _flush_trigger(self):
        """Pending outputs, switched when the next motion starts."""
        for io, val in self._pending:
            v = 'TRUE' if val else 'FALSE'
            if self.OUTPUT_MODE == 'trigger':
                self._line('TRIGGER WHEN DISTANCE=0 DELAY=0 DO $OUT[%i]=%s PRIO=-1' % (io, v))
            elif self.OUTPUT_MODE == 'out_c':
                self._line('$OUT_C[%i]=%s' % (io, v))
            else:
                self._line('$OUT[%i]=%s' % (io, v))
        self._pending = []

    def _flush_direct(self):
        """Pending outputs, switched now (the advance run is stopped first so they don't switch early)."""
        if not self._pending:
            return
        if self.OUTPUT_MODE != 'direct':
            self._line('WAIT SEC 0')
        for io, val in self._pending:
            self._line('$OUT[%i]=%s' % (io, 'TRUE' if val else 'FALSE'))
        self._pending = []

    def setDO(self, io_var, io_value):
        val = bool(io_value) if not isinstance(io_value, str) else io_value.strip().lower() not in ('', '0', 'false', 'off')
        self._pending.append((int(io_var), val))

    def setAO(self, io_var, io_value):
        self._flush_direct()
        self._line('$ANOUT[%i]=%s' % (int(io_var), _num(io_value, 4)))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        self._flush_direct()
        v = 'TRUE' if (io_value if not isinstance(io_value, str) else io_value not in ('0', 'false', 'off')) else 'FALSE'
        if timeout_ms is not None and timeout_ms >= 0:
            self._line('; wait for $IN[%i] with a %.1f s timeout: add a timer check here if needed' % (int(io_var), timeout_ms / 1000.0))
        self._line('WAIT FOR $IN[%i]==%s' % (int(io_var), v))

    def Pause(self, time_ms):
        self._flush_direct()
        self._line('WAIT SEC %s' % _num(max(time_ms, 0) / 1000.0, 3))

    # ---------------- motion ----------------
    def _pos(self, pose):
        x, y, z, c, b, a = pose_to_xyzrpw_zyx(pose)       # r about X = C, p about Y = B, w about Z = A
        d = self.DECIMALS
        return '{X %s,Y %s,Z %s,A %s,B %s,C %s}' % (_num(x, d), _num(y, d), _num(z, d), _num(a, 4), _num(b, 4), _num(c, 4))

    def _blend(self):
        return ' C_DIS' if self._zone > 0 else ''

    def MoveJ(self, pose, joints, conf_RLF=None):
        if joints is None:
            raise ValueError('MoveJ needs joint values')
        self._flush_trigger()
        j = list(joints)[:6]
        self._line('PTP {%s}' % ','.join('A%i %s' % (k + 1, _num(v, 4)) for k, v in enumerate(j)))
        self.moves += 1

    def MoveL(self, pose, joints, conf_RLF=None):
        self._flush_trigger()
        if self.moves == 0:
            # the first motion of a KRL program must be PTP (block coincidence run); Cartesian PTP keeps the
            # robot's current status and turn
            self._line('PTP %s' % self._pos(pose))
        else:
            self._line('LIN %s%s' % (self._pos(pose), self._blend()))
        self.moves += 1

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        self._flush_trigger()
        self._line('CIRC %s,%s%s' % (self._pos(pose1), self._pos(pose2), self._blend()))
        self.moves += 1

    # ---------------- settings ----------------
    def setSpeed(self, speed_mms):
        v = max(float(speed_mms), 0.001) / 1000.0
        if v != self._speed:
            self._line('$VEL.CP=%s' % _num(v, 4))
            self._speed = v

    def setSpeedJoints(self, speed_degs):
        pass

    def setAcceleration(self, accel_mmss):
        pass

    def setAccelerationJoints(self, accel_degss):
        pass

    def setZoneData(self, zone_mm):
        self._zone = max(float(zone_mm), 0.0)
        if self._zone > 0:
            self._line('$APO.CDIS=%s' % _num(self._zone, 2))

    def setFrame(self, pose, frame_id=0, frame_name=''):
        fid = int(frame_id) if frame_id and int(frame_id) > 0 else 1
        self._line('$BASE=BASE_DATA[%i] ; %s' % (fid, frame_name or 'base'))

    def setTool(self, pose, tool_id=0, tool_name=''):
        tid = int(tool_id) if tool_id and int(tool_id) > 0 else 1
        self._line('$TOOL=TOOL_DATA[%i] ; %s' % (tid, tool_name or 'tool'))
        self._line('$LOAD=LOAD_DATA[%i]' % tid)
        self._line('$IPO_MODE=#BASE')

    def RunCode(self, code, is_function_call=False):
        self._flush_direct()
        self._line(('%s()' % code) if is_function_call and not str(code).rstrip().endswith(')') else str(code))

    def RunMessage(self, message, iscomment=False):
        self._line('; ' + str(message))

    # ---------------- files ----------------
    def _header(self, name):
        return ['&ACCESS RVP', '&REL 1', 'DEF %s( )' % name]

    def _init(self):
        return [';FOLD INI', '  BAS(#INITMOV,0)', ';ENDFOLD', '$ADVANCE=%i' % self.ADVANCE, 'BAS(#VEL_PTP,%s)' % _num(self.PTP_SPEED_PCT, 1)]

    @staticmethod
    def _dat(name):
        return '\r\n'.join(['&ACCESS RVP', '&REL 1', 'DEFDAT %s' % name, 'ENDDAT', ''])

    def program_files(self):
        """{file name: text} for the program (split into sub-programs if MAX_MOVES_PER_FILE is set)."""
        name = self.PROG_NAME
        body = self.PG
        files = {}
        limit = int(self.MAX_MOVES_PER_FILE or 0)
        moves = [k for k, l in enumerate(body) if l.startswith(('LIN ', 'PTP ', 'CIRC '))]
        if limit <= 0 or len(moves) <= limit:
            lines = self._header(name) + self._init() + ['  ' + l for l in body] + ['END', '']
            files[name + '.src'] = '\r\n'.join(lines)
            files[name + '.dat'] = self._dat(name)
            return files
        # split at motion boundaries; the main program sets up and calls the parts in order
        cuts = [0] + [moves[k] for k in range(limit, len(moves), limit)] + [len(body)]
        parts = []
        for p in range(len(cuts) - 1):
            pname = krl_name('%s_%03i' % (name[:19], p + 1))
            parts.append(pname)
            files[pname + '.src'] = '\r\n'.join(self._header(pname) + ['  ' + l for l in body[cuts[p]:cuts[p + 1]]] + ['END', ''])
            files[pname + '.dat'] = self._dat(pname)
        main = self._header(name) + self._init() + ['  %s()' % p for p in parts] + ['END', '']
        files[name + '.src'] = '\r\n'.join(main)
        files[name + '.dat'] = self._dat(name)
        return files

    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, krl_name(progname) + '.zip')
        with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
            for fn, text in self.program_files().items():
                z.writestr(fn, text.encode('utf-8'))
        self.PROG_FILES = [path]
        return path
