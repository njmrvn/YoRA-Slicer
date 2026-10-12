"""Robot output: the Borunte .act package through posts/Borunte_FGF.py (built on posts/borunte_act.py).

Before each move to a point the post gets Extruder(E) with the point's absolute E, then MoveL to the point (tool Z
normal to the bed, or tilted with angled layers); the post turns the E values into the extruder outputs.
"""
import importlib.util
import math
import os
import re
import sys

from .settings import PROJECT_DIR, parse_float_list, parse_int_list
from .slicer import KIND_RETRACT
from .strategy import tool_rotation

POSTS_DIR = os.path.join(PROJECT_DIR, 'posts')
POST_NAME = 'Borunte_FGF'


def load_post_class():
    """Load the project's posts/Borunte_FGF.py (built on posts/borunte_act.py)."""
    if POSTS_DIR not in sys.path:
        sys.path.insert(0, POSTS_DIR)
    path = os.path.join(POSTS_DIR, POST_NAME + '.py')
    mod = sys.modules.get(POST_NAME)
    if mod is None or os.path.normcase(os.path.abspath(mod.__file__)) != os.path.normcase(path):
        spec = importlib.util.spec_from_file_location(POST_NAME, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[POST_NAME] = mod
        spec.loader.exec_module(mod)
    return mod.RobotPost


def post_settings(s, stats):
    """Post-processor settings derived from the slicer settings."""
    full = s['flow_full_scale'] if s['flow_full_scale'] > 0 else stats.get('flow_full_scale', 0) or None
    out = {
        'EXTRUDER_ENABLE_DO': int(s['extruder_enable_do']),
        'FLOW_LEVEL_DOS': parse_int_list(s['flow_level_dos']),
        'FLOW_MODE': s['flow_mode'],
        'E_ABSOLUTE': True,
        'START_DELAY_S': float(s['start_delay']),
        'DEF_SPD_MMS': float(s['def_spd_mms']),
    }
    if full:
        out['FLOW_RATE_FULL' if s['flow_mode'] == 'rate' else 'FLOW_RATIO_FULL'] = float(full)
    return out


def safe_name(name):
    name = re.sub(r'[^A-Za-z0-9_]', '_', name or 'Print').strip('_') or 'Print'
    return name if name[0].isalpha() else 'P_' + name


def export_act(tp, s, out_dir, name, robot_path=None):
    """Write the Borunte zip package. Returns (zip_path, info).
    With a solved robot_path the nozzle rotation about Z follows the solver and the program starts
    with a joint move to the solved start joints (known configuration)."""
    RobotPost = load_post_class()
    from borunte_act import eye, rotx, rotz, transl   # numpy 4x4 poses (posts/ is on sys.path after load_post_class)

    name = safe_name(name)
    r = RobotPost(POST_NAME, 'BRTIRUS1820A', 6)
    for k, v in post_settings(s, tp.stats).items():
        setattr(r, k, v)
    r.SHOW_PROGRAM_ZIP = False
    r.SHOW_PROGRAM_TXT = False

    if robot_path is not None:
        joints = [float(v) for v in robot_path.joints[0]]
        rz_of = lambda i: float(robot_path.rz[i])
    else:
        joints = parse_float_list(s['start_joints']) if s['start_joints'].strip() else None
        rz_of = lambda i: float(s['tool_rz'])
    down = rotx(math.pi)

    r.ProgStart(name)
    r.setFrame(eye(), int(s['user_frame_id']), 'Bed')
    r.setTool(eye(), int(s['tool_id']), 'Nozzle')
    r.setZoneData(int(s['blending']))
    if joints:
        r.MoveJ(None, joints, None)
    last_e, last_f, prev = 0.0, None, None
    for i in range(len(tp)):
        x, y, z, e, f = float(tp.x[i]), float(tp.y[i]), float(tp.z[i]), float(tp.e[i]), float(tp.f[i])
        if abs(e - last_e) > 1e-7:
            r.RunCode('Extruder(%.5f)' % e, True)
            last_e = e
        here = (round(x, 4), round(y, 4), round(z, 4))
        if int(tp.kind[i]) == KIND_RETRACT and here == prev:
            continue   # E change in place: the post consumes it on the next Extruder() call
        if f != last_f:
            r.setSpeed(f)
            last_f = f
        r.MoveL(transl(x, y, z) @ tool_rotation(tp, i) @ down @ rotz(math.radians(rz_of(i))), None, None)
        prev = here
    if joints and robot_path is None:
        r.MoveJ(None, joints, None)
    r.ProgFinish(name)
    os.makedirs(out_dir, exist_ok=True)
    r.ProgSave(os.path.abspath(out_dir), name, False, False)
    zip_path = os.path.join(out_dir, name + '.zip')
    actions = {}
    for ins in r.PG:
        actions[ins['action']] = actions.get(ins['action'], 0) + 1
    info = {'instructions': len(r.PG), 'actions': actions, 'log': r.LOG.strip(),
            'post_settings': post_settings(s, tp.stats), 'robot_checked': robot_path is not None}
    if not os.path.isfile(zip_path):
        raise RuntimeError('The Borunte post did not write %s' % zip_path)
    return zip_path, info
