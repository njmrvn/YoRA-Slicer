"""Local web server for the YoRA Slicer GUI (127.0.0.1 only).

Needs Python 3.10+ with numpy and opencv-python (see requirements.txt; "YoRA Slicer.bat" sets this up).
    python -m yora.server --open
"""
import argparse
import json
import mimetypes
import os
import shutil
import subprocess
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

from . import gcode, mesh, robot, settings
from .planner import Toolpath
from .slicer import KIND_RETRACT, KIND_TRAVEL, KIND_NAMES
from . import slice_to_toolpath, __version__

WEB_DIR = os.path.join(settings.PROJECT_DIR, 'web')
OUT_DIR = os.path.join(settings.PROJECT_DIR, 'out')
SAMPLES_DIR = os.path.join(settings.PROJECT_DIR, 'samples')


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.model_name = None
        self.model_raw = None       # triangles as loaded
        self.model = None           # triangles placed on the bed
        self.transform = {'scale': [100, 100, 100], 'rotation': [0, 0, 0], 'center': None}
        self.toolpath = None
        self.slice_cache = {}       # the last sliced layers (reused when only the surface pattern changes)
        self.job_name = 'Print'
        self.source = None          # 'slice' | 'gcode'
        self.task = {'state': 'idle', 'stage': '', 'progress': 0.0, 'error': None, 'started': 0}
        self.robot_path = None      # robotpath.RobotPath for self.toolpath
        self.robot_key = None       # (toolpath id, robot settings) it was solved for
        self.robot_samples = None   # (skeleton points, move indices) for playback
        self.rtask = {'state': 'idle', 'stage': '', 'progress': 0.0, 'error': None}

    def place(self, s):
        t = self.transform
        center = t['center'] or [s['bed_x'] / 2, s['bed_y'] / 2]
        self.model = mesh.place(self.model_raw, t['scale'], t['rotation'], center)

    def model_info(self, s):
        if self.model is None:
            return None
        lo, hi = mesh.bounds(self.model)
        fits = bool(lo[0] >= 0 and lo[1] >= 0 and hi[0] <= s['bed_x'] and hi[1] <= s['bed_y'] and hi[2] <= s['bed_z'])
        return {'name': self.model_name, 'triangles': int(len(self.model)), 'min': lo.tolist(), 'max': hi.tolist(),
                'size': (hi - lo).tolist(), 'fits': fits, 'transform': self.transform}


STATE = State()

ROBOT_KEYS = ('robot_model', 'tcp', 'tool_model', 'tool_model_pose', 'tool_model_scale', 'bed_frame', 'tool_rz', 'rotz_range', 'rotz_step', 'sing_margin', 'lock_config',
              'start_joints', 'joint_jump_max', 'check_step', 'floor_margin')


def robot_key(s):
    return (id(STATE.toolpath), tuple(str(s[k]) for k in ROBOT_KEYS))


def current_robot_path(s):
    """The solved robot path, only if it matches the current toolpath and robot cell settings."""
    if STATE.robot_path is not None and STATE.robot_key == robot_key(s):
        return STATE.robot_path
    return None


def robot_result_json(rp):
    n = len(rp.err)
    idx = np.unique(np.linspace(0, n - 1, min(n, 1200)).astype(np.int64)) if n else np.zeros(0, dtype=np.int64)
    tp = STATE.toolpath
    errs = np.nonzero(rp.err)[0]
    if len(errs) > 2000:
        errs = errs[np.linspace(0, len(errs) - 1, 2000).astype(np.int64)]
    return {'summary': rp.summary, 'issues': rp.issues,
            'trace': {'move': idx.tolist(), 'joints': rp.joints[idx].round(2).tolist(), 'err': rp.err[idx].tolist(),
                      'layer': tp.layer[idx].tolist() if tp is not None else []},
            'error_points': [[float(tp.x[i]), float(tp.y[i]), float(tp.z[i]), int(rp.err[i])] for i in errs] if tp is not None else []}


def run_robot_check(s):
    from . import kinematics, robotpath
    task = STATE.rtask
    try:
        def progress(stage, frac):
            task['stage'], task['progress'] = stage, float(frac)
        tp = STATE.toolpath
        if tp is None:
            raise RuntimeError('Slice a model or import G-code first')
        model = kinematics.load_robot(s['robot_model'])
        rp = robotpath.solve_robot_path(tp, s, model, progress)
        samples = robotpath.sim_samples(rp, tp)
        with STATE.lock:
            STATE.robot_path, STATE.robot_key, STATE.robot_samples = rp, robot_key(s), samples
        task['state'] = 'done'
    except Exception as e:
        traceback.print_exc()
        task['state'], task['error'] = 'error', '%s: %s' % (type(e).__name__, e)


def result_json(tp):
    """Stats plus per-layer offsets into the two preview buffers."""
    ext_idx, trv_idx = preview_indices(tp)
    nl = len(tp.layer_z)
    ext_off = np.searchsorted(tp.layer[ext_idx], np.arange(nl + 1)).tolist() if len(ext_idx) else [0] * (nl + 1)
    trv_off = np.searchsorted(tp.layer[trv_idx], np.arange(nl + 1)).tolist() if len(trv_idx) else [0] * (nl + 1)
    return {'stats': tp.stats, 'layer_z': [round(z, 4) for z in tp.layer_z], 'warnings': tp.warnings,
            'extrude_offsets': ext_off, 'travel_offsets': trv_off, 'source': STATE.source, 'job': STATE.job_name,
            'kinds': {int(k): v for k, v in KIND_NAMES.items()}}


def preview_indices(tp):
    idx = np.arange(1, len(tp))
    moved = (tp.x[1:] != tp.x[:-1]) | (tp.y[1:] != tp.y[:-1]) | (tp.z[1:] != tp.z[:-1])
    idx = idx[moved & (tp.kind[1:] != KIND_RETRACT)]
    travel = tp.kind[idx] == KIND_TRAVEL
    return idx[~travel], idx[travel]


def preview_buffer(tp, which):
    """Float32 rows: x0 y0 z0 x1 y1 z1 kind level layer speed e_per_mm layer_height move.
    e_per_mm (E units per mm of path) and the layer height let the 3D view draw each bead at its real size;
    move (the toolpath move index) lets the robot simulation reveal the path up to where the nozzle is."""
    ext_idx, trv_idx = preview_indices(tp)
    idx = ext_idx if which == 'extrude' else trv_idx
    a = np.empty((len(idx), 13), dtype=np.float32)
    a[:, 0], a[:, 1], a[:, 2] = tp.x[idx - 1], tp.y[idx - 1], tp.z[idx - 1]
    a[:, 3], a[:, 4], a[:, 5] = tp.x[idx], tp.y[idx], tp.z[idx]
    a[:, 6], a[:, 7], a[:, 8], a[:, 9] = tp.kind[idx], tp.level[idx], tp.layer[idx], tp.f[idx]
    seg = np.sqrt((tp.x[idx] - tp.x[idx - 1]) ** 2 + (tp.y[idx] - tp.y[idx - 1]) ** 2 + (tp.z[idx] - tp.z[idx - 1]) ** 2)
    a[:, 10] = np.where(seg > 1e-9, (tp.e[idx] - tp.e[idx - 1]) / np.maximum(seg, 1e-9), 0.0) if len(idx) else 0.0
    lz = np.asarray(tp.layer_z, dtype=float)
    lh = np.diff(np.concatenate([[0.0], lz])) if len(lz) else np.zeros(1)
    lh[lh <= 0] = lz[0] if len(lz) and lz[0] > 0 else 1.0
    a[:, 11] = lh[np.clip(tp.layer[idx], 0, len(lh) - 1)] if len(idx) else 0.0
    a[:, 12] = idx
    return a.tobytes()


def run_slice(s):
    task = STATE.task
    try:
        def progress(stage, frac):
            task['stage'], task['progress'] = stage, float(frac)
        with STATE.lock:
            STATE.place(s)
            model = STATE.model
        tp = slice_to_toolpath(model, s, progress, cache=STATE.slice_cache)
        with STATE.lock:
            STATE.toolpath = tp
            STATE.source = 'slice'
        task['state'] = 'done'
    except Exception as e:
        traceback.print_exc()
        task['state'], task['error'] = 'error', '%s: %s' % (type(e).__name__, e)


class Handler(BaseHTTPRequestHandler):
    server_version = 'YoRA/' + __version__

    def log_message(self, fmt, *args):
        if '/api/status' not in (args[0] if args else ''):
            sys.stderr.write('%s\n' % (fmt % args))

    # ---- helpers ----
    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def send_bytes(self, data, ctype, filename=None, extra=None):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        if filename:
            self.send_header('Content-Disposition', 'attachment; filename="%s"' % filename)
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        n = int(self.headers.get('Content-Length') or 0)
        return self.rfile.read(n) if n else b''

    def json_body(self):
        data = self.body()
        return json.loads(data.decode('utf-8')) if data else {}

    def fail(self, msg, code=400):
        self.send_json({'error': msg}, code)

    # ---- routes ----
    def do_GET(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        try:
            if url.path == '/api/spec':
                return self.send_json({'spec': settings.spec_json(), 'defaults': settings.DEFAULTS,
                                       'profiles': settings.list_profiles(), 'version': __version__})
            if url.path == '/api/profile':
                return self.send_json(settings.load_profile(q.get('name', [''])[0]))
            if url.path == '/api/status':
                t = dict(STATE.task)
                if t['state'] == 'done' and STATE.toolpath is not None:
                    t['result'] = result_json(STATE.toolpath)
                return self.send_json(t)
            if url.path == '/api/model.bin':
                if STATE.model is None:
                    return self.fail('No model loaded', 404)
                return self.send_bytes(STATE.model.astype(np.float32).tobytes(), 'application/octet-stream')
            if url.path == '/api/toolpath.bin':
                if STATE.toolpath is None:
                    return self.fail('Nothing sliced yet', 404)
                return self.send_bytes(preview_buffer(STATE.toolpath, q.get('kind', ['extrude'])[0]), 'application/octet-stream')
            if url.path == '/api/robot/models':
                from . import kinematics
                return self.send_json({'models': kinematics.list_robots()})
            if url.path == '/api/robot/model':
                from . import kinematics
                m = kinematics.load_robot(q.get('name', [''])[0])
                d = m.to_json()
                keys = ('name', 'source', 'lower', 'upper', 'home', 'axes', 'points', 'links_home', 'flange_home', 'hull', 'post')
                return self.send_json({k: d[k] for k in keys if k in d} |
                                      {'fk_check_max_error': d.get('fk_check_max_error'),
                                       'has_mesh': os.path.isfile(kinematics.mesh_path(m.name))})
            if url.path == '/api/robot/status':
                t = dict(STATE.rtask)
                if t['state'] == 'done' and STATE.robot_path is not None:
                    t['result'] = robot_result_json(STATE.robot_path)
                return self.send_json(t)
            if url.path == '/api/robot/sim.bin':
                if STATE.robot_samples is None:
                    return self.fail('Run the robot check first', 404)
                return self.send_bytes(STATE.robot_samples.astype(np.float32).tobytes(), 'application/octet-stream')
            if url.path == '/api/robot/mesh.bin':
                from . import kinematics
                p = kinematics.mesh_path(os.path.basename(q.get('name', [''])[0]))
                if not os.path.isfile(p):
                    return self.fail('No 3D model for this robot: load its .robot file', 404)
                with open(p, 'rb') as f:
                    return self.send_bytes(f.read(), 'application/octet-stream')
            if url.path == '/api/posts':
                from . import post
                return self.send_json({'posts': post.list_posts(), 'events': post.EVENTS, 'placeholders': post.PLACEHOLDERS,
                                       'languages': {k: {'label': v['label'], 'raw': v['raw'],
                                                         'options': [{'key': o[0], 'label': o[1], 'type': o[2], 'default': o[3], **o[4]}
                                                                     for o in v['options']]} for k, v in post.LANGUAGES.items()}})
            if url.path == '/api/post':
                from . import post
                return self.send_json(post.load_post(q.get('name', [''])[0]))
            if url.path == '/api/materials':
                from . import material
                return self.send_json({'materials': material.list_materials(), 'types': material.TYPES})
            if url.path == '/api/cells':
                from . import cell
                return self.send_json({'cells': cell.list_cells()})
            if url.path == '/api/cell':
                from . import cell
                STATE.robot_path = None          # the cell may change the robot's joint limits
                return self.send_json(cell.load_cell(q.get('name', [''])[0]))
            if url.path == '/api/cell/export':
                from . import cell
                name, data = cell.export_bundle(q.get('name', [''])[0])
                return self.send_bytes(data, 'application/zip', filename=name + '.yoracell')
            if url.path == '/api/robot/tools':
                from . import tool
                return self.send_json({'tools': tool.list_tools()})
            if url.path == '/api/robot/tool.bin':
                from . import tool
                name = q.get('name', [''])[0]
                if name not in tool.list_tools():
                    return self.fail('Tool model not found', 404)
                return self.send_bytes(tool.mesh_bytes(name), 'application/octet-stream')
            if url.path == '/api/robot/library':
                from . import robot_file
                return self.send_json({'files': robot_file.find_robot_files()})
            if url.path == '/api/patterns':
                from . import pattern
                return self.send_json(pattern.list_images())
            if url.path == '/api/pattern/image':
                from . import pattern
                full = pattern.image_path(q.get('name', [''])[0])
                if not full or not os.path.isfile(full):
                    return self.fail('Not found', 404)
                with open(full, 'rb') as f:
                    return self.send_bytes(f.read(), mimetypes.guess_type(full)[0] or 'application/octet-stream')
            if url.path == '/api/samples':
                files = sorted(f for f in os.listdir(SAMPLES_DIR)
                               if os.path.splitext(f)[1].lower() in ('.stl', '.obj', '.3mf', '.gcode')) if os.path.isdir(SAMPLES_DIR) else []
                return self.send_json(files)
            if url.path.startswith('/samples/'):
                full = os.path.normpath(os.path.join(SAMPLES_DIR, os.path.basename(url.path)))
                if not os.path.isfile(full):
                    return self.fail('Not found', 404)
                with open(full, 'rb') as f:
                    return self.send_bytes(f.read(), 'application/octet-stream')
            return self.static(url.path)
        except Exception as e:
            traceback.print_exc()
            return self.fail('%s: %s' % (type(e).__name__, e), 500)

    def do_POST(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        try:
            if url.path == '/api/model':
                return self.post_model(q)
            if url.path == '/api/shutdown':
                # a newer copy of the app is starting on this port: hand it over (the server only listens on 127.0.0.1;
                # the custom header can't be sent by other web pages without a CORS preflight, which is never allowed)
                if self.headers.get('X-YoRA-Takeover') != '1':
                    return self.fail('Forbidden', 403)
                self.send_json({'stopping': True, 'pid': os.getpid()})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return
            if url.path == '/api/object':
                return self.post_object()
            if url.path == '/api/slice':
                return self.post_slice()
            if url.path == '/api/gcode':
                return self.post_gcode(q)
            if url.path == '/api/profile':
                b = self.json_body()
                return self.send_json({'saved': settings.save_profile(b['name'], b['settings']),
                                       'profiles': settings.list_profiles()})
            if url.path == '/api/export/gcode':
                return self.export_gcode()
            if url.path == '/api/export/act':
                return self.export_act()
            if url.path == '/api/robot/upload':
                return self.robot_upload(q)
            if url.path == '/api/post':
                from . import post
                return self.send_json(post.save_post(self.json_body()))
            if url.path == '/api/material':
                from . import material
                body = self.json_body()
                return self.send_json(material.save_material(body.get('material') or {}, body.get('old_name')))
            if url.path == '/api/material/delete':
                from . import material
                material.delete_material(self.json_body()['name'])
                return self.send_json({'materials': material.list_materials()})
            if url.path == '/api/post/delete':
                from . import post
                post.delete_post(self.json_body()['name'])
                return self.send_json({'posts': post.list_posts()})
            if url.path == '/api/post/check':
                from . import post
                return self.send_json({'problems': post.check_post(self.json_body())})
            if url.path == '/api/post/preview':
                return self.post_preview()
            if url.path == '/api/export/program':
                return self.export_program()
            if url.path == '/api/cell':
                from . import cell
                b = self.json_body()
                return self.send_json(cell.save_cell(b['name'], b.get('settings'), b.get('joints')))
            if url.path == '/api/cell/delete':
                from . import cell
                cell.delete_cell(self.json_body()['name'])
                return self.send_json({'cells': cell.list_cells()})
            if url.path == '/api/cell/import':
                from . import cell
                STATE.robot_path = None
                return self.send_json(cell.import_bundle(self.body()))
            if url.path == '/api/pattern/image':
                from . import pattern
                return self.send_json({'name': pattern.save_image(self.body(), q.get('name', ['picture.png'])[0])})
            if url.path == '/api/robot/tool_upload':
                from . import tool
                name, info = tool.save_tool(self.body(), q.get('name', ['tool.stl'])[0])
                return self.send_json(dict(info, name=name))
            if url.path.startswith('/api/robot/'):
                return self.robot_post(url.path[len('/api/robot/'):])
            return self.fail('Unknown endpoint', 404)
        except Exception as e:
            traceback.print_exc()
            return self.fail('%s: %s' % (type(e).__name__, e), 500)

    def robot_upload(self, q):
        """A .robot file dropped on the app: keep a copy in profiles/robots and import it."""
        from . import kinematics
        name = os.path.basename(q.get('name', ['robot.robot'])[0])
        if not name.lower().endswith('.robot'):
            return self.fail('Pick a .robot robot model file')
        os.makedirs(kinematics.ROBOTS_DIR, exist_ok=True)
        path = os.path.join(kinematics.ROBOTS_DIR, name)
        with open(path, 'wb') as f:
            f.write(self.body())
        return self.send_json(self.robot_loaded(kinematics.import_robot_file(path)))

    @staticmethod
    def robot_loaded(model):
        from . import kinematics
        STATE.robot_path = None
        return {'name': model.name, 'post': model.data.get('post', ''), 'source': model.data['source'],
                'has_mesh': os.path.isfile(kinematics.mesh_path(model.name))}

    def robot_post(self, action):
        from . import kinematics, robotpath
        b = self.json_body()
        if action == 'load_file':
            path = os.path.normpath(b['path'])
            if not (path.lower().endswith('.robot') and os.path.isfile(path)):
                return self.fail('Not a .robot file: %s' % path)
            return self.send_json(self.robot_loaded(kinematics.import_robot_file(path)))
        if action == 'limits':
            model = kinematics.load_robot(b['name'])
            lower, upper = [float(v) for v in b['lower']], [float(v) for v in b['upper']]
            if len(lower) != 6 or len(upper) != 6 or any(l >= u for l, u in zip(lower, upper)):
                return self.fail('Each joint needs lower < upper')
            model.lower, model.upper = np.array(lower), np.array(upper)
            kinematics.save_robot(model)
            STATE.robot_path = None
            return self.send_json({'saved': True})
        s = settings.normalize(b.get('settings'))
        if action == 'check':
            if STATE.toolpath is None:
                return self.fail('Slice a model or import G-code first')
            if STATE.rtask['state'] == 'running':
                return self.fail('Robot check already running')
            STATE.rtask = {'state': 'running', 'stage': 'robot', 'progress': 0.0, 'error': None}
            threading.Thread(target=run_robot_check, args=(s,), daemon=True).start()
            return self.send_json({'started': True})
        if action == 'pose':
            model = kinematics.load_robot(s['robot_model'])
            joints = b.get('joints')
            if joints is None:
                pref = settings.parse_float_list(s['start_joints']) if s['start_joints'].strip() else None
                joints = pref if pref and len(pref) == 6 else model.home.tolist()
            return self.send_json({'points': robotpath.robot_points_bed(model, s, np.array(joints, dtype=float)),
                                   'joints': list(joints)})
        if action == 'jog':
            # Move the robot by hand: joints given directly, or a TCP pose in the bed frame solved with IK
            model = kinematics.load_robot(s['robot_model'])
            q = np.array(b['joints'], dtype=float)
            if b.get('pose') is not None:
                q_new = robotpath.jog_ik(model, s, q, [float(v) for v in b['pose']])
                if q_new is None:
                    return self.send_json({'ok': False, 'joints': q.tolist(), 'tcp': robotpath.tcp_bed(model, s, q),
                                           'problems': ['unreachable']})
                q = q_new
            sv = robotpath.Solver(model, s)
            bits = sv.problems(q) | sv.collision(q)
            return self.send_json({'ok': True, 'joints': q.round(4).tolist(), 'tcp': [round(v, 3) for v in robotpath.tcp_bed(model, s, q)],
                                   'problems': [nm for bit, nm in robotpath.ERR_NAMES.items() if bits & bit]})
        if action == 'tool_tip':
            from . import tool
            return self.send_json({'tcp': tool.find_nozzle_tip(s)})
        if action == 'reach':
            model = kinematics.load_robot(s['robot_model'])
            return self.send_json(robotpath.reach_map(model, s, int(b.get('n', 21)), int(b.get('n', 21)), float(b.get('z', 0.0))))
        return self.fail('Unknown robot action', 404)

    def post_model(self, q):
        name = os.path.basename(q.get('name', ['model.stl'])[0])
        s = settings.normalize(json.loads(q.get('settings', ['{}'])[0]))
        tris = mesh.load_mesh(self.body(), name)
        with STATE.lock:
            STATE.model_name, STATE.model_raw = name, tris
            STATE.transform = {'scale': [100, 100, 100], 'rotation': [0, 0, 0], 'center': None}
            STATE.job_name = robot.safe_name(os.path.splitext(name)[0])
            STATE.toolpath = None
            STATE.task = {'state': 'idle', 'stage': '', 'progress': 0.0, 'error': None, 'started': 0}
            STATE.place(s)
            return self.send_json({'model': STATE.model_info(s)})

    def post_object(self):
        b = self.json_body()
        s = settings.normalize(b.get('settings'))
        with STATE.lock:
            if STATE.model_raw is None:
                return self.fail('No model loaded')
            t = b.get('transform', {})
            STATE.transform = {'scale': [float(v) for v in t.get('scale', [100] * 3)],
                               'rotation': [float(v) for v in t.get('rotation', [0] * 3)],
                               'center': [float(v) for v in t['center']] if t.get('center') else None}
            STATE.place(s)
            return self.send_json({'model': STATE.model_info(s)})

    def post_slice(self):
        b = self.json_body()
        s = settings.normalize(b.get('settings'))
        if STATE.model_raw is None:
            return self.fail('Load a model first')
        if STATE.task['state'] == 'running':
            return self.fail('Already slicing')
        STATE.task = {'state': 'running', 'stage': 'slicing', 'progress': 0.0, 'error': None, 'started': time.time()}
        threading.Thread(target=run_slice, args=(s,), daemon=True).start()
        return self.send_json({'started': True})

    def post_gcode(self, q):
        name = os.path.basename(q.get('name', ['print.gcode'])[0])
        s = settings.normalize(json.loads(q.get('settings', ['{}'])[0]))
        text = self.body().decode('utf-8', errors='replace')
        tp, info = gcode.read_gcode(text, s)
        with STATE.lock:
            STATE.toolpath, STATE.source = tp, 'gcode'
            STATE.job_name = robot.safe_name(os.path.splitext(name)[0])
            STATE.task = {'state': 'done', 'stage': '', 'progress': 1.0, 'error': None, 'started': time.time()}
        out = result_json(tp)
        out['import'] = info
        return self.send_json(out)

    def _toolpath_for_export(self, s):
        tp = STATE.toolpath
        if tp is None:
            raise RuntimeError('Slice a model or import G-code first')
        # Flow levels and full scale follow the current Extruder I/O settings
        from .planner import analyze
        analyze(tp, s)
        return tp

    def export_gcode(self):
        s = settings.normalize(self.json_body().get('settings'))
        tp = self._toolpath_for_export(s)
        text = gcode.write_gcode(tp, s, STATE.job_name).encode('utf-8')
        path = self._save(STATE.job_name + '.gcode', text)
        return self.send_bytes(text, 'text/plain', os.path.basename(path), {'X-Saved-Path': path})

    def export_act(self):
        s = settings.normalize(self.json_body().get('settings'))
        tp = self._toolpath_for_export(s)
        zip_path, info = robot.export_act(tp, s, OUT_DIR, STATE.job_name, current_robot_path(s))
        rp = current_robot_path(s)
        info['robot_ok'] = bool(rp.summary['ok']) if rp is not None else None
        with open(zip_path, 'rb') as f:
            data = f.read()
        return self.send_bytes(data, 'application/zip', os.path.basename(zip_path),
                               {'X-Saved-Path': zip_path, 'X-Export-Info': json.dumps(info)})

    def _post_def(self, b, s):
        from . import post
        return b.get('post') or post.load_post(s['post_name'] if s['post_name'] else 'Borunte .act')

    def post_preview(self):
        from . import post
        b = self.json_body()
        s = settings.normalize(b.get('settings'))
        if STATE.toolpath is None:
            return self.fail('Slice a model or import G-code first, then preview the program')
        tp = self._toolpath_for_export(s)
        return self.send_json(post.preview(tp, s, STATE.job_name, current_robot_path(s), self._post_def(b, s),
                                           int(b.get('lines', 80))))

    def export_program(self):
        """Export with the chosen post-processor (Borunte .act, KUKA KRL, G-code, or a user-made one)."""
        from . import post
        b = self.json_body()
        s = settings.normalize(b.get('settings'))
        tp = self._toolpath_for_export(s)
        rp = current_robot_path(s)
        path, info = post.export_program(tp, s, OUT_DIR, STATE.job_name, rp, self._post_def(b, s))
        info['robot_ok'] = bool(rp.summary['ok']) if rp is not None else None
        with open(path, 'rb') as f:
            data = f.read()
        ctype = 'application/zip' if path.endswith('.zip') else 'text/plain'
        return self.send_bytes(data, ctype, os.path.basename(path), {'X-Saved-Path': path, 'X-Export-Info': json.dumps(info)})

    def _save(self, filename, data):
        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, filename)
        with open(path, 'wb') as f:
            f.write(data)
        return path

    def static(self, path):
        rel = 'index.html' if path in ('', '/') else path.lstrip('/')
        if rel.rstrip('/') == 'classic' and not rel.endswith('/'):
            self.send_response(302)                       # /classic -> /classic/ so relative links resolve
            self.send_header('Location', '/classic/')
            self.end_headers()
            return
        if rel.endswith('/'):
            rel += 'index.html'                           # the classic interface lives in web/classic/
        full = os.path.normpath(os.path.join(WEB_DIR, rel))
        if not full.startswith(os.path.normpath(WEB_DIR)) or not os.path.isfile(full):
            return self.fail('Not found', 404)
        with open(full, 'rb') as f:
            data = f.read()
        ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
        if full.endswith('.js'):
            ctype = 'text/javascript'
        return self.send_bytes(data, ctype)


def open_app_window(url):
    """Open as a standalone app window (Edge/Chrome --app), else the default browser."""
    candidates = [os.path.expandvars(r'%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe'),
                  os.path.expandvars(r'%ProgramFiles%\Microsoft\Edge\Application\msedge.exe'),
                  os.path.expandvars(r'%ProgramFiles%\Google\Chrome\Application\chrome.exe'),
                  shutil.which('msedge') or '', shutil.which('chrome') or '']
    for exe in candidates:
        if exe and os.path.isfile(exe):
            subprocess.Popen([exe, '--app=' + url, '--window-size=1500,920'])
            return
    webbrowser.open(url)


class AppServer(ThreadingHTTPServer):
    # Never share the port: on Windows, address reuse lets a second server bind next to an old one that keeps answering
    allow_reuse_address = False
    daemon_threads = True


def take_over_port(port, timeout=5.0):
    """If an earlier YoRA Slicer still runs on the port (closing its window doesn't stop the server), ask it to stop,
    so this copy - the code just launched - serves the app. Returns a note for the console."""
    import urllib.request
    base = 'http://127.0.0.1:%i' % port
    try:
        with urllib.request.urlopen(base + '/api/spec', timeout=1.0) as r:
            old = json.loads(r.read().decode('utf-8')).get('version', '?')
    except Exception:
        return ''                     # nothing there, or not ours
    try:
        req = urllib.request.Request(base + '/api/shutdown', data=b'{}', method='POST', headers={'X-YoRA-Takeover': '1'})
        urllib.request.urlopen(req, timeout=2.0).read()
    except Exception:
        pass
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            urllib.request.urlopen(base + '/api/spec', timeout=0.3)
            time.sleep(0.2)
        except Exception:
            return 'Stopped an older YoRA Slicer (%s) that was still running on port %i.' % (old, port)
    return 'An older YoRA Slicer on port %i did not stop.' % port


def main():
    ap = argparse.ArgumentParser(description='YoRA Slicer')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--open', action='store_true', help='open the GUI window')
    ap.add_argument('--ui', choices=['new', 'classic'], default='new', help='which interface the window opens')
    args = ap.parse_args()
    note = take_over_port(args.port)
    if note:
        print(note)
    try:
        httpd = AppServer(('127.0.0.1', args.port), Handler)
    except OSError as e:
        print('Port %i is in use by another program (%s). Close it, or start with --port <another port>.' % (args.port, e))
        sys.exit(1)
    url = 'http://127.0.0.1:%i/' % args.port
    print('YoRA Slicer %s running at %s  (Ctrl+C to stop)' % (__version__, url))
    if args.open:
        threading.Timer(0.5, open_app_window, args=(url + ('classic/' if args.ui == 'classic' else ''),)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
