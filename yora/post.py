"""Post-processors: one program writer per robot language, with user-editable events.

A post-processor definition (built in, or a JSON file in profiles/posts/) holds
    name, language ('borunte_act' | 'kuka_krl' | 'gcode'), options for that language, and events.

Events are short templates run at fixed points of the program:
    program_start, program_end, layer_start, layer_end, extruder_on, extruder_off, travel_start, travel_end
Each line of an event is one of these commands (they work in every language) or raw code:
    DO <n> ON|OFF                switch a digital output
    WAIT <seconds>               dwell
    WAIT_DI <n> ON|OFF           wait for a digital input
    COMMENT <text>               a comment in the program
    anything else                copied as is (KUKA KRL and G-code only; Borunte .act has no raw code)
Placeholders such as {layer}, {z} or {program} are replaced (see PLACEHOLDERS).

The extruder itself (enable output + flow level outputs, start delay) is handled by the FGF logic from
posts/Borunte_FGF.py for the robot languages, so Borunte and KUKA programs switch it at the same points.
"""
import json
import math
import os
import re
import tempfile
import time
import zipfile

from .settings import PROFILE_DIR, parse_float_list
from .slicer import KIND_RETRACT, KIND_TRAVEL
from .strategy import tool_rotation

POSTS_DIR = os.path.join(PROFILE_DIR, 'posts')
FORMAT = 'yora-post'

EVENTS = [
    ('program_start', 'Program start', 'After the tool and base are selected, before the first move.'),
    ('layer_start', 'Layer start', 'Before the first move of each layer.'),
    ('extruder_on', 'Extruder on', 'Right after the extruder outputs switch on (and the start delay).'),
    ('extruder_off', 'Extruder off', 'Right after the extruder output switches off.'),
    ('travel_start', 'Before travel', 'Before the first move of a travel (the extruder switches off with that move).'),
    ('travel_end', 'After travel', 'After the last travel move, before printing resumes.'),
    ('layer_end', 'Layer end', 'After the last move of each layer.'),
    ('program_end', 'Program end', 'After the last move and the extruder is off.'),
]
EVENT_KEYS = [k for k, _, _ in EVENTS]

PLACEHOLDERS = {
    'program': 'program name', 'post': 'post-processor name', 'date': 'date and time of export',
    'layers': 'number of layers', 'layer': 'layer number (1 = first)', 'z': 'layer height above the bed (mm)',
    'layer_height': 'layer thickness (mm)', 'moves': 'number of moves', 'time_min': 'print time (min)',
    'material_g': 'material (g)', 'tool': 'controller tool number', 'base': 'controller base / user frame number',
    'extruder_do': 'extruder enable output',
}
from .material import PLACEHOLDERS as _MATERIAL_PLACEHOLDERS, placeholders as _material_placeholders  # noqa: E402
PLACEHOLDERS.update(_MATERIAL_PLACEHOLDERS)

LANGUAGES = {
    'borunte_act': {
        'label': 'Borunte .act (BRTIRUS)',
        'ext': '.zip',
        'raw': False,
        'options': [],
    },
    'kuka_krl': {
        'label': 'KUKA KRL (KR C4 / KSS 8)',
        'ext': '.zip',
        'raw': True,
        'options': [
            ('output_mode', 'Output switching', 'choice', 'trigger', {'options': ['trigger', 'out_c', 'direct'],
             'help': 'trigger: TRIGGER WHEN DISTANCE=0 DO $OUT[n], switched exactly at the start of the next move '
                     '(recommended). out_c: $OUT_C[n]. direct: $OUT[n] (switches early, in the advance run).'}),
            ('ptp_speed_pct', 'PTP speed', 'float', 20.0, {'unit': '%', 'min': 1, 'max': 100}),
            ('advance', '$ADVANCE', 'int', 3, {'min': 1, 'max': 5, 'help': 'Motion blocks planned ahead; needed for corner blending.'}),
            ('max_moves_per_file', 'Split into files of', 'int', 0, {'unit': 'moves', 'min': 0, 'max': 10000000,
             'help': '0 = one file. Large jobs can be split into sub-programs called in order from the main program.'}),
        ],
    },
    'gcode': {
        'label': 'G-code (absolute E)',
        'ext': '.gcode',
        'raw': True,
        'options': [
            ('do_on', 'Digital output ON', 'text', 'M42 P{n} S255', {'help': 'Code for "DO n ON"; {n} = output number.'}),
            ('do_off', 'Digital output OFF', 'text', 'M42 P{n} S0', {}),
            ('wait', 'Wait', 'text', 'G4 P{ms}', {'help': '{ms} milliseconds, {s} seconds.'}),
            ('wait_di', 'Wait for input', 'text', 'M226 P{n} S{state}', {'help': '{state} = 1 or 0.'}),
        ],
    },
}

BUILTIN = {
    'Borunte .act': {'language': 'borunte_act', 'options': {}, 'events': {},
                     'description': 'Borunte BRTIRUS controllers. Same files as before (byte for byte).'},
    'KUKA KRL (KR C4)': {
        'language': 'kuka_krl', 'options': {},
        'description': 'KUKA KR C4 with the extruder on digital outputs. Copy the .src and .dat to KRC:\\R1\\Program.',
        'events': {
            'program_start': 'COMMENT {program}: {layers} layers, {time_min} min, {material_g} g\nCOMMENT Base {base} = bed origin, tool {tool} = nozzle tip',
            'layer_start': 'COMMENT Layer {layer} of {layers}, Z {z} mm',
        }},
    'G-code': {'language': 'gcode', 'options': {},
               'description': 'Plain G-code with absolute E, for other machines and tools.',
               'events': {'layer_start': '; layer {layer} of {layers}, Z {z}'}},
}


# ---------------- definitions ----------------
def safe_name(name):
    return re.sub(r'[^A-Za-z0-9 _.()+-]', '_', str(name)).strip(' .') or 'Post'


def _path(name):
    return os.path.join(POSTS_DIR, safe_name(name) + '.json')


def default_options(language):
    return {k: d for k, _, _, d, _ in LANGUAGES[language]['options']}


def normalize(defn):
    """A complete, valid definition (unknown keys dropped, defaults filled in)."""
    lang = defn.get('language', 'borunte_act')
    if lang not in LANGUAGES:
        raise ValueError('Unknown post-processor language: %s' % lang)
    opts = default_options(lang)
    types = {k: t for k, _, t, _, _ in LANGUAGES[lang]['options']}
    for k, v in (defn.get('options') or {}).items():
        if k in opts:
            try:
                opts[k] = float(v) if types[k] == 'float' else int(float(v)) if types[k] == 'int' else str(v)
            except (TypeError, ValueError):
                pass
    events = {k: str((defn.get('events') or {}).get(k, '') or '') for k in EVENT_KEYS}
    return {'format': FORMAT, 'name': safe_name(defn.get('name', 'Post')), 'language': lang, 'options': opts,
            'events': events, 'description': str(defn.get('description', '') or '')}


def list_posts():
    out = [{'name': n, 'language': d['language'], 'builtin': True} for n, d in BUILTIN.items()]
    if os.path.isdir(POSTS_DIR):
        for f in sorted(os.listdir(POSTS_DIR)):
            if f.endswith('.json'):
                try:
                    d = load_post(os.path.splitext(f)[0])
                    out.append({'name': d['name'], 'language': d['language'], 'builtin': False})
                except Exception:
                    pass
    return out


def load_post(name):
    if name in BUILTIN:
        return dict(normalize(dict(BUILTIN[name], name=name)), builtin=True)
    with open(_path(name), 'r', encoding='utf-8') as f:
        d = json.load(f)
    if d.get('format') != FORMAT:
        raise ValueError('Not an YoRA post-processor')
    return dict(normalize(dict(d, name=name)), builtin=False)


def save_post(defn):
    d = normalize(defn)
    if d['name'] in BUILTIN:
        raise ValueError('"%s" is built in: save your version under another name' % d['name'])
    os.makedirs(POSTS_DIR, exist_ok=True)
    with open(_path(d['name']), 'w', encoding='utf-8') as f:
        json.dump(d, f, indent=1)
    return d


def delete_post(name):
    if name in BUILTIN:
        raise ValueError('Built-in post-processors cannot be deleted')
    p = _path(name)
    if os.path.isfile(p):
        os.remove(p)


# ---------------- events ----------------
CMD = re.compile(r'^\s*(DO|WAIT_DI|WAIT|COMMENT)\b\s*(.*)$', re.IGNORECASE)


def _fill(text, ctx):
    return re.sub(r'\{(\w+)\}', lambda m: str(ctx[m.group(1)]) if m.group(1) in ctx else m.group(0), text)


def parse_event(text):
    """Lines of an event as ('do', n, bool) / ('wait', s) / ('wait_di', n, bool) / ('comment', text) / ('raw', text).
    Raises ValueError with the line number for malformed commands."""
    out = []
    for k, line in enumerate(str(text or '').splitlines(), 1):
        if not line.strip():
            continue
        m = CMD.match(line)
        if not m:
            out.append(('raw', line.rstrip()))
            continue
        cmd, rest = m.group(1).upper(), m.group(2).strip()
        try:
            if cmd == 'COMMENT':
                out.append(('comment', rest))
            elif cmd == 'WAIT':
                out.append(('wait', float(rest)))
            else:
                parts = rest.split()
                n = int(parts[0])
                state = parts[1].upper() in ('ON', '1', 'TRUE', 'HIGH')
                if parts[1].upper() not in ('ON', 'OFF', '1', '0', 'TRUE', 'FALSE', 'HIGH', 'LOW'):
                    raise ValueError
                out.append(('do' if cmd == 'DO' else 'wait_di', n, state))
        except (ValueError, IndexError):
            raise ValueError('Line %i: "%s" - expected %s' % (k, line.strip(), {
                'WAIT': 'WAIT <seconds>', 'DO': 'DO <n> ON|OFF', 'WAIT_DI': 'WAIT_DI <n> ON|OFF'}.get(cmd, cmd)))
    return out


def check_post(defn):
    """List of problems in a definition's events (empty = fine)."""
    d = normalize(defn)
    problems = []
    for key, label, _ in EVENTS:
        try:
            cmds = parse_event(d['events'][key])
        except ValueError as e:
            problems.append('%s: %s' % (label, e))
            continue
        if not LANGUAGES[d['language']]['raw'] and any(c[0] == 'raw' for c in cmds):
            problems.append('%s: raw code lines are not possible in %s (use DO / WAIT / WAIT_DI)' % (label, LANGUAGES[d['language']]['label']))
    return problems


# ---------------- export ----------------
def _context(tp, s, name, post_name):
    st = tp.stats
    return {'program': name, 'post': post_name, 'date': time.strftime('%Y-%m-%d %H:%M'),
            'layers': st.get('layers', len(tp.layer_z)), 'moves': len(tp),
            'time_min': '%.0f' % (st.get('time_s', 0) / 60.0), 'material_g': '%.0f' % st.get('mass_g', 0),
            'tool': int(s['tool_id']), 'base': int(s['user_frame_id']), 'extruder_do': int(s['extruder_enable_do']),
            'layer': 0, 'z': 0, 'layer_height': 0, **_material_placeholders(s)}


def _layer_ctx(ctx, tp, layer):
    zs = tp.layer_z
    z = zs[layer] if 0 <= layer < len(zs) else 0.0
    h = z - (zs[layer - 1] if layer > 0 and layer - 1 < len(zs) else 0.0)
    ctx.update(layer=layer + 1, z='%.2f' % z, layer_height='%.2f' % h)


def export_program(tp, s, out_dir, name, robot_path=None, defn=None):
    """Write the program for the post-processor `defn` (default: Borunte .act). Returns (path, info)."""
    d = normalize(defn or dict(BUILTIN['Borunte .act'], name='Borunte .act'))
    problems = check_post(d)
    if problems:
        raise ValueError('Post-processor "%s": %s' % (d['name'], '; '.join(problems)))
    if d['language'] == 'gcode':
        return _export_gcode(tp, s, out_dir, name, d)
    return _export_robot(tp, s, out_dir, name, robot_path, d)


def _robot_post_class(language):
    from . import robot
    if language == 'borunte_act':
        return robot.load_post_class()
    robot.load_post_class()                       # puts posts/ on sys.path
    import sys
    from kuka_krl import KukaKrlPost
    fgf = sys.modules[robot.POST_NAME]
    return type('KukaFGF', (fgf.FGFMixin, KukaKrlPost), {})


def _export_robot(tp, s, out_dir, name, robot_path, d):
    from . import robot
    cls = _robot_post_class(d['language'])           # also puts posts/ on sys.path
    from borunte_act import eye, rotx, rotz, transl   # numpy poses
    name = robot.safe_name(name)
    r = cls(robot.POST_NAME, 'robot', 6)
    for k, v in robot.post_settings(s, tp.stats).items():
        setattr(r, k, v)
    r.SHOW_PROGRAM_ZIP = False
    r.SHOW_PROGRAM_TXT = False
    o = d['options']
    if d['language'] == 'kuka_krl':
        r.OUTPUT_MODE, r.PTP_SPEED_PCT = o['output_mode'], o['ptp_speed_pct']
        r.ADVANCE, r.MAX_MOVES_PER_FILE = o['advance'], o['max_moves_per_file']

    ctx = _context(tp, s, name, d['name'])
    parsed = {k: parse_event(d['events'][k]) for k in EVENT_KEYS}
    counts = {k: 0 for k in EVENT_KEYS}

    def run(event):
        cmds = parsed[event]
        if not cmds:
            return
        counts[event] += 1
        for c in cmds:
            if c[0] == 'do':
                r.setDO(c[1], 1 if c[2] else 0)
            elif c[0] == 'wait':
                r.Pause(c[1] * 1000.0)
            elif c[0] == 'wait_di':
                r.waitDI(c[1], 1 if c[2] else 0)
            elif c[0] == 'comment':
                r.RunMessage(_fill(c[1], ctx), True)
            else:
                r.RunCode(_fill(c[1], ctx), False)

    r.event_hook = run
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
    r.setZoneData(int(s['blending']) if d['language'] == 'borunte_act' else float(s['blending']))
    run('program_start')
    if joints:
        r.MoveJ(None, joints, None)
    last_e, last_f, prev = 0.0, None, None
    cur_layer, in_travel = None, False
    for i in range(len(tp)):
        x, y, z, e, f = float(tp.x[i]), float(tp.y[i]), float(tp.z[i]), float(tp.e[i]), float(tp.f[i])
        layer, kind = int(tp.layer[i]), int(tp.kind[i])
        if layer != cur_layer:
            if cur_layer is not None:
                run('layer_end')
            _layer_ctx(ctx, tp, layer)
            run('layer_start')
            cur_layer = layer
        if kind == KIND_TRAVEL and not in_travel:
            run('travel_start')
            in_travel = True
        elif kind != KIND_TRAVEL and kind != KIND_RETRACT and in_travel:
            run('travel_end')
            in_travel = False
        if abs(e - last_e) > 1e-7:
            r.RunCode('Extruder(%.5f)' % e, True)
            last_e = e
        here = (round(x, 4), round(y, 4), round(z, 4))
        if kind == KIND_RETRACT and here == prev:
            continue   # E change in place: the post consumes it on the next Extruder() call
        if f != last_f:
            r.setSpeed(f)
            last_f = f
        r.MoveL(transl(x, y, z) @ tool_rotation(tp, i) @ down @ rotz(math.radians(rz_of(i))), None, None)
        prev = here
    if cur_layer is not None:
        run('layer_end')
    if joints and robot_path is None and d['language'] == 'borunte_act':
        r.MoveJ(None, joints, None)
    r.ProgFinish(name)                       # extruder off (+ extruder_off event), then the program_end event
    path = r.ProgSave(os.path.abspath(out_dir), name, False, False)
    if not path:
        path = os.path.join(out_dir, name + '.zip')
    info = {'post': d['name'], 'language': d['language'], 'file': os.path.basename(path), 'events_run': counts,
            'robot_checked': robot_path is not None, 'log': (r.LOG or '').strip()}
    if d['language'] == 'borunte_act':
        info['instructions'] = len(r.PG)
    else:
        info['moves'] = r.moves
        info['files'] = sorted(r.program_files().keys())
    return path, info


def _export_gcode(tp, s, out_dir, name, d):
    from . import gcode
    o = d['options']
    ctx = _context(tp, s, name, d['name'])
    parsed = {k: parse_event(d['events'][k]) for k in EVENT_KEYS}

    def lines(event, layer=None):
        if layer is not None:
            _layer_ctx(ctx, tp, layer)
        out = []
        for c in parsed[event]:
            if c[0] == 'do':
                out.append(_fill(o['do_on' if c[2] else 'do_off'], {'n': c[1]}))
            elif c[0] == 'wait':
                out.append(_fill(o['wait'], {'ms': int(round(c[1] * 1000)), 's': '%g' % c[1]}))
            elif c[0] == 'wait_di':
                out.append(_fill(o['wait_di'], {'n': c[1], 'state': 1 if c[2] else 0}))
            elif c[0] == 'comment':
                out.append('; ' + _fill(c[1], ctx))
            else:
                out.append(_fill(c[1], ctx))
        return out

    text = gcode.write_gcode(tp, s, name, hook=lines)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name + '.gcode')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    return path, {'post': d['name'], 'language': 'gcode', 'file': os.path.basename(path), 'lines': text.count('\n')}


def preview(tp, s, name, robot_path, defn, max_lines=80):
    """The beginning of the program this post would write (main file), as text lines."""
    tmp = tempfile.mkdtemp()
    path, info = export_program(tp, s, tmp, name, robot_path, defn)
    if path.endswith('.gcode'):
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.read().splitlines()
    else:
        z = zipfile.ZipFile(path)
        main = [n for n in z.namelist() if n.lower().endswith(('.act', '.src'))]
        main = sorted(main, key=len)[0]
        text = z.read(main).decode('utf-8')
        if main.endswith('.act'):
            prog = json.loads(text.splitlines()[0])
            lines = ['%5i  %s' % (k + 1, json.dumps(ins, separators=(',', ':'))) for k, ins in enumerate(prog)]
        else:
            lines = text.splitlines()
        info['file_in_zip'] = main
    return {'lines': lines[:max_lines], 'total_lines': len(lines), 'info': info}
