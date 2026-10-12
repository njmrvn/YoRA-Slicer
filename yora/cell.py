"""Robot cells: save and load the whole robot setup (robot, tool, frames, positions).

A cell holds every 'Robot cell' and 'Robot & bed' setting (robot model, tool TCP, tool 3D model and where it sits on
the flange, bed frame and size, controller frame/tool numbers, rotation, configuration and check settings, program
event), the robot's joint limits and the jogged robot position.

    profiles/cells/<name>.json   saved in the project; the robot model and tool model are referenced by name
    <name>.yoracell               a zip to move a cell to another PC: cell.json plus the robot model (kinematics and
                                 3D links) and the tool model, so nothing else has to be copied
"""
import io
import json
import os
import re
import time
import zipfile

from . import kinematics, tool
from .settings import PROFILE_DIR, SPEC, normalize

CELLS_DIR = os.path.join(PROFILE_DIR, 'cells')
FORMAT = 'yora-cell'
VERSION = 1
CELL_GROUPS = ('Robot cell', 'Robot & bed')


def cell_keys():
    return [k for g, k, *_ in SPEC if g in CELL_GROUPS]


def safe_name(name):
    return re.sub(r'[^A-Za-z0-9 _.-]', '_', str(name)).strip(' .') or 'Robot cell'


def cell_path(name):
    return os.path.join(CELLS_DIR, safe_name(name) + '.json')


def list_cells():
    if not os.path.isdir(CELLS_DIR):
        return []
    return sorted(os.path.splitext(f)[0] for f in os.listdir(CELLS_DIR) if f.endswith('.json'))


def make_cell(name, values, joints=None):
    s = normalize(values)
    cell = {'format': FORMAT, 'version': VERSION, 'name': safe_name(name), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
            'settings': {k: s[k] for k in cell_keys()},
            'joints': [round(float(v), 4) for v in joints] if joints is not None and len(joints) == 6 else None}
    if s['robot_model'] in kinematics.list_robots():
        r = kinematics.load_robot(s['robot_model'])
        cell['robot_limits'] = {'lower': r.lower.tolist(), 'upper': r.upper.tolist()}
    return cell


def save_cell(name, values, joints=None):
    cell = make_cell(name, values, joints)
    os.makedirs(CELLS_DIR, exist_ok=True)
    with open(cell_path(cell['name']), 'w', encoding='utf-8') as f:
        json.dump(cell, f, indent=1)
    return cell


def check_cell(cell):
    """Validate a cell dict; returns it with only known settings."""
    if not isinstance(cell, dict) or cell.get('format') != FORMAT:
        raise ValueError('Not an YoRA robot cell')
    if int(cell.get('version', 0)) > VERSION:
        raise ValueError('This cell was saved by a newer version of the slicer')
    keys = set(cell_keys())
    cell['settings'] = {k: v for k, v in (cell.get('settings') or {}).items() if k in keys}
    return cell


def load_cell(name):
    """The cell, with its saved joint limits applied to the robot model, and a list of missing parts."""
    with open(cell_path(name), 'r', encoding='utf-8') as f:
        cell = check_cell(json.load(f))
    return apply_assets(cell)


def apply_assets(cell):
    s = cell['settings']
    missing = []
    robot = s.get('robot_model', '')
    if robot and robot in kinematics.list_robots():
        lim = cell.get('robot_limits')
        if lim and len(lim.get('lower', [])) == 6 and len(lim.get('upper', [])) == 6:
            r = kinematics.load_robot(robot)
            import numpy as np
            if not (np.allclose(r.lower, lim['lower']) and np.allclose(r.upper, lim['upper'])):
                r.lower, r.upper = np.array(lim['lower'], dtype=float), np.array(lim['upper'], dtype=float)
                kinematics.save_robot(r)
    elif robot:
        missing.append('robot model "%s"' % robot)
    t = s.get('tool_model', '')
    if t and t not in tool.list_tools():
        missing.append('tool model "%s"' % t)
    cell['missing'] = missing
    return cell


def delete_cell(name):
    p = cell_path(name)
    if os.path.isfile(p):
        os.remove(p)


# ---------------- .yoracell bundle (move a cell to another PC) ----------------
def export_bundle(name):
    with open(cell_path(name), 'r', encoding='utf-8') as f:
        cell = check_cell(json.load(f))
    s = cell['settings']
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('cell.json', json.dumps(cell, indent=1))
        robot = s.get('robot_model', '')
        if robot in kinematics.list_robots():
            z.write(os.path.join(kinematics.ROBOTS_DIR, robot + '.json'), 'robots/%s.json' % robot)
            if os.path.isfile(kinematics.mesh_path(robot)):
                z.write(kinematics.mesh_path(robot), 'robots/%s.mesh.bin' % robot)
        t = s.get('tool_model', '')
        if t in tool.list_tools():
            z.write(tool.tool_path(t), 'tools/%s.stl' % t)
    return cell['name'], buf.getvalue()


def import_bundle(data):
    """Install a .yoracell: its robot and tool models go to profiles/robots and profiles/tools, the cell to
    profiles/cells (existing files with the same names are replaced). Returns the loaded cell."""
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        names = z.namelist()
        if 'cell.json' not in names:
            raise ValueError('Not an YoRA cell file (cell.json missing)')
        cell = check_cell(json.loads(z.read('cell.json').decode('utf-8')))
        for n in names:
            base = os.path.basename(n)
            if not base or base != n.split('/')[-1] or '..' in n:
                continue
            if n.startswith('robots/') and (base.endswith('.json') or base.endswith('.mesh.bin')):
                os.makedirs(kinematics.ROBOTS_DIR, exist_ok=True)
                with open(os.path.join(kinematics.ROBOTS_DIR, base), 'wb') as f:
                    f.write(z.read(n))
            elif n.startswith('tools/') and base.lower().endswith('.stl'):
                os.makedirs(tool.TOOLS_DIR, exist_ok=True)
                with open(os.path.join(tool.TOOLS_DIR, base), 'wb') as f:
                    f.write(z.read(n))
                tool._load.cache_clear()
                tool._hull.cache_clear()
    os.makedirs(CELLS_DIR, exist_ok=True)
    with open(cell_path(cell['name']), 'w', encoding='utf-8') as f:
        json.dump({k: v for k, v in cell.items() if k != 'missing'}, f, indent=1)
    return apply_assets(cell)
