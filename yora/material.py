"""Material library: name, brand, type, heating zones with their temperatures, bed temperature, drying, density.

Materials are JSON files in profiles/materials/ (yours), plus a few built-in examples that are read only (Duplicate
to make your own). The selected material is the 'material' setting: it is saved with print profiles and robot cells,
sets the density (and the flow multiplier when the material has one) and fills the {material}, {zone_temps},
{nozzle_temp} ... placeholders of the post-processor events (yora/post.py).
"""
import json
import os
import re

from .settings import PROFILE_DIR

MATERIALS_DIR = os.path.join(PROFILE_DIR, 'materials')
FORMAT = 'yora-material-1'
TYPES = ['PLA', 'PETG', 'ABS', 'ASA', 'PP', 'PE', 'TPU', 'PA', 'PC', 'Composite', 'Other']
MAX_ZONES = 12

# Examples only: typical starting points, check the material data sheet.
BUILTIN = {
    'Generic PLA pellets': {'brand': 'Generic', 'type': 'PLA', 'color': '#e8e2d4', 'density': 1.24, 'bed_temp': 60,
                            'zones': [{'name': 'Nozzle', 'temp': 205}],
                            'drying_temp': 55, 'drying_hours': 4,
                            'notes': 'Example values for a single heating zone. Check your material data sheet.'},
    'Generic PETG pellets': {'brand': 'Generic', 'type': 'PETG', 'color': '#9fc6d8', 'density': 1.27, 'bed_temp': 75,
                             'zones': [{'name': 'Feed', 'temp': 200}, {'name': 'Zone 2', 'temp': 225},
                                       {'name': 'Zone 3', 'temp': 235}, {'name': 'Nozzle', 'temp': 240}],
                             'drying_temp': 65, 'drying_hours': 4,
                             'notes': 'Example values for a 4-zone pellet extruder, rising towards the nozzle.'},
    'Generic PP-GF pellets': {'brand': 'Generic', 'type': 'Composite', 'color': '#8a8a8a', 'density': 1.05, 'bed_temp': 0,
                              'zones': [{'name': 'Feed', 'temp': 190}, {'name': 'Zone 2', 'temp': 215},
                                        {'name': 'Nozzle', 'temp': 225}],
                              'drying_temp': 80, 'drying_hours': 2,
                              'notes': 'Glass-filled polypropylene, 3 zones. Example values: check your data sheet.'},
}


def _num(v, default=0.0, lo=None, hi=None):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    if lo is not None:
        x = max(lo, x)
    if hi is not None:
        x = min(hi, x)
    return round(x, 3)


def normalize(d):
    """A clean material definition (raises ValueError if it cannot be one)."""
    name = str(d.get('name', '')).strip()
    if not name:
        raise ValueError('The material needs a name')
    zones = []
    for i, z in enumerate(d.get('zones') or []):
        zname = str((z or {}).get('name', '')).strip() or ('Zone %d' % (i + 1))
        zones.append({'name': zname[:40], 'temp': _num((z or {}).get('temp'), 200.0, 0, 600)})
    if not zones:
        raise ValueError('Add at least one heating zone with its temperature')
    if len(zones) > MAX_ZONES:
        raise ValueError('At most %d heating zones' % MAX_ZONES)
    color = str(d.get('color', '') or '')
    return {
        'format': FORMAT, 'name': name[:80],
        'brand': str(d.get('brand', '')).strip()[:80],
        'type': str(d.get('type', '')).strip()[:40],
        'color': color if re.fullmatch(r'#[0-9a-fA-F]{6}', color) else '',
        'zones': zones,
        'bed_temp': _num(d.get('bed_temp'), 0.0, 0, 200),
        'density': _num(d.get('density'), 1.2, 0.1, 20),
        'flow': _num(d.get('flow'), 0.0, 0, 300),             # 0 = keep the print profile's flow multiplier
        'drying_temp': _num(d.get('drying_temp'), 0.0, 0, 200),
        'drying_hours': _num(d.get('drying_hours'), 0.0, 0, 96),
        'notes': str(d.get('notes', ''))[:2000],
    }


def _path(name):
    safe = re.sub(r'[^A-Za-z0-9 _.-]', '_', name).strip() or 'material'
    return os.path.join(MATERIALS_DIR, safe + '.json')


def list_materials():
    out = [dict(normalize(dict(d, name=n)), builtin=True) for n, d in BUILTIN.items()]
    if os.path.isdir(MATERIALS_DIR):
        for f in sorted(os.listdir(MATERIALS_DIR), key=str.lower):
            if f.endswith('.json'):
                try:
                    out.append(load_material(os.path.splitext(f)[0]))
                except Exception:   # not a material file
                    pass
    return out


def load_material(name):
    if name in BUILTIN:
        return dict(normalize(dict(BUILTIN[name], name=name)), builtin=True)
    with open(_path(name), 'r', encoding='utf-8') as f:
        d = json.load(f)
    if d.get('format') != FORMAT:
        raise ValueError('Not an YoRA material')
    return dict(normalize(d), builtin=False)


def find_material(name):
    """The material called `name`, or None (no material selected, or the file is gone)."""
    if not name:
        return None
    try:
        return load_material(name)
    except Exception:
        return None


def save_material(defn, old_name=None):
    d = normalize(defn)
    if d['name'] in BUILTIN:
        raise ValueError('"%s" is a built-in example: save yours under another name' % d['name'])
    os.makedirs(MATERIALS_DIR, exist_ok=True)
    with open(_path(d['name']), 'w', encoding='utf-8') as f:
        json.dump(d, f, indent=1)
    if old_name and old_name != d['name'] and old_name not in BUILTIN and os.path.isfile(_path(old_name)):
        os.remove(_path(old_name))                         # renamed
    return dict(d, builtin=False)


def delete_material(name):
    if name in BUILTIN:
        raise ValueError('Built-in examples cannot be deleted')
    p = _path(name)
    if os.path.isfile(p):
        os.remove(p)


def zone_text(m):
    """'Feed 200C, Nozzle 240C': plain ASCII, as robot controllers store programs in old code pages."""
    return ', '.join('%s %gC' % (z['name'], z['temp']) for z in m['zones'])


def placeholders(s):
    """Post-processor placeholders for the selected material (empty strings when none is selected)."""
    m = find_material(s.get('material', ''))
    if m is None:
        return {'material': '', 'material_brand': '', 'material_type': '', 'zone_temps': '', 'nozzle_temp': '',
                'zone_count': 0, 'bed_temp': ''}
    out = {'material': m['name'], 'material_brand': m['brand'], 'material_type': m['type'],
           'zone_temps': zone_text(m), 'nozzle_temp': '%g' % m['zones'][-1]['temp'], 'zone_count': len(m['zones']),
           'bed_temp': '%g' % m['bed_temp']}
    for i, z in enumerate(m['zones'], 1):
        out['temp_%d' % i] = '%g' % z['temp']
    return out


PLACEHOLDERS = {
    'material': 'material name', 'material_brand': 'material brand', 'material_type': 'material type',
    'zone_temps': 'all heating zones, e.g. "Feed 200C, Nozzle 240C"', 'nozzle_temp': 'temperature of the last zone (°C)',
    'temp_1': 'temperature of heating zone 1 (temp_2, temp_3 ... for the others)', 'zone_count': 'number of heating zones',
    'bed_temp': 'bed temperature (°C)',
}
