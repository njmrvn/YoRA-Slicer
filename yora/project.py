"""YoRA project files (.yora): everything needed to open a job again, on this computer or another one.

A .yora file is a zip:
    project.json        format, app version, name, all settings, the parts (name, file, scale, rotation, position)
    parts/<n>.stl       each part as loaded (before scaling and rotating), binary STL
    profiles/...        the files the settings point to, so the project opens complete elsewhere: the material,
                        the post-processor, the tool model, the surface pattern picture and the robot model

Opening a project puts those files into profiles/ when they are missing. A file of the same name that is already
there and different is kept (the user's own copy wins) and reported.
"""
import io
import json
import os
import re
import zipfile

from . import __version__, mesh, settings
from .settings import PROFILE_DIR

FORMAT = 'yora-project-1'
EXT = '.yora'
MAX_UNPACKED = 1 << 30          # refuse projects that unpack to more than 1 GB
FOLDERS = ('materials', 'posts', 'tools', 'patterns', 'robots')


def safe_name(name):
    return re.sub(r'[^A-Za-z0-9 _.()+-]', '_', os.path.splitext(os.path.basename(str(name or '')))[0]).strip(' .') or 'Project'


def linked_files(s):
    """[(folder, file name, path)] of the files in profiles/ that the settings use and that exist."""
    from . import kinematics, material, pattern, post, tool
    out = []

    def add(folder, path):
        if path and os.path.isfile(path):
            out.append((folder, os.path.basename(path), path))
    if s.get('material'):
        add('materials', material._path(s['material']))
    if s.get('post_name') and s['post_name'] not in post.BUILTIN:
        add('posts', post._path(s['post_name']))
    if s.get('tool_model'):
        add('tools', tool.tool_path(s['tool_model']))
    if s.get('pattern_image'):
        add('patterns', pattern.image_path(s['pattern_image']))
    if s.get('robot_model'):
        add('robots', os.path.join(kinematics.ROBOTS_DIR, s['robot_model'] + '.json'))
        add('robots', kinematics.mesh_path(s['robot_model']))
    return out


def save(plate, s, name):
    """The project as .yora bytes."""
    s = settings.normalize(s)
    parts = []
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        for i, p in enumerate(plate.parts):
            fn = 'parts/%d.stl' % (i + 1)
            z.writestr(fn, mesh.to_stl_bytes(p['raw']))
            parts.append({'name': p['name'], 'file': fn, 'transform': p['transform']})
        files = []
        for folder, fname, path in linked_files(s):
            arc = 'profiles/%s/%s' % (folder, fname)
            z.write(path, arc)
            files.append(arc)
        z.writestr('project.json', json.dumps({'format': FORMAT, 'app': 'YoRA Slicer', 'version': __version__,
                                               'name': name, 'settings': s, 'parts': parts, 'files': files}, indent=1))
    return buf.getvalue()


def load(data):
    """Read .yora bytes. Returns {name, settings, parts: [(name, raw triangles, transform)], notes}; the linked
    files are put into profiles/ as described above."""
    try:
        z = zipfile.ZipFile(io.BytesIO(data))
    except zipfile.BadZipFile:
        raise ValueError('Not a YoRA project file')
    with z:
        if sum(i.file_size for i in z.infolist()) > MAX_UNPACKED:
            raise ValueError('The project is too large to open')
        try:
            meta = json.loads(z.read('project.json').decode('utf-8'))
        except KeyError:
            raise ValueError('Not a YoRA project file (project.json missing)')
        if meta.get('format') != FORMAT:
            raise ValueError('Unknown project format: %s' % meta.get('format'))
        names = set(z.namelist())
        parts = []
        for p in meta.get('parts', []):
            fn = str(p.get('file', ''))
            if fn not in names:
                raise ValueError('The project is missing %s' % fn)
            raw = mesh.load_mesh(z.read(fn), 'part.stl')
            t = p.get('transform') or {}
            transform = {'scale': [float(v) for v in t.get('scale', [100] * 3)],
                         'rotation': [float(v) for v in t.get('rotation', [0] * 3)],
                         'center': [float(v) for v in t['center']] if t.get('center') else None}
            parts.append((str(p.get('name') or 'part'), raw, transform))
        notes = []
        for arc in names:
            bits = arc.split('/')
            if len(bits) != 3 or bits[0] != 'profiles' or bits[1] not in FOLDERS or not bits[2]:
                continue
            fname = os.path.basename(bits[2])
            if fname != bits[2] or fname.startswith('.'):
                continue
            target = os.path.join(PROFILE_DIR, bits[1], fname)
            content = z.read(arc)
            if os.path.isfile(target):
                with open(target, 'rb') as f:
                    if f.read() != content:
                        notes.append('Kept your own %s/%s (the project has a different one).' % (bits[1], fname))
                continue
            os.makedirs(os.path.dirname(target), exist_ok=True)
            with open(target, 'wb') as f:
                f.write(content)
            notes.append('Added %s/%s from the project.' % (bits[1], fname))
    return {'name': safe_name(meta.get('name')), 'settings': settings.normalize(meta.get('settings')),
            'parts': parts, 'notes': sorted(notes)}
