"""Surface pattern: waves, spikes, bumps, noise or a picture pressed into the outer wall (settings group
'Surface pattern').

The model is sliced as usual; each outer wall loop (and each spiral vase turn) is then resampled along its length and
every point is moved sideways, perpendicular to the wall, towards the air, by

    offset = depth * height factor * paint mask * slope mask * (pattern(point) - base)

pattern() is 0..1. tau counts waves along the wall. With 'fit' (Whole waves per lap, the default) the outline is
followed up the part as a stack (same centre, similar perimeter, layer after layer) and the stack gets ONE whole wave
count N for its full height: N = round(mean perimeter / wavelength), or Waves per lap. On each lap
    closed loop:  tau = N * f + shift * layer
    spiral turn:  tau = (N + shift) * lambda,   lambda = layer + f   (continuous along the spiral: no step)
with f the fraction of the lap measured from a fixed direction (+X from the stack centre), not from the seam. The
effective wavelength (lap perimeter / N) follows the shape, so waves stay in phase from layer to layer whatever the
diameter: ribs stay vertical and diagonals keep their angle. Without fit: tau = distance / wavelength.

    sine     0.5 - 0.5 cos(2 pi f)                     f = wave fraction; the phase moves by the shift on every layer
    zigzag   triangle, sharp tip at mid-wave           (0 = stacked ribs, 50 % = lattice / mesh look)
    spikes   a sharp tip of the given width at mid-wave, plain wall between
    bumps    round domes on a grid: wavelength apart along the wall, row height apart up it; each row moves along
             by the shift (50 % = staggered)
    noise    smooth random relief (value noise), wavelength across and row height up; the seed picks the variation
    image    a greyscale picture wrapped round the part (cylindrical: angle about the part centre -> picture X,
             height -> picture Y); dark or light parts raised

base: outward 0 (the wall only grows), inward 1 (only cuts in), centered 0.5. The paint mask limits the pattern to the
areas painted on the part (or keeps it off them): brush strokes (points in bed coordinates) and surfaces picked with
the fill tool (triangles of a part, so they move with it). The slope mask keeps the pattern off walls that lean more
than a set angle. Both look up the part's surface next to each wall point: the triangles cut at that height.
Paths are (n, 2) or (n, 3) arrays; distances are measured in XY.

Pattern layers: up to MAX_LAYERS patterns on one part. Layer 1 is the pattern_* settings; layers 2.. are kept in
pattern_layers (JSON, each its own pattern_* values). Each has its own area (paint mask) and wave count. Going up
the list, a 'cover' layer replaces the layers below where its mask is (cross-fading over Edge blend at the edge of a
picked surface), an 'add' layer adds its relief to them:
    cover:  offset = offset * (1 - k) + d        add:  offset = offset + d        d = depth * k * (pattern - base)
Wave points go only where a layer shows; plain wall keeps the path's own points.
"""
import json
import math
import os

import numpy as np

from .settings import DEFAULTS, PROFILE_DIR, TYPES

PATTERNS = ('none', 'sine', 'zigzag', 'spikes', 'bumps', 'noise', 'image')
WAVES = ('sine', 'zigzag', 'spikes')          # 1D profiles along the wall, shifted per layer
BASES = {'outward': 0.0, 'inward': 1.0, 'centered': 0.5}
PATTERN_DIR = os.path.join(PROFILE_DIR, 'patterns')
IMAGE_EXT = ('.png', '.jpg', '.jpeg', '.bmp')
IMAGE_MAX = 1024                               # pictures are scaled down to this many pixels on the long side
MAX_LAYERS = 4
LAYER_KEYS = ('pattern_blend', 'pattern_hidden')   # per layer, besides the pattern_* settings


def _on(ls):
    """One layer's settings make a pattern."""
    return ls.get('pattern', 'none') in PATTERNS[1:] and ls.get('pattern_amplitude', 0) > 0 \
        and ls.get('pattern_wavelength', 0) > 0 and not ls.get('pattern_hidden', False)


def _coerce(v, key):
    typ, d = TYPES[key], DEFAULTS[key]
    try:
        if typ == 'bool':
            return v if isinstance(v, bool) else str(v).lower() in ('1', 'true', 'yes', 'on')
        if typ == 'int':
            return int(round(float(v)))
        if typ == 'float':
            return float(v)
        return str(v)
    except (TypeError, ValueError):
        return d


def extra_layers(s):
    """Layers 2.. as settings dicts: the defaults with the layer's own pattern_* values (not layer 1's)."""
    try:
        raw = json.loads(s.get('pattern_layers') or '[]')
    except (TypeError, ValueError):
        return []
    out = []
    base = {k: v for k, v in s.items() if not k.startswith('pattern')}
    base.update({k: v for k, v in DEFAULTS.items() if k.startswith('pattern') and k != 'pattern_layers'})
    for d in raw[:MAX_LAYERS - 1] if isinstance(raw, list) else []:
        if not isinstance(d, dict):
            continue
        ls = dict(base)
        for k, v in d.items():
            if k in ls and k.startswith('pattern') and k != 'pattern_layers':
                ls[k] = _coerce(v, k)
        out.append(ls)
    return out


def all_layers(s):
    """[(layer number from 1, settings)] of every layer, shown or not."""
    return [(1, s)] + [(i + 2, ls) for i, ls in enumerate(extra_layers(s))]


def shown_layers(s):
    """The layers that make a pattern (on, with depth, not hidden)."""
    return [(n, ls) for n, ls in all_layers(s) if _on(ls)]


def active(s):
    return bool(shown_layers(s))


# ------------------------------------------------------------------ pattern functions (0..1)
def profile(kind, f, width=0.5):
    """Wave height 0..1 at wave fraction f (sine, zigzag, spikes)."""
    f = np.mod(f, 1.0)
    if kind == 'zigzag':
        return 1.0 - np.abs(2.0 * f - 1.0)
    if kind == 'spikes':
        hw = max(min(width, 1.0), 0.02) / 2.0
        return np.clip(1.0 - np.abs(f - 0.5) / hw, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(2.0 * math.pi * f)


def corner_fractions(kind, width=0.5):
    """Wave fractions where the profile has a sharp corner: a point is placed exactly there."""
    if kind == 'zigzag':
        return [0.0, 0.5]
    if kind == 'spikes':
        hw = max(min(width, 1.0), 0.02) / 2.0
        return [0.5 - hw, 0.5, 0.5 + hw]
    return []


def bumps(tau, z, lam, row_h, width, shift):
    """Round domes: one per wave along the wall and per row up it, each row moved along by `shift` waves."""
    row = np.floor(z / row_h)
    fu = np.mod(tau + shift * row, 1.0) - 0.5
    fz = np.mod(z / row_h, 1.0) - 0.5
    r = max(min(width, 1.0), 0.02) * min(lam, row_h) / 2.0     # dome radius, mm
    d2 = ((fu * lam) ** 2 + (fz * row_h) ** 2) / (r * r)
    return np.sqrt(np.clip(1.0 - d2, 0.0, 1.0))


def _hash(i, j, seed):
    h = (i.astype(np.int64) * 374761393 + j.astype(np.int64) * 668265263 + seed * 1442695041) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return (h ^ (h >> 16)).astype(np.float64) / 4294967296.0


def value_noise(x, y, period, seed):
    """Smooth noise 0..1; repeats every `period` along x when period is a whole number (closes round a loop)."""
    i0, j0 = np.floor(x), np.floor(y)
    fx, fy = x - i0, y - j0
    sx, sy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    i0, j0 = i0.astype(np.int64), j0.astype(np.int64)
    i1 = i0 + 1
    if period:
        i0, i1 = np.mod(i0, period), np.mod(i1, period)
    h00, h10 = _hash(i0, j0, seed), _hash(i1, j0, seed)
    h01, h11 = _hash(i0, j0 + 1, seed), _hash(i1, j0 + 1, seed)
    return (h00 + (h10 - h00) * sx) * (1 - sy) + (h01 + (h11 - h01) * sx) * sy


def noise(tau, z, row_h, period, seed):
    v = value_noise(tau, z / row_h, period, seed) + 0.5 * value_noise(2 * tau, 2 * z / row_h, 2 * period if period else None, seed + 1)
    return np.clip((v / 1.5 - 0.2) / 0.6, 0.0, 1.0)            # spread the middle-heavy values over 0..1


# ------------------------------------------------------------------ pictures
_IMAGES = {}


def image_path(name):
    name = os.path.basename(str(name or ''))
    return os.path.join(PATTERN_DIR, name) if name else ''


def list_images():
    if not os.path.isdir(PATTERN_DIR):
        return []
    return sorted(f for f in os.listdir(PATTERN_DIR) if f.lower().endswith(IMAGE_EXT))


def decode_image(data):
    """Bytes of a PNG / JPG -> greyscale float array 0 (black) .. 1 (white), row 0 at the top. Transparent pixels
    count as white (a logo on a transparent background raises the logo when dark parts are raised)."""
    import cv2
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError('Not a picture this app can read (use PNG or JPG)')
    img = img.astype(np.float32) / (65535.0 if img.dtype == np.uint16 else 255.0)
    if img.ndim == 2:
        g = img
    else:
        g = img[:, :, :3] @ np.array([0.114, 0.587, 0.299], dtype=np.float32)   # BGR -> luminance
        if img.shape[2] == 4:
            a = img[:, :, 3]
            g = g * a + (1.0 - a)
    h, w = g.shape
    k = IMAGE_MAX / max(h, w)
    if k < 1:
        g = cv2.resize(g, (max(int(w * k), 1), max(int(h * k), 1)), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(g, dtype=np.float32)


def save_image(data, name):
    """Keep an uploaded picture in profiles/patterns (checked first). Returns the stored file name."""
    import re
    decode_image(data)
    base, ext = os.path.splitext(os.path.basename(str(name or 'picture.png')))
    ext = ext.lower() if ext.lower() in IMAGE_EXT else '.png'
    safe = re.sub(r'[^A-Za-z0-9 _.-]', '_', base).strip() or 'picture'
    os.makedirs(PATTERN_DIR, exist_ok=True)
    with open(os.path.join(PATTERN_DIR, safe + ext), 'wb') as f:
        f.write(data)
    return safe + ext


def load_image(name):
    path = image_path(name)
    if not path or not os.path.isfile(path):
        return None
    key = (path, os.path.getmtime(path))
    if key not in _IMAGES:
        with open(path, 'rb') as f:
            _IMAGES[key] = decode_image(f.read())
    return _IMAGES[key]


def sample_image(img, fx, fy):
    """Bilinear lookup at picture fractions fx (0 left .. 1 right), fy (0 top .. 1 bottom)."""
    h, w = img.shape
    px, py = np.clip(fx, 0, 1) * (w - 1), np.clip(fy, 0, 1) * (h - 1)
    x0, y0 = np.floor(px).astype(int), np.floor(py).astype(int)
    x1, y1 = np.minimum(x0 + 1, w - 1), np.minimum(y0 + 1, h - 1)
    tx, ty = px - x0, py - y0
    top = img[y0, x0] * (1 - tx) + img[y0, x1] * tx
    bot = img[y1, x0] * (1 - tx) + img[y1, x1] * tx
    return top * (1 - ty) + bot * ty


def picture(x, y, z, s, ctx):
    """Picture wrapped round the part: 0..1 (1 = raised), 0 outside the picture."""
    img = ctx.get('image')
    if img is None:
        return np.zeros_like(z)
    cx, cy = ctx['center']
    ang = np.degrees(np.arctan2(y - cy, x - cx))
    rel = np.mod(ang - s.get('pattern_image_angle', -90.0) + 180.0, 360.0) - 180.0   # 0 at the picture centre
    span = max(s.get('pattern_image_span', 360.0), 1.0)
    fx = rel / span + 0.5                    # angle grows to the right, as seen from outside
    inside = np.ones(len(z), dtype=bool)
    if s.get('pattern_image_tile', False):
        fx = np.mod(fx, 1.0)
    else:
        inside &= (fx >= 0) & (fx <= 1)
    z0 = s.get('pattern_z_min', 0.0)
    z1 = s.get('pattern_z_max', 0.0) or ctx.get('ztop', z0 + 1.0)
    fy = (z1 - z) / max(z1 - z0, 1e-6)
    inside &= (fy >= 0) & (fy <= 1)
    v = sample_image(img, fx, fy)
    if s.get('pattern_image_raise', 'dark') == 'dark':
        v = 1.0 - v
    return np.where(inside, v, 0.0)


# ------------------------------------------------------------------ where: height range, paint
def height_factor(s, z):
    """0..1 depth scale at model height z: height range with fade, times the vertical wave."""
    z = np.asarray(z, dtype=float)
    lo = s.get('pattern_z_min', 0.0)
    hi = s.get('pattern_z_max', 0.0)
    fade = max(s.get('pattern_fade', 0.0), 0.0)
    if fade > 0:
        k = np.clip((z - lo) / fade, 0.0, 1.0)
        if hi > 0:
            k = k * np.clip((hi - z) / fade, 0.0, 1.0)
    else:
        k = np.where(z >= lo - 1e-9, 1.0, 0.0)
        if hi > 0:
            k = k * np.where(z <= hi + 1e-9, 1.0, 0.0)
    period = s.get('pattern_v_period', 0.0)
    if period > 0:
        k = k * (0.5 - 0.5 * np.cos(2.0 * math.pi * z / period))
    return k


def parse_points(text):
    """'x,y,z; x,y,z; ...' -> (n, 3) array or None."""
    pts = []
    for part in str(text or '').replace('\n', ';').split(';'):
        v = [float(t) for t in part.replace(' ', ',').split(',') if t.strip()]
        if len(v) >= 3:
            pts.append(v[:3])
    return np.array(pts, dtype=float) if pts else None


def paint_mask(s, P, x, y, z, faces=None):
    """1 where the pattern goes, 0 where it does not, soft at the brush edge. faces: 0..1 per point, on a surface
    picked with the fill tool (surface_masks)."""
    mode = s.get('pattern_paint_mode', 'everywhere')
    if mode == 'everywhere':
        return np.ones(len(z))
    r = max(s.get('pattern_brush', 15.0), 0.1)
    m = np.zeros(len(z)) if faces is None else np.asarray(faces, dtype=float)
    if P is not None and len(P):
        near = P[np.abs(P[:, 2] - z.mean()) <= r + (z.max() - z.min()) / 2 + 1e-6]
        if len(near):
            Q = np.column_stack([x, y, z])
            d = np.full(len(Q), np.inf)
            for k in range(0, len(near), 512):
                C = near[k:k + 512]
                d = np.minimum(d, np.sqrt(((Q[:, None, :] - C[None]) ** 2).sum(2)).min(1))
            m = np.maximum(m, np.clip((r - d) / (0.3 * r), 0.0, 1.0))
    return m if mode == 'painted' else 1.0 - m


def slope_mask(s, lean):
    """1 where the wall leans at most Skip slopes over from vertical, 0 beyond, faded over 5 degrees."""
    limit = s.get('pattern_slope_limit', 0.0)
    if limit <= 0 or lean is None:
        return 1.0
    return np.clip((limit + 2.5 - lean) / 5.0, 0.0, 1.0)


# ------------------------------------------------------------------ surfaces picked with the fill tool
def parse_faces(text):
    """'id:a-b,c,...; id:...' -> {part id: sorted array of triangle indices of that part}."""
    out = {}
    for part in str(text or '').split(';'):
        if ':' not in part:
            continue
        pid, _, rng = part.partition(':')
        try:
            pid = int(pid)
        except ValueError:
            continue
        idx = []
        for r in rng.split(','):
            a, _, b = r.strip().partition('-')
            if a.strip().isdigit():
                lo = int(a)
                hi = int(b) if b.strip().isdigit() else lo
                idx.append(np.arange(lo, max(hi, lo) + 1))
        if idx:
            out[pid] = np.unique(np.concatenate(idx))
    return out


def format_faces(faces):
    """Inverse of parse_faces: runs of following indices are written a-b."""
    parts = []
    for pid in sorted(faces):
        idx = np.unique(np.asarray(faces[pid], dtype=np.int64))
        if not len(idx):
            continue
        cut = np.nonzero(np.diff(idx) != 1)[0]
        lo, hi = idx[np.r_[0, cut + 1]], idx[np.r_[cut, len(idx) - 1]]
        parts.append('%d:%s' % (pid, ','.join(str(a) if a == b else '%d-%d' % (a, b) for a, b in zip(lo, hi))))
    return '; '.join(parts)


def remap_faces(text, ids):
    """The picked surfaces with part ids renamed {old: new} (a project opened again gets new part ids)."""
    f = parse_faces(text)
    return format_faces({ids[k]: v for k, v in f.items() if k in ids})


def remap_all_faces(s, ids):
    """Settings with the picked surfaces of every pattern layer renamed to the new part ids {old: new}."""
    s = dict(s)
    if s.get('pattern_faces'):
        s['pattern_faces'] = remap_faces(s['pattern_faces'], ids)
    try:
        raw = json.loads(s.get('pattern_layers') or '[]')
    except (TypeError, ValueError):
        return s
    if isinstance(raw, list) and any(isinstance(d, dict) and d.get('pattern_faces') for d in raw):
        for d in raw:
            if isinstance(d, dict) and d.get('pattern_faces'):
                d['pattern_faces'] = remap_faces(d['pattern_faces'], ids)
        s['pattern_layers'] = json.dumps(raw)
    return s


def face_flags(text, parts):
    """Picked surfaces -> bool per triangle of all parts together. parts: [(id, triangle count)] in plate order.
    None when nothing is picked."""
    f = parse_faces(text)
    if not f:
        return None
    flags = np.zeros(sum(n for _, n in parts), dtype=bool)
    start = 0
    for pid, n in parts:
        idx = f.get(pid)
        if idx is not None:
            flags[start + idx[idx < n]] = True
        start += n
    return flags if flags.any() else None


def _adjacency(tris):
    """Triangle neighbours across shared edges (vertices welded to 1e-4 mm): CSR arrays (start, neighbour)."""
    v = np.round(np.asarray(tris, dtype=float).reshape(-1, 3) / 1e-4).astype(np.int64)
    _, vid = np.unique(v, axis=0, return_inverse=True)
    vid = vid.reshape(-1, 3)
    e = np.concatenate([vid[:, [0, 1]], vid[:, [1, 2]], vid[:, [2, 0]]])
    f = np.tile(np.arange(len(vid)), 3)
    e.sort(1)
    key = e[:, 0] * (int(vid.max()) + 1) + e[:, 1]
    o = np.argsort(key, kind='stable')
    key, f = key[o], f[o]
    a, b = [], []
    for k in range(1, 4):                                 # an edge can be shared by up to 4 triangles (non-manifold)
        same = np.nonzero(key[k:] == key[:-k])[0]
        a.append(f[same])
        b.append(f[same + k])
    a, b = np.concatenate(a + b), np.concatenate(b + a)
    o = np.argsort(a, kind='stable')
    a, b = a[o], b[o]
    start = np.searchsorted(a, np.arange(len(vid) + 1))
    return start, b


def normals(tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    return n / np.maximum(np.linalg.norm(n, axis=1), 1e-18)[:, None]


_ADJ = {}


def fill_region(tris, seed, angle, cache_key=None):
    """Triangles reached from `seed` by crossing edges where the surface bends at most `angle` degrees (flood fill
    by angle). tris: (n, 3, 3) of one part. Returns the sorted triangle indices."""
    tris = np.asarray(tris, dtype=float)
    adj = _ADJ.get(cache_key) if cache_key is not None else None
    if adj is None or len(adj[0]) != len(tris) + 1:
        adj = _adjacency(tris)
        if cache_key is not None:
            _ADJ.clear()
            _ADJ[cache_key] = adj
    start, nb = adj
    n = normals(tris)
    cos = math.cos(math.radians(min(max(angle, 0.0), 180.0))) - 1e-9
    seen = np.zeros(len(tris), dtype=bool)
    seen[seed] = True
    front = np.array([seed])
    while len(front):
        cnt = start[front + 1] - start[front]
        src = np.repeat(front, cnt)
        to = nb[np.repeat(start[front], cnt) + np.arange(int(cnt.sum())) - np.repeat(np.cumsum(cnt) - cnt, cnt)]
        ok = ~seen[to] & ((n[src] * n[to]).sum(1) >= cos)
        front = np.unique(to[ok])
        seen[front] = True
    return np.nonzero(seen)[0]


def surface(tris, parts=None):
    """What the masks need from the part's surface: the triangles, their height range and how far each leans from
    vertical (deg). parts: [(part id, triangle count)] in the order of tris, to find the triangles picked with the
    fill tool (each layer has its own)."""
    tris = np.asarray(tris, dtype=float)
    nz = normals(tris)[:, 2]
    return {'tris': tris, 'zlo': tris[:, :, 2].min(1), 'zhi': tris[:, :, 2].max(1),
            'lean': np.degrees(np.arcsin(np.clip(np.abs(nz), 0.0, 1.0))),
            'parts': list(parts) if parts is not None else [(1, len(tris))]}


def surface_hits(surf, xy, z, reach):
    """For each wall point (xy, model height z): the index of the nearest triangle cut at that height, -1 when no
    surface is within reach."""
    from .overhang import nearest_segment
    m = len(xy)
    out = np.full(m, -1, dtype=np.int64)
    if surf is None or not m:
        return out
    z0 = float(np.median(z))
    sel = np.nonzero((surf['zlo'] <= z0) & (surf['zhi'] >= z0) & (surf['zhi'] > surf['zlo']))[0]
    if not len(sel):
        return out
    T = surf['tris'][sel]
    dz = T[:, :, 2] - z0
    dz = np.where(dz == 0.0, 1e-9, dz)
    a, b = [0, 1, 2], [1, 2, 0]
    cross = (dz[:, a] > 0) != (dz[:, b] > 0)                     # (k, 3) edges cut by the plane
    keep = cross.sum(1) == 2
    T, dz, cross, sel = T[keep], dz[keep], cross[keep], sel[keep]
    if not len(sel):
        return out
    t = dz[:, a] / np.where(cross, dz[:, a] - dz[:, b], 1.0)
    P = T[:, a, :2] + t[:, :, None] * (T[:, b, :2] - T[:, a, :2])  # (k, 3, 2) cut point on each edge
    e = np.argsort(~cross, axis=1, kind='stable')[:, :2]
    r = np.arange(len(sel))
    _, w = nearest_segment(np.asarray(xy, dtype=float), P[r, e[:, 0]], P[r, e[:, 1]], reach)
    hit = w >= 0
    out[hit] = sel[w[hit]]
    return out


def soften(u, m, width, L, closed):
    """A 0/1 mask along the path (positions u, length L) turned into a ramp `width` mm wide, centred on each edge."""
    if width <= 0 or len(u) < 2:
        return m
    on = m > 0.5
    ch = np.nonzero(on[1:] != on[:-1])[0]
    if not len(ch):
        return m
    edges = (u[ch] + u[ch + 1]) / 2
    d = np.abs(u[:, None] - edges[None, :])
    if closed:
        d = np.minimum(d, L - d)
    d = d.min(1)
    return np.where(on, np.clip(0.5 + d / width, 0.0, 1.0), np.clip(0.5 - d / width, 0.0, 1.0))


# ------------------------------------------------------------------ applying it to a path
def _outlines(layer):
    """The outer wall loops of a layer (outer boundaries and holes) and its spiral vase turn."""
    out = [layer.spiral] if layer.spiral is not None else []
    out += [p for st in layer.stages for stage in st for p in stage if p.kind == 1 and p.closed]   # KIND_OUTER
    return out


def stacks(s, layers):
    """Follow each outline up the part: a loop joins the stack of the layer below whose centre is within half its
    radius and whose perimeter is within 30 %. Each stack gets one wave count N. Returns ({id(path): {n, center}},
    report: per stack N, its effective wavelength range and number of layers, largest first)."""
    lam = max(float(s['pattern_wavelength']), 1e-6)
    found = []
    for li, layer in enumerate(layers):
        for p in _outlines(layer):
            xy = p.pts[:, :2]
            ring = np.vstack([xy, xy[:1]]) if p.closed else xy
            L = float(np.hypot(*np.diff(ring, axis=0).T).sum())
            if L < 1e-6:
                continue
            c, r = xy.mean(0), L / (2 * math.pi)
            best, score = None, None
            for st in found:
                if st['taken'] == li or st['last'] < li - 2:
                    continue
                ratio = L / st['L']
                d = float(np.hypot(*(c - st['c'])))
                if not 0.7 < ratio < 1 / 0.7 or d > 0.5 * r:
                    continue
                sc = d / r + abs(math.log(ratio))
                if score is None or sc < score:
                    best, score = st, sc
            if best is None:
                best = {'Ls': [], 'cs': [], 'paths': []}
                found.append(best)
            best.update(c=c, L=L, last=li, taken=li)
            best['Ls'].append(L); best['cs'].append(c); best['paths'].append(id(p))
    if not found:
        return {}, []
    mean = [float(np.mean(st['Ls'])) for st in found]
    main = int(np.argmax([m * len(st['Ls']) for m, st in zip(mean, found)]))
    waves = int(s.get('pattern_waves', 0) or 0)
    loops, report = {}, []
    for st, m in zip(found, mean):
        n = max(1, int(round(waves * m / mean[main]))) if waves > 0 else max(1, int(round(m / lam)))
        center = tuple(np.mean(st['cs'], axis=0))
        for pid in st['paths']:
            loops[pid] = {'n': n, 'center': center}
        report.append({'n': n, 'wavelength_min': min(st['Ls']) / n, 'wavelength_max': max(st['Ls']) / n,
                       'layers': len(st['Ls']), 'perimeter': m})
    report.sort(key=lambda r: -r['perimeter'] * r['layers'])
    return loops, report


def lap_start(pts, cum, center):
    """Arc length where the path crosses the +X direction from the centre (where the lap fraction is 0);
    0 when it does not cross it."""
    a = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    a0, a1 = a[:-1], a[1:]
    right = (pts[:-1, 0] > center[0]) | (pts[1:, 0] > center[0])
    up = right & (a0 <= 0) & (a1 > 0) & (a1 - a0 < math.pi)       # counter-clockwise crossing
    down = right & (a0 > 0) & (a1 <= 0) & (a0 - a1 < math.pi)     # clockwise
    hit = np.nonzero(up | down)[0]
    if not len(hit):
        return 0.0
    i = int(hit[0])
    t = -a0[i] / (a1[i] - a0[i]) if a1[i] != a0[i] else 0.0
    return float(cum[i] + t * (cum[i + 1] - cum[i]))


def needs_surface(s, has_faces=True):
    """Whether the masks look at the part's surface: a slope limit, or picked surfaces that count, in any layer."""
    for _, ls in shown_layers(s):
        if ls.get('pattern_slope_limit', 0.0) > 0:
            return True
        if ls.get('pattern_paint_mode', 'everywhere') != 'everywhere' and parse_faces(ls.get('pattern_faces', '')):
            return True
    return False


def _layer_ctx(ls, layers, surf):
    c = {'s': ls, 'loops': {}, 'report': [], 'image': None, 'paint': None, 'picked': None}
    if ls.get('pattern_fit', True) and ls.get('pattern') != 'image':
        c['loops'], c['report'] = stacks(ls, layers)
    if ls.get('pattern') == 'image':
        c['image'] = load_image(ls.get('pattern_image', ''))
    if ls.get('pattern_paint_mode', 'everywhere') != 'everywhere':
        c['paint'] = parse_points(ls.get('pattern_paint', ''))
        if surf is not None:
            c['picked'] = face_flags(ls.get('pattern_faces', ''), surf['parts'])
    return c


def context(s, layers, surf=None):
    """What the pattern needs to know about the whole part: centre (picture wrap), top height, the part's surface
    (slope mask, picked surfaces) and, per shown layer, its picture, paint and the wave count of every outline
    (stacks). report: layer 1's wave counts; reports: every layer's."""
    ctx = {'center': (0.0, 0.0), 'ztop': 0.0, 'surface': surf, 'layers': []}
    lo, hi, top = np.array([np.inf, np.inf]), np.array([-np.inf, -np.inf]), 0.0
    for l in layers:
        top = max(top, l.z - l.shift)
        paths = ([l.spiral] if l.spiral is not None else []) + [p for st in l.stages for stage in st for p in stage]
        for p in paths:
            if p.kind == 1:   # KIND_OUTER
                lo = np.minimum(lo, p.pts[:, :2].min(0))
                hi = np.maximum(hi, p.pts[:, :2].max(0))
    if np.isfinite(lo).all():
        ctx['center'] = tuple((lo + hi) / 2)
    ctx['ztop'] = top
    for n, ls in shown_layers(s):
        c = _layer_ctx(ls, layers, surf)
        c['n'] = n
        ctx['layers'].append(c)
    ctx['reports'] = {c['n']: c['report'] for c in ctx['layers'] if c['report']}
    ctx['report'] = ctx['reports'].get(1, [])
    return ctx


def _xy_cum(pts):
    return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts[:, :2], axis=0).T))])


def _turn(pts, closed):
    """Turning angle (rad) at each vertex."""
    d = np.diff(pts[:, :2], axis=0)
    a = np.arctan2(d[:, 1], d[:, 0])
    t = np.zeros(len(pts))
    t[1:-1] = np.abs((a[1:] - a[:-1] + math.pi) % (2 * math.pi) - math.pi)
    if closed and len(a) > 1:
        t[0] = t[-1] = abs((a[0] - a[-1] + math.pi) % (2 * math.pi) - math.pi)
    return t


def wave_positions(L, a, b, n_per_wave, corners):
    """Arc lengths (0 < u < L) of every 1/n of a wave and the profile corners, tau(u) = a * u + b."""
    t0, t1 = b, a * L + b
    fr = np.unique(np.concatenate([np.arange(n_per_wave) / n_per_wave, np.mod(corners, 1.0)]))
    waves = np.arange(math.floor(min(t0, t1)) - 1, math.ceil(max(t0, t1)) + 1)
    taus = (waves[:, None] + fr[None, :]).ravel()
    u = np.sort((taus - b) / a)
    return u[(u > 1e-6) & (u < L - 1e-6)]


def merge_positions(L, fixed, u, gap):
    """The fixed arc lengths (ends, kept vertices) plus the wave points u that are at least gap[i] mm from every
    fixed one. The ends and kept vertices must stay (the turns of a spiral join there), so wave points closer than
    60 % of a spacing to one of them are left out: no very short moves, which would slow the robot down at every
    seam while the extruder keeps pushing."""
    fixed = np.unique(np.concatenate([[0.0, L], fixed]))
    if len(u):
        j = np.clip(np.searchsorted(fixed, u), 1, len(fixed) - 1)
        near = np.minimum(np.abs(u - fixed[j - 1]), np.abs(fixed[j] - u))
        u = u[near >= gap]
    u = np.unique(np.concatenate([fixed, u]))
    if len(u) > 2:                                     # kept vertices right next to each other
        d = np.diff(u)
        u = np.concatenate([[u[0]], u[1:][d > 1e-4]])
        u[-1] = L
    return u


def sample_positions(cum, a, b, n_per_wave, corners, keep):
    """Arc lengths where points go for one pattern over the whole path: the wave points, both ends and the kept
    vertices (see merge_positions)."""
    L = cum[-1]
    return merge_positions(L, cum[keep], wave_positions(L, a, b, n_per_wave, corners), 0.6 / (abs(a) * n_per_wave))


def _phase(ls, lc, pts, cum, closed, layer_index, spiral, u0, key, state):
    """(a, b, period) of one layer on this path: tau(u) = a * u + b counts waves along it."""
    kind = ls['pattern']
    L = cum[-1]
    lam = ls['pattern_wavelength']
    shift = ls.get('pattern_shift', 0.0) / 100.0
    layer_shift = shift if kind in WAVES else 0.0       # bumps shift per row; noise and pictures do not shift
    stack = (lc.get('loops') or {}).get(key) if key is not None else None
    if ls.get('pattern_fit', True) and stack:
        # one wave count for the whole stack; the lap fraction counts from the +X direction of its centre
        n = stack['n']
        u_ref = lap_start(pts, cum, stack['center'])
        if spiral:   # tau = (N + shift) * laps: the lap count goes on from turn to turn
            # where this turn starts, in laps, from the geometry (its +X crossing); the lap count carried from the
            # turn before only picks the whole lap, so the turns join and the waves stay over each other
            lap0 = layer_index - u_ref / L
            if 'lap' in state:
                lap0 += round(state['lap'] - lap0)
            state['lap'] = lap0 + 1.0
            k = n + layer_shift
            return k / L, k * lap0, n
        return n / L, layer_shift * layer_index - n * u_ref / L, n
    if ls.get('pattern_fit', True) and (closed or spiral):   # no stack known (a single path): whole waves on it
        n = max(int(round(L / lam)), 1)
        if spiral:
            return (n + layer_shift) / L, layer_shift * layer_index, n
        return n / L, layer_shift * layer_index, n
    b = (u0 / lam + layer_shift * layer_index) if spiral else layer_shift * layer_index
    return 1.0 / lam, b, None


def _relief(ls, lc, tau, xy, z, period, ctx):
    """pattern(point) 0..1 of one layer."""
    kind = ls['pattern']
    lam = ls['pattern_wavelength']
    width = ls.get('pattern_spike_width', 50.0) / 100.0
    row_h = max(ls.get('pattern_row_height', 20.0), 0.1)
    if kind == 'bumps':
        return bumps(tau, z, lam, row_h, width, ls.get('pattern_shift', 0.0) / 100.0)
    if kind == 'noise':
        return noise(tau, z, row_h, period, int(ls.get('pattern_seed', 1)))
    if kind == 'image':
        return picture(xy[:, 0], xy[:, 1], z, ls, {'center': ctx['center'], 'ztop': ctx['ztop'], 'image': lc['image']})
    return profile(kind, tau, width)


def _mask(ls, lc, u, xy, z, tri, surf, L, closed):
    """k 0..1 of one layer along the path: height range, paint (brush and picked surfaces, the picked edge
    softened over Edge blend), slope."""
    picked = None
    if lc.get('picked') is not None and tri is not None:
        picked = np.where(tri >= 0, lc['picked'][np.maximum(tri, 0)], False).astype(float)
        picked = soften(u, picked, ls.get('pattern_edge_blend', 0.0), L, closed)
    lean = np.where(tri >= 0, surf['lean'][np.maximum(tri, 0)], 0.0) if tri is not None else None
    return height_factor(ls, z) * paint_mask(ls, lc.get('paint'), xy[:, 0], xy[:, 1], z, picked) * slope_mask(ls, lean)


def _interp(u, cum, pts):
    return np.column_stack([np.interp(u, cum, pts[:, k]) for k in range(pts.shape[1])])


def apply(pts, s, closed, layer_index, air_sign=1.0, model_z=None, spiral=False, u0=0.0, z_shift=0.0, ctx=None, key=None,
          spiral_state=None):
    """Pattern one path. pts: (n, 2|3), closed loops with the first point repeated at the end (start = seam).
    air_sign: +1 when the air is on the right of the direction of travel (outer boundary of a CCW loop), -1 for a
    hole. model_z: model height of a flat path. (n, 3) paths carry their own Z: model height = Z - z_shift (a layer
    lifted onto a raft). spiral: an open spiral turn that continues into the next one (phase grows along it).
    u0: distance already printed along the spiral before this turn (used when the wavelength is not fitted).
    ctx: context() of the whole part (picture centre, paint, wave counts); None = worked out from this path.
    key: id() of the slicer's Path, to find its stack (wave count) in ctx. spiral_state: a dict kept from turn to
    turn of a spiral vase: the lap count goes on from turn to turn, so the turns join without a step.
    Returns the new (m, 2|3) path."""
    pts = np.asarray(pts, dtype=float)
    if not active(s) or len(pts) < 3:
        return pts
    if ctx is None:
        ctx = {'center': tuple(pts[:, :2].mean(0)), 'ztop': model_z or 0.0, 'surface': None,
               'layers': [dict(_layer_ctx(ls, [], None), n=n) for n, ls in shown_layers(s)]}
    lcs = ctx['layers']
    if not lcs:
        return pts
    cum = _xy_cum(pts)
    L = cum[-1]
    if L < 1e-6:
        return pts
    surf = ctx.get('surface')
    state = spiral_state if spiral_state is not None else {}
    keep = np.nonzero(_turn(pts, closed) >= math.radians(20))[0]
    zof = (lambda q: q[:, 2] - z_shift) if pts.shape[1] == 3 else (lambda q: np.full(len(q), 0.0 if model_z is None else model_z))
    reach = 5.0 * s.get('line_width', 10.0) + 2.0

    # each layer's phase and wave points
    lay = []
    for lc in lcs:
        ls = lc['s']
        a, b, period = _phase(ls, lc, pts, cum, closed, layer_index, spiral, u0, key, state.setdefault(lc['n'], {}))
        npw = max(int(ls.get('pattern_points', 12)), 2)
        corners = corner_fractions(ls['pattern'], ls.get('pattern_spike_width', 50.0) / 100.0) if ls['pattern'] in WAVES else []
        lay.append({'lc': lc, 'a': a, 'b': b, 'period': period, 'npw': npw, 'corners': corners,
                    'sp': 1.0 / (abs(a) * npw), 'blend': ls.get('pattern_blend', 'cover')})

    # where each layer shows: its mask k on a fine grid along the path, then what is left of it under the cover
    # layers above (e). A single layer with no area, height range or slope limit shows everywhere.
    masked = surf is not None or any(
        c['lc']['s'].get('pattern_paint_mode', 'everywhere') != 'everywhere' or c['lc']['s'].get('pattern_z_min', 0) > 0
        or c['lc']['s'].get('pattern_z_max', 0) > 0 or c['lc']['s'].get('pattern_fade', 0) > 0 for c in lay)
    if len(lay) == 1 and not masked:
        c = lay[0]
        u = sample_positions(cum, c['a'], c['b'], c['npw'], c['corners'], keep)
        grid = None
    else:
        h = max(min(c['sp'] for c in lay) / 2.0, 0.2)
        ug = np.linspace(0.0, L, int(min(max(math.ceil(L / h), 2), 40000)) + 1)
        qg = _interp(ug, cum, pts)
        zg = zof(qg)
        trig = surface_hits(surf, qg[:, :2], zg, reach) if surf is not None else None
        ks = [_mask(c['lc']['s'], c['lc'], ug, qg[:, :2], zg, trig, surf, L, closed) for c in lay]
        es = []
        for i, k in enumerate(ks):
            e = k.copy()
            for j in range(i + 1, len(ks)):
                if lay[j]['blend'] != 'add':
                    e = e * (1.0 - ks[j])
            es.append(e)
        grid = (ug, ks)
        # wave points of each layer within one of its spacings of where it shows
        wave, gaps, owner = [], [], []
        for i, (c, e) in enumerate(zip(lay, es)):
            w = wave_positions(L, c['a'], c['b'], c['npw'], c['corners'])
            on = np.concatenate([[0], np.cumsum(e > 1e-6)])
            lo = np.searchsorted(ug, w - c['sp'], 'left')
            hi = np.searchsorted(ug, w + c['sp'], 'right')
            w = w[on[hi] - on[lo] > 0]
            wave.append(w)
            gaps.append(np.full(len(w), 0.6 * c['sp']))
            owner.append(np.full(len(w), i))
        wave, gaps, owner = np.concatenate(wave), np.concatenate(gaps), np.concatenate(owner)
        # the path's own points where no layer shows (plain wall keeps its shape)
        anyon = np.concatenate([[0], np.cumsum(np.max(es, axis=0) > 1e-6)])
        lo = np.searchsorted(ug, cum - h, 'left')
        hi = np.searchsorted(ug, cum + h, 'right')
        plain = np.nonzero(anyon[hi] - anyon[lo] == 0)[0]
        fixed = cum[np.unique(np.concatenate([keep, plain]))]
        if len(lay) > 1 and len(wave):
            # points of two layers right next to each other: keep the later layer's (it is on top)
            o = np.lexsort((-owner, wave))
            wave, gaps, owner = wave[o], gaps[o], owner[o]
            ok = np.ones(len(wave), dtype=bool)
            last = 0
            for i in range(1, len(wave)):
                if owner[i] != owner[last] and wave[i] - wave[last] < 0.5 * min(gaps[i], gaps[last]) / 0.6:
                    if owner[i] > owner[last]:
                        ok[last] = False
                        last = i
                    else:
                        ok[i] = False
                else:
                    last = i
            wave, gaps = wave[ok], gaps[ok]
        if len(wave):
            o = np.argsort(wave)
            wave, gaps = wave[o], gaps[o]
            fx = np.unique(np.concatenate([[0.0, L], fixed]))
            j = np.clip(np.searchsorted(fx, wave), 1, len(fx) - 1)
            near = np.minimum(np.abs(wave - fx[j - 1]), np.abs(fx[j] - wave))
            wave = wave[near >= gaps]
        u = merge_positions(L, fixed, wave, 0.0)
    q = _interp(u, cum, pts)

    # unit normals: central differences of the resampled path (wrapping round a closed loop)
    xy = q[:, :2]
    if closed:
        prev = np.vstack([xy[-2:-1], xy[:-1]])
        nxt = np.vstack([xy[1:], xy[1:2]])
    else:
        prev = np.vstack([xy[:1], xy[:-1]])
        nxt = np.vstack([xy[1:], xy[-1:]])
    t = nxt - prev
    t /= np.maximum(np.hypot(*t.T), 1e-12)[:, None]
    nrm = np.column_stack([t[:, 1], -t[:, 0]]) * air_sign      # right of travel, towards the air

    z = zof(q)
    tri = surface_hits(surf, xy, z, reach) if surf is not None and grid is None else None
    off = np.zeros(len(q))
    for i, c in enumerate(lay):
        ls, lc = c['lc']['s'], c['lc']
        g = _relief(ls, lc, c['a'] * u + c['b'], xy, z, c['period'], ctx)
        if grid is None:
            k = _mask(ls, lc, u, xy, z, tri, surf, L, closed)
        else:
            k = np.interp(u, grid[0], grid[1][i])
        d = ls['pattern_amplitude'] * k * (g - BASES.get(ls.get('pattern_direction', 'outward'), 0.0))
        off = off + d if (c['blend'] == 'add' and i > 0) else off * (1.0 - k) + d
    q[:, :2] = xy + nrm * off[:, None]
    if closed:
        q[-1] = q[0]
    return q


def is_hole(loop, others):
    """Is the closed loop a hole, i.e. inside an odd number of the other loops of the layer?"""
    p = loop[0, :2]
    inside = 0
    for o in others:
        o = o[:, :2]
        x, y = o[:, 0], o[:, 1]
        x2, y2 = np.roll(x, -1), np.roll(y, -1)
        cross = (y > p[1]) != (y2 > p[1])
        with np.errstate(divide='ignore', invalid='ignore'):
            xi = x + (p[1] - y) * (x2 - x) / (y2 - y)
        inside += int(np.count_nonzero(cross & (xi > p[0])) % 2)
    return inside % 2 == 1


def _warnings_one(ls, tag):
    out = []
    w = ls['line_width']
    A = ls['pattern_amplitude']
    kind = ls['pattern']
    if kind in WAVES and ls.get('pattern_shift', 0.0) and A > 0.5 * w:
        out.append(f'{tag}: with a layer shift, an amplitude over half the line width ({0.5 * w:g} mm) '
                   'leaves beads hanging in the air between layers. Lower the amplitude or test the overhang.')
    if kind != 'image' and ls['pattern_wavelength'] < 2 * w:
        out.append(f'{tag}: a wavelength under twice the line width ({2 * w:g} mm) blurs into a thick bead.')
    if not ls.get('spiral_vase') and ls.get('walls', 1) > 1 and ls.get('pattern_direction', 'outward') != 'outward':
        out.append(f'{tag}: inward / centered waves run into the inner wall. Use outward, one wall or spiral vase.')
    if kind == 'image' and load_image(ls.get('pattern_image', '')) is None:
        out.append(f'{tag}: no picture chosen (or the file is gone). Upload one in the Surface pattern card.')
    if (ls.get('pattern_paint_mode', 'everywhere') == 'painted' and parse_points(ls.get('pattern_paint', '')) is None
            and not parse_faces(ls.get('pattern_faces', ''))):
        what = 'this layer does not show' if 'layer' in tag else 'the wall stays plain'
        out.append(f'{tag}: "only painted areas" is chosen but nothing is painted, so {what}.')
    return out


def warnings(s):
    layers = shown_layers(s)
    if not layers:
        return []
    many = len(all_layers(s)) > 1
    out = []
    for n, ls in layers:
        out += _warnings_one(ls, f'Surface pattern layer {n}' if many else 'Surface pattern')
    w = s['line_width']
    add = sum(ls['pattern_amplitude'] for i, (n, ls) in enumerate(layers) if i == 0 or ls.get('pattern_blend') == 'add')
    if len(layers) > 1 and add > w:
        out.append(f'Surface pattern: the layers set to Add (with the bottom one) reach {add:g} mm deep together, more '
                   f'than the line width ({w:g} mm). Where they overlap the bead can tear off the wall.')
    for i, (n, ls) in enumerate(layers):
        if i and ls.get('pattern_blend', 'cover') != 'add' and ls.get('pattern_paint_mode', 'everywhere') == 'everywhere' \
                and ls.get('pattern_z_min', 0) <= 0 and ls.get('pattern_z_max', 0) <= 0 and ls.get('pattern_slope_limit', 0) <= 0:
            out.append(f'Surface pattern: layer {n} covers the whole wall, so the layers below it do not show. Give it '
                       'an area (Paint or Fill) or set it to Add.')
    return out
