"""Surface pattern: waves, spikes, bumps, noise or a picture pressed into the outer wall (settings group
'Surface pattern').

The model is sliced as usual; each outer wall loop (and each spiral vase turn) is then resampled along its length and
every point is moved sideways, perpendicular to the wall, towards the air, by

    offset = depth * height factor * paint mask * (pattern(point) - base)

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
areas painted on the part (or keeps it off them). Paths are (n, 2) or (n, 3) arrays; distances are measured in XY.
"""
import math
import os

import numpy as np

from .settings import PROFILE_DIR

PATTERNS = ('none', 'sine', 'zigzag', 'spikes', 'bumps', 'noise', 'image')
WAVES = ('sine', 'zigzag', 'spikes')          # 1D profiles along the wall, shifted per layer
BASES = {'outward': 0.0, 'inward': 1.0, 'centered': 0.5}
PATTERN_DIR = os.path.join(PROFILE_DIR, 'patterns')
IMAGE_EXT = ('.png', '.jpg', '.jpeg', '.bmp')
IMAGE_MAX = 1024                               # pictures are scaled down to this many pixels on the long side


def active(s):
    return s.get('pattern', 'none') in PATTERNS[1:] and s.get('pattern_amplitude', 0) > 0 \
        and s.get('pattern_wavelength', 0) > 0


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


def paint_mask(s, P, x, y, z):
    """1 where the pattern goes, 0 where it does not, soft at the brush edge."""
    mode = s.get('pattern_paint_mode', 'everywhere')
    if mode == 'everywhere':
        return np.ones(len(z))
    r = max(s.get('pattern_brush', 15.0), 0.1)
    m = np.zeros(len(z))
    if P is not None and len(P):
        near = P[np.abs(P[:, 2] - z.mean()) <= r + (z.max() - z.min()) / 2 + 1e-6]
        if len(near):
            Q = np.column_stack([x, y, z])
            d = np.full(len(Q), np.inf)
            for k in range(0, len(near), 512):
                C = near[k:k + 512]
                d = np.minimum(d, np.sqrt(((Q[:, None, :] - C[None]) ** 2).sum(2)).min(1))
            m = np.clip((r - d) / (0.3 * r), 0.0, 1.0)
    return m if mode == 'painted' else 1.0 - m


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


def context(s, layers):
    """What the pattern needs to know about the whole part: centre (picture wrap), top height, picture, paint,
    and the wave count of every outline (stacks)."""
    ctx = {'center': (0.0, 0.0), 'ztop': 0.0, 'image': None, 'paint': None, 'loops': {}, 'report': []}
    if s.get('pattern_fit', True) and s.get('pattern') != 'image':
        ctx['loops'], ctx['report'] = stacks(s, layers)
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
    if s.get('pattern') == 'image':
        ctx['image'] = load_image(s.get('pattern_image', ''))
    if s.get('pattern_paint_mode', 'everywhere') != 'everywhere':
        ctx['paint'] = parse_points(s.get('pattern_paint', ''))
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


def sample_positions(cum, a, b, n_per_wave, corners, keep):
    """Arc lengths where points go: every 1/n of a wave (and the profile corners), plus both ends and the kept
    vertices. tau(u) = a * u + b. The ends and kept vertices must stay (the turns of a spiral join there), so wave
    points closer than 60 % of a spacing to one of them are left out: no very short moves, which would slow the robot
    down at every seam while the extruder keeps pushing."""
    L = cum[-1]
    t0, t1 = b, a * L + b
    fr = np.unique(np.concatenate([np.arange(n_per_wave) / n_per_wave, np.mod(corners, 1.0)]))
    waves = np.arange(math.floor(min(t0, t1)) - 1, math.ceil(max(t0, t1)) + 1)
    taus = (waves[:, None] + fr[None, :]).ravel()
    u = np.sort((taus - b) / a)
    u = u[(u > 1e-6) & (u < L - 1e-6)]
    fixed = np.unique(np.concatenate([[0.0, L], cum[keep]]))
    gap = 0.6 / (abs(a) * n_per_wave)                  # 60 % of the spacing of the wave points, mm
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
    kind = s['pattern']
    if ctx is None:
        ctx = {'center': tuple(pts[:, :2].mean(0)), 'ztop': model_z or 0.0,
               'image': load_image(s.get('pattern_image', '')) if kind == 'image' else None,
               'paint': parse_points(s.get('pattern_paint', ''))}
    lam = s['pattern_wavelength']
    width = s.get('pattern_spike_width', 50.0) / 100.0
    shift = s.get('pattern_shift', 0.0) / 100.0
    layer_shift = shift if kind in WAVES else 0.0       # bumps shift per row; noise and pictures do not shift
    cum = _xy_cum(pts)
    L = cum[-1]
    if L < 1e-6:
        return pts
    period = None
    stack = (ctx.get('loops') or {}).get(key) if key is not None else None
    if s.get('pattern_fit', True) and stack:
        # one wave count for the whole stack; the lap fraction counts from the +X direction of its centre
        n = period = stack['n']
        u_ref = lap_start(pts, cum, stack['center'])
        if spiral:   # tau = (N + shift) * laps: the lap count goes on from turn to turn
            # where this turn starts, in laps, from the geometry (its +X crossing); the lap count carried from the
            # turn before only picks the whole lap, so the turns join and the waves stay over each other
            state = spiral_state if spiral_state is not None else {}
            lap0 = layer_index - u_ref / L
            if 'lap' in state:
                lap0 += round(state['lap'] - lap0)
            state['lap'] = lap0 + 1.0
            k = n + layer_shift
            a, b = k / L, k * lap0
        else:
            a, b = n / L, layer_shift * layer_index - n * u_ref / L
    elif s.get('pattern_fit', True) and (closed or spiral):   # no stack known (a single path): whole waves on it
        n = period = max(int(round(L / lam)), 1)
        if spiral:
            a, b = (n + layer_shift) / L, layer_shift * layer_index
        else:
            a, b = n / L, layer_shift * layer_index
    else:
        a = 1.0 / lam
        b = (u0 / lam + layer_shift * layer_index) if spiral else layer_shift * layer_index
    npw = max(int(s.get('pattern_points', 12)), 2)
    keep = np.nonzero(_turn(pts, closed) >= math.radians(20))[0]
    u = sample_positions(cum, a, b, npw, corner_fractions(kind, width) if kind in WAVES else [], keep)
    q = np.column_stack([np.interp(u, cum, pts[:, k]) for k in range(pts.shape[1])])

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

    z = q[:, 2] - z_shift if q.shape[1] == 3 else np.full(len(q), 0.0 if model_z is None else model_z)
    tau = a * u + b
    row_h = max(s.get('pattern_row_height', 20.0), 0.1)
    if kind == 'bumps':
        g = bumps(tau, z, lam, row_h, width, shift)
    elif kind == 'noise':
        g = noise(tau, z, row_h, period, int(s.get('pattern_seed', 1)))
    elif kind == 'image':
        g = picture(xy[:, 0], xy[:, 1], z, s, ctx)
    else:
        g = profile(kind, tau, width)
    base = BASES.get(s.get('pattern_direction', 'outward'), 0.0)
    k = height_factor(s, z) * paint_mask(s, ctx.get('paint'), xy[:, 0], xy[:, 1], z)
    off = s['pattern_amplitude'] * k * (g - base)
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


def warnings(s):
    if not active(s):
        return []
    out = []
    w = s['line_width']
    A = s['pattern_amplitude']
    kind = s['pattern']
    if kind in WAVES and s.get('pattern_shift', 0.0) and A > 0.5 * w:
        out.append(f'Surface pattern: with a layer shift, an amplitude over half the line width ({0.5 * w:g} mm) '
                   'leaves beads hanging in the air between layers. Lower the amplitude or test the overhang.')
    if kind != 'image' and s['pattern_wavelength'] < 2 * w:
        out.append(f'Surface pattern: a wavelength under twice the line width ({2 * w:g} mm) blurs into a thick bead.')
    if not s.get('spiral_vase') and s.get('walls', 1) > 1 and s.get('pattern_direction', 'outward') != 'outward':
        out.append('Surface pattern: inward / centered waves run into the inner wall. Use outward, one wall or spiral vase.')
    if kind == 'image' and load_image(s.get('pattern_image', '')) is None:
        out.append('Surface pattern: no picture chosen (or the file is gone). Upload one in the Surface pattern card.')
    if s.get('pattern_paint_mode', 'everywhere') == 'painted' and parse_points(s.get('pattern_paint', '')) is None:
        out.append('Surface pattern: "only painted areas" is chosen but nothing is painted, so the wall stays plain.')
    return out
