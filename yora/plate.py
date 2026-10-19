"""The parts on the bed. Each part keeps its mesh as loaded and its own scale, rotation and position; slicing uses all
of them together as one triangle soup (the slicer unites overlapping solids)."""
import copy

import numpy as np

from . import mesh

GAP = 10.0   # mm kept free between a new part and the parts already on the bed


class Plate:
    def __init__(self):
        self.parts = []          # dicts: id, name, raw (triangles as loaded), transform, placed (on the bed)
        self.next_id = 1
        self.removed = {}        # id -> removed part, so undo can put it back (restore)

    def __bool__(self):
        return bool(self.parts)

    def find(self, pid):
        for p in self.parts:
            if p['id'] == pid:
                return p
        raise KeyError('No part %s on the bed' % pid)

    @staticmethod
    def _place(p, s):
        t = p['transform']
        center = t['center'] or [s['bed_x'] / 2, s['bed_y'] / 2]
        p['placed'] = mesh.place(p['raw'], t['scale'], t['rotation'], center)

    def add(self, name, raw, s, transform=None):
        """Add a part. Without a position (center None) it is set down in a free spot on the bed: the middle when the
        bed is empty or the middle is free."""
        t = copy.deepcopy(transform) if transform else {'scale': [100, 100, 100], 'rotation': [0, 0, 0], 'center': None}
        p = {'id': self.next_id, 'name': name, 'raw': raw, 'transform': t}
        self.next_id += 1
        self._place(p, s)
        if t.get('center') is None and self.parts:
            lo, hi = mesh.bounds(p['placed'])
            t['center'] = free_spot(hi[:2] - lo[:2], [mesh.bounds(q['placed']) for q in self.parts], s)
            self._place(p, s)
        self.parts.append(p)
        return p

    def update(self, pid, transform, s):
        p = self.find(pid)
        p['transform'] = transform
        self._place(p, s)
        return p

    def remove(self, pid):
        p = self.find(pid)
        self.parts.remove(p)
        self.removed[pid] = p

    def duplicate(self, pid, s):
        """A copy of the part (same size and rotation) in a free spot."""
        p = self.find(pid)
        t = copy.deepcopy(p['transform'])
        t['center'] = None
        return self.add(p['name'], p['raw'], s, t)

    def clear(self, forget=False):
        """Take every part off the bed (kept for undo unless forget: a new project or G-code)."""
        if not forget:
            self.removed.update({p['id']: p for p in self.parts})
        else:
            self.removed = {}
        self.parts = []

    def snapshot(self):
        """The arrangement: which parts are on the bed, in order, and where (for undo)."""
        return [{'id': p['id'], 'transform': copy.deepcopy(p['transform'])} for p in self.parts]

    def restore(self, snap, s):
        """Back to an arrangement from snapshot(): parts not in it are taken off (kept), removed ones come back."""
        pool = dict(self.removed)
        pool.update({p['id']: p for p in self.parts})
        parts = []
        for e in snap:
            p = pool.pop(int(e['id']), None)
            if p is None:
                continue                       # forgotten (another project was opened since)
            t = e['transform']
            p['transform'] = {'scale': [float(v) for v in t['scale']], 'rotation': [float(v) for v in t['rotation']],
                              'center': [float(v) for v in t['center']] if t.get('center') else None}
            self._place(p, s)
            parts.append(p)
        self.removed.update(pool)
        self.parts = parts

    def place_all(self, s):
        for p in self.parts:
            self._place(p, s)

    def triangles(self):
        """All parts on the bed, one (n, 3, 3) array (None when empty)."""
        if not self.parts:
            return None
        return np.concatenate([p['placed'] for p in self.parts])

    def info(self, s):
        if not self.parts:
            return None
        parts, start = [], 0
        for p in self.parts:
            lo, hi = mesh.bounds(p['placed'])
            n = len(p['placed'])
            parts.append({'id': p['id'], 'name': p['name'], 'triangles': int(n), 'start': start,
                          'min': lo.tolist(), 'max': hi.tolist(), 'size': (hi - lo).tolist(),
                          'fits': fits(lo, hi, s), 'transform': p['transform']})
            start += n
        lo = np.min([q['min'] for q in parts], axis=0)
        hi = np.max([q['max'] for q in parts], axis=0)
        return {'name': parts[0]['name'] if len(parts) == 1 else '%d parts' % len(parts), 'triangles': start,
                'min': lo.tolist(), 'max': hi.tolist(), 'size': (hi - lo).tolist(), 'fits': all(q['fits'] for q in parts),
                'transform': parts[0]['transform'], 'parts': parts}


def fits(lo, hi, s):
    return bool(lo[0] >= 0 and lo[1] >= 0 and hi[0] <= s['bed_x'] and hi[1] <= s['bed_y'] and hi[2] <= s['bed_z'])


def free_spot(size, taken, s):
    """Centre (x, y) for a footprint of `size` that keeps GAP from the boxes `taken` [(lo, hi), ...]: the free spot
    closest to the bed middle, searched on a 5 mm grid. When nothing on the bed is free: just right of the other parts
    (outside the print area, where it is flagged, rather than on top of another part)."""
    bx, by = s['bed_x'], s['bed_y']
    w, h = size[0] / 2 + GAP / 2, size[1] / 2 + GAP / 2
    mid = np.array([bx / 2, by / 2])
    xs = np.arange(w - GAP / 2, bx - w + GAP / 2 + 1e-9, 5.0) if bx >= 2 * w - GAP else np.array([bx / 2])
    ys = np.arange(h - GAP / 2, by - h + GAP / 2 + 1e-9, 5.0) if by >= 2 * h - GAP else np.array([by / 2])
    X, Y = np.meshgrid(xs, ys)
    c = np.column_stack([X.ravel(), Y.ravel()])
    ok = np.ones(len(c), dtype=bool)
    for lo, hi in taken:
        ok &= ~((c[:, 0] - w < hi[0] + GAP / 2) & (c[:, 0] + w > lo[0] - GAP / 2) &
                (c[:, 1] - h < hi[1] + GAP / 2) & (c[:, 1] + h > lo[1] - GAP / 2))
    if not ok.any():
        if not taken:
            return mid.tolist()
        right = max(hi[0] for _, hi in taken) + GAP + size[0] / 2
        return [float(right), float(by / 2)]
    c = c[ok]
    return c[int(np.argmin(np.hypot(*(c - mid).T)))].tolist()
