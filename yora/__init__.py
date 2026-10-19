"""YoRA Slicer: slice models for robot FGF printing and export robot programs (Borunte, KUKA) or G-code."""
import hashlib

import numpy as np

from .settings import DEFAULTS, normalize
from .mesh import load_mesh, place
from .slicer import slice_mesh
from .planner import plan_layers

__version__ = '0.1.0'


# settings that only change the planned moves, not the sliced layers: changing only these reuses the last slice
PLAN_ONLY_PREFIXES = ('pattern', 'overhang')


def slice_key(tris, settings):
    keep = tuple(sorted((k, v) for k, v in settings.items() if not k.startswith(PLAN_ONLY_PREFIXES)))
    return hashlib.sha1(np.ascontiguousarray(tris, dtype=np.float64).tobytes()).hexdigest(), keep


def slice_to_toolpath(tris, settings, progress=None, cache=None):
    """Mesh already placed on the bed -> Toolpath. cache: a dict kept between calls; when only plan-only settings
    (the surface pattern) changed since the last call, the sliced layers are reused and only the moves are planned."""
    from .strategy import is_angled, slice_angled
    from .overhang import annotate
    if is_angled(settings):
        tp = slice_angled(tris, settings, progress)
        if progress:
            progress('planning', 1.0)
        return annotate(tp, settings)
    key = slice_key(tris, settings) if cache is not None else None
    if cache is not None and cache.get('key') == key:
        result = cache['result']
    else:
        result = slice_mesh(tris, settings, progress)
        if cache is not None:
            cache.clear()
            cache.update(key=key, result=result)
    if progress:
        progress('planning', 0.0)
    tp = plan_layers(result, settings)
    if progress:
        progress('planning', 1.0)
    return annotate(tp, settings)
