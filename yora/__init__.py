"""YoRA Slicer: slice models for robot FGF printing and export robot programs (Borunte, KUKA) or G-code."""
from .settings import DEFAULTS, normalize
from .mesh import load_mesh, place
from .slicer import slice_mesh
from .planner import plan_layers

__version__ = '0.1.0'


def slice_to_toolpath(tris, settings, progress=None):
    """Mesh already placed on the bed -> Toolpath."""
    from .strategy import is_angled, slice_angled
    if is_angled(settings):
        tp = slice_angled(tris, settings, progress)
        if progress:
            progress('planning', 1.0)
        return tp
    result = slice_mesh(tris, settings, progress)
    if progress:
        progress('planning', 0.0)
    tp = plan_layers(result, settings)
    if progress:
        progress('planning', 1.0)
    return tp
