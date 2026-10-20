"""Slicer settings: one flat dict, described by SPEC so the GUI can build its form from it."""
import json
import os
import re

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILE_DIR = os.path.join(PROJECT_DIR, 'profiles')

# (group, key, label, type, default, extra)
# type: float | int | bool | choice | text. extra: unit, min, max, options, help
SPEC = [
    # ---- Print
    ('Layers & walls', 'layer_height', 'Layer height', 'float', 2.0, {'unit': 'mm', 'min': 0.1, 'max': 20}),
    ('Layers & walls', 'first_layer_height', 'First layer height', 'float', 2.0, {'unit': 'mm', 'min': 0.1, 'max': 20}),
    ('Layers & walls', 'line_width', 'Line width', 'float', 5.0, {'unit': 'mm', 'min': 0.2, 'max': 50,
                                                                  'help': 'Bead width. Usually 1.1-1.5x the nozzle diameter.'}),
    ('Layers & walls', 'first_layer_width_pct', 'First layer width', 'float', 120, {'unit': '%', 'min': 50, 'max': 300}),
    ('Layers & walls', 'walls', 'Walls', 'int', 2, {'min': 0, 'max': 50}),
    ('Layers & walls', 'wall_order', 'Wall order', 'choice', 'inner_first', {'options': ['inner_first', 'outer_first']}),
    ('Layers & walls', 'top_layers', 'Top solid layers', 'int', 2, {'min': 0, 'max': 50}),
    ('Layers & walls', 'bottom_layers', 'Bottom solid layers', 'int', 2, {'min': 0, 'max': 50}),
    ('Layers & walls', 'spiral_vase', 'Spiral vase', 'bool', False, {'help': 'One continuous outer wall with Z rising along the path. Bottom layers stay solid.'}),
    ('Layers & walls', 'brim_loops', 'Brim loops', 'int', 0, {'min': 0, 'max': 50}),

    ('Raft', 'raft_type', 'Raft', 'choice', 'none', {'options': ['none', 'outline', 'hull', 'rectangle', 'fitted_rectangle', 'circle'],
        'help': 'Layers printed under the part. outline: the shape of the part (holes filled). hull: the closed area '
                'of the part (an arc-shaped part gets a half disc). rectangle: the bounding box. fitted rectangle: '
                'the smallest rectangle, turned to fit the part. circle: the smallest circle around the part.'}),
    ('Raft', 'raft_offset', 'Raft offset', 'float', 5, {'unit': 'mm', 'min': 0, 'max': 500,
        'help': 'How far the raft reaches beyond the part on every side.'}),
    ('Raft', 'raft_layers', 'Raft layers', 'int', 2, {'min': 1, 'max': 20}),
    ('Raft', 'raft_layer_height', 'Raft layer height', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 20,
        'help': '0 = the first layer height.'}),
    ('Raft', 'raft_angle', 'Raft line angle', 'float', 45, {'unit': 'deg', 'min': -360, 'max': 360,
        'help': 'Direction of the raft lines on the first raft layer (0 = along X).'}),
    ('Raft', 'raft_angle_step', 'Angle change per raft layer', 'float', 90, {'unit': 'deg', 'min': -180, 'max': 180,
        'help': 'Added to the line angle on each next raft layer (90 = crossed layers, 0 = all the same).'}),
    ('Raft', 'raft_density', 'Raft density', 'float', 60, {'unit': '%', 'min': 5, 'max': 100,
        'help': 'How much of the raft area the lines cover (100 % = solid). The lines are joined into one continuous '
                'zig-zag per raft layer.'}),
    ('Raft', 'raft_top_solid', 'Solid top raft layer', 'bool', True,
        {'help': 'With 2 or more raft layers, the last one is solid (100 %), so the part starts on a smooth, closed surface. '
                 'A one-layer raft always uses the raft density.'}),
    ('Raft', 'raft_outline', 'Raft outline loop', 'bool', False,
        {'help': 'Print a loop around the raft edge on every raft layer before the lines. Off = lines only.'}),
    ('Raft', 'raft_gap', 'Gap to the part', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 10,
        'help': 'Extra height between the raft top and the part (larger = easier to remove, weaker bond).'}),
    ('Raft', 'raft_speed', 'Raft speed', 'float', 30, {'unit': 'mm/s', 'min': 1, 'max': 1000,
        'help': 'The first raft layer uses the first layer speed.'}),

    ('Seam & layer change', 'seam', 'Seam position', 'choice', 'aligned',
     {'options': ['aligned', 'rear', 'direction', 'custom', 'path', 'nearest', 'random'],
        'help': 'Where each wall loop starts and ends. aligned: on top of the previous layer seam. rear: the back (+Y). '
                'direction: the side at the seam direction angle. custom: closest to a point (Pick point on the part). '
                'path: follows a line drawn on the part (Draw seam line). nearest: closest to the nozzle. random: anywhere.'}),
    ('Seam & layer change', 'seam_angle', 'Seam direction', 'float', 90, {'unit': 'deg', 'min': -360, 'max': 360,
        'help': 'direction mode: side of the part where loops start, seen from above. 0 = +X, 90 = +Y (rear), 180 = -X, -90 = -Y.'}),
    ('Seam & layer change', 'seam_path', 'Seam line', 'text', '',
        {'help': 'path mode: points x,y,z; x,y,z; ... on the part (bed mm), drawn with Draw seam line. Each layer starts '
                 'where the line crosses its height.'}),
    ('Seam & layer change', 'seam_x', 'Custom seam point X', 'float', 0, {'unit': 'mm', 'min': -100000, 'max': 100000,
        'help': 'Bed coordinates. Used when Seam position = custom.'}),
    ('Seam & layer change', 'seam_y', 'Custom seam point Y', 'float', 0, {'unit': 'mm', 'min': -100000, 'max': 100000}),
    ('Seam & layer change', 'seam_corner', 'Move seam to a corner within', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 1000,
        'help': 'Start the loop at the sharpest corner (>= 20 deg) this close to the seam point, where the seam shows least. 0 = off.'}),
    ('Seam & layer change', 'layer_change', 'Layer change at the seam', 'choice', 'step', {'options': ['step', 'ramp', 'scarf'],
        'help': 'step: the nozzle moves up one layer at the seam. ramp: each loop ends at the seam and the next layer '
                'carries straight on, rising over the ramp length (a short helix; needs an aligned, rear or custom seam). '
                'scarf: each loop rises over the ramp, goes round and fills its own ramp wedge (works with any seam).'}),
    ('Seam & layer change', 'ramp_length', 'Ramp length', 'float', 20, {'unit': 'mm', 'min': 0, 'max': 10000,
        'help': 'Length of path over which Z rises by one layer (limited to 90 % of the loop).'}),
    ('Seam & layer change', 'ramp_start_height', 'Ramp start height', 'float', 0, {'unit': '% layer', 'min': 0, 'max': 100,
        'help': 'Bead thickness at the start of the ramp. 0 = starts on the previous layer; flow follows the bead thickness.'}),
    ('Seam & layer change', 'ramp_walls', 'Ramp on', 'choice', 'all_walls', {'options': ['all_walls', 'outer_wall']}),
    ('Seam & layer change', 'seam_gap', 'Seam gap (- = overlap)', 'float', 0, {'unit': 'mm', 'min': -1000, 'max': 1000,
        'help': 'Step mode: stop this far before the loop start (positive) or run past it (negative) to tune the seam blob.'}),
    ('Seam & layer change', 'wipe_length', 'Wipe after loop', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 1000,
        'help': 'Keep moving along the loop this far with the extruder off before travelling, to wipe the nozzle.'}),

    ('Printing strategy', 'print_strategy', 'Layer strategy', 'choice', 'horizontal', {'options': ['horizontal', 'angled'],
        'help': 'horizontal: flat layers. angled: layers on planes tilted by the slicing angle (e.g. 45 deg), so the '
                'robot can build overhangs out to that angle without support. The planes are shown on the part in Prepare.'}),
    ('Printing strategy', 'slice_angle', 'Slicing plane angle', 'float', 45, {'unit': 'deg', 'min': 0, 'max': 80,
        'help': 'Tilt of the layer planes from horizontal (angled strategy).'}),
    ('Printing strategy', 'slice_direction', 'Tilt direction', 'float', 0, {'unit': 'deg', 'min': -360, 'max': 360,
        'help': 'Direction in the bed XY plane the layers lean and grow towards: 0 = +X, 90 = +Y, 180 = -X, -90 = -Y.'}),
    ('Printing strategy', 'base_height', 'Horizontal base height', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 10000,
        'help': 'Print the part below this height with flat layers first, then the rest with angled layers on top of '
                'it. 0 = angled layers start on the bed.'}),
    ('Printing strategy', 'nozzle_orientation', 'Nozzle orientation', 'choice', 'normal', {'options': ['normal', 'vertical'],
        'help': 'normal: the nozzle tilts with the layers (perpendicular to the layer plane). vertical: the nozzle '
                'always points straight down.'}),

    ('Point control', 'point_mode', 'Point distribution', 'choice', 'simplify', {'options': ['simplify', 'uniform', 'adaptive'],
        'help': 'simplify: fewest points within the curve tolerance (spacing changes with the shape and from layer to '
                'layer). uniform: points at an even spacing, starting at the seam, so with an aligned seam they line up '
                'layer over layer. adaptive: within the curve tolerance, but between the min and max spacing (long even '
                'steps on straight parts, short ones in curves).'}),
    ('Point control', 'simplify', 'Curve tolerance', 'float', 0.2, {'unit': 'mm', 'min': 0, 'max': 10,
        'help': 'How far the path may deviate from the model outline. Used by simplify and adaptive.'}),
    ('Point control', 'point_spacing', 'Point spacing (uniform)', 'float', 2, {'unit': 'mm', 'min': 0.05, 'max': 1000,
        'help': 'Distance between points in uniform mode.'}),
    ('Point control', 'point_fit', 'Fit spacing to each section', 'bool', True,
        {'help': 'Uniform: adjust the spacing slightly so every loop (or the part between two kept corners) divides '
                 'into equal steps, with no short step at the end.'}),
    ('Point control', 'point_min_spacing', 'Min point spacing', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 1000,
        'help': 'Points closer than this are merged (corners and seams stay). Adaptive: the shortest step. 0 = off.'}),
    ('Point control', 'point_max_spacing', 'Max point spacing', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 10000,
        'help': 'Long segments are split so points are never further apart than this (useful for the robot to '
                'follow long straight moves). 0 = off.'}),
    ('Point control', 'point_keep_corners', 'Keep sharp corners', 'bool', True,
        {'help': 'Corners sharper than the corner angle always get a point, so they stay sharp.'}),
    ('Point control', 'point_corner_angle', 'Corner angle', 'float', 30, {'unit': 'deg', 'min': 1, 'max': 180,
        'help': 'A change of direction of at least this much counts as a corner.'}),
    ('Point control', 'point_smooth', 'Smoothing passes', 'int', 0, {'min': 0, 'max': 50,
        'help': 'Smooth the small zig-zag of the traced outline before placing points (corners stay). 0 = off.'}),
    ('Point control', 'point_apply', 'Apply to', 'choice', 'walls', {'options': ['walls', 'all'],
        'help': 'walls: walls, brim and spiral vase. all: infill too.'}),

    ('Surface pattern', 'pattern', 'Pattern', 'choice', 'none',
     {'options': ['none', 'sine', 'zigzag', 'spikes', 'bumps', 'noise', 'image'],
        'help': 'Relief pressed into the outer wall (and the spiral vase): the nozzle moves in and out along the wall. '
                'sine: smooth waves. zigzag: straight lines with sharp tips. spikes: sharp spikes with plain wall '
                'between. bumps: round domes in rows. noise: smooth random relief. image: a picture wrapped round '
                'the part. Inner walls and infill stay plain.'}),
    ('Surface pattern', 'pattern_amplitude', 'Depth', 'float', 3, {'unit': 'mm', 'min': 0, 'max': 200,
        'help': 'How far the wall moves between the bottom and the top of a wave.'}),
    ('Surface pattern', 'pattern_wavelength', 'Wavelength', 'float', 20, {'unit': 'mm', 'min': 0.5, 'max': 10000,
        'help': 'Length of one wave along the wall.'}),
    ('Surface pattern', 'pattern_fit', 'Whole waves per lap', 'bool', True,
        {'help': 'One whole number of waves per lap for the full height of the part (each outline gets its own), so '
                 'the waves line up layer after layer: ribs stay vertical and diagonals keep their angle. The '
                 'wavelength stretches a little where the part is wider. Off: a fixed wavelength in mm.'}),
    ('Surface pattern', 'pattern_waves', 'Waves per lap', 'int', 0, {'min': 0, 'max': 100000,
        'help': 'The number of waves round the part. 0 = from the wavelength (mean perimeter / wavelength). The '
                'chosen number and the wavelength it gives are shown in the Surface pattern card after slicing.'}),
    ('Surface pattern', 'pattern_shift', 'Shift per layer', 'float', 0, {'unit': '% wave', 'min': -100, 'max': 100,
        'help': 'How far the waves move along the wall on each layer. 0 = stacked waves (vertical ribs). 50 = each '
                'layer sits between the waves below (lattice / mesh look). Small values twist the ribs.'}),
    ('Surface pattern', 'pattern_direction', 'Direction', 'choice', 'outward', {'options': ['outward', 'centered', 'inward'],
        'help': 'outward: the wall only grows (inner walls untouched). centered: half out, half in. inward: the wall '
                'only cuts in (best with one wall or spiral vase).'}),
    ('Surface pattern', 'pattern_spike_width', 'Spike / bump width', 'float', 50, {'unit': '%', 'min': 2, 'max': 100,
        'help': 'spikes: how much of each wave the spike takes up; the rest is plain wall. bumps: dome diameter as a '
                'share of the wavelength or the row height (the smaller).'}),
    ('Surface pattern', 'pattern_row_height', 'Row height', 'float', 20, {'unit': 'mm', 'min': 0.5, 'max': 10000,
        'help': 'bumps: distance between the rows of domes up the wall (Shift per layer then moves each row). '
                'noise: the size of the relief upwards (Wavelength is the size along the wall).'}),
    ('Surface pattern', 'pattern_seed', 'Noise variation', 'int', 1, {'min': 0, 'max': 1000000,
        'help': 'noise: another number gives another random relief.'}),
    ('Surface pattern', 'pattern_image', 'Picture', 'text', '', {'hidden': True,
        'help': 'image: the picture file (uploaded in the Surface pattern card, kept in profiles/patterns).'}),
    ('Surface pattern', 'pattern_image_raise', 'Raised parts', 'choice', 'dark', {'options': ['dark', 'light'],
        'help': 'image: dark: black parts of the picture stand out (a black logo on white). light: white parts stand out.'}),
    ('Surface pattern', 'pattern_image_angle', 'Picture centre direction', 'float', -90, {'unit': 'deg', 'min': -360, 'max': 360,
        'help': 'image: the side of the part where the middle of the picture goes, seen from above. 0 = +X, 90 = +Y '
                '(rear), -90 = -Y (front).'}),
    ('Surface pattern', 'pattern_image_span', 'Picture width', 'float', 360, {'unit': 'deg', 'min': 1, 'max': 360,
        'help': 'image: how far round the part the picture reaches (360 = all the way round). Its height is From '
                'height to To height (the whole part when both are 0).'}),
    ('Surface pattern', 'pattern_image_tile', 'Repeat the picture', 'bool', False,
        {'help': 'image: repeat the picture all the way round, every Picture width degrees.'}),
    ('Surface pattern', 'pattern_paint_mode', 'Painted areas', 'choice', 'everywhere',
     {'options': ['everywhere', 'painted', 'unpainted'],
        'help': 'everywhere: the whole wall. painted: only where you painted on the part (Paint in the Surface pattern '
                'card). unpainted: everywhere except the painted areas.'}),
    ('Surface pattern', 'pattern_brush', 'Brush size', 'float', 15, {'unit': 'mm', 'min': 0.5, 'max': 1000,
        'help': 'Radius of the paint brush; the pattern fades out over the outer third.'}),
    ('Surface pattern', 'pattern_paint', 'Painted points', 'text', '', {'hidden': True,
        'help': 'Points x,y,z; ... painted on the part (model position on the bed).'}),
    ('Surface pattern', 'pattern_points', 'Points per wave', 'int', 12, {'min': 2, 'max': 200,
        'help': 'Path points in each wave (the sharp tips of zigzag and spikes always get one). More = smoother sine, '
                'but more robot moves: use corner blending. image: a point every Wavelength / Points per wave mm.'}),
    ('Surface pattern', 'pattern_z_min', 'From height', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 100000,
        'help': 'Model height where the pattern starts (below it the wall is plain).'}),
    ('Surface pattern', 'pattern_z_max', 'To height', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 100000,
        'help': 'Model height where the pattern ends. 0 = to the top.'}),
    ('Surface pattern', 'pattern_fade', 'Fade in / out', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 100000,
        'help': 'The depth grows from 0 over this height above From and shrinks to 0 below To.'}),
    ('Surface pattern', 'pattern_v_period', 'Vertical wave', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 100000,
        'help': 'The depth rises and falls with height over this period, giving bands of pattern and plain wall. '
                '0 = same depth at every height.'}),

    ('Infill', 'infill_density', 'Sparse infill density', 'float', 20, {'unit': '%', 'min': 0, 'max': 100}),
    ('Infill', 'infill_pattern', 'Sparse infill pattern', 'choice', 'rectilinear', {'options': ['rectilinear', 'concentric']}),
    ('Infill', 'infill_angle', 'Infill angle', 'float', 45, {'unit': 'deg', 'min': -180, 'max': 180}),
    ('Infill', 'infill_overlap', 'Infill/wall overlap', 'float', 25, {'unit': '%', 'min': 0, 'max': 100}),
    ('Infill', 'connect_infill', 'Connect infill lines', 'bool', True, {'help': 'Zig-zag between lines inside the part. Fewer extruder start/stops.'}),

    ('Speed', 'outer_wall_speed', 'Outer wall', 'float', 30, {'unit': 'mm/s', 'min': 1, 'max': 1000}),
    ('Speed', 'inner_wall_speed', 'Inner wall', 'float', 40, {'unit': 'mm/s', 'min': 1, 'max': 1000}),
    ('Speed', 'infill_speed', 'Sparse infill', 'float', 50, {'unit': 'mm/s', 'min': 1, 'max': 1000}),
    ('Speed', 'solid_infill_speed', 'Solid infill', 'float', 40, {'unit': 'mm/s', 'min': 1, 'max': 1000}),
    ('Speed', 'first_layer_speed', 'First layer', 'float', 20, {'unit': 'mm/s', 'min': 1, 'max': 1000}),
    ('Speed', 'travel_speed', 'Travel', 'float', 150, {'unit': 'mm/s', 'min': 1, 'max': 2000}),

    ('Retraction', 'retract_length', 'Retract length', 'float', 0.0, {'unit': 'E', 'min': 0, 'max': 1000,
                                                                      'help': 'In E units. Any retraction switches the extruder off in Borunte_FGF.'}),
    ('Retraction', 'retract_min_travel', 'Retract if travel longer than', 'float', 10, {'unit': 'mm', 'min': 0, 'max': 10000}),
    ('Retraction', 'z_hop', 'Z hop on travel', 'float', 3.0, {'unit': 'mm', 'min': 0, 'max': 500}),

    ('Material', 'material', 'Material', 'text', '', {'hidden': True,
        'help': 'The material chosen in the Material card (Prepare): sets the density and the {material}, {zone_temps} ... '
                'placeholders of the program events.'}),
    ('Material', 'e_mode', 'E units', 'choice', 'volumetric', {'options': ['volumetric', 'filament'],
                                                               'help': 'volumetric: E in mm3 (pellet/FGF). filament: E in mm of filament.'}),
    ('Material', 'filament_diameter', 'Filament diameter', 'float', 1.75, {'unit': 'mm', 'min': 0.5, 'max': 10}),
    ('Material', 'flow', 'Flow multiplier', 'float', 100, {'unit': '%', 'min': 10, 'max': 300}),
    ('Material', 'overhang_limit', 'Max overhang angle', 'float', 45, {'unit': 'deg', 'min': 0, 'max': 89,
        'help': 'The steepest overhang (from vertical) this material prints without sagging. Walls past it are '
                'flagged after slicing; colour the preview by Overhang angle to see them.'}),
    ('Material', 'density', 'Material density', 'float', 1.2, {'unit': 'g/cm3', 'min': 0.1, 'max': 20}),

    ('Robot & bed', 'bed_x', 'Print area X', 'float', 1000, {'unit': 'mm', 'min': 10, 'max': 10000}),
    ('Robot & bed', 'bed_y', 'Print area Y', 'float', 1000, {'unit': 'mm', 'min': 10, 'max': 10000}),
    ('Robot & bed', 'bed_z', 'Print height', 'float', 800, {'unit': 'mm', 'min': 10, 'max': 10000}),
    ('Robot & bed', 'nozzle_diameter', 'Nozzle diameter', 'float', 4.0, {'unit': 'mm', 'min': 0.1, 'max': 50}),
    ('Robot & bed', 'user_frame_id', 'Controller user frame #', 'int', 1, {'min': 0, 'max': 99,
                                                                          'help': 'Borunte user frame taught at the bed origin.'}),
    ('Robot & bed', 'tool_id', 'Controller tool #', 'int', 1, {'min': 0, 'max': 99, 'help': 'Borunte tool with the TCP at the nozzle tip.'}),
    ('Robot & bed', 'safe_z', 'Approach / retreat height', 'float', 50, {'unit': 'mm', 'min': 0, 'max': 2000}),
    ('Robot & bed', 'blending', 'Corner blending (smooth)', 'int', 0, {'min': 0, 'max': 100,
                                                                     'help': 'Written to the Borunte smooth field. Verify its effect on your controller.'}),
    ('Robot & bed', 'def_spd_mms', 'Controller 100% speed', 'float', 1000, {'unit': 'mm/s', 'min': 10, 'max': 10000,
                                                                          'help': 'Borunte speeds are written as % of this value.'}),

    ('Extruder I/O', 'extruder_enable_do', 'Extruder enable DO', 'int', 1, {'min': 0, 'max': 999}),
    ('Extruder I/O', 'flow_level_dos', 'Flow level DOs (LSB first)', 'text', '2,3,4', {'help': 'Empty = on/off only. 3 outputs = 7 levels.'}),
    ('Extruder I/O', 'flow_mode', 'Flow level from', 'choice', 'rate', {'options': ['rate', 'per_mm'],
                                                                       'help': 'rate: E per second (screw RPM). per_mm: E per mm of path.'}),
    ('Extruder I/O', 'flow_full_scale', 'Full-scale flow (0 = auto)', 'float', 0, {'min': 0, 'max': 1e6,
                                                                                    'help': 'E/s (rate) or E/mm (per_mm) at the top level. Auto = the highest value in this job.'}),
    ('Extruder I/O', 'start_delay', 'Delay after extruder on', 'float', 0.0, {'unit': 's', 'min': 0, 'max': 60}),

    # ---- Robot cell (shown in the Robot workspace)
    ('Robot cell', 'robot_model', 'Robot model', 'text', 'BORUNTE BRTIRUS 18020A', {}),
    ('Robot cell', 'post_name', 'Post-processor', 'text', 'Borunte .act', {'help': 'Program language and events used by Export.'}),
    ('Robot cell', 'tcp', 'Tool TCP (nozzle tip)', 'text', '0,0,150,0,0,0',
     {'help': 'X,Y,Z,Rx,Ry,Rz of the nozzle tip relative to the robot flange (mm, deg).'}),
    ('Robot cell', 'tool_model', 'Tool 3D model', 'text', '', {'help': 'STL / OBJ / 3MF of the tool head, stored in profiles/tools.'}),
    ('Robot cell', 'tool_model_pose', 'Tool model on the flange', 'text', '0,0,0,0,0,0',
     {'help': "X,Y,Z,Rx,Ry,Rz of the model's origin relative to the flange. 0 if the CAD origin is the flange centre with Z out of the flange."}),
    ('Robot cell', 'tool_model_scale', 'Tool model scale', 'float', 100, {'unit': '%', 'min': 0.001, 'max': 1e6,
                                                                         'help': '1000% for a model exported in cm, 100000% for metres.'}),
    ('Robot cell', 'bed_frame', 'Bed frame (user frame)', 'text', '600,-500,0,0,0,0',
     {'help': 'X,Y,Z,Rx,Ry,Rz of the bed origin relative to the robot base (mm, deg). Slicer X/Y/Z are in this frame.'}),
    ('Robot cell', 'tool_rz', 'Tool rotation about Z', 'float', 0, {'unit': 'deg', 'min': -360, 'max': 360,
                                                                    'help': 'Preferred rotation of the nozzle about its own Z axis.'}),
    ('Robot cell', 'rotz_range', 'Free rotation about tool Z', 'float', 180, {'unit': '± deg', 'min': 0, 'max': 180,
                                                                              'help': 'The nozzle is round, so the robot may turn it about Z to avoid limits and singularities.'}),
    ('Robot cell', 'rotz_step', 'Rotation search step', 'float', 10, {'unit': 'deg', 'min': 1, 'max': 90}),
    ('Robot cell', 'sing_margin', 'Wrist singularity margin |J5| ≥', 'float', 8, {'unit': 'deg', 'min': 0, 'max': 60}),
    ('Robot cell', 'lock_config', 'Lock robot configuration', 'bool', True,
     {'help': 'Keep front/rear, elbow and wrist the same for the whole print (no arm flips mid-print).'}),
    ('Robot cell', 'start_joints', 'Preferred start joints', 'text', '',
     {'help': 'J1..J6 (deg). Sets the configuration and the joint move to the first point. Empty = choose automatically.'}),
    ('Robot cell', 'joint_jump_max', 'Max joint jump between points', 'float', 30, {'unit': 'deg', 'min': 1, 'max': 360}),
    ('Robot cell', 'check_step', 'Check long moves every', 'float', 25, {'unit': 'mm', 'min': 1, 'max': 1000}),
    ('Robot cell', 'floor_margin', 'Robot links above bed by', 'float', 20, {'unit': 'mm', 'min': -2000, 'max': 2000,
                                                                           'help': 'Simple collision check: robot joints must stay this far above the bed surface.'}),

    ('Advanced', 'resolution', 'Slicing resolution (0 = auto)', 'float', 0, {'unit': 'mm', 'min': 0, 'max': 5,
                                                                              'help': 'Raster cell size used for offsets. Auto = line width / 10.'}),
]

DEFAULTS = {key: default for _, key, _, _, default, _ in SPEC}
TYPES = {key: typ for _, key, _, typ, _, _ in SPEC}


def spec_json():
    return [{'group': g, 'key': k, 'label': l, 'type': t, 'default': d, **x} for g, k, l, t, d, x in SPEC]


def normalize(values):
    """Merge user values over defaults and coerce types. Unknown keys are dropped."""
    s = dict(DEFAULTS)
    for key, value in (values or {}).items():
        if key not in TYPES:
            continue
        typ = TYPES[key]
        try:
            if typ == 'float':
                value = float(value)
            elif typ == 'int':
                value = int(round(float(value)))
            elif typ == 'bool':
                value = value if isinstance(value, bool) else str(value).lower() in ('1', 'true', 'yes', 'on')
            else:
                value = str(value)
        except (TypeError, ValueError):
            continue
        s[key] = value
    return s


def parse_int_list(text):
    return [int(v) for v in re.split(r'[\s,;]+', str(text).strip()) if v]


def parse_float_list(text):
    return [float(v) for v in re.split(r'[\s,;]+', str(text).strip()) if v]


def _profile_path(name):
    safe = re.sub(r'[^A-Za-z0-9 _.-]', '_', name).strip() or 'profile'
    return os.path.join(PROFILE_DIR, safe + '.json')


def list_profiles():
    if not os.path.isdir(PROFILE_DIR):
        return []
    return sorted(os.path.splitext(f)[0] for f in os.listdir(PROFILE_DIR) if f.endswith('.json'))


def load_profile(name):
    with open(_profile_path(name), 'r', encoding='utf-8') as f:
        return normalize(json.load(f))


def save_profile(name, values):
    os.makedirs(PROFILE_DIR, exist_ok=True)
    path = _profile_path(name)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(normalize(values), f, indent=2)
    return path
