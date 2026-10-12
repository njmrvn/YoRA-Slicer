# YoRA Slicer — slicing for robot FGF 3D printing (Borunte, KUKA)

A slicer in the spirit of PrusaSlicer / OrcaSlicer / Bambu Studio, built for **large-nozzle pellet (FGF)
printing with a Borunte BRTIRUS robot**. You load a model, slice it, preview the toolpath and the extruder
signals, and then output:

1. **Robot program** with a post-processor you choose: **Borunte `.act`**, **KUKA KRL** (KR C4), **G-code**, or your own
   version of one of them with event templates (see *Post-processors*)
2. **G-code** (absolute E, no E resets) for other machines and tools

It also **imports G-code from Orca / Prusa / Bambu / Cura**. You can slice there and use this app only for the robot side.

<img width="2555" height="1406" alt="image" src="https://github.com/user-attachments/assets/f5611b4c-29a4-4562-8899-1447d60a6b63" />
<img width="2550" height="1405" alt="image" src="https://github.com/user-attachments/assets/990cc2f3-4fd7-4411-9993-5bfbec0bcb4c" />


## Start

Double-click **`YoRA Slicer.bat`**. The GUI opens in an app window. Slicing, robot models, robot check,
simulation and the robot program export all run in the app; it needs only Python, numpy and OpenCV.

The app needs Python 3.10 or newer (python.org or the Microsoft Store). On the first start the launcher runs
**`Setup YoRA.bat`**, which makes a private environment in `%USERPROFILE%\.yora\venv` (outside OneDrive) and
installs `requirements.txt` (numpy, OpenCV). The 3D view loads three.js from cdn.jsdelivr.net, so the first start needs internet.

From a terminal in the project folder (any Python 3.10+ with the requirements):
```bash
python yora_slicer.py --open
```

## Interface
Four steps across the top: **1 Prepare** (object and print settings), **2 Preview** (toolpath and print summary),
**3 Robot** (sub-tabs *Setup · Jog · Path · Check*), **4 Export** (post-processor, program preview, export).
Print settings open on *Essentials*; *All* shows every setting, and the search box finds any setting by name or help
text. Help texts appear when you point at or click a setting. The **⋯** menu sets the theme (Auto / Light / Dark / Maroon / YoRA). All colours are tokens in
`web/tokens.css`: Light is teal on light grey; Dark is the Material dark palette (#121212 surfaces, purple #BB86FC
primary, teal #03DAC6 secondary for checkboxes and sliders, #CF6679 errors); Maroon is maroon, beige and black (warm black
surfaces, beige text and controls, maroon #550000 buttons). YoRA is a light greige theme (#B0AA98, #C3BDA7, #B8B29B surfaces, #574D48 / #4A413C
warm browns for text and selected items) with a fine dot texture. Surfaces and the 3D grid stay neutral, the primary colour marks buttons, the active step and selected
items, and red is kept for errors and alarms. The 3D view reads the same tokens at run time. Print-settings groups stay
open until you close them (remembered between sessions).
It also sets the **style**: **Clay** (default: soft raised panels and buttons, recessed inputs, rounded shapes, Nunito /
DM Sans fonts, gentle lift and press motion; `web/clay.css`) or **Flat** (the plain look). Style and theme are
independent, and the style changes only the look.

**The previous interface is kept unchanged** at `/classic/`: **⋯ → Switch to the classic interface**, and back with
**New interface** in its top bar. The app remembers the choice; `YoRA Slicer.bat --ui classic` also opens it.
Settings, profiles, robot cells and models are shared. The classic files are a frozen copy in `web/classic/`
(to make it the default for good, copy them over `web/`).

## Workflow

| Step | In the app |
|---|---|
| Load | **Open…**, drag-and-drop, or **Samples…** (STL / OBJ / 3MF to slice; G-code to import) |
| Place | Object panel: scale, rotate X/Y/Z, position on the bed. Models always sit on Z=0, and the size turns red if the model is outside the print area |
| Settings | Profile + grouped settings (layers & walls, infill, speed, retraction, material, robot & bed, extruder I/O). Changed values get a dot |
| Slice | Background slicing with progress |
| Preview | Layer slider (↑/↓ keys), single-layer view, travel moves, **Points** (every point of the toolpath = the targets the robot moves through, with their count), color by **feature**, **extruder flow level (DO)** or **speed**. **Show as** *Toolpath (lines)* or *Extrusions*: solid beads at the layer height, with the width worked out from the extruded volume (so over- and under-extrusion shows). The status bar shows time, material, peak flow (mm³/s and kg/h) and flow-level changes |
| Export | Step **4 · Export**: pick the post-processor, **Preview program**, **Export**; also G-code. Files are also saved in `out/` |

### Slicing engine (`yora/slicer.py`)
- Mesh is cut at mid-layer and turned into oriented loops, then a raster mask with **non-zero winding**, so overlapping or touching shells merge and inverted meshes still work.
- A Euclidean distance transform finds the wall regions (`w/2 + k·w` from the outline). Each wall point is then moved
  onto the **exact offset of the model's cross-section** (the mesh cut, not the pixel grid), so walls are smooth and
  within the *Path simplify tolerance* of the model (no 0.5 mm staircase), then simplified to that tolerance.
- **Infill:** rectilinear (alternating ±90° per layer) or concentric. Solid **top/bottom** layers are found by comparing masks of neighbouring layers. Lines are **connected into zig-zags** inside the part to cut extruder start/stops.
- **Brim**, **spiral vase** (continuous Z rise, solid bottom layers), inner-first or outer-first walls.
- **Seam & layer change** (own settings group):
  - *Seam position*: aligned (each loop starts on the previous layer's seam of the same wall), rear, **direction** (the
    side at *Seam direction*, 0 = +X, 90 = +Y), **custom** (closest to a point), **path** (follows a line drawn on the
    part: each layer starts where the line crosses its height), nearest, random. The loop starts exactly at that point
    (a vertex is inserted), not at the nearest existing vertex. Works with angled layers too (points and lines are in
    the bed frame).
  - **Seam card in Prepare**: *Pick point* (click on the part), *Draw line* (left-drag on the part paints the line,
    right-drag turns the view, Enter / *Done* finishes, Backspace undoes, Esc cancels), *Reset* (back to aligned). The
    current seam is drawn on the model; in the preview, tick **Seams** to mark where every outer wall loop starts.
  - *Move seam to a corner within*: start at the sharpest corner (≥ 20°) this close to the seam point, where it shows least.
  - *Layer change at the seam*:
    - **step**: Z moves up one layer at the seam.
    - **ramp**: each loop ends exactly at the seam and the next layer carries straight on from there, rising one layer
      over *Ramp length* (a short helix at every layer change, no travel back). The ramp starts exactly where the loop below
      ended and moves across onto the new wall while it climbs, so layers join even where the wall shifts (cones, twists). Each ramp lies on the previous one, so
      the bead stays a full layer thick; the first ramp of a wall grows from *Ramp start height*, and the wedge is
      filled once where the wall ends (top of the part). Needs an aligned, rear or custom seam.
    - **scarf**: every loop rises over the ramp, goes round, then passes over the ramp again filling its own wedge
      (ends one ramp length past the seam). Works with any seam position.
    Ramp and scarf use the same material as a step. *Ramp on*: all walls or the outer wall only. Not on the first layer.
  - *Seam gap*: stop this far before the loop start (or overlap past it with a negative value). *Wipe after loop*: keep
    moving along the loop with the extruder off before travelling.
- Retraction and Z-hop on long travels. Bead cross-section is a stadium shape. E can be **volumetric (mm³)** or filament mm.

### Extruder signal
The extruder is driven through digital outputs, the only extruder I/O the Borunte `.act` writer supports:
- `Extruder enable DO` (default DO1) switches the extruder on/off.
- `Flow level DOs` (default 2,3,4) carry the level in binary, least significant bit first, so 3 outputs give 7 levels.
- **Flow from `rate`** (default): the level follows E per **second** (E/mm × speed), which is what screw RPM sets. `per_mm` follows E per mm of path.
- `Full-scale flow = 0` scales the job's peak flow to the top level. Set a fixed value (your extruder's maximum) so levels mean the same thing on every job.

Preview "Extruder flow level (DO)" uses the same rule as the post, so what you see is what gets written.

## Robot workspace
Open the **Robot** tab. Everything for setting up the robot and checking the print runs **inside the slicer**
(`yora/kinematics.py`, `yora/robotpath.py`).

| Step | In the Robot tab |
|---|---|
| Save the station | **Robot cell** bar at the top of the Robot tab: *Save as…* stores the whole setup in `profiles/cells/<name>.json`: robot model and its joint limits, tool 3D model and where it sits on the flange, TCP, bed frame and print area, controller frame/tool numbers, rotation / configuration / check settings and the jogged robot position. *Load* puts it all back. *Export* makes a `.yoracell` file that also contains the robot (kinematics + 3D links) and tool models, and *Import…* installs it on another PC |
| Robot model with correct joint limits | **1 · Robot model**: *Load robot file (.robot)* opens a `.robot` robot model file (from your Downloads, Desktop or Documents, or drag-and-drop). It reads the kinematics **and the 3D model of every link** and saves `profiles/robots/<name>.json`. Joint limits can be edited and saved. |
| Jog the robot | **Move robot (jog)**: a slider per joint, Cartesian jog of the nozzle in the bed frame (X/Y/Z ±, rotate about tool Z), *Home*, *Start joints*, *Use as start joints*, and a live TCP readout with limit, singularity and collision warnings. |
| Tool (TCP at the nozzle tip) | **2 · Tool**: *Import model…* loads your tool head (STL / OBJ / 3MF, kept in `profiles/tools/`). Place it on the flange (position + scale; 0 if the CAD origin is the flange centre with Z out of the flange), then *Set TCP at nozzle tip* finds the narrow end of the model and points tool Z out of the nozzle. The TCP (X Y Z Rx Ry Rz) can also be typed. The model is drawn on the robot and included in the collision check |
| Reference frame at the bed origin | **3 · Bed frame**: pose of the bed origin relative to the robot base. *Show reachable bed area* colours the bed green or red |
| Tool orientation normal to the bed | Always: nozzle Z points straight down |
| Free rotation about tool Z | **4**: ± range and search step. Rotation stays constant where possible and changes only to avoid limits or singularities (minimum orientation change) |
| Preferred configuration, locked | **4**: *Lock robot configuration*, *Preferred start joints* (or *Use robot home*) |
| Simulation: reach, singularity, collision, cycle time | **6 · Check robot path** solves the joints for every move with closed-form IK (all 8 configurations). With *Lock robot configuration* on, problem-free stretches are solved in one vectorised step and only moves with a problem go through the move-by-move search, with the same result (tube sample: 21.6 s → 1.0 s). It reports unreachable points, joint limits, wrist singularity (\|J5\| below the margin), configuration jumps, and collisions. With a robot loaded from a `.robot` file the collision check uses the **3D links** (links 2–6) against the bed area and the part already printed; otherwise it checks the joint positions against the bed plane. You also get joint charts against the limits, J6 wind-up, limit margins, cycle time, a clickable problem list and red markers on the part. The **simulation** plays the real 3D robot on a clock: 1× real time, 5× (normal), 20×, 100× (fast) or 500×. It shows the time, layer and move, and colliding links turn red. With *print as it moves* the toolpath is drawn only up to where the nozzle is, so it grows behind the tool as it prints (lines or extrusions) |
| Custom post | Export Borunte .act uses the checked path: the solved rotation for each move, and a joint move to the solved start joints |

### Robot files (`yora/robot_file.py`)
`.robot` robot model files are read directly (the file layout is described at the top of `robot_file.py`; the
kinematics were checked against manufacturers' DH tables and datasheets, `tests/test_robot_file.py`).
- Kinematics: the modified DH (DHM) table, joint senses and limits become the same PoE model the IK already uses.
- 3D links: every link's coloured triangles, merged per colour (`profiles/robots/<name>.mesh.bin`), drawn with
  smooth normals and posed by forward kinematics in the browser (`web/robot.js`, same math as `kinematics.py`).
  All triangles are kept up to 1.5 million; bigger models are simplified by merging nearby points (surfaces stay
  closed). Extra parts such as the weight-compensation cylinder of large KUKA arms are drawn on the column (they
  turn with A1; their small swing with A2 is not animated). The home joints come from the file (KUKA: 0,-90,90,0,0,0).
- Collision points: ~150 surface points per link (support points in 150 directions), so the lowest point of a link in
  any pose is one of them.
- Works for 6-axis arms (ABB, Borunte, Comau, Fanuc, Kawasaki, KUKA, Motoman, UR, ...).
  External axes, turntables and grippers (1–4 axis `.robot` files) are listed but can't be loaded.

## Raft (`yora/raft.py`, print settings → Raft)
Layers printed under the part; the part is lifted onto them. **Raft** type:

| Type | Raft area |
|---|---|
| outline | the shape of the part's first layer (holes filled) |
| hull | the closed area of the part: an arc-shaped part gets a half disc, an L a triangle |
| rectangle | the bounding box (sides along X / Y) |
| fitted_rectangle | the smallest rectangle around the part, turned to fit it |
| circle | the smallest circle around the part |

*Raft offset*: how far the raft reaches beyond the part. *Raft line angle* and *Angle change per raft layer* (90 =
crossed layers). *Raft density* (%, lower layers). The lines of each raft layer are one continuous zig-zag (joined
along the raft edge). Also: raft layers, raft layer height (0 = first layer height), solid top raft layer, outline
loop (off by default), gap to the part and raft speed. The raft is its own feature type (Raft) in the
preview and in G-code (`;TYPE:raft`); with a raft there is no brim. Not used with angled layers (use the horizontal
base there).

## Materials (`yora/material.py`, Prepare → Material card)
Pick the material in the **Material** card; **New** / **Edit** opens the material editor:
name, brand, type, colour, **heating zones** (*1 zone*: one print temperature, or *Multiple zones*: up to 12 named
zones, e.g. Feed → Zone 2 → Nozzle, each with its own temperature), bed temperature, density, flow multiplier
(0 = keep the print profile's), drying temperature / time and notes. Materials are saved in `profiles/materials/`;
three built-in examples (PLA, PETG, PP-GF) are read only (editing one saves your own copy).
Choosing a material sets the density (used for the material weight) and the flow multiplier if the material has one,
and it is saved with print profiles. In post-processor events: `{material} {material_brand} {material_type}
{zone_temps} {nozzle_temp} {temp_1} {temp_2} ... {zone_count} {bed_temp}` (e.g. Program start → `COMMENT {material}:
{zone_temps}`, or in G-code `M104 S{nozzle_temp}`).

## Printing strategy (`yora/strategy.py`, print settings → Printing strategy)
| Setting | What it does |
|---|---|
| Layer strategy | **horizontal** (flat layers) or **angled**: layers on planes tilted by the slicing angle, so overhangs up to that angle print without support |
| Slicing plane angle | tilt of the layer planes from horizontal (e.g. 45°) |
| Tilt direction | direction in bed XY the layers lean and grow towards (0 = +X, 90 = +Y, 180 = −X) |
| Horizontal base height | the part below this height is printed with flat layers first, then the angled layers on top (0 = angled from the bed) |
| Nozzle orientation | **normal**: the nozzle tilts with the layers (perpendicular to them); **vertical**: always straight down |

With *angled*, **Prepare** draws the slicing planes on the part (contour lines, one translucent plane, an arrow for the
direction the layers grow, and the base plane). **Drag the ball at the arrow tip with the mouse** to tilt and turn
the plane (Shift: 5° steps); the angle and direction settings follow. The model is sliced in the tilted frame with the normal slicer (walls,
infill, seams, point control unchanged) and the toolpath is rotated back. The robot check, simulation, part-collision
check and the Borunte / KUKA programs follow the tilted nozzle; G-code carries positions only.

## Point control (`yora/points.py`, print settings → Point control)
Where the points of each path sit (tick **Points** in the preview to see them):

| Setting | What it does |
|---|---|
| Point distribution | **simplify** (default, as before): fewest points within the curve tolerance; spacing varies with the shape and from layer to layer. **uniform**: even spacing starting at the seam, so with an aligned seam the points line up layer over layer. **adaptive**: within the tolerance, between min and max spacing |
| Curve tolerance | max deviation from the model outline (simplify, adaptive) |
| Point spacing (uniform), Fit spacing to each section | the step; *fit* makes every loop (or corner-to-corner part) divide into equal steps |
| Min / Max point spacing | merge points closer than min; split segments longer than max (0 = off) |
| Keep sharp corners, Corner angle | corners always get a point |
| Smoothing passes | take out the small zig-zag of the traced outline first |
| Apply to | walls (walls, brim, spiral vase) or all (infill too) |

## Post-processors (`yora/post.py`, step 4 · Export → Edit…)
A post-processor = a **language** + its options + **events**. Built in (read only, *Duplicate* to make your own):

| Post-processor | Writes |
|---|---|
| Borunte .act | the BRTIRUS `.zip` package, byte-identical to the export before post-processors existed |
| KUKA KRL (KR C4) | `.src` + `.dat` (KSS 8): `$BASE`/`$TOOL` from *Controller user frame #* / *tool #*, first motion PTP, `LIN {X,Y,Z,A,B,C}` (A, B, C = rotation about Z, Y, X), `$VEL.CP` in m/s, `$APO.CDIS` + `C_DIS` from *Corner blending* (mm), extruder outputs switched with `TRIGGER WHEN DISTANCE=0 … DO $OUT[n]` so they change exactly at the start of the move. Options: output switching (trigger / `$OUT_C` / direct), PTP speed, `$ADVANCE`, split into sub-programs of N moves |
| G-code | absolute-E G-code; DO / WAIT commands map to editable codes (default `M42`, `G4`, `M226`) |

The extruder outputs (enable DO + flow-level DOs, start delay, from *Extruder I/O*) are written the same way for
Borunte and KUKA (the `FGFMixin` in `posts/Borunte_FGF.py`, on top of `posts/borunte_act.py` or `posts/kuka_krl.py`).

**Events**: *Program start, Layer start, Extruder on, Extruder off, Before travel, After travel, Layer end, Program end*.
One command per line: `DO n ON|OFF`, `WAIT seconds`, `WAIT_DI n ON|OFF`, `COMMENT text`; in KUKA KRL and G-code any
other line is copied as raw code (Borunte `.act` has no raw code). Placeholders: `{program} {layer} {layers} {z}
{layer_height} {time_min} {material_g} {tool} {base} {extruder_do} {date} {post} {moves}`. The editor checks every line
as you type, and *Preview with this job* shows the start of the program. Your post-processors are saved in
`profiles/posts/`; the one chosen for export is part of the robot cell.

On KUKA, a `WAIT` stops the advance run, so outputs before it are set directly after `WAIT SEC 0` (the KRL idiom), and
the robot stops there: use waits only where a stop is intended (for example *Extruder on* → `WAIT 0.3`).
The KUKA output has not been run on a real KR C4 yet: test it in T1 first (check `BASE_DATA` / `TOOL_DATA` numbers,
output numbers, joint signs of the first PTP and the program size).

## Direct `.act` export
The `.act` package is written by **`posts/borunte_act.py`**, the project's own Borunte writer. Its output is pinned
byte for byte by reference fingerprints (`tests/test_borunte_act.py`, 67 programs), and it runs in linear time (the vase
sample exports in about 5 seconds).

Export calls the post with `Extruder(E)` before each `MoveL` and uses:
- **controller user frame #** = bed origin taught on the robot, and **tool #** = nozzle TCP. `.act` stores only these numbers, so they must match the Bed frame and Tool entered in the Robot tab.
- tool Z pointing down (`Rx = 180°`). After a robot check, each move gets its solved rotation about Z and the program starts with a `MoveJ` to the solved start joints. Without a check, the fixed *tool rotation about Z* is used and you get a warning.

### Borunte `.act` format
| action | meaning | key fields |
|---|---|---|
| 4 | joint move | `pos.m0..m5` joints, `speed` %, `smooth` |
| 10 | linear move | `pos.m0..m5` = X Y Z Rx Ry Rz, `speed` %, `smooth` |
| 17 | circular move | two points |
| 200 | digital output | `point` = output, `pointStatus` |
| 100 / type 0 | wait for input | `point`, `pointStatus` |
| 100 / type 100 | delay | `limit` s |
| 800 / 801 | select user frame / tool | `coordID` / `toolID` |
| 60000 | end | |

Speed is a percentage of `DEF_SPD_MMS` (1000 mm/s) at 0.1 % resolution. `smooth` is the zone value truncated and capped at 9.
The zip also holds `.fnc`, `.variables`, `.timers`, `.counters` and `.palletStyle`. The field-by-field rules are at the top of `posts/borunte_act.py`.

## Project layout
```
YoRA Slicer.bat        launcher
yora/                  engine + server
  mesh.py                STL / OBJ / 3MF loading, placement
  slicer.py              layers, walls, infill, brim, spiral vase
  planner.py             ordering, seams, travel/retract, E, stats, flow levels
  gcode.py               G-code writer and reader (Prusa/Orca/Bambu/Cura)
  robot.py               Borunte .act export
  cell.py                robot cells: save / load / export / import the whole robot setup
  kinematics.py          robot model (product of exponentials), closed-form IK, import from a .robot file
  robot_file.py          .robot robot model reader: DHM kinematics, limits, 3D link meshes
  robotpath.py           joint solving, free Z rotation, config lock, reach/limit/singularity/collision checks, simulation track, jog IK
  settings.py            settings spec (drives the GUI form), profiles
  server.py              local HTTP API (127.0.0.1 only)
web/                     GUI (index.html, app.js, material.js = material card and editor, tokens.css = all colours, clay.css = Clay style, robot.js = robot 3D view / jog / simulation, post.js = post-processor editor, style.css)
posts/Borunte_FGF.py     Borunte post: Extruder(E) -> Borunte DO actions
posts/borunte_act.py     own Borunte .act writer (byte-exact, linear time)
posts/kuka_krl.py        KUKA KRL writer (KR C4: .src / .dat)
yora/post.py           post-processors: languages, events, export, preview (profiles/posts/ for yours)
web/classic/             the previous interface, unchanged (served at /classic/)
profiles/                saved profiles (JSON)
samples/                 vase, bracket, tube STLs; square.gcode
tools/                   make_samples.py, gcode_replay.py (CLI)
tests/                   unit + end-to-end tests
out/                     exported files
```

Tests (126):
```bash
python -m unittest discover -s tests -v
```
The run takes about a minute and needs nothing besides the requirements.

## Check before the first real print
- **Wiring:** match `Extruder enable DO` and `Flow level DOs` to how your extruder controller is wired, and test the outputs with the robot in T1 before printing.
- **Frames:** teach the bed user frame and nozzle tool on the controller with the numbers set under *Robot & bed*.
- **`smooth`:** what the Borunte smooth field (Corner blending) does on your controller hasn't been verified. Start with 0.
- **Program size:** large jobs make tens of thousands of instructions. Check the controller's limit, and raise *Path simplify tolerance* if needed.
- **No analog flow:** the `.act` writer ignores `setAO`; the analog action code is unknown.
- **Not implemented yet:** supports, bridges, variable layer height, minimum layer time, multi-object plates, non-planar paths.
