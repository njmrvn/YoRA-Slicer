# YoRA Slicer

A slicer for large-format 3D printing with an industrial robot arm and a pellet extruder.

<img width="2555" height="1406" alt="image" src="https://github.com/user-attachments/assets/f5611b4c-29a4-4562-8899-1447d60a6b63" />
<img width="2550" height="1405" alt="image" src="https://github.com/user-attachments/assets/990cc2f3-4fd7-4411-9993-5bfbec0bcb4c" />

## How it works

1. **Setup**: your machine. Pick or save a robot cell (robot, tool, bed and controller) and check that the robot can reach the bed corners.
2. **Prepare**: add one or more parts, place them on the bed and choose the material and print settings.
3. **Preview**: slice and check the toolpath layer by layer, add surface patterns and look at overhangs.
4. **Robot check**: make sure the robot can reach every point within its limits without collisions, and watch it print in a 3D simulation.
5. **Export**: save the robot program (or G-code) and run it on your machine.

## Quick guide

Your first print, step by step:

1. **Set up the machine once.** In **Setup**, choose your robot, then your tool: its 3D model and the nozzle tip (TCP). Then the bed: its size and position. Under *Jog*, press **Go to bed corners**: green corners are reachable, red ones are not. Save it all as a robot cell so you only do this once.
2. **Add your parts.** In **Prepare**, open a STEP, STL, OBJ or 3MF file (or a sample), and add more parts if you like. Click a part to select it; double-click it to move or turn it with the mouse.
3. **Choose how it prints.** Pick a material, then set the layer height, walls, infill and seam. **Essentials** shows the main settings, **All** shows every setting.
4. **Slice.** Press **Slice**. In **Preview**, drag the layer slider to look inside, colour the toolpath by overhang to find steep walls, and try a surface pattern if you want texture.
5. **Check the robot.** In **Robot check**, press **Check robot path**. Fix anything marked red (reach, limits, collisions), then play the simulation.
6. **Export and print.** In **Export**, pick your robot's program format and save it. Copy the program to your controller and print.

Press **Save** (Ctrl+S) to keep everything as a `.yora` project, and **Ctrl+Z** to undo a change.

## Features

- Walls, infill, brim, raft and spiral vase
- STEP, STL, OBJ and 3MF import
- Several parts on the bed: add, duplicate and arrange them
- Project files (`.yora`): the parts, every setting and the files they use, in one file
- Angled layers for printing overhangs without supports
- Overhang view: colours steep walls and warns past the material's limit
- Seam control: pick a point or draw a line on the part
- Surface patterns: waves, spikes, bumps, noise or a picture on the outer wall, in up to four layers, on surfaces you paint or pick
- Even point spacing for smooth robot motion
- Robot reach, limit and collision checks with a 3D simulation
- Material library with heating zone temperatures
- Editable post-processor with start / layer / end events
- Undo / redo (Ctrl+Z / Ctrl+Y)
- Light, dark and colour themes

## Surface patterns

Add texture to a part without changing the model. After slicing, open the **Surface pattern** card in **Preview**, pick a pattern and move the sliders: the toolpath updates as you go.

- **Sine, zigzag, spikes**: waves along the wall. Shift each layer by half a wave for a woven lattice.
- **Bumps** and **noise**: rows of domes or a random relief.
- **Image**: wrap a PNG or JPG (a logo or a texture) around the part.
- **Paint**: brush the pattern onto just the areas you want, or keep it off them.
- **Fill**: click a surface to pick all of it at once. On a STEP part, one click picks one CAD face, and Alt+click also spreads over smooth edges. On STL, OBJ and 3MF parts it spreads until an edge bends more than the fill angle, so one click picks a flat side or the whole round body of a vase. Picked surfaces move with the part.
- **Skip slopes over**: keep the pattern off walls that lean more than a set angle, such as a shoulder or a chamfer.

<img src="docs/surface-pattern.png" width="640" alt="A vase with a woven sine pattern">

### Pattern layers

Put up to four patterns on one part, for example a sine on the body and a picture on one face. The chips at the top of the card (`1 · Sine`, `2 · Zigzag`, `+ Layer`) pick the layer that the card edits.

1. Set up the first pattern as usual.
2. Press **+ Layer**, choose its pattern, then click the surfaces where it goes (Fill is already on). Press **Done**.
3. Choose how it meets the layers below: **Cover** replaces them where this layer is, **Add** puts its relief on top. **Edge blend** fades between them at the edges of a picked surface.

Each layer can be hidden, duplicated or deleted, and keeps its own waves per lap.

<img src="docs/surface-layers.png" width="640" alt="A bracket with a sine pattern on every wall and a deep zigzag on its front face">

## Tool angles A, B, C

After **Check robot path**, the Check tab shows the tool angles at the point on screen. **A** turns the nozzle around the vertical axis, **B** tilts it (90 points straight down with KUKA's tool X out of the nozzle) and **C** rolls it. These are not joints: the robot works out A1 to A6 from them.

YoRA holds A steady and only changes it when a joint would pass its limit. To choose the angles yourself, tick **Set A, B, C by hand**. The robot moves as you drag, the whole path is checked again when you let go, and the program gets exactly what you typed.

The values hold for the whole print, so A can no longer change to get out of a joint limit, and they replace the nozzle tilt of angled layers. A B other than 90 tilts the nozzle and the bead, and the check does not tell you if the nozzle would rub the walls, so try it on a small print first.

## KUKA corner blending

In the KUKA post-processor, **Blending type** chooses how the robot rounds each point:

- `c_dis`: starts a set distance from the point, the **Corner blending** setting in mm. 0 is an exact stop at every point.
- `c_vel`: starts when the speed drops to the **C_VEL speed** percent. It ignores Corner blending.

## Getting started

Needs Windows and Python 3.10 or newer.

1. Download or clone this repository.
2. Double-click **`YoRA Slicer.bat`**. The first start installs what it needs (about a minute, needs internet).
3. Load a model, a sample or a `.yora` project and press **Slice**.

After updating YoRA, close its window and start `YoRA Slicer.bat` again.

## Build the .exe

`Build exe.bat` packs YoRA Slicer into one folder that runs on a Windows PC without Python: start `YoRA Slicer.exe` in it. To give it to someone, zip the folder (`%USERPROFILE%\.yora\dist\YoRA Slicer`).

It opens in its own Edge or Chrome window and stops when you close that window. It works offline. Your materials, posts, robots and tools are kept in `%USERPROFILE%\.yora\profiles`, not in the program folder, so they are not part of the zip. Windows may warn about a new unsigned program the first time.

## License

GNU AGPL-3.0-or-later, see [LICENSE](LICENSE).

STEP files are read with [occt-import-js](https://github.com/kovacsv/occt-import-js) and [Open CASCADE](https://dev.opencascade.org/), both under the LGPL 2.1. They are unchanged in `web/vendor/occt`, with their license texts.
