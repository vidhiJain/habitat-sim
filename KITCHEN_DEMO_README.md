# Kitchen Demo

![kitchen_sim](https://user-images.githubusercontent.com/6557808/124810163-da9d9600-df15-11eb-9deb-1341ddf293af.png)

## Overview

The Kitchen Demo is a prototype implemented as some new C++ classes plus some changes to viewer.cpp (our C++ interactive viewer). Much of the code here is hacky and not intended for long-term use.

See this Slack thread in CVMLP #humans-in-habitatvr for demo videos:
https://cvmlp.slack.com/archives/C01NZ9Z0VKL/p1620079431028700

## Building and Running

Starting by building the Habitat viewer from source (do *not* install via Conda or Docker):
https://github.com/facebookresearch/habitat-sim/blob/master/BUILD_FROM_SOURCE.md

Download the kitchen 3D assets (objects, stage, URDF files):
https://drive.google.com/file/d/1GsZiCErr4xrXzt4SsK78xDzoKtuT78dU/view?usp=sharing

Unzip and place the files at the following locations:
- `habitat-sim/data/objects`
- `habitat-sim/data/stages`
- `habitat-sim/data/URDF`

From the `habitat-sim` working directory, run the viewer executable with these command-line options:
`build/utils/viewer/viewer data/stages/apartment_0_f0_stage.glb --enable-physics --stage-requires-lighting --disable-navmesh`

## Mouse and Keyboard Controls

First, watch the videos here to understand the expected usage:
https://cvmlp.slack.com/archives/C01NZ9Z0VKL/p1620079431028700

Basic controls:
- Left-click and drag to look around. You can also look around with the arrow keys.
- Use Z and X to adjust your height up and down.
- Use WASD to move around.
- Right-click and hold to grab and hold an object such as the milk carton.

While holding an object:
- Use the normal movement keys to move around with the held object.
- Move the mouse cursor up/down to raise/lower the object.
- Move the mouse cursor left/right to rotate the object around the up axis.
- Hold spacebar while moving the cursor left/right to rotate the object around the forward axis. The rotation here is intended to support pouring.
- Beware that right-click will also grab and move large objects like the counters!

Right-click and hold to grab and hold the oven door. While held:
- Move away from the oven with WASD to open the door. Moving the cursor up/down also somewhat works.
- Beware this interaction with the oven door and articulated links in general is not very polished.

## Customizing the Scene

The stage is specified on the command line. The objects including the oven are hard-coded in src/esp/scripted/KitchenSetup.cpp.

## Lighthouse Assets

The current demo uses a small subset of the Lighthouse assets. To learn more:
https://fb.workplace.com/notes/221595716185290
https://drive.google.com/drive/u/1/folders/1EkmA5aBX5j081q9FiZr1t-9vS7cYwPmK
