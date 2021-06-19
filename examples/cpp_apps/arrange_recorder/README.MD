# Arrange Recorder

## Overview

This app lets a user rearrange objects in a scene.

Coming soon: a session is recorded to disk as a sequence of moves, where each move includes a teleport/move of a single object, followed by a sequence of keyframes to capture additional dynamics as the scene comes to rest.

## Building and Running

Note: Arrange Recorder is not currently organized as its own executable. For now, it is implemented inside the Habitat viewer.
1. Start by building the Habitat viewer [from source](https://github.com/facebookresearch/habitat-sim/blob/master/BUILD_FROM_SOURCE.md) (do *not* install via Conda or Docker). You should build with the following flags: `build.sh --bullet`
1. Set your working directory: `cd examples/cpp_apps/arrange_recorder/`
1. Download [data_v1.zip](https://drive.google.com/file/d/1SE7TnLVSIemwREHbvQ8M1a5kWamANWMU/view?usp=sharing) and extract into the existing `arrange_recorder/data` folder: `unzip data_v1.zip -d data`
1. `chmod +x arrange_recorder.sh` and run `arrange_recorder.sh`

## Mouse and Keyboard Controls

First, watch the video here to understand the expected usage:
https://cvmlp.slack.com/archives/C022Y1UKLJD/p1624041660010100

Basic controls:
- Mouse-over a small object and notice a red circle; left-click to grab and hold the object.
- Mouse-over a door or drawer and notice an orange circle; left-click to open/close.

While holding an object:
- Hit `F` to rotate the object.
- Left-click to place the object.

Movement and camera controls:
- Right-click and drag to look around. You can also look around with the arrow keys.
- Use `Z` and `X` to adjust your height up and down.
- Use `WASD` to move around.

## Customizing the Scene Setup

1. Open `data/lighthouse_kitchen_dataset/scenes/scene0.physics_keyframe.json` in any text editor.
2. Edit translations or quaternion rotations, save, and hit `T` in the app to hot-reload changes.
3. Alternately, rearrange objects in the app and hit `R` to overwrite `scene0.physics_keyframe.json`.
4. To add/remove rigid objects, edit `data/lighthouse_kitchen_dataset/scenes/scene0.scene_instance.json` and re-start the app (hot-reloading isn't supported on this file).
5. To add/remove URDF objects, directly edit `src/utils/viewer/viewer.cpp` (look for existing references to urdf files). Then, rebuild. This is a temporary workaround as we work to improve URDF loading.

## Recording and saving rearrangement episodes

Coming soon!

## Lighthouse Assets

The current demo uses a small subset of the Lighthouse assets. To learn more:
- https://fb.workplace.com/notes/221595716185290
- https://drive.google.com/drive/u/1/folders/1EkmA5aBX5j081q9FiZr1t-9vS7cYwPmK
