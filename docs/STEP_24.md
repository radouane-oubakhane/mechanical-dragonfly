# Step 24 — Synchronized mechanical transmission

## Usage

From the project root, with its existing virtual environment activated:

```sh
python simulation/step_24_transmission.py
python tests/test_step_24.py
python simulation/preview_step_24.py --frames 8
```

The pre-generated `models/dragonfly_transmission_animated.glb` contains hover, forward and turn_left clips. Use an animation-capable GLB viewer; Trimesh does not play glTF animations. For custom frequencies, generate a Step 23 profile GLB first, then pass `--profiles path/to/custom.glb` to Step 24. The default nominal-scale source is the original unbatched animated model. Do not use a scaled model with these nominal pivot coordinates. No additional dependencies are needed.

## Architecture

The six moving groups per channel are driver gear, output gear plus crank, rocker, clevis, piston rod and crank-to-rocker link. The neutral transforms are reconstructed from Steps 01, 04 and 05 before batching. The existing material pipeline is reapplied in a temporary directory; it regenerates UVs, so texture placement on newly separated parts can differ. Each clip retains the 16 earlier tracks and adds 40 TRS channels. Four drive channels are linked to the same independent wing controls. A positive gear phase rotates the output gear by that phase, with the driver rotating by its negative. The rocker uses the same flap angle as the wing, about its own pivot. The piston rod and connecting rod are oriented and axially scaled to reach their moving endpoints; the clevis translates to the rocker endpoint. The fixed housings remain static. Geometry is unchanged at the neutral pose before batching; during animation rod length varies.

## Limits

The original crank, rocker and wing hinge are not a solved mechanical linkage. The visual rig is phase-locked, not a force or torque simulation. The rod and link stretch and rotate, rather than obeying rigid length and slider constraints. No mechanical stops, bearing ball motion, collision detection or physical gear contact have been simulated. The wing pitch axis is virtual, as in Stage 23. Do not interpret these animations as proof of manufacturability. The original source and prior exports remain available.
