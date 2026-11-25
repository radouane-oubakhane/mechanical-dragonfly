# Step 23 — Wing kinematics

The advanced Stage 20 animated GLB is the input. No existing meshes, materials,
textures, rig nodes or original binary accessor data are edited. A separate GLB
contains three named clips (`hover`, `forward`, `turn_left`), each with 16 tracks:
four new wing rotations and the original twelve head/abdomen rotations.

## Controls

Each wing has independent frequency (Hz), flap amplitude, phase, feather-pitch
amplitude, flap bias, pitch bias and pitch phase (degrees). The flap axis is
body X, mirrored left/right; feather pitch rotates about the neutral span axis
before flap. Positive flap raises both wings; positive pitch raises the +X edge.
The ±35° flap and ±15° pitch bounds are software constraints only. Pitch is a
virtual axis rotating the wing and existing rotor together, not a manufactured
second hinge. The axes are defined at the existing root pivot. No dynamic
collision, force, transmission linkage or lift validation is performed.

| Profile | Frequency | Fore/hind phase lag | Flap amplitude | Pitch amplitude |
|---|---:|---:|---|---|
| hover | 3 Hz | -90° | 30°/28° | 10° |
| forward | 4 Hz | -60° | 25°/22° | 9° |
| turn_left | 3 Hz | -90° | left 19° / right 30° | left 8° / right 12° |

These are illustrative slow-motion profiles, not experimentally validated flight.
The default clip duration is two seconds, sampled at 120 Hz (241 keys per wing),
and starts at a profile-specific pose rather than the neutral pose. Timeline
validation rejects incomplete cycles and fewer than 12 samples per wing period.
For other durations choose whole periods of the original two-second body clip.
The 12 existing body tracks retain their original 60 Hz sampling for the default
length; custom longer lengths repeat them through quaternion interpolation.

## Commands

```bash
python simulation/step_23_wing_kinematics.py
python simulation/step_23_wing_kinematics.py --profile hover --frequency 30 --fps 480 --output models/dragonfly_hover_30hz.glb
python simulation/preview_step_23.py --frames 8
python tests/test_step_23.py
```

Use an animation-capable glTF viewer and select a named clip. The standard
Trimesh viewer does not play GLB animations. The included GIF and contact sheet
are actual OpenGL renders of eight sampled poses of one forewing cycle, shown
at a slowed display rate (100 ms per frame). The preview auto-fits the camera
per pose, and the body tracks do not loop over one short wing cycle, so it is
not a perfectly seamless cinematic loop. The two-second GLB clips do loop.

## Future work

Step 24: synchronized crank, gear and piston motion. Current wing motion is
kinematic only; no transmission synchronization, wing deformation, wing-body
collision test or aerodynamic flight simulation. The full previous regression
suite was not rerun in this stage.
