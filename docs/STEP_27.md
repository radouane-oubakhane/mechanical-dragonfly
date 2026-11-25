# Step 27 — Six-leg inverse kinematics

Run `python simulation/step_27_leg_kinematics.py` to export `models/dragonfly_legs_animated.glb`. The model retains all three previous clips and their 56 tracks; each receives 42 additional leg rotation tracks (seven joints per leg). No new dependencies. Open the GLB in an animation-capable PBR viewer. `python simulation/preview_step_27.py --frames 8` renders a GIF and contact sheet; `python tests/test_step_27.py` validates the new stage.

## Interface

`leg_geometry(leg)` returns the seven neutral pivot positions, foot anchor and signed axes in body coordinates, in metres. Leg identifiers are `1_left`, `1_right`, `2_left`, `2_right`, `3_left`, `3_right`. `forward_kinematics(leg, angles)` accepts seven radians and returns pivot positions, foot position and final xyzw quaternion. `inverse_kinematics(leg, target, initial=None, tolerance=2e-5)` returns angles, residual in metres and a reachable flag. Always inspect the flag. The IK uses an analytical yaw and planar two-link solution with a bounded numerical fallback; ankle counter-rotation maintains the foot pitch. Three distal axes remain at neutral in the authored profiles; they can be set explicitly through FK. The neutral model's near-extended legs limit the Cartesian workspace.

`foot_target(leg, phase, mode='walk', stride=.002, lift=.0007)` supplies periodic body-relative targets. The default alternating tripod groups are (front left, middle right, rear left) and their opposite. The stance uses 2 mm travel and 0.7 mm inward bias; the swing raises the foot by 0.7 mm. The initial proposed 4 mm stride was unreachable. The conservative parameters avoid that issue. `gait` validates every requested target and rejects unreachable paths.

```bash
python simulation/step_27_leg_kinematics.py --mode landing --output models/legs_landing.glb
python simulation/step_27_leg_kinematics.py --mode tuck --output models/legs_tuck.glb
python simulation/step_27_leg_kinematics.py --mode idle --output models/legs_idle.glb
```

The landing profile is a periodic extension/retraction demonstration, not ground-contact control. The tuck is a small retracted pose. The default walk is a body-relative gait layered on the previous flight clips; no body translation, foot contact, dynamic collision checking, grip control or force simulation is present. The feet are targets at the terminal tarsal anchors, not exact claw contacts. The continuous harness moves rigidly with the femur and may separate visually; the pistons and springs are carried by their original rigid segments rather than separately actuated. The re-rigging rebuilds UV charts from the unbatched source, so the original texture data is retained but individual UV coordinates may differ. No validated hardware design is implied.
