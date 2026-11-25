# Step 26 — Coordinated abdominal articulation

The Stage 25 animated model is the source; the Stage 26 asset replaces only its ten abdominal rotation tracks. All prior models remain available. No new dependencies.

## Run

```bash
python simulation/step_26_abdominal_articulation.py
python simulation/step_26_abdominal_articulation.py --mode posture --vertical 12 --lateral -10 --output models/abdomen_posture.glb
python simulation/preview_step_26.py --frames 12
python tests/test_step_26.py
```

The bundled `models/dragonfly_abdomen_animated.glb` has three clips: hover (traveling wave), forward (upward biased balancing motion), turn_left (lateral biased motion). `--mode idle`, `--mode wave`, `--mode balance`, `--mode turn`, and `--mode posture` override this mapping. The posture angles are distributed sums of local hinge rotations, not exact end-effector orientation. Positive vertical raises the abdomen; positive lateral moves the rearward chain right. The model coordinates are +X toward head, +Y left, +Z up. Local hinge axes alternate Y/Z. The existing chain and armor hierarchy are unchanged.

`AbdomenController.update(desired, dt)` accepts ten independent degree values and a positive timestep in seconds. It clamps and smooths them, applying per-joint slew limits. `forward_kinematics(pose, offsets)` computes ten body-frame pivots and the nominal terminal attachment (not the distal end of the cerci). `rig_offsets(doc)` reads the offsets from the actual model. The controller and FK are reusable for later interactive control; exported profiles are analytically generated and checked against the same position/rate limits rather than filtered through the stateful controller.

Default limits, proximal to distal, are 3,3,4,4,5,5,6,6,7,7 degrees and 30,30,35,35,40,40,45,45,50,50 degrees/s. All clips retain 56 tracks (46 unchanged), with 241 samples per abdominal track. The preview is a four-frame sampled illustration, not a real-time recording.

Limitations: rigid-body kinematics only; no cable deformation, collision certification, joint torque, physical balance or flight dynamics. Angular limits are software assumptions. Combined rotations are noncommutative. The preexisting geometry and material buffers remain exact; animation data is appended. No hardware GPU performance claim. The previous full regression suite was not rerun.
