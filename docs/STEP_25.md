# Step 25 — Head articulation

The Stage 24 source is immutable. The default output `models/dragonfly_head_animated.glb` replaces two rotation tracks per clip, preserving 54 other tracks, geometry, materials and texture data exactly. All angles are degrees, time seconds, positions metres in body coordinates (+X forward, +Y left, +Z up). Positive pitch elevates the virtual forward gaze; the compound eyes are not a single forward-facing camera.

## Run

From the project root in the existing virtual environment:

```sh
python simulation/step_25_head_articulation.py
python simulation/step_25_head_articulation.py --mode track --target 0.25 0.04 0.02 --output models/head_target.glb
python simulation/step_25_head_articulation.py --mode idle --output models/head_idle.glb
python simulation/preview_step_25.py --frames 12
python tests/test_step_25.py
```

Open the output in an animation-capable glTF viewer. By default `hover` scans, `forward` illustrates attitude compensation, and `turn_left` tracks a moving virtual point. `--mode scan|track|idle|stabilize` overrides all three clips. A custom fixed target requires `track` mode. The two-second clips are periodic; each head track uses 241 keys.

## API

- `HeadRig.from_document(doc)` extracts the two pivot positions and checks the longitudinal offset and hierarchy.
- `aim_direction(direction, limits)` and `aim_target(target, rig, limits)` return bounded yaw and elevation in degrees. Target tracking accounts for the 3.9 mm offset between pivots. Unreachable targets saturate; exact aim is only guaranteed for reachable directions.
- `stabilize(world_direction, body_attitude, limits)` accepts a SciPy body-to-world Rotation and aims in the compensating body-frame direction. No roll correction or physical body motion is implemented.
- `HeadController.update(desired, dt)` is a stateful exponential response with hard slew limits for a future interactive application. Offline authored profiles use the analytical angle trajectories and explicitly check their sampled rates; they do not run the stateful filter.
- `head_rotations(pose)` returns two glTF xyzw quaternions.
- `export(source, destination, mode, target, limits)` creates a new GLB without overwriting its source.

Software angle limits: yaw ±25°, pitch ±18°. Rate limits: yaw 75°/s, pitch 60°/s. These are chosen for visual motion, not physically certified collision limits. The forward clip assumes a virtual oscillating body attitude; the body itself does not move. Sensor-based tracking, independent eye orientation, actual flight stabilization and collision testing are not implemented. No new dependencies.
