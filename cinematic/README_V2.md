# Last Flight V2: collision and camera upgrade

Requires Mechanical Dragonfly Stage 30 and the original Last Flight update. Extract into the existing project root; keep `.venv`. Your existing scenario JSON is not overwritten. `rendering/step_22_studio.py` gains an optional floor shader argument, preserving the default shader and fixed floor support. The original `cinematic/generate.py` stays available.

```bash
source .venv/bin/activate
python cinematic/generate_realistic.py --preview
xdg-open renders/last_flight_v2_preview.mp4
python cinematic/test_physics_v2.py
```

Render a short continuous test of flight, before committing to an entire film:

```bash
python cinematic/generate_realistic.py --start 10 --end 12 --width 960 --height 540 --fps 60 --wing-rate 4 --motion-samples 2 --output renders/flight_test.mp4
```

Full film, high quality (expensive):

```bash
python cinematic/generate_realistic.py --width 1920 --height 1080 --fps 60 --supersample 2 --wing-rate 4 --motion-samples 3 --crf 15
```

Use `--resume` with identical settings. Each render has its own frame directory and manifest; preview and full movie have distinct default output names. `--physics-only` creates a CSV and JSON report without rendering. `--start` / `--end` select an excerpt. `--wing-rate` only changes the visual wing and drive playback speed, not the aerodynamic forces. `--motion-samples` averages multiple subframes in display-linear space, approximating shutter blur at added render cost. The crash preview is rendered at 24 fps, 480x270 with one sample and default wing playback.

## Physical scope

Powered flight and takeoff are authored continuous trajectories, not aerodynamically solved. At loss of lift (default 18s), linear and angular velocity are inherited from the trajectory; a 480 Hz numerical rigid-body solver applies gravity, quadratic drag, approximate torque-free angular dynamics, contact impulses, friction, restitution and rolling damping. The mass/inertia and friction/restitution are assumptions, not measured material properties. A convex hull of the neutral model is used for ground contact; external joints smoothly return to that pose before the crash. Collision impulses do not break or deform the geometry. Impact time emerges from the simulation, rather than the old scripted `impact` event. The default scene settles overturned. This is not a validated general rigid-body or aerodynamic solver. No mechanical damage, sparks, smoke or post-impact limb twitch are modeled. Lighting uses the existing HDR-based raster renderer and world-space procedural floor grain; it does not have path tracing, physically accurate self-shadows, refraction or global illumination. The model is still a visual concept.

Default physics: gravity 9.81 m/s², 35 g, friction .55, restitution .12, 480 Hz. Edit `Physics` in `cinematic/physics_v2.py` if needed. Use existing JSON to modify timeline/altitude/distance; `impact` remains a legacy timeline marker, not a forced event. Tests validate default scenario and simple contact/free-fall cases, not all possible user configurations.
