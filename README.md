# Mechanical Dragonfly

> **Repository preparation:** Generated GLB models, renders, and telemetry are kept outside Git. Existing local files remain available. A fresh clone needs the required models before running the studio or cinematic renderer; see [asset setup](docs/ASSETS.md). The EGL renderer has only been validated with Mesa locally. Kaggle and NVIDIA GPU rendering remain untested.

This repository produces a biomimetic mechanical dragonfly from deterministic Python geometry, without requiring Blender. It includes materials, animation, approximate flight physics, a control studio, and cinematic export. It is a visual 3D engineering concept, not a manufacturing-ready or flight-tested robot.

## Quick start (Linux; native EGL renderer tested on Mesa)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

For the complete control studio, run `python app/server.py`. For cinematic video, install FFmpeg and run `python rendering/step_30_cinematic.py --help`. See [the final guide](docs/STEP_30.md).

The bundled final static model is `models/step_15_assembled.glb`. Open it with a glTF-compatible 3D viewer. To rebuild any stage use `python main.py --stage 1` through `--stage 15`. Use `--scale 2` to double the dimensions or `--output path.glb` to change export destination. For the optional local viewer install `pyglet<2` and run `python main.py --show` (requires a graphical desktop and OpenGL). The viewer is optional and was not tested in the headless build environment.

## Animation

`models/dragonfly_animated.glb` contains a two-second looping mechanical motion demonstration (four wing joints, two neck axes, ten abdominal joints; 16 rotation tracks). Play it in a GLB viewer supporting animations. Wings, optical head parts and armor follow their corresponding rigid joints. The abdomen is a nested chain. A compatible viewer must enable looping if desired.

```bash
python simulation/animation.py
# For a custom-sized generated source, pass the same scale to both scripts:
python main.py --stage 15 --scale 2 --output models/custom.glb
python simulation/animation.py --source models/custom.glb --scale 2 --output models/custom_animated.glb
```

Animation is a slow illustrative motion, not biological flapping speed. The legs, actuators and gear mechanisms remain static; spring and cable deformation, full joint limit checks and collision avoidance are not simulated. The default clip is sampled at 60 fps; no video render is included.

## Tests

```bash
python run_tests.py
python tests/test_step_15.py
```

Tests cover the existing geometric contracts and final exported meshes, and the animation structure. See `docs/VALIDATION.txt` for the actual test run. Earlier stages are retained byte-for-byte; no geometry was replaced to accommodate the armor.

## Layout

- `components/`: fifteen independent parametric construction modules.
- `tests/`: stage-specific regression tests.
- `models/`: individual component exports and cumulative GLBs for all 15 stages, plus animated GLB.
- `simulation/animation.py`: GLB hierarchy and rotation-track generator.
- `docs/`: roadmap, construction notes and validation results.
- `main.py`: one entry point for generation; `run_tests.py`: regression runner.

## Final specification

Body 255.715 mm; wingspan 400.182 mm; height 60.44 mm in neutral pose. 5,769 separate meshes, four wings, 1,144 membrane panels, 482 compound lenses, six legs and ten abdominal segments. Coordinate convention: +X forward, +Y left, +Z dorsal; geometry is exported in meters with the original Z-up coordinate convention (some viewers use Y-up). The assembly uses named local 4×4 anchor transforms. The final animated export includes a hierarchical motion rig; a general-purpose Assembly class is not implemented.

Materials are untextured PBR approximations; transparent surfaces use alpha blending. Lighting and photorealistic rendering depend on the viewer. No aerodynamic, structural, electronic, torque, manufacturing tolerance or comprehensive collision certification was performed. Individual meshes are not fused into a printable solid.

See [roadmap](docs/ROADMAP.md) and [final stage](docs/README_stage_15.md).

## Phase II — Step 16: optimized GLB exports

Use the existing pre-generated models (no full procedural rebuild needed):

```bash
python optimization/step_16_geometry.py --mode balanced
python optimization/step_16_geometry.py --mode performance --animated
python -c "import trimesh; trimesh.load('models/dragonfly_static_balanced.glb', force='scene').show()"
python tests/test_step_16.py
python optimization/benchmark.py
```

Open the animated GLBs in an animation-capable glTF viewer. Trimesh's viewer is for static inspection. See `docs/STEP_16.md` for mode trade-offs. Original source and models remain untouched; `requirements.txt` now explicitly includes SciPy and the optional viewer dependency used in Phase I.

## Phase II — Step 17: procedural metallic materials

Pre-generated static and animated Balanced GLBs with embedded textures:
`models/dragonfly_static_balanced_metals.glb` and `models/dragonfly_animated_balanced_metals.glb`.

```bash
python -m pip install -r requirements.txt
python materials/step_17_metals.py
python materials/step_17_metals.py --animated
python tests/test_step_17.py
```

Use an animation-capable glTF PBR viewer for best results; Trimesh is only for geometry inspection and may not show all material effects. For a compatibility export omit anisotropy with `--no-anisotropy`. Step 16 optimizer is for untextured input; optimize before enriching. See [STEP_17](docs/STEP_17.md) for technical details and limitations.

## Phase II — Step 18: Woven carbon composite

Generate metal + carbon textured models using `python materials/step_18_carbon.py` and `python materials/step_18_carbon.py --animated`. Pre-generated models are `models/dragonfly_static_balanced_carbon.glb` and `models/dragonfly_animated_balanced_carbon.glb`. For best appearance use a PBR-capable glTF viewer with optional clearcoat and anisotropy support. Run `python tests/test_step_18.py` to verify. See `docs/STEP_18.md`.

## Phase II — Step 19: Advanced optical materials

Run `python materials/step_19_optics.py` and `python materials/step_19_optics.py --animated` to create the dielectric optical versions. Pre-generated models: `models/dragonfly_static_balanced_optics.glb` and `models/dragonfly_animated_balanced_optics.glb`. The nine optical materials use IOR, transmission, volume absorption and subtle thin-film coatings. Wings are unchanged. Use a viewer supporting these glTF extensions; the Stage 18 models remain the alpha-blended fallback. Run `python tests/test_step_19.py`. See `docs/STEP_19.md` for the finish values and limitations.

## Phase II — Step 20: Wing iridescence

Run `python materials/step_20_wing_iridescence.py` (add `--animated` for the animated model). The pre-generated `models/dragonfly_static_balanced_iridescent.glb` and `models/dragonfly_animated_balanced_iridescent.glb` include the three new wing maps. Use a PBR viewer with iridescence and transmission support. `--no-coatings` disables wing coatings only; Stage 19 retains alpha-blended wings and Stage 18 retains alpha-blended optics. Run `python tests/test_step_20.py`. See `docs/STEP_20.md`.

## Phase II — Step 21: dedicated renderer (Linux)

Run `python rendering/step_21_renderer.py --supersample 2` to create `renders/dragonfly_stage21.png` directly from the Stage 20 GLB. For camera options run `python rendering/step_21_renderer.py --help`. See [Step 21](docs/STEP_21.md) for requirements, limitations and rendering examples. The native EGL backend was tested on Mesa software OpenGL, not hardware GPU. All prior geometry, models, textures, and source code remain available.

## Step 22 — Studio rendering

```bash
python rendering/step_22_studio.py --preset graphite --composition hero --supersample 2
python rendering/step_22_studio.py --preset cinematic --composition portrait --output renders/closeup.png
python tests/test_step_22.py
```

See `docs/STEP_22.md` for presets, camera options, limitations and the three bundled renders. No new Python dependencies.

## Step 23 — Independent wing kinematics

Run `python simulation/step_23_wing_kinematics.py` to generate `models/dragonfly_wing_kinematics.glb` with three selectable clips. Use `python simulation/preview_step_23.py --frames 8` for the rendered slow-motion GIF and contact sheet. Run `python tests/test_step_23.py`. See `docs/STEP_23.md` for parameter definitions, software limits and the distinction between visual animation and physical flight. No new dependencies.

## Step 24 — Synchronized transmission

Run `python simulation/step_24_transmission.py` to regenerate the textured animated GLB. Open `models/dragonfly_transmission_animated.glb` in an animation-capable glTF viewer. Run `python tests/test_step_24.py` and see `docs/STEP_24.md`. This is a phase-locked visual rig, not a rigid-body mechanical solution. No new dependencies.

## Step 25 — Head articulation

Run `python simulation/step_25_head_articulation.py` to generate the new head-motion GLB (already included). `python simulation/preview_step_25.py --frames 12` produces a preview, and `python tests/test_step_25.py` validates the controller and retained animation. See `docs/STEP_25.md` for target tracking and stabilization examples and limitations.

## Step 26 — Coordinated abdominal articulation

Run `python simulation/step_26_abdominal_articulation.py` to generate `models/dragonfly_abdomen_animated.glb` (included). `python simulation/preview_step_26.py --frames 12` renders a motion preview; `python tests/test_step_26.py` verifies the ten joint tracks and previous asset preservation. See `docs/STEP_26.md` for custom postures, the independent controller and limitations. No new dependencies.

## Step 27 — Six-leg inverse kinematics

Run `python simulation/step_27_leg_kinematics.py` to regenerate the complete leg rig, `python tests/test_step_27.py` to validate, and `python simulation/preview_step_27.py --frames 8` to render a preview. The pre-generated model is `models/dragonfly_legs_animated.glb`. See `docs/STEP_27.md` for controls and limitations. No new dependencies.


## Step 28 — Flight physics

`python simulation/step_28_flight.py` creates `models/dragonfly_flight.glb` and `telemetry/flight_*.csv`. Select `flight_hover` or `flight_forward` in a compatible GLB viewer. Run `python tests/test_step_28.py`. See `docs/STEP_28.md` for assumptions and limits.

## Step 29 — Control Studio

Run `python app/server.py` to open the local browser application. WebGL2 required; no CDN or extra packages. See `docs/STEP_29.md`.

## Step 30 — Cinematic production

Run `python rendering/step_30_cinematic.py --sequence showcase --width 640 --height 360 --fps 6 --duration 2 --titles` for a small preview. Use `--sequence turntable`, `assembly`, `macro`, `motion`, or `still` for individual outputs. Install FFmpeg (`sudo apt install ffmpeg`) for MP4 export. See [STEP_30](docs/STEP_30.md) and [final validation](docs/VALIDATION_STEP_30.txt). The complete project is a visual and simulation prototype, not a verified flying robot.
