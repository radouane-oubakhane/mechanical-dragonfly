# Generated assets and fresh-clone setup

This repository tracks source code, scenarios, documentation, and the small texture maps in `textures/`. It excludes generated files in `models/`, `renders/`, and `telemetry/`. Existing local copies are not deleted by `.gitignore`.

## Last Flight cinematic

The current `cinematic/generate_realistic.py` and `cinematic/generate.py` scripts use `models/dragonfly_legs_animated.glb`. Place that file at the exact path before rendering. Its current local size is about 71 MiB. The public repository does not yet provide a download URL; a release asset or Kaggle dataset can be published separately and linked here.

After obtaining the asset, verify it before a long render:

```bash
test -s models/dragonfly_legs_animated.glb
python cinematic/generate_realistic.py --preview --width 640 --height 360 --fps 12 --output renders/preview.mp4
```

FFmpeg and a working EGL/OpenGL environment are also required for MP4 output. A detected Kaggle GPU does not prove that EGL is using it; check the OpenGL renderer string before relying on GPU performance.

## Other entry points

`app/server.py` requires stage models plus `dragonfly_flight.glb`, `dragonfly_static_balanced_iridescent.glb`, and `dragonfly_static_performance.glb`. The studio renderer uses `dragonfly_static_balanced_iridescent.glb`; the Step 30 showcase also uses stage models and `dragonfly_legs_animated.glb`. These assets must be generated locally or distributed separately before those entry points work in a fresh clone. See the stage commands in the main README and docs.

## Publishing assets

Before publishing an asset bundle, record checksums and make the distribution URL explicit here. Keep generated videos and temporary frames outside the source repository. The project has no chosen public license yet; select one before public release.
