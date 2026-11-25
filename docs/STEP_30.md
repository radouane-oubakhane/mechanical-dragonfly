# Step 30 — Cinematic production

The final stage adds an offline Python shot planner and MP4 exporter on top of the native Stage 22 studio renderer. It uses the actual bundled GLB models, not generated concept imagery. All 15 construction stages, advanced materials, motion clips, flight telemetry, and the control studio are retained.

## Ubuntu quick start

```bash
source .venv/bin/activate
python -m pip install -r requirements.txt
sudo apt install libegl1 libgl1 libgl1-mesa-dri ffmpeg
python app/server.py
```

## Film production

```bash
# Fast preview, every construction stage + orbit + macro + articulation
python rendering/step_30_cinematic.py --sequence showcase --width 640 --height 360 --fps 6 --duration 2 --titles
# Full quality turntable
python rendering/step_30_cinematic.py --sequence turntable --width 1920 --height 1080 --fps 30 --duration 8 --supersample 2 --ground
# All 15 cumulative stages, each held for the same number of frames
python rendering/step_30_cinematic.py --sequence assembly --fps 24 --duration 15 --titles
# Macro camera move
python rendering/step_30_cinematic.py --sequence macro --preset cinematic --duration 5
# Slow motion (one-quarter speed); uses existing fully articulated GLB clips
python rendering/step_30_cinematic.py --sequence motion --clip forward --motion-speed .25 --duration 8 --fps 30
# 4K still
python rendering/step_30_cinematic.py --sequence still --width 3840 --height 2160 --supersample 2 --output renders/dragonfly_4k.png
```

`--duration` is per shot in the showcase, and the assembly segment rounds up to an integer hold per stage (at least one frame). `--keep-frames DIRECTORY` retains individual PNGs. Each video includes a JSON report. Output MP4 is silent H.264/yuv420p with fast-start metadata. No external assets are required. Default output is under `renders/`. Use `--help` for all settings.

## Implementation decisions

- The camera and studio renderer gained optional fixed framing parameters; existing calls use their previous behavior. This is the only intentional modification to earlier code besides the README and regression runner if changed.
- The assembly film uses the original stage GLBs (simpler materials), the remaining shots use the advanced static or articulated model. Assembly stages appear by cuts, not an animated insertion of each part.
- The animation sampler bakes the existing body-relative motion clips into temporary GLBs. `--motion-speed .25` means one-quarter source speed. Flight trajectory clips remain in `models/dragonfly_flight.glb` and are accessible via the control studio; they are not part of this film sequence.
- Video is offline frame-by-frame rendering, not real-time capture. Floor shadows and glass are approximations, not ray-traced. Hardware GPU and live WebGL remain unverified in this environment. No physical manufacturing or flight certification is implied.

## Validation

```bash
python tests/test_step_30.py
python run_tests.py
```

The six new tests cover the shot planner, all 15 stage paths, fixed framing, motion sampling and H.264 encoding. See the final validation report for the actual checks performed and any limitations.
