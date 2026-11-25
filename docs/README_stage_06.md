# Mechanical Dragonfly — Stage 06/15

Cumulative procedural Python project. Geometry is generated with Trimesh; Blender is not required. All prior component source files are unchanged. Tests were relocated and their model paths adjusted. Coordinates: meters internally, +X head, +Y left, +Z up. Concept geometry, not a fabrication or flight-ready design.

## Run

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python main.py --stage 6
python run_tests.py
```

Optional interactive preview: `pip install 'pyglet<2'` then `python main.py --stage 6 --show` (requires a graphical environment). Export a different size with `--scale 2`. Stage 01–06 may be selected with `--stage`.

## Layout

- `components/`: original standalone stage generators, one module per stage. Use root `main.py` to export assembled stages to `models/`.
- `models/`: all standalone and cumulative GLB exports through stage 06.
- `tests/`: available regression tests for stages 03–06.
- `docs/`: stage documentation and roadmap.
- `requirements.txt`: pinned dependencies.

## Stage 06

Four wing root assemblies each contain 30 meshes, including fixed clevis, bushings, pivot pin, moving hub, socket jaws, torsion spring, encoder and stop blocks. All 428 earlier meshes are preserved; total 548. Wing anchors are at (±13, ±25.525, 17.7) mm; positive X local points outboard, local Y is longitudinal flap axis. Each hinge has `mount`, `wing`, `flap_pivot`, `encoder`, `spring` frames. Fixed/rotor geometry is named separately to support later kinematics. Spring and encoder are visual representations, stop limits and internal collisions not simulated. No claim of functional force transmission or validated motion.

The current API uses per-stage functions and explicit anchor composition. The full Assembly class and animation system remain future work.
