# Stage 08 — Compound optical eyes

Each eye mounts on the head's existing outward-facing socket frame; no previous component source was changed. The +X local optical axis points outward. The 14.10 mm diameter flange fits the 6.15–7.10 mm head socket annulus, and the eye extends 6.57 mm from the socket face.

## Components and interfaces

- `facet_centers()` returns a 241×2 array of hex lattice centers in mm (pitch 0.78 mm).
- `optical_surface(uv, offset)` projects planar points onto a half ellipsoid; output is XYZ in mm.
- `create_facet(center, radius)` generates separate closed hexagonal bezel and convex lens meshes in mm.
- `create_compound_eye(side, scale=1.0)` returns a Module with 511 meshes and three local frames: `mount`, `optical_axis`, `data`.
- `build_stage_08(scale=1.0)` returns `(eyes, scene)`, with a dictionary of left/right modules and the cumulative scene. Total: 1,656 meshes.

The dome has 241 individual lenses and 241 titanium bezels per eye, a light-absorbing substrate, a retaining ring, a mounting flange, a sensor PCB, silicon sensor, twelve screws and twelve contacts. PBR tinted glass uses alpha blending and deterministic color variation, not physically simulated transmission or interference. The eye is static and the electronics are illustrative. Clearance is checked at head vertices in the optical mount footprint, not a full mesh collision analysis. The renderer's transparency ordering can affect appearance. No optical simulation or animation is included yet.

## Run

`python main.py --stage 8` rebuilds the cumulative GLB. `python run_tests.py` runs all available regressions. The next stage adds facial armor, auxiliary sensors and antennae using the existing head anchors.
