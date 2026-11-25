# Stage 10 — Articulated abdominal spine

Ten parametric open-frame vertebrae, 165 mm combined pitch length. The body reaches 250.411 mm in the neutral pose. Each segment has eight local anchors: mount, next, armor, pivot, wire_in_left, wire_out_left, wire_in_right, wire_out_right. Local X points forward. The pivot frame local Z alternates between global Y and Z in the neutral pose. The first module includes a chassis adapter; the last includes a terminal flange. For Stage 11 use `segment_dimensions(index)` and `abdominal_placements(scale)` with each segment's `armor` anchor. The dimensions describe nominal interface radii, not an armor clearance envelope.

## Generate and validate

```bash
python main.py --stage 10
PYTHONPATH=components python tests/test_step_10.py
python run_tests.py
```

The stage adds 448 meshes, for a cumulative 2178. The complete standalone abdomen and the first vertebra are also exported. Geometry is static. Joint bores and axial fork/hub gaps are modeled, but dynamic articulation, internal collision checking, flexible cable deformation, actual electrical continuity, structural simulation and detailed textures are not implemented. The chassis adapter deliberately overlaps the prior mounting ring at its fasteners and bridges. Cable paths are visual approximations. The initial nine stages and their test sources and models are unchanged. Prior-stage regression results in VALIDATION.txt are retained from Stage 09; Stage 10 tests and CLI were run anew.
