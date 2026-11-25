# Stage 13 — Four mechanical wing skeletons

Four parametric wing frameworks attach to the Stage 06 wing sockets using the existing chassis → transmission → actuator → hinge transformation chain. No prior source geometry was modified.

## Geometry and interface

- 307 meshes per wing: 8 continuous longitudinal veins, 286 crossveins, 4 root braces, 1 tongue, 2 composite cheeks, 4 fasteners, 1 index marker and 1 distal reinforcement.
- 286 polygonal cells per wing, 1144 total. Seeded slight irregularity in the cellular lattice is deterministic. Forewings are narrower and longer than hindwings. The surface has mild camber.
- Nominal 400 mm forewing tip span; actual mesh bounds 400.182 mm wide including vein thickness.
- `make_wing_plan(side, pair, scale)` returns a `WingPlan` containing nominal local mm vertices, ordered outline, cells, continuous longitudinal paths and crossvein edges. Convert to meters with `.001 * plan.scale`. Local X is outward, local Z dorsal. Local Y is mirrored by side. `create_wing_skeleton(plan)` returns a `Module` with `mount`, `membrane` and `tip` rigid anchors. `wing_placements(scale)` resolves all four mounts. `build_stage_13(scale)` assembles all wings.
- For Stage 14, generate membrane geometry from the exact same `WingPlan.points_mm` and `cells`. Use `membrane` anchor, identity relative to wing mount. Triangulate each cell consistently; right-side polygon winding is mirrored, so account for normals. No need to recompute the outline.

## Execution

```bash
python main.py --stage 13
PYTHONPATH=components python tests/test_step_13.py
```

Bundled models: `step_13_wing.glb`, `step_13_wings.glb`, `step_13_assembled.glb`.

## Limits

This is a static visual geometry concept. The venation is a stylized structured lattice, not a scan of a real dragonfly wing. The cells have no membranes yet. Material factors approximate finishes without weave textures. The test checks planar cell coverage, neutral fore/hind separation, rigid mounting, individual mesh validity, scale and export round trips. It does not certify joint motion, aerodynamic performance, mass, strength, self-intersection of swept tubes, or comprehensive collision clearance. The root fitting meets the existing socket face; a physically locked attachment is not validated. Earlier full regression suite was not rerun in this stage.
