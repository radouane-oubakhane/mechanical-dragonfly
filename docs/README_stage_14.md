# Stage 14 — Transparent wing membranes

Four independently generated membrane modules attach to the Stage 13 wing skeletons without modifying earlier geometry. Each contains 286 closed triangular or quadrilateral film panels using the exact shared `WingPlan` cell lattice and local camber. Each panel is inset 0.6% towards its centroid to avoid coincident side walls; the existing veins cover the boundaries. Nominal vertical thickness tapers from 0.045 mm at the root to 0.025 mm at the tip. A deterministic four-color, low-opacity PBR palette uses glTF alpha blending. The tint is an approximation, not physical iridescence or refraction.

## Interfaces

- `membrane_thickness(span_fraction)`: nominal mm thickness at a normalized span position.
- `create_membrane_cell(points_mm, span_mm, scale=1.0)`: closed Trimesh in meters.
- `create_wing_membrane(plan)`: Module with `mount` and `tip` anchors.
- `build_stage_14(scale=1.0)`: membrane modules and cumulative scene.

## Reproduce

```bash
python main.py --stage 14
PYTHONPATH=components python tests/test_step_14.py
```

The component script exports all three Stage 14 reference models: `python components/step_14_wing_membranes.py`. Tests compare the exported Stage 13 and Stage 14 geometry and transforms, validate all 5,671 exported meshes, check each procedural membrane against the cell boundary and thickness, and verify GLB material metadata. The first fit check found a conservative edge displacement above the thinnest vein radius; the inset was reduced from 1.5% to 0.6%, and the corrected checks passed (maximum 0.05632 mm including half-thickness). No optical render, aerodynamic simulation, dynamic collision check or full prior regression rerun was performed for this stage.

## Output

4 wings, 1,144 new panels, 5,671 total meshes. Body length 255.7147 mm, wingspan 400.1821 mm, height 59.4453 mm. Transparency appearance is renderer-dependent. The panels are separate closed solids, not a simulated continuous physical skin. The mechanical framework already supplies the reinforced boundaries.
