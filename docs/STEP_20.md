# Step 20 — Wing thin-film iridescence

This stage enriches the Stage 19 Balanced static and animated GLBs without rebuilding the procedural geometry. All four wing materials and 1,144 panel instances are upgraded. The original 504,756 triangle instances and all 16 animation channels remain unchanged. Original buffer bytes are preserved as a prefix; new maps and UV, normal, tangent and indexed position attributes are appended only for wing primitives. Triangle corners are exactly equal before and after.

## Optical parameters

| Finish | Transmission | Iridescence strength | Coating thickness bounds (nm) |
|---|---:|---:|---:|
| Neutral | 0.97 | 0.30 | 105–205 |
| Cool | 0.96 | 0.38 | 125–235 |
| Teal | 0.96 | 0.38 | 145–255 |
| Violet | 0.95 | 0.42 | 165–280 |

All use substrate IOR 1.46, coating IOR 1.38, nonmetallic thin-walled transmission, and spatially varying roughness (0.075–0.20 before quantization). The 25–45 µm structural membrane thickness is unchanged; it is not the nanometer-scale coating thickness. No volume extension is applied to wing film. The values are artistic assumptions, not measured material properties.

Three deterministic 512×512 periodic PNG maps are generated in `textures/wings`: `wing_thickness.png` (linear green channel), `wing_metalrough.png` (green roughness, blue metallic = 0), and `wing_normal.png` (tangent-space RGB, normal scale 0.25). An 18 mm texture repeat in local geometry coordinates uses the previous planar chart function. Thickness variations drive the glTF `KHR_materials_iridescence` extension; no rainbow is baked into base color. The two other wing extensions are `KHR_materials_ior` and `KHR_materials_transmission`.

## Usage

```bash
python materials/step_20_wing_iridescence.py
python materials/step_20_wing_iridescence.py --animated
python materials/step_20_wing_iridescence.py --size 1024
python materials/step_20_wing_iridescence.py --no-coatings
python tests/test_step_20.py
```

Default outputs: `models/dragonfly_static_balanced_iridescent.glb` and `models/dragonfly_animated_balanced_iridescent.glb`. `--no-coatings` disables interference on wings only, not on Stage 19 lenses. Stage 19 retains the previous alpha-blended wing appearance. Stage 18 is the fallback for viewers lacking transmission support on all optics. No new dependencies.

## Limits

A renderer supporting the optical extensions is needed to see the intended effects. Trimesh's basic viewer may render transmission surfaces opaque. Visual rendering, spectral accuracy, real-world optical properties, aerodynamic performance and GPU benchmarks have not been validated. Flat chart seams can be visible on thin edges. The full earlier regression suite was not rerun; Stage 20 tests verify preservation of prior geometry, material definitions, embedded maps and animation. This is not a full spectral interference simulation. Step 21 will address the rendering engine.
