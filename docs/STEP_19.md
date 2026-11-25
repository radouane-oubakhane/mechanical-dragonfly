# Step 19 — Optical materials

Material-only upgrade of Step 18 Balanced GLBs. The original BIN chunk is byte-for-byte identical. Nine named optical materials are upgraded (six compound-eye shades, one auxiliary eye material and two inspection windows). Wing films are deliberately excluded until Step 20.

| Finish | IOR | Transmission | Roughness | Representative thickness | Absorption distance | Coating strength |
|---|---:|---:|---:|---:|---:|---:|
| Compound lens | 1.52 | 0.94 | 0.09 | 0.22 mm | 0.75 mm | 0.24 |
| Ocellus | 1.52 | 0.96 | 0.075 | 0.50 mm | 1.4 mm | 0.20 |
| Power inspection polymer | 1.49 | 0.97 | 0.12 | 0.36 mm | 4 mm | 0 |
| Thorax glazing | 1.52 | 0.98 | 0.10 | 0.40 mm | 6 mm | 0 |

Uses KHR_materials_ior, KHR_materials_transmission, KHR_materials_volume and KHR_materials_iridescence. Alpha mode is OPAQUE and metallic factor is zero for the dielectric model; transmission, not alpha blending, is responsible for seeing through it. IOR governs Fresnel behavior in supporting renderers. Volume thickness is a representative optical path, not measured lens geometry. Coating thickness bounds are in nanometers; without a thickness texture the glTF extension uses the maximum thickness. The six original shades are retained through derived absorption and surface tints. No texture additions or new geometry.

```bash
python materials/step_19_optics.py
python materials/step_19_optics.py --animated
python materials/step_19_optics.py --no-coatings
python tests/test_step_19.py
```

Prebuilt models: `models/dragonfly_static_balanced_optics.glb`, `models/dragonfly_animated_balanced_optics.glb`. Use a glTF PBR renderer supporting the extensions. Viewers that ignore transmission may show opaque glass; use the original Step 18 carbon GLBs for alpha-blended compatibility. The uncoated option removes only iridescence, not transmission. No physical spectral simulation, lens imaging, ray-traced rendering or visual verification performed. The source eye meshes remain faceted. Values are artistic dielectric approximations, not calibrated measurements. The full previous regression suite was not rerun; the Step 19 tests compare all source binary data and all non-target JSON fields.
