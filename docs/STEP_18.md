# Step 18 — Woven carbon composite

Adds two deterministic 2/2 twill finishes to the Step 17 metal-enriched GLBs. Four tows per axis, alternating warp/weft crossings, twelve modeled brightness bands per tow, periodic micro-noise and tangent-space normal relief. No geometry displacement. Eight explicitly named carbon materials are updated; generic black polymers, insulation, glass and metal are unchanged.

| Finish | Tile period | Nominal tow width | Clearcoat factor |
|---|---:|---:|---:|
| Structural twill | 2.4 mm | 0.6 mm | 0.42 |
| Fine twill | 1.6 mm | 0.4 mm | 0.62 |

Each finish has four 512x512 PNGs: sRGB base-color multiplier, linear metallic/roughness, tangent-space normal and anisotropy direction/strength. Original material tint and metallic factors are retained. Clearcoat approximates protective resin. The glTF extensions KHR_materials_anisotropy and KHR_materials_clearcoat are optional; base maps work independently. UV charts reuse Step 17's dominant-plane method and may produce seams. This is a visual approximation, not a physical composite laminate model. No photorealistic rendering or GPU benchmarking performed.

Run from project root:

```bash
python materials/step_18_carbon.py
python materials/step_18_carbon.py --animated
python tests/test_step_18.py
```

Use --size 1024 for higher resolution; --seed changes microvariation; --no-clearcoat and --no-anisotropy disable the corresponding new extensions. For other modes, first generate matching Step 17 metal models. No new dependencies. Output GLBs append data to source binary; original models remain intact. Both models retain 504,756 triangles; animated version retains all 16 tracks. The static and animated files contain 32 embedded maps each (24 metals plus 8 carbon). Static model: 1,709 mesh nodes, 8 recharted unique primitives; animated: 1,845 mesh nodes, 35 recharted unique primitives. Step 18 tests passed for both shipped models and static Trimesh reimport; full previous regression suite not rerun.
