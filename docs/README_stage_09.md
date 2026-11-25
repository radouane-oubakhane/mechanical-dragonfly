# Stage 09 — Facial sensors and head armor

Adds 74 closed meshes and nine local rigid frames to the existing head. The facial module has segmented titanium panels, three tinted auxiliary lenses (ocelli), two segmented antennae, paired mandibles, dark vent-like louver bars, copper sensor traces and fasteners. The louver bars are raised details, not through-holes. Mandibles are static, with named moving components and pivot frames for future kinematics.

`create_facial_armor(scale=1.0) -> Module` returns the geometry and anchors. The mount aligns to `head.facial_armor`; antenna bases are derived from the existing antenna frames rather than independently hard-coded. `build_stage_09(scale=1.0)` returns the module and cumulative scene. Nominal assembly: 1730 meshes, 85.405 × 51.05 × 30.525 mm. Previous source files are unchanged.

Run `python main.py --stage 9` or `python run_tests.py` from the project root. Models are in `models/`. The tests cover mesh validity, previous geometry, rigid frames, bounding-envelope separation from both eyes, proportional scaling and GLB round trips. These are static visual concepts, not functional sensors or validated jaw mechanisms. The clearance test does not cover every face/head intersection or internal joint movement. GLB alpha blending approximates tinted glass.

Next: ten articulated abdominal skeletal segments using the existing chassis abdomen anchor.
