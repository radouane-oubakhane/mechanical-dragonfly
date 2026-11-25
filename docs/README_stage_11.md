# Stage 11 — Abdominal armor and mechanical tail

Ten removable armor modules attach to each existing vertebra's `armor` anchor; the terminal tail attaches to the tenth vertebra's `next` anchor. No preceding component is edited. The shell mesh generator builds tapered annular sectors with paired inner/outer surfaces and boundary walls, including three true through-slots in each of the two lateral panels per segment.

Each segment has four panels, 16 studs and 16 heads, plus raised seven-segment cyan identification markers. Shell thickness is 0.42 mm with nominal radial clearance 0.35 mm and coverage from 16% to 84% of vertebral pitch. Tail includes a flange, hollow housing, sensor core/tip and two cerci with pivot pins and bearings. All dimensions scale uniformly.

## Rebuild

From project root, install requirements and run `python main.py --stage 11`. For separate component exports, run `python components/step_11_abdominal_armor.py`. For new validation, use `PYTHONPATH=components python tests/test_step_11.py`. Use `python run_tests.py` for all available tests.

## Results and limitations

479 new meshes; 2,657 total; nominal envelope 255.71473812 × 51.05 × 30.525 mm. Step 11 regression verifies previous geometry, watertightness, winding, positive volumes, shell topology, spine vertex clearance, exposed joint gaps, mount alignment, scaling and GLB round trips. The earlier full suite was not rerun. Static geometry only: no simulated articulation, comprehensive collisions, physical validation or detailed carbon weave textures. Panels are separated rather than overlapping, and segment markings are raised rather than engraved. Fasteners are simplified visual components, not validated threaded fasteners.
