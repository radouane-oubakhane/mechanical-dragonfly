# Project Dragonfly — Stage 04: Thorax transmission

Install Python 3.10+ and dependencies (tested with NumPy 2.3.5, Trimesh 4.11.1, Shapely 2.1.2):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements_stage_04.txt
python step_04_transmission.py
python test_step_03.py
python test_step_04.py
```

`build_stage_04(scale=1.0)` returns `(transmission: Module, assembled: trimesh.Scene)`.
The new module mounts at the existing chassis transmission anchor, on the dorsal side.
For each suffix `fore_left`, `fore_right`, `hind_left`, `hind_right`, it exposes
`actuator_<suffix>` (local +X points outward), `driver_shaft_<suffix>` and
`output_shaft_<suffix>` (local +Z is the shaft axis). All transforms are in meters.
For child placement, use chassis transmission anchor @ transmission socket @ inverse(child mount).

The GLBs are static concept geometry. Gears use simplified involute-inspired profiles with deliberate clearance; tests check sampled non-overlap, not torque transfer, contact stresses, or dynamic behavior. Bearings and fasteners are visual approximations, and mounting contacts intentionally touch or overlap. No actual motors, sensors, or animation are included yet. Anchors remain in Python, not GLB nodes.
