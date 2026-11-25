# Project Dragonfly — Stage 03

Run in Python 3.10+ with NumPy and Trimesh:

```sh
python -m pip install numpy trimesh
python step_03_electronics.py
python test_step_03.py
```

`step_03_assembled.glb` is the cumulative model. `step_03_electronics.glb` uses the local PCB mount origin; `step_03_power_harness.glb` uses chassis coordinates. Prior stages are unchanged. All Python anchors are 4×4 local rigid transforms in meters. The board has power_positive, power_negative, head_data, abdomen_data, actuator_fore_bus, actuator_hind_bus and mount anchors. The harness has mount and four endpoint anchors. The board uses a nominal 35 × 9.5 × 0.65 mm substrate. The stage is a visual concept, not a manufacturable or electrically validated circuit; targeted geometric tests are not a comprehensive collision solver.
