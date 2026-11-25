# Dragonfly — Stage 05: Wing actuators

Requires Python 3.10+ and dependencies in requirements_stage_04.txt.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements_stage_04.txt
python step_05_actuators.py
python test_step_05.py
```

For optional interactive visualization install `pyglet<2` and call `build_stage_05()[1].show()` on a desktop with OpenGL.

`create_wing_actuator(side, pair, scale=1.0)` returns a Module with six local frames: mount, output, gear_input, electrical_positive, electrical_negative, rocker_pivot. Local +X points outward. The Stage 04 socket defines each placement; multiply chassis transmission frame by the actuator socket frame to obtain the actuator world transform. All internal units are meters. Stage 06 should attach a wing hinge to the output frame using the same matrix composition.

`build_stage_05(scale=1.0)` returns a dictionary of four modules and a cumulative scene. The standalone GLB contains the fore-left actuator in its local coordinate frame. The cumulative GLB contains all five stages. Anchors are Python matrices, not GLB nodes. Earlier source files are unchanged.

Static geometry validation: previous mesh and transform preservation, finite vertices, watertightness, winding, positive volumes, socket and shaft center alignment, positive AABB overlap check against prior geometry (boundary contact permitted), scale and invalid inputs, GLB round-trip counts and bounds. Not a collision sweep, kinematic simulation, motor design or validated flight mechanism. Electrical terminals are placeholders for later wiring. The gear/rocker linkage is a static concept, not a proven motion conversion design.
