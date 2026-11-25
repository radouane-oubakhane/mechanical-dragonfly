# Stage 07 — Head skeleton and neck

Adds an open titanium/carbon-composite cranial cage with a static two-axis neck, yaw and pitch pins/bushings, stylized servo housings, rib struts and bosses, two hollow optical socket rings and antenna/face interfaces. 86 meshes, 9 local anchor frames. All geometry uses meters; nominal design dimensions are authored in millimeters.

## API

`create_head(scale=1.0) -> Module` returns a local scene and anchors: mount, yaw_pivot, pitch_pivot, eye_left, eye_right, facial_armor, antenna_left, antenna_right, head_data. Local +X is the optical mounting normal of each eye socket, tilted forward from lateral by about 19 degrees. Local +X of each pivot frame is its rotation axis. The `fixed_`, `yaw_`, and `pitch_` prefixes identify prospective moving groups; no actual joint hierarchy or animation is implemented yet.

`build_stage_07(scale=1.0) -> tuple[Module, trimesh.Scene]` attaches the head to the existing chassis neck anchor. Prior component sources are unmodified.

## Run

`python main.py --stage 7` exports the complete assembly; `python components/step_07_head.py` regenerates the head and cumulative stage models; `python run_tests.py` runs the regression suite.

## Validation and limitations

634 total meshes, cumulative dimensions approximately 82.403 x 51.05 x 30.525 mm. Regression tests verify geometry retention, individual watertightness, winding, positive volumes, front-plane clearance, rail-pad contact, optical mounting orientations, scaling and GLB round trips. These are static concept meshes, not a manufacturing model: internal part contacts/overlaps, dynamic clearance, servo torque and electrical routing are not comprehensively validated. PBR materials are untextured. Anchors remain Python metadata rather than exported GLB nodes. The eye lenses and facial armor are deliberately deferred to Steps 08 and 09.
