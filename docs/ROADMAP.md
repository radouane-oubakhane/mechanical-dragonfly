# Construction roadmap

- [x] 01 Chassis skeleton
- [x] 02 Power core
- [x] 03 Electronic nervous system
- [x] 04 Thorax transmission
- [x] 05 Wing actuators
- [x] 06 Wing root joints
- [x] 07 Head skeleton and neck
- [x] 08 Compound optical eyes
- [x] 09 Facial sensors and head armor
- [x] 10 Abdominal spine
- [x] 11 Abdominal armor and tail
- [x] 12 Six articulated legs
- [x] 13 Four wing skeletons
- [x] 14 Wing membranes
- [x] 15 Thorax armor and final integration

Retain all preceding stages, meshes, attachment frames, docs and tests. Add new component files rather than rewriting existing ones unless an explicitly documented interface fix is required. Update main.py and this roadmap at every stage; final release should include complete source, tests, exports, docs and a reproducible entry point. Kinematics/rendering and manufacturability are distinct from the static geometry build.

## Phase II
- [x] Step 16: geometry batching, cache, CPU benchmarks, rig preservation (true geometric LOD deferred).
- [ ] Step 17: advanced metallic materials.
