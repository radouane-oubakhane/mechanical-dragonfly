# Step 16 — Geometry optimization

The restricted GLB optimizer batches geometry in the coordinate system of its immediate parent, retaining the 16 original animated joint nodes, their hierarchy and exact keyframes. It deduplicates identical material parameters (preserving alternate names in material extras), shares identical geometry references in retained blend nodes, and packs used accessors into a new binary buffer. A source and output SHA-256 cache avoids repeated work. The source GLB is never overwritten.

| Mode | Static mesh nodes | Animated mesh nodes |
|---|---:|---:|
| Ultra | 5769 | 5769 |
| Balanced | 1709 | 1845 |
| Performance | 91 | 239 |

All modes preserve 504756 triangles. Both optimized modes contain 91 materials versus 93 in the original. Balanced preserves the separate alpha blend nodes, but actual transparency ordering is renderer dependent; Performance merges transparent panels by parent and material and can display sorting artifacts. Mesh-node count is a proxy for draw submissions, not a measured GPU draw-call count. Batching bakes transforms and increases file size because some original accessors are no longer shared. Batched parts are no longer individually selectable; the Ultra files retain full part names and editable topology. This version supports the original untextured position-only indexed triangle GLBs; it rejects textured and skinned models. Advanced textures require extending the optimizer. True geometric LOD/decimation, GPU instancing extensions, GPU FPS profiling and interactive viewer/render quality testing are deferred. No collision or dynamics work is added here.

Validation: `python tests/test_step_16.py` passed. It compares every triangle in parent space by material (absolute tolerance 2e-8 m), the rig hierarchy, all 16 animation tracks, alpha node counts, cache invalidation and byte-exact Ultra copies. The six outputs were loaded using Trimesh for the CPU load-and-bounds benchmark. No earlier procedural regression suite was rerun; all old component/test/model files are unchanged. Benchmark results are in BENCHMARK_STEP_16.json (two runs each, not GPU frame rates).
