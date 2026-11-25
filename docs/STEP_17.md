# Step 17 — Metallic material library

Eight deterministic, tiled 512×512 PBR finishes: brushed titanium, dark titanium, polished steel, hardened steel, copper, bronze, gold and tin. Each includes base-color modulation, packed roughness/metallic and tangent-space normal maps. Original color and metallic factors remain intact; roughness is supplied by the texture. Optional `KHR_materials_anisotropy` provides directional reflection in supporting renderers. The normal maps represent fine relief, not actual geometric scratches.

The input GLBs are the untextured Stage 16 Balanced static and animated models. For metal primitives, dominant signed planar UV charts use millimeter-scale tiles. Face corners are split at chart and hard-normal boundaries and exactly deduplicated within matching attributes. The indexed triangle corner sequence, transforms and rig remain identical. Nonmetal materials and primitives are unchanged. All 24 maps are embedded in each GLB and stored separately under `textures/metals/`.

Run `python materials/step_17_metals.py --help` to select input, output, texture size, seed, quality mode and anisotropy. Only the Balanced static and animated exports have been generated and validated for this stage. The optimizer must be applied before texturing, since Step 16 accepts position-only untextured GLBs.

## Validation and limitations

See STEP_17_VALIDATION.txt and JSON reports. Both GLBs retain 504756 triangles; animation retains 16 channels. All previous project source and model files are retained. Static GLB reimport was tested in Trimesh, but no rendered visual comparison or GPU benchmark was performed. The planar projection can introduce seams and brush direction discontinuities; flat face normals can produce faceting on curved parts. These are artistic approximations rather than measured real-material BRDFs. Full optical simulation, carbon weave, studio lighting and physical rendering are later stages. Larger files reflect explicit UVs, normals, tangents and retained original accessors; the lighter untextured models remain available. Earlier regression tests were not rerun.
