# Step 22 — Professional studio environment

Run from the project root (no new Python dependencies):

```bash
python rendering/step_22_studio.py --preset graphite --composition hero --width 1800 --height 1200 --zoom 1.1 --supersample 2 --output renders/dragonfly_studio_graphite.png
python rendering/step_22_studio.py --preset gallery --composition top --width 1600 --height 1100 --supersample 2 --output renders/dragonfly_studio_gallery.png
python rendering/step_22_studio.py --preset cinematic --composition portrait --width 1600 --height 1000 --supersample 2 --output renders/dragonfly_studio_cinematic.png
python tests/test_step_22.py
```

Presets graphite (neutral), gallery (light), cinematic (warm key and cool fill). Compositions hero, top and portrait (intentional crop). CLI overrides azimuth, elevation, target (meters), zoom, exposure, projection and supersampling. `--no-ground` switches to the gradient backdrop. Ground views require positive camera elevation. Linux EGL/OpenGL requirements are unchanged from Step 21.

`studio.py` defines deterministic studio configurations, generates a floating-point equirectangular HDR radiance map (512×256), and rasterizes a projected silhouette and near-floor contact mask from opaque scene triangles. The GPU uses the radiance map with approximate mip-level roughness filtering, three direct lights, and a ray-plane backdrop pass. Stage 21 remains intact; Step 22 uses a separate render entry point and shaders built on the same scene/camera/backend modules.

Limitations: HDR lighting is synthetic, not a captured HDRI, and roughness mip filtering is not physically convolved IBL. Ground shadows use blurred projected opaque silhouettes and a near-floor contact approximation, not ray tracing, self-shadowing or full ambient occlusion. Transmissive geometry does not cast modeled colored shadows. Transparent surfaces retain the sorted blending approximation from Step 21. Rendering tested using Mesa llvmpipe (CPU), not dedicated GPU. Full earlier regression suite not rerun. All three supplied images render the real procedural model, not generated illustrations.

Next: Step 23 — wing kinematics.
