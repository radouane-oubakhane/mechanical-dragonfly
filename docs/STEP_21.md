# Step 21 — Offscreen PBR renderer

Native EGL/OpenGL 3.3 + GLSL backend (Linux). ModernGL was not available in the build environment and installing packages was blocked, so the backend is implemented with Python ctypes. This uses GPU acceleration if an EGL hardware driver is available; the delivered images were rendered with Mesa llvmpipe (CPU). No Blender, display server or additional Python dependency is needed. On Ubuntu, if EGL/GL are missing, install `sudo apt install libegl1 libgl1 libgl1-mesa-dri` or the appropriate GPU vendor driver.

```bash
python rendering/step_21_renderer.py --width 1800 --height 1200 --azimuth 12 --elevation 65 --zoom 1.13 --supersample 2
python rendering/step_21_renderer.py --width 1200 --height 900 --azimuth 40 --elevation 30 --zoom 4 --target 0.035 0 0.005 --supersample 2 --output renders/dragonfly_head_detail.png
python tests/test_step_21.py
```

The reader is specific to the project's embedded GLBs; it bakes scene transforms for static rendering and reads embedded textures directly. It handles matrix and TRS nodes, indexed triangles, UVs, normals and tangents. Animated GLBs may be loaded in their rest pose only. The source remains unchanged. The shader uses a GGX-style BRDF, three directional lights, analytic environment reflections, approximate anisotropy/clearcoat and RGB thin-film interference. It uses sorted alpha blending to approximate transmission (no true refraction, volume absorption or linear-HDR transparency compositing). It does not implement ray tracing, shadows, HDR image-based lighting, interactive controls or animation playback. Camera controls are CLI arguments. Supersampling is optional. Stage 22 will introduce improved studio lighting, environments and presentation.

The full model and head close-up are real rasterized outputs. Geometry is intentionally the unchanged procedural concept: it is faceted in places and is not a finished photorealistic production render. Stage 21 tests ran; earlier full regression suite was not rerun.
