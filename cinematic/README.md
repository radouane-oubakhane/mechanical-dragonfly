# The Last Flight — scenario add-on for Dragonfly V2.0

From the project root, activate the existing .venv. No new Python packages are needed beyond the V2.0 requirements, and FFmpeg/OpenGL must already be working.

Quick storyboard (12 rendered stills, 2 FPS, six seconds):

    python cinematic/generate.py --preview --output renders/last_flight_preview.mp4

Full 25-second sequence (600 frames):

    python cinematic/generate.py --width 1280 --height 720 --fps 24 --output renders/last_flight.mp4

Full HD: use --width 1920 --height 1080 --supersample 2. Render time increases substantially. Frames are retained beside the MP4 in a directory named <output stem>_frames. A repeat with --resume and otherwise identical options resumes from completed PNG files, or re-encodes an existing sequence. Without --resume, previous frame PNGs in that directory are cleared. FFmpeg uses -nostdin and CRF 17 by default. Edit scenarios/last_flight.json for event times, duration, height, travel distance, failed wing and lighting. This initial template requires the same eleven actions in the same order.

The existing articulated GLB is sampled, transformed and shaded with the studio renderer. The floor remains at z=0. Pose-dependent clearance prevents geometry passing through the floor; height is the lowest posed vertex's clearance. Body flight, falling, bounce and roll are authored animation curves, not a dynamic contact/rigid-body solution. The failed actuator stops, remaining motion decays, and the emissive status material and optical tint dim. No actual fracture, particles, sound or new photorealistic rendering backend is implemented. Cameras cut between shot groups. This update adds a scenario generator and changes three existing renderer files with backward-compatible optional inputs.

Tested: pose samples retain 504,756 triangles, expected ground clearance, shutdown power = 0, and twelve actual frames encoded into a six-second MP4. Not tested: full 600-frame video or full historical regression suite.
