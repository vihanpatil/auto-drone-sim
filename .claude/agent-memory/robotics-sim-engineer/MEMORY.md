# robotics-sim-engineer memory index

- [Toolchain version pins](toolchain_version_pins.md) — ADR-004 confirmed (Humble+Harmonic); exact component branches + why ArduPilot pins to a master SHA, not a stable tag.
- [macOS/arm64 bringup gotchas](macos_arm64_bringup_gotchas.md) — no GPU passthrough, arm64 package risk, Docker networking, X11 flakiness, bind-mount slowness.
- [Bringup file layout](bringup_file_layout.md) — where WEEK1_BRINGUP.md / sim/docker/Dockerfile / scripts live, and the devops handoff boundary.
- [User context](user_context.md) — solo dev, Apple Silicon Mac, 7-8 week deadline, container-first by default.
- [Spike clip generator](spike_clip_generator.md) — sim/spike/ layout, schema decisions, camera-footprint-dwell waypoint gotcha, no-numpy dev env note.
- [Farm world layout](farm_world_layout.md) — sim/worlds/farmguard_field.sdf generator pattern, tree-height-vs-mission-altitude design decision, SDF comment/actor gotchas.
- [ADR-007 NDVI sensor mount](adr007_ndvi_sensor_mount.md) — thermal-sensor SDF mechanics (per-visual plugin, L16 packing formula), nested-model+fixed-joint attachment to iris_with_gimbal, Gate 0/1/2 structure.
