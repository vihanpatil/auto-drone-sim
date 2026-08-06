---
name: adr007-ndvi-sensor-mount
description: How the ADR-007 dual-band NDVI sensor mount (RGB + Gazebo thermal-as-synthetic-NIR) was built into farmguard_field.sdf -- the exact SDF mechanics (thermal plugin syntax, temperature packing formula, the nested-model+fixed-joint attachment to iris_with_gimbal), the Gate 0/1/2 structure, and what's still genuinely unverified.
metadata:
  type: project
---

Built 2026-08-05 (Weeks 5-6) per ADR-007 (`docs/DECISIONS.md`, ACCEPTED, external-review-passed).
Source: `config/ndvi_camera.json` (single source of truth for all numbers below) +
`scripts/gen_farm_world.py` (generator) + `sim/worlds/farmguard_field.sdf` (output, 20 models incl.
the sensor mount, 40 per-visual `<temperature>` plugins). Gate runbook: `docs/WEEK5_VALIDATION.md`.
Related: [[farm_world_layout]] (the base world this extends), [[toolchain_version_pins]].

**Everything below was verified against the PINNED-BRANCH SOURCE this session (gz-sim8 == Harmonic,
ros_gz `ros2` branch, ArduPilot/ardupilot_gazebo `main`) by fetching raw GitHub content — not from
training-data memory.** That's a meaningfully stronger claim than "I recall Gazebo docs say X," and
is exactly how to re-verify if these facts seem to have drifted by the time you read this (branch
names in `CLAUDE.md`'s "Pinned versions" section, re-fetch the same files at those refs).

**Thermal sensor mechanics (per-visual, not per-model):**
- Per-visual temperature is a **plugin inside `<visual>`**, not a bare `<temperature>` tag:
  `<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>285.0</temperature></plugin>`.
  Confirmed against `gz-sim8`'s own `examples/worlds/thermal_camera.sdf`. Miss this on any visual
  and it silently renders ambient temperature (flat/meaningless — the exact failure mode ADR-007's
  review named). 40 visuals in this world need it: 1 ground + 18×2 tree trunk/canopy + 3 birds.
- **L16 raw pixel packing is `raw_uint16 = round(temperature_K / resolution_K_per_count)` —
  NOT offset by min_temp.** Verified against `gz-sensors8/src/ThermalCameraSensor.cc` (resolution
  default 0.01K/10mK) and `gz-rendering8`'s `ogre2/src/Ogre2ThermalCamera.cc`
  (`color = (temp / this->resolution) / ...`). The camera-level `min_temp`/`max_temp`
  (set via a SEPARATE plugin, `gz-sim-thermal-sensor-system`/`gz::sim::systems::ThermalSensor`, on
  the `<sensor>` itself) mainly clamp the visible/display range — they are the calibration span
  used to derive ρ_nir = (T−min_temp)/(max_temp−min_temp), not an offset in the packing formula.
  Easy to get backwards; if a future NDVI decode looks wrong by a constant factor, check this first.
- `ros_gz_bridge` converts `gz.msgs.PixelFormatType::L_INT16` → ROS `mono16` directly (confirmed
  from `ros_gz`'s own `src/convert/sensor_msgs.cpp` on the `ros2` branch) — no cv_bridge needed,
  `np.frombuffer(bytes(msg.data), dtype="<u2")` decodes it (little-endian, `is_bigendian=false`).
- `<camera_info_topic>` must be a child of `<camera>`, NOT a sibling of `<sensor><topic>` — a
  sdformat `camera.sdf` spec-file gotcha (`sdf/1.9/camera.sdf` on the `sdf14` branch), easy to place
  wrong by analogy with `<topic>`.

**Attaching a NEW sensor mount to an externally-vendored vehicle model, without forking it:** the
project already `<include>`s `model://iris_with_gimbal` (ArduPilot/ardupilot_gazebo, pinned SHA) —
its `gimbal_small_3d` sub-model is a real 3-axis RC-controlled gimbal, NOT what ADR-007 wants (needs
a RIGID nadir mount). The mechanism used: wrap the include in a NEW outer `<model>`
(`iris_with_gimbal_ndvi`) that adds a sibling `<link>` (the sensor mount, sensors only, no
visual/collision needed — a link just needs `<inertial>`) plus a `<joint type="fixed">` whose
`<parent>` is the SCOPED name reaching into the nested include:
`iris_with_gimbal::iris_with_standoffs::base_link`. This is the SAME nested-model-composition
pattern `iris_with_gimbal/model.sdf` itself already uses one level shallower (its own
`gimbal_joint`'s parent is `iris_with_standoffs::base_link`) — traced from the actual pinned-branch
file, not guessed. **This exact scoped name is NOT run/confirmed live yet** — it's the one thing in
this build riskier than the review's own named kill-switch (Gate 0/thermal loading). If `gz sim`
can't resolve the parent link on this joint, the documented fallback is dropping the
`iris_with_gimbal::` prefix (`iris_with_standoffs::base_link`) in `config/ndvi_camera.json`'s
`mount.parent_link_scoped_from_wrapper`, regenerate, retry. See `docs/WEEK5_VALIDATION.md` Gate 0's
troubleshooting section for the full writeup.
- A `<joint>` as a **direct child of `<world>`** (not nested in a `<model>`) is technically SDF-legal
  since 1.8 but explicitly **not supported by Gazebo or any other known software**
  (`gazebosim/sdformat` issue #1115) — don't reach for it even though it looks like the obvious
  simpler answer; the nested-model+fixed-joint approach above is the real supported pattern.
- The alternative considered and rejected: `gz-sim-detachable-joint-system` (a C++ plugin, supports
  permanent-attachment use even without ever publishing to its detach topic) — works, but its
  entity-resolution logic (`DetachableJoint::GetChildModelAndLinkEntities`, traced from
  `gz-sim8/src/systems/detachable_joint/DetachableJoint.cc`) requires the child LINK to be a
  **direct** child of the resolved child MODEL entity — meaning you'd have to point `child_model` at
  the deeply-nested `iris_with_gimbal::iris_with_standoffs` model directly (same depth problem, just
  phrased differently), with less-transparent runtime matching heuristics than a plain declarative
  `<joint>`. Went with the plain joint for "boring but explainable."

**Gate structure (mirrors Week 3's pattern, `docs/WEEK5_VALIDATION.md`):** Gate 0 = kill switch
(does `gz-sim-thermal-system` load on ogre2 — must run FIRST, before trusting anything else), Gate 1
= the four `/fg/sensor/*` topics present/correctly-encoded/rate (via `sim/bridge/fg_sensor_bridge.yaml`,
a ros_gz_bridge YAML config, GZ_TO_ROS-only, `ros_topic_name`==`gz_topic_name` since I made them
identical by construction), Gate 2 = `scripts/check_ndvi_bands.py`, the canopy/soil/bird pixel smoke
test — works on the **raw NIR band directly** (no NDVI-fusion math, that's flight-software's
downstream `ndvi_node`), reuses the ALREADY-PROVEN boustrophedon mission flight (not a new
gz-teleport procedure) on the geometric argument that lane x=15m's camera footprint fully covers
bird_0's track. This exactly matches `docs/ROADMAP.md`'s Week-5 "Committed ordering" table (items
1/2, written concurrently by `product-lead`/`tech-lead` in the same session) — good independent
confirmation the gate design lines up with what the rest of the team already expects.

**Temperature calibration chosen** (`config/ndvi_camera.json`): min_temp=270K, max_temp=330K,
resolution=0.01K/count. canopy ρ_nir=0.85 (321K), trunk ρ_nir=0.35 (291K, not one of the 3 gate
classes but every tree visual still needs SOME value), soil ρ_nir=0.20 (282K), bird ρ_nir=0.05
(273K) — deliberately monotonic, each step ≥9K/≥0.15ρ, so Gate 2's assertion threshold
(`--min-rho-gap 0.08`) has real slack against render-edge blending.

**Cross-agent observation (useful pattern, not a one-off):** this session's working tree had
several *other* subagents' uncommitted work already present mid-task (`devops-reliability-engineer`'s
Week-5 CI job, `product-lead`'s ROADMAP re-cut, `tech-lead`'s ADR-007 memory) — confirmed by
`git status`/`git diff` mid-task, not assumed. `devops`'s `scripts/ci_sim_smoke.sh` already
referenced this exact build (`iris_with_gimbal_ndvi`, `fg_sensor_mount`) and flagged the real
consequence correctly: Gazebo renders every attached sensor unconditionally every frame regardless
of ROS subscribers, so their DDS-free CI smoke job now pays the ogre2 render cost too — they've
already scoped a fallback (a camera-free CI-only world copy) as a documented, NOT-YET-BUILT
cut-list item, correctly deferred until a real CI run shows it's needed. Don't build that fallback
speculatively if asked to touch CI world variants later — check `docs/WEEK5_CI_GAZEBO.md`'s cut-list
item 6 first, it's already scoped.
