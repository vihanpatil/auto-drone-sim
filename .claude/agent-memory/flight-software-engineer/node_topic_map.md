---
name: node-topic-map
description: FieldGuard ROS 2 node/topic map, package layout, and locked AP_DDS /ap/* contract as of Weeks 3-4 (avoidance loop live on real stack) + Weeks 5-6 (NDVI nodes added, unit-tested only)
metadata:
  type: project
---

Current as of 2026-08-05 (Weeks 3-4 avoidance loop demonstrated live; Weeks 5-6 NDVI fusion/georef
built and unit-tested, render still pending a human Docker session). Supersedes the old Week-2-only
version of this memory (kept the still-true parts, corrected/extended the rest).

## Package layout (`src/fieldguard_planning/`)
Still **not a real colcon/ament package** (no `package.xml`/`setup.py`) as of Weeks 5-6 —
`docs/WEEK3_AVOIDANCE_DEMO.md` flags this as "a small follow-up." Everything runs via
`PYTHONPATH=src:$PYTHONPATH python3 -m fieldguard_planning.<node>` (note: **prepend**, don't
replace — a bare `PYTHONPATH=src` inside the container wipes out ROS 2's own Python path and gives
`ModuleNotFoundError: No module named 'rclpy'`). If this ever gets promoted to a colcon package,
that prepend gotcha becomes moot but is worth remembering for now.

Two dependency tiers inside the package (a project-blessed, documented split, not an accident):
- **stdlib-only**: `geofence.py`, `coverage.py`, `mission_waypoints.py`, `avoidance_types.py`,
  `avoidance_policy.py`, `avoidance_executor.py`, `ros2_adapter.py`'s pure `enu_to_geodetic`, and
  `ndvi_georef.py`'s single-point transform functions (`pixel_to_latlon`, `world_enu_to_pixel`,
  etc. — no numpy needed for one point/ray). Runs on a bare interpreter, zero installs.
- **numpy-dependent** (a deliberate, scoped exception, documented in each module's own docstring):
  `ndvi_fusion.py` (per-pixel image math) and `ndvi_georef.py`'s `NdviHeatmapGrid` (accumulates
  numpy NDVI arrays). Already-blessed dependency (`requirements-eval.txt`, `eval/baseline_ndvi.py`,
  `scripts/check_ndvi_bands.py` all use it) — just newly extended into `src/fieldguard_planning/`
  for this one image-processing slice. If this pattern grows, it's worth a DECISIONS.md entry
  formalizing the dependency boundary; flagged for tech-lead, not decided unilaterally.
- Every rclpy-touching file (`avoidance_node.py`, `ros2_adapter.py`'s `Ros2VehicleSink`,
  `ndvi_node.py`) imports rclpy **lazily inside `build_node()`/`main()`**, never at module top —
  this is what lets the whole test suite run without a sourced ROS 2 environment. Follow this
  pattern for any future node.

## Reactive-avoidance loop (Weeks 3-4) — CONFIRMED LIVE 2026-08-05
- Interface boundary: `avoidance_types.py` (`Detection`, `DroneState`, `AvoidanceManeuver`,
  `Decision` enum: PROCEED/DIVERT/HOLD).
- Decision policy (perception-ml-engineer's `avoidance_policy.py`) decides *when/where*; executor
  (`avoidance_executor.py`, mine) takes control, flies it, resumes, books coverage-debt.
- ADR-006 maneuver shape, CONFIRMED live: `AUTO -> GUIDED -> one 3D-vetted setpoint -> GUIDED ->
  AUTO`, `MIS_RESTART=0` makes AUTO resume the SAME next waypoint (verified: reached #3, took
  control heading to #4, resumed at #4, continued #5-#8, no restart at #1).
- Safety backstop: every DIVERT setpoint is re-vetted through `GeofenceMap.is_safe_3d` in the
  executor even though the policy already checked it. Reject -> HOLD, never fly an unvetted point.
- `AvoidanceExecutor.finalize()` builds the terminal coverage ledger from the ACTUAL flown path via
  `coverage.coverage_from_path` — every canonical grid cell is visited exactly once by
  construction, so a cell can never be silently absent from the ledger (worst case: explicit
  `debt`, the allowed ADR-002 v1 outcome).
- Live bringup order that matters (real gotcha, cost real debugging time): **start the micro-ROS
  agent BEFORE SITL.** AP_DDS pings the agent at startup; if the agent isn't listening on UDP 2019
  yet, SITL prints `AP: DDS: No ping response, exiting` and NO `/ap/*` data ever flows (the
  avoidance loop then sees a permanently frozen pose). Order: Gazebo -> micro-ROS agent -> SITL
  (`--enable-DDS`) -> the avoidance node.
- Demo: `python3 -m fieldguard_planning.avoidance_node --demo` injects a scripted bird at ENU
  (30,30,15) via `proximity_bird_source` (triggers within 10m, lingers 12s) — tightened from an
  earlier wider trigger radius specifically so it doesn't fire on adjacent lanes (< lane spacing).
  Full recipe: `docs/WEEK3_AVOIDANCE_DEMO.md`.
- `current_waypoint()` is DERIVED (nearest mission waypoint to current pose), not read from AP_DDS —
  no mission-current service exists at the pinned SHA (ADR-006 "why no waypoint-index juggling").
  Fine for resume bookkeeping since ArduPilot's own `MIS_RESTART=0` owns the actual resume.

## Locked AP_DDS `/ap/*` interface (ADR-005, CONFIRMED live — all 18 topics enumerated, matched exactly)
- `/ap/pose/filtered` (`geometry_msgs/PoseStamped`) — **frame_id says "base_link" but LIES**; the
  message *content* is world-ENU position relative to the EKF/home origin. Trust content, ignore
  frame_id, for this topic specifically. Same REP-105 mislabeling on `/ap/twist/filtered.linear`
  (world ENU) vs `.angular` (body-frame) — two frames under one message, don't cross them.
- `/ap/gps_global_origin/filtered` (`geographic_msgs/GeoPointStamped`) — WGS-84 EKF origin, the
  correct RUNTIME anchor for `/ap/pose/filtered`'s ENU frame (as opposed to
  `config/field_polygon.json`'s `home_lat`/`home_lon`, which is only the offline/test default and
  *should* match but isn't guaranteed to bit-for-bit).
- Command topics work OPPOSITE to telemetry: for `/ap/cmd_gps_pose` /
  `/ap/cmd_vel`, `frame_id` **is honored** as a real switch (`"map"` = world-ENU, transformed to NED
  internally; `"base_link"` = body-frame via `ahrs.body_to_earth()`). Command `"map"`, always —
  sending `base_link` by mistake flies a body-frame dodge.
- `/ap/cmd_gps_pose`/`/ap/cmd_vel` are ONLY honored in GUIDED + armed
  (`ready_for_external_control()`); ArduPilot silently drops them otherwise. This is why the
  executor must switch mode BEFORE sending a setpoint, every time.
- DDS_ENABLE is **not** on by default at the param-storage level even though the compiled default
  is `ENABLED_BY_DEFAULT=1` — the SITL instance's persisted `eeprom.bin` (named Docker volume) keeps
  whatever was saved the first time that param existed. Always load the explicit param file
  (`config/sitl_params/dds_udp.parm`), don't trust "the code says it defaults on."
- `ardupilot_msgs/msg/GlobalPosition` fields (verified against the pinned build): `header,
  coordinate_frame, type_mask, latitude, longitude, altitude, velocity, acceleration_or_force, yaw`.
  `ardupilot_msgs/srv/ModeSwitch.Request` has `mode` (uint8). ArduCopter mode numbers: AUTO=3,
  GUIDED=4.

## NDVI fusion + georef stitch (Weeks 5-6) — UNIT-TESTED ONLY, render pending Docker Gates 0-2
- ADR-007 locked contract: IN `/fg/sensor/rgb/image` (rgb8) + `/fg/sensor/nir/image` (mono16) +
  their camera_info; OUT `/fg/ndvi/image` (32FC1 ∈[-1,1], AUTHORITATIVE), `/fg/ndvi/camera_info`,
  `/fg/ndvi/preview` (rgb8, human-only, non-authoritative).
- Files: `ndvi_fusion.py` (pure fusion math + stale-pair guard, numpy), `ndvi_georef.py` (the
  pixel->lat/lon transform + `NdviHeatmapGrid` stitch accumulator, stdlib math for the transform
  itself), `ndvi_node.py` (thin rclpy adapter, `message_filters.ApproximateTimeSynchronizer`).
- Stale-pair guard (ADR-007 amendment): max stamp delta = `0.25 / update_rate_hz` (50ms at this
  project's configured 5Hz). Exceeding it DROPS the pair (never emits a mispaired NDVI) and
  increments a logged `dropped_pair_count` — same "instrument every event" discipline as the
  avoidance executor's `_log`.
- NDVI 0/0 guard: sentinel is `0.0` (neutral — a 0/0 pixel carries no vegetation signal either way),
  never a silent NaN; every occurrence is counted (`zero_denom_count`).
- Georef body-frame convention: FLU (X-forward,Y-left,Z-up), world ENU — matches
  `avoidance_node.py`'s existing yaw-extraction assumption, not a new/second convention. Mount
  extrinsic (ADR-007 nadir mount, quat_wxyz=(0,1,0,0)): camera<->body axis map is a diagonal
  `(+1,-1,-1)` sign flip (self-inverse, same tuple works both directions).
- The transform reuses `ros2_adapter.enu_to_geodetic` for the final ENU->lat/lon step rather than
  reimplementing it — one ENU<->geodetic transform for the whole project (mission planner +
  avoidance executor + georef stitch all share it).
- `NdviHeatmapGrid` reuses `coverage.py`'s canonical 720-cell grid (same `cell_id`s as the
  coverage-debt ledger) rather than inventing a second grid — a cell never imaged is `None` in
  `mean_grid()`, the NDVI-mapping analog of an explicit coverage-debt cell.
- **Nothing here has run against the real render yet.** `/fg/sensor/*` topics don't exist until a
  human runs Gates 0-2 of `docs/WEEK5_VALIDATION.md` (Docker + real Gazebo thermal-sensor render).
  All 39 new tests (`test_ndvi_fusion.py`, `test_ndvi_georef.py`) validate against synthetic/
  hand-computed fixtures only — same "built ahead of Docker validation" pattern as the avoidance
  policy/executor were before Week-3's Gate 2.
- Added **Gate 3** to `docs/WEEK5_VALIDATION.md`: re-fly the Week-3 avoidance demo against the new
  `iris_with_gimbal_ndvi` vehicle model (2 new cameras + 40 thermal plugins added render load) and
  confirm dodge->hold->resume still completes + RTF doesn't collapse — a regression check, not a
  new ADR-007 claim.

## Field/geofence/mission constants (still true, Week 2 origin)
- Field: `config/field_polygon.json` — 75m(E) x 60m(N) rectangle, home = (-35.363262, 149.165237,
  584m elev), mission altitude 15m, ground plane assumed flat at local-ENU z=0 (also the georef
  stitch's flat-field assumption).
- Trees: `config/static_obstacles.json` — 18 trees, 3 rows at x=15,40,65 (y=5..55, spacing 10m).
  `obstacle_radius_m=2.0` is the geofence field (not `canopy_radius_m=1.3`). Tree row 0 (x=15) sits
  exactly on mission lane x=15 — -2.0m XY clearance, safe only because of the 11.5m vertical margin
  (canopy top 3.5m vs cruise 15m). This is the row already primed for forcing a real XY-plane dodge
  scenario if one is ever needed (rows 1/2 are offset from their lanes and aren't).

## ArduPilot/MAVLink gotchas (running list)
- `GZ_SIM_RESOURCE_PATH` must include `ardupilot_gazebo`'s `share` dir or the world fails to load.
- First SITL boot after a fresh build shows `Frame: UNSUPPORTED` — `FRAME_CLASS`/`FRAME_TYPE` only
  apply after a `reboot`, not just `param set`.
- `DISARM_DELAY 0` needed or the vehicle auto-disarms ~10s after arming before a mission starts.
- `AUTO_OPTIONS 3` needed to allow arming + auto-takeoff directly into AUTO mode.
- `MIS_RESTART 0` required for the avoidance executor's resume assumption to hold (ADR-006).
- **Agent BEFORE SITL** (see above) — the single most time-costly ordering gotcha found in Week 3-4
  live validation.
