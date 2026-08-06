# Config

Data-driven scenario + mission configuration so scenarios can be added without code changes:
- `field_polygon.json` — field boundary / boundary survey (home lat/lon + ENU polygon), shared by
  the mission generator (`scripts/gen_boustrophedon.py`) and the farm world generator
  (`scripts/gen_farm_world.py`) so they stay geometrically consistent.
- `missions/` — mission / coverage parameters (`boustrophedon.waypoints`, QGC WPL 110).
- `static_obstacles.json` — ADR-001 known-static-obstacle (tree) geofence export: hand-authored row
  layout + generator-computed per-tree positions/radii. The contract `flight-software-engineer`'s
  geofence/planner reads directly (schema documented in `sim/README.md`).
- `birds/` — scripted dynamic bird actor trajectories (2-3 birds; MVP scope, not a flock). Currently
  `birds/farm_world_birds.json` (the farm world's actors); `sim/spike/scenario_default.json` has a
  separate bird scripting for the NDVI-vs-RGB spike clip (different field size/purpose, same style).
- `sitl_params/dds_udp.parm` — enables AP_DDS (the ROS 2 `/ap/*` bridge) over UDP; load with
  `sim_vehicle.py --add-param-file`. See `docs/WEEK1_BRINGUP.md` §6b and
  `docs/DECISIONS.md` for the locked topic/frame-id contract this unblocks.
- `ndvi_camera.json` — ADR-007 dual-band NDVI sensor mount (Weeks 5-6): RGB (Red band, also the
  ADR-003 NDVI+RGB comparison arm) + Gazebo thermal sensor (repurposed as synthetic NIR) intrinsics,
  the sensor-mount attachment pose, and the per-material-class `<temperature>` calibration table
  (canopy/trunk/soil/bird). Consumed by `scripts/gen_farm_world.py` (world) and
  `scripts/check_ndvi_bands.py` (the Gate 2 pixel smoke test, `docs/WEEK5_VALIDATION.md`).
