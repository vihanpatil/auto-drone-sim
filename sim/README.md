# Gazebo simulation assets

Owned by `robotics-sim-engineer`. Scenario parameters are data-driven (`config/`) so scenarios can
be added without code changes — see [Regenerating the world](#regenerating-the-world).

- `worlds/farmguard_field.sdf` — the custom farm world: bounded field polygon (green ground plane),
  3 orchard tree rows (18 trees, static/geofenced obstacles), 3 scripted bird actors (dynamic
  obstacles), the `iris_with_gimbal` vehicle, and (ADR-007, Weeks 5-6) the dual-band NDVI sensor
  mount (`iris_with_gimbal_ndvi`) plus a calibrated `<temperature>` on every visual in the world.
  **Generated** by `scripts/gen_farm_world.py` — don't hand-edit; edit the `config/` inputs below
  and regenerate.
- `bridge/fg_sensor_bridge.yaml` — ros_gz_bridge config for the four `/fg/sensor/*` topics
  (ADR-007). See `docs/WEEK5_VALIDATION.md` Gate 1.
- `models/` — reserved for future mesh/model assets. Still empty: the farm world (including the
  Week 5-6 NDVI sensor mount) uses only inline SDF primitives + first-class Gazebo sensor types, no
  external meshes, so it has no dependency on this directory or on `GZ_SIM_RESOURCE_PATH` beyond
  what `ardupilot_gazebo` already requires (see docs/WEEK1_BRINGUP.md §5) — a deliberate choice to
  avoid adding new resource-path risk.
- `spike/` — the ADR-003 NDVI-vs-RGB spike clip generator (separate concern, see `spike/README.md`).
- `docker/` — the Week 1 starter container (see `docs/WEEK1_BRINGUP.md`).

## The farm world's config-driven inputs

| File | What it defines | Consumed by |
|---|---|---|
| `config/field_polygon.json` | field boundary (home lat/lon, ENU polygon), mission altitude | `scripts/gen_boustrophedon.py` (mission) and `scripts/gen_farm_world.py` (world) — **the reason both stay geometrically consistent** |
| `config/static_obstacles.json` | tree row layout (input) + the flattened per-tree obstacle list (generated output) | `scripts/gen_farm_world.py` (world); **flight-software's geofence/planner** (see contract below) |
| `config/birds/farm_world_birds.json` | 3 scripted bird actor trajectories (piecewise-linear waypoints, there-and-back loops) | `scripts/gen_farm_world.py` (world) |
| `config/ndvi_camera.json` | ADR-007 dual-band NDVI sensor mount: intrinsics/rate, the sensor-mount attachment (parent link + pose), and the per-material-class `<temperature>` calibration table | `scripts/gen_farm_world.py` (world); `scripts/check_ndvi_bands.py` (Gate 2 smoke test) |

## NDVI sensor topics (ADR-007, Weeks 5-6)

Locked contract, `docs/DECISIONS.md` ADR-007 — **not confirmed live yet**, see
`docs/WEEK5_VALIDATION.md`:

- `/fg/sensor/rgb/image` (`rgb8`) + `/fg/sensor/rgb/camera_info` — Red band; also the ADR-003
  NDVI+RGB comparison arm.
- `/fg/sensor/nir/image` (`mono16`) + `/fg/sensor/nir/camera_info` — Gazebo's thermal sensor
  repurposed as synthetic NIR, per-visual `<temperature>` authored from `config/ndvi_camera.json`.
- `/fg/ndvi/image` + `/fg/ndvi/camera_info` + `/fg/ndvi/preview` — **not built yet**; the fused
  NDVI node is `flight-software-engineer`'s downstream Weeks 5-6 scope.

Bridge these four topics with `sim/bridge/fg_sensor_bridge.yaml`
(`ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=sim/bridge/fg_sensor_bridge.yaml`).
Validate with `scripts/check_ndvi_bands.py` — see `docs/WEEK5_VALIDATION.md` Gate 2.

### Regenerating the world

```bash
python3 scripts/gen_farm_world.py
```

Reads the three files above, rewrites `config/static_obstacles.json`'s `obstacles` array (computed
fresh from its own `layout` section) and `sim/worlds/farmguard_field.sdf` **in the same run**, so
the geofence export and the Gazebo world can never drift apart — there is exactly one place
(`scripts/gen_farm_world.py:compute_obstacles`) that turns tree-row layout into tree positions.
The script also checks the output SDF is well-formed XML before writing it (`xml.etree`) and that
every computed tree position falls inside the field polygon (raises if not). Change tree rows, add
a 4th bird, or resize the field by editing the `config/` inputs and rerunning — no code changes.

## Static-obstacle geofence contract (for `flight-software-engineer`)

**File**: `config/static_obstacles.json`, `obstacles` array. **Frame**: local ENU meters (x=East,
y=North, z=Up), origin = `config/field_polygon.json`'s `home_lat`/`home_lon` — same convention as
`sim/spike/README.md`. Per ADR-001 (`docs/DECISIONS.md`), these are **known static obstacles from a
pre-flight boundary survey** — geofence against them directly, do not run detection on them; the
runtime perception/avoidance loop is reserved for the bird actors (genuinely unplanned dynamic
obstacles).

Each entry:
```jsonc
{
  "id": "tree_row0_0",          // stable, unique
  "type": "tree",
  "row_id": 0,
  "pos_m": [15.0, 5.0, 0.0],     // ENU meters, ground-level (x,y,z=0)
  "obstacle_radius_m": 2.0,       // USE THIS for geofence exclusion (canopy radius + safety margin)
  "canopy_radius_m": 1.3,          // Gazebo collision/visual geometry only, not the geofence radius
  "height_m": 3.5                   // approximate total tree height (trunk + canopy), not a tight bbox
}
```
Read `obstacle_radius_m` for the exclusion radius, not `canopy_radius_m` — the latter is what the
Gazebo model's collision sphere actually uses, the former already includes a safety margin on top.
Tree height (3.5m) is well below `config/field_polygon.json`'s `mission_altitude_m` (15m), so the
existing boustrophedon mission physically clears every tree — this is why "the mission flies through
the world" held even before the avoidance loop existed. (The reactive-avoidance loop is now built and
demonstrated live — see `docs/WEEK3_AVOIDANCE_DEMO.md`; it adds its own 3D safety gate on top of this
geofence for dodge maneuvers that may leave cruise altitude, `geofence.is_safe_3d`.)

**Consumer implementation (Week 2, `flight-software-engineer`):** `src/fieldguard_planning/geofence.py`
loads this file and exposes `GeofenceMap.from_file(...)` with `is_point_excluded(x, y)`,
`excluding_obstacle(x, y)`, `point_clearance(x, y)`, and `segment_clearance(p1, p2)` /
`check_path(points)` for mission-leg (not just point) checks — see that module's docstring and
`tests/fieldguard_planning/test_geofence.py`. Stdlib-only on purpose so the whole avoidance loop that
now consumes it (`avoidance_policy.py`, `avoidance_executor.py`) stays unit-testable without a ROS 2 /
Gazebo dependency just to ask "is this point a known tree." Still not packaged as a colcon package (no
`package.xml`); the live ROS 2 node runs via `python3 -m fieldguard_planning.avoidance_node` — colcon
packaging (for `ros2 run`) is a documented follow-up.

## Frame conventions (must match across the project)

Same as `sim/spike/README.md`: **world ENU meters** (x=East, y=North, z=Up), matching REP-103 /
what the AP_DDS ROS 2 bridge publishes — not ArduPilot's internal NED. The world's
`spherical_coordinates` (lat/lon/elevation) is identical to `ardupilot_gz`'s stock `iris_runway.sdf`
and to `config/field_polygon.json`'s `home_lat`/`home_lon`, so `(0,0)` in this ENU frame is both the
SITL home and the field polygon's SW corner — no offset to track between the mission, the world, and
the obstacle export.

## Launching the world (human, inside the Docker container — see `docs/WEEK1_BRINGUP.md`)

**Quick start:** `scripts/run_farm_mission.sh` (run inside the container) packages Shell A below
into one command and prints the exact Shell B recipe — added by `flight-software-engineer` in Week
2 as the mission-through-world launch helper, not a new launch path (see that script's header
comment for why Shell B stays manual). The manual two-shell flow below is still the documented
ground truth if you want to run it by hand or the script needs debugging.

Follow the same **two-shell flow** already proven for `iris_runway.sdf` in
`docs/WEEK1_BRINGUP.md` §5-6, just pointing at this world file directly instead of relying on a ROS
2 launch file (`ardupilot_gz_bringup`'s launch files hardcode their own world path internally, so
they can't point at a custom world without editing them — the two-shell flow sidesteps that and is
also what WEEK1_BRINGUP.md already recommends as the more debuggable default):

```bash
# Shell A — start the world and leave it running:
source /root/ardu_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/root/ardu_ws/install/ardupilot_gazebo/share"
gz sim -v4 -s -r --headless-rendering /workspace/fieldguard/sim/worlds/farmguard_field.sdf
```

**Verify:** the world stays up — no `Unable to find uri` / `Failed to load a world` (the same
failure modes WEEK1_BRINGUP.md §5 already documents for `iris_runway.sdf`; this world only adds
inline primitives + one `model://iris_with_gimbal` include, so if it fails, it's almost certainly
the same `GZ_SIM_RESOURCE_PATH` issue, not something new). `gz topic -l | grep model/tree_row0_0`
or `gz model -m tree_row0_0 -p` (Gazebo Harmonic CLI) confirm a tree loaded where expected; a bird
actor's pose changing over consecutive `gz model -m bird_0 -p` calls confirms it's actually moving.

```bash
# Shell B — SITL, wired to Gazebo (identical to WEEK1_BRINGUP.md §6 -- vehicle model/pose is the
# same iris_with_gimbal at the same spawn pose, so nothing about the SITL side changes):
cd /root/ardu_ws/src/ardupilot
export PATH="$PWD/Tools/autotest:$PATH"
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON
```

Then fly the existing mission exactly as in `docs/WEEK1_BRINGUP.md` §8 (`wp load
/workspace/fieldguard/config/missions/boustrophedon.waypoints`, `mode auto`, `arm throttle`) — no
mission regeneration needed, since `config/field_polygon.json` (and therefore this world) matches
the polygon that mission already sweeps.

## What's been validated statically vs. what still needs a human Docker run

Validated in this (non-Docker) session:
- `sim/worlds/farmguard_field.sdf` is well-formed XML (`xml.etree.ElementTree`, cross-checked with
  `xmllint --noout`).
- Structural sanity: 20 `<model>` elements (1 ground plane + 18 trees + 1 `iris_with_gimbal_ndvi`
  vehicle/sensor-mount wrapper), 3 `<actor>` elements, no duplicate model/actor names, and exactly
  40 per-visual `gz::sim::systems::Thermal` plugins (1 ground + 18×2 tree trunk/canopy + 3 birds —
  every visual in the world, ADR-007 Weeks 5-6).
- Every computed tree position falls inside `config/field_polygon.json`'s polygon (generator
  raises otherwise).
- `config/static_obstacles.json`'s `obstacles` array numerically matches the tree `<pose>` values
  actually written into the SDF (both come from the same in-memory list in one generator run).
- World-level plugins, `spherical_coordinates`, and the vehicle `<include>`/pose are copied
  verbatim from `ardupilot_gz`'s own `iris_runway.sdf`/`iris_with_gimbal` model (fetched and
  diffed against upstream during this session) — not hand-guessed.
- The actor `<script><trajectory>` syntax (simple sphere visual + waypoint keyframes, no skin/mesh,
  `auto_start`, `loop`) matches Gazebo Harmonic's documented actor pattern and the `gz-sim`
  `examples/worlds/actor.sdf` reference example (also fetched this session).
- **(Week 2, `flight-software-engineer`)** The mission's flight path checked numerically against
  the geofence export with `scripts/check_mission_geofence.py` (uses the new
  `src/fieldguard_planning/geofence.py` + `mission_waypoints.py`, stdlib-only, no Docker needed):
  the mission regenerates byte-for-byte from the checked-in `config/` inputs (confirmed unchanged),
  and **in the XY plane** the return leg of lane x=15 runs directly along tree row 0's centerline
  (min clearance **-2.0 m**, i.e. 0 m lateral separation minus the 2.0 m `obstacle_radius_m`) —
  every other leg clears by ≥3.0 m. This is expected and safe *only* because of the ≥11.5 m
  vertical separation (15 m mission altitude − 3.5 m tree height); see
  `docs/WEEK1_BRINGUP.md`-style framing above. Flagged for Week 3-4: tree row 0 is already primed
  to be the "always-clips-in-XY" case if avoidance work later needs a forced dodge scenario
  (lower altitude / taller trees), whereas rows 1-2 sit 3-8 m off every lane by design.

**NOT validated here (needs the human-operated Docker container, per this project's standing
constraint — see `docs/WEEK1_BRINGUP.md`):**
- That `gz sim` actually loads this SDF end-to-end (semantic SDF validity beyond well-formed XML —
  e.g. whether Harmonic's actor system accepts a mesh-less `<visual>` exactly as written).
- That the boustrophedon mission flies through the world without a physics-engine surprise (e.g.
  collision margins, ground-plane friction) — geometrically (XY + Z together) it clears every tree,
  but "should" isn't "confirmed" until it's flown.
- That the bird actors visibly move along their trajectories in a running simulation (not just
  that the SDF `<trajectory>` keyframes are well-formed).
- Render/performance headroom for 3 actors + 18 static models on this project's known-slow
  llvmpipe software rendering path (macOS Docker Desktop, no GPU passthrough — see
  `docs/WEEK1_BRINGUP.md` "Known macOS gotchas" #1). Not expected to be a problem (all primitive
  geometry, no textures/meshes) but unconfirmed. Now also carries two camera-type sensors per frame
  (ADR-007) — see the next bullet.
- **(Weeks 5-6, ADR-007)** Everything about the dual-band NDVI sensor mount: whether
  `gz-sim-thermal-system`/`gz-sim-thermal-sensor-system` actually load on this pinned Harmonic +
  ogre2 build, whether the `iris_with_gimbal_ndvi` wrapper model's fixed-joint sensor-mount
  attachment resolves, and whether the per-visual `<temperature>` calibration produces genuinely
  different canopy/soil/bird readings. This is a **separate, dedicated gate** —
  `docs/WEEK5_VALIDATION.md` — not folded into the two-shell flow above.

**Human next step:** run the two-shell flow above once, confirm the world stays up and the mission
completes, then update this section (or `docs/ROADMAP.md`) with the result. If it fails, the most
likely first suspect (per this project's own prior debugging history) is `GZ_SIM_RESOURCE_PATH`
missing `ardupilot_gazebo`'s `share` dir — check that before anything else. Then run
`docs/WEEK5_VALIDATION.md`'s three gates (Gate 0 is a hard kill-switch — do it first).
