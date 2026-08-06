# FieldGuard — Decision Log (ADR-lite)

Owner: `tech-lead` (with `product-lead` for scope calls). **Every non-trivial choice goes here** with
the alternative rejected and a one-sentence reason. Per the playbook's escalation rule, when two
roles disagree the `product-lead` wins for v1 **and the disagreement is recorded here as a tradeoff.**
This log is the engineer's interview script for "why did you build it this way?"

Format per entry:

```
## ADR-NNN: <title>   (YYYY-MM-DD, status: accepted | superseded | proposed)
Decision: <what we're doing>
Alternative(s) rejected: <what we didn't do>
Why: <one to two sentences the engineer can say out loud in an interview>
Owner / roles involved:
```

---

## ADR-000: Build FieldGuard entirely in simulation (2026-07-27, accepted)
Decision: Develop the whole system in sim (Gazebo + ArduPilot SITL + ROS 2); no live hardware in v1.
Alternative(s) rejected: Fly on the single real NDVI-camera drone. Rejected — hardware team is
temporarily unavailable to add sensors, and sim lets us iterate on the hard autonomy problem safely
and reproducibly.
Why: Simulation is the honest, correct choice for iterating on safety-critical reactive avoidance —
and it lets us also simulate a second-sensor config to quantify sensor ROI, which hardware can't.
Owner / roles: product-lead, tech-lead, robotics-sim-engineer.

## ADR-001: Geofence trees as known static obstacles from a pre-flight boundary survey (2026-07-27, accepted)
Decision: Treat tree rows as known static obstacles from a pre-flight boundary survey; reserve the
perception/avoidance loop for genuinely unplanned dynamic obstacles (birds).
Alternative(s) rejected: Detect trees at runtime too. Rejected — ag operators already map field
boundaries in advance, so this is a legitimate real-world assumption, and it cleanly isolates the
actual hard problem (unplanned dynamic obstacles) instead of blurring it with static-map building.
Why: It mirrors how real ag operations work and focuses engineering effort on the differentiator.
Owner / roles: tech-lead, flight-software-engineer.

## ADR-002: v1 replanning = "avoid, return to next waypoint"; full coverage-debt reconciliation is a stretch goal (2026-07-27, accepted)
Decision: Ship the simplest correct avoidance-then-resume for v1; document full coverage-debt
reconciliation (requeue every missed cell) as an explicit stretch goal.
Alternative(s) rejected: Build full reconciliation up front. Rejected — it risks blocking v1 on the
hardest sub-problem; shipping the simple version first keeps the core loop demoable on schedule.
Why: Protects the deadline while keeping the harder version as documented, defensible interview material.
Owner / roles: product-lead, tech-lead, flight-software-engineer.

---

## ADR-003: NDVI-vs-RGB detection approach  (2026-08-04, status: ACCEPTED — confirmation-pending; spike landed, see docs/SPIKE_ndvi_vs_rgb.md §Outcome)
Decision: Detect directly on the **NDVI-rendered frame itself** (approach (a), NDVI-direct), faithful
to the single-NDVI-camera hardware (ADR-000). The synthetic-RGB pass (b) is **retained but not as the
detection path** — it becomes the NDVI+RGB comparison arm that quantifies what a second sensor buys.
No trained model is justified yet: the classical-CV blob baseline already clears the safety bar, so
any future model must beat it on the same `eval/` harness to earn its place (pre-empts scope creep).
Deciding numbers (spike clip `sim/spike/out/spike_seed42`, seed 42, 30s@10fps, 3 birds, blob baseline):
  - (a) NDVI-direct: precision 0.445, recall 0.981, **FNR 0.019, per-bird-track FNR 0.000**
  - (b) synthetic RGB: precision 1.000, recall 0.981, **FNR 0.019, per-bird-track FNR 0.000**
  Decision rule (spike §3) fires for (a): per-bird FNR 0.000 ≤ 0.10 AND frame FNR within 0.10 of (b)
  (gap 0.000) → fidelity wins the precision tiebreak. The feared failure mode (bird over bare low-NDVI
  soil → false negative) did NOT occur: caught 12/12 visible frames, birds read negative NDVI cleanly
  below soil (~0.15). (a)'s precision gap is explained, not mysterious: 66/66 false positives are ONE
  static clutter feature (zero random-noise FPs), suppressible later by the static-obstacle-map
  sanity-check + blob motion-tracking — a wasteful dodge is cheap, a missed bird is not.
Alternative(s) rejected: (b) detect on a synthetic RGB pass. Rejected as the detection path — it would
make the headline demo depend on a sensor the real drone doesn't have (an interview liability), and it
was no safer here (identical FNR), so there is no safety reason to pay the fidelity cost.
Why: We detect on the exact frame the real NDVI camera produces, so nothing about the perception demo
has to be walked back — and the numbers show the NDVI-only signal catches every bird as reliably as
RGB does, so fidelity costs us nothing but easily-suppressible extra dodges.
Open follow-up (do not silently forget): the spike clip is a **SYNTHETIC stand-in, not a real Gazebo
render** (`meta.json synthetic:true`) — the numbers validate the eval harness and give a strong first
signal but do NOT yet validate against the real render. The framing call is made **now** (default (a)
was never in real danger of falsification), but ADR-003 must be **re-confirmed by re-running
`eval/run_spike.sh` on the real Gazebo NDVI render** before it is treated as fully validated.
Owner / roles: perception-ml-engineer (decided on metric), tech-lead (recorded).

---

## ADR-004: Pin the simulation toolchain versions  (2026-07-27, status: ACCEPTED)
Confirmed by `robotics-sim-engineer` against ArduPilot's `ardupilot_gz` docs and the
`aerial-autonomy-stack` reference (both pin the same stack); no landscape shift as of mid-2026.
Exact pins live in `CLAUDE.md` "Pinned versions" and the bringup steps in `docs/WEEK1_BRINGUP.md`.
Note: `ardupilot_gazebo` uses the `ros2` branch (not `main`), and ArduPilot firmware tracks `master`
(not a stable Copter tag) because the AP_DDS/ROS 2 bridge surface tracks master — the one remaining
open item is capturing the exact firmware commit SHA once the Week 1 build is green.
Decision: pin **Gazebo Harmonic (LTS)** + ArduPilot's **`ardupilot_gz`** ROS 2
integration on **ROS 2 Humble** (Ubuntu 22.04), matching ArduPilot's officially documented and
CI-tested stack, run inside a **Docker/Ubuntu container** (the dev machine is macOS, where this
stack is not practically supported natively).
Alternative(s) rejected:
  (a) **ROS 2 Jazzy + Harmonic** — newer LTS, longer support horizon, but ArduPilot's docs and CI
      primarily exercise Humble, so it carries more first-run setup risk. Kept as the fallback.
  (b) **Native macOS install** — rejected; Gazebo + ArduPilot SITL + ROS 2 aren't practically
      supported on macOS, and fighting that would burn the Week 1-2 gate.
  (c) **Gazebo Garden** — rejected; Harmonic is the current LTS and the release ArduPilot targets.
Why: the Week 1-2 gate is "get a mission flying," so following ArduPilot's most-documented,
most-tested combination minimizes setup risk on the critical path; longevity is secondary for a
time-boxed portfolio build.
Owner / roles: robotics-sim-engineer (research + confirm exact branch/tags), devops-reliability-engineer
(container image), tech-lead (recorded). Promote to `accepted` with exact pins written into
`CLAUDE.md` once robotics-sim-engineer confirms compatibility.

## ADR-005: Enable AP_DDS explicitly + lock the /ap/* topic/service/frame contract to the pinned ArduPilot SHA   (2026-08-04, status: ACCEPTED — CONFIRMED live 2026-08-05)
**CONFIRMED 2026-08-05 (Week-3 Gate 2, `docs/WEEK3_VALIDATION.md`):** all 18 `/ap/*` topics enumerated
below appeared on the running bridge, exactly matching this source-verified list (14 publishers + 4 `/ap`
subscribers; the 5th subscriber is the bare `/clock`). **Correction to the original enablement claim:**
AP_DDS is **compiled OUT of SITL by default** (`-DAP_DDS_ENABLED=0`) — SITL must be built with
`sim_vehicle.py --enable-DDS` first, or the `DDS_ENABLE` param does not even exist and no `/ap/*` topics
appear. The param file alone is NOT sufficient. (An earlier draft implied DDS was compiled-in by default;
that conflated the `AP_DDS_ENABLED` compile gate with the `DDS_ENABLE` param value — see WEEK1_BRINGUP §6b.)
Decision: Build SITL with `--enable-DDS`, **then** enable the bridge via an explicit param file
(`config/sitl_params/dds_udp.parm`: DDS_ENABLE=1, DDS_UDP_PORT=2019), loaded through
`sim_vehicle.py --add-param-file` rather than relying on `ardupilot_gz_bringup`'s launch file. Keep DDS_USE_NS=0 (compiled default) so
names stay a flat `/ap/<name>`. Lock the following `/ap/*` interface — verified directly from source at
ArduPilot commit `9895756d874ec9128d50918f6747a83706f4e221` (V4.8.0-dev, CLAUDE.md "Pinned commit
SHAs"), every `#if AP_DDS_*_ENABLED` gate checked, not guessed — as the contract Week 3-4
perception/planner ROS 2 nodes code against:
  Publishers: /ap/time (builtin_interfaces/Time), /ap/navsat (sensor_msgs/NavSatFix, frame_id=GPS
  instance index as string), /ap/tf_static (tf2_msgs/TFMessage, base_link->GPS_<i>), /ap/battery
  (sensor_msgs/BatteryState, frame_id=battery instance index), /ap/imu/experimental/data
  (sensor_msgs/Imu, frame_id=base_link_ned), /ap/pose/filtered (geometry_msgs/PoseStamped,
  frame_id=base_link **but content is ENU position relative to EKF/home origin — REP-105 mislabeling,
  treat content not frame_id as authoritative**), /ap/twist/filtered (geometry_msgs/TwistStamped,
  frame_id=base_link; linear=world ENU, angular=body-frame — two frames under one label), /ap/airspeed
  (ardupilot_msgs/Airspeed), /ap/rc (ardupilot_msgs/Rc), /ap/geopose/filtered
  (geographic_msgs/GeoPoseStamped), /ap/goal_lla (geographic_msgs/GeoPointStamped), /ap/clock
  (rosgraph_msgs/Clock), /ap/gps_global_origin/filtered (geographic_msgs/GeoPointStamped, WGS-84 EKF
  origin — the anchor for pose/filtered's ENU frame), /ap/status (ardupilot_msgs/Status).
  Subscribers: **/clock** (rosgraph_msgs/Clock — note: **NOT /ap/clock**, an absolute-path special
  case in the topic table), /ap/joy (sensor_msgs/Joy), /ap/tf (tf2_msgs/TFMessage), /ap/cmd_vel
  (geometry_msgs/TwistStamped), /ap/cmd_gps_pose (ardupilot_msgs/GlobalPosition).
  Services (ArduPilot=server): /ap/arm_motors, /ap/mode_switch, /ap/prearm_check,
  /ap/experimental/takeoff, /ap/set_parameters, /ap/get_parameters.
  Source: libraries/AP_DDS/{AP_DDS_Topic_Table.h, AP_DDS_Service_Table.h, AP_DDS_Client.h,
  AP_DDS_Client.cpp, AP_DDS_config.h, AP_DDS_Frames.h} @ ardupilot commit
  9895756d874ec9128d50918f6747a83706f4e221.
Alternative(s) rejected:
  (a) Use `ardupilot_gz_bringup`'s default DDS enablement (auto-loads dds_udp.parm + dds_use_ns.parm,
      auto-spawns micro_ros_agent). Rejected — that launch file hardcodes its own world path (already
      rejected project-wide, sim/README.md / WEEK1_BRINGUP.md), and its default DDS_USE_NS=1 would
      namespace every topic under /v<sysid>/ for no benefit in a single-vehicle project.
  (b) Trust the compiled-in ENABLED_BY_DEFAULT=1 and skip explicit enablement. Rejected — a SITL
      instance's eeprom.bin (persisted on our named Docker volume) keeps whatever DDS_ENABLE value was
      saved the first time that param existed; a later compiled-default change does not retroactively
      re-enable an existing instance. Explicit + reproducible beats implicit.
  (c) Take topic/frame names from ArduPilot's ROS 2 wiki/docs. Rejected — ROADMAP already flags these
      names have moved between versions; the reproducibility anchor is the pinned commit SHA, not a
      version-unspecified doc page.
Why: The Week 3-4 avoidance loop is a ROS 2 control path that consumes ArduPilot telemetry and issues
guided commands over these exact names/types/frames, so I locked the contract by reading AP_DDS source
at the exact commit we build — that way perception and planner nodes can be written in parallel against
names that won't silently drift, with a concrete re-verification target the day we bump the SHA.
Open follow-up (do not silently forget): this contract is verified from **source at the pinned SHA**,
but the live bridge only comes up in the human Docker run — so the actual `ros2 topic list` /
`ros2 topic hz` confirmation against a running SITL+micro-ROS-agent is still owed. Confirm the topics
appear with these names/types before treating ADR-005 as fully validated (same pattern as ADR-003's
"re-confirm on the real Gazebo render").
Owner / roles: flight-software-engineer (verified source + drafted), tech-lead (records).

## ADR-006: Reactive-avoidance executor = AUTO->GUIDED->AUTO, we own the maneuver policy   (2026-08-05, status: ACCEPTED — CONFIRMED live 2026-08-05)
**CONFIRMED 2026-08-05 (Week-3 Gate 3, `docs/WEEK3_VALIDATION.md`):** with `MIS_RESTART=0`, AUTO→GUIDED→AUTO
resumed the interrupted leg (reached #3, took control heading to #4, handed back → resumed at #4, continued
#5→#8), no restart at #1. The resume mechanism the executor depends on works on the real stack.
Decision: On a dynamic bird detection during the AUTO boustrophedon mission, **our** executor node
takes control by switching AUTO -> GUIDED (via the `/ap/mode_switch` service, ardupilot_msgs/ModeSwitch,
locked in ADR-005), commands a **single pre-vetted avoidance setpoint** in GUIDED, then switches
GUIDED -> AUTO to resume coverage. Verified mechanism, all cited @ pinned ArduPilot commit
`9895756d874ec9128d50918f6747a83706f4e221`:
  - **Maneuver command (primary):** a discrete guided position setpoint on `/ap/cmd_gps_pose`
    (ardupilot_msgs/GlobalPosition, WGS-84, anchored to `/ap/gps_global_origin/filtered` from ADR-005).
    Alternative primitive `/ap/cmd_vel` (geometry_msgs/TwistStamped, world-ENU) is also valid but is a
    velocity we'd have to integrate to safety-check; a position target is the thing the safety gate
    actually evaluates, so it is the v1 primitive. **Both are honored only in GUIDED + armed:**
    `AP_DDS_ExternalControl.cpp::handle_velocity_control` / `handle_global_position_control` ->
    `AP_ExternalControl_Copter::set_linear_velocity_and_yaw_rate` / `set_global_position`, each gated by
    `ready_for_external_control()` = `copter.flightmode->in_guided_mode() && copter.motors->armed()`
    (ArduCopter/AP_ExternalControl_Copter.cpp @ SHA). This reconciles the ADR-005 note that `/ap/cmd_vel`
    is a subscriber: it is a live input, but ArduPilot silently drops it unless we are in GUIDED.
  - **Frame the executor MUST command in:** world-**ENU** with `header.frame_id = "map"`. Unlike
    `/ap/pose/filtered` (ADR-005: content authoritative, frame_id lies), for these command topics the
    `frame_id` **is honored** as a real switch: `handle_velocity_control` transforms `"map"` ENU -> NED
    as `{linear.y, linear.x, -linear.z}`, whereas `"base_link"` is treated as body frame via
    `ahrs.body_to_earth()` (AP_DDS_ExternalControl.cpp @ SHA). Sending `base_link` by mistake would fly a
    body-frame dodge. Command `"map"`/ENU.
  - **Resume mechanism:** re-entering AUTO runs `ModeAuto::run()` -> `mission.start_or_resume()`
    (ArduCopter/mode_auto.cpp @ SHA), which calls `resume()` unless `MIS_RESTART==1`
    (AP_Mission.cpp::start_or_resume @ SHA). We **pin `MIS_RESTART=0`** in the param file (same explicit
    discipline as ADR-005) so AUTO deterministically resumes the leg it was flying and continues to the
    **same next waypoint** it was navigating to when interrupted — exactly the ADR-002 v1 behavior, no
    index manipulation required.
  - **Why no waypoint-index juggling:** AP_DDS at this SHA exposes **no mission-current service** (ADR-005
    table: mode_switch/arm/prearm/takeoff/get+set_parameters only). Skipping/requeuing cells would need
    `AP_Mission::set_current_cmd` reachable only via MAVLink `MAV_CMD_DO_SET_MISSION_CURRENT` — a second
    control channel. v1 doesn't need it (natural resume suffices), which is a verified concrete reason the
    full coverage-debt reconciliation (ADR-002 stretch) is genuinely harder, not just deferred.
Safety requirement handed to flight-software (build, not decided here): the executor MUST pass the
candidate avoidance setpoint (and ideally the swept path to it) through a **3D safety gate BEFORE**
switching to GUIDED — the target must lie outside every geofenced-tree obstacle volume AND within
altitude bounds. `config`/`geofence.py` is currently **XY-only**; extend it to altitude-aware so a dodge
cannot climb/descend into a canopy or breach the ceiling (QA's `geo_avoid_into_tree` is the regression).
If the gate rejects the primary dodge, fall back to hover-in-GUIDED; never execute an unvetted maneuver.
Log every takeover (trigger detection id + AUTO->GUIDED), the maneuver target + gate verdict, and the
resume (GUIDED->AUTO + resumed waypoint) per the CLAUDE.md instrumentation rule.
Alternative(s) rejected:
  (a) Pure MAVLink mission manipulation / `DO_REPOSITION`. Rejected for v1 — it adds a second control
      channel alongside the AP_DDS bridge we already locked (ADR-005) for no v1 benefit; keep one bus.
      (It's the natural home for the ADR-002 *stretch* requeue, which genuinely needs it — noted above.)
  (b) Lean on ArduPilot's built-in object avoidance (BendyRuler/Dijkstra + `OA_*`/proximity). Rejected —
      that path is built for known-obstacle/proximity avoidance and would move the reactive decision
      **into the autopilot**, deleting the exact thing this project exists to show (priority #1); our
      differentiator is that *our* code sees, decides, and acts, and we can log and defend every step.
Why: We keep the avoidance brain in our own ROS 2 code — detect, safety-gate, switch to GUIDED, command
one vetted setpoint, then hand control back to AUTO which resumes the mission on its own — because that
is the whole point of the project (priority #1), and every takeover, maneuver, and resume is a line in a
log I can walk an interviewer through.
Open follow-up (do not silently forget): the interface is verified from **source @ the pinned SHA**, but
the live behavior — that GUIDED accepts our setpoint mid-mission and AUTO with `MIS_RESTART=0` actually
resumes to the intended waypoint — must be **confirmed in the human Docker run** before ADR-006 is fully
validated (same pattern as ADR-003 real-render and ADR-005 live-topic checks; batch all three).
Owner / roles: tech-lead (decided + verified source@SHA), flight-software-engineer (builds executor +
3D geofence), perception-ml-engineer (detection trigger), qa-safety-reviewer (`geo_avoid_into_tree`).

## ADR-007: Produce the dual-band NDVI frame with an RGB camera (Red) + Gazebo's thermal sensor repurposed as synthetic NIR; NDVI computed in a ROS 2 node   (2026-08-05, status: ACCEPTED — confirmation-pending; render mechanism unproven live)
Decision: Render the two NDVI bands as **two co-located Gazebo Harmonic sensors on one rigid nadir
mount**, and compute the index in ROS 2, not in the render:
  - **Red band** = the **R channel of a standard `type="camera"` (R8G8B8) sensor**. That same RGB
    image is *also* the ADR-003 comparison arm (NDVI+RGB), so the Red-band source doubles as the
    second-sensor arm — zero extra cameras for the comparison.
  - **NIR band** = a **`type="thermal"` sensor (L16) repurposed as synthetic NIR**. Gazebo's thermal
    camera reads a *per-object scalar signature you author in SDF* (via `gz-sim-thermal-system`'s
    `<temperature>` / `<heat_signature>` on each visual), **independent of visible color and
    lighting** — which is exactly the "per-model reflectance property so vegetation reads high-NIR,
    soil/water/birds low" that option (a) calls for, delivered by a **first-class documented sensor
    instead of a hand-written shader**. Each world material carries a `<temperature>` that encodes its
    NIR reflectance, calibrated into the sensor's `[min_temp,max_temp]` so the bridged `mono16` maps
    **linearly to NIR reflectance ρ_nir∈[0,1]** (the one calibration knob; lives in the camera pkg,
    not this ADR).
  - **NDVI** is computed in a dedicated ROS 2 node (`ndvi_node`), **not** baked into the render:
    it pairs the two bridged images by nearest sim-time stamp
    (`message_filters` ApproximateTime), rescales R/255→ρ_red∈[0,1] and mono16→ρ_nir∈[0,1], and
    publishes **NDVI=(NIR−Red)/(NIR+Red)** per pixel as `32FC1`∈[−1,1]. Rationale: Gazebo keeps
    emitting raw bands (honest), the index math stays unit-tested and offline-reproducible (the eval
    harness already consumes `float32` NDVI `.npy`), and the ROS contract is stable per ADR-005
    discipline. **Hard build requirement:** the RGB and thermal sensors MUST share identical
    intrinsics (width/height/hfov), pose (co-located nadir, matching the spike extrinsic
    `quat_wxyz=(0,1,0,0)`) and `update_rate`, so the node combines pixel-wise with no resampling.
    `use_sim_time=true`; the NDVI frame inherits the **RGB image stamp** (the georef anchor).
  - **Stale-pair guard (amendment 2026-08-05):** the node MUST enforce a **max stamp-delta of 25%
    of one frame period** when pairing Red↔NIR (default **25 ms** at the spike's 10 Hz anchor;
    scales as `0.25/update_rate`). Since both sensors share `update_rate` by construction, a correct
    pair is stamp-aligned to within render jitter and only a *dropped frame* pushes the nearest match
    toward a full period; 25% sits well above jitter yet well below the half-period point where the
    match flips to the wrong neighbor. On exceed: **drop the frame and increment a logged
    `dropped_pair` counter** (instrumentation per CLAUDE.md) rather than emit a mispaired NDVI — a
    persistently rising count is itself the signal that band rates are drifting under load.
**Locked topic/message contract** (perception + stitch code against these names, same as ADR-005):
  - `/fg/sensor/rgb/image` — `sensor_msgs/Image` (`rgb8`)  [Red band = ch0; also the NDVI+RGB arm]
  - `/fg/sensor/rgb/camera_info` — `sensor_msgs/CameraInfo`
  - `/fg/sensor/nir/image` — `sensor_msgs/Image` (`mono16`, thermal→NIR proxy)
  - `/fg/sensor/nir/camera_info` — `sensor_msgs/CameraInfo`
  - `/fg/ndvi/image` — `sensor_msgs/Image` (`32FC1`, values ∈[−1,1]) ← **authoritative frame ADR-003
    detects on and the stitch georeferences** (via ADR-005 `/ap/pose/filtered` +
    `/ap/gps_global_origin/filtered` + this stamp + `camera_info`)
  - `/fg/ndvi/camera_info` — `sensor_msgs/CameraInfo` (pass-through intrinsics for the stitch)
  - `/fg/ndvi/preview` — `sensor_msgs/Image` (`rgb8`, false-color, HUMAN-ONLY, non-authoritative)
Second-sensor comparison arm (concrete): **NDVI+RGB, reusing the RGB camera already needed for the
Red band.** A `type="depth"` camera (NDVI+depth) is a documented **stretch**, not v1 — we get the RGB
arm for free, depth costs a third sensor for no v1 detection benefit (ADR-003 already chose NDVI-direct).
Alternative(s) rejected:
  (b) Single camera + a **shader/material-encoded** synthetic NIR band. Rejected — it means writing and
      maintaining custom OGRE2 render passes/materials: clever but hard to explain and fragile against
      Gazebo rendering-engine updates, and the thermal sensor already gives the identical
      per-model-reflectance capability as a supported, documented sensor. Boring-but-explainable wins.
  (c) **Post-process synthetic NIR derived from RGB + a vegetation mask.** Rejected — the NIR would be a
      *function of the visible render*, so NDVI carries **no independent second-band information**; this
      is essentially what the Week-2 synthetic spike clip already did (`meta.json synthetic:true`), so
      choosing it would make the ADR-003 "re-confirm on the REAL render" step **circular and meaningless**
      — you'd re-validate the detector on a frame whose NIR is manufactured from its own RGB. The whole
      point of a real render is a genuinely independent NIR band; (c) throws that away.
Why: One normal RGB camera gives me the Red band and doubles as the comparison arm; I repurpose Gazebo's
thermal sensor — which reads an author-controlled per-object signature, not visible color — as an
**independent** synthetic NIR band; a small ROS 2 node combines them into a georeferenced NDVI frame.
It's a two-camera render (option (a)) built entirely from documented, first-class Gazebo sensors on the
already-proven `ogre2` Sensors system, so nothing about it is exotic to explain — and because the NIR
band is genuinely independent of the visible render, the ADR-003 real-render re-confirmation actually
tests something.
Known-honest caveat (say it out loud): this is a **synthetic sim NDVI**, not radiometric truth — Red
comes from a lit visible render while NIR is an illumination-independent scalar, so shadowed canopy can
spuriously raise NDVI. Mitigation: render with a fixed high sun + dominantly diffuse sky to suppress
shadows; if the ADR-003 re-run shows lighting artifacts break detection, fall back to authoring the
**Red band as a second thermal-style reflectance scalar** (both bands illumination-independent, matching
the spike's material-property NDVI model) — documented fallback, not built up front.
Open follow-up (do not silently forget): the render only comes up in the **human Docker run**, so the
whole mechanism is unproven live. Concrete re-verification targets (batch with the ADR-003 real-render
spike re-run + ADR-005 live-topic + ADR-006 live-resume checks — one Docker session):
  1. `ros2 topic list` shows the six `/fg/*` topics; `ros2 topic hz /fg/ndvi/image` at camera rate;
     `ros2 topic echo --field encoding` returns `rgb8`, `mono16`, `32FC1` respectively.
  2. Sample a canopy pixel vs. bare-soil vs. bird pixel on `/fg/ndvi/image`: canopy high-positive, soil
     near-zero/low, bird negative — this is the direct proof the NIR band is **independent** of RGB
     (the thing (c) cannot produce); if soil and canopy NDVI are indistinguishable the temperature
     authoring is wrong (every object returning ambient = flat NDVI).
  3. Point `eval/run_spike.sh` at the real render's output dir (drop-in per `sim/spike/README.md` schema)
     → re-confirm ADR-003 numbers hold on the real render.
  4. Confirm the pinned Harmonic build exposes the thermal sensor on `ogre2` (Sensors system already
     runs `ogre2`, world line 13-15) — thermal is ogre2-only; verify `gz-sim-thermal-system` loads.
  5. **Principal point (cx,cy) unpinned** — georef defaults cx,cy to image-center (`CameraIntrinsics.from_config`);
     CONFIRM empirically against the real `/fg/*/camera_info` once Gate 1 publishes it (log in WEEK5_VALIDATION.md).
  6. **Georef anchor rule (DECIDED, from stitch build):** anchor to the **live `/ap/gps_global_origin/filtered`**
     (WGS-84 EKF origin, ADR-005) at runtime — authoritative; `config/field_polygon.json` home is used **only**
     for offline/test. `home_lat/lon/alt` is a transform param, sourced live and config-defaulted offline.
  7. **Dependency boundary (DECIDED, from stitch build):** `fieldguard_planning` stays **stdlib-only** for the
     planning/avoidance core; **numpy** is permitted **only** in the NDVI image-math modules (`ndvi_fusion.py`,
     `ndvi_georef.py`) — genuine array math, already project-blessed via `requirements-eval.txt`.
Owner / roles: tech-lead (decided + verified against Gazebo Harmonic sensor docs + ros_gz bridge),
robotics-sim-engineer (builds the two-sensor mount + per-model temperature authoring + bridge),
perception-ml-engineer (ADR-003 re-run on `/fg/ndvi/image`), flight-software-engineer (georef stitch
consumes `/fg/ndvi/*` + ADR-005 pose/origin).
