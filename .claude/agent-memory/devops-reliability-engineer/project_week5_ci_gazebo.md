---
name: project-week5-ci-gazebo
description: Weeks 5-6 headless Docker/Gazebo CI job — promoted from deferred to committed 2026-08-05; feasibility verdict, what's built vs unverified, full plan in docs/WEEK5_CI_GAZEBO.md
metadata:
  type: project
---

An external review promoted the deferred "headless Docker/Gazebo CI job" to committed Weeks 5-6 scope
(2026-08-05), hard-timeboxed at 2-3 days, explicit instruction to not add scope. Full committed plan,
timebox breakdown, and cut-list: `docs/WEEK5_CI_GAZEBO.md` — read that file before touching this
workstream again, this memory is a pointer + the parts worth not re-deriving.

**Feasibility verdict on GitHub-hosted-runner Gazebo rendering (the crux question), grounded in actual
research, not a guess:**
- `ArduPilot/ardupilot_gazebo`'s own CI (`ubuntu-build.yml` etc.) on hosted `ubuntu-22.04` runners is
  **build + lint only** — never launches Gazebo or SITL. Strong signal from the plugin's own
  maintainers about what's actually load-bearing.
- `ArduPilot/ardupilot`'s own SITL autotest CI **does** fly real SITL missions on hosted runners — but
  that's plain built-in-physics SITL, **no Gazebo, no rendering**. So "SITL alone on hosted runners":
  proven feasible (by upstream's own CI). "SITL + Gazebo + rendering on hosted runners": **no known
  precedent anywhere**, including upstream.
- Resource math: `docs/WEEK1_BRINGUP.md`'s own documented minimum is ≥6 CPU/≥8GB RAM/≥40GB disk, and
  the workspace build has already OOM'd once on a MORE generous local Docker Desktop config than that
  minimum. GitHub-hosted public `ubuntu-latest` is 4 vCPU/16GB RAM/**14GB SSD** — the disk figure alone
  is close to disqualifying.
- **Verdict**: implement the image + CI job, but do NOT claim it works on hosted runners without a live
  test. Gate the job to `workflow_dispatch` only until a human confirms one green run. Fallback if
  hosted runners can't do it: self-hosted runner (not implemented — needs a persistently-online human
  machine).

**What got built this session (2026-08-05), all without a live Docker/Gazebo runner:**
- `sim/docker/Dockerfile.ci` — bakes the workspace at the EXACT pinned SHAs from CLAUDE.md (not
  floating branches) into image layers. Deliberately built WITHOUT `--enable-DDS` (scope cut — the
  smoke job doesn't need the ROS2/avoidance loop, which is already unit-tested sim-agnostically
  elsewhere; adding DDS would double the ArduPilot build via a full waf reconfigure).
- `.github/workflows/sim-image.yml` — builds+pushes that image to GHCR, decoupled from every-push
  cadence (manual dispatch or on Dockerfile/pin changes only).
- `scripts/gen_boustrophedon.py` reused UNCHANGED (`--width 20 --height 30 --spacing 15 --alt 15` → 2
  lanes, 4 coverage waypoints) — no new CLI flags needed, confirmed by actually running it.
- `scripts/ci_sim_smoke.py` — non-interactive pymavlink driver (mirrors the human-proven
  `docs/WEEK1_BRINGUP.md` §6 sequence: FRAME_CLASS/TYPE+reboot dance, DISARM_DELAY, AUTO_OPTIONS=3,
  mission upload, poll `MISSION_ITEM_REACHED`, wait disarm). QGC-WPL parsing + waypoint-counting logic
  ACTUALLY RUN and verified locally; the live MAVLink control flow is unverified (no SITL available).
  **Single riskiest unverified assumption, flagged inline**: default connection is
  `tcp:127.0.0.1:5760` (SITL's own default port, used because `sim_vehicle.py --no-mavproxy` means
  nothing listens on MAVProxy's usual `udp:14550`) — check this first if the driver can't connect.
- `scripts/ci_sim_smoke.sh` — orchestrates `gz sim --headless-rendering` + `sim_vehicle.py
  --no-mavproxy` (reused verbatim from the already-proven interactive recipe, NOT a hand-rolled
  `arducopter` binary invocation — deliberately lower-risk since this session can't test it).
- `scripts/check_sim_smoke.py` — regression gate, same driver/gate split as
  [[known_ci_flake_check_mission_geofence]]'s pattern and `check_spike_regression.py`. **Fully
  unit-verified**: 5 fixture JSONs (pass, missed-waypoint, driver-error, never-disarmed/timeout,
  missing-file) all produced the correct exit code + failure reason. This is the one piece of this
  whole workstream that got the full "bug hunter, run it, don't just write it" treatment — everything
  else needed a live SITL/Gazebo process this session didn't have.
- `.github/workflows/ci.yml` `build-test-sim` job — added `workflow_dispatch: {}` to the top-level
  `on:` block (was previously push/pull_request only — needed it or the manual gate is unreachable).
  Job itself `if: github.event_name == 'workflow_dispatch'`.

**Eval-gate hook**: explicitly NOT built. Discovered MID-SESSION (important — an earlier draft of this
memory said the world file had no camera/thermal sensors; that was true when first checked and stopped
being true partway through this same session): `robotics-sim-engineer` landed ADR-007's sensor mount
(`fg_sensor_mount`, RGB+thermal on the `iris_with_gimbal_ndvi` vehicle model) into
`sim/worlds/farmguard_field.sdf`, uncommitted, in a parallel session, and `product-lead` re-cut
`docs/ROADMAP.md`'s Weeks 5-6 table the same way. **Two consequences**: (1) `docs/ROADMAP.md` now
explicitly sequences this CI job AFTER a human-run render-verification session (thermal kill-switch,
pixel smoke test, ADR-003 real-render re-confirm) — don't gate on real-render FNR until that session
lands scored output; (2) because the sensor mount is on the vehicle this smoke job flies, Gazebo
renders it every frame regardless of whether DDS/the topics are consumed — the "no camera needed"
scope-reduction this task originally leaned on no longer avoids the ogre2 render path, only the ROS2
bridge. See `docs/WEEK5_CI_GAZEBO.md`'s "Update, discovered mid-session" callout + cut-list item 6
(camera-free CI-only world, escape hatch if rendering makes the smoke job unreliable).
**Takeaway for next time**: this repo's working tree can have concurrent uncommitted changes from
other tiger-team agents mid-session — re-check assumptions (e.g. `grep` a file) if a task runs long
enough that another agent could plausibly have touched the same asset.

**Next time this comes up**: check whether a human has actually run `sim-image.yml` /
`build-test-sim` since 2026-08-05 — if so, this memory's "unverified" framing is stale and
`docs/WEEK5_CI_GAZEBO.md` should have been updated with real results (check that file's own status
line first).
