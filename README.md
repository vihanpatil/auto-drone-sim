# FieldGuard 🛸🌾

> Autonomous drone survey system in simulation: **live reactive obstacle avoidance** +
> **NDVI crop-health mapping**, on the ArduPilot + Gazebo + ROS 2 stack.

**Status (2026-08-05):** Weeks 1–4 complete. The **reactive-avoidance loop — the core differentiator —
has been demonstrated end-to-end, live, on the real ArduPilot SITL + Gazebo + ROS 2 stack**: during a
boustrophedon survey the drone detects a dynamic obstacle, takes control, flies a 3D-safe dodge, holds
clear, and resumes coverage without silently dropping a cell. NDVI health mapping (Weeks 5–6) and the
farmer-facing dashboard (Week 7) are next. This is a **simulation-only portfolio project** (no live
hardware, ADR-000). See `CLAUDE.md` for the summary, `docs/ROADMAP.md` for status, `docs/SPEC.md` for
the full spec.

## Why this is interesting
Commercial ag-drone platforms (DJI, DroneDeploy, Sentera/John Deere, Trimble) fly **pre-surveyed
static missions**. FieldGuard adds the thing they don't: **live reactive avoidance of unplanned
dynamic obstacles, with coverage integrity** — every cell the dodge disturbs is either still covered
or explicitly logged as coverage *debt*, never silently skipped. NDVI health mapping falls out of the
same flight/camera pipeline. (Full coverage-debt *reconciliation* — requeue and re-fly every missed
cell — is a documented stretch goal, ADR-002; v1 ships "avoid, return to next waypoint" + honest debt.)

## Headline metrics (from the `eval/` harness)
- **Detection:** per-bird-track FNR **0.000** (every bird seen before closest approach) on the fixed-seed
  spike clip; the classical-CV blob baseline clears the safety bar, so no trained model is justified yet
  (ADR-003). _Caveat: the deciding clip is a **synthetic** stand-in; ADR-003 is re-confirmed on the real
  NDVI render in Weeks 5–6._
- **Avoidance loop:** demonstrated live — one clean AUTO→GUIDED→AUTO takeover/resume per encounter, the
  dodge setpoint 3D-vetted against the tree geofence, coverage-debt ledger honest by construction.
- **Coverage integrity:** the ledger partition invariant (`coverage.check_ledger`) makes a
  silently-skipped cell a **test failure**; 5 previously-pending safety assertions (coverage-integrity +
  avoid-into-tree, across 4 scenarios) now pass against real flight logs the loop produced.
- **Automated tests:** 94 (`tests/fieldguard_planning`, run via `PYTHONPATH=src python3 -m unittest
  discover -s tests/fieldguard_planning`, + the eval spike harness), green in CI. The avoidance-loop/
  coverage/geofence suite is stdlib-only (zero installs); the Weeks 5-6 NDVI fusion + georef-stitch
  tests (`test_ndvi_fusion.py`, `test_ndvi_georef.py`) need numpy (already pinned in
  `requirements-eval.txt`) — a scoped, documented exception, not a project-wide dependency change.

## Architecture (short version)
```
Gazebo farm world  ─►  (NDVI camera: Weeks 5-6) ─►  perception: detect dynamic obstacle
   (ardupilot_gazebo)          │                          │  (blob detector, ADR-003)
ArduPilot SITL  ◄── AP_DDS ──  ROS 2 avoidance node:  policy (when/where to dodge, 3D-safe)
   /ap/mode_switch, /ap/cmd_gps_pose          │      + executor (take over, resume, book coverage-debt)
                                              └─►  NDVI health map (georeferenced) ─► light dashboard
```
The boustrophedon coverage mission flies over MAVLink; the reactive avoidance loop commands ArduPilot
over the **AP_DDS `/ap/*` bridge** (ADR-005/006). Full detail: `docs/SPEC.md`. Design tradeoffs &
rationale: `docs/DECISIONS.md`.

## Run it
Runs in Docker (the stack isn't practically supported natively on macOS, ADR-004).
1. **Bring up the sim** — `docs/WEEK1_BRINGUP.md` (build the image, then Gazebo → micro-ROS agent →
   SITL, *in that order* — the agent must be listening before SITL's DDS client starts).
2. **Reproduce the live avoidance demo** — `docs/WEEK3_AVOIDANCE_DEMO.md` (runs the loop against a
   scripted bird; writes a flight log to `eval/results/live_flight_log.json`).
3. **Run the tests / eval harness (no Docker needed)** —
   `python3 -m unittest discover -s tests/fieldguard_planning` and `bash eval/run_spike.sh` (needs the
   pinned deps in `requirements-eval.txt`). Both run in CI (`.github/workflows/ci.yml`).

## How this repo is built
Developed with a Claude Code **tiger team** — eight specialized subagents in `.claude/agents/`
(product, tech-lead, perception/ML, sim, flight-software, devops, QA/safety, GTM). See
[`TIGER_TEAM_GUIDE.md`](TIGER_TEAM_GUIDE.md). Start a work session with `/standup`.
