# Weeks 5-6 — Headless Docker/Gazebo CI Job (devops-reliability-engineer, 2026-08-05)

**Status: committed, hard-timeboxed at 2-3 days.** Promoted from the Weeks 3-4 "deferred, non-blocking"
line (`docs/ROADMAP.md`) by an external review: *"The genuinely impressive missing piece is the
deferred headless Docker/Gazebo CI job... promote it to Week 5-6, budget 2-3 days, and timebox it
hard."* This doc is the committed plan plus an honest account of what got built vs. what still needs a
human. **Owner: `devops-reliability-engineer`.** Does not edit `docs/ROADMAP.md` — `product-lead` owns
that file; see the one-paragraph status at the bottom of this doc for them to paste in.

## Feasibility verdict on headless-render CI on GitHub-hosted runners (the crux question)

**Short answer: full Gazebo Harmonic + ArduPilot SITL + ROS 2, with real camera/thermal sensor
rendering, on a GitHub-hosted runner is not proven feasible — and there is no known precedent for it,
including from the authoritative upstream maintainers of these exact components.** This is not a guess;
it's grounded in three checks run before writing any YAML:

1. **`ArduPilot/ardupilot_gazebo`'s own CI** (`.github/workflows/ubuntu-build.yml`, `ccpcheck.yml`,
   `ccplint.yml`) runs on `ubuntu-22.04` hosted runners but is **build + lint + static-analysis only**
   — it compiles the plugin against an apt-installed Gazebo, with `ccache`, and stops. It never
   launches Gazebo or ArduPilot SITL. If the team that owns the plugin doesn't run a live sim on hosted
   runners, that's the strongest available signal about what's actually load-bearing here.
2. **`ArduPilot/ardupilot`'s own SITL autotest CI** (`test_sitl_copter.yml` etc.) *does* run real
   flights on hosted `ubuntu-22.04` runners — but that's **plain built-in SITL physics, no Gazebo, no
   rendering**, in a prebuilt `ardupilot-dev-base` container with `ccache` and a test matrix split into
   groups to fit the time budget. This is good evidence that **SITL flight alone is hosted-runner
   feasible**; it says nothing about Gazebo rendering.
3. **Resource math**: `docs/WEEK1_BRINGUP.md`'s own documented minimum spec is **≥6 CPUs, ≥8GB RAM,
   ≥40GB disk**, and the workspace build has **already OOM'd once** at higher parallelism on a Docker
   Desktop config *more generously resourced* than that minimum. GitHub-hosted public `ubuntu-latest`
   runners are **4 vCPU / 16GB RAM / 14GB SSD** — the disk figure alone (14GB vs. a documented 40GB
   minimum, before any Gazebo world assets or a multi-GB pulled image) is close to disqualifying for a
   from-scratch build, and tight even for pulling a prebuilt image on top of a repo checkout.

**Conclusion, stated plainly (no oversell):** baking the pinned-SHA workspace into an image and
publishing it decouples the *build* from every CI run, which is the right fix for the resource
math above — but whether the resulting image, once pulled onto a hosted runner, can actually
**initialize Gazebo's headless render path (EGL + software OpenGL) and fly** is genuinely unverified
and unprecedented even by upstream. This session had no live Docker/Gazebo runner to test that
end-to-end, so the honest status is: **implemented and locally reviewed, gated to manual dispatch, not
claimed green.**

**Update, discovered mid-session (2026-08-05):** this repo's working tree is a live, parallel
workstream — `robotics-sim-engineer` landed the ADR-007 sensor mount (`fg_sensor_mount`: a co-located
RGB + thermal camera pair) into `sim/worlds/farmguard_field.sdf` and `product-lead` re-cut
`docs/ROADMAP.md`'s Weeks 5-6 plan **while this CI task was in progress** (all uncommitted at the time
of writing). Two things follow from that, both making the render crux **more** relevant, not less:
1. `docs/ROADMAP.md`'s new Weeks 5-6 table explicitly sequences this CI job (item 4) **after** a
   thermal-system "kill switch" check, the two-sensor pixel smoke test, and the ADR-003 real-render
   re-confirmation (items 1-3) — all in one batched human Docker session, same discipline as
   `docs/WEEK3_VALIDATION.md`'s gates. **This CI job should not be the first thing that touches the
   real render** — let items 1-3 prove the render works at all before layering CI automation on top of
   it. If item 1 (does `gz-sim-thermal-system` even load on the pinned Harmonic+ogre2 stack) fails, this
   whole plan doc's scope shrinks to "flight only, camera sensors physically removed from the CI
   world" — see the cut-list's new item 6.
2. Because the sensor mount is attached to the **same vehicle model** (`iris_with_gimbal_ndvi`) this
   smoke job flies, `gz sim`'s Sensors system will now initialize and render **two camera-type sensors
   every frame regardless of whether this smoke job's mission needs their topics** — Gazebo renders
   every attached sensor unconditionally, not just subscribed ones. The "no camera render needed for a
   flight-only smoke" scope-reduction this doc originally argued for (see `Dockerfile.ci`'s and
   `scripts/ci_sim_smoke.sh`'s comments) **no longer avoids the ogre2 rendering path** — it only avoids
   the ROS 2/DDS bridge and the `ndvi_node`. The rendering crux is now unavoidable even for this job,
   which is exactly why item 6 below exists as an escape hatch.

**Realistic fallback if hosted runners can't do it:** a **self-hosted runner** registered against the
same machine/Docker config already proven in `docs/WEEK3_VALIDATION.md` (the human's own Docker
Desktop session, or a cloud VM with the documented minimum spec). This sidesteps the resource ceiling
entirely because it's the same environment already known to work — the cost is an operational one (a
machine has to be online to pick up the job), which is a reasonable tradeoff to document for a
portfolio CI but is **not implemented here** — it needs a persistently-available machine, which is the
human's call, not a code change.

## What was implemented this session (no live runner available)

| Artifact | What it does | Verified how |
|---|---|---|
| `sim/docker/Dockerfile.ci` | Bakes ROS 2 Humble + Gazebo Harmonic + `ardupilot_gazebo`/`ardupilot_gz` + ArduPilot SITL (pinned to the **exact commit SHAs** in `CLAUDE.md`, not floating branches) into image layers, `--enable-DDS` deliberately **excluded** (see cut-list #1) | Manual review only. **Not built** in this session — see "What needs the human" |
| `.github/workflows/sim-image.yml` | Builds `Dockerfile.ci`, pushes to GHCR (`ghcr.io/<owner>/fieldguard-sim`). Manual dispatch or on Dockerfile/pin changes — **decoupled from every push**, which is the direct fix for "CI shouldn't rebuild from scratch every run" | `actionlint` clean. Never executed (no live runner/GHCR credentials in this session) |
| `scripts/gen_boustrophedon.py` (reused, unchanged) | Generates the short scripted mission: `--width 20 --height 30 --spacing 15 --alt 15` → 2 lanes, 4 coverage waypoints, 7 mission items | **Executed locally**: confirmed exact output (`/tmp/ci_smoke_test.waypoints`), byte-identical geometry to the existing 6-lane mission's first 2 lanes (same generator, same seed home lat/lon — deterministic, no RNG involved in this planner) |
| `scripts/ci_sim_smoke.py` | Non-interactive pymavlink driver: connect → EKF settle → one-time FRAME_CLASS/TYPE+reboot (the documented `docs/WEEK1_BRINGUP.md` §6 gotcha) → upload mission → AUTO+arm → poll `MISSION_ITEM_REACHED` → wait disarm → write a JSON summary | **QGC-WPL parser + waypoint-counting logic executed and checked locally** against the real generated mission file (4/4 coverage waypoints, correctly excluding the home placeholder row). **The live MAVLink control flow itself is UNVERIFIED** — no SITL to connect to in this session |
| `scripts/ci_sim_smoke.sh` | Orchestrates `gz sim --headless-rendering` + `sim_vehicle.py --no-mavproxy` (reuses the exact entry point already proven interactively, not a hand-rolled binary invocation) + the Python driver, with timeouts and cleanup traps | `bash -n` syntax-checked. **Never run** — needs a live container |
| `scripts/check_sim_smoke.py` | Regression gate: reads the JSON summary, fails the build on any missed waypoint, un-armed, un-disarmed, timeout, or driver error | **Fully unit-verified locally** — 5 fixture JSONs (clean pass, missed waypoint, driver error, never-disarmed/timeout, missing file) all produce the expected exit code and failure reasons. This is the one artifact in this list with the same "run it, don't just write it" discipline applied to every other CI gate in this repo |
| `.github/workflows/ci.yml` `build-test-sim` job | Pulls the GHCR image, runs the smoke script + gate, uploads artifacts. **Gated to `workflow_dispatch` only** — will not run on every push until a human confirms one green run | `actionlint` clean. Never executed |

**Never claim green without proof** (standing rule for this role): none of the above has run against a
live Gazebo/SITL process. Every piece that *could* be exercised without one — the mission generator,
the WPL parser, the regression-gate script — was actually run, not just read for plausibility. Every
piece that needs a live process is explicitly marked unverified in its own file header, with the
single riskiest unverified assumption called out inline (the `tcp:127.0.0.1:5760` default connection
port for SITL without MAVProxy — flagged as the first thing to check if the driver can't connect).

## What needs the human (in order)

1. **Build `Dockerfile.ci` once, locally**, on the already-proven Docker Desktop setup
   (`docker build -f sim/docker/Dockerfile.ci -t fieldguard-sim:ci .`, after building
   `sim/docker/Dockerfile` first). Watch for: disk usage, whether the colcon build OOMs at `-j2`, and
   whether the non-interactive `./waf configure --board sitl && ./waf copter` step produces a binary
   that boots and flies like the one already proven in `docs/WEEK3_VALIDATION.md`.
2. **Run `scripts/ci_sim_smoke.sh` inside that image**, manually, watching the raw logs. This is where
   the unverified assumptions (SITL's default port, the FRAME_CLASS/TYPE reboot timing, `gz sim
   --headless-rendering` actually initializing on this host) get resolved. Fix forward in the script
   itself — the structure (driver writes JSON, gate script grades it) should not need to change even if
   the MAVLink sequencing does. **Sequencing note (see the "Update, discovered mid-session" callout
   above): do this AFTER `docs/ROADMAP.md`'s Week-5 items 1-3 (thermal kill-switch, pixel smoke test,
   ADR-003 real-render re-confirmation) in the same batched Docker session** — those checks answer
   "does the render even load on this stack," which this smoke job now depends on whether it wants to
   or not, since the vehicle model it flies carries the camera/thermal sensor pair.
3. **Manually trigger `sim-image.yml`** (`workflow_dispatch`) on the actual repo, and separately
   **manually trigger `ci.yml`'s `build-test-sim` job**, to learn whether the hosted-runner resource
   math above (§ Feasibility verdict) is actually disqualifying or just tight. If it fails on disk/OOM/
   timeout, that confirms the fallback (self-hosted runner) is the real next step, not a bigger hosted
   tier (GitHub doesn't currently offer more disk on the same free/standard tier for a portfolio repo).
4. Only after a **confirmed green run**, flip `build-test-sim`'s `if: github.event_name ==
   'workflow_dispatch'` to run on every push/PR (or `main`-only, human's call) — per this role's
   standing feedback: never claim CI is green from a plausible-looking YAML change; verify, then widen
   the gate.

## Timebox breakdown (2-3 days, hard cap — per the review's explicit instruction)

| Day | Scope | Done this session? |
|---|---|---|
| **1** | `Dockerfile.ci` (pinned SHAs, workspace baked in, DDS excluded) + `sim-image.yml` (GHCR publish, decoupled from push cadence) | ✅ written + `actionlint`-clean; ⏳ never built/run (needs the human, step 1 above) |
| **2** | Scripted mission (reuse `gen_boustrophedon.py`, no new flags) + non-interactive driver (`ci_sim_smoke.py`/`.sh`) + regression gate (`check_sim_smoke.py`) + wire `build-test-sim` into `ci.yml`, manual-dispatch-gated | ✅ written; mission generator + WPL parser + gate script **actually executed and verified**; live MAVLink flow ⏳ unverified (needs the human, step 2 above) |
| **3 (buffer)** | Human runs steps 1-4 above, fixes whatever breaks (something will — this stack has broken in a new way at every prior gate: 6 bugs at Week 3's Docker session alone), flips the trigger once green | Not done — explicitly the human's slot, held open on purpose rather than guessed at |

**If Day 3 isn't enough**, that is the timebox working as intended, not a failure: stop, ship what's
green (even if that's "the image builds and boots SITL, but the mission driver needs another pass"),
and apply the cut-list below rather than let this bleed into Week 6's NDVI-pipeline work — the review's
whole point was "Weeks 5-6 slipping is the #1 project risk."

## Cut-list (apply in this order if the timebox runs long)

1. **Cut the CI job from running automatically at all.** Ship `Dockerfile.ci` + the scripts as
   documented, human-runnable infrastructure (`docs/WEEK1_BRINGUP.md`-style, run by hand in the
   container) without a working hosted-runner CI trigger. The resume-relevant claim becomes "a
   reproducible headless-sim image + a scripted regression-gated smoke test exist and run
   reproducibly" rather than "GitHub Actions runs it on every push" — still real, still defensible,
   just honestly scoped to what a hosted runner can actually do. This is the single biggest,
   highest-leverage cut if the resource math in the Feasibility verdict turns out to be disqualifying.
2. **Cut `--enable-DDS` / the ROS 2 avoidance loop from the CI smoke entirely** (already the default
   scope decision, not just a fallback — see `Dockerfile.ci`'s own header). The avoidance loop is
   already unit-tested sim-agnostically in the no-Docker `planning-and-eval` job; the Gazebo job's only
   remaining job is proving "the built asset pipeline (world, mission, SITL, pinned versions) actually
   boots and flies," which doesn't need DDS.
3. **Cut Docker image hardening** (non-root user, multi-stage build, minimized layers). Ship a
   single-stage, root-user image — exactly what `sim/docker/Dockerfile` (the Week 1 image) already is,
   consistent with this repo's existing risk posture, not a new regression.
4. **Cut `ccache` persistence across CI runs** (GH Actions `actions/cache` wiring for `~/.ccache`).
   `ccache` is still installed and on `PATH` (helps local iterative rebuilds); just accept a cold cache
   on every `sim-image.yml` run rather than spending time wiring cross-run cache persistence.
5. **Cut the eval-gate hook entirely for now** (see below) — it was never going to be built this
   iteration; this line exists so a future pass doesn't have to rediscover that it's still open.
6. **If camera rendering makes even the flight-only smoke unreliable/too slow in CI** (a real risk now
   that the vehicle model carries a live RGB+thermal sensor pair — see the "Update, discovered
   mid-session" callout above): temporarily comment out `farmguard_field.sdf`'s `fg_sensor_mount`
   link/joint for a **CI-only** copy of the world (e.g. `sim/worlds/farmguard_field_ci_smoke.sdf`,
   generated or hand-trimmed), so this job proves flight/world/build integration without paying the
   render cost at all, while the real render stays exercised only in the human's batched Docker
   session (`docs/ROADMAP.md`'s items 1-3). Not built now — only reach for this if item 2 above (a
   confirmed-green run) shows rendering is actually the blocker, not a hypothetical one.

## Eval-gate hook (not built — explicitly deferred, not silently forgotten)

The task asked for a hook, not an implementation, and as of this session there is nothing PROVEN to
gate on yet. **Correction (discovered mid-session, see the callout above): ADR-007's sensor mount
(`fg_sensor_mount`, co-located RGB + thermal cameras) has already landed in
`sim/worlds/farmguard_field.sdf`, uncommitted, by a parallel `robotics-sim-engineer` session** — an
earlier draft of this section said no camera/thermal sensors existed in the world file; that was true
when first checked and is no longer true. What has NOT happened yet is **live verification** that the
render actually works (`docs/ROADMAP.md`'s Week-5 items 1-3: does `gz-sim-thermal-system` load, do
canopy/soil/bird pixels differ, does `eval/run_spike.sh` re-confirm ADR-003 on real frames) — those are
explicitly sequenced in `docs/ROADMAP.md` as a human Docker session that comes **before** this CI job
(item 4). Once that session lands real, scored render output, the natural extension point is:
`build-test-sim` runs `eval/run_spike.sh` against the real render's output (already a drop-in per
`sim/spike/README.md`'s schema — a deliberate design choice at spike time, not new work) and gates on
FNR regression via the same pattern as `scripts/check_spike_regression.py`. Left as a comment in
`ci.yml` at the `build-test-sim` job, not implemented — building it now would mean testing against a
render this session could not itself verify, which is exactly the kind of premature/unverifiable work
this role's standing discipline says not to do. **This is a sequencing decision, not a scope gap**: the
render-verification session is `docs/ROADMAP.md` items 1-3, owned by `robotics-sim-engineer` +
`perception-ml-engineer`; this CI job is item 4, downstream of it by design.

## Resource sanity (for whoever runs this next)

Restating `docs/WEEK1_BRINGUP.md`'s minimum spec here because it's the actual gating fact for this
task: **≥6 CPUs, ≥8GB RAM, ≥40GB disk** for the interactive bringup. `Dockerfile.ci`'s baked build adds
image-layer overhead on top of that (a full ROS 2 Humble desktop install + Gazebo Harmonic + a compiled
ArduPilot binary easily reaches several GB even before any world assets) — budget a full local build at
**30-90+ minutes** on a reasonably provisioned machine, and expect the first attempt to fail on
something not yet seen (consistent with every prior gate in this project: 6 new bugs surfaced at the
Week 3 Docker session alone). Don't schedule this as the last thing done before a demo.

## One-paragraph status for `product-lead` to paste into `docs/ROADMAP.md`

> **Weeks 5-6 CI (2026-08-05, devops-reliability-engineer):** promoted from deferred to committed per
> external review, hard-timeboxed at 2-3 days (`docs/WEEK5_CI_GAZEBO.md`). Implemented everything
> verifiable without a live Docker/Gazebo runner: `sim/docker/Dockerfile.ci` (pinned-SHA workspace
> baked into image layers, DDS/ROS2-avoidance-loop deliberately excluded from this scope),
> `.github/workflows/sim-image.yml` (GHCR publish, decoupled from per-push cadence), a non-interactive
> 2-lane scripted smoke mission (`scripts/ci_sim_smoke.{sh,py}`, reusing the existing mission generator
> unchanged) and a fully unit-verified regression gate (`scripts/check_sim_smoke.py` — 5/5 fixture
> cases pass/fail correctly). Wired into `ci.yml`'s `build-test-sim` job, **manual-dispatch-only until a
> human confirms one green run** — none of the live-Gazebo/SITL pieces have executed yet in this
> session. Honest feasibility verdict, grounded in the upstream `ardupilot_gazebo`/`ardupilot` projects'
> own CI (they build/lint-only or run plain-SITL-no-Gazebo on hosted runners — full Gazebo rendering on
> a GitHub-hosted runner has no known precedent): this **may not fit GitHub-hosted runner resources**
> (14GB disk vs. our own documented 40GB minimum); the fallback is a self-hosted runner, not implemented
> here. Cut-list is explicit if Day 3 isn't enough — see the doc.
