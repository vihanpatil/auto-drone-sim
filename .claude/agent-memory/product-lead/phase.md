---
name: phase
description: Current FieldGuard week/phase and the external-review steer governing Weeks 5-6
metadata:
  type: project
---

**As of 2026-08-05: Week 5, NDVI phase — UNBLOCKED and building.**

Weeks 3-4 core (detect→avoid→return-to-next-waypoint loop) is COMPLETE and demonstrated live on the
real ArduPilot+Gazebo+ROS 2 stack. ADR-007 (thermal-sensor-as-synthetic-NIR, ACCEPTED
confirmation-pending) landed 2026-08-05 and passed external review — the one architecture risk for
Weeks 5-6 is retired.

**Why:** An external review landed 2026-08-05 and is the user's steer. Its headline: the #1 risk is NOT
that scope is too small — it's that Weeks 5-6 slip (unproven render + georeferenced stitch + deferred
ADR-003 + comparison arm) and Week 7 arrives with **no demo video and no dashboard**. Europe-trip
deadline is ~4-5 weeks out from now.

**How to apply:** Committed Week-5 ordering (no reordering, encoded in ROADMAP.md Weeks 5-6 plan):
(1) kill-switch — verify `gz-sim-thermal-system` loads on pinned Harmonic+ogre2 FIRST, before authoring
temps [robotics-sim]; (2) two-sensor mount + canopy/soil/bird pixel smoke test, watch for flat-NDVI
silent failure [robotics-sim]; (3) re-run `eval/run_spike.sh` on real render to close ADR-003, real-render
blob precision was 0.445 [perception-ml]; (4) headless Docker/Gazebo CI job promoted deferred→COMMITTED,
timeboxed 2-3 days hard [devops]. ADR-003 real-render + ADR-007's four `/fg/*` render checks batch into
ONE Docker session (same discipline as Week-3 gates). See [[scope-guards]].
