#!/usr/bin/env bash
# Headless, non-interactive smoke flight for CI (Weeks 5-6 devops task, 2026-08-05).
#
# ⚠️ UNVERIFIED LIVE -- see docs/WEEK5_CI_GAZEBO.md for the honest feasibility verdict. Orchestrates
# the SAME two pieces docs/WEEK3_VALIDATION.md Gate 1 proves interactively (Gazebo + the farm world,
# then ArduPilot SITL flying the boustrophedon mission through it) but scripted end-to-end: no human
# at a MAVProxy prompt, no gzclient GUI, fixed timeouts throughout. Deliberately reuses the same world
# (sim/worlds/farmguard_field.sdf) and the same mission generator (scripts/gen_boustrophedon.py) as
# the rest of the project -- this is NOT a new sim path, just a shorter/scripted flight through the
# existing one (a 2-lane subset instead of the full 6-lane survey, for CI wall-clock budget).
#
# NOTE (added mid-Week-5, 2026-08-05): the world's vehicle model (iris_with_gimbal_ndvi) now carries
# the ADR-007 RGB+thermal sensor mount (landed by a parallel robotics-sim-engineer session). This
# script doesn't consume those topics (no DDS here), but Gazebo still renders both sensors every
# frame regardless of subscribers -- this smoke job no longer avoids the ogre2 rendering path just by
# skipping DDS. See docs/WEEK5_CI_GAZEBO.md's "Update, discovered mid-session" callout and cut-list
# item 6 (a camera-free CI-only world) if that turns out to make this job unreliable/too slow.
#
# NOT run from sim/docker/Dockerfile (the Week 1 interactive image) -- this needs the BAKED workspace
# from sim/docker/Dockerfile.ci (colcon build + ArduPilot SITL binary already built, no --enable-DDS,
# see that file's header for the scope cut). Run this INSIDE that image / the ghcr.io published tag.
#
# Usage (inside the fieldguard-sim:ci container, at /workspace/fieldguard):
#   scripts/ci_sim_smoke.sh
#
# Env overrides (all optional):
#   MISSION_WIDTH_M / MISSION_HEIGHT_M / MISSION_SPACING_M / MISSION_ALT_M  -- gen_boustrophedon.py args
#   SMOKE_TIMEOUT_S  -- overall flight budget passed to ci_sim_smoke.py (default 300)
#   GZ_STARTUP_WAIT_S -- fixed settle time after launching gz sim before starting SITL (default 20)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found -- this script must run inside the" >&2
  echo "fieldguard-sim CI image (sim/docker/Dockerfile.ci), not on the host." >&2
  exit 1
fi
if [[ ! -f "${ROS_WS_SETUP:-/root/ardu_ws/install/setup.bash}" ]]; then
  echo "ERROR: ${ROS_WS_SETUP:-/root/ardu_ws/install/setup.bash} not found -- the workspace was not" >&2
  echo "baked into this image (are you running the Week 1 interactive image instead of the CI one?)." >&2
  exit 1
fi

MISSION_WIDTH_M="${MISSION_WIDTH_M:-20}"
MISSION_HEIGHT_M="${MISSION_HEIGHT_M:-30}"
MISSION_SPACING_M="${MISSION_SPACING_M:-15}"
MISSION_ALT_M="${MISSION_ALT_M:-15}"
SMOKE_TIMEOUT_S="${SMOKE_TIMEOUT_S:-300}"
GZ_STARTUP_WAIT_S="${GZ_STARTUP_WAIT_S:-20}"

WORLD="$REPO_ROOT/sim/worlds/farmguard_field.sdf"
MISSION_FILE="/tmp/ci_smoke.waypoints"
RESULTS_DIR="$REPO_ROOT/eval/results"
GZ_LOG="/tmp/ci_smoke_gz.log"
SITL_LOG="/tmp/ci_smoke_sitl.log"

mkdir -p "$RESULTS_DIR"

# colcon/ROS overlay setup files reference unbound vars under `set -u` (see scripts/run_farm_mission.sh
# for the same pattern) -- drop -u just around sourcing, then restore it.
set +u
source /opt/ros/humble/setup.bash
source "${ROS_WS_SETUP:-/root/ardu_ws/install/setup.bash}"
set -u
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:/root/ardu_ws/install/ardupilot_gazebo/share"

echo "[ci_sim_smoke.sh] Generating a short (${MISSION_WIDTH_M}x${MISSION_HEIGHT_M} m, ${MISSION_SPACING_M} m spacing) mission -> $MISSION_FILE"
python3 "$REPO_ROOT/scripts/gen_boustrophedon.py" \
  --width "$MISSION_WIDTH_M" --height "$MISSION_HEIGHT_M" \
  --spacing "$MISSION_SPACING_M" --alt "$MISSION_ALT_M" \
  -o "$MISSION_FILE"
# Deliberately reuses gen_boustrophedon.py's default home lat/lon, which already matches this world's
# spherical_coordinates and config/field_polygon.json's home (see sim/README.md "Frame conventions") --
# no new coordinate risk introduced by shrinking the mission.

PIDS=()
cleanup() {
  echo "[ci_sim_smoke.sh] Cleaning up background processes..."
  for pid in "${PIDS[@]:-}"; do
    kill -9 "$pid" >/dev/null 2>&1 || true
  done
  pkill -9 -f 'arducopter' >/dev/null 2>&1 || true
  pkill -9 -f 'gz sim' >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[ci_sim_smoke.sh] Starting Gazebo headless (log: $GZ_LOG) ..."
gz sim -v4 -s -r --headless-rendering "$WORLD" > "$GZ_LOG" 2>&1 &
PIDS+=($!)

echo "[ci_sim_smoke.sh] Waiting ${GZ_STARTUP_WAIT_S}s for the world to load..."
sleep "$GZ_STARTUP_WAIT_S"
if grep -qE 'Unable to find uri|Failed to load a world' "$GZ_LOG"; then
  echo "[ci_sim_smoke.sh] FAIL: Gazebo failed to load the world -- see $GZ_LOG" >&2
  tail -n 50 "$GZ_LOG" >&2 || true
  exit 1
fi
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
  echo "[ci_sim_smoke.sh] FAIL: gz sim process died during startup -- see $GZ_LOG" >&2
  tail -n 50 "$GZ_LOG" >&2 || true
  exit 1
fi
echo "[ci_sim_smoke.sh] Gazebo world up (process alive, no load-failure strings in the log)."

echo "[ci_sim_smoke.sh] Starting ArduPilot SITL (JSON backend, gazebo-iris, no --enable-DDS -- see"
echo "  sim/docker/Dockerfile.ci header for why DDS is out of scope for this smoke job). Log: $SITL_LOG"
# Reuses sim_vehicle.py -- the SAME entry point docs/WEEK1_BRINGUP.md §6 and docs/WEEK3_VALIDATION.md
# Gate 1 already prove works, just with --no-mavproxy added so nothing tries to open an interactive
# console (sim_vehicle.py's documented flag to skip launching MAVProxy; still builds/launches the
# arducopter SITL binary itself). Deliberately NOT hand-constructing a raw `arducopter` binary
# invocation (guessing --home/--defaults flag formats) -- reusing the already-human-verified recipe
# is much lower-risk than a from-scratch one this session cannot test.
cd /root/ardu_ws/src/ardupilot
export PATH="$PWD/Tools/autotest:$PATH"
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy \
  > "$SITL_LOG" 2>&1 &
PIDS+=($!)
cd "$REPO_ROOT"
# UNVERIFIED assumption to check first if the driver can't connect: with --no-mavproxy, no MAVProxy
# re-broadcasts telemetry to udp:14550/14551 (those are MAVProxy's own output ports) -- the SITL
# binary's OWN default mavlink listen port is documented as tcp:127.0.0.1:5760, independent of
# MAVProxy, which is why ci_sim_smoke.py's --connection default below is tcp, not udp:14550. If this
# is wrong for this checkout, `grep -i "Serial\|listening\|port" "$SITL_LOG"` should show the actual
# port SITL bound.

echo "[ci_sim_smoke.sh] Waiting for SITL to build (if needed) and come up before handing off to the pymavlink driver..."
sleep 25

echo "[ci_sim_smoke.sh] Running the scripted mission driver..."
set +e
python3 "$REPO_ROOT/scripts/ci_sim_smoke.py" \
  --mission "$MISSION_FILE" \
  --connection tcp:127.0.0.1:5760 \
  --out "$RESULTS_DIR/sim_smoke.json" \
  --timeout "$SMOKE_TIMEOUT_S"
DRIVER_EXIT=$?
set -e

echo "[ci_sim_smoke.sh] Driver exited $DRIVER_EXIT. SITL tail:"
tail -n 30 "$SITL_LOG" || true

exit "$DRIVER_EXIT"
