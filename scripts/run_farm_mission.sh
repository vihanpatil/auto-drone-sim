#!/usr/bin/env bash
# Fly the existing boustrophedon mission through the new farm world (sim/worlds/farmguard_field.sdf).
#
# This is Shell A of the two-shell flow sim/README.md already documents ("Launching the world") --
# this script just packages that exact, already-reviewed command into one invocation instead of
# hand-typing it, so there's one fewer place to introduce a path/env typo. It deliberately does NOT
# add a new launch path, a new GZ_SIM_RESOURCE_PATH dependency, or a ros2-launch-file wrapper --
# sim/README.md explains why the two-shell flow (not ardupilot_gz_bringup's launch files, which
# hardcode their own world path) is the right way to point at a custom world, and
# docs/WEEK1_BRINGUP.md's whole point is not reopening that class of bug.
#
# Shell B (SITL) stays a manual, interactive step on purpose: docs/WEEK1_BRINGUP.md §6 requires
# watching for the EKF-ready message and the one-time FRAME_CLASS/TYPE reboot before it's safe to
# arm, and blindly scripting past that has already cost this project debugging time once -- see
# docs/WEEK1_BRINGUP.md gotchas. This script prints the exact Shell B recipe (unchanged from
# sim/README.md / docs/WEEK1_BRINGUP.md §8) so it doesn't have to be re-found across docs.
#
# Run this INSIDE the fieldguard-sim container (scripts/sim_docker_run.sh), not on the macOS host.
#
# Usage (inside the container):
#   scripts/run_farm_mission.sh
set -euo pipefail

REPO_IN_CONTAINER="/workspace/fieldguard"
WORLD="$REPO_IN_CONTAINER/sim/worlds/farmguard_field.sdf"
MISSION="$REPO_IN_CONTAINER/config/missions/boustrophedon.waypoints"
DDS_PARAM_FILE="$REPO_IN_CONTAINER/config/sitl_params/dds_udp.parm"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found -- this script must run inside the" >&2
  echo "fieldguard-sim container (see scripts/sim_docker_run.sh), not on the macOS host." >&2
  exit 1
fi
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: $WORLD not found -- if you moved/renamed the world file, update this script and" >&2
  echo "sim/README.md together." >&2
  exit 1
fi

# colcon/ROS setup files reference unbound vars (COLCON_TRACE, AMENT_*, etc.) and are NOT safe to
# source under `set -u` — it aborts with "COLCON_TRACE: unbound variable". Drop -u just around the
# sourcing, then restore it. Standard pattern for sourcing ROS 2 overlays from a strict script.
set +u
source /root/ardu_ws/install/setup.bash
set -u
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:/root/ardu_ws/install/ardupilot_gazebo/share"

cat <<EOF
[run_farm_mission] Shell A: starting Gazebo (headless) with $WORLD
[run_farm_mission] Verify: no 'Unable to find uri' / 'Failed to load a world'
  (gz topic -l | grep model/tree_row0_0   -- confirms a tree loaded
   gz model -m bird_0 -p                  -- run twice, pose should change -- confirms birds move)

[run_farm_mission] Once the world is up, open a SECOND shell into this same container
  (docker exec -it fieldguard-sim bash) and run Shell B manually -- this mission has NOT changed
  (config/field_polygon.json already matches this world's field boundary, see sim/README.md):

  cd /root/ardu_ws/src/ardupilot
  export PATH="\$PWD/Tools/autotest:\$PATH"
  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --enable-DDS \\
    --add-param-file=$DDS_PARAM_FILE
  # --enable-DDS is REQUIRED: AP_DDS is compiled OUT of SITL by default (AP_DDS_ENABLED=0), so without
  # it the DDS_ENABLE param does not even exist and no /ap/* topics ever appear (WEEK3_VALIDATION Gate 2).

  # At the MAVProxy prompt, wait for the EKF-ready message first (docs/WEEK1_BRINGUP.md §6), then:
  mode rtl                  # land any current hover; wait for DISARMED
  wp load $MISSION
  wp list                   # confirm 15 items loaded
  param set DISARM_DELAY 0
  param set AUTO_OPTIONS 3  # bit0=allow arm in AUTO, bit1=auto-takeoff
  mode auto
  arm throttle              # arms AND auto-starts: NAV_TAKEOFF, the 6 lanes, then RTL

[run_farm_mission] Open a THIRD shell (docker exec -it fieldguard-sim bash) for the AP_DDS agent --
  needed for /ap/* ROS 2 topics (Week 3-4 perception/planner nodes), not for flying the mission
  itself (that's plain MAVLink). See docs/WEEK1_BRINGUP.md §6b / docs/DECISIONS.md for the locked
  /ap/* topic+frame contract:

  source /root/ardu_ws/install/setup.bash
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019

  # Then, from any shell with ROS 2 sourced:
  ros2 topic list | grep '^/ap'      # expect ~19 topics
  ros2 topic hz /ap/pose/filtered    # steady rate, not zero

[run_farm_mission] Open a FOURTH shell (docker exec -it fieldguard-sim bash) for the ADR-007 NDVI
  sensor bridge -- Gazebo now also carries /fg/sensor/rgb/* and /fg/sensor/nir/* (dual-band NDVI
  camera, see docs/DECISIONS.md ADR-007 and sim/README.md). NOT confirmed live yet -- run
  docs/WEEK5_VALIDATION.md's Gate 0 BEFORE Gate 1 flying above (thermal-sensor kill-switch check):

  source /root/ardu_ws/install/setup.bash
  ros2 run ros_gz_bridge parameter_bridge --ros-args \\
    -p config_file:=$REPO_IN_CONTAINER/sim/bridge/fg_sensor_bridge.yaml

  # Then, from any shell with ROS 2 sourced:
  ros2 topic list | grep '^/fg/sensor'    # expect 4 topics
  ros2 topic hz /fg/sensor/nir/image      # steady rate, not zero

  # Gate 2 (the actual proof -- run once Shell B is armed and flying, and this bridge is up):
  python3 $REPO_IN_CONTAINER/scripts/check_ndvi_bands.py

[run_farm_mission] Starting Gazebo now (Ctrl-C here stops the world) ...
EOF

exec gz sim -v4 -s -r --headless-rendering "$WORLD"
