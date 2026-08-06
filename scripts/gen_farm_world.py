#!/usr/bin/env python3
"""Generate the FieldGuard farm world SDF + regenerate the static-obstacle geofence export.

Single generator, run once, produces two artifacts from one in-memory tree list so they can never
drift apart (ADR-001: the geofence export IS the contract flight-software consumes, and it must
always describe exactly what's physically in the Gazebo world):

  1. sim/worlds/farmguard_field.sdf      -- the Gazebo world (field ground plane, orchard tree
                                             rows as static models, scripted bird actors as
                                             dynamic obstacles, the iris_with_gimbal vehicle plus
                                             the ADR-007 dual-band NDVI sensor mount, and a
                                             per-visual <temperature> plugin on every visual in the
                                             world -- ground, every tree trunk/canopy, every bird).
  2. config/static_obstacles.json         -- rewritten in place: the 'obstacles' array is replaced
                                             with the flattened, authoritative per-tree list
                                             computed from that same file's 'layout' section.
                                             ('layout'/'tree_defaults' are the hand-authored input;
                                             'obstacles' is always generated -- don't hand-edit it.)

Reads (all data-driven, no code changes needed for a new scenario):
  - config/field_polygon.json        (field boundary, home lat/lon, mission altitude)
  - config/static_obstacles.json     (tree row layout + tree geometry defaults)
  - config/birds/farm_world_birds.json (scripted dynamic bird actor trajectories)
  - config/ndvi_camera.json          (ADR-007 dual-band sensor mount: intrinsics/rate + the
                                       per-material-class temperature calibration table)

World-level plugin set, spherical_coordinates, and the vehicle <include>/pose are copied verbatim
from ardupilot_gz's own iris_runway.sdf (the world already proven to load in this project's
docs/WEEK1_BRINGUP.md) -- deliberately not reinvented, to avoid reopening the exact "world fails to
load" class of bug that doc already fought through. Trees and the ground plane are inline SDF
primitives (box/cylinder/sphere, flat-color materials) with no external model:// or mesh
references, so this world introduces zero new GZ_SIM_RESOURCE_PATH risk beyond what iris_runway.sdf
already requires (ardupilot_gazebo's share dir, per WEEK1_BRINGUP.md Section 5).

Usage:
    python3 scripts/gen_farm_world.py
    python3 scripts/gen_farm_world.py --world-out sim/worlds/farmguard_field.sdf \\
        --obstacles-out config/static_obstacles.json

Dependency: stdlib only (json, xml.etree for a well-formedness check).
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_FIELD_POLYGON = REPO_ROOT / "config" / "field_polygon.json"
DEFAULT_STATIC_OBSTACLES = REPO_ROOT / "config" / "static_obstacles.json"
DEFAULT_BIRDS = REPO_ROOT / "config" / "birds" / "farm_world_birds.json"
DEFAULT_NDVI_CAMERA = REPO_ROOT / "config" / "ndvi_camera.json"
DEFAULT_WORLD_OUT = REPO_ROOT / "sim" / "worlds" / "farmguard_field.sdf"

GROUND_MARGIN_M = 25.0  # ground plane extends this far past the field polygon bbox on each side


def point_in_polygon(x, y, poly):
    """Standard ray-casting point-in-polygon test (works for the rectangle today and any simple
    polygon later, since config/field_polygon.json's 'polygon_m' isn't assumed to stay a rectangle)."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1):
            inside = not inside
    return inside


def compute_obstacles(static_cfg: dict, polygon_m):
    defaults = static_cfg["tree_defaults"]
    obstacles = []
    for row in static_cfg["layout"]["rows"]:
        row_id = row["row_id"]
        x = row["x_m"]
        y = row["y_start_m"]
        idx = 0
        while y <= row["y_end_m"] + 1e-6:
            if not point_in_polygon(x, y, polygon_m):
                raise ValueError(
                    f"tree row {row_id} tree #{idx} at ({x},{y}) falls outside the field polygon "
                    f"{polygon_m} -- fix config/static_obstacles.json 'layout' or "
                    f"config/field_polygon.json 'polygon_m'"
                )
            obstacles.append({
                "id": f"tree_row{row_id}_{idx}",
                "type": "tree",
                "row_id": row_id,
                "pos_m": [round(x, 3), round(y, 3), 0.0],
                "obstacle_radius_m": defaults["obstacle_radius_m"],
                "canopy_radius_m": defaults["canopy_radius_m"],
                "height_m": round(defaults["trunk_height_m"] + defaults["canopy_height_m"], 3),
            })
            idx += 1
            y += row["tree_spacing_m"]
    return obstacles


def field_bbox(polygon_m):
    xs = [p[0] for p in polygon_m]
    ys = [p[1] for p in polygon_m]
    return min(xs), max(xs), min(ys), max(ys)


# --------------------------------------------------------------------------------------
# SDF text generation (inline primitives only -- no external model:// or mesh references)
# --------------------------------------------------------------------------------------
def sdf_world_header(field_cfg: dict) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="farmguard_field">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- World-level plugin set copied verbatim from ardupilot_gz's iris_runway.sdf: this exact
         combination is already proven to load in this project (docs/WEEK1_BRINGUP.md Section 5-6);
         not reinvented here on purpose. -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>

    <scene>
      <ambient>0.9 0.9 0.9</ambient>
      <background>0.7 0.8 0.9</background>
      <sky></sky>
    </scene>

    <!-- Same home as ardupilot_gz's iris_runway.sdf, so config/missions/boustrophedon.waypoints
         (generated relative to this lat/lon) covers exactly this world without regeneration. -->
    <spherical_coordinates>
      <latitude_deg>{field_cfg['home_lat']}</latitude_deg>
      <longitude_deg>{field_cfg['home_lon']}</longitude_deg>
      <elevation>{field_cfg['home_elevation_m']}</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
"""


def sdf_temperature_plugin(temperature_k: float, indent: str = "          ") -> str:
    """ADR-007: gz-sim-thermal-system per-visual plugin -- the thing that makes a visual register a
    calibrated value on the thermal ('synthetic NIR') sensor instead of silently falling back to
    ambient temperature (the review's called-out failure mode). Must be a child of <visual>, not
    <link>/<model> (matches the pinned-branch gz-sim8/Harmonic example,
    examples/worlds/thermal_camera.sdf)."""
    return (
        f'\n{indent}<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">'
        f"\n{indent}  <temperature>{temperature_k}</temperature>"
        f"\n{indent}</plugin>"
    )


def sdf_ground_plane(polygon_m, ndvi_cfg: dict) -> str:
    x0, x1, y0, y1 = field_bbox(polygon_m)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    w = (x1 - x0) + 2 * GROUND_MARGIN_M
    h = (y1 - y0) + 2 * GROUND_MARGIN_M
    soil_temp = ndvi_cfg["temperature_calibration"]["soil"]["temperature_k"]
    return f"""
    <model name="field_ground">
      <static>true</static>
      <pose>{cx} {cy} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{w:.2f} {h:.2f}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{w:.2f} {h:.2f}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.30 0.42 0.20 1</ambient>
            <diffuse>0.30 0.42 0.20 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>{sdf_temperature_plugin(soil_temp)}
        </visual>
      </link>
    </model>
"""


def sdf_vehicle_with_ndvi_sensor(ndvi_cfg: dict) -> str:
    """The vehicle include (same URI + spawn pose as ardupilot_gz's iris_runway.sdf -- spawns at
    field polygon origin (0,0), the mission's home/first waypoint) PLUS the ADR-007 dual-band NDVI
    sensor mount, added by wrapping model://iris_with_gimbal in a new outer <model> (nested SDF
    model composition -- the same pattern iris_with_gimbal/model.sdf itself uses to attach its own
    gimbal to iris_with_standoffs) rather than forking the external, pinned-SHA vehicle model. See
    config/ndvi_camera.json ("mount") for the full derivation/verification notes on the
    parent-link scoped name -- this is the one piece of this world NOT confirmed live yet
    (docs/WEEK5_VALIDATION.md Gate 1 troubleshooting has the fallback to try if it fails to spawn).
    """
    m = ndvi_cfg["mount"]
    cam = ndvi_cfg["camera"]
    th = ndvi_cfg["thermal"]
    mx, my, mz, mroll, mpitch, myaw = m["mount_pose_xyz_rpy"]
    ixx, iyy, izz = m["mount_inertia_diag"]
    w, h = cam["image_width_px"], cam["image_height_px"]
    hfov = cam["horizontal_fov_rad"]
    rate = cam["update_rate_hz"]
    near, far = cam["clip_near_m"], cam["clip_far_m"]
    topics = cam["topics"]
    # Bridged gz<->ROS topic names must match exactly (no leading slash on the gz side; ros_gz_bridge
    # normalizes it -- see sim/bridge/fg_sensor_bridge.yaml).
    rgb_topic = topics["rgb_image"].lstrip("/")
    rgb_info_topic = topics["rgb_camera_info"].lstrip("/")
    nir_topic = topics["nir_image"].lstrip("/")
    nir_info_topic = topics["nir_camera_info"].lstrip("/")
    return f"""
    <model name="iris_with_gimbal_ndvi">
      <pose degrees="true">0 0 0.195 0 0 90</pose>
      <include>
        <uri>model://iris_with_gimbal</uri>
      </include>

      <!-- ADR-007: co-located RGB (Red band) + thermal (synthetic NIR) camera pair, ONE rigid
           nadir mount (no gimbal), rigidly fixed to the airframe (iris_with_standoffs::base_link),
           NOT the 3-axis RC gimbal. Both sensors live on this SAME link at the SAME zero-offset
           pose, so intrinsics/pose/rate are identical by construction: pixel-wise fusion needs no
           resampling. See config/ndvi_camera.json for the source values and calibration notes. -->
      <link name="{m['mount_link_name']}">
        <pose>{mx} {my} {mz} {mroll} {mpitch} {myaw}</pose>
        <inertial>
          <mass>{m['mount_mass_kg']}</mass>
          <inertia>
            <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{iyy}</iyy><iyz>0</iyz>
            <izz>{izz}</izz>
          </inertia>
        </inertial>

        <sensor name="fg_rgb_camera" type="camera">
          <pose>0 0 0 0 0 0</pose>
          <camera>
            <camera_info_topic>{rgb_info_topic}</camera_info_topic>
            <horizontal_fov>{hfov}</horizontal_fov>
            <image>
              <width>{w}</width>
              <height>{h}</height>
              <format>R8G8B8</format>
            </image>
            <clip><near>{near}</near><far>{far}</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <visualize>true</visualize>
          <topic>{rgb_topic}</topic>
        </sensor>

        <sensor name="fg_nir_camera" type="thermal">
          <pose>0 0 0 0 0 0</pose>
          <camera>
            <camera_info_topic>{nir_info_topic}</camera_info_topic>
            <horizontal_fov>{hfov}</horizontal_fov>
            <image>
              <width>{w}</width>
              <height>{h}</height>
              <format>{th['format']}</format>
            </image>
            <clip><near>{near}</near><far>{far}</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <visualize>true</visualize>
          <topic>{nir_topic}</topic>
          <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
            <min_temp>{th['min_temp_k']}</min_temp>
            <max_temp>{th['max_temp_k']}</max_temp>
            <resolution>{th['resolution_k_per_count']}</resolution>
          </plugin>
        </sensor>
      </link>

      <joint name="fg_sensor_mount_joint" type="fixed">
        <parent>{m['parent_link_scoped_from_wrapper']}</parent>
        <child>{m['mount_link_name']}</child>
      </joint>
    </model>
"""


def sdf_tree(obstacle: dict, defaults: dict, ndvi_cfg: dict) -> str:
    x, y, _ = obstacle["pos_m"]
    trunk_r = defaults["trunk_radius_m"]
    trunk_h = defaults["trunk_height_m"]
    canopy_r = defaults["canopy_radius_m"]
    canopy_z = trunk_h + defaults["canopy_height_m"] / 2.0
    name = obstacle["id"]
    calib = ndvi_cfg["temperature_calibration"]
    trunk_temp = calib["trunk"]["temperature_k"]
    canopy_temp = calib["canopy"]["temperature_k"]
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="trunk">
        <pose>0 0 {trunk_h / 2:.3f} 0 0 0</pose>
        <collision name="trunk_collision">
          <geometry><cylinder><radius>{trunk_r}</radius><length>{trunk_h}</length></cylinder></geometry>
        </collision>
        <visual name="trunk_visual">
          <geometry><cylinder><radius>{trunk_r}</radius><length>{trunk_h}</length></cylinder></geometry>
          <material>
            <ambient>0.35 0.22 0.10 1</ambient>
            <diffuse>0.35 0.22 0.10 1</diffuse>
          </material>{sdf_temperature_plugin(trunk_temp)}
        </visual>
      </link>
      <link name="canopy">
        <pose>0 0 {canopy_z:.3f} 0 0 0</pose>
        <collision name="canopy_collision">
          <geometry><sphere><radius>{canopy_r}</radius></sphere></geometry>
        </collision>
        <visual name="canopy_visual">
          <geometry><sphere><radius>{canopy_r}</radius></sphere></geometry>
          <material>
            <ambient>0.12 0.32 0.12 1</ambient>
            <diffuse>0.12 0.32 0.12 1</diffuse>
          </material>{sdf_temperature_plugin(canopy_temp)}
        </visual>
      </link>
    </model>
"""


def sdf_bird_actor(bird: dict, ndvi_cfg: dict) -> str:
    r = bird["physical_radius_m"]
    rgba = " ".join(str(c) for c in bird["color_rgba"])
    loop = "true" if bird.get("loop", True) else "false"
    bird_temp = ndvi_cfg["temperature_calibration"]["bird"]["temperature_k"]
    waypoints = "\n".join(
        f'          <waypoint><time>{wp["t_s"]:.2f}</time>'
        f'<pose>{wp["x_m"]} {wp["y_m"]} {wp["z_m"]} 0 0 {math.radians(wp["yaw_deg"]):.4f}</pose>'
        f"</waypoint>"
        for wp in bird["waypoints"]
    )
    return f"""
    <actor name="{bird['bird_id']}">
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>{r}</radius></sphere></geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>{sdf_temperature_plugin(bird_temp)}
        </visual>
      </link>
      <script>
        <loop>{loop}</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="fly">
{waypoints}
        </trajectory>
      </script>
    </actor>
"""


def build_sdf(field_cfg, static_cfg, birds_cfg, ndvi_cfg, obstacles) -> str:
    parts = [sdf_world_header(field_cfg)]
    parts.append(sdf_ground_plane(field_cfg["polygon_m"], ndvi_cfg))
    parts.append(sdf_vehicle_with_ndvi_sensor(ndvi_cfg))
    parts.append("\n    <!-- Static obstacles (ADR-001): known tree positions from a pre-flight "
                  "boundary survey, geofenced; see config/static_obstacles.json for the "
                  "machine-readable export flight-software consumes. Kept in sync with this world "
                  "by scripts/gen_farm_world.py generating both from the same layout. -->")
    for obs in obstacles:
        parts.append(sdf_tree(obs, static_cfg["tree_defaults"], ndvi_cfg))
    parts.append("\n    <!-- Scripted dynamic obstacles (2-3 birds, MVP scope); see "
                  "config/birds/farm_world_birds.json for the trajectory source data. -->")
    for bird in birds_cfg["birds"]:
        parts.append(sdf_bird_actor(bird, ndvi_cfg))
    parts.append("\n  </world>\n</sdf>\n")
    return "".join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--field-polygon", type=Path, default=DEFAULT_FIELD_POLYGON)
    ap.add_argument("--static-obstacles", type=Path, default=DEFAULT_STATIC_OBSTACLES)
    ap.add_argument("--birds", type=Path, default=DEFAULT_BIRDS)
    ap.add_argument("--ndvi-camera", type=Path, default=DEFAULT_NDVI_CAMERA)
    ap.add_argument("--world-out", type=Path, default=DEFAULT_WORLD_OUT)
    ap.add_argument("--obstacles-out", type=Path, default=None,
                     help="defaults to --static-obstacles (rewritten in place)")
    args = ap.parse_args()
    obstacles_out = args.obstacles_out or args.static_obstacles

    field_cfg = json.loads(args.field_polygon.read_text())
    static_cfg = json.loads(args.static_obstacles.read_text())
    birds_cfg = json.loads(args.birds.read_text())
    ndvi_cfg = json.loads(args.ndvi_camera.read_text())

    obstacles = compute_obstacles(static_cfg, field_cfg["polygon_m"])
    static_cfg["obstacles"] = obstacles
    obstacles_out.write_text(json.dumps(static_cfg, indent=2) + "\n")
    print(f"[gen_farm_world] {len(obstacles)} static tree obstacles -> {obstacles_out}")

    sdf_text = build_sdf(field_cfg, static_cfg, birds_cfg, ndvi_cfg, obstacles)

    # Static validation: the output must at minimum be well-formed XML. This does NOT confirm the
    # SDF is semantically valid to gz-sim (that needs the real `gz sim` parser, which only runs in
    # the human-operated Docker container -- see sim/README.md).
    try:
        ET.fromstring(sdf_text)
    except ET.ParseError as e:
        raise SystemExit(f"[gen_farm_world] generated SDF is not well-formed XML: {e}")

    args.world_out.parent.mkdir(parents=True, exist_ok=True)
    args.world_out.write_text(sdf_text)
    n_temp_visuals = 1 + 2 * len(obstacles) + len(birds_cfg["birds"])  # ground + trunk/canopy + birds
    print(f"[gen_farm_world] well-formed XML confirmed; wrote world "
          f"({len(birds_cfg['birds'])} bird actors, {len(obstacles)} trees, "
          f"ADR-007 NDVI sensor mount, {n_temp_visuals} calibrated <temperature> visuals) "
          f"-> {args.world_out}")


if __name__ == "__main__":
    main()
