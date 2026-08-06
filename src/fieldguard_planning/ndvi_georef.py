"""Georeferenced NDVI stitch (Weeks 5-6, ADR-007 downstream) -- THE LOAD-BEARING SEAM.

Tech-lead's standing flag: a single sign/frame error here yields a plausible-but-wrong map that NO
test catches unless the transform is checked against hand-computed fixtures, not just round-trip
self-consistency. This module was built test-first against `tests/fieldguard_planning/
test_ndvi_georef.py` -- read that file's fixture derivations alongside this one; they are the actual
point of this deliverable, more than the code itself.

Pipeline (composes with the frame contracts already locked for the avoidance loop -- nothing about
those is reinvented here):

    NDVI pixel (u, v)
        -> normalized camera-frame ray            (pinhole intrinsics, config/ndvi_camera.json)
        -> body-frame ray                          (fixed nadir-mount extrinsic quat_wxyz=(0,1,0,0):
                                                     "camera X=body+X, Y=body-Y, Z=body-Z",
                                                     config/ndvi_camera.json "mount")
        -> world-ENU ray                           (drone orientation quaternion from
                                                     /ap/pose/filtered; per ADR-005 the message's
                                                     frame_id LIES -- content is world-ENU, and this
                                                     module trusts content only, never frame_id)
        -> ground intersection                     (flat-field assumption, local-ENU z=0 -- see
                                                     "FLAT-FIELD ASSUMPTION" below)
        -> lat/lon                                 (`ros2_adapter.enu_to_geodetic`, ALREADY TESTED
                                                     and used by the avoidance executor's setpoint
                                                     conversion -- reused here verbatim, not
                                                     reimplemented, so the mission planner and the
                                                     georef stitch share exactly one ENU<->geodetic
                                                     transform. Anchored to /ap/gps_global_origin/
                                                     filtered at runtime -- ADR-005 names that as the
                                                     authoritative anchor for /ap/pose/filtered's ENU
                                                     frame; config/field_polygon.json's home_lat/lon
                                                     is only the offline/test default.)

BODY-FRAME CONVENTION: FLU (X-forward, Y-left, Z-up), world ENU (X-east, Y-north, Z-up); the pose
orientation quaternion rotates body(FLU) -> world(ENU). This is the SAME convention
`avoidance_node.py._on_pose` already assumes when it extracts yaw with the standard ENU/FLU
`atan2(2*(w*z+x*y), 1-2*(y^2+z^2))` formula -- this module is consistent with that existing,
already-live assumption, not a second one.

FLAT-FIELD ASSUMPTION (v1, explicit, not silently baked in): the ground is a single plane at
local-ENU z=0 -- matches config/field_polygon.json's coordinate_frame and every static obstacle's
z_m=0 in config/static_obstacles.json. No terrain relief is modeled. Acceptable at this project's
15 m cruise altitude and ~63 deg camera FOV over a flat sim field; a real deployment over
non-flat terrain would need a DEM here instead of a constant `ground_z_m` -- out of scope
(the task's framing: "don't over-engineer terrain").

CAMERA MOUNT OFFSET: `config/ndvi_camera.json` mounts the sensor 8 cm below the airframe origin
along body Z (`mount_pose_xyz_rpy` = [0,0,-0.08,...]). Included here (`MOUNT_OFFSET_BODY_M`,
`camera_world_position`) rather than approximated as zero, because it's already known exactly and
costs one extra rotate+add -- there's no reason to introduce error where none is required.

Dependency note: same scoped exception as `ndvi_fusion.py` -- `NdviHeatmapGrid.accumulate_frame`
samples a 2D NDVI array and is written to accept either a numpy array or a plain nested-list image
(duck-typed via `__getitem__`), but the project's real NDVI frames are numpy (`ndvi_fusion.py`
output). The single-point transform functions (`pixel_to_latlon`, `world_enu_to_pixel`, etc.) are
plain stdlib `math` -- no numpy needed for a single point, matching `mission_waypoints.py` /
`ros2_adapter.py`'s existing pure-math style.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .coverage import CoverageCell
from .ros2_adapter import enu_to_geodetic

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w) -- geometry_msgs/Quaternion field order

# ADR-007 mount extrinsic (config/ndvi_camera.json "mount"), quat_wxyz=(0,1,0,0) == 180 deg about
# body X. As a diagonal +/-1 map from camera-frame axes to body-frame axes:
#   camera X = body +X   camera Y = body -Y   camera Z = body -Z
# This is self-inverse (each entry is its own inverse), so the same tuple maps camera->body AND
# body->camera; `camera_ray_to_body` and `world_ray_to_camera_frame` both use it directly.
CAMERA_TO_BODY_SIGNS: Vec3 = (1.0, -1.0, -1.0)

# config/ndvi_camera.json mount.mount_pose_xyz_rpy translation component (body-frame metres).
MOUNT_OFFSET_BODY_M: Vec3 = (0.0, 0.0, -0.08)

DEFAULT_GROUND_Z_M = 0.0  # flat-field assumption, see module docstring


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics. `fx`/`fy` derived from `config/ndvi_camera.json`'s
    horizontal_fov_rad/image_width_px (its own docstring already checks this reproduces fx=fy~=520px
    -- verified again independently in test_ndvi_georef.py's fixture derivation).  cx/cy default to
    the image center (no distortion/principal-point calibration offset modeled -- v1 assumption)."""
    width_px: int
    height_px: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_config(cls, width_px: int, height_px: int, horizontal_fov_rad: float,
                    cx: Optional[float] = None, cy: Optional[float] = None) -> "CameraIntrinsics":
        fx = fy = (width_px / 2.0) / math.tan(horizontal_fov_rad / 2.0)
        return cls(width_px=width_px, height_px=height_px, fx=fx, fy=fy,
                   cx=cx if cx is not None else width_px / 2.0,
                   cy=cy if cy is not None else height_px / 2.0)


# --------------------------------------------------------------------------------------------------
# Rotation helpers (pure stdlib math; no numpy needed for a single point/ray)
# --------------------------------------------------------------------------------------------------
def _quat_to_rotation_matrix(q: Quat) -> Tuple[Vec3, Vec3, Vec3]:
    """(x, y, z, w) -> 3x3 body-to-world rotation matrix (row-major tuples). Defensively
    re-normalizes (a slightly denormalized quaternion from a message is a real-world possibility,
    not a hypothetical)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def rotate_body_to_world(v_body: Vec3, orientation_q: Quat) -> Vec3:
    r = _quat_to_rotation_matrix(orientation_q)
    x, y, z = v_body
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z,
        r[1][0] * x + r[1][1] * y + r[1][2] * z,
        r[2][0] * x + r[2][1] * y + r[2][2] * z,
    )


def rotate_world_to_body(v_world: Vec3, orientation_q: Quat) -> Vec3:
    """Inverse of `rotate_body_to_world` -- applies R^T (rotation matrices are orthonormal)."""
    r = _quat_to_rotation_matrix(orientation_q)
    x, y, z = v_world
    return (
        r[0][0] * x + r[1][0] * y + r[2][0] * z,
        r[0][1] * x + r[1][1] * y + r[2][1] * z,
        r[0][2] * x + r[1][2] * y + r[2][2] * z,
    )


# --------------------------------------------------------------------------------------------------
# Forward transform: pixel -> lat/lon
# --------------------------------------------------------------------------------------------------
def pixel_to_camera_ray(u_px: float, v_px: float, intr: CameraIntrinsics) -> Vec3:
    """Pixel -> normalized camera-frame ray direction (OpenCV/pinhole convention: X-right, Y-down,
    Z-forward-into-scene). Not unit length -- direction only; scale is resolved at ground
    intersection."""
    x = (u_px - intr.cx) / intr.fx
    y = (v_px - intr.cy) / intr.fy
    return (x, y, 1.0)


def camera_ray_to_body(ray_cam: Vec3) -> Vec3:
    sx, sy, sz = CAMERA_TO_BODY_SIGNS
    return (sx * ray_cam[0], sy * ray_cam[1], sz * ray_cam[2])


def camera_world_position(drone_position_enu: Vec3, orientation_q: Quat,
                          mount_offset_body_m: Vec3 = MOUNT_OFFSET_BODY_M) -> Vec3:
    ox, oy, oz = rotate_body_to_world(mount_offset_body_m, orientation_q)
    return (drone_position_enu[0] + ox, drone_position_enu[1] + oy, drone_position_enu[2] + oz)


def intersect_ground_enu(camera_pos_enu: Vec3, ray_world: Vec3,
                         ground_z_m: float = DEFAULT_GROUND_Z_M) -> Optional[Tuple[float, float]]:
    """Ray-plane intersection with the flat ground plane z=`ground_z_m`. Returns None if the ray
    does not point down toward the ground (camera pitched up / pointing above horizontal, or exactly
    horizontal) -- never divides by (near-)zero silently."""
    cz = camera_pos_enu[2]
    rz = ray_world[2]
    if rz >= -1e-9:
        return None
    t = (ground_z_m - cz) / rz
    if t <= 0.0:
        return None
    return (camera_pos_enu[0] + t * ray_world[0], camera_pos_enu[1] + t * ray_world[1])


def pixel_to_ground_enu(u_px: float, v_px: float, intr: CameraIntrinsics,
                        drone_position_enu: Vec3, orientation_q: Quat,
                        ground_z_m: float = DEFAULT_GROUND_Z_M,
                        mount_offset_body_m: Vec3 = MOUNT_OFFSET_BODY_M) -> Optional[Tuple[float, float]]:
    """Full pixel -> (east_m, north_m) ground point, stopping short of the lat/lon conversion (used
    directly by `NdviHeatmapGrid`, which accumulates in the same local-ENU frame `coverage.py`
    already uses)."""
    ray_cam = pixel_to_camera_ray(u_px, v_px, intr)
    ray_body = camera_ray_to_body(ray_cam)
    ray_world = rotate_body_to_world(ray_body, orientation_q)
    cam_pos = camera_world_position(drone_position_enu, orientation_q, mount_offset_body_m)
    return intersect_ground_enu(cam_pos, ray_world, ground_z_m)


def pixel_to_latlon(u_px: float, v_px: float, intr: CameraIntrinsics,
                    drone_position_enu: Vec3, orientation_q: Quat,
                    home_lat: float, home_lon: float, home_alt_m: float,
                    ground_z_m: float = DEFAULT_GROUND_Z_M,
                    mount_offset_body_m: Vec3 = MOUNT_OFFSET_BODY_M) -> Optional[Tuple[float, float]]:
    """The full pixel -> (lat_deg, lon_deg) transform. Returns None if the pixel's ray never
    intersects the ground plane (camera not pointing down enough -- should not happen for a nadir
    mount in normal flight, but the caller must handle it rather than assume every pixel resolves)."""
    ground = pixel_to_ground_enu(u_px, v_px, intr, drone_position_enu, orientation_q,
                                 ground_z_m, mount_offset_body_m)
    if ground is None:
        return None
    gx, gy = ground
    lat, lon, _ = enu_to_geodetic(gx, gy, ground_z_m, home_lat, home_lon, home_alt_m)
    return lat, lon


# --------------------------------------------------------------------------------------------------
# Inverse transform: world ENU ground point -> pixel (used by the stitch to sample cell centers)
# --------------------------------------------------------------------------------------------------
def world_ray_to_camera_frame(ray_world: Vec3, orientation_q: Quat) -> Vec3:
    ray_body = rotate_world_to_body(ray_world, orientation_q)
    sx, sy, sz = CAMERA_TO_BODY_SIGNS  # self-inverse diagonal +/-1 map, see module docstring
    return (sx * ray_body[0], sy * ray_body[1], sz * ray_body[2])


def world_enu_to_pixel(point_enu: Vec3, drone_position_enu: Vec3, orientation_q: Quat,
                       intr: CameraIntrinsics,
                       mount_offset_body_m: Vec3 = MOUNT_OFFSET_BODY_M) -> Optional[Tuple[float, float]]:
    """Inverse of `pixel_to_ground_enu` (for a ground-plane point, not an arbitrary 3D point): world
    point -> the pixel it projects to, or None if it falls behind/beside the camera (not in the
    downward viewing cone) -- the caller must still range-check against `intr.width_px/height_px`
    for "is this pixel actually in frame"."""
    cam_pos = camera_world_position(drone_position_enu, orientation_q, mount_offset_body_m)
    d_world = (point_enu[0] - cam_pos[0], point_enu[1] - cam_pos[1], point_enu[2] - cam_pos[2])
    d_cam = world_ray_to_camera_frame(d_world, orientation_q)
    if d_cam[2] <= 1e-9:
        return None
    x_n = d_cam[0] / d_cam[2]
    y_n = d_cam[1] / d_cam[2]
    return (intr.cx + x_n * intr.fx, intr.cy + y_n * intr.fy)


# --------------------------------------------------------------------------------------------------
# Stitch: accumulate per-frame NDVI footprints into the canonical coverage grid (coverage.py)
# --------------------------------------------------------------------------------------------------
class NdviHeatmapGrid:
    """Per-cell running-mean NDVI, on the SAME canonical grid `coverage.build_grid` produces --
    reused rather than reinvented (per the task's "don't reinvent the georef the boustrophedon
    planner already assumes"), so an NDVI heatmap cell and a coverage-ledger cell are the same
    `cell_id` and can be joined directly (e.g. by the eventual dashboard).

    For every accumulated frame, every canonical cell's center is projected into that frame via the
    inverse transform (`world_enu_to_pixel`); cells that land in-bounds get the frame's NDVI value at
    that pixel folded into a running mean. This is O(n_cells) per frame (a few hundred cells,
    single-digit frames/sec) -- the same complexity class `AvoidanceExecutor._at_risk_cells` already
    accepts at this project's scale, not a new performance concern.

    A cell never imaged (`mean_grid()[cell_id] is None`) is the NDVI-mapping equivalent of a
    coverage-debt cell -- explicit, not silently absent, matching the ledger discipline in
    `coverage.py`/`avoidance_executor.py`.
    """

    def __init__(self, cells: Sequence[CoverageCell], intr: CameraIntrinsics,
                mount_offset_body_m: Vec3 = MOUNT_OFFSET_BODY_M):
        self.cells: List[CoverageCell] = list(cells)
        self.intr = intr
        self.mount_offset_body_m = mount_offset_body_m
        self._sum: Dict[str, float] = {}
        self._count: Dict[str, int] = {}
        self.frames_accumulated: int = 0

    def accumulate_frame(self, ndvi_image, drone_position_enu: Vec3, orientation_q: Quat) -> int:
        """Fold one NDVI frame into the running per-cell mean. Returns the number of cells this
        frame updated (instrumentation -- a frame updating 0 cells, e.g. a bad pose, is a signal
        worth a caller logging, same spirit as `dropped_pair_count`)."""
        n_updated = 0
        for cell in self.cells:
            pix = world_enu_to_pixel((cell.cx_m, cell.cy_m, DEFAULT_GROUND_Z_M),
                                     drone_position_enu, orientation_q, self.intr,
                                     self.mount_offset_body_m)
            if pix is None:
                continue
            u, v = pix
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= ui < self.intr.width_px and 0 <= vi < self.intr.height_px):
                continue
            val = float(ndvi_image[vi][ui])
            self._sum[cell.cell_id] = self._sum.get(cell.cell_id, 0.0) + val
            self._count[cell.cell_id] = self._count.get(cell.cell_id, 0) + 1
            n_updated += 1
        self.frames_accumulated += 1
        return n_updated

    def mean_grid(self) -> Dict[str, Optional[float]]:
        """cell_id -> running-mean NDVI, or None for a cell never imaged by any accumulated frame."""
        return {
            cell.cell_id: (self._sum[cell.cell_id] / self._count[cell.cell_id]
                          if cell.cell_id in self._count else None)
            for cell in self.cells
        }

    def sample_counts(self) -> Dict[str, int]:
        """cell_id -> number of frames that contributed a sample (0 if never imaged)."""
        return {cell.cell_id: self._count.get(cell.cell_id, 0) for cell in self.cells}
