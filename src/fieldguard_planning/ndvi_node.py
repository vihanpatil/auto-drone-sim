"""ROS 2 bringup node for the NDVI fusion pipeline (Weeks 5-6, ADR-007 downstream).

Wires the locked `/fg/*` contract (ADR-007) to the tested fusion core in `ndvi_fusion.py`:

    /fg/sensor/rgb/image (rgb8)  ──┐
                                    ├─▶ message_filters.ApproximateTimeSynchronizer ─▶ NdviFuser.fuse
    /fg/sensor/nir/image (mono16)──┘         (slop = ndvi_fusion.max_stamp_delta_s)      │
                                                                                          ▼
                                                        /fg/ndvi/image (32FC1, AUTHORITATIVE)
                                                        /fg/ndvi/camera_info (pass-through from rgb)
                                                        /fg/ndvi/preview (rgb8, human-only)

Everything except THIS file is sim-agnostic and unit-tested (`ndvi_fusion.py`: rescale/compute_ndvi/
the 0/0 guard, the stale-pair drop path, decode_rgb8/decode_mono16, the preview colormap). This node
only does rclpy wiring -- same "thin adapter" discipline as `avoidance_node.py`/`ros2_adapter.py`
(Week 3-4): rclpy imports lazily inside `build_node()`/`main()` so the sibling pure modules stay
importable (and testable) on a bare interpreter with no ROS 2 environment.

STATUS: NOT RUN LIVE. The render this node depends on (`/fg/sensor/rgb/image`,
`/fg/sensor/nir/image`) has not rendered yet -- gated on the human Docker session,
`docs/WEEK5_VALIDATION.md` Gates 0-2. This file is written and ready to run the moment those topics
exist; do not treat anything here as exercised against the real render until Gate 2 is green and
this node has actually been run against it (mirrors how `avoidance_node.py` was written ahead of
its own Week-3 Docker validation).

VERIFY-IN-CONTAINER items (cannot be checked outside Docker/ROS 2, same category as
`ros2_adapter.py`'s note):
  * `message_filters` is a standard ROS 2 Humble package (`ros-humble-message-filters`) but is not
    used anywhere else in this repo yet -- confirm it's on the container's install line
    (`docs/WEEK1_BRINGUP.md` / the Dockerfile) before first run; add it if missing.
  * `use_sim_time` is NOT hardcoded here (matches `avoidance_node.py`'s convention) -- launch with
    `--ros-args -p use_sim_time:=true` per ADR-007's "use_sim_time=true" requirement, or the NDVI
    frame's stamp arithmetic (delta vs. the stale-pair guard) will compare wall-clock stamps against
    sim-time stamps.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .ndvi_fusion import NdviFuser, decode_mono16, decode_rgb8, load_camera_config, ndvi_to_preview_rgb


def build_node():
    """Construct the rclpy node. Factory pattern (matches avoidance_node.build_node) so the
    (untestable-off-sim) rclpy/message_filters import stays lazy."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data  # BEST_EFFORT -- matches the Gazebo camera
    # sensor bridge's publisher QoS (same gotcha flagged in avoidance_node.py for /ap/pose/filtered).
    from sensor_msgs.msg import CameraInfo, Image
    import message_filters

    class NdviNode(Node):
        def __init__(self):
            super().__init__("fieldguard_ndvi")
            cfg = load_camera_config()
            self.fuser = NdviFuser.from_config(cfg)
            topics = cfg["camera"]["topics"]

            self.ndvi_pub = self.create_publisher(Image, "/fg/ndvi/image", 10)
            self.ndvi_info_pub = self.create_publisher(CameraInfo, "/fg/ndvi/camera_info", 10)
            self.preview_pub = self.create_publisher(Image, "/fg/ndvi/preview", 10)

            self._rgb_info: Optional[CameraInfo] = None
            self.create_subscription(CameraInfo, topics["rgb_camera_info"], self._on_rgb_info,
                                     qos_profile_sensor_data)

            rgb_sub = message_filters.Subscriber(self, Image, topics["rgb_image"],
                                                 qos_profile=qos_profile_sensor_data)
            nir_sub = message_filters.Subscriber(self, Image, topics["nir_image"],
                                                 qos_profile=qos_profile_sensor_data)
            # ADR-007 amendment: message_filters' own slop is set to the SAME 25%-of-period bound
            # ndvi_fusion.NdviFuser re-enforces per-pair below -- belt-and-suspenders, not redundant:
            # this slop only decides which NIR message gets HANDED to the callback as "nearest";
            # NdviFuser.fuse() independently re-checks the actual delta and is what increments
            # dropped_pair_count if the nearest available match still isn't close enough (e.g. a
            # dropped NIR frame widened the true nearest gap beyond tolerance).
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, nir_sub], queue_size=10, slop=self.fuser.max_delta_s)
            self._sync.registerCallback(self._on_pair)

            self.get_logger().info(
                f"fieldguard_ndvi up: rgb={topics['rgb_image']} nir={topics['nir_image']} "
                f"update_rate={self.fuser.update_rate_hz}Hz slop={self.fuser.max_delta_s * 1000:.1f}ms")

        def _on_rgb_info(self, msg: "CameraInfo") -> None:
            self._rgb_info = msg

        def _on_pair(self, rgb_msg, nir_msg) -> None:
            rgb_stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            nir_stamp = nir_msg.header.stamp.sec + nir_msg.header.stamp.nanosec * 1e-9
            red_u8 = decode_rgb8(rgb_msg.height, rgb_msg.width, rgb_msg.data)[:, :, 0]
            nir_u16 = decode_mono16(nir_msg.height, nir_msg.width, nir_msg.data)

            result = self.fuser.fuse(rgb_stamp, red_u8, nir_stamp, nir_u16)
            if not result.accepted:
                self.get_logger().warn(
                    f"[ndvi] dropped stale pair: delta={result.stamp_delta_s * 1000:.1f}ms > "
                    f"{self.fuser.max_delta_s * 1000:.1f}ms "
                    f"(dropped_pair_count={self.fuser.dropped_pair_count})")
                return

            # NDVI inherits the RGB stamp -- the georef anchor (ADR-007).
            out = Image()
            out.header = rgb_msg.header
            out.height, out.width = result.ndvi.shape
            out.encoding = "32FC1"
            out.is_bigendian = 0
            out.step = out.width * 4
            out.data = np.ascontiguousarray(result.ndvi, dtype=np.float32).tobytes()
            self.ndvi_pub.publish(out)

            if self._rgb_info is not None:
                info = self._rgb_info
                info.header = rgb_msg.header
                self.ndvi_info_pub.publish(info)

            preview = Image()
            preview.header = rgb_msg.header
            preview.height, preview.width = result.ndvi.shape
            preview.encoding = "rgb8"
            preview.is_bigendian = 0
            rgb_prev = ndvi_to_preview_rgb(result.ndvi)
            preview.step = preview.width * 3
            preview.data = np.ascontiguousarray(rgb_prev, dtype=np.uint8).tobytes()
            self.preview_pub.publish(preview)

            if self.fuser.fused_count % 25 == 1:  # heartbeat, not every frame (avoid log spam)
                self.get_logger().info(
                    f"[ndvi] fused_count={self.fuser.fused_count} "
                    f"dropped_pair_count={self.fuser.dropped_pair_count} "
                    f"zero_denom_count={result.zero_denom_count}")

        def status(self) -> dict:
            return {"fused_count": self.fuser.fused_count,
                   "dropped_pair_count": self.fuser.dropped_pair_count}

    if not rclpy.ok():
        rclpy.init()
    return rclpy, NdviNode()


def main(argv=None):
    rclpy, node = build_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"fieldguard_ndvi shutting down: {node.status()}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
