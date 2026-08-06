#!/usr/bin/env python3
"""ADR-007 Gate 2: the canopy-vs-soil-vs-bird pixel smoke test.

*** THIS IS THE GATE BEFORE ANY NDVI-STITCH WORK BEGINS. *** Not run yet -- see
docs/WEEK5_VALIDATION.md. Everything in this file is source/doc-verified only until a human runs
it inside the fieldguard-sim container.

What this proves (and why it's the direct proof option (c) in ADR-007 could never produce): it
samples the RAW `/fg/sensor/nir/image` band -- Gazebo's thermal sensor repurposed as synthetic NIR
-- and shows that canopy, bare soil, and a bird actor read back three genuinely different,
well-separated values. Because the NIR band comes from a per-object <temperature> authored in SDF
(config/ndvi_camera.json's calibration table, baked into sim/worlds/farmguard_field.sdf by
scripts/gen_farm_world.py) and NOT derived from the visible render, this is proof the two bands are
independent -- a post-processed-from-RGB synthetic NIR (the rejected option (c)) could never show
three genuinely different classes this way, since by construction its NIR would just be a function
of the same RGB pixels it's being compared against.

Deliberately does NOT compute the fused NDVI=(NIR-Red)/(NIR+Red) formula or build the future
`ndvi_node` (flight-software's Weeks 5-6 scope, downstream of this gate) -- it works on the raw
NIR band, per ADR-007's own Gate 2 wording ("Sample NDVI (or the raw NIR band)"). A lightweight
red-channel-based NDVI proxy is computed too (see --skip-ndvi-proxy) as a bonus cross-check, but the
PASS/FAIL verdict is decided on the raw NIR band alone, which is the simpler and more direct claim.

How to run (inside the fieldguard-sim container, ROS 2 sourced, AFTER Gate 0 and Gate 1 pass --
see docs/WEEK5_VALIDATION.md):

    1. Gazebo + the ros_gz bridge (sim/bridge/fg_sensor_bridge.yaml) must already be running.
    2. Fly the EXISTING boustrophedon mission (scripts/run_farm_mission.sh, unchanged from
       Weeks 1-4) -- this script does NOT drive the vehicle itself. Reusing the already-proven
       mission flight (rather than inventing a new gz-teleport procedure) is deliberate: the
       mission's lane at x=15m has a >=63 deg-hfov ground footprint of ~+/-9.2m at 15m altitude,
       which fully covers bird_0's entire x=20m, y=[5,55]m track (config/birds/farm_world_birds.json)
       -- bird_0 loops every 16.67s and the drone dwells on that lane for much longer than one loop,
       so a bird-in-frame event is a near-geometric-certainty during a single mission run, not luck.
    3. In a 5th shell: `python3 scripts/check_ndvi_bands.py` (defaults to a 240s window, generous
       for one full 6-lane mission).

Exit code 0 = PASS (canopy/soil/bird raw-NIR means are all present and separated by at least
--min-rho-gap); exit code 1 = FAIL, or "inconclusive" (some class never observed -- rerun, or widen
--tolerance / --duration-s; a true FAIL, where classes ARE observed but not separated, means the
temperature authoring is wrong -- see config/ndvi_camera.json).

Dependencies: numpy + rclpy (imported lazily in main(), so this file stays importable -- e.g. for
--print-calibration -- without a sourced ROS 2 environment, matching this repo's convention in
src/fieldguard_planning/avoidance_node.py).
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_NDVI_CAMERA = REPO_ROOT / "config" / "ndvi_camera.json"

NIR_TOPIC = "/fg/sensor/nir/image"
RGB_TOPIC = "/fg/sensor/rgb/image"

# Classes we classify pixels into, keyed to config/ndvi_camera.json's "temperature_calibration".
# "trunk" is authored in the world (every tree needs SOME temperature -- see gen_farm_world.py) but
# is not one of the three classes ADR-007 Gate 2 asks for, so it's tracked for visibility only and
# excluded from the PASS/FAIL verdict.
GATE_CLASSES = ("canopy", "soil", "bird")
ALL_CLASSES = ("canopy", "trunk", "soil", "bird")


def load_calibration(path: Path) -> Dict[str, dict]:
    cfg = json.loads(path.read_text())
    return cfg["thermal"], cfg["temperature_calibration"]


def decode_mono16(msg) -> np.ndarray:
    """sensor_msgs/Image, encoding='mono16' -> (H,W) uint16. Matches ros_gz_bridge's packing
    (little-endian, is_bigendian=false -- verified against the pinned ros_gz `ros2` branch's
    src/convert/sensor_msgs.cpp GZ->ROS conversion, see config/ndvi_camera.json 'thermal' note)."""
    if msg.encoding != "mono16":
        raise ValueError(f"expected mono16 on {NIR_TOPIC}, got '{msg.encoding}' -- wrong topic/bridge?")
    buf = np.frombuffer(bytes(msg.data), dtype="<u2")
    return buf.reshape(msg.height, msg.width)


def decode_rgb8(msg) -> np.ndarray:
    """sensor_msgs/Image, encoding='rgb8' -> (H,W,3) uint8."""
    if msg.encoding != "rgb8":
        raise ValueError(f"expected rgb8 on {RGB_TOPIC}, got '{msg.encoding}' -- wrong topic/bridge?")
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return buf.reshape(msg.height, msg.width, 3)


class BandStats:
    """Running per-class pixel count + mean rho_nir (and mean proxy-NDVI, when an RGB frame is
    available) across the whole capture window. Streaming (no full-frame history kept) so this
    scales to a multi-minute mission without unbounded memory growth."""

    def __init__(self, thermal_cfg: dict, calib: dict, tolerance: float):
        self.min_temp = thermal_cfg["min_temp_k"]
        self.max_temp = thermal_cfg["max_temp_k"]
        self.resolution = thermal_cfg["resolution_k_per_count"]
        self.calib = calib
        self.tolerance = tolerance
        self.pixel_count = {c: 0 for c in ALL_CLASSES}
        self.rho_sum = {c: 0.0 for c in ALL_CLASSES}
        self.ndvi_sum = {c: 0.0 for c in ALL_CLASSES}
        self.ndvi_count = {c: 0 for c in ALL_CLASSES}
        self.frames_seen = 0
        self.frames_with_rgb = 0

    def rho_nir(self, raw_u16: np.ndarray) -> np.ndarray:
        temp_k = raw_u16.astype(np.float64) * self.resolution
        rho = (temp_k - self.min_temp) / (self.max_temp - self.min_temp)
        return np.clip(rho, 0.0, 1.0)

    def ingest(self, nir_raw: np.ndarray, rgb: Optional[np.ndarray]) -> None:
        self.frames_seen += 1
        rho = self.rho_nir(nir_raw)
        red_norm = None
        if rgb is not None and rgb.shape[:2] == rho.shape:
            self.frames_with_rgb += 1
            red_norm = rgb[:, :, 0].astype(np.float64) / 255.0

        for cls in ALL_CLASSES:
            target = self.calib[cls]["rho_nir"]
            mask = np.abs(rho - target) <= self.tolerance
            n = int(mask.sum())
            if n == 0:
                continue
            self.pixel_count[cls] += n
            self.rho_sum[cls] += float(rho[mask].sum())
            if red_norm is not None:
                nir_m = rho[mask]
                red_m = red_norm[mask]
                proxy_ndvi = (nir_m - red_m) / (nir_m + red_m + 1e-6)
                self.ndvi_sum[cls] += float(proxy_ndvi.sum())
                self.ndvi_count[cls] += n

    def mean_rho(self, cls: str) -> Optional[float]:
        n = self.pixel_count[cls]
        return (self.rho_sum[cls] / n) if n else None

    def mean_ndvi(self, cls: str) -> Optional[float]:
        n = self.ndvi_count[cls]
        return (self.ndvi_sum[cls] / n) if n else None

    def report_line(self) -> str:
        parts = [f"frames={self.frames_seen} (rgb-paired={self.frames_with_rgb})"]
        for cls in ALL_CLASSES:
            mr = self.mean_rho(cls)
            mr_s = f"{mr:.3f}" if mr is not None else "  --"
            parts.append(f"{cls}: n={self.pixel_count[cls]:>8d} mean_rho={mr_s}")
        return " | ".join(parts)

    def verdict(self, min_rho_gap: float):
        """Returns (passed: bool, reason: str)."""
        missing = [c for c in GATE_CLASSES if self.pixel_count[c] == 0]
        if missing:
            return False, (
                f"INCONCLUSIVE: never observed any pixel for class(es) {missing} within "
                f"+/-{self.tolerance} of their calibrated rho_nir -- rerun (birds loop "
                "continuously so a longer --duration-s should catch one), or widen --tolerance."
            )
        canopy, soil, bird = (self.mean_rho(c) for c in GATE_CLASSES)
        gap_canopy_soil = canopy - soil
        gap_soil_bird = soil - bird
        if gap_canopy_soil < min_rho_gap or gap_soil_bird < min_rho_gap:
            return False, (
                f"FAIL: classes were observed but not materially separated "
                f"(canopy-soil gap={gap_canopy_soil:.3f}, soil-bird gap={gap_soil_bird:.3f}, "
                f"need >= {min_rho_gap:.3f} each) -- check config/ndvi_camera.json's per-visual "
                "temperature authoring; a near-zero gap usually means some visuals fell back to "
                "ambient temperature instead of their calibrated value."
            )
        return True, (
            f"PASS: canopy({canopy:.3f}) > soil({soil:.3f}) > bird({bird:.3f}), "
            f"gaps canopy-soil={gap_canopy_soil:.3f} soil-bird={gap_soil_bird:.3f} "
            f"(both >= {min_rho_gap:.3f}). The raw NIR band genuinely distinguishes all three "
            "classes -- independent of the RGB/Red band."
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ndvi-camera", type=Path, default=DEFAULT_NDVI_CAMERA)
    ap.add_argument("--duration-s", type=float, default=240.0,
                     help="capture window; default comfortably exceeds one full 6-lane mission")
    ap.add_argument("--report-every-s", type=float, default=15.0)
    ap.add_argument("--tolerance", type=float, default=0.05,
                     help="rho_nir +/- window around each calibrated class value to count a pixel "
                          "as belonging to that class")
    ap.add_argument("--min-rho-gap", type=float, default=0.08,
                     help="minimum required mean_rho gap between canopy>soil and soil>bird for PASS "
                          "(calibration guarantees >=0.15 each -- see config/ndvi_camera.json -- "
                          "0.08 leaves slack for render blending on object edges/antialiasing)")
    ap.add_argument("--out", type=Path, default=None, help="optional: dump the final summary as JSON")
    ap.add_argument("--print-calibration", action="store_true",
                     help="print the loaded calibration table and exit (no ROS 2 needed)")
    args = ap.parse_args()

    thermal_cfg, calib = load_calibration(args.ndvi_camera)

    if args.print_calibration:
        print(json.dumps({"thermal": thermal_cfg, "temperature_calibration": calib}, indent=2))
        return 0

    # rclpy imported lazily so --print-calibration works without a sourced ROS 2 environment.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    stats = BandStats(thermal_cfg, calib, args.tolerance)
    state = {"latest_rgb": None}

    rclpy.init()
    node = Node("fg_check_ndvi_bands")

    def on_rgb(msg: Image) -> None:
        state["latest_rgb"] = decode_rgb8(msg)

    def on_nir(msg: Image) -> None:
        nir = decode_mono16(msg)
        stats.ingest(nir, state["latest_rgb"])

    node.create_subscription(Image, RGB_TOPIC, on_rgb, 5)
    node.create_subscription(Image, NIR_TOPIC, on_nir, 5)

    print(f"[check_ndvi_bands] subscribed to {RGB_TOPIC} and {NIR_TOPIC}; capturing for "
          f"{args.duration_s:.0f}s (Ctrl-C to stop early and print the verdict so far)")

    t_start = time.monotonic()
    t_last_report = t_start
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

    try:
        while not stop["flag"] and (time.monotonic() - t_start) < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - t_last_report >= args.report_every_s:
                print(f"[check_ndvi_bands] t={now - t_start:5.1f}s  {stats.report_line()}")
                t_last_report = now
    finally:
        node.destroy_node()
        rclpy.shutdown()

    passed, reason = stats.verdict(args.min_rho_gap)
    print(f"[check_ndvi_bands] {reason}")

    if args.out:
        summary = {
            "passed": passed,
            "reason": reason,
            "frames_seen": stats.frames_seen,
            "frames_with_rgb": stats.frames_with_rgb,
            "classes": {
                c: {
                    "pixel_count": stats.pixel_count[c],
                    "mean_rho_nir": stats.mean_rho(c),
                    "mean_proxy_ndvi": stats.mean_ndvi(c),
                    "calibrated_rho_nir": calib[c]["rho_nir"],
                }
                for c in ALL_CLASSES
            },
        }
        args.out.write_text(json.dumps(summary, indent=2) + "\n")
        print(f"[check_ndvi_bands] wrote summary -> {args.out}")

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
