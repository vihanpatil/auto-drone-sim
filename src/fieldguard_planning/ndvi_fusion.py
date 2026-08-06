"""NDVI band fusion (Weeks 5-6, ADR-007 downstream) -- the pure-Python/numpy core of `ndvi_node`.

Mirrors the Week 3-4 avoidance-loop discipline (`avoidance_policy.py` / `avoidance_executor.py`):
sim-agnostic, unit-tested logic in this module; the thin rclpy binding (`ndvi_node.py`) only does
topic wiring and hands real messages to the functions/classes here.

Contract locked in ADR-007 (`docs/DECISIONS.md`):
  IN:  `/fg/sensor/rgb/image` (rgb8), `/fg/sensor/nir/image` (mono16), + their camera_info
  OUT: `/fg/ndvi/image` (32FC1, in/[-1,1], AUTHORITATIVE), `/fg/ndvi/camera_info`,
       `/fg/ndvi/preview` (rgb8, human-only, non-authoritative)

Pipeline this module implements:
  1. Pair an RGB frame with a NIR frame by nearest sim-time stamp (`nearest_index`,
     `pair_and_fuse_stream` for offline/batch use; the live node instead gets an
     already-nearest-matched pair from `message_filters.ApproximateTimeSynchronizer`).
  2. Enforce the ADR-007 stale-pair guard: max stamp delta = 25% of one frame period
     (`max_stamp_delta_s`). A pair exceeding it is DROPPED, never fused, and increments a logged
     `dropped_pair_count` -- CLAUDE.md's "log every ... event" rule, applied here the same way
     `avoidance_executor.py` logs every takeover/gate_reject/debt event.
  3. Rescale each band to a reflectance proxy in [0,1] (`rescale_red`, `rescale_nir` -- the latter
     per `config/ndvi_camera.json`'s thermal.decode_formula) and compute
     NDVI = (NIR-Red)/(NIR+Red) (`compute_ndvi`), with an explicit, counted sentinel for the 0/0
     case -- never a silent NaN (see NDVI_ZERO_DENOM_SENTINEL below).

NDVI inherits the RGB frame's stamp as the georef anchor (ADR-007) -- enforced in `ndvi_node.py`
(the caller), not here; this module is stamp-value-agnostic beyond the staleness comparison.

Dependency note (a DELIBERATE, SCOPED deviation from this package's usual "stdlib only" rule --
see geofence.py/coverage.py/avoidance_executor.py docstrings): this module imports numpy. Full
640x480 image fusion is impractical in pure-Python loops at any usable frame rate, and numpy is
already a first-class, project-blessed dependency for exactly this kind of pixel-array work
(requirements-eval.txt; eval/baseline_ndvi.py already consumes float32 NDVI arrays; even
scripts/check_ndvi_bands.py imports numpy eagerly at module scope). The rest of `fieldguard_planning`
(avoidance loop, coverage, geofence) stays stdlib-only on purpose -- this is a scoped exception for
the image-processing slice, not a project-wide change. Flagged for tech-lead: if this dependency
boundary should be formalized, it's a one-line DECISIONS.md note, not a design change.
Tests for this module therefore need numpy installed (already true for this repo's `eval/` venv;
see requirements-eval.txt) -- unlike the rest of `tests/fieldguard_planning`, which runs on a bare
interpreter with zero installs.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_NDVI_CAMERA_CONFIG = REPO_ROOT / "config" / "ndvi_camera.json"

# NDVI is undefined at NIR+Red==0 (both bands read zero reflectance -- degenerate, not "no
# vegetation"). We define it as a single, documented sentinel rather than emit NaN, so downstream
# accumulation (NdviHeatmapGrid) and any consumer never has to special-case NaN propagation. 0.0
# ("neutral") is chosen over -1/+1 because a 0/0 pixel carries no vegetation signal either way --
# it should not bias a stitched cell toward "healthy" or "bare/anomalous". Every occurrence is
# counted (`zero_denom_count`) and logged, per CLAUDE.md's "log every ... event" instrumentation
# rule, so a rising count is visible rather than silently absorbed.
NDVI_ZERO_DENOM_SENTINEL = 0.0


def load_camera_config(path: Path = DEFAULT_NDVI_CAMERA_CONFIG) -> dict:
    return json.loads(Path(path).read_text())


# --------------------------------------------------------------------------------------------------
# Band rescaling (config/ndvi_camera.json "camera" / "thermal" blocks)
# --------------------------------------------------------------------------------------------------
def rescale_red(red_u8: np.ndarray) -> np.ndarray:
    """R8G8B8 channel-0 (0..255) -> reflectance proxy rho_red in [0,1]."""
    return red_u8.astype(np.float64) / 255.0


def rescale_nir(nir_u16: np.ndarray, min_temp_k: float, max_temp_k: float,
                resolution_k_per_count: float) -> np.ndarray:
    """mono16 thermal raw counts -> reflectance proxy rho_nir in [0,1], per
    config/ndvi_camera.json thermal.decode_formula: T_kelvin = raw * resolution_k_per_count;
    rho_nir = clip((T_kelvin - min_temp_k) / (max_temp_k - min_temp_k), 0, 1)."""
    t_kelvin = nir_u16.astype(np.float64) * resolution_k_per_count
    span = max_temp_k - min_temp_k
    rho = (t_kelvin - min_temp_k) / span
    return np.clip(rho, 0.0, 1.0)


def compute_ndvi(rho_red: np.ndarray, rho_nir: np.ndarray) -> Tuple[np.ndarray, int]:
    """NDVI = (NIR-Red)/(NIR+Red), elementwise, with the 0/0 guard. Returns (ndvi, zero_denom_count)
    -- the count is the instrumentation signal, never silently dropped."""
    if rho_red.shape != rho_nir.shape:
        raise ValueError(f"rho_red shape {rho_red.shape} != rho_nir shape {rho_nir.shape}")
    denom = rho_nir + rho_red
    zero_mask = denom == 0.0
    zero_denom_count = int(np.count_nonzero(zero_mask))
    safe_denom = np.where(zero_mask, 1.0, denom)  # placeholder only; overwritten by the mask below
    ndvi = (rho_nir - rho_red) / safe_denom
    ndvi = np.where(zero_mask, NDVI_ZERO_DENOM_SENTINEL, ndvi)
    return ndvi.astype(np.float32), zero_denom_count


# --------------------------------------------------------------------------------------------------
# Raw sensor_msgs/Image decode (pure numpy; testable without rclpy -- pass msg.height/width/data)
# --------------------------------------------------------------------------------------------------
def decode_rgb8(height: int, width: int, data: bytes) -> np.ndarray:
    """sensor_msgs/Image encoding='rgb8' -> (height, width, 3) uint8 array."""
    arr = np.frombuffer(data, dtype=np.uint8)
    return arr.reshape((height, width, 3))


def decode_mono16(height: int, width: int, data: bytes) -> np.ndarray:
    """sensor_msgs/Image encoding='mono16' -> (height, width) uint16 array."""
    arr = np.frombuffer(data, dtype=np.uint16)
    return arr.reshape((height, width))


# --------------------------------------------------------------------------------------------------
# Stale-pair guard (ADR-007 amendment 2026-08-05)
# --------------------------------------------------------------------------------------------------
def max_stamp_delta_s(update_rate_hz: float) -> float:
    """25% of one frame period. At this project's configured 5 Hz (config/ndvi_camera.json), that's
    50 ms; at the ADR-007 amendment's 10 Hz reference example, 25 ms."""
    return 0.25 / update_rate_hz


def nearest_index(target_stamp_s: float, candidate_stamps: Sequence[float]) -> Optional[int]:
    """Index of the candidate stamp nearest `target_stamp_s`, or None if there are no candidates.
    Pure/stdlib -- the offline equivalent of what message_filters.ApproximateTimeSynchronizer does
    live for a pair of subscriptions."""
    if not candidate_stamps:
        return None
    best_i, best_d = 0, math.inf
    for i, s in enumerate(candidate_stamps):
        d = abs(s - target_stamp_s)
        if d < best_d:
            best_i, best_d = i, d
    return best_i


# --------------------------------------------------------------------------------------------------
# Fusion + instrumentation
# --------------------------------------------------------------------------------------------------
@dataclass
class FusionResult:
    accepted: bool
    ndvi: Optional[np.ndarray]
    rgb_stamp_s: float
    nir_stamp_s: float
    stamp_delta_s: float
    zero_denom_count: int
    reason: str  # "ok" | "stale_pair" | "no_nir_frames"


@dataclass(frozen=True)
class StampedFrame:
    """One decoded band frame + its sim-time stamp (seconds, float). `data` is whatever `fuse()`
    expects for that band (red_u8 2D array for RGB frames, nir_u16 2D array for NIR frames)."""
    stamp_s: float
    data: np.ndarray


class NdviFuser:
    """Stateful fusion pipeline: enforces the stale-pair guard, computes NDVI, and keeps the
    `dropped_pair_count` / `fused_count` counters + an event log -- the same "instrument everything"
    pattern as `AvoidanceExecutor` (`_log`, per-event counters). One instance per running node
    (or per offline batch job); safe to call `fuse()` repeatedly."""

    def __init__(self, update_rate_hz: float, min_temp_k: float, max_temp_k: float,
                resolution_k_per_count: float):
        self.update_rate_hz = update_rate_hz
        self.min_temp_k = min_temp_k
        self.max_temp_k = max_temp_k
        self.resolution_k_per_count = resolution_k_per_count
        self.max_delta_s = max_stamp_delta_s(update_rate_hz)

        self.dropped_pair_count: int = 0
        self.fused_count: int = 0
        self.event_log: List[dict] = []

    @classmethod
    def from_config(cls, config: Optional[dict] = None) -> "NdviFuser":
        cfg = config if config is not None else load_camera_config()
        return cls(
            update_rate_hz=float(cfg["camera"]["update_rate_hz"]),
            min_temp_k=float(cfg["thermal"]["min_temp_k"]),
            max_temp_k=float(cfg["thermal"]["max_temp_k"]),
            resolution_k_per_count=float(cfg["thermal"]["resolution_k_per_count"]),
        )

    def _log(self, kind: str, **detail) -> None:
        self.event_log.append({"seq": len(self.event_log), "kind": kind, **detail})

    def fuse(self, rgb_stamp_s: float, red_u8: np.ndarray,
             nir_stamp_s: float, nir_u16: np.ndarray) -> FusionResult:
        delta = abs(rgb_stamp_s - nir_stamp_s)
        if delta > self.max_delta_s:
            self.dropped_pair_count += 1
            self._log("dropped_pair", reason="stale_pair", rgb_stamp_s=rgb_stamp_s,
                      nir_stamp_s=nir_stamp_s, stamp_delta_s=delta, max_delta_s=self.max_delta_s,
                      dropped_pair_count=self.dropped_pair_count)
            return FusionResult(False, None, rgb_stamp_s, nir_stamp_s, delta, 0, "stale_pair")

        rho_red = rescale_red(red_u8)
        rho_nir = rescale_nir(nir_u16, self.min_temp_k, self.max_temp_k, self.resolution_k_per_count)
        ndvi, zero_denom_count = compute_ndvi(rho_red, rho_nir)
        self.fused_count += 1
        self._log("fused", rgb_stamp_s=rgb_stamp_s, nir_stamp_s=nir_stamp_s, stamp_delta_s=delta,
                  zero_denom_count=zero_denom_count, fused_count=self.fused_count)
        return FusionResult(True, ndvi, rgb_stamp_s, nir_stamp_s, delta, zero_denom_count, "ok")


def pair_and_fuse_stream(rgb_frames: Sequence[StampedFrame], nir_frames: Sequence[StampedFrame],
                         fuser: NdviFuser) -> List[FusionResult]:
    """Offline/batch pairing + fusion over two whole streams (e.g. reprocessing a rosbag/clip): for
    every RGB frame (the georef anchor, ADR-007) find the nearest NIR frame by stamp
    (`nearest_index`) and fuse. This is the pure-Python equivalent of what
    `message_filters.ApproximateTimeSynchronizer` does live in `ndvi_node.py`; the same stale-pair
    guard applies either way via `fuser.fuse()`.

    If `nir_frames` is empty, every RGB frame is counted as a dropped pair (reason
    'no_nir_frames') -- never silently skipped."""
    nir_stamps = [f.stamp_s for f in nir_frames]
    results: List[FusionResult] = []
    for rgb in rgb_frames:
        idx = nearest_index(rgb.stamp_s, nir_stamps)
        if idx is None:
            fuser.dropped_pair_count += 1
            fuser._log("dropped_pair", reason="no_nir_frames", rgb_stamp_s=rgb.stamp_s,
                      dropped_pair_count=fuser.dropped_pair_count)
            results.append(FusionResult(False, None, rgb.stamp_s, math.nan, math.inf, 0,
                                        "no_nir_frames"))
            continue
        nir = nir_frames[idx]
        results.append(fuser.fuse(rgb.stamp_s, rgb.data, nir.stamp_s, nir.data))
    return results


# --------------------------------------------------------------------------------------------------
# Human-only false-color preview (/fg/ndvi/preview -- ADR-007: non-authoritative)
# --------------------------------------------------------------------------------------------------
def ndvi_to_preview_rgb(ndvi: np.ndarray) -> np.ndarray:
    """NDVI in [-1,1] -> rgb8 false-color preview, red(low) -> yellow(mid) -> green(high). Purely
    cosmetic -- nothing downstream makes a decision from this array; `/fg/ndvi/image` (32FC1) is
    authoritative (ADR-007)."""
    t = np.clip((ndvi.astype(np.float64) + 1.0) / 2.0, 0.0, 1.0)  # [-1,1] -> [0,1]
    r = np.where(t < 0.5, 255.0, 255.0 * (1.0 - (t - 0.5) * 2.0))
    g = np.where(t < 0.5, 255.0 * (t * 2.0), 255.0)
    b = np.zeros_like(r)
    return np.stack([r, g, b], axis=-1).round().astype(np.uint8)
