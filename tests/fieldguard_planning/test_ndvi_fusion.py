"""Tests for ndvi_fusion.py -- NDVI math (incl. the 0/0 guard), the ADR-007 stale-pair drop path,
and raw image decode. Requires numpy (see ndvi_fusion.py's module docstring for why this module is
a scoped exception to the package's usual stdlib-only rule) -- unlike the rest of
tests/fieldguard_planning, which runs on a bare interpreter.
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

import numpy as np  # noqa: E402

from fieldguard_planning.ndvi_fusion import (  # noqa: E402
    NDVI_ZERO_DENOM_SENTINEL,
    NdviFuser,
    StampedFrame,
    compute_ndvi,
    decode_mono16,
    decode_rgb8,
    load_camera_config,
    max_stamp_delta_s,
    ndvi_to_preview_rgb,
    nearest_index,
    pair_and_fuse_stream,
    rescale_nir,
    rescale_red,
)

CFG = load_camera_config()


class TestLoadCameraConfig(unittest.TestCase):
    def test_expected_fields_present(self):
        self.assertEqual(CFG["camera"]["image_width_px"], 640)
        self.assertEqual(CFG["camera"]["image_height_px"], 480)
        self.assertEqual(CFG["camera"]["update_rate_hz"], 5.0)
        self.assertEqual(CFG["thermal"]["min_temp_k"], 270.0)
        self.assertEqual(CFG["thermal"]["max_temp_k"], 330.0)
        self.assertEqual(CFG["thermal"]["resolution_k_per_count"], 0.01)


class TestRescaleRed(unittest.TestCase):
    def test_known_values(self):
        red = np.array([0, 128, 255], dtype=np.uint8)
        out = rescale_red(red)
        self.assertAlmostEqual(out[0], 0.0, places=9)
        self.assertAlmostEqual(out[1], 128.0 / 255.0, places=9)
        self.assertAlmostEqual(out[2], 1.0, places=9)


class TestRescaleNir(unittest.TestCase):
    """Cross-checked against config/ndvi_camera.json's own temperature_calibration table: canopy
    rho_nir=0.85 <-> temperature_k=321.0, computed as min_temp_k + rho*(max_temp_k-min_temp_k) =
    270 + 0.85*60 = 321.0. raw uint16 count = round(321.0 / 0.01) = 32100 per the decode_formula."""

    def test_canopy_calibration_point_round_trips(self):
        raw = np.array([32100], dtype=np.uint16)
        rho = rescale_nir(raw, min_temp_k=270.0, max_temp_k=330.0, resolution_k_per_count=0.01)
        self.assertAlmostEqual(float(rho[0]), 0.85, places=3)

    def test_soil_and_bird_calibration_points(self):
        # soil: rho=0.20 -> T=270+0.20*60=282.0K -> raw=28200
        # bird: rho=0.05 -> T=270+0.05*60=273.0K -> raw=27300
        raw = np.array([28200, 27300], dtype=np.uint16)
        rho = rescale_nir(raw, 270.0, 330.0, 0.01)
        self.assertAlmostEqual(float(rho[0]), 0.20, places=3)
        self.assertAlmostEqual(float(rho[1]), 0.05, places=3)

    def test_clips_outside_declared_range(self):
        # raw count corresponding to T below min_temp_k or above max_temp_k must clip to [0,1],
        # never go negative or exceed 1 (a sensor artifact/off-calibration pixel should not corrupt
        # NDVI with an out-of-physical-range reflectance).
        raw = np.array([0, 65535], dtype=np.uint16)  # T=0K and T=655.35K -- both outside [270,330]
        rho = rescale_nir(raw, 270.0, 330.0, 0.01)
        self.assertAlmostEqual(float(rho[0]), 0.0, places=9)
        self.assertAlmostEqual(float(rho[1]), 1.0, places=9)


class TestComputeNdvi(unittest.TestCase):
    def test_known_formula_value(self):
        # red=0.2, nir=0.85 -> (0.85-0.2)/(0.85+0.2) = 0.65/1.05 = 0.619047619...
        red = np.array([[0.2]])
        nir = np.array([[0.85]])
        ndvi, zero_count = compute_ndvi(red, nir)
        self.assertAlmostEqual(float(ndvi[0, 0]), 0.65 / 1.05, places=5)
        self.assertEqual(zero_count, 0)

    def test_bird_reads_negative(self):
        # bird rho_nir=0.05 over full-reflectance red backdrop is an extreme case; use the
        # calibration table's actual bird/canopy-adjacent contrast instead: red=0.5 nir=0.05 (a
        # non-vegetation, low-NIR object against a mid-red background) -> negative NDVI.
        red = np.array([[0.5]])
        nir = np.array([[0.05]])
        ndvi, _ = compute_ndvi(red, nir)
        self.assertLess(float(ndvi[0, 0]), 0.0)

    def test_zero_denominator_guard_single_pixel(self):
        red = np.array([[0.0]])
        nir = np.array([[0.0]])
        ndvi, zero_count = compute_ndvi(red, nir)
        self.assertEqual(zero_count, 1)
        self.assertEqual(float(ndvi[0, 0]), NDVI_ZERO_DENOM_SENTINEL)
        self.assertFalse(np.isnan(ndvi[0, 0]), "must never emit NaN silently")

    def test_zero_denominator_guard_mixed_frame(self):
        """A frame with SOME degenerate pixels must sentinel only those, leave the rest correct,
        and count exactly the degenerate ones -- not vacuously all-zero, not silently dropped."""
        red = np.array([[0.0, 0.2], [0.5, 0.0]])
        nir = np.array([[0.0, 0.85], [0.5, 0.3]])
        # pixel (0,0): 0/0 -> sentinel.  (0,1): normal.  (1,0): 0.5&0.5 -> 0.0 (real zero, NOT
        # degenerate -- denom=1.0 != 0).  (1,1): 0.0&0.3 -> denom=0.3, ndvi=1.0 (nir only).
        ndvi, zero_count = compute_ndvi(red, nir)
        self.assertEqual(zero_count, 1)
        self.assertEqual(float(ndvi[0, 0]), NDVI_ZERO_DENOM_SENTINEL)
        self.assertAlmostEqual(float(ndvi[0, 1]), 0.65 / 1.05, places=5)
        self.assertAlmostEqual(float(ndvi[1, 0]), 0.0, places=9)  # genuine zero, not sentinel path
        self.assertAlmostEqual(float(ndvi[1, 1]), 1.0, places=9)

    def test_shape_mismatch_raises(self):
        with self.assertRaises(ValueError):
            compute_ndvi(np.zeros((2, 2)), np.zeros((3, 3)))


class TestDecodeImages(unittest.TestCase):
    def test_decode_rgb8_round_trip(self):
        h, w = 2, 3
        # pixel (0,0)=red, (0,1)=green, (0,2)=blue, row 1 = mid-gray
        pixels = [
            [255, 0, 0], [0, 255, 0], [0, 0, 255],
            [10, 20, 30], [40, 50, 60], [70, 80, 90],
        ]
        data = bytes(v for px in pixels for v in px)
        arr = decode_rgb8(h, w, data)
        self.assertEqual(arr.shape, (2, 3, 3))
        self.assertEqual(list(arr[0, 0]), [255, 0, 0])
        self.assertEqual(list(arr[1, 2]), [70, 80, 90])

    def test_decode_mono16_round_trip(self):
        h, w = 2, 2
        vals = np.array([[100, 200], [30000, 65535]], dtype=np.uint16)
        arr = decode_mono16(h, w, vals.tobytes())
        self.assertEqual(arr.shape, (2, 2))
        self.assertEqual(int(arr[1, 1]), 65535)
        self.assertEqual(int(arr[0, 0]), 100)


class TestStalePairGuard(unittest.TestCase):
    def test_max_stamp_delta_matches_config_update_rate(self):
        # 0.25 / 5.0 Hz = 0.05s = 50ms, per ADR-007's amendment scaling rule.
        self.assertAlmostEqual(max_stamp_delta_s(5.0), 0.05, places=9)
        # ADR-007 amendment's own reference example: 10 Hz -> 25 ms.
        self.assertAlmostEqual(max_stamp_delta_s(10.0), 0.025, places=9)

    def _fuser(self):
        return NdviFuser(update_rate_hz=5.0, min_temp_k=270.0, max_temp_k=330.0,
                         resolution_k_per_count=0.01)

    def test_within_tolerance_pair_is_fused(self):
        fuser = self._fuser()
        red = np.full((2, 2), 128, dtype=np.uint8)
        nir = np.full((2, 2), 32100, dtype=np.uint16)  # canopy calibration point
        result = fuser.fuse(rgb_stamp_s=10.000, red_u8=red, nir_stamp_s=10.010, nir_u16=nir)
        self.assertTrue(result.accepted)
        self.assertEqual(result.reason, "ok")
        self.assertIsNotNone(result.ndvi)
        self.assertEqual(fuser.fused_count, 1)
        self.assertEqual(fuser.dropped_pair_count, 0)
        self.assertEqual(fuser.event_log[-1]["kind"], "fused")

    def test_exactly_at_boundary_is_accepted(self):
        """delta == max_delta_s exactly must be accepted (the guard is '> max', not '>= max') --
        pin the boundary behavior explicitly so it can't silently flip."""
        fuser = self._fuser()
        red = np.zeros((1, 1), dtype=np.uint8)
        nir = np.zeros((1, 1), dtype=np.uint16)
        result = fuser.fuse(rgb_stamp_s=0.0, red_u8=red, nir_stamp_s=fuser.max_delta_s, nir_u16=nir)
        self.assertTrue(result.accepted)

    def test_stale_pair_is_dropped_and_counted(self):
        """THE required explicit case: a stale NIR frame (delta > 25% of one frame period) must be
        DROPPED, never fused into a mispaired NDVI, and the dropped_pair_count must increment."""
        fuser = self._fuser()
        red = np.full((2, 2), 128, dtype=np.uint8)
        nir = np.full((2, 2), 32100, dtype=np.uint16)
        result = fuser.fuse(rgb_stamp_s=10.000, red_u8=red, nir_stamp_s=10.060,  # 60ms > 50ms bound
                            nir_u16=nir)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "stale_pair")
        self.assertIsNone(result.ndvi)
        self.assertEqual(fuser.dropped_pair_count, 1)
        self.assertEqual(fuser.fused_count, 0)
        self.assertEqual(fuser.event_log[-1]["kind"], "dropped_pair")
        self.assertEqual(fuser.event_log[-1]["dropped_pair_count"], 1)

    def test_dropped_pair_counter_accumulates_across_calls(self):
        fuser = self._fuser()
        red = np.zeros((1, 1), dtype=np.uint8)
        nir = np.zeros((1, 1), dtype=np.uint16)
        for i in range(3):
            fuser.fuse(rgb_stamp_s=float(i), red_u8=red, nir_stamp_s=float(i) + 1.0, nir_u16=nir)
        self.assertEqual(fuser.dropped_pair_count, 3)
        self.assertEqual(fuser.fused_count, 0)


class TestNearestIndex(unittest.TestCase):
    def test_picks_closest(self):
        self.assertEqual(nearest_index(10.08, [9.9, 10.0, 10.1, 10.2]), 2)

    def test_empty_candidates_returns_none(self):
        self.assertIsNone(nearest_index(1.0, []))

    def test_tie_prefers_first_seen(self):
        self.assertEqual(nearest_index(10.0, [9.9, 10.1]), 0)


class TestPairAndFuseStream(unittest.TestCase):
    def _fuser(self):
        return NdviFuser(update_rate_hz=5.0, min_temp_k=270.0, max_temp_k=330.0,
                         resolution_k_per_count=0.01)

    def test_regular_stream_all_fused(self):
        red = np.full((1, 1), 100, dtype=np.uint8)
        nir = np.full((1, 1), 30000, dtype=np.uint16)
        rgb_frames = [StampedFrame(t, red) for t in (0.0, 0.2, 0.4, 0.6)]
        nir_frames = [StampedFrame(t + 0.005, nir) for t in (0.0, 0.2, 0.4, 0.6)]  # 5ms jitter
        fuser = self._fuser()
        results = pair_and_fuse_stream(rgb_frames, nir_frames, fuser)
        self.assertEqual(len(results), 4)
        self.assertTrue(all(r.accepted for r in results))
        self.assertEqual(fuser.fused_count, 4)
        self.assertEqual(fuser.dropped_pair_count, 0)

    def test_a_stale_nir_frame_is_dropped_not_mispaired(self):
        """A single NIR frame drop mid-stream (e.g. a missed render) must not cause the surviving
        RGB frame to silently pair with a NIR frame from the WRONG period -- it must be dropped and
        counted instead. This is the exact scenario the ADR-007 amendment exists to prevent: at 5Hz
        (200ms period) losing one NIR frame pushes the nearest surviving match ~200ms away, far past
        the 50ms bound."""
        red = np.full((1, 1), 100, dtype=np.uint8)
        nir = np.full((1, 1), 30000, dtype=np.uint16)
        rgb_frames = [StampedFrame(t, red) for t in (0.0, 0.2, 0.4, 0.6)]
        # NIR frame at t=0.2 is MISSING (simulates a dropped render frame).
        nir_frames = [StampedFrame(t, nir) for t in (0.0, 0.4, 0.6)]
        fuser = self._fuser()
        results = pair_and_fuse_stream(rgb_frames, nir_frames, fuser)
        self.assertEqual(len(results), 4)
        self.assertTrue(results[0].accepted)   # rgb t=0.0 <-> nir t=0.0, delta=0
        self.assertFalse(results[1].accepted)  # rgb t=0.2 <-> nearest nir is t=0.0 or 0.4, delta=0.2s
        self.assertEqual(results[1].reason, "stale_pair")
        self.assertTrue(results[2].accepted)   # rgb t=0.4 <-> nir t=0.4, delta=0
        self.assertTrue(results[3].accepted)   # rgb t=0.6 <-> nir t=0.6, delta=0
        self.assertEqual(fuser.dropped_pair_count, 1)
        self.assertEqual(fuser.fused_count, 3)

    def test_empty_nir_stream_drops_every_rgb_frame(self):
        red = np.full((1, 1), 100, dtype=np.uint8)
        rgb_frames = [StampedFrame(t, red) for t in (0.0, 0.2)]
        fuser = self._fuser()
        results = pair_and_fuse_stream(rgb_frames, [], fuser)
        self.assertEqual(len(results), 2)
        self.assertTrue(all(not r.accepted for r in results))
        self.assertTrue(all(r.reason == "no_nir_frames" for r in results))
        self.assertEqual(fuser.dropped_pair_count, 2)


class TestNdviPreviewColormap(unittest.TestCase):
    def test_low_mid_high_spot_checks(self):
        ndvi = np.array([[-1.0, 0.0, 1.0]])
        preview = ndvi_to_preview_rgb(ndvi)
        self.assertEqual(preview.shape, (1, 3, 3))
        self.assertEqual(list(preview[0, 0]), [255, 0, 0])   # ndvi=-1 -> red
        self.assertEqual(list(preview[0, 1]), [255, 255, 0])  # ndvi=0 -> yellow
        self.assertEqual(list(preview[0, 2]), [0, 255, 0])   # ndvi=+1 -> green


if __name__ == "__main__":
    unittest.main()
