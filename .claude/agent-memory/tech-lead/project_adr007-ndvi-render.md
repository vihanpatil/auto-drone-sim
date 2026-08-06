---
name: adr007-ndvi-render
description: ADR-007 decided — dual-band NDVI = RGB camera (Red) + Gazebo thermal sensor repurposed as synthetic NIR; NDVI computed in a ROS 2 node; contract locked, render unproven live
metadata:
  type: project
---

ADR-007 (how the dual-band NDVI frame is produced in Gazebo Harmonic) is **ACCEPTED,
confirmation-pending** as of 2026-08-05. Decided the Weeks 5-6 NDVI-render architecture.

**Decision:** option (a) two-camera render, built from first-class Gazebo sensors:
- Red band = R channel of a standard `type="camera"` (R8G8B8) sensor; that RGB image is ALSO the
  ADR-003 NDVI+RGB comparison arm (Red source doubles as comparison arm — no extra camera).
- NIR band = a `type="thermal"` (L16) sensor **repurposed** as synthetic NIR. Gazebo's thermal
  camera reads a per-object scalar you author in SDF (`gz-sim-thermal-system` `<temperature>` /
  `<heat_signature>` per visual), independent of visible color/lighting — exactly the per-model
  NIR-reflectance property option (a) needs, from a documented sensor instead of a hand-written
  shader. Each material's `<temperature>` encodes NIR reflectance, calibrated into
  `[min_temp,max_temp]` so bridged mono16 maps linearly to ρ_nir∈[0,1] (one calibration knob).
- NDVI computed in a ROS 2 node (`ndvi_node`), NOT in-render: pairs the two images by nearest
  sim-time stamp, publishes NDVI=(NIR−Red)/(NIR+Red) as 32FC1∈[−1,1]. Keeps bands raw/honest,
  index math unit-tested & offline-reproducible (eval harness already eats float32 NDVI .npy).

**Rejected:** (b) shader/material-encoded NIR — custom OGRE2 passes, clever-but-fragile, thermal
gives same capability. (c) post-process NIR from RGB+veg-mask — NIR would be a function of RGB so
NDVI carries no independent band; it's what the synthetic Week-2 spike already did, so it would make
[[adr003-ndvi-detection]]'s real-render re-confirmation circular. Independence of the NIR band is
the whole point.

**Locked topic contract** (perception + stitch code against these, ADR-005 discipline):
`/fg/sensor/rgb/image` (rgb8), `/fg/sensor/rgb/camera_info`, `/fg/sensor/nir/image` (mono16),
`/fg/sensor/nir/camera_info`, `/fg/ndvi/image` (32FC1 ∈[−1,1], authoritative — ADR-003 detects on
it, stitch georeferences it), `/fg/ndvi/camera_info`, `/fg/ndvi/preview` (rgb8 human-only).
Comparison arm v1 = NDVI+RGB (reuse RGB camera); NDVI+depth = documented stretch.

**Hard build requirement for robotics-sim-engineer:** RGB + thermal sensors MUST be co-located on
one rigid nadir mount with IDENTICAL intrinsics/pose/update_rate (pose matches spike extrinsic
quat_wxyz=(0,1,0,0)), and EVERY world material must carry a `<temperature>` encoding its NIR
reflectance (canopy high, soil/water/birds low). No per-model temperature → thermal returns ambient
for everything → flat/meaningless NDVI. Thermal is ogre2-only; world already runs Sensors on ogre2.

**Honest caveat / fallback:** Red is from a lit visible render, NIR is illumination-independent, so
shadowed canopy can spuriously raise NDVI. Mitigate with fixed high sun + diffuse sky; if the
ADR-003 re-run shows lighting artifacts break detection, fall back to a second thermal-style
reflectance scalar for the Red band too (both bands illumination-independent).

**Open follow-up (confirmation-pending, human Docker run):** render is unproven live. Batch with the
[[adr003-ndvi-detection]] real-render spike re-run + ADR-005 live-topic + ADR-006 live-resume checks
in ONE Docker session. Targets: (1) six /fg/* topics present, correct encodings, hz at camera rate;
(2) canopy vs soil vs bird NDVI genuinely differ (proves NIR independence); (3) eval/run_spike.sh
on the real render dir re-confirms ADR-003; (4) `gz-sim-thermal-system` loads on ogre2.
