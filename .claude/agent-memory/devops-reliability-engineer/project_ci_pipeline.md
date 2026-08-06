---
name: project-ci-pipeline
description: FieldGuard CI structure, pinned deps, and exact commands (Week 2 planning+eval job 2026-08-04; Weeks 5-6 build-test-sim job added 2026-08-05, see [[project_week5_ci_gazebo]])
metadata:
  type: project
---

CI lives at `.github/workflows/ci.yml`, on plain `ubuntu-latest`, `python-version: "3.12"` via
`actions/setup-python@v5`. Three jobs now (as of 2026-08-05; `build-test-sim` was promoted from a
commented-out stub to a real, manual-dispatch-gated job — see [[project_week5_ci_gazebo]] for the
full plan, feasibility verdict, and what's still unverified):

1. `validate-config` — runs `scripts/validate_agents.py` (needs `pyyaml`, installed inline/unpinned
   in that job — deliberately left untouched per the Week 2 task's explicit instruction to keep it
   "intact"; a future pass should pin it too).
2. `planning-and-eval` (added Week 2, 2026-08-04) — runs, in order:
   - `python3 -m unittest discover -s tests/fieldguard_planning -v` (stdlib only, no deps, 34 tests:
     27 pass + 7 skipped/self-activating pending Week 3-4 avoidance-loop scenarios).
   - `pip install -r requirements-eval.txt` (numpy==2.5.1, scipy==1.18.0 — see
     [[reference_pinned_versions]]).
   - Smoke-runs `scripts/gen_boustrophedon.py` and `scripts/gen_farm_world.py` against `/tmp`
     outputs (never overwrites the checked-in `config/static_obstacles.json` /
     `sim/worlds/farmguard_field.sdf` in CI), then `scripts/check_mission_geofence.py || true` —
     see [[known_ci_flake_check_mission_geofence]] for why that one is `|| true`.
   - `python3 sim/spike/gen_spike_clip.py --seed 42 --out sim/spike/out/spike_seed42` (regenerates
     the gitignored synthetic clip fresh every run — `sim/spike/out/` is gitignored on purpose).
   - `bash eval/run_spike.sh` (label GT → both baselines → score; writes `eval/results/`, also
     gitignored).
   - `python3 scripts/check_spike_regression.py eval/results/spike_scores.json` — new devops-owned
     regression gate, asserts `approaches.a_ndvi_direct.per_bird_track_fnr == 0.0` on the seed-42
     baseline (parses `score.py`'s own machine-readable JSON output; does not invent a new format).
   - Uploads `eval/results/` as the `eval-spike-metrics` artifact (`if: always()`).
3. `build-test-sim` (added 2026-08-05, Weeks 5-6, `if: github.event_name == 'workflow_dispatch'` —
   NOT on every push yet) — pulls `ghcr.io/<owner>/fieldguard-sim:latest` (published by the separate
   `sim-image.yml` workflow, decoupled from push cadence), runs `scripts/ci_sim_smoke.sh` (headless
   `gz sim` + `sim_vehicle.py --no-mavproxy` + a pymavlink driver flying a 2-lane scripted mission
   through the farm world, no camera/DDS), then gates on `scripts/check_sim_smoke.py`. **Never
   executed against a live runner as of 2026-08-05** — see [[project_week5_ci_gazebo]] before touching
   this job or flipping its trigger off manual-dispatch.

Local verification commands (run these, not `pip install` blind, before touching this pipeline
again):
```
python3 -m unittest discover -s tests/fieldguard_planning -v
pip install -r requirements-eval.txt
python3 sim/spike/gen_spike_clip.py --seed 42 --out sim/spike/out/spike_seed42
PYTHON=python3 bash eval/run_spike.sh
python3 scripts/check_spike_regression.py eval/results/spike_scores.json
actionlint .github/workflows/ci.yml     # brew install actionlint if missing
```

Reference run (2026-08-04, Python 3.12.12, numpy 2.5.1, scipy 1.18.0, macOS arm64 — CI is
ubuntu-latest x86_64 but both have manylinux cp312 wheels, verified against PyPI): spike scoring on
seed 42 is **exactly reproducible byte-for-byte** across independent re-runs (regenerate clip + rerun
harness twice, diffed `spike_scores.json`, identical). Headline numbers: `a_ndvi_direct` TP=53 FP=66
FN=1, precision=0.445, recall=0.981, FNR=0.019, per_bird_track_fnr=0.0; `b_synthetic_rgb` TP=53 FP=0
FN=1, precision=1.0, same FNR. ADR-003 verdict: ADOPT (a) NDVI-direct.
