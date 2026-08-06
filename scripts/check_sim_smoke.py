#!/usr/bin/env python3
"""CI regression gate for the headless Gazebo/SITL smoke flight (Weeks 5-6 devops task, 2026-08-05).

Reads the machine-readable eval/results/sim_smoke.json that scripts/ci_sim_smoke.py writes and
asserts the scripted mission actually flew: every coverage waypoint reached, no unhandled error, no
timeout, armed, and disarmed cleanly at the end (RTL landed). Same split as
scripts/check_spike_regression.py: the driver script's job is to run and record, this script's job is
to decide pass/fail -- keeps the driver's exit code meaning "did I crash unexpectedly" separate from
"did the flight meet the bar", exactly like eval/run_spike.sh vs this file's sibling.

This is deliberately a narrow structural check (all N waypoints reached, no crash/timeout) -- not a
flight-quality metric (e.g. cross-track error). If Weeks 5-6 wires the real NDVI render into this same
CI image, the natural next gate here is avoidance-loop success / coverage-debt completeness on a real
(not synthetic) flight log -- that hook is intentionally NOT built yet (see docs/WEEK5_CI_GAZEBO.md
"Eval-gate hook" -- the render this would score doesn't exist in the sim yet).

Usage:
    python3 scripts/check_sim_smoke.py eval/results/sim_smoke.json
"""
import argparse
import json
import sys
from pathlib import Path


def evaluate(data: dict) -> list[str]:
    """Return a list of failure reasons; empty list == PASS."""
    failures = []

    if data.get("error"):
        failures.append(f"driver reported an error: {data['error']}")

    total = data.get("waypoints_total")
    reached = data.get("waypoints_reached")
    if total is None or reached is None:
        failures.append("waypoints_total/waypoints_reached missing from summary -- schema changed?")
    elif total <= 0:
        failures.append(f"waypoints_total={total} -- the mission file itself looks empty/malformed")
    elif reached < total:
        failures.append(f"only reached {reached}/{total} waypoints -- mission did not complete")

    if not data.get("armed_ok"):
        failures.append("armed_ok is False -- the vehicle never armed")

    if not data.get("disarmed_ok"):
        failures.append("disarmed_ok is False -- did not disarm cleanly (crash, hang, or still armed "
                         "at timeout)")

    if data.get("timed_out"):
        failures.append("timed_out is True -- the flight did not finish within the CI budget")

    return failures


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("summary_json", type=Path, nargs="?",
                     default=Path("eval/results/sim_smoke.json"))
    args = ap.parse_args()

    if not args.summary_json.exists():
        print(f"[check_sim_smoke] FAIL: {args.summary_json} does not exist -- did "
              f"scripts/ci_sim_smoke.sh run before this step?", file=sys.stderr)
        return 1

    data = json.loads(args.summary_json.read_text())
    failures = evaluate(data)

    print(f"[check_sim_smoke] waypoints_reached={data.get('waypoints_reached')}/"
          f"{data.get('waypoints_total')} armed_ok={data.get('armed_ok')} "
          f"disarmed_ok={data.get('disarmed_ok')} timed_out={data.get('timed_out')} "
          f"duration_s={data.get('duration_s')}")

    if failures:
        print("[check_sim_smoke] FAIL:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    print("[check_sim_smoke] PASS: scripted mission completed cleanly, all waypoints reached.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
