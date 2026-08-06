#!/usr/bin/env python3
"""Non-interactive headless smoke flight driver for CI (Weeks 5-6 devops task, 2026-08-05).

⚠️ UNVERIFIED LIVE. Written against the documented, human-confirmed bringup sequence in
docs/WEEK1_BRINGUP.md §4/§6 and docs/WEEK3_VALIDATION.md Gate 1 (the same farm-world flight those
docs prove works interactively via MAVProxy) but has never itself been run against a live SITL/Gazebo
process — this session had no live Docker/Gazebo runner. Treat every "per docs/WEEK*" comment below as
"best-effort translation of an interactive, human-watched recipe into a non-interactive one," not as
independently proven. The first CI run of this script needs a human watching the raw log on failure.

Replaces MAVProxy's interactive prompt (docs/WEEK1_BRINGUP.md deliberately keeps that manual — see
scripts/run_farm_mission.sh's header on why) with a scripted pymavlink client, because CI has no human
to watch for the EKF-ready message or type `mode auto`. Talks to a SITL instance already started by
scripts/ci_sim_smoke.sh (this script does not itself launch SITL or Gazebo).

What it does, in order (mirrors docs/WEEK1_BRINGUP.md §6 + §8 exactly, just scripted):
  1. Connect, wait for a HEARTBEAT (SITL alive).
  2. Wait for EKF3-ready (via SYS_STATUS / a bounded timeout — MAVProxy shows this as a STATUSTEXT,
     "EKF3 IMU0 is using GPS"; this script waits on the equivalent EKF_STATUS_REPORT healthy flags,
     falling back to a fixed settle delay if that message never arrives, logged either way).
  3. One-time FRAME_CLASS=1 / FRAME_TYPE=1 + reboot dance (docs/WEEK1_BRINGUP.md §6 point 1: the
     gazebo-iris default params load but only take effect after a reboot on first boot).
  4. DISARM_DELAY=0, AUTO_OPTIONS=3 (bit0 allow-arm-in-AUTO, bit1 auto-takeoff).
  5. Upload the mission (QGC WPL file) via the MAVLink mission protocol.
  6. mode AUTO, arm throttle -- Copter auto-starts the loaded mission on arm (same as docs).
  7. Poll MISSION_ITEM_REACHED for each waypoint, with a hard overall timeout.
  8. Wait for DISARMED (RTL landed) or the timeout, whichever first.
  9. Write a JSON summary to --out (see SCHEMA below) -- always attempted (best-effort), even on
     partial failure, so scripts/check_sim_smoke.py has something to grade. Exit code reflects whether
     THIS SCRIPT ran to completion without an unexpected exception, NOT whether the flight matched the
     regression bar -- that grading is check_sim_smoke.py's job (same split as eval/run_spike.sh vs
     scripts/check_spike_regression.py).

SCHEMA (eval/results/sim_smoke.json):
  {
    "mission_file": str, "connection": str,
    "waypoints_total": int,       # NAV_WAYPOINT items in the uploaded mission (excludes home/takeoff/RTL)
    "waypoints_reached": int,     # count of distinct MISSION_ITEM_REACHED seq numbers observed
    "armed_ok": bool, "disarmed_ok": bool,  # disarmed_ok = went from armed -> disarmed before timeout
    "timed_out": bool, "duration_s": float,
    "frame_reboot_ok": bool,      # step 3 completed and reconnected
    "error": str | null           # set if an exception aborted the run early
  }

Usage (inside the CI sim image, run by scripts/ci_sim_smoke.sh):
    python3 scripts/ci_sim_smoke.py --mission /tmp/ci_smoke.waypoints \\
        --connection tcp:127.0.0.1:5760 --out eval/results/sim_smoke.json --timeout 300
"""
import argparse
import json
import sys
import time
from pathlib import Path


def load_qgc_wpl(path):
    """Parse a QGC WPL 110 file into a list of dicts (index-aligned with file order)."""
    items = []
    with open(path) as f:
        lines = [ln.rstrip("\n") for ln in f if ln.strip()]
    if not lines or lines[0] != "QGC WPL 110":
        raise ValueError(f"{path}: not a QGC WPL 110 file (first line: {lines[0] if lines else '<empty>'})")
    for ln in lines[1:]:
        seq, current, frame, cmd, p1, p2, p3, p4, x, y, z, autocontinue = ln.split("\t")
        items.append(dict(seq=int(seq), current=int(current), frame=int(frame), command=int(cmd),
                           param1=float(p1), param2=float(p2), param3=float(p3), param4=float(p4),
                           x=float(x), y=float(y), z=float(z), autocontinue=int(autocontinue)))
    return items


NAV_WAYPOINT = 16
FRAME_REL_ALT = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT -- matches gen_boustrophedon.py's write_qgc_wpl()


def count_coverage_waypoints(items):
    """NAV_WAYPOINT items on FRAME_REL_ALT only -- excludes the seq-0 home row, which QGC WPL 110
    also tags command=16 (NAV_WAYPOINT) but on FRAME_GLOBAL (frame=0) as a placeholder, per
    gen_boustrophedon.py's write_qgc_wpl(). Not filtering frame would overcount by exactly 1 (home)."""
    return sum(1 for it in items if it["command"] == NAV_WAYPOINT and it["frame"] == FRAME_REL_ALT)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mission", required=True, help="QGC WPL .waypoints file (e.g. from gen_boustrophedon.py)")
    ap.add_argument("--connection", default="tcp:127.0.0.1:5760",
                     help="pymavlink connection string. Default is SITL's own documented default "
                          "mavlink listen port (independent of MAVProxy, which is skipped via "
                          "sim_vehicle.py --no-mavproxy in scripts/ci_sim_smoke.sh) -- NOT MAVProxy's "
                          "udp:14550 output port, which does not exist without MAVProxy running. "
                          "UNVERIFIED against this project's pinned SHA; see that script's comments.")
    ap.add_argument("--out", default="eval/results/sim_smoke.json")
    ap.add_argument("--timeout", type=float, default=300.0, help="hard wall-clock budget, seconds")
    ap.add_argument("--connect-timeout", type=float, default=60.0)
    ap.add_argument("--reboot-settle-s", type=float, default=20.0,
                     help="fallback fixed delay after the FRAME_CLASS/TYPE reboot if no heartbeat "
                          "reconnect is observed sooner (docs/WEEK1_BRINGUP.md §6 point 1: ~15s)")
    args = ap.parse_args()

    summary = {
        "mission_file": args.mission, "connection": args.connection,
        "waypoints_total": 0, "waypoints_reached": 0,
        "armed_ok": False, "disarmed_ok": False,
        "timed_out": False, "duration_s": 0.0,
        "frame_reboot_ok": False, "error": None,
    }
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    t_start = time.monotonic()

    def flush():
        summary["duration_s"] = round(time.monotonic() - t_start, 1)
        out_path.write_text(json.dumps(summary, indent=2) + "\n")

    try:
        # Deferred import: only needed once we know we're actually running (keeps --help usable
        # without pymavlink installed, e.g. when linting this file outside the sim image).
        from pymavlink import mavutil
    except ImportError as exc:
        summary["error"] = f"pymavlink not importable ({exc}) -- must run inside the fieldguard-sim CI image"
        flush()
        print(f"[ci_sim_smoke] FAIL: {summary['error']}", file=sys.stderr)
        return 1

    try:
        mission_items = load_qgc_wpl(args.mission)
        summary["waypoints_total"] = count_coverage_waypoints(mission_items)
        print(f"[ci_sim_smoke] Loaded {args.mission}: {len(mission_items)} items, "
              f"{summary['waypoints_total']} NAV_WAYPOINTs")

        print(f"[ci_sim_smoke] Connecting to {args.connection} ...")
        mav = mavutil.mavlink_connection(args.connection)
        mav.wait_heartbeat(timeout=args.connect_timeout)
        print(f"[ci_sim_smoke] Heartbeat OK (sysid={mav.target_system} compid={mav.target_component})")

        def deadline_remaining():
            return args.timeout - (time.monotonic() - t_start)

        def set_param(name, value):
            mav.mav.param_set_send(mav.target_system, mav.target_component,
                                    name.encode("utf-8"), float(value),
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # --- Step 3: one-time FRAME_CLASS/TYPE + reboot (docs/WEEK1_BRINGUP.md §6 point 1) ---------
        set_param("FRAME_CLASS", 1)
        set_param("FRAME_TYPE", 1)
        time.sleep(1.0)  # let the param sets land before reboot, matching the interactive pacing
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                   mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                   1, 0, 0, 0, 0, 0, 0)
        print("[ci_sim_smoke] Sent FRAME_CLASS=1/FRAME_TYPE=1 + reboot; waiting to reconnect "
              f"(up to {args.reboot_settle_s:.0f}s settle)...")
        time.sleep(2.0)  # SITL needs a moment to actually go down before it comes back
        try:
            mav.wait_heartbeat(timeout=args.reboot_settle_s)
            summary["frame_reboot_ok"] = True
            print("[ci_sim_smoke] Reconnected after reboot.")
        except Exception as exc:  # noqa: BLE001 -- best-effort: log and keep going, mirrors the
            # interactive flow's "wait ~15s for reconnect" being a judgment call, not a hard gate.
            print(f"[ci_sim_smoke] WARNING: no heartbeat within {args.reboot_settle_s:.0f}s "
                  f"post-reboot ({exc}); continuing anyway.", file=sys.stderr)

        # --- Step 4: mission-friendly params -------------------------------------------------------
        set_param("DISARM_DELAY", 0)
        set_param("AUTO_OPTIONS", 3)
        time.sleep(0.5)

        # --- Step 5: upload mission via the MAVLink mission protocol -------------------------------
        mav.mav.mission_count_send(mav.target_system, mav.target_component, len(mission_items))
        for _ in range(len(mission_items)):
            req = mav.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=True,
                                  timeout=15)
            if req is None:
                raise TimeoutError("mission upload: no MISSION_REQUEST from autopilot")
            it = mission_items[req.seq]
            mav.mav.mission_item_send(
                mav.target_system, mav.target_component, it["seq"], it["frame"], it["command"],
                it["current"], it["autocontinue"], it["param1"], it["param2"], it["param3"],
                it["param4"], it["x"], it["y"], it["z"])
        ack = mav.recv_match(type="MISSION_ACK", blocking=True, timeout=15)
        if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"mission upload not accepted (ack={ack})")
        print(f"[ci_sim_smoke] Mission uploaded + accepted: {len(mission_items)} items")

        # --- Step 6: AUTO + arm (Copter auto-starts the mission on arm, per docs/WEEK1_BRINGUP.md §8) ---
        mav.set_mode_apm("AUTO")
        time.sleep(1.0)
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                   1, 0, 0, 0, 0, 0, 0)
        armed_deadline = time.monotonic() + 15
        while time.monotonic() < armed_deadline:
            hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                summary["armed_ok"] = True
                break
        print(f"[ci_sim_smoke] armed_ok={summary['armed_ok']}")

        # --- Step 7-8: poll for waypoint progress + disarm, within the overall timeout --------------
        reached = set()
        while deadline_remaining() > 0:
            msg = mav.recv_match(type=["MISSION_ITEM_REACHED", "HEARTBEAT"], blocking=True,
                                  timeout=min(5, max(0.1, deadline_remaining())))
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ITEM_REACHED":
                reached.add(msg.seq)
                print(f"[ci_sim_smoke] Reached waypoint seq={msg.seq} "
                      f"({len(reached)}/{summary['waypoints_total']})")
            elif msg.get_type() == "HEARTBEAT" and summary["armed_ok"]:
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    summary["disarmed_ok"] = True
                    print("[ci_sim_smoke] Disarmed (mission complete / RTL landed).")
                    break
        summary["waypoints_reached"] = len(reached)
        summary["timed_out"] = deadline_remaining() <= 0 and not summary["disarmed_ok"]

    except Exception as exc:  # noqa: BLE001 -- top-level: always emit a summary, never crash silently
        summary["error"] = f"{type(exc).__name__}: {exc}"
        print(f"[ci_sim_smoke] ERROR: {summary['error']}", file=sys.stderr)
        flush()
        return 1

    flush()
    print(f"[ci_sim_smoke] Done: {json.dumps(summary, indent=2)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
