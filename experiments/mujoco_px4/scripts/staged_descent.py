#!/usr/bin/env python3
"""Descend through relative altitude stages and timestamp each plateau."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil

OFFBOARD = 6.0
POSITION_ONLY = 0b0000_1111_1111_1000


def send_setpoint(link, target) -> None:
    link.mav.set_position_target_local_ned_send(
        0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, POSITION_ONLY,
        *target, 0, 0, 0, 0, 0, 0, 0, 0,
    )


def position(link, timeout=5.0):
    msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return msg, (float(msg.x), float(msg.y), float(msg.z))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drops", default="0,0.35,0.70,1.05,1.40")
    parser.add_argument("--hold-sim", type=float, default=1.5)
    parser.add_argument("--timeout-wall", type=float, default=180.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    drops = [float(value) for value in args.drops.split(",")]

    link = mavutil.mavlink_connection("udpin:127.0.0.1:14550", source_system=255)
    link.wait_heartbeat(timeout=30)
    sample = position(link, 10)
    if sample is None:
        raise SystemExit("no LOCAL_POSITION_NED")
    _, start = sample

    for _ in range(40):
        send_setpoint(link, start)
        time.sleep(0.05)
    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1.0, OFFBOARD,
        0.0, 0.0, 0.0, 0.0, 0.0,
    )
    ack = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

    stages = []
    for index, drop in enumerate(drops):
        target = (start[0], start[1], start[2] + drop)
        deadline = time.monotonic() + args.timeout_wall
        settled_at = None
        first_sim = None
        last_msg = None
        last_pos = None
        while time.monotonic() < deadline:
            send_setpoint(link, target)
            sample = position(link, 0.05)
            if sample is not None:
                last_msg, last_pos = sample
                sim_time = float(last_msg.time_boot_ms) * 1e-3
                first_sim = sim_time if first_sim is None else first_sim
                error = math.dist(last_pos, target)
                if error < 0.12 and abs(float(last_msg.vz)) < 0.15:
                    settled_at = sim_time if settled_at is None else settled_at
                    if sim_time - settled_at >= args.hold_sim:
                        break
                else:
                    settled_at = None
            time.sleep(0.02)
        success = settled_at is not None and last_msg is not None \
            and float(last_msg.time_boot_ms) * 1e-3 - settled_at >= args.hold_sim
        stages.append({
            "index": index,
            "drop_m": drop,
            "target_ned": target,
            "position_ned": last_pos,
            "target_error_m": None if last_pos is None else math.dist(last_pos, target),
            "settled_start_sim_s": settled_at,
            "settled_end_sim_s": (
                None if last_msg is None else float(last_msg.time_boot_ms) * 1e-3
            ),
            "success": success,
        })
        print(json.dumps(stages[-1]), flush=True)
        if not success:
            break

    result = {
        "offboard_ack": None if ack is None else int(ack.result),
        "initial_ned": start,
        "hold_sim_s": args.hold_sim,
        "stages": stages,
        "pass": len(stages) == len(drops) and all(stage["success"] for stage in stages),
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
