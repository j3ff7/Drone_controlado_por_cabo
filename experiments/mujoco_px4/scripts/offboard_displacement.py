#!/usr/bin/env python3
"""Command one 0.5 m local-NED displacement and return using MAVLink Offboard."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil

OFFBOARD = 6.0
TYPE_MASK_POSITION_ONLY = 0b0000_1111_1111_1000


def setpoint(link, north: float, east: float, down: float) -> None:
    link.mav.set_position_target_local_ned_send(
        0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, TYPE_MASK_POSITION_ONLY,
        north, east, down, 0, 0, 0, 0, 0, 0, 0, 0,
    )


def command(link, command_id: int, *params: float) -> None:
    link.mav.command_long_send(
        link.target_system, link.target_component, command_id, 0,
        *(list(params) + [0.0] * (7 - len(params))),
    )


def latest_position(link, timeout: float = 5.0):
    msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    return None if msg is None else (float(msg.x), float(msg.y), float(msg.z))


def hold_until(link, target, timeout: float, tolerance: float):
    deadline = time.monotonic() + timeout
    position = None
    settled_since = None
    while time.monotonic() < deadline:
        setpoint(link, *target)
        msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.05)
        if msg is not None:
            position = (float(msg.x), float(msg.y), float(msg.z))
            error = math.dist(position, target)
            if error < tolerance:
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= 3.0:
                    return position, True
            else:
                settled_since = None
        time.sleep(0.02)
    return position, False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dx", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    link = mavutil.mavlink_connection("udpin:127.0.0.1:14550", source_system=255)
    link.wait_heartbeat(timeout=30)
    start = latest_position(link, 10)
    if start is None:
        raise SystemExit("no LOCAL_POSITION_NED")

    for _ in range(40):
        setpoint(link, *start)
        time.sleep(0.05)
    command(link, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1.0, OFFBOARD, 0.0)
    ack = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    target = (start[0] + args.dx, start[1], start[2])
    reached, reached_ok = hold_until(link, target, args.timeout, 0.10)
    returned, return_ok = hold_until(link, start, args.timeout, 0.10)
    metrics = {
        "start_ned": start,
        "target_ned": target,
        "reached_ned": reached,
        "returned_ned": returned,
        "commanded_dx_m": args.dx,
        "achieved_dx_m": None if reached is None else reached[0] - start[0],
        "target_error_m": None if reached is None else math.dist(reached, target),
        "return_error_m": None if returned is None else math.dist(returned, start),
        "offboard_ack": None if ack is None else int(ack.result),
        "target_reached": reached_ok,
        "return_reached": return_ok,
    }
    args.output.write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))
    return 0 if reached_ok and return_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
