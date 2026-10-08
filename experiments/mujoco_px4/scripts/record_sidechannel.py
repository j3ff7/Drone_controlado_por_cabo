#!/usr/bin/env python3
"""Record MuJoCo ground truth and produce compact M0 metrics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from pathlib import Path

import numpy as np
import psutil


def euler_deg(q: list[float]) -> tuple[float, float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=14650)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    target = ("127.0.0.1", args.port)
    sock.sendto(json.dumps({"v": 1, "type": "subscribe"}).encode(), target)

    rows: list[dict] = []
    wall_start = time.monotonic()
    deadline = wall_start + args.duration
    cpu_samples: list[float] = []
    while time.monotonic() < deadline:
        if args.stop_file is not None and args.stop_file.exists():
            break
        try:
            msg = json.loads(sock.recvfrom(65507)[0])
        except socket.timeout:
            continue
        if msg.get("type") != "ground_truth":
            continue
        roll, pitch, yaw = euler_deg(msg["q_frd_ned"])
        tether = msg.get("tether") or {}
        rows.append({
            "time": msg["time"],
            "wall_elapsed_s": time.monotonic() - wall_start,
            "pn": msg["pos_ned"][0], "pe": msg["pos_ned"][1], "pd": msg["pos_ned"][2],
            "vn": msg["vel_ned"][0], "ve": msg["vel_ned"][1], "vd": msg["vel_ned"][2],
            "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
            **{f"motor_{i}": v for i, v in enumerate(msg["actuators"][:4])},
            "tether_tension_n": tether.get("tension", 0.0),
            "tether_error_m": tether.get("connection_error_norm", 0.0),
            "tether_anchor_drift_m": tether.get("anchor_drift", 0.0),
            "tether_azimuth_deg": tether.get("azimuth_deg", 0.0),
            "tether_elevation_deg": tether.get("elevation_deg", 0.0),
            "tether_ground_contacts": tether.get("ground_contacts", 0),
            "tether_ground_contact_links": tether.get("ground_contact_links", 0),
            "total_contacts": tether.get("total_contacts", 0),
        })
        cpu_samples.append(psutil.cpu_percent(interval=None))

    sock.sendto(json.dumps({"v": 1, "type": "unsubscribe"}).encode(), target)
    sock.close()
    if not rows:
        raise SystemExit("no side-channel samples received")

    with (args.output / "ground_truth.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    max_altitude = max(-r["pd"] for r in rows)
    # Keep the settled top plateau and exclude ascent / landing. A suffix of all
    # airborne samples accidentally makes landing look like poor hover.
    hover = [r for r in rows if -r["pd"] > max_altitude - 0.25 and r["motor_0"] > 0.1]
    def rms(values: list[float]) -> float | None:
        return None if not values else float(np.sqrt(np.mean(np.square(values))))

    metrics = {
        "samples": len(rows),
        "sim_duration_s": rows[-1]["time"] - rows[0]["time"],
        "wall_duration_s": rows[-1]["wall_elapsed_s"] - rows[0]["wall_elapsed_s"],
        "rtf": ((rows[-1]["time"] - rows[0]["time"])
                / max(rows[-1]["wall_elapsed_s"] - rows[0]["wall_elapsed_s"], 1e-9)),
        "max_altitude_m": max_altitude,
        "hover_samples": len(hover),
        "hover_rms_xy_m": rms([math.hypot(r["pn"], r["pe"]) for r in hover]),
        "hover_rms_z_about_mean_m": rms([(-r["pd"] - np.mean([-h["pd"] for h in hover])) for r in hover]) if hover else None,
        "hover_roll_max_abs_deg": max((abs(r["roll_deg"]) for r in hover), default=None),
        "hover_pitch_max_abs_deg": max((abs(r["pitch_deg"]) for r in hover), default=None),
        "motor_mean_hover": [float(np.mean([r[f"motor_{i}"] for r in hover])) for i in range(4)] if hover else None,
        "cpu_total_mean_percent": float(np.mean(cpu_samples)) if cpu_samples else None,
        "nan_detected": any(not math.isfinite(float(v)) for r in rows for v in r.values()),
        "tether_tension_max_n": max(r["tether_tension_n"] for r in rows),
        "tether_connection_error_max_m": max(r["tether_error_m"] for r in rows),
        "tether_anchor_drift_max_m": max(r["tether_anchor_drift_m"] for r in rows),
    }
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
