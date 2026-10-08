#!/usr/bin/env python3
"""Run a tether-only MJCF and report stability/contact metrics."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np
import psutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    dt = float(model.opt.timestep)
    steps = round(args.duration / dt)
    wall0 = time.monotonic()
    cpu0 = psutil.cpu_times()
    max_speed = 0.0
    min_z = math.inf
    max_anchor_drift = 0.0
    anchor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tether_anchor")
    anchor0 = data.site_xpos[anchor_id].copy()
    nan = False
    for _ in range(steps):
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            nan = True
            break
        max_speed = max(max_speed, float(np.max(np.abs(data.qvel))))
        min_z = min(min_z, float(np.min(data.xpos[:, 2])))
        max_anchor_drift = max(max_anchor_drift, float(np.linalg.norm(data.site_xpos[anchor_id] - anchor0)))
    wall = time.monotonic() - wall0
    cpu1 = psutil.cpu_times()
    cpu_used = (cpu1.user + cpu1.system) - (cpu0.user + cpu0.system)
    metrics = {
        "model_build": "PASS",
        "simulation_stable": not nan,
        "sim_time_s": float(data.time),
        "wall_time_s": wall,
        "rtf": float(data.time) / wall,
        "cpu_total_percent": 100.0 * cpu_used / max(wall, 1e-9) / psutil.cpu_count(),
        "nan": nan,
        "max_abs_qvel": max_speed,
        "minimum_body_z_m": min_z,
        "anchor_drift_m": max_anchor_drift,
        "contacts_final": int(data.ncon),
        "pass": not nan and max_anchor_drift < 1e-9,
    }
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
