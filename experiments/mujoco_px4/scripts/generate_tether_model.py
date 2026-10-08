#!/usr/bin/env python3
"""Generate a native MuJoCo ball-joint tether, optionally connected to X500."""

from __future__ import annotations

import argparse
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def fmt(values) -> str:
    return " ".join(f"{float(v):.17g}" for v in values)


def loop_points(count: int, length: float, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """A horizontal loop returning to the UAV, with exact discrete arc length."""
    s = np.linspace(0.0, 1.0, count + 1)
    def points(radius: float) -> np.ndarray:
        return np.column_stack((
            start[0] + radius * np.sin(2 * math.pi * s),
            start[1] + radius * (1 - np.cos(2 * math.pi * s)),
            start[2] + (end[2] - start[2]) * s,
        ))
    lo, hi = 0.0, length
    for _ in range(80):
        mid = (lo + hi) / 2
        arc = np.linalg.norm(np.diff(points(mid), axis=0), axis=1).sum()
        if arc < length:
            lo = mid
        else:
            hi = mid
    out = points((lo + hi) / 2)
    out[-1] = end
    return out


def straight_points(count: int, length: float, start: np.ndarray) -> np.ndarray:
    return np.column_stack((
        np.linspace(start[0], start[0] + length, count + 1),
        np.full(count + 1, start[1]),
        np.full(count + 1, start[2]),
    ))


def add_chain(world: ET.Element, points: np.ndarray, mass_total: float, radius: float) -> None:
    parent = world
    for index, delta in enumerate(np.diff(points, axis=0), start=1):
        pos = points[0] if index == 1 else delta_prev
        body = ET.SubElement(parent, "body", name=f"tether_link_{index}", pos=fmt(pos))
        ET.SubElement(
            body, "joint", name=f"tether_joint_{index}", type="ball",
            damping="0.002", armature="1e-6",
        )
        ET.SubElement(
            body, "geom", name=f"tether_geom_{index}", type="capsule",
            fromto=fmt([0, 0, 0, *delta]), size=f"{radius:.17g}",
            mass=f"{mass_total / (len(points) - 1):.17g}",
            contype="2", conaffinity="1", rgba="0.04 0.04 0.04 1",
        )
        parent = body
        delta_prev = delta
    ET.SubElement(parent, "site", name="tether_endpoint", pos=fmt(delta_prev),
                  size="0.012", rgba="0 0.3 1 1")
    ET.SubElement(world, "site", name="tether_anchor", pos=fmt(points[0]),
                  size="0.015", rgba="1 0.1 0.1 1")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--links", type=int, default=30)
    parser.add_argument("--length", type=float, default=2.5)
    parser.add_argument("--rho", type=float, default=0.06)
    parser.add_argument("--radius", type=float, default=0.003)
    parser.add_argument("--tether-only", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.links < 2 or args.length <= 0 or args.rho <= 0 or args.radius <= 0:
        parser.error("links >= 2 and positive length/rho/radius are required")

    root_dir = Path(__file__).resolve().parents[1]
    source = ET.parse(root_dir / "models/x500.xml")
    root = source.getroot()
    world = root.find("worldbody")
    assert world is not None
    if args.tether_only:
        for body in list(world.findall("body")):
            world.remove(body)
        sensors = root.find("sensor")
        if sensors is not None:
            root.remove(sensors)
        points = straight_points(args.links, args.length, np.array([0.0, 0.0, 0.01]))
        root.set("model", f"tether_static_n{args.links}")
    else:
        base = world.find("body[@name='base_link']")
        assert base is not None
        ET.SubElement(base, "site", name="tether_attach", pos="0 0 -0.12",
                      size="0.012", rgba="0 0.8 0.2 1")
        start = np.array([0.0, 0.0, 0.01])
        end = np.array([0.0, 0.0, 0.12])
        points = loop_points(args.links, args.length, start, end)
        equality = ET.SubElement(root, "equality")
        ET.SubElement(
            equality, "connect", name="tether_uav_connection",
            # MuJoCo interprets connect.anchor in body1's local frame. The last
            # body origin is the start of the last segment; its endpoint is the
            # final segment delta, not the endpoint's world coordinate.
            body1=f"tether_link_{args.links}", body2="base_link",
            anchor=fmt(points[-1] - points[-2]),
            solref="0.01 1", solimp="0.95 0.99 0.001",
        )
        root.set("model", f"x500_tether_n{args.links}")

    add_chain(world, points, args.rho * args.length, args.radius)
    ET.indent(source, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    source.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"model={args.output}")
    print(f"N={args.links} L={args.length:.6f} mass={args.rho * args.length:.6f}")
    print(f"initial_polyline={np.linalg.norm(np.diff(points, axis=0), axis=1).sum():.12f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
