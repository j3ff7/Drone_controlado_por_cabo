#!/usr/bin/env python3
"""Correlate stage-local RTF with tether-floor contact count."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path

import numpy as np


def write_summary_artifacts(output: Path, table: list[dict]) -> None:
    csv_path = output.with_name("contact_rtf_summary.csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)

    width, height = 820, 500
    left, right, top, bottom = 80, 35, 55, 70
    plot_w, plot_h = width - left - right, height - top - bottom
    max_links = 70.0
    max_rtf = max(row["rtf"] for row in table) * 1.12

    def x(value: float) -> float:
        return left + plot_w * value / max_links

    def y(value: float) -> float:
        return top + plot_h * (1.0 - value / max_rtf)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:sans-serif;fill:#222}.grid{stroke:#ddd;stroke-width:1}.axis{stroke:#222;stroke-width:1.5}.curve{fill:none;stroke:#1769aa;stroke-width:2}.point{fill:#d33;stroke:white;stroke-width:1.5}</style>',
        f'<text x="{width / 2}" y="28" text-anchor="middle" font-size="20">RTF vs tether links touching the floor</text>',
    ]
    for links in range(0, 71, 10):
        px = x(links)
        lines.extend([
            f'<line class="grid" x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top + plot_h}"/>',
            f'<text x="{px:.1f}" y="{top + plot_h + 25}" text-anchor="middle" font-size="13">{links}</text>',
        ])
    for index in range(6):
        value = max_rtf * index / 5
        py = y(value)
        lines.extend([
            f'<line class="grid" x1="{left}" y1="{py:.1f}" x2="{left + plot_w}" y2="{py:.1f}"/>',
            f'<text x="{left - 12}" y="{py + 4:.1f}" text-anchor="end" font-size="13">{value:.3f}</text>',
        ])
    lines.extend([
        f'<line class="axis" x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}"/>',
        f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}"/>',
        f'<text x="{left + plot_w / 2}" y="{height - 18}" text-anchor="middle" font-size="15">Mean number of contacting links (of 70)</text>',
        f'<text x="20" y="{top + plot_h / 2}" text-anchor="middle" font-size="15" transform="rotate(-90 20 {top + plot_h / 2})">RTF</text>',
    ])
    points = " ".join(
        f'{x(row["ground_contact_links_mean"]):.1f},{y(row["rtf"]):.1f}'
        for row in table
    )
    lines.append(f'<polyline class="curve" points="{points}"/>')
    for row in table:
        px = x(row["ground_contact_links_mean"])
        py = y(row["rtf"])
        lines.extend([
            f'<circle class="point" cx="{px:.1f}" cy="{py:.1f}" r="5"/>',
            f'<text x="{px + 8:.1f}" y="{py - 8:.1f}" font-size="12">{row["altitude_mean_m"]:.2f} m</text>',
        ])
    lines.append('</svg>')
    output.with_name("contact_rtf.svg").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("stages", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = list(csv.DictReader(args.csv.open(encoding="utf-8")))
    profile = json.loads(args.stages.read_text(encoding="utf-8"))
    table = []
    for stage in profile["stages"]:
        begin, end = stage["settled_start_sim_s"], stage["settled_end_sim_s"]
        selected = [row for row in rows if begin is not None and begin <= float(row["time"]) <= end]
        if len(selected) < 2:
            continue
        sim_dt = float(selected[-1]["time"]) - float(selected[0]["time"])
        wall_dt = float(selected[-1]["wall_elapsed_s"]) - float(selected[0]["wall_elapsed_s"])
        contacts = [int(float(row["tether_ground_contacts"])) for row in selected]
        contact_links = [int(float(row["tether_ground_contact_links"])) for row in selected]
        altitudes = [-float(row["pd"]) for row in selected]
        roll = [abs(float(row["roll_deg"])) for row in selected]
        pitch = [abs(float(row["pitch_deg"])) for row in selected]
        tensions = [float(row["tether_tension_n"]) for row in selected]
        table.append({
            "stage": stage["index"],
            "drop_m": stage["drop_m"],
            "altitude_mean_m": statistics.mean(altitudes),
            "samples": len(selected),
            "rtf": sim_dt / wall_dt,
            "ground_contacts_mean": statistics.mean(contacts),
            "ground_contacts_median": statistics.median(contacts),
            "ground_contacts_max": max(contacts),
            "ground_contact_links_mean": statistics.mean(contact_links),
            "ground_contact_links_median": statistics.median(contact_links),
            "ground_contact_links_max": max(contact_links),
            "tension_mean_n": statistics.mean(tensions),
            "tension_max_n": max(tensions),
            "roll_max_abs_deg": max(roll),
            "pitch_max_abs_deg": max(pitch),
        })
    correlation = None
    if len(table) >= 2:
        correlation = float(np.corrcoef(
            [row["ground_contact_links_mean"] for row in table],
            [row["rtf"] for row in table],
        )[0, 1])
    result = {
        "profile_pass": profile["pass"],
        "stages_analyzed": len(table),
        "contact_rtf_correlation": correlation,
        "stages": table,
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    if table:
        write_summary_artifacts(args.output, table)
    print(json.dumps(result, indent=2))
    return 0 if profile["pass"] and len(table) == len(profile["stages"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
