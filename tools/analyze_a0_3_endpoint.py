#!/usr/bin/env python3
"""Analise do endpoint fisico de A0.3: lei da constraint e momento r x F.

Calcula offline o torque que a forca da constraint impoe ao veiculo,
tau = r x F_drone, com r = p_attach - CoM_veiculo obtido da geometria do SDF,
e confronta com a resposta angular observada. Nao introduz sensor no plugin.
"""
import sys

sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import bisect  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from pathlib import Path  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
DRONE_MODEL = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'x500_tether_attach' / 'model.sdf'
K, C, FMAX = 5.0, 0.5, 3.0


def parse_xyz(text):
    if not text:
        return np.zeros(3)
    return np.array([float(v) for v in text.split()][:3])


def vehicle_geometry(model_path, attach_link='tether_attach_link'):
    """Massa total, CoM composto e braco r = p_attach - CoM, no frame do base_link."""
    model = ET.parse(model_path).getroot().find('model')
    total_mass, moment, attach_pos = 0.0, np.zeros(3), None
    for link in model.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.findtext('mass'))
        pos = parse_xyz(link.findtext('pose')) + parse_xyz(inertial.findtext('pose'))
        if link.attrib['name'] == attach_link:
            attach_pos = parse_xyz(link.findtext('pose'))
        total_mass += mass
        moment += mass * pos
    if attach_pos is None:
        raise SystemExit(f'{attach_link} nao encontrado em {model_path}')
    com = moment / total_mass
    return total_mass, com, attach_pos - com


def read_csv(path):
    with Path(path).open() as handle:
        return list(csv.DictReader(handle))


def to_float(value):
    if value in (None, ''):
        return math.nan
    try:
        parsed = float(value)
    except ValueError:
        return math.nan
    return parsed


def sim_clock(world_rows):
    walls, sims = [], []
    for row in world_rows:
        t_wall, sim = to_float(row['t_wall']), to_float(row['sim_time'])
        if not (math.isfinite(t_wall) and math.isfinite(sim)):
            continue
        if walls and (t_wall <= walls[-1] or sim < sims[-1]):
            continue
        walls.append(t_wall)
        sims.append(sim)

    def to_sim(t_wall):
        if not walls:
            return math.nan
        i = bisect.bisect_left(walls, t_wall)
        if i <= 0:
            return sims[0] + (t_wall - walls[0])
        if i >= len(walls):
            return sims[-1] + (t_wall - walls[-1])
        w0, w1, s0, s1 = walls[i - 1], walls[i], sims[i - 1], sims[i]
        return s0 if w1 <= w0 else s0 + (s1 - s0) * (t_wall - w0) / (w1 - w0)
    return to_sim


def load_series(path, to_sim):
    rows = read_csv(path)
    arr = lambda k: np.array([to_float(r[k]) for r in rows])  # noqa: E731
    t = np.array([to_sim(to_float(r['t_wall'])) for r in rows])
    keep = np.isfinite(t)
    return t[keep], arr('x')[keep], arr('y')[keep], arr('z')[keep]


def centered_derivative(t, values, half=3):
    out = np.full_like(values, np.nan)
    for i in range(len(values)):
        lo, hi = max(0, i - half), min(len(values) - 1, i + half)
        dt = t[hi] - t[lo]
        if dt > 0:
            out[i] = (values[hi] - values[lo]) / dt
    return out


def rot_body_to_world(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def summarize(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {'rms': None, 'max': None, 'mean': None, 'samples': 0}
    return {
        'rms': float(np.sqrt(np.mean(values ** 2))),
        'max': float(np.max(np.abs(values))),
        'mean': float(np.mean(values)),
        'samples': int(values.size),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--prefix', required=True)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    plot_dir = run_dir / 'plots'
    plot_dir.mkdir(parents=True, exist_ok=True)

    total_mass, com, r_body = vehicle_geometry(DRONE_MODEL)

    world_rows = read_csv(run_dir / f'{args.prefix}_world_stats.csv')
    to_sim = sim_clock(world_rows)
    rtfs = np.array([to_float(r['rtf']) for r in world_rows])
    rtfs = rtfs[np.isfinite(rtfs)]

    te, ex, ey, ez = load_series(run_dir / f'{args.prefix}_error.csv', to_sim)
    tf, fx, fy, fz = load_series(run_dir / f'{args.prefix}_force.csv', to_sim)
    ts, e_norm, f_norm, sat = load_series(run_dir / f'{args.prefix}_stats.csv', to_sim)

    mission_rows = read_csv(run_dir / 'px4_offboard_horizontal_mission.csv')
    m_t = np.array([to_sim(to_float(r['t_epoch'])) for r in mission_rows])
    m = {k: np.array([to_float(r[k]) for r in mission_rows])
         for k in ('roll_deg', 'pitch_deg', 'yaw_deg', 'e_xy', 'ez', 'x', 'y', 'z')}
    phases = [r['phase'] for r in mission_rows]

    lo, hi = float(np.nanmin(m_t)), float(np.nanmax(m_t))
    window = (ts >= lo) & (ts <= hi)

    # Lei da constraint, componente a componente.
    n = min(len(te), len(tf))
    mx = -K * ex[:n] - C * centered_derivative(te, ex)[:n]
    my = -K * ey[:n] - C * centered_derivative(te, ey)[:n]
    mz = -K * ez[:n] - C * centered_derivative(te, ez)[:n]
    norm = np.sqrt(mx ** 2 + my ** 2 + mz ** 2)
    scale = np.where(norm > FMAX, FMAX / np.maximum(norm, 1e-12), 1.0)
    mx, my, mz = mx * scale, my * scale, mz * scale
    residual = np.sqrt((fx[:n] - mx) ** 2 + (fy[:n] - my) ** 2 + (fz[:n] - mz) ** 2)
    t_law = te[:n]
    settled = np.isfinite(t_law) & (t_law > t_law[0] + 1.0)

    # tau = r x F_drone, com F_drone = -F_publicado (a publicacao e a forca no cabo).
    # r e fixo no corpo; rotacionar para o mundo usando a atitude interpolada.
    roll = np.interp(tf[:n], m_t, np.radians(m['roll_deg']))
    pitch = np.interp(tf[:n], m_t, np.radians(m['pitch_deg']))
    yaw = np.interp(tf[:n], m_t, np.radians(m['yaw_deg']))
    f_drone = np.stack([-fx[:n], -fy[:n], -fz[:n]], axis=1)
    tau = np.empty_like(f_drone)
    f_body = np.empty_like(f_drone)
    for i in range(len(tau)):
        R = rot_body_to_world(roll[i], pitch[i], yaw[i])
        fb = R.T @ f_drone[i]           # forca no frame do corpo
        f_body[i] = fb
        tau[i] = R @ np.cross(r_body, fb)
    tau_norm = np.linalg.norm(tau, axis=1)
    f_horiz_body = np.linalg.norm(f_body[:, :2], axis=1)

    summary = {
        'run_dir': str(run_dir),
        'prefix': args.prefix,
        'geometry': {
            'vehicle_mass_kg': total_mass,
            'com_rel_base_link_m': com.tolist(),
            'p_attach_rel_base_link_m': (com + r_body).tolist(),
            'r_attach_minus_com_m': r_body.tolist(),
            'r_norm_m': float(np.linalg.norm(r_body)),
        },
        'constraint': {
            'K_n_per_m': K, 'C_ns_per_m': C, 'fmax_n': FMAX,
            'samples_in_window': int(np.count_nonzero(window)),
            'error_norm': summarize(e_norm[window]),
            'force_norm': summarize(f_norm[window]),
            'saturation_fraction': float(np.mean(sat[window] >= 0.5)) if np.any(window) else None,
            'k_estimated_n_per_m': (
                float(np.sum(e_norm * f_norm) / np.sum(e_norm ** 2))
                if np.sum(e_norm ** 2) > 0 else None),
            'law_residual_settled_n': summarize(residual[settled]),
            'law_residual_all_n': summarize(residual),
            'law_residual_relative_pct': (
                100.0 * summarize(residual[settled])['rms'] / summarize(f_norm[window])['rms']
                if summarize(f_norm[window])['rms'] else None),
        },
        'torque_r_cross_f': {
            'tau_norm_nm': summarize(tau_norm),
            'force_horizontal_body_n': summarize(f_horiz_body),
            'force_vertical_body_n': summarize(f_body[:, 2]),
            'note': 'tau = r x F_drone calculado offline a partir da geometria e da '
                    'forca publicada; o plugin nao aplica torque explicito.',
        },
        'attitude': {
            'roll_max_abs_deg': float(np.nanmax(np.abs(m['roll_deg']))),
            'pitch_max_abs_deg': float(np.nanmax(np.abs(m['pitch_deg']))),
            'roll_rms_deg': float(np.sqrt(np.nanmean(m['roll_deg'] ** 2))),
            'pitch_rms_deg': float(np.sqrt(np.nanmean(m['pitch_deg'] ** 2))),
        },
        'control': {
            'rms_xy_tracking_m': float(np.sqrt(np.nanmean(np.array(
                [m['e_xy'][i] for i, p in enumerate(phases)
                 if p in ('translate_out', 'return_home')]) ** 2))) if any(
                p in ('translate_out', 'return_home') for p in phases) else None,
            'rms_z_hover_steady_m': float(np.sqrt(np.nanmean(np.array(
                [m['ez'][i] for i, p in enumerate(phases) if p == 'climb_hover'][-100:]) ** 2))),
        },
        'rtf': {
            'mean': float(np.mean(rtfs)) if rtfs.size else None,
            'min': float(np.min(rtfs)) if rtfs.size else None,
            'p05': float(np.percentile(rtfs, 5)) if rtfs.size else None,
        },
    }

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(plot_dir / name, dpi=110)
        plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(ts, e_norm, lw=0.7)
    axes[0].set_ylabel('|e| [m]')
    axes[0].grid(alpha=0.3)
    axes[0].set_title(f'A0.3 {args.prefix} - endpoint fisico tether_attach_link')
    axes[1].plot(ts, f_norm, lw=0.7, color='tab:red')
    axes[1].axhline(FMAX, ls='--', color='k', lw=0.8, label='Fmax = 3 N')
    axes[1].set_ylabel('|F| [N]')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    axes[2].plot(tf[:n], tau_norm, lw=0.7, color='tab:green')
    axes[2].set_ylabel('|r x F| [N.m]')
    axes[2].set_xlabel('t_sim [s]')
    axes[2].grid(alpha=0.3)
    save(fig, f'{args.prefix}_constraint_and_torque.png')

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(e_norm, f_norm, '.', ms=1, alpha=0.3, label='amostras')
    span = np.linspace(0, float(np.nanmax(e_norm)) * 1.05, 50)
    ax.plot(span, K * span, 'k--', lw=1.2, label='K|e| (K = 5 N/m)')
    ax.set_xlabel('|e| [m]')
    ax.set_ylabel('|F| [N]')
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title(f'A0.3 {args.prefix} - |F| vs |e|')
    save(fig, f'{args.prefix}_force_vs_error.png')

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(m_t, m['roll_deg'], label='roll')
    axes[0].plot(m_t, m['pitch_deg'], label='pitch')
    axes[0].set_ylabel('atitude [deg]')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(tf[:n], f_horiz_body, lw=0.7, label='|F_xy| no corpo')
    axes[1].plot(tf[:n], tau_norm, lw=0.7, label='|r x F| [N.m]')
    axes[1].set_xlabel('t_sim [s]')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    axes[0].set_title(f'A0.3 {args.prefix} - atitude vs torque do tether')
    save(fig, f'{args.prefix}_attitude_vs_torque.png')

    summary['plots'] = sorted(str(p) for p in plot_dir.glob(f'{args.prefix}_*.png'))
    (run_dir / f'{args.prefix}_analysis.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
