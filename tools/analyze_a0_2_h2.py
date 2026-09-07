#!/usr/bin/env python3
"""Caracteriza a constraint force-based a partir dos dados de H2.

Reconstroi o relogio wall->sim a partir de /stats, junta a missao OFFBOARD com as
series /cabo/*, verifica a lei F = -K e - C e_dot componente a componente e gera
os plots headless da rodada A0.2.
"""
import sys

# matplotlib do sistema foi compilado contra numpy 1.x; o numpy 2.x local do
# usuario quebra a importacao. Priorizar dist-packages apenas neste processo.
sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import bisect  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


K = 5.0
C = 0.5
FMAX = 3.0


def read_csv(path):
    with Path(path).open() as handle:
        return list(csv.DictReader(handle))


def to_float(value):
    if value is None or value == '':
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def sanitized_clock(world_rows):
    """Descarta amostras nao finitas ou nao monotonicas do relogio de simulacao."""
    walls, sims, dropped = [], [], 0
    for row in world_rows:
        t_wall = to_float(row['t_wall'])
        sim = to_float(row['sim_time'])
        if not (math.isfinite(t_wall) and math.isfinite(sim)):
            dropped += 1
            continue
        if walls and (t_wall <= walls[-1] or sim < sims[-1]):
            dropped += 1
            continue
        walls.append(t_wall)
        sims.append(sim)
    return walls, sims, dropped


def make_to_sim(walls, sims):
    def to_sim(t_wall):
        if not walls:
            return math.nan
        index = bisect.bisect_left(walls, t_wall)
        if index <= 0:
            return sims[0] + (t_wall - walls[0])
        if index >= len(walls):
            return sims[-1] + (t_wall - walls[-1])
        w0, w1, s0, s1 = walls[index - 1], walls[index], sims[index - 1], sims[index]
        if w1 <= w0:
            return s0
        return s0 + (s1 - s0) * (t_wall - w0) / (w1 - w0)
    return to_sim


def load_series(path, to_sim):
    rows = read_csv(path)
    t = np.array([to_sim(to_float(row['t_wall'])) for row in rows])
    x = np.array([to_float(row['x']) for row in rows])
    y = np.array([to_float(row['y']) for row in rows])
    z = np.array([to_float(row['z']) for row in rows])
    keep = np.isfinite(t)
    return t[keep], x[keep], y[keep], z[keep]


def centered_derivative(t, values, half_window=3):
    """Derivada central com janela larga para atenuar ruido de passo fixo."""
    out = np.full_like(values, np.nan)
    n = len(values)
    for i in range(n):
        lo = max(0, i - half_window)
        hi = min(n - 1, i + half_window)
        dt = t[hi] - t[lo]
        if dt > 0:
            out[i] = (values[hi] - values[lo]) / dt
    return out


def rms(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values ** 2))) if values.size else None


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
    parser.add_argument('--prefix', default='h2')
    parser.add_argument('--plot-dir', default=None)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    plot_dir = Path(args.plot_dir) if args.plot_dir else run_dir / 'plots'
    plot_dir.mkdir(parents=True, exist_ok=True)

    world_rows = read_csv(run_dir / f'{args.prefix}_world_stats.csv')
    walls, sims, dropped_clock = sanitized_clock(world_rows)
    to_sim = make_to_sim(walls, sims)
    rtfs = np.array([to_float(row['rtf']) for row in world_rows])
    rtfs = rtfs[np.isfinite(rtfs)]

    te, ex, ey, ez = load_series(run_dir / f'{args.prefix}_error.csv', to_sim)
    tf, fx, fy, fz = load_series(run_dir / f'{args.prefix}_force.csv', to_sim)
    ts, e_norm, f_norm, sat = load_series(run_dir / f'{args.prefix}_stats.csv', to_sim)
    ta, a_f, a_m, a_flag = load_series(run_dir / f'{args.prefix}_anchor.csv', to_sim)

    mission_rows = read_csv(run_dir / 'px4_offboard_horizontal_mission.csv')
    m_t = np.array([to_sim(to_float(row['t_epoch'])) for row in mission_rows])
    mission = {
        key: np.array([to_float(row[key]) for row in mission_rows])
        for key in ('x_ref', 'y_ref', 'z_ref', 'x', 'y', 'z',
                    'ex', 'ey', 'ez', 'e_xy', 'roll_deg', 'pitch_deg')
    }
    phases = [row['phase'] for row in mission_rows]

    # Janela de voo: do inicio do stream OFFBOARD ao fim da missao.
    flight_lo, flight_hi = float(np.nanmin(m_t)), float(np.nanmax(m_t))
    window = (ts >= flight_lo) & (ts <= flight_hi)

    # Lei da constraint: comparacao componente a componente.
    dex = centered_derivative(te, ex)
    dey = centered_derivative(te, ey)
    dez = centered_derivative(te, ez)
    # /cabo/conexao/error e /cabo/conexao/force sao publicados no mesmo PreUpdate;
    # alinhar por indice quando os comprimentos coincidem, senao por interpolacao.
    n = min(len(te), len(tf))
    model_x = -K * ex[:n] - C * dex[:n]
    model_y = -K * ey[:n] - C * dey[:n]
    model_z = -K * ez[:n] - C * dez[:n]
    model_norm = np.sqrt(model_x ** 2 + model_y ** 2 + model_z ** 2)
    scale = np.where(model_norm > FMAX, FMAX / np.maximum(model_norm, 1e-12), 1.0)
    model_x, model_y, model_z = model_x * scale, model_y * scale, model_z * scale

    stiffness_only_x = -K * ex[:n]
    stiffness_only_y = -K * ey[:n]
    stiffness_only_z = -K * ez[:n]

    residual_x = fx[:n] - model_x
    residual_y = fy[:n] - model_y
    residual_z = fz[:n] - model_z
    residual_stiff = np.sqrt(
        (fx[:n] - stiffness_only_x) ** 2
        + (fy[:n] - stiffness_only_y) ** 2
        + (fz[:n] - stiffness_only_z) ** 2
    )
    residual_full = np.sqrt(residual_x ** 2 + residual_y ** 2 + residual_z ** 2)

    # A derivada central nao tem historico no primeiro instante publicado, o que
    # cria um transiente artificial de verificacao. Reportar com e sem essa janela.
    t_law = te[:n]
    settled = np.isfinite(t_law) & (t_law > (t_law[0] + 1.0))

    summary = {
        'run_dir': str(run_dir),
        'constraint_parameters': {'K_n_per_m': K, 'C_ns_per_m': C, 'fmax_n': FMAX},
        'clock': {
            'world_stats_rows': len(world_rows),
            'dropped_clock_rows': dropped_clock,
            'sim_time_span_s': (sims[-1] - sims[0]) if len(sims) > 1 else None,
            'rtf_mean': float(np.mean(rtfs)) if rtfs.size else None,
            'rtf_min': float(np.min(rtfs)) if rtfs.size else None,
            'rtf_p05': float(np.percentile(rtfs, 5)) if rtfs.size else None,
        },
        'flight_window_sim_s': [flight_lo, flight_hi],
        'constraint_uav': {
            'samples_in_window': int(np.count_nonzero(window)),
            'error_norm': summarize(e_norm[window]),
            'force_norm': summarize(f_norm[window]),
            'saturation_fraction': float(np.mean(sat[window] >= 0.5)) if np.any(window) else None,
            'error_components': {
                'ex': summarize(ex), 'ey': summarize(ey), 'ez': summarize(ez),
            },
            'force_components': {
                'fx': summarize(fx), 'fy': summarize(fy), 'fz': summarize(fz),
            },
        },
        'law_check': {
            'samples': int(n),
            'residual_full_model_n': summarize(residual_full),
            'residual_full_model_settled_n': summarize(residual_full[settled]),
            'residual_full_model_settled_samples': int(np.count_nonzero(settled)),
            'residual_full_model_percentiles_n': {
                'p50': float(np.nanpercentile(residual_full, 50)),
                'p95': float(np.nanpercentile(residual_full, 95)),
                'p99': float(np.nanpercentile(residual_full, 99)),
            },
            'residual_stiffness_only_n': summarize(residual_stiff),
            'residual_stiffness_only_settled_n': summarize(residual_stiff[settled]),
            'residual_per_axis_n': {
                'x': summarize(residual_x),
                'y': summarize(residual_y),
                'z': summarize(residual_z),
            },
            'k_estimated_from_norms_n_per_m': (
                float(np.sum(e_norm * f_norm) / np.sum(e_norm ** 2))
                if np.sum(e_norm ** 2) > 0 else None
            ),
        },
        'anchor_endpoint': {
            'topic': '/cabo/anchor/stats',
            'messages': int(len(ta)),
            'force_magnitude': 'N/D',
            'moment_magnitude': 'N/D',
            'nan_fraction_force_field': float(np.mean(~np.isfinite(a_f))) if len(a_f) else None,
            'availability_flag_values': sorted({float(v) for v in a_flag[:50]}) if len(a_flag) else [],
            'note': 'x=|F| e y=|M| sao NaN por ausencia de medida; z=0 indica indisponibilidade.',
        },
        'mission_control': {
            'rms_xy_tracking_m': rms(
                [mission['e_xy'][i] for i, p in enumerate(phases)
                 if p in ('translate_out', 'return_home')]),
            'roll_max_abs_deg': float(np.nanmax(np.abs(mission['roll_deg']))),
            'pitch_max_abs_deg': float(np.nanmax(np.abs(mission['pitch_deg']))),
        },
    }

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(plot_dir / name, dpi=110)
        plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, ref, act, label in zip(
            axes,
            ('x_ref', 'y_ref', 'z_ref'), ('x', 'y', 'z'), ('X [m]', 'Y [m]', 'Z [m] (NED)')):
        ax.plot(m_t, mission[ref], label=f'{ref}')
        ax.plot(m_t, mission[act], label=f'{act}')
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=8)
    axes[-1].set_xlabel('t_sim [s]')
    fig.suptitle('H2 - referencia vs posicao (OFFBOARD, dx=0.5 m, h=2.0 m)')
    save(fig, 'h2_position_tracking.png')

    fig, ax = plt.subplots(figsize=(10, 4))
    for key in ('ex', 'ey', 'ez'):
        ax.plot(m_t, mission[key], label=key)
    ax.set_xlabel('t_sim [s]')
    ax.set_ylabel('erro de posicao [m]')
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title('H2 - erro de rastreamento do X500')
    save(fig, 'h2_position_error.png')

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(m_t, mission['roll_deg'], label='roll')
    ax.plot(m_t, mission['pitch_deg'], label='pitch')
    ax.set_xlabel('t_sim [s]')
    ax.set_ylabel('atitude [deg]')
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title('H2 - roll e pitch')
    save(fig, 'h2_attitude.png')

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(ts, e_norm, lw=0.7)
    axes[0].set_ylabel('|e| [m]')
    axes[0].grid(alpha=0.3)
    axes[0].set_title('H2 - erro da constraint e forca aplicada')
    axes[1].plot(ts, f_norm, lw=0.7, color='tab:red')
    axes[1].axhline(FMAX, ls='--', color='k', lw=0.8, label='Fmax = 3 N')
    axes[1].set_ylabel('|F| [N]')
    axes[1].set_xlabel('t_sim [s]')
    axes[1].grid(alpha=0.3)
    axes[1].legend()
    save(fig, 'h2_constraint_error_force.png')

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(e_norm, f_norm, '.', ms=1, alpha=0.3, label='amostras')
    span = np.linspace(0, float(np.nanmax(e_norm)) * 1.05, 50)
    ax.plot(span, K * span, 'k--', lw=1.2, label='K*|e| (K = 5 N/m)')
    ax.set_xlabel('|e| [m]')
    ax.set_ylabel('|F| [N]')
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title('H2 - |F| vs |e|')
    save(fig, 'h2_force_vs_error.png')

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, comp, published, modeled in zip(
            axes, ('x', 'y', 'z'), (fx[:n], fy[:n], fz[:n]), (model_x, model_y, model_z)):
        ax.plot(tf[:n], published, lw=0.7, label=f'F{comp} publicado')
        ax.plot(te[:n], modeled, lw=0.7, ls='--', label=f'-K e{comp} - C de{comp}/dt')
        ax.set_ylabel(f'F{comp} [N]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[-1].set_xlabel('t_sim [s]')
    fig.suptitle('H2 - lei da constraint componente a componente')
    save(fig, 'h2_law_components.png')

    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot(ts, sat, lw=0.8)
    ax.set_ylim(-0.1, 1.1)
    ax.set_xlabel('t_sim [s]')
    ax.set_ylabel('saturado (0/1)')
    ax.grid(alpha=0.3)
    ax.set_title(f'H2 - estado de saturacao (fracao = {summary["constraint_uav"]["saturation_fraction"]})')
    save(fig, 'h2_saturation.png')

    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot(np.arange(len(rtfs)), rtfs, lw=0.6)
    ax.set_xlabel('amostra de /stats')
    ax.set_ylabel('RTF')
    ax.grid(alpha=0.3)
    ax.set_title('H2 - real time factor')
    save(fig, 'h2_rtf.png')

    summary['plots'] = sorted(str(p) for p in plot_dir.glob('*.png'))
    out = run_dir / 'h2_characterization.json'
    out.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
