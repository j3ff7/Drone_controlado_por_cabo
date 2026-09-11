#!/usr/bin/env python3
"""Analisa a estimativa de tensao no lado terrestre (A5).

Compara o estimador completo (com termo inercial) e o quase-estatico contra a forca de
saida `F_exit`, a tangente local e a forca da constraint do lado do UAV.
"""
import sys

sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    if not Path(path).exists():
        return None
    with Path(path).open() as handle:
        rows = list(csv.DictReader(handle))
    arr = lambda k: np.array([float(r[k]) if r[k] not in ('', None) else math.nan for r in rows])  # noqa: E731
    return {'t': arr('t_sim'), 'x': arr('x'), 'y': arr('y'), 'z': arr('z')}


def stats(values, name):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    if not finite.size:
        return {'name': name, 'samples': 0}
    diff = np.diff(finite)
    return {
        'name': name,
        'samples': int(finite.size),
        'nan_count': int(values.size - finite.size),
        'nan_fraction': float((values.size - finite.size) / values.size),
        'mean': float(np.mean(finite)),
        'rms': float(np.sqrt(np.mean(finite ** 2))),
        'max': float(np.max(finite)),
        'min': float(np.min(finite)),
        'std': float(np.std(finite)),
        # Ruido de alta frequencia: desvio das diferencas amostra a amostra.
        'step_noise_std': float(np.std(diff)) if diff.size else None,
        'step_noise_max': float(np.max(np.abs(diff))) if diff.size else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--prefix', required=True)
    args = parser.parse_args()

    run = Path(args.run_dir)
    tension = load(run / f'{args.prefix}_tensao.csv')
    exit_force = load(run / f'{args.prefix}_exit_force.csv')
    tangent = load(run / f'{args.prefix}_exit_tangent.csv')
    uav = load(run / f'{args.prefix}_force.csv')
    if tension is None:
        raise SystemExit(f'sem series de tensao em {run}')

    available = tension['z'] >= 0.5
    t_full, t_qs = tension['x'], tension['y']

    f_exit_mag = (np.sqrt(exit_force['x'] ** 2 + exit_force['y'] ** 2 + exit_force['z'] ** 2)
                  if exit_force else None)
    uav_mag = (np.sqrt(uav['x'] ** 2 + uav['y'] ** 2 + uav['z'] ** 2) if uav else None)
    tangent_norm = (np.sqrt(tangent['x'] ** 2 + tangent['y'] ** 2 + tangent['z'] ** 2)
                    if tangent else None)

    report = {
        'run_dir': str(run),
        'prefix': args.prefix,
        'availability_fraction': float(np.mean(available)),
        'gaps_unavailable': int(np.count_nonzero(~available)),
        'T_est_full': stats(t_full, 'T_est (I*a incluido)'),
        'T_est_quasi_static': stats(t_qs, 'T_est quase-estatico'),
        'F_exit_magnitude': stats(f_exit_mag, '|F_exit|') if f_exit_mag is not None else None,
        'F_uav_magnitude': stats(uav_mag, '|F_uav|') if uav_mag is not None else None,
        'tangent_unit_norm': stats(tangent_norm, '|t_hat|') if tangent_norm is not None else None,
    }

    if f_exit_mag is not None:
        # |F_exit| e publicado com o termo inercial, entao a fracao axial tem de ser
        # comparada contra o estimador completo, nao contra o quase-estatico.
        n = min(len(t_full), len(f_exit_mag))
        ratio = t_full[:n] / np.where(f_exit_mag[:n] > 1e-9, f_exit_mag[:n], np.nan)
        finite = ratio[np.isfinite(ratio)]
        report['axial_fraction_of_total'] = {
            'note': 'T_est completo dividido por |F_exit|; 1,0 significa forca puramente axial',
            'mean': float(np.mean(finite)) if finite.size else None,
            'min': float(np.min(finite)) if finite.size else None,
            'max': float(np.max(finite)) if finite.size else None,
        }
    if uav_mag is not None:
        n = min(len(t_qs), len(uav_mag))
        report['ground_vs_uav'] = {
            'T_est_qs_mean': float(np.nanmean(t_qs[:n])),
            'F_uav_mean': float(np.nanmean(uav_mag[:n])),
            'difference_mean': float(np.nanmean(t_qs[:n] - uav_mag[:n])),
        }

    plot_dir = run / 'plots'
    plot_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(tension['t'], t_full, lw=0.6, label='T_est com I*a')
    axes[0].plot(tension['t'], t_qs, lw=1.0, label='T_est quase-estatico')
    axes[0].set_ylabel('T_est [N]')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    if f_exit_mag is not None:
        axes[1].plot(exit_force['t'], f_exit_mag, lw=0.8, color='tab:red', label='|F_exit|')
    if uav_mag is not None:
        axes[1].plot(uav['t'], uav_mag, lw=0.8, color='tab:blue', label='|F_uav|')
    axes[1].set_ylabel('forca [N]')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    if tangent is not None:
        for key, lbl in (('x', 'tx'), ('y', 'ty'), ('z', 'tz')):
            axes[2].plot(tangent['t'], tangent[key], lw=0.7, label=lbl)
    axes[2].set_ylabel('t_hat')
    axes[2].set_xlabel('t_sim [s]')
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)
    fig.suptitle(f'A5 - tensao no lado terrestre ({args.prefix})')
    fig.tight_layout()
    fig.savefig(plot_dir / f'{args.prefix}_tensao.png', dpi=110)
    plt.close(fig)

    (run / f'{args.prefix}_tension_analysis.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
