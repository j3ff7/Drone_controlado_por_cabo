#!/usr/bin/env python3
"""B2 - consolida o sweep de escalabilidade (L, l, N) em tabelas e graficos.

Le os diretorios produzidos por `run_b1_discretization.py` sob `results/b2/` e monta a
comparacao por comprimento total e por resolucao do cabo.
"""
import sys

sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import importlib.util  # noqa: E402
import json  # noqa: E402
from pathlib import Path  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('b1', ROOT / 'tools' / 'run_b1_discretization.py')
B1 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(B1)


def collect(results_dir):
    rows = []
    for run_dir in sorted(Path(results_dir).glob('*/n*')):
        if not (run_dir / 'px4.log').exists():
            continue
        summary = B1.summarize(run_dir, 'estatico', run_dir / 'px4.log')
        manifest = run_dir / 'estatico_record_manifest.json'
        span = None
        if manifest.exists():
            span = json.loads(manifest.read_text()).get('sim_time_span_s')
        # L vem do nome do diretorio pai: LxpX_lYpY
        tag = run_dir.parent.name
        length = float(tag.split('_')[0][1:].replace('p', '.'))
        n_links = int(run_dir.name[1:])
        rtf = B1.column(run_dir / 'estatico_world_stats.csv', 'rtf')
        rtf = rtf[np.isfinite(rtf)]
        rows.append({
            'tag': tag, 'length_m': length, 'n_links': n_links,
            'segment_length_m': length / n_links,
            'link_mass_kg': 0.06 * length / n_links,
            'rtf_mean': float(np.mean(rtf)) if rtf.size else None,
            'rtf_p05': float(np.percentile(rtf, 5)) if rtf.size else None,
            'wall_per_sim_s': summary.get('wall_per_sim_s'),
            'error': summary['constraint_error_m'],
            'force': summary['force_uav_n'],
            'tension': summary['T_est_quasi_static_n'],
            'saturation': summary['saturation_fraction'],
            'sim_time_span_s': span,
            'stable': summary['stable'],
            'dart_abort': summary['dart_abort'],
        })
    return rows


def fmt(value, digits=4):
    return 'N/D' if value is None else f'{value:.{digits}f}'


def table(rows):
    header = ('| L [m] | N | l [m] | m_link [kg] | RTF med | RTF p05 | s parede/s sim '
              '| \\|e\\| RMS | \\|e\\| max | \\|F\\| RMS | \\|F\\| max | T_est RMS | T_est max '
              '| sat | estado |')
    lines = [header, '| ' + ' | '.join(['---'] * 15) + ' |']
    for row in sorted(rows, key=lambda r: (r['length_m'], r['n_links'])):
        e, f, t = row['error'], row['force'], row['tension']
        get = lambda d, k: fmt(d[k]) if d else 'N/D'  # noqa: E731
        lines.append(
            f"| {row['length_m']:.1f} | {row['n_links']} | {row['segment_length_m']:.4f} "
            f"| {row['link_mass_kg']:.5f} | {fmt(row['rtf_mean'])} | {fmt(row['rtf_p05'])} "
            f"| {fmt(row['wall_per_sim_s'], 3)} | {get(e, 'rms')} | {get(e, 'max')} "
            f"| {get(f, 'rms')} | {get(f, 'max')} | {get(t, 'rms')} | {get(t, 'max')} "
            f"| {fmt(row['saturation'])} | {'ok' if row['stable'] else 'ABORT'} |")
    return '\n'.join(lines)


def plots(rows, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    lengths = sorted({r['length_m'] for r in rows})
    ok = [r for r in rows if r['stable'] and r['rtf_mean'] is not None]

    def series(length, xkey, ykey, sub=None):
        picked = [r for r in ok if r['length_m'] == length]
        picked.sort(key=lambda r: r[xkey] if sub is None else r[xkey])
        xs, ys = [], []
        for r in picked:
            value = r[ykey][sub] if sub else r[ykey]
            if value is not None:
                xs.append(r[xkey])
                ys.append(value)
        return xs, ys

    specs = [
        ('rtf_vs_n.png', 'n_links', 'rtf_mean', None, 'N (numero de elos)', 'RTF medio', 'RTF x N'),
        ('rtf_vs_segment.png', 'segment_length_m', 'rtf_mean', None,
         'comprimento do segmento l [m]', 'RTF medio', 'RTF x l'),
        ('tension_vs_segment.png', 'segment_length_m', 'tension', 'rms',
         'comprimento do segmento l [m]', 'T_est RMS [N]', 'T_est x l'),
        ('force_vs_segment.png', 'segment_length_m', 'force', 'rms',
         'comprimento do segmento l [m]', '|F_uav| RMS [N]', '|F_uav| x l'),
        ('error_vs_segment.png', 'segment_length_m', 'error', 'rms',
         'comprimento do segmento l [m]', 'constraint error RMS [m]', 'erro de constraint x l'),
    ]
    for name, xkey, ykey, sub, xlabel, ylabel, title in specs:
        fig, ax = plt.subplots(figsize=(8, 5))
        for length in lengths:
            xs, ys = series(length, xkey, ykey, sub)
            if xs:
                ax.plot(xs, ys, 'o-', label=f'L = {length:.1f} m')
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(f'B2 - {title}')
        ax.grid(alpha=0.3)
        if lengths:
            ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out_dir / name, dpi=110)
        plt.close(fig)

    # RTF x L, uma curva por resolucao nominal
    fig, ax = plt.subplots(figsize=(8, 5))
    for nominal in sorted({round(r['segment_length_m'], 2) for r in ok}):
        picked = sorted([r for r in ok if round(r['segment_length_m'], 2) == nominal],
                        key=lambda r: r['length_m'])
        if picked:
            ax.plot([r['length_m'] for r in picked], [r['rtf_mean'] for r in picked],
                    'o-', label=f'l ~ {nominal:.2f} m')
    ax.set_xlabel('comprimento total L [m]')
    ax.set_ylabel('RTF medio')
    ax.set_title('B2 - RTF x L')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / 'rtf_vs_length.png', dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', default='results/b2')
    args = parser.parse_args()

    rows = collect(args.results_dir)
    if not rows:
        raise SystemExit(f'nenhuma corrida encontrada em {args.results_dir}')

    out = Path(args.results_dir)
    text = table(rows)
    (out / 'b2_scalability.md').write_text(text + '\n')
    (out / 'b2_scalability.json').write_text(json.dumps(rows, indent=2))
    plots(rows, out / 'plots')
    print(text)
    print()
    stable = [r for r in rows if r['stable']]
    if stable:
        print('maior N estavel  =', max(r['n_links'] for r in stable))
        print('maior L estavel  =', max(r['length_m'] for r in stable))
        print('menor l estavel  =', min(r['segment_length_m'] for r in stable))


if __name__ == '__main__':
    main()
