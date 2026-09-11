#!/usr/bin/env python3
"""Compara etapas do plano a partir dos artefatos ja gravados, sem rodar simulacao.

Substitui os comparadores por-etapa: recebe pares `rotulo=diretorio` e monta a tabela
com missao, constraint e, quando existir, o estado do reel. Grandezas sem dado gravado
saem como `N/D`, nunca como zero.

Exemplo:
  ./tools/compare_stages.py --out results/a2/comparacao_a1_a2 \\
      "A1 estacao fixa=results/a1/horizontal" "A2 reel passivo=results/a2/horizontal"
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_runs = _load('compare_a0_2_runs')
_pair = _load('compare_a0_2_a0_3')

ND = _runs.ND
fmt = _runs.fmt


def find_analysis(run_dir):
    """Localiza o JSON de analise da corrida, seja o formato de A0.2 ou o atual."""
    candidates = sorted(run_dir.glob('*_analysis.json')) + [run_dir / 'h2_characterization.json']
    for path in candidates:
        if path.exists():
            return path.name
    return None


def reel_metrics(run_dir):
    """Estatisticas de theta/omega do reel, se a corrida gravou o topico."""
    matches = sorted(run_dir.glob('*_reel.csv'))
    if not matches:
        return None
    with matches[0].open() as handle:
        rows = list(csv.DictReader(handle))

    theta, omega, flags = [], [], []
    for row in rows:
        try:
            t, o, f = float(row['x']), float(row['y']), float(row['z'])
        except (TypeError, ValueError):
            continue
        if not (math.isfinite(t) and math.isfinite(o)):
            continue
        theta.append(t)
        omega.append(o)
        flags.append(f)

    if not theta:
        return None
    return {
        'samples': len(theta),
        'available_fraction': sum(1 for f in flags if f >= 0.5) / len(flags),
        'theta_min_rad': min(theta),
        'theta_max_rad': max(theta),
        'theta_range_rad': max(theta) - min(theta),
        'omega_abs_max_rad_s': max(abs(o) for o in omega),
        'omega_rms_rad_s': math.sqrt(sum(o * o for o in omega) / len(omega)),
    }


def collect(run_dir):
    run_dir = Path(run_dir)
    analysis = find_analysis(run_dir)
    return {
        'run_dir': str(run_dir),
        'mission': _runs.mission_metrics(run_dir),
        'constraint': _pair.constraint_metrics(run_dir, analysis) if analysis else None,
        'reel': reel_metrics(run_dir),
    }


def build_table(report, columns):
    lines = ['| Metrica | ' + ' | '.join(columns) + ' |',
             '| --- | ' + ' | '.join(['---'] * len(columns)) + ' |']

    def row(name, getter, digits=3):
        cells = []
        for label in columns:
            try:
                cells.append(fmt(getter(report[label]), digits))
            except (TypeError, KeyError):
                cells.append(ND)
        lines.append(f'| {name} | ' + ' | '.join(cells) + ' |')

    row('OFFBOARD mantido', lambda e: e['mission']['offboard_held_during_flight'])
    row('failsafe', lambda e: e['mission']['failsafe_state_seen'])
    row('dx comandado [m]', lambda e: e['mission']['dx_commanded_m'])
    row('dx realizado [m]', lambda e: e['mission']['dx_realized_m'])
    row('erro de retorno [m]', lambda e: e['mission']['return_error_m'])
    row('RMS XY [m]', lambda e: e['mission']['rms_xy_tracking_m'])
    row('RMS Z hover estacionario [m]', lambda e: e['mission']['rms_z_hover_steady_m'])
    row('roll max [deg]', lambda e: e['mission']['roll_max_abs_deg'])
    row('pitch max [deg]', lambda e: e['mission']['pitch_max_abs_deg'])
    row('constraint |e| RMS [m]', lambda e: e['constraint']['error_rms_m'])
    row('constraint |e| max [m]', lambda e: e['constraint']['error_max_m'])
    row('|F| RMS [N]', lambda e: e['constraint']['force_rms_n'])
    row('|F| max [N]', lambda e: e['constraint']['force_max_n'])
    row('saturation_fraction', lambda e: e['constraint']['saturation_fraction'])
    row('K estimado [N/m]', lambda e: e['constraint']['k_estimated_n_per_m'], 4)
    row('residuo da lei [%]', lambda e: e['constraint']['law_residual_pct'], 2)
    row('|r x F| RMS [N.m]', lambda e: e['constraint']['tau_rms_nm'], 5)
    row('RTF medio', lambda e: e['constraint']['rtf_mean'])
    row('RTF p05', lambda e: e['constraint']['rtf_p05'])
    row('reel theta range [rad]', lambda e: e['reel']['theta_range_rad'])
    row('reel |omega| max [rad/s]', lambda e: e['reel']['omega_abs_max_rad_s'])
    row('reel omega RMS [rad/s]', lambda e: e['reel']['omega_rms_rad_s'], 4)
    row('reel estado disponivel', lambda e: e['reel']['available_fraction'])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs='+', metavar='ROTULO=DIRETORIO')
    parser.add_argument('--out', help='prefixo de saida para .md e .json')
    args = parser.parse_args()

    columns, report = [], {}
    for item in args.runs:
        if '=' not in item:
            raise SystemExit(f'esperado ROTULO=DIRETORIO, recebido: {item!r}')
        label, run_dir = item.split('=', 1)
        columns.append(label)
        report[label] = collect(run_dir)

    table = build_table(report, columns)
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.with_suffix('.md').write_text(table + '\n')
        out.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(table)


if __name__ == '__main__':
    main()
