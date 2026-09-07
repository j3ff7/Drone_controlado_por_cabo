#!/usr/bin/env python3
"""Compara a baseline A0.2 (base_link + offset) com A0.3 (tether_attach_link fisico).

Reutiliza os extratores de `compare_a0_2_runs.py`; nao roda simulacao. Inclui a
corrida de controle feita na mesma sessao de A0.3 com o endpoint antigo, que separa
o efeito da mudanca de endpoint da variacao entre sessoes.
"""
import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    'compare_a0_2_runs', ROOT / 'tools' / 'compare_a0_2_runs.py')
_base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_base)

ND = _base.ND
fmt = _base.fmt
mission_metrics = _base.mission_metrics


def constraint_metrics(run_dir, analysis_name):
    path = run_dir / analysis_name
    if not path.exists():
        return None
    data = json.loads(path.read_text())

    if 'constraint_uav' in data:
        # Formato produzido por analyze_a0_2_h2.py.
        constraint = data['constraint_uav']
        law = data['law_check']
        return {
            'error_rms_m': constraint['error_norm']['rms'],
            'error_max_m': constraint['error_norm']['max'],
            'force_rms_n': constraint['force_norm']['rms'],
            'force_max_n': constraint['force_norm']['max'],
            'saturation_fraction': constraint['saturation_fraction'],
            'k_estimated_n_per_m': law['k_estimated_from_norms_n_per_m'],
            'law_residual_n': law['residual_full_model_settled_n']['rms'],
            'law_residual_pct': (
                100.0 * law['residual_full_model_settled_n']['rms']
                / constraint['force_norm']['rms']),
            'tau_rms_nm': None,          # A0.2 nao computou r x F
            'rtf_mean': data['clock']['rtf_mean'],
            'rtf_p05': data['clock']['rtf_p05'],
        }

    constraint = data['constraint']
    return {
        'error_rms_m': constraint['error_norm']['rms'],
        'error_max_m': constraint['error_norm']['max'],
        'force_rms_n': constraint['force_norm']['rms'],
        'force_max_n': constraint['force_norm']['max'],
        'saturation_fraction': constraint['saturation_fraction'],
        'k_estimated_n_per_m': constraint['k_estimated_n_per_m'],
        'law_residual_n': constraint['law_residual_settled_n']['rms'],
        'law_residual_pct': constraint['law_residual_relative_pct'],
        'tau_rms_nm': data['torque_r_cross_f']['tau_norm_nm']['rms'],
        'rtf_mean': data['rtf']['mean'],
        'rtf_p05': data['rtf']['p05'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', default='results')
    args = parser.parse_args()

    base = Path(args.results_dir)
    runs = {
        'A0.2 offset virtual': (base / 'a0_2' / 'h2_tether_ancorado', 'h2_characterization.json'),
        'A0.3 link fisico': (base / 'a0_3' / 'horizontal', 'horizontal_analysis.json'),
        'controle offset (sessao A0.3)': (
            base / 'a0_3' / 'controle_base_link_offset', 'controle_analysis.json'),
    }

    report = {}
    for label, (run_dir, analysis) in runs.items():
        report[label] = {
            'run_dir': str(run_dir),
            'mission': mission_metrics(run_dir),
            'constraint': constraint_metrics(run_dir, analysis),
        }

    columns = list(runs)
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
    row('residuo da lei [N]', lambda e: e['constraint']['law_residual_n'], 4)
    row('residuo da lei [%]', lambda e: e['constraint']['law_residual_pct'], 2)
    row('|r x F| RMS [N.m]', lambda e: e['constraint']['tau_rms_nm'], 5)
    row('RTF medio', lambda e: e['constraint']['rtf_mean'])
    row('RTF p05', lambda e: e['constraint']['rtf_p05'])

    table = '\n'.join(lines)
    out = base / 'a0_3' / 'comparacao_a0_2_a0_3.md'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(table + '\n')
    (base / 'a0_3' / 'comparacao_a0_2_a0_3.json').write_text(json.dumps(report, indent=2))
    print(table)


if __name__ == '__main__':
    main()
