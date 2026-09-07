#!/usr/bin/env python3
"""Compara H0, H1 e H2 a partir dos CSV/JSON ja gravados em results/a0_2/.

Nao roda simulacao: apenas extrai metricas comparaveis dos artefatos existentes.
Grandezas sem dado gravado sao reportadas como 'N/D', nunca como zero.
"""
import argparse
import csv
import json
import math
from pathlib import Path


ND = 'N/D'
TRACKING_PHASES = ('translate_out', 'return_home')


def read_rows(path):
    with Path(path).open() as handle:
        return list(csv.DictReader(handle))


def num(row, field):
    value = row.get(field)
    if value in (None, ''):
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def rms(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean_tail(rows, phase, field, count=40):
    values = [num(r, field) for r in rows if r['phase'] == phase]
    values = [v for v in values[-count:] if v is not None]
    return sum(values) / len(values) if values else None


def px4_main_mode(custom_mode):
    if custom_mode is None:
        return None
    return (int(custom_mode) >> 16) & 0xFF


def mission_metrics(run_dir):
    csv_path = run_dir / 'px4_offboard_horizontal_mission.csv'
    json_path = run_dir / 'px4_offboard_horizontal_mission.json'
    if not csv_path.exists():
        return None
    rows = read_rows(csv_path)
    summary = json.loads(json_path.read_text()) if json_path.exists() else {}

    tracking = [r for r in rows if r['phase'] in TRACKING_PHASES]
    hover = [r for r in rows if r['phase'] == 'climb_hover']
    offboard_rows = [r for r in rows if r['phase'] in ('climb_hover',) + TRACKING_PHASES]
    offboard_ok = bool(offboard_rows) and all(
        px4_main_mode(num(r, 'custom_mode')) == 6 for r in offboard_rows
    )
    statuses = {int(v) for v in (num(r, 'system_status') for r in rows) if v is not None}

    # A fase climb_hover inclui a subida do solo ate 2 m: o RMS de ez sobre ela e
    # dominado pelo transiente. A janela final isola o hover estacionario.
    hover_tail = hover[-100:]  # ~5 s a 20 Hz
    x_hover = mean_tail(rows, 'climb_hover', 'x')
    x_out = mean_tail(rows, 'translate_out', 'x')
    x_back = mean_tail(rows, 'return_home', 'x')

    return {
        'samples': len(rows),
        'offboard_held_during_flight': offboard_ok,
        'system_status_values': sorted(statuses),
        'failsafe_state_seen': any(s >= 5 for s in statuses),
        'mission_failed_flag': summary.get('failed'),
        'dx_commanded_m': summary.get('dx_m'),
        'dx_realized_m': (x_out - x_hover) if (x_out is not None and x_hover is not None) else None,
        'return_error_m': (x_back - x_hover) if (x_back is not None and x_hover is not None) else None,
        'rms_xy_tracking_m': rms([num(r, 'e_xy') for r in tracking]),
        'rms_z_hover_full_phase_m': rms([num(r, 'ez') for r in hover]),
        'rms_z_hover_steady_m': rms([num(r, 'ez') for r in hover_tail]),
        'roll_max_abs_deg': max((abs(v) for v in (num(r, 'roll_deg') for r in rows) if v is not None), default=None),
        'pitch_max_abs_deg': max((abs(v) for v in (num(r, 'pitch_deg') for r in rows) if v is not None), default=None),
    }


def tether_metrics(run_dir):
    path = run_dir / 'h2_characterization.json'
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    constraint = data['constraint_uav']
    return {
        'force_rms_n': constraint['force_norm']['rms'],
        'force_max_n': constraint['force_norm']['max'],
        'error_rms_m': constraint['error_norm']['rms'],
        'error_max_m': constraint['error_norm']['max'],
        'saturation_fraction': constraint['saturation_fraction'],
        'rtf_mean': data['clock']['rtf_mean'],
        'rtf_min': data['clock']['rtf_min'],
        'rtf_p05': data['clock']['rtf_p05'],
    }


def fmt(value, digits=3):
    if value is None:
        return ND
    if isinstance(value, bool):
        return 'sim' if value else 'nao'
    if isinstance(value, float):
        return f'{value:.{digits}f}'
    return str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', default='results/a0_2')
    args = parser.parse_args()

    base = Path(args.results_dir)
    runs = {
        'H0 X500 puro': base / 'h0_x500_puro',
        'H1 tether livre': base / 'h1_tether_livre',
        'H2 tether ancorado': base / 'h2_tether_ancorado',
    }

    report = {}
    for label, run_dir in runs.items():
        report[label] = {
            'run_dir': str(run_dir),
            'mission': mission_metrics(run_dir),
            'tether': tether_metrics(run_dir),
        }

    columns = list(runs)
    lines = ['| Metrica | ' + ' | '.join(columns) + ' |',
             '| --- | ' + ' | '.join(['---'] * len(columns)) + ' |']

    def row(name, getter, digits=3):
        cells = []
        for label in columns:
            entry = report[label]
            try:
                cells.append(fmt(getter(entry), digits))
            except (TypeError, KeyError):
                cells.append(ND)
        lines.append(f'| {name} | ' + ' | '.join(cells) + ' |')

    row('OFFBOARD mantido', lambda e: e['mission']['offboard_held_during_flight'])
    row('failsafe (system_status>=CRITICAL)', lambda e: e['mission']['failsafe_state_seen'])
    row('dx comandado [m]', lambda e: e['mission']['dx_commanded_m'])
    row('dx realizado [m]', lambda e: e['mission']['dx_realized_m'])
    row('erro de retorno [m]', lambda e: e['mission']['return_error_m'])
    row('RMS XY [m]', lambda e: e['mission']['rms_xy_tracking_m'])
    row('RMS Z fase climb_hover [m]', lambda e: e['mission']['rms_z_hover_full_phase_m'])
    row('RMS Z hover estacionario [m]', lambda e: e['mission']['rms_z_hover_steady_m'])
    row('roll max [deg]', lambda e: e['mission']['roll_max_abs_deg'])
    row('pitch max [deg]', lambda e: e['mission']['pitch_max_abs_deg'])
    row('|F_uav| RMS [N]', lambda e: e['tether']['force_rms_n'])
    row('|F_uav| max [N]', lambda e: e['tether']['force_max_n'])
    row('|e| RMS [m]', lambda e: e['tether']['error_rms_m'])
    row('saturation_fraction', lambda e: e['tether']['saturation_fraction'])
    row('RTF medio', lambda e: e['tether']['rtf_mean'])
    row('RTF p05', lambda e: e['tether']['rtf_p05'])

    table = '\n'.join(lines)
    (base / 'comparacao_h0_h1_h2.md').write_text(table + '\n')
    (base / 'comparacao_h0_h1_h2.json').write_text(json.dumps(report, indent=2))
    print(table)


if __name__ == '__main__':
    main()
