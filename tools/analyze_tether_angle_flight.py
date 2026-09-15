#!/usr/bin/env python3
"""Metricas de uma etapa (estatica ou de voo) da campanha de forca + angulos + colisao.

Entradas no --run-dir (tools/run_tether_angle_collision_campaign.sh):
  plugin_*.csv                     topicos do TetherForceConstraint (t_wall, t_sim, x, y, z)
  connection_w0.15_summary.json/.csv  poses: penetracao no solo, tangente Python, cobertura, RTF
  px4_offboard_horizontal_mission.{json,csv}  (so voo)
Grava metrics.json com os valores e o veredito da etapa.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def rows(path):
    if not path.exists():
        return []
    with path.open() as handle:
        return list(csv.DictReader(handle))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def stats(values):
    v = sorted(x for x in values if math.isfinite(x))
    if not v:
        return None
    pick = lambda q: v[min(len(v) - 1, int(q * (len(v) - 1)))]  # noqa: E731
    return {'min': v[0], 'p05': pick(0.05), 'median': statistics.median(v), 'p95': pick(0.95),
            'max': v[-1], 'n': len(v)}


def vec_series(run, name):
    return [(num(r['t_wall']), num(r['t_sim']), num(r['x']), num(r['y']), num(r['z']))
            for r in rows(run / f'plugin_{name}.csv')]


def max_gap(series):
    t = sorted(s[1] for s in series if math.isfinite(s[1]))
    return max((b - a for a, b in zip(t, t[1:])), default=math.inf)


def second_half(r):
    return r[len(r) // 2:]


def mean_xy(r):
    xs = [num(x['x']) for x in r if math.isfinite(num(x['x']))]
    ys = [num(x['y']) for x in r if math.isfinite(num(x['y']))]
    return (statistics.mean(xs), statistics.mean(ys)) if xs and ys else (math.nan, math.nan)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--px4-log', required=True, nargs='+',
                        help='log(s) do PX4 e, com servidor separado, do Gazebo')
    parser.add_argument('--static', action='store_true')
    parser.add_argument('--penetration-tol', type=float, default=0.002,
                        help='penetracao maxima aceita da superficie de colisao no solo [m]')
    parser.add_argument('--gap-tol', type=float, default=0.25, help='maior buraco aceito no t_sim [s]')
    parser.add_argument('--rtf-min', type=float, default=0.8)
    args = parser.parse_args()
    run = Path(args.run_dir)

    pose = json.loads((run / 'connection_w0.15_summary.json').read_text())
    series = {n: vec_series(run, n) for n in ('force', 'force_body', 'stats', 't_hat_world', 't_hat_body',
                                              'angles_body', 'angles_world', 'drone_rpy')}
    nan_plugin = sum(1 for n in ('force', 't_hat_world', 't_hat_body', 'angles_body', 'angles_world')
                     for s in series[n] for c in s[2:4] if not math.isfinite(c))
    gaps = {n: max_gap(series[n]) for n in ('force', 't_hat_world', 't_hat_body', 'angles_body')}
    force_norm = [math.sqrt(s[2] ** 2 + s[3] ** 2 + s[4] ** 2) for s in series['force']]
    saturated = sum(1 for s in series['stats'] if s[4] > 0.5)
    log = ''.join(Path(p).read_text(errors='replace') for p in args.px4_log if Path(p).exists())
    aborts = sum(log.count(t) for t in ('Assertion', 'Aborted', 'assertion "'))
    failsafe = log.lower().count('failsafe')
    rtf = (pose.get('rtf') or {}).get('mean') or 0.0
    penetration = pose.get('penetration_settled_max_m' if args.static else 'penetration_max_m', math.inf)

    result = {
        'etapa': run.name, 'estatica': args.static,
        'forca_conexao_N': stats(force_norm),
        'forca_body_z_N': stats([s[4] for s in series['force_body']]),
        'saturacao_amostras': saturated,
        'elevacao_body_deg': stats([s[3] for s in series['angles_body']]),
        'azimute_body_deg': stats([s[2] for s in series['angles_body']]),
        'elevacao_world_deg': stats([s[3] for s in series['angles_world']]),
        'azimute_world_deg': stats([s[2] for s in series['angles_world']]),
        'desalinhamento_forca_tangente_deg': stats([s[4] for s in series['angles_body']]),
        't_hat_world_mediano': [statistics.median([s[k] for s in series['t_hat_world']]) for k in (2, 3, 4)]
        if series['t_hat_world'] else None,
        't_hat_body_mediano': [statistics.median([s[k] for s in series['t_hat_body']]) for k in (2, 3, 4)]
        if series['t_hat_body'] else None,
        'amostras_plugin': {n: len(s) for n, s in series.items()},
        'maior_buraco_t_sim_s': gaps, 'nan_plugin': nan_plugin, 'nan_poses': pose.get('nan_values'),
        'penetracao_max_m': penetration,
        'superficie_z_min_m': pose.get('surface_z_min_settled_m' if args.static else 'surface_z_min_m'),
        'raio_colisao_m': pose.get('collision_radius_m'),
        'rtf_medio': rtf, 'tempo_simulado_coberto_s': pose.get('sim_time_covered_s'),
        'drone_z_inicial_m': pose.get('drone_z_initial_m'), 'attach_z_inicial_m': pose.get('attach_z_initial_m'),
        'drone_z_max_m': pose.get('drone_z_max_m'),
        'abortos': aborts, 'linhas_failsafe': failsafe,
    }
    checks = {
        'sem_aborto': aborts == 0,
        'sem_failsafe': failsafe == 0,
        'sem_nan': nan_plugin == 0 and (pose.get('nan_values') or 0) == 0,
        'sem_buracos': all(g <= args.gap_tol for g in gaps.values()),
        'cobertura': (pose.get('sim_time_covered_s') or 0.0) >= 0.85 * pose.get('duration_wall_s', 0) * rtf,
        'rtf_aceitavel': rtf >= args.rtf_min,
        'colisao_solo': penetration <= args.penetration_tol,
        'forca_publicada': bool(force_norm),
    }

    if not args.static:
        mission = json.loads((run / 'px4_offboard_horizontal_mission.json').read_text())
        mrows = rows(run / 'px4_offboard_horizontal_mission.csv')
        phases = {}
        for r in mrows:
            phases.setdefault(r['phase'], []).append(r)
        hover = mean_xy(second_half(phases.get('climb_hover', [])))
        out_xy = mean_xy(second_half(phases.get('translate_out', [])))
        back_xy = mean_xy(second_half(phases.get('return_home', [])))
        hold = second_half(phases.get('climb_hover', [])) + phases.get('translate_out', []) + phases.get('return_home', [])
        ez = [num(r['ez']) for r in hold if math.isfinite(num(r['ez']))]
        per_phase = {}
        for name, prow in phases.items():
            t0, t1 = num(prow[0]['t_epoch']), num(prow[-1]['t_epoch'])
            sel = [s for s in series['angles_body'] if t0 <= s[0] <= t1]
            fsel = [math.sqrt(s[2] ** 2 + s[3] ** 2 + s[4] ** 2) for s in series['force'] if t0 <= s[0] <= t1]
            per_phase[name] = {'elevacao_body_mediana_deg': statistics.median([s[3] for s in sel]) if sel else None,
                               'azimute_body_mediano_deg': statistics.median([s[2] for s in sel]) if sel else None,
                               'forca_mediana_N': statistics.median(fsel) if fsel else None,
                               'amostras': len(sel)}
        result.update({
            'dx_comandado_m': mission.get('dx_m'),
            'dx_realizado_m': out_xy[0] - hover[0], 'dy_realizado_m': out_xy[1] - hover[1],
            'erro_retorno_m': math.hypot(back_xy[0] - hover[0], back_xy[1] - hover[1]),
            'rms_xy_m': mission.get('rms_xy_tracking_m'),
            'rms_z_hover_m': math.sqrt(sum(e * e for e in ez) / len(ez)) if ez else None,
            'rms_z_missao_m': mission.get('rms_z_hover_m'),
            'roll_max_deg': mission.get('roll_max_abs_deg'), 'pitch_max_deg': mission.get('pitch_max_abs_deg'),
            'missao_falhou': mission.get('failed'), 'por_fase': per_phase,
        })
        checks['missao_ok'] = not mission.get('failed')
        checks['atitude_ok'] = max(mission.get('roll_max_abs_deg') or 99, mission.get('pitch_max_abs_deg') or 99) < 20.0
        checks['decolou'] = (pose.get('drone_z_max_m') or 0.0) > (pose.get('drone_z_initial_m') or 0.0) + 0.5
        if (mission.get('dx_m') or 0.0) > 0.0:
            checks['dx_realizado'] = abs(result['dx_realizado_m'] - mission['dx_m']) < 0.15 * max(1.0, mission['dx_m'])
            checks['retorno'] = result['erro_retorno_m'] < 0.15

    result['checks'] = checks
    result['pass'] = all(checks.values())
    (run / 'metrics.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
