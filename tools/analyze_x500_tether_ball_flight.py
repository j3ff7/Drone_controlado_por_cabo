#!/usr/bin/env python3
"""Metricas de voo do X500 + tether por BallJoint a partir da missao OFFBOARD e das poses.

Da missao (tools/px4_offboard_horizontal_mission.py): RMS XY/Z, roll/pitch, dx comandado, dx
realizado (media da 2a metade de translate_out menos a media da 2a metade de climb_hover) e erro de
retorno (media da 2a metade de return_home contra o mesmo ponto de hover). Das poses
(tools/record_x500_tether_ball.py): azimute/elevacao do primeiro segmento por fase, alinhados pelo
relogio de parede. A missao nao percebe se o simulador morreu; por isso o veredito exige tambem
cobertura de tempo simulado e ausencia de aborto.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def second_half(rows):
    return rows[len(rows) // 2:] if rows else rows


def mean_xy(rows):
    xs = [num(r['x']) for r in rows if math.isfinite(num(r['x']))]
    ys = [num(r['y']) for r in rows if math.isfinite(num(r['y']))]
    return (statistics.mean(xs), statistics.mean(ys)) if xs and ys else (math.nan, math.nan)


def med(values):
    v = [x for x in values if math.isfinite(x)]
    return statistics.median(v) if v else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--px4-log', required=True)
    parser.add_argument('--expected-wall-s', type=float, default=72.0)
    args = parser.parse_args()
    d = Path(args.run_dir)

    mission = json.loads((d / 'px4_offboard_horizontal_mission.json').read_text())
    rows = list(csv.DictReader((d / 'px4_offboard_horizontal_mission.csv').open()))
    phases = {}
    for r in rows:
        phases.setdefault(r['phase'], []).append(r)
    hover = mean_xy(second_half(phases.get('climb_hover', [])))
    out_xy = mean_xy(second_half(phases.get('translate_out', [])))
    back_xy = mean_xy(second_half(phases.get('return_home', [])))
    dx_real = out_xy[0] - hover[0]
    dy_real = out_xy[1] - hover[1]
    return_err = math.hypot(back_xy[0] - hover[0], back_xy[1] - hover[1])
    # o rms_z_hover_m da missao inclui a subida (a fase climb_hover comeca no solo); este so conta
    # trechos em que o drone ja deveria estar parado na altitude
    hold = second_half(phases.get('climb_hover', [])) + phases.get('translate_out', []) + phases.get('return_home', [])
    ez = [num(r['ez']) for r in hold if math.isfinite(num(r['ez']))]
    rms_z_hold = math.sqrt(sum(e * e for e in ez) / len(ez)) if ez else None

    geom = json.loads((d / 'tether_ball_summary.json').read_text())
    grows = list(csv.DictReader((d / 'tether_ball.csv').open())) if (d / 'tether_ball.csv').exists() else []
    per_phase = {}
    for name, prow in phases.items():
        t0, t1 = num(prow[0]['t_epoch']), num(prow[-1]['t_epoch'])
        sel = [g for g in grows if t0 <= num(g['t_wall']) <= t1]
        per_phase[name] = {'azimuth_deg_median': med([num(g['azimuth_deg']) for g in sel]),
                           'elevation_deg_median': med([num(g['elevation_deg']) for g in sel]),
                           'dist_root_base_m_max': max((num(g['dist_root_base_m']) for g in sel), default=None),
                           'samples': len(sel)}

    log = Path(args.px4_log).read_text(errors='replace')
    aborts = sum(log.count(t) for t in ('Assertion', 'Aborted'))
    failsafe = log.lower().count('failsafe')
    dist = geom.get('dist_root_base_m') or {}
    covered = geom.get('sim_time_covered_s', 0.0)
    rtf_mean = (geom.get('rtf') or {}).get('mean') or 0.0
    # simulador vivo durante toda a gravacao: tempo simulado ~ tempo de parede x RTF
    expected_sim = 0.85 * (geom.get('duration_wall_s') or args.expected_wall_s) * rtf_mean
    checks = {
        'sim_coberto_ok': rtf_mean > 0 and covered >= expected_sim,
        'sem_aborto': aborts == 0,
        'sem_failsafe': failsafe == 0,
        'missao_ok': not mission.get('failed'),
        'atitude_ok': max(mission.get('roll_max_abs_deg') or 0, mission.get('pitch_max_abs_deg') or 0) < 30.0,
        'ball_joint_integra': bool(dist) and abs(dist['max'] - 0.2) < 0.01 and abs(dist['min'] - 0.2) < 0.01,
        # colisao do cabo: raio 1,5 mm. Cabo arrastado atravessa o solo e, ao ser arrancado depois,
        # chicoteia e derruba o DART (rodada shared, horizontal da campanha baseline)
        'cabo_sem_penetrar_solo': (geom.get('z_min_cable_m') or {}).get('min', -1.0) >= -0.005,
    }
    result = {
        'pass': all(checks.values()), 'checks': checks,
        'dx_comandado_m': mission.get('dx_m'), 'dy_comandado_m': mission.get('dy_m'),
        'dx_realizado_local_m': dx_real, 'dy_realizado_local_m': dy_real, 'erro_retorno_m': return_err,
        'rms_xy_m': mission.get('rms_xy_tracking_m'), 'rms_z_hover_m': mission.get('rms_z_hover_m'),
        'rms_z_hold_m': rms_z_hold,
        'roll_max_deg': mission.get('roll_max_abs_deg'), 'pitch_max_deg': mission.get('pitch_max_abs_deg'),
        'rtf_mean': (geom.get('rtf') or {}).get('mean'), 'sim_time_covered_s': covered,
        'aborts': aborts, 'failsafe_lines': failsafe, 'dist_root_base_m': dist,
        'elevation_deg': geom.get('elevation_deg'), 'azimuth_deg': geom.get('azimuth_deg'),
        'drone_z_max_m': geom.get('drone_z_max_m'), 'per_phase': per_phase,
    }
    (d / 'flight_metrics.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
