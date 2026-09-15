#!/usr/bin/env python3
"""T0/T1 — analise da bancada de compensacao de atitude (tools/run_tether_attitude_bench.sh).

Para cada caso (corpo estatico numa atitude fixa, mesmo ponto de conexao):
  - t_hat_world medio (plugin) e o espalhamento no tempo;
  - invariancia: angulo entre t_hat_world do caso e do caso nivelado (a geometria do cabo nao muda);
  - transformacao: angulo entre t_hat_body medio e R_BW(q_caso) * t_hat_world medio;
  - previsao: azimute/elevacao no corpo previstos a partir do t_hat_world NIVELADO e da atitude;
  - verificacao independente: t_hat_world do plugin x calculo Python pelas poses;
  - ultimo elo x tangente suavizada.
Um Vector3d exatamente zero chega vazio ao gravador de texto; sem amostras de drone_rpy o caso
usa o roll/pitch/yaw das poses (tools/record_tether_connection.py).
"""
import argparse
import csv
import json
import math
from pathlib import Path


def read_vectors(path):
    if not path.exists():
        return []
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    out = []
    for r in rows:
        try:
            out.append((float(r['x']), float(r['y']), float(r['z'])))
        except (KeyError, ValueError):
            continue
    return out


def norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 0 else (math.nan,) * 3


def mean_dir(vectors):
    return norm(tuple(sum(v[k] for v in vectors) / len(vectors) for k in range(3)))


def angle_deg(a, b):
    c = sum(x * y for x, y in zip(norm(a), norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def rot_rpy(roll, pitch, yaw):
    """R_WB = Rz(yaw) Ry(pitch) Rx(roll), graus."""
    r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def world_to_body(rpy, v):
    m = rot_rpy(*rpy)
    return tuple(sum(m[i][k] * v[i] for i in range(3)) for k in range(3))   # R^T v


def az_el(t):
    return (math.degrees(math.atan2(t[1], t[0])),
            math.degrees(math.atan2(t[2], math.hypot(t[0], t[1]))))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def pose_rows(path):
    if not path.exists():
        return []
    with path.open() as handle:
        return list(csv.DictReader(handle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bench-dir', required=True)
    parser.add_argument('--reference', default='level')
    parser.add_argument('--tol-invariance-deg', type=float, default=1.0)
    parser.add_argument('--tol-transform-deg', type=float, default=0.05)
    parser.add_argument('--tol-prediction-deg', type=float, default=1.0)
    parser.add_argument('--tol-crosscheck-deg', type=float, default=1.0)
    parser.add_argument('--max-abs-elevation-for-azimuth', type=float, default=85.0,
                        help='acima deste |elevacao| o azimute nao e verificado (singular na vertical)')
    args = parser.parse_args()
    bench = Path(args.bench_dir)

    cases = {}
    for d in sorted(p for p in bench.iterdir() if (p / 'case.txt').exists()):
        spec = dict(kv.split('=') for kv in (d / 'case.txt').read_text().split()[1:])
        commanded = tuple(float(spec[k]) for k in ('roll', 'pitch', 'yaw'))
        t_world = read_vectors(d / 'plugin_t_hat_world.csv')
        t_body = read_vectors(d / 'plugin_t_hat_body.csv')
        last = read_vectors(d / 'plugin_t_hat_world_last_link.csv')
        rpy_plugin = read_vectors(d / 'plugin_drone_rpy.csv')
        poses = pose_rows(d / 'connection.csv')
        py_world = [(float(r['t_world_x']), float(r['t_world_y']), float(r['t_world_z'])) for r in poses]
        py_rpy = [(float(r['roll_deg']), float(r['pitch_deg']), float(r['yaw_deg'])) for r in poses]
        aborts = int((d / 'aborts.txt').read_text().strip() or 0) if (d / 'aborts.txt').exists() else -1
        nan = sum(1 for v in t_world + t_body for c in v if not math.isfinite(c))
        cases[d.name] = dict(commanded=commanded, t_world=t_world, t_body=t_body, last=last,
                             rpy_plugin=rpy_plugin, py_world=py_world, py_rpy=py_rpy,
                             aborts=aborts, nan=nan)

    if args.reference not in cases or not cases[args.reference]['t_world']:
        raise SystemExit(f'caso de referencia {args.reference} sem dados')
    ref_world = mean_dir(cases[args.reference]['t_world'])

    report, all_pass = {}, True
    for name, c in cases.items():
        if not c['t_world'] or not c['t_body']:
            report[name] = {'pass': False, 'erro': 'sem amostras'}
            all_pass = False
            continue
        tw, tb = mean_dir(c['t_world']), mean_dir(c['t_body'])
        rpy_source = 'plugin' if c['rpy_plugin'] else 'poses'
        rpy_samples = c['rpy_plugin'] or c['py_rpy']
        rpy_measured = tuple(sum(v[k] for v in rpy_samples) / len(rpy_samples) for k in range(3))
        predicted_body = world_to_body(c['commanded'], ref_world)
        az_pred, el_pred = az_el(predicted_body)
        az_meas, el_meas = az_el(tb)
        res = {
            'atitude_comandada_deg': c['commanded'],
            'atitude_medida_deg': rpy_measured, 'atitude_fonte': rpy_source,
            'erro_atitude_deg': max(abs(wrap(a - b)) for a, b in zip(rpy_measured, c['commanded'])),
            'amostras': {'t_hat_world': len(c['t_world']), 't_hat_body': len(c['t_body']),
                         'drone_rpy': len(c['rpy_plugin']), 'poses': len(c['py_world'])},
            't_hat_world': tw, 't_hat_body': tb,
            'angulos_world_deg': az_el(tw), 'angulos_body_deg': (az_meas, el_meas),
            'espalhamento_t_hat_world_p95_deg': sorted(angle_deg(v, tw) for v in c['t_world'])[
                int(0.95 * (len(c['t_world']) - 1))],
            'invariancia_world_deg': angle_deg(tw, ref_world),
            'residuo_transformacao_deg': angle_deg(tb, world_to_body(c['commanded'], tw)),
            'angulos_body_previstos_deg': (az_pred, el_pred),
            'erro_previsao_azimute_deg': abs(wrap(az_meas - az_pred)),
            'erro_previsao_elevacao_deg': abs(el_meas - el_pred),
            # vetor: sempre definido; o azimute nao e definido com o cabo na vertical do corpo
            'erro_previsao_vetor_deg': angle_deg(tb, predicted_body),
            'azimute_bem_condicionado': abs(el_pred) < args.max_abs_elevation_for_azimuth,
            'plugin_x_python_deg': angle_deg(tw, mean_dir(c['py_world'])) if c['py_world'] else None,
            'ultimo_elo_x_suavizada_deg': angle_deg(mean_dir(c['last']), tw) if c['last'] else None,
            'nan': c['nan'], 'abortos': c['aborts'],
        }
        checks = {
            'sem_nan_nem_aborto': c['nan'] == 0 and c['aborts'] == 0,
            'atitude_confere': res['erro_atitude_deg'] < 0.1,
            'world_invariante': res['invariancia_world_deg'] < args.tol_invariance_deg,
            'body_transformado': res['residuo_transformacao_deg'] < args.tol_transform_deg,
            'body_previsto': res['erro_previsao_vetor_deg'] < args.tol_prediction_deg
            and res['erro_previsao_elevacao_deg'] < args.tol_prediction_deg
            and (not res['azimute_bem_condicionado']
                 or res['erro_previsao_azimute_deg'] < args.tol_prediction_deg),
            'python_confere': res['plugin_x_python_deg'] is not None
            and res['plugin_x_python_deg'] < args.tol_crosscheck_deg,
        }
        res['checks'] = checks
        res['pass'] = all(checks.values())
        all_pass &= res['pass']
        report[name] = res

    out = {'pass': all_pass, 'referencia': args.reference, 't_hat_world_referencia': ref_world,
           'casos': report}
    (bench / 't1_analysis.json').write_text(json.dumps(out, indent=2))
    print(f"{'caso':12s} {'rpy cmd':>18s} {'az/el world':>15s} {'az/el body':>15s} {'prev body':>15s} "
          f"{'inv':>6s} {'resid':>7s} {'py':>6s} {'elo':>6s} pass")
    for name, r in report.items():
        if 'erro' in r:
            print(name, r['erro'])
            continue
        fmt = lambda p: f"{p[0]:7.2f}/{p[1]:6.2f}"  # noqa: E731
        print(f"{name:12s} {str(tuple(round(v) for v in r['atitude_comandada_deg'])):>18s} "
              f"{fmt(r['angulos_world_deg']):>15s} {fmt(r['angulos_body_deg']):>15s} "
              f"{fmt(r['angulos_body_previstos_deg']):>15s} {r['invariancia_world_deg']:6.3f} "
              f"{r['residuo_transformacao_deg']:7.4f} {r['plugin_x_python_deg']:6.3f} "
              f"{(r['ultimo_elo_x_suavizada_deg'] or 0):6.3f} vet={r['erro_previsao_vetor_deg']:.4f} "
              f"{'' if r['azimute_bem_condicionado'] else '(azimute singular) '}{r['pass']}")
    print('T1 PASS' if all_pass else 'T1 FAIL')


if __name__ == '__main__':
    main()
