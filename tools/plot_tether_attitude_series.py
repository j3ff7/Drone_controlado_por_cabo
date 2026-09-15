#!/usr/bin/env python3
"""S3 — series temporais que mostram a compensacao de atitude, e o residuo da transformacao.

Le plugin_drone_rpy.csv, plugin_angles_world.csv, plugin_angles_body.csv, plugin_t_hat_world.csv e
plugin_t_hat_body.csv de uma etapa de voo e gera:
  s3_series.svg  paineis: roll/pitch/yaw; azimute world x body; elevacao world x body
  s3_series.csv  series alinhadas por t_sim (amostra mais proxima de cada topico)
  s3_metrics.json residuo |angulo(t_hat_body, R_BW(rpy) t_hat_world)| e diferencas world-body
SVG escrito a mao: o matplotlib desta maquina nao importa (NumPy 2 x modulo compilado com 1.x).
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def load(path):
    if not path.exists():
        return [], []
    with path.open() as handle:
        data = sorted((float(r['t_sim']), (float(r['x']), float(r['y']), float(r['z'])))
                      for r in csv.DictReader(handle))
    return [d[0] for d in data], [d[1] for d in data]


def nearest(times, values, t, tol):
    i = bisect.bisect_left(times, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(times) and abs(times[j] - t) <= tol:
            if best is None or abs(times[j] - t) < abs(times[best] - t):
                best = j
    return values[best] if best is not None else None


def rot_rpy(roll, pitch, yaw):
    r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def world_to_body(rpy, v):
    m = rot_rpy(*rpy)
    return tuple(sum(m[i][k] * v[i] for i in range(3)) for k in range(3))


def angle_deg(a, b):
    na = math.sqrt(sum(c * c for c in a))
    nb = math.sqrt(sum(c * c for c in b))
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def percentile(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, int(q * (len(v) - 1)))] if v else None


def svg_panel(x0, y0, w, h, t, series, title, unit):
    """series = [(rotulo, cor, valores)]"""
    vals = [v for _, _, s in series for v in s if v is not None and math.isfinite(v)]
    if not vals or not t:
        return f'<text x="{x0}" y="{y0 + 20}">{title}: sem dados</text>'
    lo, hi = min(vals), max(vals)
    if hi - lo < 1e-6:
        lo, hi = lo - 1, hi + 1
    pad = 0.05 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    t0, t1 = t[0], t[-1] if t[-1] > t[0] else t[0] + 1
    sx = lambda v: x0 + (v - t0) / (t1 - t0) * w  # noqa: E731
    sy = lambda v: y0 + h - (v - lo) / (hi - lo) * h  # noqa: E731
    out = [f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="none" stroke="#999"/>',
           f'<text x="{x0}" y="{y0 - 6}" font-size="13" font-weight="bold">{title}</text>',
           f'<text x="{x0 - 6}" y="{y0 + 10}" font-size="10" text-anchor="end">{hi:.1f}</text>',
           f'<text x="{x0 - 6}" y="{y0 + h}" font-size="10" text-anchor="end">{lo:.1f}</text>',
           f'<text x="{x0 - 34}" y="{y0 + h / 2}" font-size="10" text-anchor="end">{unit}</text>']
    if lo < 0 < hi:
        out.append(f'<line x1="{x0}" x2="{x0 + w}" y1="{sy(0):.1f}" y2="{sy(0):.1f}" stroke="#ddd"/>')
    for k, (label, color, s) in enumerate(series):
        step = max(1, len(t) // 1500)
        pts = ' '.join(f'{sx(t[i]):.1f},{sy(s[i]):.1f}' for i in range(0, len(t), step)
                       if s[i] is not None and math.isfinite(s[i]))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.2"/>')
        out.append(f'<text x="{x0 + w - 150 + 0 * k}" y="{y0 + 14 + 13 * k}" font-size="11" fill="{color}">{label}</text>')
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--tol', type=float, default=0.01, help='tolerancia de alinhamento t_sim [s]')
    parser.add_argument('--pose-csv', default='connection_w0.15.csv',
                        help='poses com a tangente calculada em Python (verificacao independente)')
    parser.add_argument('--cross-tol', type=float, default=0.02,
                        help='tolerancia de alinhamento plugin x poses [s]')
    args = parser.parse_args()
    run = Path(args.run_dir)

    tr, rpy = load(run / 'plugin_drone_rpy.csv')
    tw_t, tw = load(run / 'plugin_t_hat_world.csv')
    tb_t, tb = load(run / 'plugin_t_hat_body.csv')
    aw_t, aw = load(run / 'plugin_angles_world.csv')
    ab_t, ab = load(run / 'plugin_angles_body.csv')
    if not tr:
        raise SystemExit('sem drone_rpy')

    aligned, residual = [], []
    for t, a in zip(tr, rpy):
        w = nearest(tw_t, tw, t, args.tol)
        b = nearest(tb_t, tb, t, args.tol)
        angw = nearest(aw_t, aw, t, args.tol)
        angb = nearest(ab_t, ab, t, args.tol)
        if None in (w, b, angw, angb):
            continue
        res = angle_deg(b, world_to_body(a, w))
        residual.append(res)
        aligned.append({'t_sim': t, 'roll': a[0], 'pitch': a[1], 'yaw': a[2],
                        'azimuth_world': angw[0], 'elevation_world': angw[1],
                        'azimuth_body': angb[0], 'elevation_body': angb[1], 'residuo_deg': res})
    if not aligned:
        raise SystemExit('sem amostras alinhadas')

    with (run / 's3_series.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(aligned[0]))
        writer.writeheader()
        writer.writerows(aligned)

    wrap = lambda a: (a + 180.0) % 360.0 - 180.0  # noqa: E731
    el_diff = [abs(r['elevation_body'] - r['elevation_world']) for r in aligned]
    az_yaw = [abs(wrap(r['azimuth_body'] - (r['azimuth_world'] - r['yaw']))) for r in aligned
              if r['elevation_world'] > -80.0]
    metrics = {
        'amostras_alinhadas': len(aligned),
        'residuo_transformacao_deg': {'mediana': percentile(residual, 0.5), 'p95': percentile(residual, 0.95),
                                      'max': max(residual)},
        'roll_deg': {'min': min(r['roll'] for r in aligned), 'max': max(r['roll'] for r in aligned)},
        'pitch_deg': {'min': min(r['pitch'] for r in aligned), 'max': max(r['pitch'] for r in aligned)},
        'yaw_deg': {'min': min(r['yaw'] for r in aligned), 'max': max(r['yaw'] for r in aligned)},
        'diferenca_elevacao_body_world_deg': {'mediana': percentile(el_diff, 0.5), 'p95': percentile(el_diff, 0.95),
                                              'max': max(el_diff)},
        'azimute_body_menos_world_menos_yaw_deg': {'mediana': percentile(az_yaw, 0.5), 'p95': percentile(az_yaw, 0.95),
                                                   'n_elevacao_acima_de_-80': len(az_yaw)} if az_yaw else None,
    }
    # Verificacao independente: o residuo acima so mostra que os topicos do plugin sao coerentes
    # entre si (mesmo quaternion, mesmo passo). Aqui o t_hat do plugin e comparado com o calculo
    # Python a partir de /world/<mundo>/pose/info (tools/record_tether_connection.py), com carimbo
    # de tempo simulado proprio; o alinhamento e pela amostra mais proxima em t_sim.
    pose_csv = run / args.pose_csv
    if pose_csv.exists():
        with pose_csv.open() as handle:
            prow = sorted(csv.DictReader(handle), key=lambda r: float(r['t_sim']))
        pt = [float(r['t_sim']) for r in prow]
        pw = [(float(r['t_world_x']), float(r['t_world_y']), float(r['t_world_z'])) for r in prow]
        pb = [(float(r['t_hat_body_x']), float(r['t_hat_body_y']), float(r['t_hat_body_z'])) for r in prow]
        dw, db = [], []
        for t, w, b in zip(pt, pw, pb):
            wp = nearest(tw_t, tw, t, args.cross_tol)
            bp = nearest(tb_t, tb, t, args.cross_tol)
            if wp is not None and all(math.isfinite(c) for c in w):
                dw.append(angle_deg(w, wp))
            if bp is not None and all(math.isfinite(c) for c in b):
                db.append(angle_deg(b, bp))
        metrics['plugin_x_python_pelas_poses_deg'] = {
            'tolerancia_alinhamento_s': args.cross_tol,
            't_hat_world': {'mediana': percentile(dw, 0.5), 'p95': percentile(dw, 0.95), 'n': len(dw)},
            't_hat_body': {'mediana': percentile(db, 0.5), 'p95': percentile(db, 0.95), 'n': len(db)},
        }
    metrics['pass'] = metrics['residuo_transformacao_deg']['p95'] < 0.5
    (run / 's3_metrics.json').write_text(json.dumps(metrics, indent=2))

    t = [r['t_sim'] - aligned[0]['t_sim'] for r in aligned]
    col = lambda k: [r[k] for r in aligned]  # noqa: E731
    w, h = 900, 190
    body = [svg_panel(70, 40, w, h, t, [('roll', '#d62728', col('roll')), ('pitch', '#1f77b4', col('pitch')),
                                        ('yaw', '#2ca02c', col('yaw'))], 'Atitude do X500', 'graus'),
            svg_panel(70, 40 + (h + 50), w, h, t, [('azimute world', '#7f7f7f', col('azimuth_world')),
                                                   ('azimute body', '#9467bd', col('azimuth_body'))],
                      'Azimute do tether: world x body', 'graus'),
            svg_panel(70, 40 + 2 * (h + 50), w, h, t, [('elevacao world', '#7f7f7f', col('elevation_world')),
                                                       ('elevacao body', '#ff7f0e', col('elevation_body'))],
                      'Elevacao do tether: world x body', 'graus'),
            svg_panel(70, 40 + 3 * (h + 50), w, h, t, [('residuo transformacao', '#000000', col('residuo_deg'))],
                      'angulo(t_hat_body, R_BW t_hat_world)', 'graus'),
            f'<text x="70" y="{40 + 4 * (h + 50) - 20}" font-size="11">t simulado [s] desde {aligned[0]["t_sim"]:.1f}; '
            f'{run.name}</text>']
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w + 100}" height="{40 + 4 * (h + 50)}" '
           f'font-family="sans-serif"><rect width="100%" height="100%" fill="white"/>' + '\n'.join(body) + '</svg>')
    (run / 's3_series.svg').write_text(svg)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
