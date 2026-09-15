#!/usr/bin/env python3
"""T2 — ultimo elo x tangente suavizada, a partir das poses de uma etapa.

Le connection_w<janela>.csv (tools/record_tether_connection.py, uma gravacao por janela, mesmas poses)
e compara, por janela:
  - angulo entre t_hat_world suavizada e a direcao do ultimo elo (-R_N x);
  - jitter: RMS da variacao angular amostra a amostra [graus], e por segundo simulado;
  - elevacao no corpo: mediana, desvio padrao e amplitude p05-p95.
A direcao do ultimo elo vem da mesma gravacao (colunas last_world_*), entao a comparacao e na
mesma amostra.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def angle_deg(a, b):
    na = math.sqrt(sum(c * c for c in a))
    nb = math.sqrt(sum(c * c for c in b))
    if not (na > 0 and nb > 0):
        return math.nan
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def pct(values, q):
    v = sorted(x for x in values if math.isfinite(x))
    return v[min(len(v) - 1, int(q * (len(v) - 1)))] if v else None


def elevation(v):
    return math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))


def series_stats(t, vectors, body_elev):
    steps = [angle_deg(a, b) for a, b in zip(vectors, vectors[1:])]
    steps = [s for s in steps if math.isfinite(s)]
    dt = [b - a for a, b in zip(t, t[1:]) if b > a]
    duration = (t[-1] - t[0]) if len(t) > 1 else 0.0
    rms = math.sqrt(sum(s * s for s in steps) / len(steps)) if steps else None
    return {
        'jitter_rms_deg_por_amostra': rms,
        'variacao_total_deg_por_s': (sum(steps) / duration) if duration > 0 else None,
        'dt_mediano_s': statistics.median(dt) if dt else None,
        'elevacao_body_mediana_deg': statistics.median(body_elev) if body_elev else None,
        'elevacao_body_desvio_deg': statistics.pstdev(body_elev) if len(body_elev) > 1 else None,
        'elevacao_body_p05_p95_deg': (pct(body_elev, 0.05), pct(body_elev, 0.95)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--windows', default='0.15,0.5,1.0')
    args = parser.parse_args()
    run = Path(args.run_dir)

    out = {'etapa': run.name, 'janelas': {}}
    last_done = False
    for w in args.windows.split(','):
        path = run / f'connection_w{w}.csv'
        if not path.exists():
            continue
        with path.open() as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            continue
        t = [float(r['t_sim']) for r in rows]
        smooth = [(float(r['t_world_x']), float(r['t_world_y']), float(r['t_world_z'])) for r in rows]
        last = [(float(r['last_world_x']), float(r['last_world_y']), float(r['last_world_z'])) for r in rows]
        body_el = [float(r['elevation_body_deg']) for r in rows if math.isfinite(float(r['elevation_body_deg']))]
        diff = [angle_deg(a, b) for a, b in zip(smooth, last)]
        entry = series_stats(t, smooth, body_el)
        entry.update({
            'amostras': len(rows),
            'janela_usada_m': statistics.median(float(r['window_m']) for r in rows),
            'suavizada_x_ultimo_elo_deg': {'mediana': pct(diff, 0.5), 'p95': pct(diff, 0.95),
                                           'max': pct(diff, 1.0)},
        })
        out['janelas'][w] = entry
        if not last_done:
            last_el = [float(r['elevation_deg']) for r in rows if math.isfinite(float(r['elevation_deg']))]
            out['ultimo_elo'] = series_stats(t, last, last_el)
            out['ultimo_elo']['amostras'] = len(rows)
            last_done = True

    (run / 't2_windows.json').write_text(json.dumps(out, indent=2))
    print(f"{'metodo':>14s} {'jitter rms':>11s} {'deg/s':>8s} {'el med':>8s} {'el std':>7s} {'x ultimo elo med/p95':>22s}")
    if 'ultimo_elo' in out:
        u = out['ultimo_elo']
        print(f"{'ultimo elo':>14s} {u['jitter_rms_deg_por_amostra']:11.4f} {u['variacao_total_deg_por_s']:8.2f} "
              f"{u['elevacao_body_mediana_deg']:8.2f} {u['elevacao_body_desvio_deg']:7.3f} {'-':>22s}")
    for w, e in out['janelas'].items():
        d = e['suavizada_x_ultimo_elo_deg']
        print(f"{'w=' + w + ' m':>14s} {e['jitter_rms_deg_por_amostra']:11.4f} {e['variacao_total_deg_por_s']:8.2f} "
              f"{e['elevacao_body_mediana_deg']:8.2f} {e['elevacao_body_desvio_deg']:7.3f} "
              f"{d['mediana']:10.3f}/{d['p95']:<10.3f}")


if __name__ == '__main__':
    main()
