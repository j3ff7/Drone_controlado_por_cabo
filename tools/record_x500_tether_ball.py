#!/usr/bin/env python3
"""Geometria em runtime do X500 + tether por BallJoint (wrapper de tools/launch_x500_tether_ball.py).

Le `/world/<mundo>/dynamic_pose/info`, que traz as poses RELATIVAS AO PAI: links do X500 ao modelo,
links do cabo ao modelo aninhado `cabo_anexado`, que e relativo ao modelo do veiculo.
Nao usar `pose/info`: ele nao atualiza os links do modelo aninhado (o cabo pareceria congelado).

Grava, por amostra: posicao/atitude do drone, pose da raiz do cabo, distancia raiz <-> base_link
(integridade da BallJoint), azimute/elevacao do primeiro segmento no frame do drone, ponta do cabo
e z minimo do cabo; e o RTF de /world/<mundo>/stats. O resumo inclui o tempo SIMULADO coberto,
que denuncia se o servidor morreu no meio da gravacao.
"""
import argparse
import csv
import json
import math
import re
import statistics
import subprocess
import threading
import time
from pathlib import Path


NUM = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|-?inf'
IDENTITY = (0.0, 0.0, 0.0, 1.0)


def quat_rotate(q, v):
    x, y, z, w = q
    ux, uy, uz = v
    tx, ty, tz = 2 * (y * uz - z * uy), 2 * (z * ux - x * uz), 2 * (x * uy - y * ux)
    return (ux + w * tx + (y * tz - z * ty), uy + w * ty + (z * tx - x * tz), uz + w * tz + (x * ty - y * tx))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def compose(parent, child):
    (pp, pq), (cp, cq) = parent, child
    r = quat_rotate(pq, cp)
    return ((pp[0] + r[0], pp[1] + r[1], pp[2] + r[2]), quat_mul(pq, cq))


def roll_pitch_deg(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def segment_angles(drone_q, segment_q):
    """Azimute/elevacao do primeiro segmento no frame do drone (x frente, y esquerda, z cima).

    A raiz do cabo esta no drone e o segmento se estende no +x local, entao a direcao que SAI do
    drone ao longo do cabo e +R_seg (1,0,0). Azimute 0 = frente, +90 = esquerda; elevacao -90 =
    cabo pendurado reto abaixo.
    """
    t_world = quat_rotate(segment_q, (1.0, 0.0, 0.0))
    x, y, z, w = drone_q
    t = quat_rotate((-x, -y, -z, w), t_world)
    return t, math.degrees(math.atan2(t[1], t[0])), math.degrees(math.atan2(t[2], math.hypot(t[0], t[1])))


def parse_poses(text):
    poses = {}
    for block in re.split(r'\n(?=pose \{)', text):
        name = re.search(r'name: "([^"]+)"', block)
        if not name:
            continue

        def vec(tag, keys, default):
            m = re.search(tag + r' \{(.*?)\}', block, re.S)
            out = dict(default)
            if m:
                for k, v in re.findall(rf'([xyzw]): ({NUM})', m.group(1)):
                    out[k] = float(v)
            return tuple(out[k] for k in keys)

        poses.setdefault(name.group(1), (vec('position', 'xyz', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
                                         vec('orientation', 'xyzw', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0})))
    return poses


def sample_from_poses(poses, vehicle, cable, t_sim, t_wall):
    drone = poses[vehicle]
    nested = compose(drone, poses[cable])
    base = compose(drone, poses['base_link'])
    root = compose(nested, poses['raiz_cabo'])
    seg1 = compose(nested, poses['segment_1'])
    tip = compose(nested, poses['ponta_cabo'])
    zs = [compose(nested, p)[0][2] for n, p in poses.items() if n.startswith('segment_')]
    roll, pitch = roll_pitch_deg(base[1])
    t, az, el = segment_angles(base[1], seg1[1])
    return {'t_sim': t_sim, 't_wall': t_wall,
            'drone_x': base[0][0], 'drone_y': base[0][1], 'drone_z': base[0][2],
            'roll_deg': roll, 'pitch_deg': pitch,
            'root_x': root[0][0], 'root_y': root[0][1], 'root_z': root[0][2],
            'dist_root_base_m': math.dist(root[0], base[0]),
            'seg1_tx': t[0], 'seg1_ty': t[1], 'seg1_tz': t[2], 'azimuth_deg': az, 'elevation_deg': el,
            'tip_x': tip[0][0], 'tip_y': tip[0][1], 'tip_z': tip[0][2],
            'z_min_cable_m': min(zs) if zs else math.nan, 'segments': len(zs)}


class Stream(threading.Thread):
    def __init__(self, topic, on_message):
        super().__init__(daemon=True)
        self.proc = subprocess.Popen(['gz', 'topic', '-e', '-t', topic], stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True)
        self.on_message = on_message

    def run(self):
        lines = []
        for line in self.proc.stdout:
            if line.strip():
                lines.append(line)
            elif lines:
                self.on_message(''.join(lines))
                lines = []

    def stop(self):
        self.proc.terminate()


def stats(values):
    v = sorted(x for x in values if isinstance(x, float) and math.isfinite(x))
    if not v:
        return None
    pick = lambda q: v[min(len(v) - 1, int(q * (len(v) - 1)))]  # noqa: E731
    return {'min': v[0], 'p05': pick(0.05), 'median': statistics.median(v), 'p95': pick(0.95),
            'max': v[-1], 'n': len(v)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, required=True, help='segundos de parede')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--prefix', default='tether_ball')
    parser.add_argument('--world', default='default')
    parser.add_argument('--vehicle', default='x500_tether_ball_0')
    parser.add_argument('--cable', default='cabo_anexado')
    parser.add_argument('--every', type=int, default=3, help='usa 1 de cada N mensagens de pose')
    args = parser.parse_args()

    rows, rtf, counter, lock = [], [], [0], threading.Lock()

    def on_pose(text):
        counter[0] += 1
        if counter[0] % args.every:
            return
        stamp = re.search(r'header \{.*?stamp \{(.*?)\}', text, re.S)
        t_sim = 0.0
        if stamp:
            sec = re.search(r'sec: (\d+)', stamp.group(1))
            nsec = re.search(r'nsec: (\d+)', stamp.group(1))
            t_sim = (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9
        poses = parse_poses(text)
        try:
            row = sample_from_poses(poses, args.vehicle, args.cable, t_sim, time.time())
        except KeyError:
            return
        with lock:
            rows.append(row)

    def on_stats(text):
        m = re.search(rf'real_time_factor: ({NUM})', text)
        if m:
            with lock:
                rtf.append(float(m.group(1)))

    streams = [Stream(f'/world/{args.world}/dynamic_pose/info', on_pose),
               Stream(f'/world/{args.world}/stats', on_stats)]
    for s in streams:
        s.start()
    time.sleep(args.duration)
    for s in streams:
        s.stop()
    time.sleep(0.5)

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    with lock:
        data, rtfs = list(rows), list(rtf)
    if data:
        with (out / f'{args.prefix}.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)
    col = lambda k: [r[k] for r in data]  # noqa: E731
    summary = {'samples': len(data), 'duration_wall_s': args.duration}
    if data:
        summary.update({
            'sim_time_covered_s': data[-1]['t_sim'] - data[0]['t_sim'],
            'root_pose_first': [data[0]['root_x'], data[0]['root_y'], data[0]['root_z']],
            'dist_root_base_m': stats(col('dist_root_base_m')),
            'azimuth_deg': stats(col('azimuth_deg')),
            'elevation_deg': stats(col('elevation_deg')),
            'z_min_cable_m': stats(col('z_min_cable_m')),
            'roll_max_abs_deg': max(abs(v) for v in col('roll_deg')),
            'pitch_max_abs_deg': max(abs(v) for v in col('pitch_deg')),
            'drone_z_first_m': data[0]['drone_z'], 'drone_z_max_m': max(col('drone_z')),
            'nan_values': sum(1 for r in data for v in r.values() if isinstance(v, float) and not math.isfinite(v)),
        })
    if rtfs:
        summary['rtf'] = {'mean': sum(rtfs) / len(rtfs), 'min': min(rtfs), 'n': len(rtfs)}
    (out / f'{args.prefix}_summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
