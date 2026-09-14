#!/usr/bin/env python3
"""Integridade da conexao fisica tether <-> X500 a partir das poses do Gazebo.

Com juntas fisicas nas duas pontas nao ha constraint de forca, entao `constraint_error`
passa a ser o que ele e fisicamente para uma junta: a distancia entre a ponta do cabo e o
pivo no drone. Deve ficar em ~0; qualquer crescimento e junta violada.

Le `/world/<mundo>/pose/info` (poses por entidade: links relativos ao modelo pai, modelos
relativos ao mundo) e `/world/<mundo>/stats`. Grava CSV e um resumo JSON, incluindo quanto
tempo SIMULADO foi coberto — se o servidor morrer, a cobertura denuncia.
"""
import argparse
import csv
import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path


NUM = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|-?inf'


def quat_rotate(q, v):
    x, y, z, w = q
    ux, uy, uz = v
    tx, ty, tz = 2 * (y * uz - z * uy), 2 * (z * ux - x * uz), 2 * (x * uy - y * ux)
    return (ux + w * tx + (y * tz - z * ty),
            uy + w * ty + (z * tx - x * tz),
            uz + w * tz + (x * ty - y * tx))


def compose(parent, child_pos):
    p, q = parent
    r = quat_rotate(q, child_pos)
    return (p[0] + r[0], p[1] + r[1], p[2] + r[2])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def tether_angles(drone_q, last_link_q):
    """Tangente do cabo junto ao drone e (azimute, elevacao) no frame do drone, em graus.

    Mesma convencao do plugin: o ultimo elo se estende no +x local ate o drone, entao a
    direcao que sai do drone ao longo do cabo e -R_N (1,0,0); azimute 0 = frente,
    +90 = esquerda; elevacao -90 = cabo pendurado reto abaixo.
    """
    t_world = quat_rotate(last_link_q, (-1.0, 0.0, 0.0))
    t_body = quat_rotate(quat_conj(drone_q), t_world)
    azimuth = math.degrees(math.atan2(t_body[1], t_body[0]))
    elevation = math.degrees(math.atan2(t_body[2], math.hypot(t_body[0], t_body[1])))
    return t_body, azimuth, elevation


def roll_pitch_deg(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def parse_pose_block(block):
    """Um `pose {...}` do texto protobuf. Campos zero sao omitidos."""
    name = re.search(r'name: "([^"]+)"', block)
    pos = re.search(r'position \{(.*?)\}', block, re.S)
    ori = re.search(r'orientation \{(.*?)\}', block, re.S)

    def fields(text, keys, default):
        out = dict(default)
        if text:
            for k, v in re.findall(rf'([xyzw]): ({NUM})', text.group(1)):
                out[k] = float(v)
        return tuple(out[k] for k in keys)

    return (name.group(1) if name else None,
            fields(pos, 'xyz', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
            fields(ori, 'xyzw', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}))


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, required=True, help='segundos de parede')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--prefix', default='conexao')
    parser.add_argument('--world', default='default')
    parser.add_argument('--drone-model', default='x500_tether_attach_0')
    parser.add_argument('--drone-link', default='tether_attach_link')
    parser.add_argument('--tether-model', default='tether_anchor_chain')
    parser.add_argument('--links', type=int, required=True)
    parser.add_argument('--segment-length', type=float, required=True)
    parser.add_argument('--settle', type=float, default=5.0,
                        help='s simulados descartados no z_min "assentado"')
    args = parser.parse_args()

    rows, stats = [], []
    lock = threading.Lock()
    tether_links = {f'tether_link_{i}' for i in range(1, args.links + 1)}

    def on_pose(text):
        stamp = re.search(r'header \{.*?stamp \{(.*?)\}', text, re.S)
        t = 0.0
        if stamp:
            sec = re.search(r'sec: (\d+)', stamp.group(1))
            nsec = re.search(r'nsec: (\d+)', stamp.group(1))
            t = (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9
        poses = {}
        for block in re.split(r'\n(?=pose \{)', text):
            name, pos, ori = parse_pose_block(block)
            if name:
                poses.setdefault(name, (pos, ori))
        need = {args.drone_model, args.drone_link, args.tether_model, 'tether_exit_point',
                f'tether_link_{args.links}', 'tether_link_1'}
        if not need.issubset(poses):
            return
        drone = poses[args.drone_model]
        tether = poses[args.tether_model]
        attach = compose(drone, poses[args.drone_link][0])
        last_pos, last_ori = poses[f'tether_link_{args.links}']
        tip_local = quat_rotate(last_ori, (args.segment_length, 0.0, 0.0))
        tip = compose(tether, tuple(last_pos[i] + tip_local[i] for i in range(3)))
        exit_point = compose(tether, poses['tether_exit_point'][0])
        first = compose(tether, poses['tether_link_1'][0])
        zs = [compose(tether, poses[n][0])[2] for n in tether_links if n in poses]
        roll, pitch = roll_pitch_deg(drone[1])
        last_world_q = quat_mul(tether[1], last_ori)
        t_body, azimuth, elevation = tether_angles(drone[1], last_world_q)
        row = {'t_sim': t,
               'tip_x': tip[0], 'tip_y': tip[1], 'tip_z': tip[2],
               'attach_x': attach[0], 'attach_y': attach[1], 'attach_z': attach[2],
               'pivot_gap': math.dist(tip, attach), 'exit_gap': math.dist(first, exit_point),
               'z_min_tether': min(zs) if zs else math.nan,
               'drone_x': drone[0][0], 'drone_y': drone[0][1], 'drone_z': drone[0][2],
               'roll_deg': roll, 'pitch_deg': pitch,
               'tan_body_x': t_body[0], 'tan_body_y': t_body[1], 'tan_body_z': t_body[2],
               'azimuth_deg': azimuth, 'elevation_deg': elevation}
        with lock:
            rows.append(row)

    def on_stats(text):
        m = re.search(rf'real_time_factor: ({NUM})', text)
        if m:
            with lock:
                stats.append(float(m.group(1)))

    streams = [Stream(f'/world/{args.world}/pose/info', on_pose),
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
        data, rtf = list(rows), list(stats)
    if data:
        with (out / f'{args.prefix}.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)

    def finite(key, subset=None):
        vals = [r[key] for r in (subset or data)]
        return [v for v in vals if isinstance(v, float) and math.isfinite(v)]

    summary = {'samples': len(data), 'duration_wall_s': args.duration}
    if data:
        t0, t1 = data[0]['t_sim'], data[-1]['t_sim']
        settled = [r for r in data if r['t_sim'] - t0 >= args.settle] or data
        gap = finite('pivot_gap')
        nan = sum(1 for r in data for v in r.values() if isinstance(v, float) and not math.isfinite(v))
        summary.update({
            'sim_time_covered_s': t1 - t0,
            'pivot_gap_m': {'rms': math.sqrt(sum(g * g for g in gap) / len(gap)), 'max': max(gap)},
            'exit_gap_max_m': max(finite('exit_gap')),
            'z_min_tether_m': min(finite('z_min_tether')),
            'z_min_tether_settled_m': min(finite('z_min_tether', settled)),
            'roll_max_abs_deg': max(abs(v) for v in finite('roll_deg')),
            'pitch_max_abs_deg': max(abs(v) for v in finite('pitch_deg')),
            'drone_z_initial_m': data[0]['drone_z'],
            'drone_z_max_m': max(finite('drone_z')),
            'attach_z_initial_m': data[0]['attach_z'],
            'nan_values': nan,
            'azimuth_deg': {'first': data[0]['azimuth_deg'], 'last': data[-1]['azimuth_deg']},
            'elevation_deg': {'first': data[0]['elevation_deg'], 'last': data[-1]['elevation_deg'],
                              'min': min(finite('elevation_deg')), 'max': max(finite('elevation_deg'))},
        })
    if rtf:
        summary['rtf'] = {'mean': sum(rtf) / len(rtf), 'min': min(rtf), 'samples': len(rtf)}
    (out / f'{args.prefix}_summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
