#!/usr/bin/env python3
"""Captura a forma instantanea do cabo a partir de /world/default/dynamic_pose/info.

Serve para comparar geometria entre topologias (C1: ball x revolute) e entre direcoes
de perturbacao (C3: anisotropia). Reduz a poligonal dos elos a tres numeros:

  sag        distancia perpendicular maxima ao segmento saida->ponta, no plano vertical
  lateral    idem, na direcao horizontal transversal
  span       corda efetiva saida->ponta

`gz topic -e` imprime texto protobuf; campos de valor zero sao OMITIDOS, entao cada
componente ausente vale 0 e a mensagem so pode ser fechada pelo delimitador.
"""
import argparse
import json
import math
import re
import subprocess


NAME_RE = re.compile(r'^\s*name:\s*"([^"]+)"')
NUM_RE = re.compile(r'^\s*([xyzw]):\s*(-?[\d.eE+-]+|nan|-?inf)\s*$')


def capture(topic, timeout):
    out = subprocess.run(['gz', 'topic', '-e', '-t', topic, '-n', '1'],
                         capture_output=True, text=True, timeout=timeout).stdout
    poses, name, position, depth_field = {}, None, {}, None
    for line in out.splitlines():
        match = NAME_RE.match(line)
        if match:
            if name is not None and position:
                poses.setdefault(name, position)
            name, position, depth_field = match.group(1), {}, None
            continue
        if 'position' in line:
            depth_field = 'position'
            continue
        if 'orientation' in line:
            depth_field = 'orientation'
            continue
        match = NUM_RE.match(line)
        if match and depth_field == 'position':
            position[match.group(1)] = float(match.group(2))
    if name is not None and position:
        poses.setdefault(name, position)
    return poses


def shape(poses, n_links, exit_z):
    points = []
    for index in range(1, n_links + 1):
        pose = poses.get(f'tether_link_{index}')
        if pose is None:
            continue
        points.append((pose.get('x', 0.0), pose.get('y', 0.0), pose.get('z', 0.0)))
    if len(points) < 2:
        return None
    start = (0.0, 0.0, exit_z)
    end = points[-1]
    chord = [end[i] - start[i] for i in range(3)]
    norm = math.sqrt(sum(c * c for c in chord))
    unit = [c / norm for c in chord] if norm > 0 else [1.0, 0.0, 0.0]
    sag = lateral = 0.0
    for point in points:
        rel = [point[i] - start[i] for i in range(3)]
        along = sum(rel[i] * unit[i] for i in range(3))
        perp = [rel[i] - along * unit[i] for i in range(3)]
        sag = max(sag, -perp[2] if perp[2] < 0 else 0.0)
        lateral = max(lateral, abs(perp[1]))
    return {'n_points': len(points), 'span_m': norm, 'sag_m': sag,
            'lateral_m': lateral, 'tip': list(end), 'points': points}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--links', type=int, required=True)
    parser.add_argument('--exit-z', type=float, default=0.19)
    parser.add_argument('--topic', default='/world/default/dynamic_pose/info')
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    try:
        poses = capture(args.topic, args.timeout)
    except subprocess.TimeoutExpired:
        poses = {}
    result = shape(poses, args.links, args.exit_z)
    if result is None:
        raise SystemExit(f'nenhuma pose de tether_link_* em {args.topic}')
    result['topic'] = args.topic
    with open(args.output, 'w') as handle:
        json.dump(result, handle, indent=2)
    print(f"span={result['span_m']:.4f} m sag={result['sag_m']:.4f} m "
          f"lateral={result['lateral_m']:.4f} m elos={result['n_points']}")


if __name__ == '__main__':
    main()
