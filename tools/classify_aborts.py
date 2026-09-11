#!/usr/bin/env python3
"""Classifica cada configuracao sem dados de um sweep: construcao ou dinamica?

A distincao decide o que fazer a seguir e nao esta no resumo do sweep:

  Joint.cpp:537 / ConstructSdfJoint      -> o modelo nunca foi instanciado
  BallJoint.cpp:159   + PhysicsPrivate::Step -> divergiu simulando
  RevoluteJoint.cpp:186 + PhysicsPrivate::Step -> divergiu simulando

Um aborto de construcao nao depende de PX4, de voo nem de tempo de simulacao, e pode
ser reproduzido em 15 s com `tools/probe_model_construction.py`; um aborto dinamico so
aparece rodando.
"""
import argparse
import json
import re
from pathlib import Path

ESCAPE_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
DART_RE = re.compile(r'dart/dynamics/(\w+)\.cpp:(\d+)')


def classify(log_path):
    if not log_path.exists():
        return 'sem log'
    text = ESCAPE_RE.sub('', log_path.read_text(errors='replace'))
    match = DART_RE.search(text)
    if not match:
        return 'sem assercao do DART'
    where = f'{match.group(1)}.cpp:{match.group(2)}'
    if 'ConstructSdfJoint' in text:
        return f'{where} — CONSTRUCAO'
    if 'PhysicsPrivate::Step' in text:
        return f'{where} — DINAMICA'
    return where


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--indir', nargs='+', default=['results'])
    args = parser.parse_args()

    rows = []
    for root in args.indir:
        for sweep in sorted(Path(root).rglob('sweep.json')):
            for entry in json.loads(sweep.read_text()).get('configs', []):
                if entry.get('stable'):
                    continue
                log = sweep.parent / f"n{entry['n_links']:02d}" / 'px4.log'
                rows.append((str(sweep.parent), entry.get('joint_type', 'ball'),
                             entry.get('initial_shape', '?'), entry.get('length_m'),
                             entry['n_links'], entry['segment_length_m'], classify(log)))
    if not rows:
        print('nenhuma configuracao sem dados')
        return
    print(f"{'caso':<40} {'junta':<13} {'inicial':<6} {'L':>5} {'N':>5} {'l':>8}  causa")
    for case, joint, shape, length, n_links, segment, cause in rows:
        print(f'{case:<40} {joint:<13} {shape:<6} {length:>5} {n_links:>5} {segment:>8.4f}  {cause}')
    construction = sum(1 for r in rows if 'CONSTRUCAO' in r[6])
    print(f'\ntotal sem dados: {len(rows)}  |  construcao: {construction}  '
          f'|  dinamica: {len(rows) - construction}')


if __name__ == '__main__':
    main()
