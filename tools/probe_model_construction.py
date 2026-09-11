#!/usr/bin/env python3
"""Isola CONSTRUCAO do modelo de dinamica: spawna o cabo num mundo vazio, sem PX4.

Motivo: os abortos de B2/C2 em N alto nao tem a assinatura de divergencia
(`BallJoint::updateRelativeTransform`) e sim
`Joint::setTransformFromParentBodyNode` dentro de
`gz::physics::dartsim::SDFFeatures::ConstructSdfJoint`, chamada por
`PhysicsPrivate::CreatePhysicsEntities`. Isso acontece ANTES de qualquer passo de
fisica. Se o aborto se reproduzir num mundo vazio, sem UAV, sem constraint ativa e
sem gravidade agindo sobre o cabo por tempo relevante, entao o limite de N nao e
estabilidade: e construcao do modelo. Usa a cadeia de ball joints de producao.

Um `gz sim` novo por N, porque o aborto derruba o servidor.
"""
import argparse
import os
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / 'src' / 'pacote_do_drone' / 'models'
PLUGINS = ROOT / 'build' / 'gz_plugins'
TETHER_SDF = MODELS / 'tether_anchor_chain' / 'model.sdf'
ASSERTION_TOKENS = ('verifyTransform', 'Assertion', 'Aborted', 'Segmentation')


def sh(cmd, **kwargs):
    return subprocess.run(cmd, capture_output=True, text=True, **kwargs)


def gz_processes():
    listing = sh(['ps', '-eo', 'pid=,comm=']).stdout.splitlines()
    return [line.split()[0] for line in listing
            if line.split()[1:] and line.split()[1] in ('gz', 'ruby')]


def kill_gz():
    for pid in gz_processes():
        sh(['kill', '-KILL', pid])
    for _ in range(20):
        if not gz_processes():
            return True
        time.sleep(0.5)
    return False


def generate(n_links, length, collisions, radius, rho):
    target = f'{0.953 * length:.6g} 0 -0.083'
    cmd = [str(ROOT / 'tools' / 'generate_tether_anchor_chain.py'),
           '--links', str(n_links), '--length', str(length), '--rho', str(rho),
           '--radius', str(radius), '--initial-axis', 'taut', '--taut-target', target,
           '--force-constraint', '--stiffness', '5', '--damping', '0.5', '--max-force', '3']
    if collisions:
        cmd.append('--link-collisions')
    result = sh(cmd, cwd=ROOT)
    if result.returncode != 0:
        raise SystemExit(f'gerador falhou para N={n_links}: {result.stderr}')


def probe(n_links, length, collisions, radius, rho, world, log_dir, hold, label='ball'):
    generate(n_links, length, collisions, radius, rho)
    kill_gz()
    env = {**os.environ,
           'GZ_SIM_RESOURCE_PATH': f'{MODELS}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}',
           'GZ_SIM_SYSTEM_PLUGIN_PATH':
               f'{PLUGINS}:{os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")}'}
    log_path = Path(log_dir) / f'{label}_n{n_links:03d}.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_path, 'w')
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '1', world], env=env,
                              stdout=handle, stderr=subprocess.STDOUT,
                              stdin=subprocess.DEVNULL)
    world_name = Path(world).stem
    deadline = time.time() + 60
    ready = False
    while time.time() < deadline:
        if f'/world/{world_name}/stats' in sh(['gz', 'topic', '-l']).stdout:
            ready = True
            break
        time.sleep(1.0)
    result = {'n_links': n_links, 'length_m': length,
              'segment_length_m': length / n_links, 'server_ready': ready}
    if not ready:
        handle.close()
        kill_gz()
        result['constructed'] = False
        result['note'] = 'servidor nao subiu'
        return result

    request = (f'sdf_filename: "{TETHER_SDF}" name: "tether_anchor_chain" '
               'allow_renaming: false pose: {position: {z: 0}}')
    spawn = sh(['gz', 'service', '-s', f'/world/{world_name}/create',
                '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '10000', '--req', request])
    result['spawn_ack'] = 'data: true' in spawn.stdout
    time.sleep(hold)
    alive = server.poll() is None and bool(gz_processes())
    handle.close()
    text = log_path.read_text(errors='replace')
    result['assertion'] = any(token in text for token in ASSERTION_TOKENS)
    result['constructed'] = bool(alive and not result['assertion'])
    result['log'] = str(log_path)
    kill_gz()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--links', type=int, nargs='+', required=True)
    parser.add_argument('--length', type=float, default=2.5)
    parser.add_argument('--rho', type=float, default=0.06)
    parser.add_argument('--radius', type=float, default=0.003)
    parser.add_argument('--no-collisions', dest='collisions', action='store_false')
    parser.add_argument('--world', default='empty.sdf')
    parser.add_argument('--hold', type=float, default=4.0,
                        help='segundos de parede apos o spawn antes de julgar')
    parser.add_argument('--log-dir', default='results/construcao')
    args = parser.parse_args()

    print(f'{"N":>5} {"l [m]":>9} {"spawn":>6} {"construiu":>10} assercao')
    rows = []
    for n_links in args.links:
        row = probe(n_links, args.length, args.collisions,
                    args.radius, args.rho, args.world, args.log_dir, args.hold)
        rows.append(row)
        print(f"{row['n_links']:>5} {row['segment_length_m']:>9.4f} "
              f"{str(row.get('spawn_ack')):>6} {str(row['constructed']):>10} "
              f"{row.get('assertion')}", flush=True)
    return rows


if __name__ == '__main__':
    main()
