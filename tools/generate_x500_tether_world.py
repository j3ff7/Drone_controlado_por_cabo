#!/usr/bin/env python3
"""Mundo com X500 + estacao + tether ligados por JUNTAS FISICAS (arquitetura da `dev`).

Na `dev` a conexao do cabo e estatica, no SDF do MUNDO: uma junta `ball` liga a estacao a
raiz do cabo e outra liga a ponta do cabo ao `base_link` do drone, que e a raiz da arvore do
drone (ligar a um link que ja tem pai, como `tether_attach_link`, fecharia um laco
cinematico, o que o DART rejeita). Este gerador reproduz essa topologia com o X500 do PX4:

    world ─fixed─ ground_station_base ─fixed─ tether_exit_point
          ─ball─ tether_link_1 ─ball─ ... ─ball─ tether_link_N
          ─ball (junta de mundo, pivo em tether_attach_link)─ x500_tether_attach_0::base_link

O mundo `default` do PX4 e lido e recebe apenas os includes e a junta; o clone do PX4 nao e
modificado. O PX4 depois se liga ao X500 ja existente com `PX4_GZ_MODEL_NAME`, sem spawnar
outro. Sem plugin de forca (`TetherForceConstraint`), sem atuador de reel, sem prismatica.
"""
import argparse
import math
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PX4_WORLD = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'worlds' / 'default.sdf'
GENERATOR = ROOT / 'tools' / 'generate_tether_anchor_chain.py'
DEFAULT_OUT = ROOT / 'src' / 'pacote_do_drone' / 'worlds' / 'x500_tether_joint.sdf'

DRONE_MODEL = 'x500_tether_attach'
DRONE_NAME = 'x500_tether_attach_0'      # mesmo nome que o gz_bridge daria ao spawnar
TETHER_NAME = 'tether_anchor_chain'
BASE_LINK_REST_Z = 0.227                 # altura medida do base_link com o X500 pousado (A0.3)
ATTACH_OFFSET_Z = -0.12                  # tether_attach_link relativo ao base_link
EXIT_Z = 0.19                            # tether_exit_point acima da base da estacao


def build_tether(links, length, rho, radius, collisions, target, initial_axis, bulge):
    cmd = [str(GENERATOR), '--links', str(links), '--length', str(length),
           '--rho', str(rho), '--radius', str(radius), '--initial-axis', initial_axis,
           '--joint-type', 'ball', '--no-reel-actuator']
    if initial_axis == 'taut':
        cmd += ['--taut-target', ' '.join(f'{c:.6f}' for c in target), '--taut-bulge', bulge]
    if collisions:
        cmd.append('--link-collisions')
    result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f'gerador do tether falhou: {result.stderr}')
    return result.stdout


def world_sdf(drone_x, drone_y, links, joint_name):
    base_z = BASE_LINK_REST_Z
    injected = f'''
    <!-- Estacao + tether (juntas internas ball, ancorado ao mundo pelo proprio modelo) -->
    <include>
      <uri>model://{TETHER_NAME}</uri>
      <name>{TETHER_NAME}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- X500 com tether_attach_link; o PX4 se liga a ele por PX4_GZ_MODEL_NAME -->
    <include>
      <uri>model://{DRONE_MODEL}</uri>
      <name>{DRONE_NAME}</name>
      <pose>{drone_x:.6f} {drone_y:.6f} {base_z:.6f} 0 0 0</pose>
    </include>

    <!-- Conexao fisica ponta do cabo -> drone (arquitetura da dev).
         Filho = base_link, raiz da arvore do X500: a arvore continua aberta.
         A pose e relativa ao filho: o pivo fica no tether_attach_link. -->
    <joint name="{joint_name}" type="ball">
      <parent>{TETHER_NAME}::tether_link_{links}</parent>
      <child>{DRONE_NAME}::base_link</child>
      <pose>0 0 {ATTACH_OFFSET_Z:.6f} 0 0 0</pose>
    </joint>
'''
    text = PX4_WORLD.read_text()
    if "<world name='default'>" not in text:
        raise SystemExit('mundo default do PX4 com formato inesperado')
    return re.sub(r'(\n\s*</world>)', injected + r'\1', text, count=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--links', type=int, default=10)
    parser.add_argument('--length', type=float, default=2.5)
    parser.add_argument('--rho', type=float, default=0.06)
    parser.add_argument('--radius', type=float, default=0.003)
    parser.add_argument('--no-collisions', dest='collisions', action='store_false')
    parser.add_argument('--drone-x', type=float, default=1.0,
                        help='posicao do X500 no mundo [m]; a estacao fica na origem')
    parser.add_argument('--drone-y', type=float, default=0.0)
    parser.add_argument('--initial-axis', choices=('taut', 'x'), default='taut')
    parser.add_argument('--taut-bulge', choices=('down', 'up'), default='up',
                        help='up (padrao): o arco nasce acima do solo e cai sobre ele')
    parser.add_argument('--joint-name', default='tether_drone_ball')
    parser.add_argument('--output', default=str(DEFAULT_OUT))
    args = parser.parse_args()

    attach = (args.drone_x, args.drone_y, BASE_LINK_REST_Z + ATTACH_OFFSET_Z)
    target = (attach[0], attach[1], attach[2] - EXIT_Z)      # relativo ao tether_exit_point
    chord = math.sqrt(sum(c * c for c in target))
    if chord >= args.length:
        raise SystemExit(f'cabo de {args.length} m nao alcanca o X500 a {chord:.3f} m')
    report = build_tether(args.links, args.length, args.rho, args.radius, args.collisions,
                          target, args.initial_axis, args.taut_bulge)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(world_sdf(args.drone_x, args.drone_y, args.links, args.joint_name))

    for key in ('N_links', 'comprimento_por_link', 'massa_total', 'joint_type', 'colisoes_links',
                'inicializacao', 'force_constraint', 'reel_atuador'):
        for line in report.splitlines():
            if line.startswith(key + '='):
                print(line)
    print(f'drone={DRONE_NAME} em ({args.drone_x:.3f}, {args.drone_y:.3f}, {BASE_LINK_REST_Z:.3f})')
    print(f'pivo_no_drone=({attach[0]:.3f}, {attach[1]:.3f}, {attach[2]:.3f}) '
          f'corda_inicial={chord:.3f} m folga={(args.length - chord) / args.length * 100:.1f}%')
    print(f'junta_de_mundo={args.joint_name}: {TETHER_NAME}::tether_link_{args.links} -> {DRONE_NAME}::base_link (ball)')
    print(f'mundo={out}')


if __name__ == '__main__':
    main()
