#!/usr/bin/env python3
"""Gera a variante local x500_tether_attach: X500 upstream + tether_attach_link fisico.

Diferente de `x500_tethered`, esta variante **nao** embute nenhuma cadeia de cabo. Ela
apenas materializa o ponto de conexao que antes era um offset virtual dentro do
`base_link`, para que o `tether_anchor_chain` possa aplicar a forca num link real.

O modelo upstream do PX4 e lido, nunca escrito.
"""
import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PX4_X500_SDF = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'models' / 'x500' / 'model.sdf'
PX4_X500_CONFIG = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'models' / 'x500' / 'model.config'
OUT_DIR = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'x500_tether_attach'

MODEL_NAME = 'x500_tether_attach'
ATTACH_LINK = 'tether_attach_link'
ATTACH_JOINT = 'tether_attach_fixed'
ANCHOR_IN_SDF = '    <link name="rotor_0">'


def attach_block(offset_z, mass, inertia, visual_radius, collision_radius):
    collision = ''
    if collision_radius > 0:
        collision = f'''
      <collision name="tether_attach_collision">
        <geometry>
          <sphere>
            <radius>{collision_radius:.9g}</radius>
          </sphere>
        </geometry>
      </collision>'''
    return f'''
    <link name="{ATTACH_LINK}">
      <pose relative_to="base_link">0 0 {offset_z:.9g} 0 0 0</pose>
      <inertial>
        <mass>{mass:.9g}</mass>
        <inertia>
          <ixx>{inertia:.9g}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{inertia:.9g}</iyy>
          <iyz>0</iyz>
          <izz>{inertia:.9g}</izz>
        </inertia>
      </inertial>
      <visual name="tether_attach_visual">
        <geometry>
          <sphere>
            <radius>{visual_radius:.9g}</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>0.85 0.05 0.03 1.0</diffuse>
          <ambient>0.85 0.05 0.03 1.0</ambient>
        </material>
      </visual>{collision}
    </link>
    <joint name="{ATTACH_JOINT}" type="fixed">
      <parent>base_link</parent>
      <child>{ATTACH_LINK}</child>
    </joint>
'''


def add_collide_bitmask(sdf, bitmask):
    """Poe <collide_bitmask> em todas as colisoes do X500 (cada uma ja tem <surface><contact>).

    Contatos so existem se (mascara_a & mascara_b) != 0. O solo usa 65535; com o drone em 1 e o
    cabo em 2 (generate_tether_anchor_chain.py --collide-bitmask 2) o cabo nao colide com o drone.
    """
    collisions = sdf.count('<collision ')
    contacts = sdf.count('<contact>')
    if collisions == 0 or contacts != collisions or '<collide_bitmask>' in sdf:
        raise SystemExit(f'x500 upstream inesperado: {collisions} colisoes, {contacts} <contact>')
    return sdf.replace('<contact>', f'<contact>\n            <collide_bitmask>{bitmask:d}</collide_bitmask>')


def generate(offset_z, mass, inertia, visual_radius, collision_radius,
             model_name=MODEL_NAME, out_dir=OUT_DIR, collide_bitmask=None):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    sdf = PX4_X500_SDF.read_text()
    if "<model name='x500'>" in sdf:
        sdf = sdf.replace("<model name='x500'>", f"<model name='{model_name}'>", 1)
    else:
        sdf = sdf.replace('<model name="x500">', f'<model name="{model_name}">', 1)
    if f'name="{ATTACH_LINK}"' in sdf:
        raise SystemExit(f'modelo upstream ja define {ATTACH_LINK}; abortando')
    if ANCHOR_IN_SDF not in sdf:
        raise SystemExit('nao foi possivel localizar <link name="rotor_0"> no x500 upstream')

    if collide_bitmask is not None:
        sdf = add_collide_bitmask(sdf, collide_bitmask)
    block = attach_block(offset_z, mass, inertia, visual_radius, collision_radius)
    sdf = sdf.replace(ANCHOR_IN_SDF, block + ANCHOR_IN_SDF, 1)
    (out_dir / 'model.sdf').write_text(sdf)

    config = PX4_X500_CONFIG.read_text()
    config = config.replace('<name>x500</name>', f'<name>{model_name}</name>', 1)
    config = re.sub(
        r'<description>.*?</description>',
        (
            '<description>'
            f'Variante local do X500 com {ATTACH_LINK} fisico em base_link + '
            f'(0, 0, {offset_z:.9g}), sem cabo embutido. Endpoint do tether para o '
            'plugin drone_cabo::TetherForceConstraint.'
            '</description>'
        ),
        config,
        count=1,
        flags=re.DOTALL,
    )
    (out_dir / 'model.config').write_text(config)

    print(f'modelo={model_name}')
    print(f'attach_link={ATTACH_LINK}')
    print(f'attach_joint={ATTACH_JOINT} (fixed, base_link -> {ATTACH_LINK})')
    print(f'pose_relativa_base_link=0 0 {offset_z:.9g} 0 0 0')
    print(f'massa_attach={mass:.9g} kg')
    print(f'inercia_attach={inertia:.9g} kg.m^2')
    print(f'colisao_attach={"sim" if collision_radius > 0 else "nao"}')
    print(f'cabo_embutido=nao')
    print(f'collide_bitmask={collide_bitmask if collide_bitmask is not None else "padrao (65535)"}')
    print(f'sdf={out_dir / "model.sdf"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offset-z', type=float, default=-0.12,
                        help='pose do attach link em relacao ao base_link (m)')
    parser.add_argument('--mass', type=float, default=0.005)
    parser.add_argument('--inertia', type=float, default=2.0e-7)
    parser.add_argument('--visual-radius', type=float, default=0.025)
    # Sem colisao por padrao: a conexao e force-based, nao por contato, e uma
    # esfera 0,12 m abaixo do base_link fica sob o trem de pouso e introduz um
    # contato com o solo que a baseline A0.2 nao tinha.
    parser.add_argument('--collision-radius', type=float, default=0.0)
    parser.add_argument('--model-name', default=MODEL_NAME,
                        help='nome do modelo gerado (copias com mascara usam outro nome)')
    parser.add_argument('--output-dir', default=str(OUT_DIR),
                        help='diretorio do modelo; padrao = variante versionada')
    parser.add_argument('--collide-bitmask', type=int, default=None,
                        help='<collide_bitmask> de todas as colisoes do X500 (16 bits); padrao = nao emitir')
    args = parser.parse_args()
    if args.collide_bitmask is not None and not 0 <= args.collide_bitmask <= 0xFFFF:
        raise SystemExit('collide-bitmask must fit in 16 bits')
    generate(args.offset_z, args.mass, args.inertia, args.visual_radius, args.collision_radius,
             args.model_name, args.output_dir, args.collide_bitmask)


if __name__ == '__main__':
    main()
