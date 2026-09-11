#!/usr/bin/env python3
"""Gera a bancada isolada de torque do reel (A4), sem PX4 e sem tether.

O `reel_link` e o `reel_joint` sao extraidos **verbatim** do modelo de producao
`tether_anchor_chain`, de modo que os parametros da bancada nao podem divergir dos
parametros de voo: massa, inercia, raio, largura, eixo, damping e friction vem todos
do mesmo arquivo que a simulacao completa usa.
"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'
OUT = ROOT / 'src' / 'pacote_do_drone' / 'worlds' / 'reel_torque_bench.sdf'

BENCH_MODEL = 'reel_bench'
BASE_LINK = 'ground_station_base'
REEL_LINK = 'reel_link'
REEL_JOINT = 'reel_joint'


def indent(text, prefix):
    return '\n'.join(prefix + line if line.strip() else line for line in text.splitlines())


def extract(model, tag, name):
    for element in model.findall(tag):
        if element.attrib.get('name') == name:
            return element
    raise SystemExit(f'{tag} "{name}" nao encontrado em {PRODUCTION}')


def reel_parameters(model):
    link = extract(model, 'link', REEL_LINK)
    joint = extract(model, 'joint', REEL_JOINT)
    cylinder = link.find('visual/geometry/cylinder')
    return {
        'mass': float(link.findtext('inertial/mass')),
        'ixx': float(link.findtext('inertial/inertia/ixx')),
        'iyy': float(link.findtext('inertial/inertia/iyy')),
        'izz': float(link.findtext('inertial/inertia/izz')),
        'radius': float(cylinder.findtext('radius')),
        'width': float(cylinder.findtext('length')),
        'axis': joint.findtext('axis/xyz'),
        'damping': float(joint.findtext('axis/dynamics/damping')),
        'friction': float(joint.findtext('axis/dynamics/friction')),
    }


def build(torque, lever_arm, probe_wrench, step_size, actuator=None):
    model = ET.parse(PRODUCTION).getroot().find('model')
    params = reel_parameters(model)

    # Copia literal do link e da junta de producao: nada e reescrito aqui.
    reel_link_xml = indent(ET.tostring(extract(model, 'link', REEL_LINK), encoding='unicode').rstrip(), '    ')
    reel_joint_xml = indent(ET.tostring(extract(model, 'joint', REEL_JOINT), encoding='unicode').rstrip(), '    ')

    arm = lever_arm if lever_arm is not None else params['radius']
    force_x = torque / arm if arm else 0.0

    # A6: bancada com atuador. O ReelTorqueBench continua presente so para publicar
    # theta/omega com carimbo de tempo; a forca de referencia dele fica em zero.
    actuator_xml = ''
    if actuator:
        actuator_xml = f'''
      <plugin filename="libReelActuator.so" name="drone_cabo::ReelActuator">
        <reel_model>{BENCH_MODEL}</reel_model>
        <reel_joint>{REEL_JOINT}</reel_joint>
        <tau_max>{actuator["tau_max"]:.9g}</tau_max>
        <omega_max>{actuator["omega_max"]:.9g}</omega_max>
        <ramp_rate>{actuator["ramp_rate"]:.9g}</ramp_rate>
        <command_topic>{actuator["topic"]}</command_topic>
      </plugin>'''

    world = f'''<?xml version="1.0" ?>
<!--
  GERADO POR tools/generate_reel_bench.py - NAO EDITAR A MAO.

  Bancada A4: reel isolado, sem PX4, sem tether, sem ball joints. O link e a junta do
  reel sao copias literais de tether_anchor_chain/model.sdf.

  Torque de referencia: tau_ref = lever_arm * body_force_x = {arm:.9g} * {force_x:.9g}
                                = {torque:.9g} N.m
-->
<sdf version="1.9">
  <world name="reel_bench">
    <physics name="default" type="ode">
      <max_step_size>{step_size:.9g}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <gravity>0 0 -9.8</gravity>

    <model name="{BENCH_MODEL}">
      <static>false</static>
      <link name="{BASE_LINK}">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="ground_station_world_fixed" type="fixed">
        <parent>world</parent>
        <child>{BASE_LINK}</child>
      </joint>
{reel_link_xml}
{reel_joint_xml}
      <plugin filename="libReelTorqueBench.so" name="drone_cabo::ReelTorqueBench">
        <reel_model>{BENCH_MODEL}</reel_model>
        <reel_link>{REEL_LINK}</reel_link>
        <reel_joint>{REEL_JOINT}</reel_joint>
        <lever_arm>{arm:.9g}</lever_arm>
        <body_force_x>{force_x:.9g}</body_force_x>
        <probe_transmitted_wrench>{'true' if probe_wrench else 'false'}</probe_transmitted_wrench>
      </plugin>{actuator_xml}
    </model>
  </world>
</sdf>
'''
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(world)
    return params, arm, force_x


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--torque', type=float, default=0.0, help='tau_ref desejado [N.m]')
    parser.add_argument('--lever-arm', type=float, default=None,
                        help='braco de alavanca [m]; por padrao o raio do tambor')
    parser.add_argument('--probe-transmitted-wrench', action='store_true')
    parser.add_argument('--step-size', type=float, default=0.004)
    parser.add_argument('--actuator', action='store_true',
                        help='adiciona o ReelActuator (A6) a bancada')
    parser.add_argument('--tau-max', type=float, default=0.05)
    parser.add_argument('--omega-max', type=float, default=12.0)
    parser.add_argument('--ramp-rate', type=float, default=0.05)
    parser.add_argument('--command-topic', default='/cabo/tms/reel_cmd')
    args = parser.parse_args()

    actuator = None
    if args.actuator:
        actuator = {'tau_max': args.tau_max, 'omega_max': args.omega_max,
                    'ramp_rate': args.ramp_rate, 'topic': args.command_topic}
    params, arm, force_x = build(
        args.torque, args.lever_arm, args.probe_transmitted_wrench, args.step_size,
        actuator)

    print(f'reel_massa={params["mass"]:.9g} kg')
    print(f'reel_raio={params["radius"]:.9g} m')
    print(f'reel_largura={params["width"]:.9g} m')
    print(f'reel_inercia_axial={params["iyy"]:.9g} kg.m^2')
    print(f'reel_eixo={params["axis"]}')
    print(f'reel_damping={params["damping"]:.9g} N.m.s/rad')
    print(f'reel_friction={params["friction"]:.9g} N.m')
    print(f'braco={arm:.9g} m')
    print(f'forca_no_corpo={force_x:.9g} N')
    print(f'tau_ref={arm * force_x:.9g} N.m')
    print(f'probe_transmitted_wrench={args.probe_transmitted_wrench}')
    print(f'atuador={bool(actuator)}')
    if actuator:
        print(f'tau_max={actuator["tau_max"]:.9g} N.m')
        print(f'omega_max={actuator["omega_max"]:.9g} rad/s')
        print(f'ramp_rate={actuator["ramp_rate"]:.9g} N.m/s')
        print(f'topico_de_comando={actuator["topic"]}')
    print(f'mundo={OUT}')


if __name__ == '__main__':
    main()
