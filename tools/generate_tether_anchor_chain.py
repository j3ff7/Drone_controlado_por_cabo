#!/usr/bin/env python3
import argparse
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain'
OUT_SDF = OUT_DIR / 'model.sdf'
OUT_CONFIG = OUT_DIR / 'model.config'


def cylinder_inertia(mass, radius, length):
    i_transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    i_axial = 0.5 * mass * radius * radius
    return i_transverse, i_axial


def inertia_xml(ixx, iyy, izz, indent):
    return f'''{indent}<inertia>
{indent}  <ixx>{ixx:.9g}</ixx>
{indent}  <ixy>0</ixy>
{indent}  <ixz>0</ixz>
{indent}  <iyy>{iyy:.9g}</iyy>
{indent}  <iyz>0</iyz>
{indent}  <izz>{izz:.9g}</izz>
{indent}</inertia>'''


def matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matrix_to_rpy(m):
    pitch = math.atan2(-m[2][0], math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]))
    roll = math.atan2(m[2][1], m[2][2])
    yaw = math.atan2(m[1][0], m[0][0])
    return roll, pitch, yaw


def orientation_from_x_axis(vector):
    x, y, z = vector
    yaw = math.atan2(y, x)
    pitch = math.atan2(-z, math.sqrt(x * x + y * y))
    return rpy_to_matrix(0.0, pitch, yaw)


def folded_ground_vectors(link_length, target_z):
    xy_a = link_length
    xy_b = 0.309016994 * link_length
    xy_c = 0.951056516 * link_length
    residual_x = xy_a - xy_b - xy_b
    half_z = 0.5 * target_z
    last_x = -0.5 * residual_x
    last_y = math.sqrt(max(0.0, link_length * link_length - last_x * last_x - half_z * half_z))
    return [
        (xy_a, 0.0, 0.0),
        (-xy_b, xy_c, 0.0),
        (-xy_b, -xy_c, 0.0),
        (last_x, last_y, half_z),
        (last_x, -last_y, half_z),
    ]


def folded_link_xml(index, n_links, link_length, mass, radius, relative_pose, link_collisions):
    name = f'tether_link_{index}'
    parent_frame = 'anchor_link' if index == 1 else f'tether_link_{index - 1}'
    color = '0.02 0.02 0.02 1.0' if index < n_links else '0.05 0.05 0.05 1.0'
    com_pose = f'{0.5 * link_length:.9g} 0 0 0 1.57079632679 0'
    i_transverse, i_axial = cylinder_inertia(mass, radius, link_length)
    ixx, iyy, izz = i_axial, i_transverse, i_transverse
    collision = ''
    if link_collisions:
        collision = f'''
      <collision name="{name}_collision">
        <pose>{com_pose}</pose>
        <geometry>
          <cylinder>
            <radius>{0.75 * radius:.9g}</radius>
            <length>{link_length:.9g}</length>
          </cylinder>
        </geometry>
      </collision>'''

    joint_pose = '0 0 0 0 0 0' if index == 1 else f'{link_length:.9g} 0 0 0 0 0'
    return f'''
    <link name="{name}">
      <pose relative_to="{parent_frame}">{relative_pose}</pose>
      <inertial>
        <pose>{com_pose}</pose>
        <mass>{mass:.9g}</mass>
{inertia_xml(ixx, iyy, izz, '        ')}
      </inertial>
      <visual name="{name}_visual">
        <pose>{com_pose}</pose>
        <geometry>
          <cylinder>
            <radius>{radius:.9g}</radius>
            <length>{link_length:.9g}</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>{color}</diffuse>
          <ambient>{color}</ambient>
        </material>
      </visual>
{collision}
    </link>
    <joint name="tether_joint_{index}" type="ball">
      <pose relative_to="{parent_frame}">{joint_pose}</pose>
      <parent>{parent_frame}</parent>
      <child>{name}</child>
    </joint>'''


def link_xml(index, n_links, link_length, mass, radius, axis, link_collisions):
    name = f'tether_link_{index}'
    if axis == 'x':
        relative_pose = f'{link_length:.9g} 0 0 0 0 0' if index > 1 else '0 0 0 0 0 0'
        joint_offset = f'{link_length:.9g} 0 0 0 0 0'
        com_pose = f'{0.5 * link_length:.9g} 0 0 0 1.57079632679 0'
        i_transverse, i_axial = cylinder_inertia(mass, radius, link_length)
        ixx, iyy, izz = i_axial, i_transverse, i_transverse
    else:
        relative_pose = f'0 0 {-link_length:.9g} 0 0 0' if index > 1 else '0 0 0 0 0 0'
        joint_offset = f'0 0 {-link_length:.9g} 0 0 0'
        com_pose = f'0 0 {-0.5 * link_length:.9g} 0 0 0'
        i_transverse, i_axial = cylinder_inertia(mass, radius, link_length)
        ixx, iyy, izz = i_transverse, i_transverse, i_axial

    parent_frame = 'anchor_link' if index == 1 else f'tether_link_{index - 1}'
    color = '0.02 0.02 0.02 1.0' if index < n_links else '0.05 0.05 0.05 1.0'
    collision = ''
    if link_collisions:
        collision = f'''
      <collision name="{name}_collision">
        <pose>{com_pose}</pose>
        <geometry>
          <cylinder>
            <radius>{0.75 * radius:.9g}</radius>
            <length>{link_length:.9g}</length>
          </cylinder>
        </geometry>
      </collision>'''

    joint_pose = '0 0 0 0 0 0' if index == 1 else joint_offset
    return f'''
    <link name="{name}">
      <pose relative_to="{parent_frame}">{relative_pose}</pose>
      <inertial>
        <pose>{com_pose}</pose>
        <mass>{mass:.9g}</mass>
{inertia_xml(ixx, iyy, izz, '        ')}
      </inertial>
      <visual name="{name}_visual">
        <pose>{com_pose}</pose>
        <geometry>
          <cylinder>
            <radius>{radius:.9g}</radius>
            <length>{link_length:.9g}</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>{color}</diffuse>
          <ambient>{color}</ambient>
        </material>
      </visual>
{collision}
    </link>
    <joint name="tether_joint_{index}" type="ball">
      <pose relative_to="{parent_frame}">{joint_pose}</pose>
      <parent>{parent_frame}</parent>
      <child>{name}</child>
    </joint>'''


def force_constraint_plugin_xml(
    n_links,
    link_length,
    enabled,
    initial_axis,
    drone_model,
    drone_link,
    drone_offset,
    stiffness,
    damping,
    max_force,
):
    if not enabled:
        return ''
    tether_offset = f'{link_length:.9g} 0 0' if initial_axis in ('x', 'folded_ground') else f'0 0 {-link_length:.9g}'
    return f'''
    <plugin filename="libTetherForceConstraint.so" name="drone_cabo::TetherForceConstraint">
      <drone_model>{drone_model}</drone_model>
      <drone_link>{drone_link}</drone_link>
      <tether_model>tether_anchor_chain</tether_model>
      <tether_link>tether_link_{n_links}</tether_link>
      <!-- Reservado: o plugin NAO le este campo. A instrumentacao direta do
           wrench em anchor_world_fixed via EnableTransmittedWrenchCheck
           derruba o backend DART (BallJoint::updateRelativeTransform). -->
      <anchor_joint>anchor_world_fixed</anchor_joint>
      <drone_offset>{drone_offset}</drone_offset>
      <tether_offset>{tether_offset}</tether_offset>
      <stiffness>{stiffness:.9g}</stiffness>
      <damping>{damping:.9g}</damping>
      <max_force>{max_force:.9g}</max_force>
    </plugin>'''


def generate(
    n_links,
    total_length,
    rho,
    radius,
    initial_axis,
    link_collisions,
    force_constraint,
    drone_model,
    drone_link,
    drone_offset,
    stiffness,
    damping,
    max_force,
):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    link_length = total_length / n_links
    total_mass = rho * total_length
    link_mass = total_mass / n_links
    if initial_axis == 'folded_ground' and n_links != 5:
        raise SystemExit('initial-axis=folded_ground is defined for the 5-link minimum experiment')
    parts = [f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="tether_anchor_chain">
    <static>false</static>
    <link name="anchor_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1</iyy>
          <iyz>0</iyz>
          <izz>1</izz>
        </inertia>
      </inertial>
      <visual name="anchor_visual">
        <geometry>
          <sphere>
            <radius>0.035</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>0.05 0.15 0.9 1.0</diffuse>
          <ambient>0.05 0.15 0.9 1.0</ambient>
        </material>
      </visual>
      <collision name="anchor_collision">
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <joint name="anchor_world_fixed" type="fixed">
      <parent>world</parent>
      <child>anchor_link</child>
    </joint>''']
    if initial_axis == 'folded_ground':
        target_z = 0.085
        rotations = [orientation_from_x_axis(v) for v in folded_ground_vectors(link_length, target_z)]
        identity = rpy_to_matrix(0.0, 0.0, 0.0)
        previous = identity
        for i, rotation in enumerate(rotations, start=1):
            relative = matmul(transpose(previous), rotation)
            roll, pitch, yaw = matrix_to_rpy(relative)
            translation = '0 0 0' if i == 1 else f'{link_length:.9g} 0 0'
            relative_pose = f'{translation} {roll:.9g} {pitch:.9g} {yaw:.9g}'
            parts.append(folded_link_xml(i, n_links, link_length, link_mass, radius, relative_pose, link_collisions))
            previous = rotation
    else:
        for i in range(1, n_links + 1):
            parts.append(link_xml(i, n_links, link_length, link_mass, radius, initial_axis, link_collisions))
    parts.append(force_constraint_plugin_xml(
        n_links,
        link_length,
        force_constraint,
        initial_axis,
        drone_model,
        drone_link,
        drone_offset,
        stiffness,
        damping,
        max_force,
    ))
    parts.append('''
  </model>
</sdf>
''')
    OUT_SDF.write_text(''.join(parts))

    OUT_CONFIG.write_text(f'''<?xml version="1.0" ?>
<model>
  <name>tether_anchor_chain</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>drone-cabo</name>
  </author>
  <description>Independent anchored tether chain. N={n_links}, L={total_length:.3f} m, rho={rho:.3f} kg/m.</description>
</model>
''')

    i_transverse, i_axial = cylinder_inertia(link_mass, radius, link_length)
    print(f'N_links={n_links}')
    print(f'comprimento_total={total_length:.6f} m')
    print(f'comprimento_por_link={link_length:.6f} m')
    print(f'rho_linear={rho:.6f} kg/m')
    print(f'massa_total={total_mass:.6f} kg')
    print(f'massa_por_link={link_mass:.6f} kg')
    print(f'raio={radius:.6f} m')
    print(f'inercia_link_transversal={i_transverse:.9g} kg.m^2')
    print(f'inercia_link_axial={i_axial:.9g} kg.m^2')
    print(f'inicializacao={initial_axis}')
    print(f'colisoes_links={link_collisions}')
    print(f'force_constraint={force_constraint}')
    if force_constraint:
        print(f'drone_model={drone_model}')
        print(f'drone_link={drone_link}')
        print(f'drone_offset={drone_offset}')
        print(f'K={stiffness:.6f} N/m')
        print(f'C={damping:.6f} N.s/m')
        print(f'forca_max={max_force:.6f} N')
    print(f'sdf={OUT_SDF}')


def main():
    parser = argparse.ArgumentParser(description='Generate independent anchored tether model.')
    parser.add_argument('--links', type=int, required=True)
    parser.add_argument('--length', type=float, default=2.50)
    parser.add_argument('--rho', type=float, default=0.06)
    parser.add_argument('--radius', type=float, default=0.003)
    parser.add_argument('--initial-axis', choices=('x', 'z', 'folded_ground'), default='z')
    parser.add_argument('--link-collisions', action='store_true')
    parser.add_argument('--force-constraint', action='store_true')
    parser.add_argument('--drone-model', default='x500_tether_attach_0')
    parser.add_argument('--drone-link', default='tether_attach_link')
    parser.add_argument('--drone-offset', default='0 0 0')
    parser.add_argument('--stiffness', type=float, default=20.0)
    parser.add_argument('--damping', type=float, default=4.0)
    parser.add_argument('--max-force', type=float, default=20.0)
    args = parser.parse_args()
    if args.links <= 0:
        raise SystemExit('links must be positive')
    if args.length <= 0.0:
        raise SystemExit('length must be positive')
    if args.rho <= 0.0:
        raise SystemExit('rho must be positive')
    if args.radius <= 0.0:
        raise SystemExit('radius must be positive')
    if args.stiffness <= 0.0:
        raise SystemExit('stiffness must be positive')
    if args.damping < 0.0:
        raise SystemExit('damping must be non-negative')
    if args.max_force <= 0.0:
        raise SystemExit('max-force must be positive')
    generate(
        args.links,
        args.length,
        args.rho,
        args.radius,
        args.initial_axis,
        args.link_collisions,
        args.force_constraint,
        args.drone_model,
        args.drone_link,
        args.drone_offset,
        args.stiffness,
        args.damping,
        args.max_force,
    )


if __name__ == '__main__':
    main()
