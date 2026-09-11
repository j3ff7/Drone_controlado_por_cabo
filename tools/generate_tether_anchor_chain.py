#!/usr/bin/env python3
import argparse
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain'

# A1: a ancora ideal presa ao world virou uma ground station estatica com um ponto
# de saida explicito. O endpoint pertence a estacao para que, mais adiante, a base
# fixa possa ser trocada por reel/TMS/UGV sem mexer no cabo nem na constraint.
GROUND_BASE_LINK = 'ground_station_base'
GROUND_EXIT_LINK = 'tether_exit_point'
GROUND_WORLD_JOINT = 'ground_station_world_fixed'
GROUND_EXIT_JOINT = 'tether_exit_fixed'

# A2: o tether_exit_point deixa de ser montado direto na base e passa a ser montado
# num tambor com junta revolute passiva. O endpoint fica no topo do tambor, a uma
# distancia igual ao raio acima do eixo, de modo que a pose nominal de A1 e mantida.
REEL_LINK = 'reel_link'
REEL_JOINT = 'reel_joint'

# A7: comprimento variavel por junta PRISMATICA na saida. O elo de payout fica entre a
# guia e o primeiro elo rigido; sua extensao `s` e o cabo efetivamente liberado.
PAYOUT_LINK = 'tether_payout_link'
PAYOUT_JOINT = 'tether_payout'

# Preenchido em generate(): guia (sem payout) ou elo de payout (com payout).
FIRST_LINK_PARENT = GROUND_EXIT_LINK
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


def coil_vectors(n_links, link_length):
    """Cabo folgado deitado num poligono regular fechado de N lados.

    A curva subjacente e sempre a MESMA circunferencia de perimetro `L = N * l`,
    qualquer que seja N: so muda a finura da poligonal que a aproxima. E essa
    propriedade que torna o sweep de discretizacao justo — comparar N diferentes
    partindo de formas iniciais diferentes misturaria condicao inicial com
    erro de discretizacao.

    Devolve os vetores de cada aresta, no plano horizontal.
    """
    if n_links < 3:
        raise SystemExit('initial-axis=coil exige pelo menos 3 elos')
    radius_c = link_length / (2.0 * math.sin(math.pi / n_links))
    vectors = []
    for index in range(n_links):
        theta_a = 2.0 * math.pi * index / n_links
        theta_b = 2.0 * math.pi * (index + 1) / n_links
        vectors.append((
            radius_c * (math.cos(theta_b) - math.cos(theta_a)),
            radius_c * (math.sin(theta_b) - math.sin(theta_a)),
            0.0,
        ))
    return vectors


def loop_vectors(n_links, link_length):
    """Cabo folgado pendurado num laco fechado no plano vertical x-z.

    Mesma ideia do `coil` — poligono regular de N lados sobre uma circunferencia de
    perimetro `L = N * l`, identica para qualquer N — mas no plano VERTICAL e saindo
    para baixo. O `coil` horizontal deixava os 2,5 m de cabo suspensos na mesma altura
    sem apoio: ele desabava de uma vez e a cadeia de ball joints divergia depois de
    algumas dezenas de segundos (assercao `BallJoint::updateRelativeTransform`). O laco
    vertical ja nasce pendurado, que e perto do equilibrio, e nao colapsa.
    """
    if n_links < 3:
        raise SystemExit('initial-axis=loop exige pelo menos 3 elos')
    radius_c = link_length / (2.0 * math.sin(math.pi / n_links))
    vectors = []
    for index in range(n_links):
        # Comeca no topo da circunferencia descendo: o centro fica abaixo da saida.
        theta_a = math.pi / 2.0 - 2.0 * math.pi * index / n_links
        theta_b = math.pi / 2.0 - 2.0 * math.pi * (index + 1) / n_links
        vectors.append((
            radius_c * (math.cos(theta_b) - math.cos(theta_a)),
            0.0,
            radius_c * (math.sin(theta_b) - math.sin(theta_a)),
        ))
    return vectors


def taut_vectors(n_links, link_length, target):
    """Cabo ESTICADO entre a guia e o attach do UAV, distribuido em N elos iguais.

    Resolve o arco circular em que uma poligonal de N cordas de comprimento exato
    `l = L/N` liga a origem (ponto de saida) ao alvo `target` (attach do UAV, relativo
    a saida). Para N cordas de angulo 2*beta cada, num circulo de raio R:

        l = 2*R*sin(beta)          corda de cada elo
        c = 2*R*sin(N*beta)        corda total, da saida ao alvo

    logo `c/l = sin(N*beta)/sin(beta)`, resolvido por bisseccao em (0, pi/N). O arco
    curva para baixo, aproximando a catenaria.

    Por que assim, e nao `coil`/`loop`: com os extremos praticamente coincidentes
    (0,08 m) e 2,5 m de cabo, qualquer forma inicial vira um laco folgado que colapsa,
    se auto-atravessa e derruba o DART (B1 rodada 1). Afastando o UAV, a geometria
    inicial deixa de ser um laco. A familia converge para o mesmo arco de comprimento
    `L` e corda `c` conforme N cresce, que e a propriedade de fairness necessaria.
    """
    if n_links < 2:
        raise SystemExit('initial-axis=taut exige pelo menos 2 elos')
    total_length = n_links * link_length
    chord = math.sqrt(sum(component * component for component in target))
    if chord <= 0.0:
        raise SystemExit('taut-target nao pode coincidir com o ponto de saida')
    if chord >= total_length:
        raise SystemExit(
            f'taut-target a {chord:.4f} m exige cabo mais longo que {total_length:.4f} m')

    ratio = chord / link_length
    low, high = 1e-12, math.pi / n_links - 1e-12
    for _ in range(200):
        beta = 0.5 * (low + high)
        value = math.sin(n_links * beta) / math.sin(beta)
        # sin(N*b)/sin(b) decresce de N ate 0 em (0, pi/N).
        if value > ratio:
            low = beta
        else:
            high = beta
    beta = 0.5 * (low + high)
    radius_c = link_length / (2.0 * math.sin(beta))

    # Base local: u ao longo da corda, d perpendicular apontando para baixo.
    unit = [component / chord for component in target]
    down = [0.0, 0.0, -1.0]
    projection = sum(down[i] * unit[i] for i in range(3))
    sag = [down[i] - projection * unit[i] for i in range(3)]
    sag_norm = math.sqrt(sum(component * component for component in sag))
    if sag_norm < 1e-9:      # corda vertical: qualquer perpendicular serve
        sag = [1.0, 0.0, 0.0]
        sag_norm = 1.0
    sag = [component / sag_norm for component in sag]

    def point(index):
        psi = -n_links * beta + 2.0 * index * beta
        local_u = radius_c * (math.sin(psi) - math.sin(-n_links * beta))
        local_d = radius_c * (math.cos(psi) - math.cos(-n_links * beta))
        return [unit[i] * local_u + sag[i] * local_d for i in range(3)]

    return [[point(i + 1)[k] - point(i)[k] for k in range(3)] for i in range(n_links)]



# --- UniversalJoint (teste pos-C) ---------------------------------------------
# Alternativa a ball joint com 2 DOF por conexao: dois eixos ortogonais entre si e
# ortogonais ao proprio elo, escritos no frame do elo FILHO (+x ao longo do cabo):
#
#   axis  = y local  -> flexao no plano x-z do elo
#   axis2 = z local  -> flexao no plano x-y do elo
#
# Diferente das revolutes alternadas de C1-C4 (descartadas por anisotropia), CADA
# conexao tem as duas direcoes de flexao, entao pequenas flexoes sao isotropicas.
# Nao ha DOF de torcao (nenhum eixo em +x). Singularidade conhecida da universal:
# flexao de 90 graus em torno de `axis`, onde `axis2` se alinha com o elo.
# `universal_roll` gira o par de eixos em torno do +x local, sem tocar em geometria,
# massa ou pose -- serve para testar se a resposta depende da orientacao dos eixos.
JOINT_LIMIT = 1e16


def universal_axes(universal_roll):
    roll = math.radians(universal_roll)
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    return (0.0, cos_r, sin_r), (0.0, -sin_r, cos_r)


def axis_block(tag, child_name, vector):
    x, y, z = vector
    return f'''
      <{tag}>
        <xyz expressed_in="{child_name}">{x:.9g} {y:.9g} {z:.9g}</xyz>
        <limit>
          <lower>-{JOINT_LIMIT:.9g}</lower>
          <upper>{JOINT_LIMIT:.9g}</upper>
        </limit>
        <dynamics>
          <damping>0</damping>
          <friction>0</friction>
        </dynamics>
      </{tag}>'''


def tether_joint_xml(index, parent_frame, child_name, joint_pose,
                     joint_type='ball', universal_roll=0.0):
    if joint_type == 'ball':
        return f'''
    <joint name="tether_joint_{index}" type="ball">
      <pose relative_to="{parent_frame}">{joint_pose}</pose>
      <parent>{parent_frame}</parent>
      <child>{child_name}</child>
    </joint>'''
    axis1, axis2 = universal_axes(universal_roll)
    return f'''
    <joint name="tether_joint_{index}" type="universal">
      <pose relative_to="{parent_frame}">{joint_pose}</pose>
      <parent>{parent_frame}</parent>
      <child>{child_name}</child>{axis_block('axis', child_name, axis1)}{axis_block('axis2', child_name, axis2)}
    </joint>'''


def folded_link_xml(index, n_links, link_length, mass, radius, relative_pose, link_collisions,
                    joint_type='ball', universal_roll=0.0):
    name = f'tether_link_{index}'
    parent_frame = FIRST_LINK_PARENT if index == 1 else f'tether_link_{index - 1}'
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
    </link>''' + tether_joint_xml(
        index, parent_frame, name, joint_pose, joint_type, universal_roll)


def link_xml(index, n_links, link_length, mass, radius, axis, link_collisions,
             joint_type='ball', universal_roll=0.0):
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

    parent_frame = FIRST_LINK_PARENT if index == 1 else f'tether_link_{index - 1}'
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
    </link>''' + tether_joint_xml(
        index, parent_frame, name, joint_pose, joint_type, universal_roll)


def force_constraint_plugin_xml(
    payout_enabled,
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
    # O ponto de conexao fica na ponta livre do ultimo elo. `coil` usa segmentos
    # orientados no +x local, como 'x' e 'folded_ground'; so 'z' desce em -z.
    x_oriented = initial_axis in ('x', 'folded_ground', 'coil', 'loop', 'taut')
    tether_offset = (f'{link_length:.9g} 0 0' if x_oriented
                     else f'0 0 {-link_length:.9g}')
    ground_world_joint = GROUND_WORLD_JOINT
    ground_exit_link = GROUND_EXIT_LINK
    ground_reel_joint = REEL_JOINT
    ground_payout_link = PAYOUT_LINK if payout_enabled else ''
    return f'''
    <plugin filename="libTetherForceConstraint.so" name="drone_cabo::TetherForceConstraint">
      <drone_model>{drone_model}</drone_model>
      <drone_link>{drone_link}</drone_link>
      <tether_model>tether_anchor_chain</tether_model>
      <tether_link>tether_link_{n_links}</tether_link>
      <!-- Reservado: o plugin NAO le este campo. A instrumentacao direta do
           wrench neste joint via EnableTransmittedWrenchCheck derruba o backend
           DART (BallJoint::updateRelativeTransform). -->
      <anchor_joint>{ground_world_joint}</anchor_joint>
      <exit_link>{ground_exit_link}</exit_link>
      <reel_joint>{ground_reel_joint}</reel_joint>
      <tether_link_count>{n_links}</tether_link_count>
      <exit_segment_link>tether_link_1</exit_segment_link>
      <payout_link>{ground_payout_link}</payout_link>
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
    taut_target,
    force_constraint,
    drone_model,
    drone_link,
    drone_offset,
    stiffness,
    damping,
    max_force,
    station_collision,
    reel,
    exit_on_reel,
    reel_radius,
    reel_width,
    reel_mass,
    reel_damping,
    reel_friction,
    reel_actuator,
    reel_tau_max,
    reel_omega_max,
    reel_ramp_rate,
    payout,
    payout_min,
    payout_max,
    payout_r_eff,
    payout_rate_max,
    payout_damping_arg,
    payout_drive,
    payout_kp,
    payout_kd,
    payout_force_max,
    payout_mass_arg,
    joint_type='ball',
    universal_roll=0.0,
):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    # --- Geometria do suporte do TMS ----------------------------------------
    # A origem do modelo fica na FACE INFERIOR da base, de modo que spawnar em z=0
    # apoia a estacao no solo. Layout, de baixo para cima:
    #   base 0,30 x 0,30 x base_thickness  ->  duas hastes de post_height
    #   -> eixo coaxial no topo das hastes ->  tambor montado ENTRE as hastes.
    base_size = 0.30
    base_thickness = 0.02
    post_height = 0.10
    post_thickness = 0.02
    axle_radius = 0.008
    axle_z = base_thickness + post_height          # altura do eixo do tambor
    post_offset_y = 0.5 * reel_width + 0.5 * post_thickness
    axle_length = reel_width + 2.0 * post_thickness + 0.02
    # A estacao fica na origem do mundo, exatamente onde o X500 e spawnado. Com
    # collision, a estrutura interpenetra o trem de pouso, empurra o UAV para fora e
    # para baixo, estica o cabo ate saturar a constraint e a cadeia de ball joints
    # diverge: o DART aborta em BallJoint::updateRelativeTransform com
    # `math::verifyTransform(mT)`. Nesta baseline a estacao e visual: nada precisa
    # colidir com ela (as colisoes dos segmentos tambem estao desabilitadas).
    # Reabilitar so faz sentido junto com afastar a estacao do ponto de decolagem.
    # --- Reel passivo --------------------------------------------------------
    # Eixo horizontal (Y) sustentado pelas duas hastes, coaxial com o tambor. O cabo
    # deixa o tambor pela tangente superior, um raio acima do eixo. Uma forca
    # horizontal na saida gera torque `reel_radius * Fx`; uma forca vertical passa
    # pela linha do raio e nao gera torque, que e o comportamento de um carretel.
    # A3: a guia de saida e SEMPRE filha da base, nunca do tambor. Prender a raiz do
    # cabo na borda do tambor (A2) fazia dele uma manivela: qualquer rotacao deslocava
    # o ponto de saida em 2r. Num carretel real o cabo sai por uma guia fixa e a rotacao
    # troca o comprimento enrolado, o que so faz sentido quando houver payout.
    reel_block = ''
    exit_parent = GROUND_BASE_LINK
    exit_relative_pose = f'0 0 {axle_z + reel_radius:.9g} 0 0 0'
    if reel:
        i_axial = 0.5 * reel_mass * reel_radius * reel_radius
        i_transverse = reel_mass * (3.0 * reel_radius * reel_radius
                                    + reel_width * reel_width) / 12.0
        reel_block = f"""    <link name="{REEL_LINK}">
      <pose relative_to="{GROUND_BASE_LINK}">0 0 {axle_z:.9g} 0 0 0</pose>
      <inertial>
        <mass>{reel_mass:.9g}</mass>
{inertia_xml(i_transverse, i_axial, i_transverse, '        ')}
      </inertial>
      <visual name="{REEL_LINK}_visual">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{reel_radius:.9g}</radius>
            <length>{reel_width:.9g}</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.75 0.55 0.10 1.0</diffuse>
          <ambient>0.75 0.55 0.10 1.0</ambient>
        </material>
      </visual>
    </link>
    <joint name="{REEL_JOINT}" type="revolute">
      <pose relative_to="{REEL_LINK}">0 0 0 0 0 0</pose>
      <parent>{GROUND_BASE_LINK}</parent>
      <child>{REEL_LINK}</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1e16</lower>
          <upper>1e16</upper>
        </limit>
        <dynamics>
          <damping>{reel_damping:.9g}</damping>
          <friction>{reel_friction:.9g}</friction>
        </dynamics>
      </axis>
    </joint>
"""

    if reel and exit_on_reel:
        # Montagem de A2, mantida so para reproducao/controle experimental: a raiz do
        # cabo presa a BORDA do tambor. Nessa forma o tambor vira manivela e qualquer
        # rotacao desloca o ponto de saida em ate 2r.
        exit_parent = REEL_LINK
        exit_relative_pose = f'0 0 {reel_radius:.9g} 0 0 0'
    if not reel:
        exit_relative_pose = f'0 0 {axle_z:.9g} 0 0 0'

    # --- Payout (A7) ---------------------------------------------------------
    # Junta prismatica ao longo do +x local da guia, que e a direcao em que o primeiro
    # segmento ja nasce: em s=0 a geometria e identica a de A6. A extensao `s` e o cabo
    # liberado, e L = L_total + s. Escolhida entre as alternativas por preservar
    # continuidade de posicao e velocidade, nao mudar topologia em runtime, nao criar
    # nem destruir corpos e nao mexer nas ball joints, que ja se mostraram fragis.
    global FIRST_LINK_PARENT
    # O elo de payout e um trecho de cabo, e precisa ter massa de cabo. Com 1 g entre a
    # guia e os 150 g da cadeia, a razao de massa de 150:1 tornou a prismatica mal
    # condicionada e o comprimento divergiu (1e117 m) mesmo com comando zero. A massa
    # e a inercia agora saem da mesma densidade linear dos demais segmentos.
    payout_span = 0.5 * (payout_min + payout_max) if payout_max > payout_min else link_length
    payout_mass = payout_mass_arg if payout_mass_arg > 0 else rho * payout_span
    payout_i_transverse, payout_i_axial = cylinder_inertia(payout_mass, radius, payout_span)
    payout_damping = payout_damping_arg
    payout_block = ''
    FIRST_LINK_PARENT = GROUND_EXIT_LINK
    if payout:
        FIRST_LINK_PARENT = PAYOUT_LINK
        payout_block = f"""    <link name="{PAYOUT_LINK}">
      <pose relative_to="{GROUND_EXIT_LINK}">0 0 0 0 0 0</pose>
      <inertial>
        <pose>{0.5 * payout_span:.9g} 0 0 0 0 0</pose>
        <mass>{payout_mass:.9g}</mass>
{inertia_xml(payout_i_transverse, payout_i_axial, payout_i_transverse, '        ')}
      </inertial>
      <visual name="{PAYOUT_LINK}_visual">
        <pose>0 0 0 0 1.57079632679 0</pose>
        <geometry>
          <cylinder>
            <radius>{0.6 * radius:.9g}</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.90 0.45 0.05 1.0</diffuse>
          <ambient>0.90 0.45 0.05 1.0</ambient>
        </material>
      </visual>
    </link>
    <joint name="{PAYOUT_JOINT}" type="prismatic">
      <pose relative_to="{PAYOUT_LINK}">0 0 0 0 0 0</pose>
      <parent>{GROUND_EXIT_LINK}</parent>
      <child>{PAYOUT_LINK}</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>{payout_min:.9g}</lower>
          <upper>{payout_max:.9g}</upper>
        </limit>
        <dynamics>
          <damping>{payout_damping:.9g}</damping>
          <friction>0</friction>
        </dynamics>
      </axis>
    </joint>
"""

    base_collision = ''
    if station_collision:
        base_collision = f"""
      <collision name="{GROUND_BASE_LINK}_collision">
        <pose>0 0 {0.5 * base_thickness:.9g} 0 0 0</pose>
        <geometry>
          <box>
            <size>{base_size:.9g} {base_size:.9g} {base_thickness:.9g}</size>
          </box>
        </geometry>
      </collision>"""
    link_length = total_length / n_links
    total_mass = rho * total_length
    link_mass = total_mass / n_links
    if initial_axis == 'folded_ground' and n_links != 5:
        raise SystemExit('initial-axis=folded_ground is defined for the 5-link minimum experiment')
    parts = [f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="tether_anchor_chain">
    <static>false</static>
    <link name="{GROUND_BASE_LINK}">
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
      <visual name="{GROUND_BASE_LINK}_plate">
        <pose>0 0 {0.5 * base_thickness:.9g} 0 0 0</pose>
        <geometry>
          <box>
            <size>{base_size:.9g} {base_size:.9g} {base_thickness:.9g}</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.20 0.22 0.26 1.0</diffuse>
          <ambient>0.20 0.22 0.26 1.0</ambient>
        </material>
      </visual>
      <visual name="{GROUND_BASE_LINK}_post_left">
        <pose>0 {post_offset_y:.9g} {base_thickness + 0.5 * post_height:.9g} 0 0 0</pose>
        <geometry>
          <box>
            <size>{post_thickness:.9g} {post_thickness:.9g} {post_height:.9g}</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.28 0.30 0.34 1.0</diffuse>
          <ambient>0.28 0.30 0.34 1.0</ambient>
        </material>
      </visual>
      <visual name="{GROUND_BASE_LINK}_post_right">
        <pose>0 {-post_offset_y:.9g} {base_thickness + 0.5 * post_height:.9g} 0 0 0</pose>
        <geometry>
          <box>
            <size>{post_thickness:.9g} {post_thickness:.9g} {post_height:.9g}</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.28 0.30 0.34 1.0</diffuse>
          <ambient>0.28 0.30 0.34 1.0</ambient>
        </material>
      </visual>
      <visual name="{GROUND_BASE_LINK}_axle">
        <pose>0 0 {axle_z:.9g} 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{axle_radius:.9g}</radius>
            <length>{axle_length:.9g}</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.55 0.57 0.60 1.0</diffuse>
          <ambient>0.55 0.57 0.60 1.0</ambient>
        </material>
      </visual>
{base_collision}
    </link>
    <joint name="{GROUND_WORLD_JOINT}" type="fixed">
      <parent>world</parent>
      <child>{GROUND_BASE_LINK}</child>
    </joint>
{reel_block}    <link name="{GROUND_EXIT_LINK}">
      <pose relative_to="{exit_parent}">{exit_relative_pose}</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-07</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-07</iyy>
          <iyz>0</iyz>
          <izz>1e-07</izz>
        </inertia>
      </inertial>
      <visual name="{GROUND_EXIT_LINK}_visual">
        <geometry>
          <sphere>
            <radius>0.018</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>0.05 0.15 0.9 1.0</diffuse>
          <ambient>0.05 0.15 0.9 1.0</ambient>
        </material>
      </visual>
    </link>
    <joint name="{GROUND_EXIT_JOINT}" type="fixed">
      <parent>{exit_parent}</parent>
      <child>{GROUND_EXIT_LINK}</child>
    </joint>
{payout_block}''']
    if initial_axis in ('coil', 'loop', 'taut'):
        if initial_axis == 'taut':
            vectors = taut_vectors(n_links, link_length, taut_target)
        else:
            shape = coil_vectors if initial_axis == 'coil' else loop_vectors
            vectors = shape(n_links, link_length)
        rotations = [orientation_from_x_axis(v) for v in vectors]
        previous = rpy_to_matrix(0.0, 0.0, 0.0)
        for i, rotation in enumerate(rotations, start=1):
            relative = matmul(transpose(previous), rotation)
            roll, pitch, yaw = matrix_to_rpy(relative)
            translation = '0 0 0' if i == 1 else f'{link_length:.9g} 0 0'
            relative_pose = f'{translation} {roll:.9g} {pitch:.9g} {yaw:.9g}'
            parts.append(folded_link_xml(
                i, n_links, link_length, link_mass, radius, relative_pose, link_collisions,
                joint_type, universal_roll))
            previous = rotation
    elif initial_axis == 'folded_ground':
        target_z = 0.085
        rotations = [orientation_from_x_axis(v) for v in folded_ground_vectors(link_length, target_z)]
        identity = rpy_to_matrix(0.0, 0.0, 0.0)
        previous = identity
        for i, rotation in enumerate(rotations, start=1):
            relative = matmul(transpose(previous), rotation)
            roll, pitch, yaw = matrix_to_rpy(relative)
            translation = '0 0 0' if i == 1 else f'{link_length:.9g} 0 0'
            relative_pose = f'{translation} {roll:.9g} {pitch:.9g} {yaw:.9g}'
            parts.append(folded_link_xml(i, n_links, link_length, link_mass, radius, relative_pose,
                                         link_collisions, joint_type, universal_roll))
            previous = rotation
    else:
        for i in range(1, n_links + 1):
            parts.append(link_xml(i, n_links, link_length, link_mass, radius, initial_axis,
                                  link_collisions, joint_type, universal_roll))
    parts.append(force_constraint_plugin_xml(
        bool(reel and payout),
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
    # A6: atuador do reel. Sem comando publicado ele aplica torque zero, entao a
    # dinamica fica identica a de A3/A5. O reel continua desacoplado do comprimento
    # do cabo: girar o tambor nao libera nem recolhe tether. Isso e objeto de A7.
    if reel and reel_actuator:
        parts.append(f'''
    <plugin filename="libReelActuator.so" name="drone_cabo::ReelActuator">
      <reel_model>tether_anchor_chain</reel_model>
      <reel_joint>{REEL_JOINT}</reel_joint>
      <tau_max>{reel_tau_max:.9g}</tau_max>
      <omega_max>{reel_omega_max:.9g}</omega_max>
      <ramp_rate>{reel_ramp_rate:.9g}</ramp_rate>
      <command_topic>/cabo/tms/reel_cmd</command_topic>
    </plugin>''')
    if reel and payout:
        parts.append(f'''
    <plugin filename="libTetherPayout.so" name="drone_cabo::TetherPayout">
      <model_name>tether_anchor_chain</model_name>
      <reel_joint>{REEL_JOINT}</reel_joint>
      <payout_joint>{PAYOUT_JOINT}</payout_joint>
      <r_eff>{payout_r_eff:.9g}</r_eff>
      <length_nominal>{total_length:.9g}</length_nominal>
      <s_min>{payout_min:.9g}</s_min>
      <s_max>{payout_max:.9g}</s_max>
      <rate_max>{payout_rate_max:.9g}</rate_max>
      <drive>{payout_drive}</drive>
      <kp>{payout_kp:.9g}</kp>
      <kd>{payout_kd:.9g}</kd>
      <force_max>{payout_force_max:.9g}</force_max>
    </plugin>''')
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
  <description>Static ground station + tether_exit_point + independent tether chain. N={n_links}, L={total_length:.3f} m, rho={rho:.3f} kg/m.</description>
</model>
''')

    i_transverse, i_axial = cylinder_inertia(link_mass, radius, link_length)
    print(f'ground_station_base={GROUND_BASE_LINK} (fixed ao world por {GROUND_WORLD_JOINT})')
    print(f'tether_exit_point={GROUND_EXIT_LINK} (fixed a base por {GROUND_EXIT_JOINT})')
    print(f'estacao_base={base_size:.3f} x {base_size:.3f} x {base_thickness:.3f} m (placa quadrada)')
    print(f'estacao_hastes=2 x ({post_thickness:.3f} x {post_thickness:.3f} x {post_height:.3f} m) em y=+-{post_offset_y:.3f} m')
    print(f'estacao_eixo=z={axle_z:.3f} m, raio={axle_radius:.3f} m, comprimento={axle_length:.3f} m')
    print(f'estacao_collision={station_collision}')
    print(f'reel={reel}')
    if reel:
        print(f'reel_joint={REEL_JOINT} (revolute, eixo Y, {GROUND_BASE_LINK} -> {REEL_LINK})')
        print(f'reel_raio={reel_radius:.6f} m')
        print(f'reel_largura={reel_width:.6f} m')
        print(f'reel_massa={reel_mass:.6f} kg')
        print(f'reel_inercia_axial={0.5 * reel_mass * reel_radius ** 2:.9g} kg.m^2')
        print(f'reel_damping={reel_damping:.9g} N.m.s/rad')
        print(f'reel_friction={reel_friction:.9g} N.m')
        print(f'reel_folga_ao_solo={axle_z - reel_radius:.3f} m')
        print(f'reel_atuador={reel_actuator}')
        if reel_actuator:
            print(f'reel_tau_max={reel_tau_max:.9g} N.m')
            print(f'reel_omega_max={reel_omega_max:.9g} rad/s')
            print(f'reel_ramp_rate={reel_ramp_rate:.9g} N.m/s')
            print('reel_comando=/cabo/tms/reel_cmd (torque no eixo, pos-reducao)')
            print(f'payout={payout}')
            if payout:
                print(f'payout_junta={PAYOUT_JOINT} (prismatic, eixo +x da guia)')
                print(f'payout_R_eff={payout_r_eff:.9g} m')
                print(f'payout_s=[{payout_min:.9g}, {payout_max:.9g}] m')
                print(f'payout_L=[{total_length + payout_min:.9g}, {total_length + payout_max:.9g}] m')
                print(f'payout_rate_max={payout_rate_max:.9g} m/s')
                print(f'payout_massa={payout_mass:.9g} kg (rho * {payout_span:.9g} m)')
                print(f'payout_damping={payout_damping:.9g} N.s/m')
                print(f'payout_drive={payout_drive} (kp={payout_kp:.9g} N/m, kd={payout_kd:.9g} N.s/m, f_max={payout_force_max:.9g} N)')
                print('payout_sinal=omega>0 libera cabo, omega<0 recolhe')
            else:
                print('reel_acopla_comprimento=NAO')
    print('tether_exit_point=' + ('borda do tambor (montagem A2)' if (reel and exit_on_reel)
          else 'guia FIXA em ground_station_base (nao gira com o tambor)'))
    print(f'tether_exit_point_z={axle_z + reel_radius if reel else axle_z:.3f} m acima da base')
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
    print(f'joint_type={joint_type}')
    if joint_type == 'universal':
        axis1, axis2 = universal_axes(universal_roll)
        print('universal_axis1=(%s) universal_axis2=(%s) (frame do elo filho), roll=%.6g graus'
              % (','.join(f'{c:.3g}' for c in axis1), ','.join(f'{c:.3g}' for c in axis2), universal_roll))
    if initial_axis == 'taut':
        import math as _m
        _c = _m.sqrt(sum(v * v for v in taut_target))
        print(f'taut_alvo={taut_target} (relativo a saida)')
        print(f'taut_corda={_c:.6f} m, folga={(total_length - _c) / total_length * 100:.2f}%')
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
    parser.add_argument('--initial-axis',
                        choices=('x', 'z', 'folded_ground', 'coil', 'loop', 'taut'), default='z')
    parser.add_argument('--link-collisions', action='store_true')
    parser.add_argument('--taut-target', default='2.38 0 -0.083',
                        help='attach do UAV relativo ao tether_exit_point [m], '
                             'usado por --initial-axis taut')
    parser.add_argument('--force-constraint', action='store_true')
    parser.add_argument('--drone-model', default='x500_tether_attach_0')
    parser.add_argument('--drone-link', default='tether_attach_link')
    parser.add_argument('--drone-offset', default='0 0 0')
    parser.add_argument('--stiffness', type=float, default=20.0)
    parser.add_argument('--damping', type=float, default=4.0)
    parser.add_argument('--max-force', type=float, default=20.0)
    parser.add_argument('--joint-type', choices=('ball', 'universal'), default='ball',
                        help='ball: baseline (3 DOF). universal: 2 DOF de flexao por conexao, '
                             'eixos y e z locais, sem torcao')
    parser.add_argument('--universal-roll', type=float, default=0.0,
                        help='gira o par de eixos da universal em torno do +x local [graus]')
    parser.add_argument('--station-collision', action='store_true',
                        help='da collision ao plinto da estacao; exige afastar a '
                             'estacao do ponto de decolagem do UAV')
    parser.add_argument('--no-reel', dest='reel', action='store_false',
                        help='monta o tether_exit_point direto na base (geometria A1)')
    parser.add_argument('--exit-on-reel', action='store_true',
                        help='prende a raiz do cabo na borda do tambor (montagem de A2); '
                             'por padrao a saida e uma guia fixa na estacao')
    parser.add_argument('--reel-radius', type=float, default=0.07)
    parser.add_argument('--reel-width', type=float, default=0.16)
    parser.add_argument('--reel-mass', type=float, default=0.2)
    parser.add_argument('--reel-damping', type=float, default=5e-3)
    parser.add_argument('--reel-friction', type=float, default=0.0)
    parser.add_argument('--no-reel-actuator', dest='reel_actuator', action='store_false',
                        help='gera sem o atuador do reel (geometria de A3/A5)')
    parser.add_argument('--reel-tau-max', type=float, default=0.05)
    parser.add_argument('--reel-omega-max', type=float, default=12.0)
    parser.add_argument('--reel-ramp-rate', type=float, default=0.05)
    # A7 REPROVADA: a junta prismatica nao e representacao valida de payout/retraction
    # e nao deve ser usada como solucao arquitetural. Fica desligada por padrao e so
    # existe para reproduzir o experimento historico.
    parser.add_argument('--payout', dest='payout', action='store_true',
                        help='ABORDAGEM DESCARTADA (A7): junta prismatica como payout. '
                             'Mantida apenas para reproduzir o experimento que falhou.')
    parser.add_argument('--payout-min', type=float, default=0.0,
                        help='extensao minima da prismatica [m]; L_min = L + este valor')
    parser.add_argument('--payout-max', type=float, default=1.0,
                        help='extensao maxima da prismatica [m]; L_max = L + este valor')
    parser.add_argument('--payout-r-eff', type=float, default=0.07,
                        help='raio efetivo que mapeia theta do reel em comprimento [m]')
    parser.add_argument('--payout-rate-max', type=float, default=0.5,
                        help='guarda de velocidade de payout [m/s]')
    parser.add_argument('--payout-damping', type=float, default=1.0,
                        help='amortecimento da prismatica [N.s/m]; ajuda o condicionamento')
    parser.add_argument('--payout-drive', choices=('force', 'velocity', 'none'), default='force',
                        help='force: PD por SetForce (padrao); velocity: SetVelocity '
                             '(DIVERGE); none: prismatica livre, so diagnostico')
    parser.add_argument('--payout-kp', type=float, default=200.0)
    parser.add_argument('--payout-kd', type=float, default=20.0)
    parser.add_argument('--payout-force-max', type=float, default=20.0)
    parser.add_argument('--payout-mass', type=float, default=0.0,
                        help='massa do elo de payout [kg]; 0 usa rho*span')
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
        tuple(float(v) for v in args.taut_target.split()),
        args.force_constraint,
        args.drone_model,
        args.drone_link,
        args.drone_offset,
        args.stiffness,
        args.damping,
        args.max_force,
        args.station_collision,
        args.reel,
        args.exit_on_reel,
        args.reel_radius,
        args.reel_width,
        args.reel_mass,
        args.reel_damping,
        args.reel_friction,
        args.reel_actuator,
        args.reel_tau_max,
        args.reel_omega_max,
        args.reel_ramp_rate,
        args.payout,
        args.payout_min,
        args.payout_max,
        args.payout_r_eff,
        args.payout_rate_max,
        args.payout_damping,
        args.payout_drive,
        args.payout_kp,
        args.payout_kd,
        args.payout_force_max,
        args.payout_mass,
        joint_type=args.joint_type,
        universal_roll=args.universal_roll,
    )


if __name__ == '__main__':
    main()
