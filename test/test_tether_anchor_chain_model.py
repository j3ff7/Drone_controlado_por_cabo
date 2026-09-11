import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'
GENERATOR = ROOT / 'tools' / 'generate_tether_anchor_chain.py'


def parse_model():
    return ET.parse(MODEL).getroot().find('model')


def test_generator_recreates_valid_anchored_force_baseline():
    result = subprocess.run(
        [
            str(GENERATOR),
            '--links', '5',
            '--length', '2.50',
            '--rho', '0.06',
            '--radius', '0.003',
            '--initial-axis', 'folded_ground',
            '--force-constraint',
            '--stiffness', '5',
            '--damping', '0.5',
            '--max-force', '3',
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert 'N_links=5' in result.stdout
    assert 'comprimento_total=2.500000 m' in result.stdout
    assert 'massa_total=0.150000 kg' in result.stdout
    assert 'force_constraint=True' in result.stdout


def test_anchor_chain_topology_and_parameters():
    model = parse_model()
    links = {link.attrib['name']: link for link in model.findall('link')}
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    assert model.attrib['name'] == 'tether_anchor_chain'
    # A1: a ancora ideal virou ground station estatica + ponto de saida explicito.
    assert set(links) - {'tether_payout_link'} == {
        'ground_station_base',
        'reel_link',
        'tether_exit_point',
        'tether_link_1',
        'tether_link_2',
        'tether_link_3',
        'tether_link_4',
        'tether_link_5',
    }
    assert 'anchor_link' not in links
    assert len(joints) in (8, 9)   # 9 com o payout de A7 gerado
    assert 'anchor_world_fixed' not in joints

    for index in range(1, 6):
        joint = joints[f'tether_joint_{index}']
        assert joint.attrib['type'] == 'ball'
        assert joint.findtext('child') == f'tether_link_{index}'

    link_masses = [
        float(links[f'tether_link_{index}'].find('inertial/mass').text)
        for index in range(1, 6)
    ]
    assert all(math.isclose(mass, 0.03) for mass in link_masses)
    assert math.isclose(sum(link_masses), 0.15)


def test_force_constraint_configuration():
    plugin = parse_model().find('plugin')

    assert plugin.attrib['name'] == 'drone_cabo::TetherForceConstraint'
    assert plugin.attrib['filename'] == 'libTetherForceConstraint.so'
    # A0.3: o endpoint do UAV e o link fisico, nao mais base_link + offset virtual.
    assert plugin.findtext('drone_model') == 'x500_tether_attach_0'
    assert plugin.findtext('drone_link') == 'tether_attach_link'
    assert plugin.findtext('drone_offset') == '0 0 0'
    assert plugin.findtext('tether_model') == 'tether_anchor_chain'
    assert plugin.findtext('tether_link') == 'tether_link_5'
    assert plugin.findtext('anchor_joint') == 'ground_station_world_fixed'
    assert plugin.findtext('exit_link') == 'tether_exit_point'
    assert plugin.findtext('tether_offset') == '0.5 0 0'
    assert plugin.findtext('stiffness') == '5'
    assert plugin.findtext('damping') == '0.5'
    assert plugin.findtext('max_force') == '3'


def test_old_virtual_endpoint_is_no_longer_the_active_interface():
    plugin = parse_model().find('plugin')

    assert plugin.findtext('drone_link') != 'base_link'
    assert plugin.findtext('drone_offset').split() == ['0', '0', '0']


def test_ground_station_is_rigidly_fixed_to_the_world():
    model = parse_model()
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    joint = joints['ground_station_world_fixed']
    assert joint.attrib['type'] == 'fixed'
    assert joint.findtext('parent') == 'world'
    assert joint.findtext('child') == 'ground_station_base'


def parent_chain(joints, link):
    """Sobe a arvore de juntas ate a raiz, devolvendo os pais na ordem encontrada."""
    by_child = {joint.findtext('child'): joint for joint in joints.values()}
    chain, current = [], link
    while current in by_child:
        current = by_child[current].findtext('parent')
        chain.append(current)
    return chain


def compose_pose_to_model(links, name):
    """Compoe a posicao do link ate a origem do modelo seguindo `relative_to`.

    Só translacoes: todos os elos desta cadeia terrestre tem rotacao nula.
    """
    position = [0.0, 0.0, 0.0]
    current = name
    while current is not None:
        pose = links[current].find('pose')
        values = [float(v) for v in pose.text.split()]
        assert values[3:] == [0.0, 0.0, 0.0], f'{current} introduz rotacao'
        position = [position[i] + values[i] for i in range(3)]
        current = pose.attrib.get('relative_to')
    return position


def test_exit_point_belongs_to_the_station_and_feeds_the_tether():
    model = parse_model()
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    # world -> ground_station_base -> (revolute) reel_link -> tether_exit_point -> cabo
    exit_joint = joints['tether_exit_fixed']
    assert exit_joint.attrib['type'] == 'fixed'
    assert exit_joint.findtext('child') == 'tether_exit_point'

    # A3: a guia e filha direta da base. O tambor deixou de ficar no caminho.
    assert parent_chain(joints, 'tether_exit_point') == ['ground_station_base', 'world']
    # Com ou sem o payout de A7, o cabo tem de chegar a guia e dela ao mundo.
    chain = parent_chain(joints, 'tether_link_1')
    assert chain[-3:] == ['tether_exit_point', 'ground_station_base', 'world']


def test_model_origin_sits_on_the_base_underside():
    links = {link.attrib['name']: link for link in parse_model().findall('link')}
    base = links['ground_station_base']
    plate = base.find("visual[@name='ground_station_base_plate']")

    # Origem do modelo na face inferior da placa: spawnar em z=0 apoia a estacao
    # no solo, sem o antigo deslocamento magico de 0,035 m.
    assert [float(v) for v in base.findtext('pose').split()] == [0.0] * 6
    thickness = float(plate.find('geometry/box/size').text.split()[2])
    assert math.isclose(float(plate.findtext('pose').split()[2]), thickness / 2.0)


def test_exit_point_is_raised_by_the_real_support_height():
    links = {link.attrib['name']: link for link in parse_model().findall('link')}
    reel = links['reel_link']
    radius = float(reel.find('visual/geometry/cylinder/radius').text)
    axle_z = float(reel.findtext('pose').split()[2])

    # O suporte fisico tem altura, entao o ponto de saida deixou de coincidir com a
    # ancora ideal de A0.3/A1: ele agora fica na tangente superior do tambor.
    exit_z = compose_pose_to_model(links, 'tether_exit_point')[2]
    assert math.isclose(exit_z, axle_z + radius)
    assert exit_z > 0.0


def test_exit_point_adds_no_collision_body():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}

    # O cabo sai por este ponto; ele nao deve introduzir contato proprio.
    assert links['tether_exit_point'].find('collision') is None


def test_station_support_is_a_square_plate_with_two_vertical_posts_and_an_axle():
    base = {l.attrib['name']: l for l in parse_model().findall('link')}['ground_station_base']
    visuals = {v.attrib['name']: v for v in base.findall('visual')}

    plate = [float(v) for v in visuals['ground_station_base_plate']
             .find('geometry/box/size').text.split()]
    assert math.isclose(plate[0], 0.30) and math.isclose(plate[1], 0.30)

    posts = [visuals['ground_station_base_post_left'], visuals['ground_station_base_post_right']]
    for post in posts:
        size = [float(v) for v in post.find('geometry/box/size').text.split()]
        assert math.isclose(size[2], 0.10), 'haste deve ter 10 cm de altura'

    ys = sorted(float(p.findtext('pose').split()[1]) for p in posts)
    assert math.isclose(ys[0], -ys[1]), 'uma haste de cada lado do tambor'

    axle = visuals['ground_station_base_axle']
    assert axle.find('geometry/cylinder') is not None


def test_posts_top_out_exactly_at_the_axle_height():
    model = parse_model()
    base = {l.attrib['name']: l for l in model.findall('link')}['ground_station_base']
    reel = {l.attrib['name']: l for l in model.findall('link')}['reel_link']
    post = base.find("visual[@name='ground_station_base_post_left']")

    size = [float(v) for v in post.find('geometry/box/size').text.split()]
    centre_z = float(post.findtext('pose').split()[2])
    axle_z = float(reel.findtext('pose').split()[2])

    assert math.isclose(centre_z + size[2] / 2.0, axle_z)


def test_axle_is_coaxial_with_the_drum_and_spans_past_both_posts():
    model = parse_model()
    base = {l.attrib['name']: l for l in model.findall('link')}['ground_station_base']
    reel = {l.attrib['name']: l for l in model.findall('link')}['reel_link']

    axle = base.find("visual[@name='ground_station_base_axle']")
    axle_pose = [float(v) for v in axle.findtext('pose').split()]
    reel_pose = [float(v) for v in reel.findtext('pose').split()]
    drum_pose = [float(v) for v in reel.find('visual').findtext('pose').split()]

    # Mesma altura, mesma linha, e ambos girados 90 deg em torno de X (eixo ao longo de Y).
    assert math.isclose(axle_pose[2], reel_pose[2])
    assert axle_pose[0] == 0.0 and axle_pose[1] == 0.0
    assert math.isclose(axle_pose[3], drum_pose[3])

    axle_length = float(axle.find('geometry/cylinder/length').text)
    drum_width = float(reel.find('visual/geometry/cylinder/length').text)
    assert axle_length > drum_width, 'o eixo deve atravessar as hastes'


def test_drum_sits_between_the_posts_without_being_embedded():
    model = parse_model()
    links = {l.attrib['name']: l for l in model.findall('link')}
    base, reel = links['ground_station_base'], links['reel_link']

    drum_half_width = float(reel.find('visual/geometry/cylinder/length').text) / 2.0
    post = base.find("visual[@name='ground_station_base_post_left']")
    post_y = float(post.findtext('pose').split()[1])
    post_half = float(post.find('geometry/box/size').text.split()[1]) / 2.0

    # Face interna da haste no maximo encostando na face do tambor, nunca dentro dele.
    assert abs(post_y) - post_half >= drum_half_width - 1e-12

    # E o tambor nao pode estar dentro da placa da base.
    radius = float(reel.find('visual/geometry/cylinder/radius').text)
    axle_z = float(reel.findtext('pose').split()[2])
    plate_top = float(base.find("visual[@name='ground_station_base_plate']")
                      .find('geometry/box/size').text.split()[2])
    assert axle_z - radius >= plate_top, 'o tambor deve ficar acima da placa'


def test_drum_dimensions_match_the_requested_geometry():
    reel = {l.attrib['name']: l for l in parse_model().findall('link')}['reel_link']
    cylinder = reel.find('visual/geometry/cylinder')

    assert math.isclose(float(cylinder.findtext('radius')), 0.07)
    assert math.isclose(float(cylinder.findtext('length')), 0.16)


def test_station_base_has_no_collision_by_default():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}

    # A estacao ocupa a origem do mundo, o mesmo ponto onde o X500 e spawnado. Com
    # collision o plinto interpenetra o trem de pouso, o UAV e empurrado para fora,
    # o cabo satura e a cadeia de ball joints diverge ate o DART abortar em
    # BallJoint::updateRelativeTransform. Nesta baseline a estacao e visual.
    assert links['ground_station_base'].find('collision') is None


def test_tether_physics_parameters_are_unchanged_by_a1():
    model = parse_model()
    links = {link.attrib['name']: link for link in model.findall('link')}
    plugin = model.find('plugin')

    masses = [float(links[f'tether_link_{i}'].find('inertial/mass').text) for i in range(1, 6)]
    assert all(math.isclose(mass, 0.03) for mass in masses)
    assert math.isclose(sum(masses), 0.15)
    assert plugin.findtext('stiffness') == '5'
    assert plugin.findtext('damping') == '0.5'
    assert plugin.findtext('max_force') == '3'
    assert all(links[f'tether_link_{i}'].find('collision') is None for i in range(1, 6))


def test_reel_is_a_passive_revolute_hanging_off_the_base():
    model = parse_model()
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    reel = joints['reel_joint']
    assert reel.attrib['type'] == 'revolute'
    assert reel.findtext('parent') == 'ground_station_base'
    assert reel.findtext('child') == 'reel_link'
    assert reel.findtext('axis/xyz') == '0 1 0'


def test_reel_carries_nothing_so_its_rotation_cannot_move_the_exit_point():
    model = parse_model()
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    # Nenhuma junta tem reel_link como pai: o tambor e um grau de liberdade
    # isolado e instrumentado. Enquanto o comprimento for constante, girar o
    # tambor nao pode deslocar a raiz do cabo.
    assert [j.attrib['name'] for j in joints.values()
            if j.findtext('parent') == 'reel_link'] == []


def test_exit_guide_is_fixed_to_the_station_not_to_the_reel():
    model = parse_model()
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    guide = joints['tether_exit_fixed']
    assert guide.attrib['type'] == 'fixed'
    assert guide.findtext('parent') == 'ground_station_base'
    assert guide.findtext('child') == 'tether_exit_point'
    # A guia continua ancorada na base; se houver payout, ele entra depois dela.
    if 'tether_payout' in joints:
        assert joints['tether_payout'].findtext('parent') == 'tether_exit_point'


def test_reel_has_no_actuator_and_only_passive_dynamics():
    model = parse_model()
    reel = {j.attrib['name']: j for j in model.findall('joint')}['reel_joint']

    # Nenhum motor, nenhum controlador: so damping/friction passivos.
    assert reel.find('axis/dynamics/damping') is not None
    assert float(reel.findtext('axis/dynamics/damping')) > 0.0
    assert float(reel.findtext('axis/dynamics/friction')) >= 0.0
    assert reel.find('physics/ode/implicit_spring_damper') is None
    assert model.find("plugin[@filename='libJointPositionController.so']") is None
    assert model.find("plugin[@filename='libJointController.so']") is None


def test_reel_and_guide_are_both_referenced_to_the_station_base():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}

    assert links['reel_link'].find('pose').attrib['relative_to'] == 'ground_station_base'
    assert links['tether_exit_point'].find('pose').attrib['relative_to'] == 'ground_station_base'


def test_exit_guide_sits_at_the_drum_top_tangent():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}
    reel = links['reel_link']
    radius = float(reel.find('visual/geometry/cylinder/radius').text)
    axle_z = float(reel.findtext('pose').split()[2])
    exit_pose = [float(v) for v in links['tether_exit_point'].findtext('pose').split()]

    # A guia fica onde o cabo deixa o tambor pela tangente superior, alinhada com o eixo.
    assert math.isclose(exit_pose[2], axle_z + radius, rel_tol=1e-12)
    assert exit_pose[0] == 0.0 and exit_pose[1] == 0.0


def test_reel_inertia_matches_a_solid_cylinder_about_its_axis():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}
    reel = links['reel_link']
    mass = float(reel.find('inertial/mass').text)
    radius = float(reel.find('visual/geometry/cylinder/radius').text)
    width = float(reel.find('visual/geometry/cylinder/length').text)
    inertia = reel.find('inertial/inertia')

    axial = float(inertia.findtext('iyy'))
    transverse = float(inertia.findtext('ixx'))
    assert math.isclose(axial, 0.5 * mass * radius ** 2, rel_tol=1e-9)
    assert math.isclose(transverse, mass * (3 * radius ** 2 + width ** 2) / 12.0, rel_tol=1e-9)
    assert math.isclose(transverse, float(inertia.findtext('izz')), rel_tol=1e-12)


def test_plugin_points_at_the_reel_joint_for_instrumentation():
    plugin = parse_model().find('plugin')

    assert plugin.findtext('reel_joint') == 'reel_joint'


def test_plugin_tether_link_count_matches_the_actual_chain():
    model = parse_model()
    plugin = model.find('plugin')
    links = [l.attrib['name'] for l in model.findall('link')
             if l.attrib['name'].startswith('tether_link_')]

    # A5 soma Newton sobre todos os elos: se a contagem divergir, a estimativa de
    # tensao perde massa silenciosamente.
    assert int(plugin.findtext('tether_link_count')) == len(links)


def test_exit_segment_is_the_link_attached_to_the_guide():
    model = parse_model()
    joints = {j.attrib['name']: j for j in model.findall('joint')}
    plugin = model.find('plugin')

    # A tangente da tensao vem deste elo; ele tem de ser o primeiro segmento de cabo.
    assert plugin.findtext('exit_segment_link') == joints['tether_joint_1'].findtext('child')
    assert joints['tether_joint_1'].findtext('parent') in (
        'tether_exit_point', 'tether_payout_link')


def test_exit_segment_extends_along_its_local_x_axis():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}
    segment = links['tether_link_1']
    com = [float(v) for v in segment.findtext('inertial/pose').split()]

    # t_hat = R * (1,0,0) so vale se o segmento se estende no +x local: o CoM fica
    # em meio comprimento ao longo de x, e nao em y ou z.
    assert com[0] > 0.0
    assert com[1] == 0.0 and com[2] == 0.0


def payout_present():
    return 'tether_payout' in {j.attrib['name'] for j in parse_model().findall('joint')}


def test_payout_is_a_prismatic_joint_between_guide_and_first_link():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    model = parse_model()
    joints = {j.attrib['name']: j for j in model.findall('joint')}

    # A7: comprimento variavel por prismatica, sem mexer nas ball joints.
    payout = joints['tether_payout']
    assert payout.attrib['type'] == 'prismatic'
    assert payout.findtext('parent') == 'tether_exit_point'
    assert payout.findtext('child') == 'tether_payout_link'
    assert joints['tether_joint_1'].findtext('parent') == 'tether_payout_link'
    assert joints['tether_joint_1'].attrib['type'] == 'ball'


def test_payout_axis_matches_the_first_segment_direction():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    model = parse_model()
    payout = {j.attrib['name']: j for j in model.findall('joint')}['tether_payout']
    links = {l.attrib['name']: l for l in model.findall('link')}

    # Em s=0 a geometria tem de ser identica a de A6: a prismatica se estende no mesmo
    # +x em que o primeiro segmento ja nascia.
    assert payout.findtext('axis/xyz') == '1 0 0'
    assert [float(v) for v in links['tether_payout_link'].findtext('pose').split()] == [0.0] * 6
    assert links['tether_payout_link'].find('pose').attrib['relative_to'] == 'tether_exit_point'
    assert [float(v) for v in links['tether_link_1'].findtext('pose').split()] == [0.0] * 6


def test_payout_travel_is_bounded_and_never_negative():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    payout = {j.attrib['name']: j for j in parse_model().findall('joint')}['tether_payout']
    lower = float(payout.findtext('axis/limit/lower'))
    upper = float(payout.findtext('axis/limit/upper'))

    # Comprimento negativo e proibido: s nunca desce abaixo de zero.
    assert lower >= 0.0
    assert upper > lower


def test_payout_link_carries_cable_mass_not_a_token_mass():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    links = {l.attrib['name']: l for l in parse_model().findall('link')}
    payout_mass = float(links['tether_payout_link'].find('inertial/mass').text)
    segment_mass = float(links['tether_link_1'].find('inertial/mass').text)

    # O elo de payout e um trecho de cabo. Com massa simbolica (1 g contra os 150 g da
    # cadeia) a prismatica ficou mal condicionada e o comprimento divergiu ate 1e117 m
    # mesmo com comando zero. A massa tem de vir da mesma densidade linear.
    assert payout_mass > 0.1 * segment_mass
    assert payout_mass <= segment_mass


def test_payout_joint_has_damping_for_conditioning():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    payout = {j.attrib['name']: j for j in parse_model().findall('joint')}['tether_payout']

    assert float(payout.findtext('axis/dynamics/damping')) > 0.0


def test_payout_plugin_declares_the_reel_coupling():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    model = parse_model()
    plugin = model.find("plugin[@filename='libTetherPayout.so']")
    joints = {j.attrib['name']: j for j in model.findall('joint')}

    assert plugin is not None
    assert plugin.findtext('reel_joint') == 'reel_joint'
    assert plugin.findtext('payout_joint') == 'tether_payout'
    assert float(plugin.findtext('r_eff')) > 0.0
    # Os limites do plugin tem de coincidir com os da junta, senao um clampa antes do outro.
    assert float(plugin.findtext('s_min')) == float(joints['tether_payout'].findtext('axis/limit/lower'))
    assert float(plugin.findtext('s_max')) == float(joints['tether_payout'].findtext('axis/limit/upper'))


def test_payout_r_eff_matches_the_drum_radius():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    model = parse_model()
    plugin = model.find("plugin[@filename='libTetherPayout.so']")
    reel = {l.attrib['name']: l for l in model.findall('link')}['reel_link']
    radius = float(reel.find('visual/geometry/cylinder/radius').text)

    # R_eff e o raio pelo qual o cabo sai do tambor.
    assert math.isclose(float(plugin.findtext('r_eff')), radius, rel_tol=1e-9)


def test_free_body_sum_includes_the_payout_link_mass():
    if not payout_present():
        return   # modelo gerado sem payout (baseline A6)
    model = parse_model()
    plugin = model.find('plugin')

    # A5 soma Newton sobre o corpo livre do cabo. Com o payout, o elo prismatico passa
    # a fazer parte desse corpo: omiti-lo tiraria a sua massa do balanco.
    assert plugin.findtext('payout_link') == 'tether_payout_link'
