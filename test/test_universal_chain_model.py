"""UniversalJoint como alternativa a BallJoint: 2 DOF de flexao por conexao.

So o bloco da junta muda; elos, massas, inercias, colisoes e poses sao os mesmos.
`ball` continua sendo o padrao do gerador.
"""
import importlib.util
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'
GENERATOR = ROOT / 'tools' / 'generate_tether_anchor_chain.py'

spec = importlib.util.spec_from_file_location('generate_tether_anchor_chain', GENERATOR)
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

BASE_ARGS = [
    '--links', '10', '--length', '2.50', '--rho', '0.06', '--radius', '0.003',
    '--initial-axis', 'taut', '--taut-target', '2.3825 0 -0.083', '--link-collisions',
    '--force-constraint', '--stiffness', '5', '--damping', '0.5', '--max-force', '3',
]


def run(*extra):
    result = subprocess.run([str(GENERATOR)] + BASE_ARGS + list(extra), cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    return result.stdout, ET.parse(MODEL).getroot().find('model')


def joints(model):
    return [j for j in model.findall('joint') if j.attrib['name'].startswith('tether_joint_')]


def links(model):
    return [l for l in model.findall('link') if l.attrib['name'].startswith('tether_link_')]


def vec(joint, tag):
    return [float(v) for v in joint.find(tag).find('xyz').text.split()]


def test_default_is_still_ball():
    stdout, model = run()
    assert 'joint_type=ball' in stdout
    assert {j.attrib['type'] for j in joints(model)} == {'ball'}


def test_every_connection_becomes_one_universal_joint():
    _, model = run('--joint-type', 'universal')
    assert len(joints(model)) == 10
    assert {j.attrib['type'] for j in joints(model)} == {'universal'}


def test_two_orthogonal_axes_perpendicular_to_the_link():
    _, model = run('--joint-type', 'universal')
    for joint in joints(model):
        a1, a2 = vec(joint, 'axis'), vec(joint, 'axis2')
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(a1, (0, 1, 0)))
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(a2, (0, 0, 1)))
        assert math.isclose(sum(x * y for x, y in zip(a1, a2)), 0.0, abs_tol=1e-12)
        # nenhum eixo ao longo do cabo: sem DOF de torcao
        assert a1[0] == 0.0 and a2[0] == 0.0
        for tag in ('axis', 'axis2'):
            assert joint.find(tag).find('xyz').attrib['expressed_in'] == joint.find('child').text
            assert float(joint.find(tag).find('limit').find('upper').text) >= 1e15
            assert float(joint.find(tag).find('dynamics').find('damping').text) == 0.0


def test_only_the_joint_block_changes():
    _, ball = run()
    _, universal = run('--joint-type', 'universal')
    assert [ET.tostring(l) for l in links(ball)] == [ET.tostring(l) for l in links(universal)]

    def frames(model):
        return [(j.find('parent').text, j.find('child').text, j.find('pose').text,
                 j.find('pose').attrib['relative_to']) for j in joints(model)]

    assert frames(ball) == frames(universal)


def test_roll_rotates_the_axis_pair_and_keeps_it_orthogonal():
    for roll in (0.0, 30.0, 45.0, 90.0, -17.0):
        a1, a2 = gen.universal_axes(roll)
        assert math.isclose(math.hypot(*a1), 1.0) and math.isclose(math.hypot(*a2), 1.0)
        assert math.isclose(sum(x * y for x, y in zip(a1, a2)), 0.0, abs_tol=1e-12)
        assert a1[0] == 0.0 and a2[0] == 0.0


def test_no_prismatic_joint_in_any_topology():
    for extra in ((), ('--joint-type', 'universal')):
        _, model = run(*extra)
        assert not [j for j in model.findall('joint') if j.attrib['type'] == 'prismatic']
