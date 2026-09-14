"""Mundo com juntas fisicas tether <-> X500 (arquitetura da `dev` reproduzida na `shared`)."""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPOSER = ROOT / 'tools' / 'generate_x500_tether_world.py'
TETHER = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'


def compose(tmp_path, *extra):
    out = tmp_path / 'mundo.sdf'
    result = subprocess.run([str(COMPOSER), '--output', str(out)] + list(extra), cwd=ROOT,
                            capture_output=True, text=True, check=True)
    return result.stdout, ET.parse(out).getroot().find('world')


def test_world_joins_tether_tip_to_drone_root_with_a_ball(tmp_path):
    _, world = compose(tmp_path, '--drone-x', '1.2')
    joints = world.findall('joint')
    assert len(joints) == 1
    joint = joints[0]
    assert joint.attrib['type'] == 'ball'
    assert joint.find('parent').text == 'tether_anchor_chain::tether_link_10'
    # filho = raiz da arvore do X500: ligar a um link com pai fecharia um laco
    assert joint.find('child').text == 'x500_tether_attach_0::base_link'
    assert [float(v) for v in joint.find('pose').text.split()] == [0, 0, -0.12, 0, 0, 0]


def test_world_keeps_px4_default_systems_and_includes_both_models(tmp_path):
    _, world = compose(tmp_path)
    assert world.attrib['name'] == 'default'
    plugins = {p.attrib['name'] for p in world.findall('plugin')}
    assert {'gz::sim::systems::Physics', 'gz::sim::systems::Imu',
            'gz::sim::systems::AirPressure', 'gz::sim::systems::Sensors'} <= plugins
    names = {i.find('name').text for i in world.findall('include')}
    assert names == {'tether_anchor_chain', 'x500_tether_attach_0'}


def test_tether_has_ball_joints_and_no_force_plugin_or_prismatic(tmp_path):
    compose(tmp_path)
    model = ET.parse(TETHER).getroot().find('model')
    joints = [j for j in model.findall('joint') if j.attrib['name'].startswith('tether_joint_')]
    assert joints and {j.attrib['type'] for j in joints} == {'ball'}
    assert not model.findall('plugin')
    assert not [j for j in model.findall('joint') if j.attrib['type'] == 'prismatic']


def test_tip_of_the_generated_chain_lands_on_the_drone_pivot(tmp_path):
    stdout, _ = compose(tmp_path, '--drone-x', '1.2')
    assert 'pivo_no_drone=(1.200, 0.000, 0.107)' in stdout


def test_composer_refuses_a_drone_beyond_the_cable_reach(tmp_path):
    result = subprocess.run([str(COMPOSER), '--output', str(tmp_path / 'x.sdf'),
                             '--drone-x', '3.0'], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert 'nao alcanca' in result.stdout + result.stderr
