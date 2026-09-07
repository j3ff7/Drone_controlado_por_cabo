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
    assert set(links) == {
        'anchor_link',
        'tether_link_1',
        'tether_link_2',
        'tether_link_3',
        'tether_link_4',
        'tether_link_5',
    }
    assert len(joints) == 6
    assert joints['anchor_world_fixed'].attrib['type'] == 'fixed'
    assert joints['anchor_world_fixed'].findtext('parent') == 'world'
    assert joints['anchor_world_fixed'].findtext('child') == 'anchor_link'

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
    assert plugin.findtext('anchor_joint') == 'anchor_world_fixed'
    assert plugin.findtext('tether_offset') == '0.5 0 0'
    assert plugin.findtext('stiffness') == '5'
    assert plugin.findtext('damping') == '0.5'
    assert plugin.findtext('max_force') == '3'


def test_old_virtual_endpoint_is_no_longer_the_active_interface():
    plugin = parse_model().find('plugin')

    assert plugin.findtext('drone_link') != 'base_link'
    assert plugin.findtext('drone_offset').split() == ['0', '0', '0']
