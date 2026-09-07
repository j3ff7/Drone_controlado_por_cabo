import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'x500_tether_attach' / 'model.sdf'
GENERATOR = ROOT / 'tools' / 'generate_x500_tether_attach.py'
UPSTREAM = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'models' / 'x500' / 'model.sdf'

# Offset virtual usado ate A0.2, agora materializado como pose do link fisico.
LEGACY_OFFSET = (0.0, 0.0, -0.12)


def parse_model():
    return ET.parse(MODEL).getroot().find('model')


def test_generator_recreates_the_attach_variant():
    result = subprocess.run(
        [str(GENERATOR)], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
    )

    assert 'attach_link=tether_attach_link' in result.stdout
    assert 'cabo_embutido=nao' in result.stdout


def test_attach_link_exists_and_is_rigidly_fixed_to_base_link():
    model = parse_model()
    links = {link.attrib['name']: link for link in model.findall('link')}
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    assert model.attrib['name'] == 'x500_tether_attach'
    assert 'tether_attach_link' in links
    assert 'base_link' in links

    joint = joints['tether_attach_fixed']
    assert joint.attrib['type'] == 'fixed'
    assert joint.findtext('parent') == 'base_link'
    assert joint.findtext('child') == 'tether_attach_link'


def test_attach_link_pose_reproduces_the_legacy_virtual_offset_exactly():
    link = {l.attrib['name']: l for l in parse_model().findall('link')}['tether_attach_link']
    pose = link.find('pose')

    assert pose.attrib['relative_to'] == 'base_link'
    values = [float(v) for v in pose.text.split()]
    assert len(values) == 6
    assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(values[:3], LEGACY_OFFSET))
    assert all(v == 0.0 for v in values[3:]), 'attach link nao deve introduzir rotacao'


def test_attach_link_has_inertia_and_is_negligible_against_the_airframe():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}
    attach_mass = float(links['tether_attach_link'].find('inertial/mass').text)
    base_mass = float(links['base_link'].find('inertial/mass').text)

    assert attach_mass > 0.0
    assert attach_mass / base_mass < 0.01


def test_variant_embeds_no_tether_chain():
    names = {link.attrib['name'] for link in parse_model().findall('link')}

    assert not any(name.startswith('tether_link_') for name in names)
    assert 'tether_anchor_link' not in names


def test_upstream_px4_x500_is_not_modified():
    upstream = UPSTREAM.read_text()

    assert 'tether_attach_link' not in upstream
    assert "<model name='x500'>" in upstream or '<model name="x500">' in upstream


def test_attach_link_adds_no_collision_body():
    links = {l.attrib['name']: l for l in parse_model().findall('link')}

    # A conexao e force-based: uma esfera de colisao 0,12 m abaixo do base_link
    # ficaria sob o trem de pouso e criaria um contato com o solo que a baseline
    # A0.2 nao tinha, introduzindo uma segunda variavel nesta etapa.
    assert links['tether_attach_link'].find('collision') is None
