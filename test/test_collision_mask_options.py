"""Filtro de colisao cabo x drone: opcoes --collide-bitmask dos geradores (saida em diretorio temporario).

Os padroes nao podem mudar os modelos versionados; com a opcao, todas as colisoes do X500 e dos elos
recebem a mascara.
"""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ATTACH_GEN = ROOT / 'tools' / 'generate_x500_tether_attach.py'
CHAIN_GEN = ROOT / 'tools' / 'generate_tether_anchor_chain.py'
TRACKED_ATTACH = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'x500_tether_attach' / 'model.sdf'
CHAIN_ARGS = ['--links', '5', '--length', '2.5', '--rho', '0.06', '--radius', '0.003',
              '--initial-axis', 'folded_ground', '--force-constraint', '--stiffness', '5',
              '--damping', '0.5', '--max-force', '3', '--no-reel-actuator', '--link-collisions']


def run(*cmd):
    return subprocess.run([str(c) for c in cmd], cwd=ROOT, text=True, capture_output=True, check=True)


def collisions(path):
    return ET.parse(path).getroot().iter('collision')


def test_attach_generator_default_matches_tracked_variant(tmp_path):
    run(ATTACH_GEN, '--output-dir', tmp_path)
    assert (tmp_path / 'model.sdf').read_text() == TRACKED_ATTACH.read_text()
    assert '<collide_bitmask>' not in (tmp_path / 'model.sdf').read_text()


def test_attach_generator_masks_every_x500_collision(tmp_path):
    out = tmp_path / 'x500_tether_attach_mask'
    run(ATTACH_GEN, '--model-name', 'x500_tether_attach_mask', '--collide-bitmask', '1', '--output-dir', out)
    root = ET.parse(out / 'model.sdf').getroot()
    assert root.find('model').attrib['name'] == 'x500_tether_attach_mask'
    masks = [c.findtext('surface/contact/collide_bitmask') for c in collisions(out / 'model.sdf')]
    assert len(masks) == 9 and set(masks) == {'1'}
    # o resto do contato upstream e preservado
    assert all(c.find('surface/contact/ode/min_depth') is not None for c in collisions(out / 'model.sdf'))
    assert '<name>x500_tether_attach_mask</name>' in (out / 'model.config').read_text()


def test_chain_generator_mask_only_when_requested(tmp_path):
    plain, masked = tmp_path / 'plain', tmp_path / 'masked'
    run(CHAIN_GEN, *CHAIN_ARGS, '--output-dir', plain)
    run(CHAIN_GEN, *CHAIN_ARGS, '--collide-bitmask', '2', '--output-dir', masked)
    assert '<collide_bitmask>' not in (plain / 'model.sdf').read_text()
    link_masks = [c.findtext('surface/contact/collide_bitmask') for c in collisions(masked / 'model.sdf')
                  if c.attrib['name'].startswith('tether_link_')]
    assert len(link_masks) == 5 and set(link_masks) == {'2'}


def test_masks_separate_cable_from_drone_but_not_from_ground():
    ground, drone, cable = 0xFFFF, 1, 2
    assert drone & cable == 0
    assert ground & cable and ground & drone


def test_chain_generator_writes_tangent_parameters(tmp_path):
    run(CHAIN_GEN, *CHAIN_ARGS, '--tangent-window', '0.3', '--tangent-samples', '5', '--output-dir', tmp_path)
    plugin = ET.parse(tmp_path / 'model.sdf').getroot().find(".//plugin[@name='drone_cabo::TetherForceConstraint']")
    assert plugin.findtext('tangent_window') == '0.3'
    assert plugin.findtext('tangent_samples') == '5'
