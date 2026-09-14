"""Matematica do registrador de conexao fisica tether <-> X500.

As poses do Gazebo chegam por entidade (links relativos ao modelo pai), e o texto protobuf
omite campos zero; errar qualquer um dos dois faria a folga do pivo parecer grande ou nula.
"""
import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'record_tether_connection', ROOT / 'tools' / 'record_tether_connection.py')
rec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rec)

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def close(a, b, tol=1e-9):
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


def test_identity_rotation_keeps_the_vector():
    assert close(rec.quat_rotate(IDENTITY, (0.3, -0.2, 0.1)), (0.3, -0.2, 0.1))


def test_quarter_turn_about_z_maps_x_to_y():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert close(rec.quat_rotate(q, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_attach_link_is_composed_with_the_drone_model_pose():
    # X500 pousado em (0.5, 0, 0.227); tether_attach_link a (0, 0, -0.12) do modelo
    assert close(rec.compose(((0.5, 0.0, 0.227), IDENTITY), (0.0, 0.0, -0.12)), (0.5, 0.0, 0.107))


def test_composition_follows_the_drone_attitude():
    q = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))   # 90 graus de roll
    assert close(rec.compose(((0.0, 0.0, 1.0), q), (0.0, 0.0, -0.12)), (0.0, 0.12, 1.0))


def test_protobuf_block_with_omitted_zero_fields():
    block = ('pose {\n  name: "tether_attach_link"\n  id: 12\n  position {\n    z: -0.12\n  }\n'
             '  orientation {\n    w: 1\n  }\n}')
    name, pos, ori = rec.parse_pose_block(block)
    assert name == 'tether_attach_link'
    assert pos == (0.0, 0.0, -0.12)
    assert ori == IDENTITY


def test_roll_and_pitch_from_quaternion():
    roll_q = (math.sin(math.radians(15)), 0.0, 0.0, math.cos(math.radians(15)))   # 30 graus
    pitch_q = (0.0, math.sin(math.radians(10)), 0.0, math.cos(math.radians(10)))  # 20 graus
    assert close(rec.roll_pitch_deg(roll_q), (30.0, 0.0), tol=1e-6)
    assert close(rec.roll_pitch_deg(pitch_q), (0.0, 20.0), tol=1e-6)
