"""Tangente suavizada, compensacao de atitude e penetracao no solo (tools/record_tether_connection.py).

Implementacao Python independente da do plugin (TetherGeometry.hh), usada como verificacao cruzada.
"""
import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('record_tether_connection',
                                              ROOT / 'tools' / 'record_tether_connection.py')
rec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rec)


def q_axis(axis, degrees):
    h = math.radians(degrees) / 2.0
    return (axis[0] * math.sin(h), axis[1] * math.sin(h), axis[2] * math.sin(h), math.cos(h))


def close(a, b, tol=1e-9):
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


def test_window_inside_straight_link_is_link_direction():
    t, w = rec.smoothed_tangent([(0, 0, 0), (0.5, 0, -0.5), (1, 0, -1)], 0.15, 4)
    assert close(t, (math.sqrt(0.5), 0.0, -math.sqrt(0.5)))
    assert math.isclose(w, 0.15)


def test_window_is_limited_to_cable_length():
    t, w = rec.smoothed_tangent([(0, 0, 0), (0.1, 0, 0)], 1.0, 4)
    assert math.isclose(w, 0.1) and close(t, (1.0, 0.0, 0.0))


def test_window_across_a_bend_averages_the_links():
    points = [(0, 0, 0), (0.1, 0, -0.1), (0.2, 0, -0.1)]
    t, _ = rec.smoothed_tangent(points, 0.1 * math.sqrt(2) + 0.1, 5)
    _, elevation = rec.angles_deg(t)
    assert -44.0 < elevation < -1.0


def test_degenerate_polyline_is_invalid():
    assert rec.smoothed_tangent([(1, 1, 1), (1, 1, 1)], 0.15, 4) == (None, 0.0)


def test_point_at_arc_crosses_joints_and_saturates():
    points = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    assert close(rec.point_at_arc(points, 1.5), (1.0, 0.5, 0.0))
    assert close(rec.point_at_arc(points, 9.0), (1.0, 1.0, 0.0))


def test_angle_convention():
    assert close(rec.angles_deg((1, 0, 0)), (0.0, 0.0))
    assert close(rec.angles_deg((0, 1, 0)), (90.0, 0.0))
    assert math.isclose(rec.angles_deg((0, 0, -1))[1], -90.0)


def test_world_to_body_pitch_plumb_line_points_forward_down():
    # pitch +30 no gz (nariz para baixo): prumo visto do corpo = frente e para baixo
    tb = rec.world_to_body(q_axis((0, 1, 0), 30.0), (0.0, 0.0, -1.0))
    assert close(tb, (0.5, 0.0, -math.sqrt(3) / 2))
    assert close(rec.angles_deg(tb), (0.0, -60.0), 1e-9)


def test_world_to_body_roll_plumb_line_points_right_down():
    tb = rec.world_to_body(q_axis((1, 0, 0), 20.0), (0.0, 0.0, -1.0))
    assert close(rec.angles_deg(tb), (-90.0, -70.0), 1e-9)


def test_attitude_changes_body_angles_but_not_world_direction():
    t_world = (0.6, 0.0, -0.8)
    level = rec.angles_deg(rec.world_to_body((0, 0, 0, 1), t_world))
    yawed = rec.angles_deg(rec.world_to_body(q_axis((0, 0, 1), 45.0), t_world))
    assert close(level, rec.angles_deg(t_world))
    assert math.isclose(yawed[0], level[0] - 45.0, abs_tol=1e-9)
    assert math.isclose(yawed[1], level[1], abs_tol=1e-9)       # yaw nao muda a elevacao


def test_rpy_deg_matches_gz_convention():
    # R = Rz(yaw) Ry(pitch) Rx(roll)
    q = rec.quat_mul(q_axis((0, 0, 1), 30.0), rec.quat_mul(q_axis((0, 1, 0), -20.0), q_axis((1, 0, 0), 10.0)))
    assert close(rec.rpy_deg(q), (10.0, -20.0, 30.0), 1e-9)


def test_lowest_surface_of_horizontal_and_vertical_cylinders():
    assert math.isclose(rec.lowest_surface_z((0, 0, 0.01), (1, 0, 0.01), 0.002), 0.008)
    assert math.isclose(rec.lowest_surface_z((0, 0, 0.5), (0, 0, 0.01), 0.002), 0.01)
