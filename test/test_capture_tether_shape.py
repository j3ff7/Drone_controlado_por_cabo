"""Reducao da poligonal do cabo a sag/lateral/corda (C1 e C3)."""
import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'capture_tether_shape.py'

spec = importlib.util.spec_from_file_location('capture_tether_shape', TOOL)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def poses(points):
    return {f'tether_link_{i}': {'x': p[0], 'y': p[1], 'z': p[2]}
            for i, p in enumerate(points, start=1)}


def test_straight_horizontal_cable_has_no_sag_and_no_lateral():
    points = [(0.0, 0.0, 0.19), (1.0, 0.0, 0.19), (2.0, 0.0, 0.19)]
    result = capture.shape(poses(points), 3, 0.19)
    assert math.isclose(result['span_m'], 2.0, rel_tol=1e-12)
    assert math.isclose(result['sag_m'], 0.0, abs_tol=1e-12)
    assert math.isclose(result['lateral_m'], 0.0, abs_tol=1e-12)


def test_sag_is_the_perpendicular_drop_below_the_chord():
    # Ponta na mesma altura da saida: a corda e horizontal e o sag e o proprio mergulho.
    points = [(0.0, 0.0, 0.19), (1.0, 0.0, 0.19 - 0.3), (2.0, 0.0, 0.19)]
    result = capture.shape(poses(points), 3, 0.19)
    assert math.isclose(result['sag_m'], 0.3, rel_tol=1e-9)
    assert math.isclose(result['lateral_m'], 0.0, abs_tol=1e-12)


def test_lateral_is_measured_separately_from_sag():
    points = [(0.0, 0.0, 0.19), (1.0, 0.25, 0.19), (2.0, 0.0, 0.19)]
    result = capture.shape(poses(points), 3, 0.19)
    assert math.isclose(result['lateral_m'], 0.25, rel_tol=1e-9)
    assert math.isclose(result['sag_m'], 0.0, abs_tol=1e-12)


def test_sag_ignores_points_above_the_chord():
    points = [(0.0, 0.0, 0.19), (1.0, 0.0, 0.4), (2.0, 0.0, 0.19)]
    assert capture.shape(poses(points), 3, 0.19)['sag_m'] == 0.0


def test_missing_links_do_not_break_the_reduction():
    available = {'tether_link_1': {'x': 0.0, 'z': 0.19},
                 'tether_link_3': {'x': 2.0, 'z': 0.19}}
    result = capture.shape(available, 3, 0.19)
    assert result['n_points'] == 2


def test_too_few_points_returns_none():
    assert capture.shape({}, 5, 0.19) is None
