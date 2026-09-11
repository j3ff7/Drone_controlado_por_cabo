import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / 'tools' / 'generate_tether_anchor_chain.py'

spec = importlib.util.spec_from_file_location('generate_tether_anchor_chain', GENERATOR)
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def norm(vector):
    return math.sqrt(sum(component * component for component in vector))


def test_every_edge_has_exactly_the_segment_length():
    for n_links, length in ((5, 2.5), (10, 2.5), (20, 2.5), (25, 2.5)):
        segment = length / n_links
        for vector in gen.coil_vectors(n_links, segment):
            assert math.isclose(norm(vector), segment, rel_tol=1e-12)


def test_total_length_is_preserved_across_discretizations():
    # O sweep de B1 so e justo se o comprimento total nao mudar com N.
    for n_links in (5, 10, 20, 25):
        segment = 2.5 / n_links
        total = sum(norm(v) for v in gen.coil_vectors(n_links, segment))
        assert math.isclose(total, 2.5, rel_tol=1e-12)


def test_the_coil_closes_on_itself():
    for n_links in (5, 10, 20, 25):
        vectors = gen.coil_vectors(n_links, 2.5 / n_links)
        for axis in range(3):
            assert abs(sum(v[axis] for v in vectors)) < 1e-9


def test_the_coil_is_planar_and_horizontal():
    for n_links in (5, 20):
        for vector in gen.coil_vectors(n_links, 2.5 / n_links):
            assert vector[2] == 0.0


def test_underlying_curve_converges_to_the_same_circle_for_every_n():
    # A poligonal aproxima sempre a MESMA circunferencia de perimetro L: o raio tende a
    # L/(2*pi) conforme N cresce. E isso que separa discretizacao de condicao inicial.
    target = 2.5 / (2.0 * math.pi)
    previous_error = None
    for n_links in (5, 10, 20, 25):
        segment = 2.5 / n_links
        radius = segment / (2.0 * math.sin(math.pi / n_links))
        error = abs(radius - target)
        if previous_error is not None:
            assert error < previous_error
        previous_error = error
    # Em N=25 o raio ja esta a 0,26% da circunferencia limite.
    assert previous_error < 2e-3


def test_coil_rejects_degenerate_discretizations():
    import pytest
    with pytest.raises(SystemExit):
        gen.coil_vectors(2, 1.0)


TAUT_TARGET = (2.38, 0.0, -0.083)


def test_taut_edges_have_exactly_the_segment_length():
    for n_links in (5, 10, 20, 25):
        segment = 2.5 / n_links
        for vector in gen.taut_vectors(n_links, segment, TAUT_TARGET):
            assert math.isclose(norm(vector), segment, rel_tol=1e-9)


def test_taut_chain_lands_exactly_on_the_uav_attach_point():
    # A condicao inicial so e "esticada entre os dois pontos" se o ultimo elo terminar
    # no attach. Sem isso a constraint daria um puxao no instante zero.
    for n_links in (5, 10, 20, 25):
        vectors = gen.taut_vectors(n_links, 2.5 / n_links, TAUT_TARGET)
        for axis in range(3):
            assert math.isclose(sum(v[axis] for v in vectors), TAUT_TARGET[axis], abs_tol=1e-9)


def test_taut_total_length_is_preserved_across_discretizations():
    for n_links in (5, 10, 20, 25):
        segment = 2.5 / n_links
        total = sum(norm(v) for v in gen.taut_vectors(n_links, segment, TAUT_TARGET))
        assert math.isclose(total, 2.5, rel_tol=1e-9)


def test_taut_sags_downward_and_converges_to_the_same_curve():
    # A barriga tem de ser praticamente a mesma para todo N: e o que separa o efeito de
    # discretizacao do efeito de condicao inicial.
    sags = []
    for n_links in (5, 10, 20, 25):
        vectors = gen.taut_vectors(n_links, 2.5 / n_links, TAUT_TARGET)
        heights = [sum(v[2] for v in vectors[:k + 1]) for k in range(n_links)]
        sags.append(min(heights))
    assert all(sag < -0.1 for sag in sags), 'o arco deve curvar para baixo'
    assert max(sags) - min(sags) < 5e-3


def test_taut_rejects_a_target_farther_than_the_cable():
    import pytest
    with pytest.raises(SystemExit):
        gen.taut_vectors(10, 0.25, (3.0, 0.0, 0.0))
    with pytest.raises(SystemExit):
        gen.taut_vectors(10, 0.25, (0.0, 0.0, 0.0))
