"""--taut-bulge: o mesmo arco de --initial-axis taut, espelhado para cima.

Com folga grande o arco para baixo nasce atravessando o solo; elos que nascem abaixo do plano
ficam presos la. Espelhar mantem comprimento, corda e extremos e so muda o lado do arco.
"""
import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'generate_tether_anchor_chain', ROOT / 'tools' / 'generate_tether_anchor_chain.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

TARGET = (1.2, 0.0, -0.083)


def points(vectors):
    p, out = [0.0, 0.0, 0.0], [[0.0, 0.0, 0.0]]
    for v in vectors:
        p = [p[i] + v[i] for i in range(3)]
        out.append(p)
    return out


def test_default_bulge_is_down_and_unchanged():
    assert gen.taut_vectors(10, 0.25, TARGET) == gen.taut_vectors(10, 0.25, TARGET, 'down')


def test_both_bulges_keep_segment_length_and_endpoint():
    for bulge in ('down', 'up'):
        vectors = gen.taut_vectors(10, 0.25, TARGET, bulge)
        for v in vectors:
            assert math.isclose(math.sqrt(sum(c * c for c in v)), 0.25, rel_tol=1e-9)
        end = points(vectors)[-1]
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(end, TARGET))


def test_up_bulge_stays_above_the_chord_and_down_below_it():
    chord = TARGET
    for bulge, sign in (('up', 1.0), ('down', -1.0)):
        for p in points(gen.taut_vectors(10, 0.25, TARGET, bulge))[1:-1]:
            s = sum(p[i] * chord[i] for i in range(3)) / sum(c * c for c in chord)
            height = p[2] - s * chord[2]
            assert sign * height > 0.0


def test_up_bulge_is_the_reflection_of_down_across_the_chord():
    # O arco espelhado e a reflexao do original pela reta da corda, no plano do arco:
    # p_up = 2 * proj_corda(p_down) - p_down.
    norm2 = sum(c * c for c in TARGET)
    up = points(gen.taut_vectors(10, 0.25, TARGET, 'up'))
    down = points(gen.taut_vectors(10, 0.25, TARGET, 'down'))
    for a, b in zip(up, down):
        s = sum(b[i] * TARGET[i] for i in range(3)) / norm2
        reflected = [2 * s * TARGET[i] - b[i] for i in range(3)]
        assert all(math.isclose(x, y, abs_tol=1e-6) for x, y in zip(a, reflected))
