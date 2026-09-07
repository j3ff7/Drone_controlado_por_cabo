import importlib.util
from pathlib import Path


def load_collector():
    path = Path(__file__).resolve().parents[1] / 'tools' / 'collect_tether_force_stats.py'
    spec = importlib.util.spec_from_file_location('collect_tether_force_stats', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_two_saturated_messages_keeps_each_z_flag():
    collector = load_collector()
    text = """x: 0.2
y: 3
z: 1

x: 0.3
y: 3
z: 1
"""

    values, invalid = collector.parse_vector3d_stream_detailed(text)

    assert invalid == 0
    assert values == [(0.2, 3.0, 1.0), (0.3, 3.0, 1.0)]
    assert [sample[2] >= 0.5 for sample in values] == [True, True]


def test_parse_mixed_saturation_sequence_and_omitted_zero_z():
    collector = load_collector()
    text = """x: 0.1
y: 1

x: 0.2
y: 3
z: 1

x: 0.3
y: 1
z: 0
"""

    values, invalid = collector.parse_vector3d_stream_detailed(text)

    assert invalid == 0
    assert values == [(0.1, 1.0, 0.0), (0.2, 3.0, 1.0), (0.3, 1.0, 0.0)]
    assert [sample[2] >= 0.5 for sample in values] == [False, True, False]


def test_parse_marks_truncated_message_invalid():
    collector = load_collector()
    text = """x: 0.1
y: 1

x: 0.2
"""

    values, invalid = collector.parse_vector3d_stream_detailed(text)

    assert values == [(0.1, 1.0, 0.0)]
    assert invalid == 1


def test_parse_marks_nan_message_invalid():
    collector = load_collector()
    text = """x: nan
y: 1
z: 0
"""

    values, invalid = collector.parse_vector3d_stream_detailed(text)

    assert values == []
    assert invalid == 1
