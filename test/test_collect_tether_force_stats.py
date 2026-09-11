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


def test_parse_accepts_message_whose_zero_fields_were_omitted():
    collector = load_collector()
    # theta=0 e omega=0 no reel parado: o protobuf de texto omite os dois campos.
    text = "z: 1\n\nz: 1\n\n"

    values, invalid = collector.parse_vector3d_stream_detailed(text)

    assert invalid == 0
    assert values == [(0.0, 0.0, 1.0), (0.0, 0.0, 1.0)]


def test_parse_marks_message_without_closing_delimiter_invalid():
    collector = load_collector()

    values, invalid = collector.parse_vector3d_stream_detailed("x: 1\ny: 2\nz: 0\n")

    assert values == []
    assert invalid == 1


def test_parse_reads_an_all_zero_message_that_prints_no_fields():
    collector = load_collector()
    # Vector3d com x=y=z=0 nao imprime campo algum; so o header aparece.
    text = "header {\n  stamp {\n    sec: 1\n  }\n}\n\nheader {\n  stamp {\n    sec: 2\n  }\n}\n\n"

    values, invalid = collector.parse_vector3d_stream_detailed(text)

    assert invalid == 0
    assert values == [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]


def test_parse_returns_nothing_for_an_empty_stream():
    collector = load_collector()

    assert collector.parse_vector3d_stream_detailed('') == ([], 0)
    assert collector.parse_vector3d_stream_detailed('\n\n\n') == ([], 0)
