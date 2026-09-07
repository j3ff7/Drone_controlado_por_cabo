import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools' / 'record_tether_timeseries.py'

spec = importlib.util.spec_from_file_location('record_tether_timeseries', SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


def drain(parser, text):
    samples = []
    for line in text.splitlines(keepends=True):
        sample = parser.feed_line(line)
        if sample is not None:
            samples.append(sample)
    sample = parser.flush()
    if sample is not None:
        samples.append(sample)
    return samples


def test_vector_stream_closes_messages_on_delimiter():
    parser = recorder.Vector3dStreamParser()
    text = 'x: 0.2\ny: 3\nz: 1\n\nx: 0.3\ny: 1\n\n'

    assert drain(parser, text) == [(0.2, 3.0, 1.0), (0.3, 1.0, 0.0)]
    assert parser.invalid_messages == 0


def test_vector_stream_flags_truncated_message():
    parser = recorder.Vector3dStreamParser()

    assert drain(parser, 'x: 0.1\ny: 1\n\nx: 0.2\n') == [(0.1, 1.0, 0.0)]
    assert parser.invalid_messages == 1


def test_vector_stream_keeps_nan_so_unavailability_is_not_a_zero():
    parser = recorder.Vector3dStreamParser()

    samples = drain(parser, 'x: nan\ny: nan\nz: 0\n\n')

    assert len(samples) == 1
    assert math.isnan(samples[0][0])
    assert math.isnan(samples[0][1])
    assert samples[0][2] == 0.0


def test_world_stats_parser_reads_sim_time_and_rtf():
    parser = recorder.WorldStatsStreamParser()
    text = (
        'sim_time {\n  sec: 12\n  nsec: 500000000\n}\n'
        'real_time {\n  sec: 13\n  nsec: 0\n}\n'
        'iterations: 3125\n'
        'real_time_factor: 0.97\n'
        '\n'
    )

    samples = drain(parser, text)

    assert len(samples) == 1
    sim_time, real_time, rtf = samples[0]
    assert math.isclose(sim_time, 12.5)
    assert math.isclose(real_time, 13.0)
    assert math.isclose(rtf, 0.97)


def test_sim_clock_interpolates_between_world_stats_samples():
    rows = [
        {'t_wall': 100.0, 'sim_time': 0.0},
        {'t_wall': 101.0, 'sim_time': 1.0},
        {'t_wall': 102.0, 'sim_time': 1.5},
    ]

    to_sim = recorder.build_sim_clock(rows)

    assert math.isclose(to_sim(100.5), 0.5)
    assert math.isclose(to_sim(101.5), 1.25)
    assert math.isclose(to_sim(102.0), 1.5)


def test_world_stats_parser_rejects_truncated_message_instead_of_reporting_zero():
    parser = recorder.WorldStatsStreamParser()

    samples = drain(parser, 'sim_time {\n  sec: 10\n  nsec: 0\n}\n\nsim_time {\n')

    assert samples == [(10.0, 0.0, float('nan'))] or math.isnan(samples[0][2])
    assert len(samples) == 1
    assert parser.invalid_messages == 1
