import csv
import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools' / 'compare_stages.py'

spec = importlib.util.spec_from_file_location('compare_stages', SCRIPT)
stages = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stages)


def write_reel(path, rows):
    with (path / 'run_reel.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=['t_wall', 't_sim', 'x', 'y', 'z'])
        writer.writeheader()
        writer.writerows(rows)


def test_reel_metrics_summarize_theta_and_omega(tmp_path):
    write_reel(tmp_path, [
        {'t_wall': 0, 't_sim': 0, 'x': 3.0, 'y': 0.5, 'z': 1},
        {'t_wall': 1, 't_sim': 1, 'x': 3.4, 'y': -0.9, 'z': 1},
        {'t_wall': 2, 't_sim': 2, 'x': 3.2, 'y': 0.1, 'z': 1},
    ])

    metrics = stages.reel_metrics(tmp_path)

    assert metrics['samples'] == 3
    assert math.isclose(metrics['theta_range_rad'], 0.4)
    assert math.isclose(metrics['omega_abs_max_rad_s'], 0.9)
    assert metrics['available_fraction'] == 1.0


def test_reel_metrics_drop_unavailable_samples_instead_of_zeroing_them(tmp_path):
    # z=0 marca indisponibilidade e x/y vem como NaN: nao podem virar 0 rad.
    write_reel(tmp_path, [
        {'t_wall': 0, 't_sim': 0, 'x': 'nan', 'y': 'nan', 'z': 0},
        {'t_wall': 1, 't_sim': 1, 'x': 1.0, 'y': 0.2, 'z': 1},
    ])

    metrics = stages.reel_metrics(tmp_path)

    assert metrics['samples'] == 1
    assert metrics['theta_min_rad'] == 1.0


def test_reel_metrics_absent_when_no_reel_topic_was_recorded(tmp_path):
    assert stages.reel_metrics(tmp_path) is None


def test_find_analysis_prefers_the_per_run_analysis_file(tmp_path):
    (tmp_path / 'horizontal_analysis.json').write_text('{}')

    assert stages.find_analysis(tmp_path) == 'horizontal_analysis.json'
    assert stages.find_analysis(tmp_path / 'vazio') is None
