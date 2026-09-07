import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools' / 'compare_a0_2_runs.py'

spec = importlib.util.spec_from_file_location('compare_a0_2_runs', SCRIPT)
compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compare)


def test_px4_main_mode_decodes_offboard():
    assert compare.px4_main_mode(6 << 16) == 6
    assert compare.px4_main_mode(0x03040000) == 4  # AUTO/LOITER antes do arm
    assert compare.px4_main_mode(None) is None


def test_missing_metric_is_reported_as_nd_not_zero():
    assert compare.fmt(None) == 'N/D'
    assert compare.fmt(0.0) == '0.000'
    assert compare.fmt(True) == 'sim'
    assert compare.fmt(False) == 'nao'


def test_num_rejects_empty_and_non_finite_fields():
    assert compare.num({'a': ''}, 'a') is None
    assert compare.num({'a': 'nan'}, 'a') is None
    assert compare.num({'a': 'inf'}, 'a') is None
    assert compare.num({'a': '1.5'}, 'a') == 1.5


def test_rms_ignores_missing_samples():
    assert compare.rms([3.0, None, 4.0]) == math.sqrt(12.5)
    assert compare.rms([None]) is None


def test_mean_tail_uses_only_the_last_samples_of_the_phase():
    rows = [{'phase': 'climb_hover', 'x': str(v)} for v in range(10)]
    rows.append({'phase': 'translate_out', 'x': '99'})

    assert compare.mean_tail(rows, 'climb_hover', 'x', count=2) == 8.5
