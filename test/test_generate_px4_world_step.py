"""Copia do mundo default do PX4 com passo de fisica da baseline (1 ms)."""
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('generate_px4_world_step', ROOT / 'tools' / 'generate_px4_world_step.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

WORLD = ('<world name="default"><physics type="ode"><max_step_size>0.004</max_step_size>'
         '<real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate>'
         '</physics></world>')


def test_step_and_update_rate_are_replaced():
    out = gen.world_with_step(WORLD, 0.001)
    assert '<max_step_size>0.001</max_step_size>' in out
    assert '<real_time_update_rate>1000</real_time_update_rate>' in out
    assert '0.004' not in out and '>250<' not in out


def test_unexpected_world_is_rejected():
    with pytest.raises(ValueError):
        gen.world_with_step(WORLD.replace('0.004', '0.002'), 0.001)


def test_px4_default_world_is_supported_when_available():
    if not gen.PX4_DEFAULT_WORLD.exists():
        pytest.skip('PX4 local ausente')
    out = gen.world_with_step(gen.PX4_DEFAULT_WORLD.read_text(), 0.001)
    assert out.count('<max_step_size>0.001</max_step_size>') == 1
