"""Compila e executa os testes C++ da geometria do tether usada pelo TetherForceConstraint."""
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'test' / 'cpp' / 'test_tether_geometry.cc'


def test_tether_geometry_header(tmp_path):
    if shutil.which('g++') is None or shutil.which('pkg-config') is None:
        pytest.skip('g++/pkg-config indisponivel')
    flags = subprocess.run(['pkg-config', '--cflags', 'gz-math7'], capture_output=True, text=True)
    if flags.returncode != 0:
        pytest.skip('gz-math7 indisponivel')
    binary = tmp_path / 'test_tether_geometry'
    build = subprocess.run(['g++', '-std=c++17', str(SOURCE), '-o', str(binary), *flags.stdout.split()],
                           capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip().endswith('OK')
