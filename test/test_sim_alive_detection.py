"""Deteccao do servidor gz pelo `ps`.

Duas armadilhas ja custaram medicoes neste projeto: `pgrep -f 'gz sim'` casava com a
propria linha de comando de quem procurava (B1), e `comm` nao serve porque o executavel
do `gz` e um wrapper ruby — o servidor aparece como `ruby`, mesmo nome de todo `gz topic`
auxiliar (C2). Com o criterio errado o assentamento era pulado e as medidas saiam vazias.
"""
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'run_b1_discretization', ROOT / 'tools' / 'run_b1_discretization.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


SERVER_PX4 = ('gz sim --verbose=1 -r -s /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/'
              'Tools/simulation/gz/worlds/default.sdf')
SERVER_PROBE = 'gz sim -s -r -v 1 empty.sdf'


def test_recognises_the_server_started_by_px4():
    assert runner.is_server_command_line(SERVER_PX4)


def test_recognises_the_server_started_by_the_construction_probe():
    assert runner.is_server_command_line(SERVER_PROBE)


def test_ignores_the_auxiliary_gz_tools():
    for line in ('gz topic -e -t /stats -n 1',
                 'gz topic -l',
                 'gz service -s /world/default/create --reqtype gz.msgs.EntityFactory',
                 '/usr/libexec/gz/transport13/gz-transport-topic -e -t /stats -n 1'):
        assert not runner.is_server_command_line(line)


def test_ignores_px4_build_and_the_sweep_itself():
    for line in ('make px4_sitl gz_x500',
                 '/bin/sh -c cmake --build /path/build/px4_sitl_default -- gz_x500',
                 'python3 ./tools/run_b1_discretization.py --output-dir results/c2/x '
                 '--links 25 --collisions --shape',
                 'ps -eo args='):
        assert not runner.is_server_command_line(line)
