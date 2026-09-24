"""
Launch file: Drone x500 + cabo (PX4 SITL + Gazebo)

Uso:
    ros2 launch pacote_do_drone start_sim.launch.py
    ros2 launch pacote_do_drone start_sim.launch.py gerar_cabo:=false
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# ============================================================
# CONFIGURAÇÃO (ajuste aqui se algum caminho mudar)
# ============================================================
HOME_DIR = os.path.expanduser("~")
PX4_DIR = os.path.join(HOME_DIR, "PX4-Autopilot")
PACOTE_DIR = os.path.join(HOME_DIR, "Drone_controlado_por_cabo", "src", "pacote_do_drone")
BUILD_TETHER_SCRIPT = os.path.join(PACOTE_DIR, "models", "build_tether.py")
MODELS_SIM_DIR = os.path.join(PACOTE_DIR, "models", "models_sim")
X500_CABO_DIR = os.path.join(MODELS_SIM_DIR, "x500_cabo")
PX4_MODELS_LINK = os.path.join(PX4_DIR, "Tools", "simulation", "gz", "models", "x500_cabo")
AUTOSTART_ID = "4022"
PX4_BIN = os.path.join(PX4_DIR, "build", "px4_sitl_default", "bin", "px4")


def preparar_ambiente(context, *args, **kwargs):
    """
    Roda antes de subir a simulação:
    - Regenera o cabo (se gerar_cabo:=true, padrão)
    - Cria o link simbólico do x500_cabo dentro do PX4, se não existir
    - Confirma que os arquivos essenciais existem
    """
    acoes = []

    gerar = LaunchConfiguration("gerar_cabo").perform(context)

    if gerar.lower() == "true":
        acoes.append(LogInfo(msg="[launch] Gerando modelo do cabo (build_tether.py)..."))
        resultado = subprocess.run(
            ["python3", BUILD_TETHER_SCRIPT],
            capture_output=True,
            text=True,
        )
        if resultado.returncode != 0:
            acoes.append(LogInfo(msg=f"[launch] ERRO ao gerar o cabo:\n{resultado.stderr}"))
            return acoes
        acoes.append(LogInfo(msg="[launch] Cabo gerado com sucesso."))
    else:
        acoes.append(LogInfo(msg="[launch] Pulando geração do cabo (gerar_cabo:=false)."))

    # Confere arquivos essenciais
    cabo_sdf = os.path.join(MODELS_SIM_DIR, "cabo", "model.sdf")
    x500_cabo_sdf = os.path.join(X500_CABO_DIR, "model.sdf")

    if not os.path.isfile(cabo_sdf):
        acoes.append(LogInfo(msg=f"[launch] ERRO: não encontrei {cabo_sdf}"))
        return acoes
    if not os.path.isfile(x500_cabo_sdf):
        acoes.append(LogInfo(msg=f"[launch] ERRO: não encontrei {x500_cabo_sdf}"))
        return acoes

    # Cria o link simbólico dentro do PX4, se ainda não existir
    if not os.path.exists(PX4_MODELS_LINK):
        acoes.append(LogInfo(msg="[launch] Criando link simbólico do x500_cabo dentro do PX4..."))
        os.symlink(X500_CABO_DIR, PX4_MODELS_LINK)
    else:
        acoes.append(LogInfo(msg="[launch] Link simbólico do x500_cabo já existe."))

    return acoes


def gerar_launch_description(context, *args, **kwargs):
    """
    Monta o ExecuteProcess do PX4 SITL, com o GZ_SIM_RESOURCE_PATH
    e o PX4_SYS_AUTOSTART configurados no ambiente do processo.
    """
    env = os.environ.copy()
    caminho_atual = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{caminho_atual}:{MODELS_SIM_DIR}" if caminho_atual else MODELS_SIM_DIR
    env["PX4_SYS_AUTOSTART"] = AUTOSTART_ID

    return [
        LogInfo(msg="[launch] Iniciando PX4 SITL + Gazebo..."),
        ExecuteProcess(
            cmd=[PX4_BIN],
            cwd=PX4_DIR,
            env=env,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gerar_cabo",
            default_value="true",
            description="Se 'true', regenera o modelo do cabo antes de subir a simulação.",
        ),
        OpaqueFunction(function=preparar_ambiente),
        OpaqueFunction(function=gerar_launch_description),
    ])