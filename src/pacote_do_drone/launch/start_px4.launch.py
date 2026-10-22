import copy
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


HOME_DIR = os.path.expanduser("~")
PX4_DIR = os.path.join(HOME_DIR, "PX4-Autopilot")
PACKAGE_DIR = os.path.join(HOME_DIR, "Drone_controlado_por_cabo", "src", "pacote_do_drone")
MODELS_SIM_DIR = os.path.join(PACKAGE_DIR, "models", "models_sim")
BUILD_TETHER_SCRIPT = os.path.join(PACKAGE_DIR, "models", "build_tether.py")

PX4_WORLD_SDF = os.path.join(PX4_DIR, "Tools", "simulation", "gz", "worlds", "default.sdf")
PX4_MODELS_DIR = os.path.join(PX4_DIR, "Tools", "simulation", "gz", "models")
CONNECTION_WORLD_SDF = os.path.join(MODELS_SIM_DIR, "sistema_cabeado", "world.sdf")
RUNTIME_WORLD_SDF = os.path.join("/tmp", "px4_cabeado_default.sdf")
PX4_BIN = os.path.join(PX4_DIR, "build", "px4_sitl_default", "bin", "px4")
AIRFRAME_DIR = os.path.join(PX4_DIR, "ROMFS", "px4fmu_common", "init.d-posix", "airframes")

DRONE_MODEL_NAME = "sistema_cabeado"
DRONE_BRIDGE_MODEL_NAME = "x500_cabo"
DRONE_AIRFRAME_MODEL_NAME = "x500_cabo"
DRONE_AUTOSTART_ID = "4022"
ROVER_MODEL_NAME = "rover_mecanum"
ROVER_BRIDGE_MODEL_NAME = "x500_cabo::rover_mecanum"
ROVER_AIRFRAME_MODEL_NAME = "rover_mecanum"
ROVER_AUTOSTART_ID = "52000"


def sincronizar_pasta(origem, destino):
    """Copia para o PX4 a versão atual dos modelos do projeto."""
    if not os.path.isdir(origem):
        raise FileNotFoundError(f"pasta de origem não existe: {origem}")
    if os.path.islink(destino) or os.path.isfile(destino):
        os.unlink(destino)
    elif os.path.isdir(destino):
        shutil.rmtree(destino)
    shutil.copytree(origem, destino)


def montar_mundo_px4():
    """Acrescenta o modelo composto ao mundo default do PX4."""
    mundo_px4 = ET.parse(PX4_WORLD_SDF)
    raiz_px4 = mundo_px4.getroot()
    raiz_px4.set("version", "1.10")
    mundo = raiz_px4.find("world")
    if mundo is None:
        raise ValueError(f"SDF do PX4 não contém <world>: {PX4_WORLD_SDF}")

    fisica = mundo.find("physics")
    if fisica is None:
        raise ValueError(f"SDF do PX4 não contém <physics>: {PX4_WORLD_SDF}")
    for nome, valor in (("max_step_size", "0.0005"), ("real_time_update_rate", "2000")):
        elemento = fisica.find(nome)
        if elemento is None:
            elemento = ET.SubElement(fisica, nome)
        elemento.text = valor

    fragmento = ET.parse(CONNECTION_WORLD_SDF).getroot()
    world_fragmento = fragmento.find("world")
    if world_fragmento is None:
        raise ValueError(f"SDF de conexão não contém <world>: {CONNECTION_WORLD_SDF}")

    for elemento in world_fragmento:
        mundo.append(copy.deepcopy(elemento))

    ET.indent(mundo_px4, space="  ")
    mundo_px4.write(RUNTIME_WORLD_SDF, encoding="utf-8", xml_declaration=True)


def abortar(acoes, mensagem):
    acoes.append(LogInfo(msg=f"[start_px4] ERRO: {mensagem}"))
    acoes.append(EmitEvent(event=Shutdown(reason=mensagem)))
    return acoes


def montar_script_inicializacao():
    """Inicia Gazebo e PX4 na janela dedicada e encerra todos juntos."""
    return f'''#!/usr/bin/env bash
set -euo pipefail
WORLD=default
export GZ_SIM_RESOURCE_PATH="{MODELS_SIM_DIR}:{PX4_MODELS_DIR}:${{GZ_SIM_RESOURCE_PATH:-}}"
SIM_PIDS=()

encerrar_simulacao() {{
  status=$?
  trap - EXIT INT TERM HUP
  echo "[start_px4] Encerrando Gazebo e as instâncias PX4..."
  for pid in "${{SIM_PIDS[@]}}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${{SIM_PIDS[@]}}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${{SIM_PIDS[@]}}"; do
    wait "$pid" 2>/dev/null || true
  done
  exit "$status"
}}
trap encerrar_simulacao EXIT INT TERM HUP

esperar_mundo() {{
  for i in $(seq 1 120); do
    if gz service -i --service "/world/$WORLD/scene/info" 2>/dev/null | grep -q "Service providers"; then
      return 0
    fi
    sleep 1
  done
  echo "[start_px4] ERRO: o mundo PX4 default não ficou disponível." >&2
  exit 1
}}

echo "[start_px4] Iniciando Gazebo com o mundo default do PX4..."
gz sim -r -v 4 "{RUNTIME_WORLD_SDF}" &
PID_GZ=$!
SIM_PIDS+=("$PID_GZ")

esperar_mundo

cd "{PX4_DIR}"
(
  export PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_NAME="{DRONE_BRIDGE_MODEL_NAME}"
  export PX4_SYS_AUTOSTART={DRONE_AUTOSTART_ID}
  exec "{PX4_BIN}" -i 0
) &
PID_DRONE=$!
(
  export PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_NAME="{ROVER_BRIDGE_MODEL_NAME}"
  export PX4_SYS_AUTOSTART={ROVER_AUTOSTART_ID}
  exec "{PX4_BIN}" -i 1
) &
PID_ROVER=$!
SIM_PIDS+=("$PID_DRONE" "$PID_ROVER")

set +e
wait -n "$PID_GZ" "$PID_DRONE" "$PID_ROVER"
status=$?
set -e
exit "$status"
'''


def preparar_e_iniciar(context, *args, **kwargs):
    acoes = []
    gerar_cabo = LaunchConfiguration("gerar_cabo").perform(context).lower() == "true"
    so_sincronizar = LaunchConfiguration("so_sincronizar").perform(context).lower() == "true"

    if gerar_cabo:
        acoes.append(LogInfo(msg="[start_px4] Gerando o modelo do cabo..."))
        try:
            resultado = subprocess.run(
                ["python3", BUILD_TETHER_SCRIPT],
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as erro:
            return abortar(acoes, f"não foi possível gerar o cabo: {erro}")
        if resultado.returncode != 0:
            return abortar(acoes, f"build_tether.py falhou:\n{resultado.stderr}")

    for caminho in (
        os.path.join(MODELS_SIM_DIR, DRONE_MODEL_NAME, "model.sdf"),
        os.path.join(MODELS_SIM_DIR, "cabo", "model.sdf"),
        CONNECTION_WORLD_SDF,
        PX4_WORLD_SDF,
        os.path.join(PX4_MODELS_DIR, ROVER_MODEL_NAME, "model.sdf"),
        PX4_BIN,
    ):
        if not os.path.exists(caminho):
            return abortar(acoes, f"arquivo ou executável não encontrado: {caminho}")

    try:
        sincronizar_pasta(
            os.path.join(MODELS_SIM_DIR, DRONE_MODEL_NAME),
            os.path.join(PX4_MODELS_DIR, "x500_cabo"),
        )
        sincronizar_pasta(
            os.path.join(MODELS_SIM_DIR, "cabo"),
            os.path.join(PX4_MODELS_DIR, "cabo"),
        )
    except OSError as erro:
        return abortar(acoes, f"falha ao sincronizar os modelos: {erro}")

    for airframe in (
        os.path.join(AIRFRAME_DIR, f"{DRONE_AUTOSTART_ID}_gz_{DRONE_AIRFRAME_MODEL_NAME}"),
        os.path.join(AIRFRAME_DIR, f"{ROVER_AUTOSTART_ID}_gz_{ROVER_AIRFRAME_MODEL_NAME}"),
    ):
        if not os.path.isfile(airframe):
            return abortar(acoes, f"airframe PX4 não encontrado: {airframe}")

    if so_sincronizar:
        acoes.append(LogInfo(msg="[start_px4] Modelos sincronizados; nenhum processo foi iniciado."))
        return acoes

    try:
        montar_mundo_px4()
    except (OSError, ET.ParseError, ValueError) as erro:
        return abortar(acoes, f"não foi possível montar o mundo PX4 com a junta SDF: {erro}")

    script_simulacao = montar_script_inicializacao()
    acoes.extend([
        LogInfo(msg="[start_px4] Abrindo um terminal para Gazebo e as duas instâncias PX4."),
        ExecuteProcess(
            cmd=[
                "gnome-terminal",
                "--wait",
                "--title=Simulação PX4 cabeada",
                "--",
                "bash",
                "-lc",
                script_simulacao,
            ],
            output="screen",
        ),
    ])
    return acoes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gerar_cabo",
            default_value="true",
            description="Regenera o modelo do cabo antes de iniciar a simulação.",
        ),
        DeclareLaunchArgument(
            "so_sincronizar",
            default_value="false",
            description="Sincroniza os modelos com o PX4 sem iniciar Gazebo ou PX4.",
        ),
        OpaqueFunction(function=preparar_e_iniciar),
    ])
