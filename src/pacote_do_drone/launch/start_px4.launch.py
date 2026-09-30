import os
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

# ============================================================
# CONFIGURAÇÃO (ajuste aqui se algum caminho mudar)
# ============================================================
HOME_DIR = os.path.expanduser("~")
PX4_DIR = os.path.join(HOME_DIR, "PX4-Autopilot")
PACOTE_DIR = os.path.join(HOME_DIR, "Drone_controlado_por_cabo", "src", "pacote_do_drone")

BUILD_TETHER_SCRIPT = os.path.join(PACOTE_DIR, "models", "build_tether.py")
MODELS_SIM_DIR = os.path.join(PACOTE_DIR, "models", "models_sim")

# Modelos no repositório (origem)
X500_CABO_SRC = os.path.join(MODELS_SIM_DIR, "x500_cabo")
CABO_SRC = os.path.join(MODELS_SIM_DIR, "cabo")

# Destino dentro do PX4
PX4_GZ_MODELS_DIR = os.path.join(PX4_DIR, "Tools", "simulation", "gz", "models")
X500_CABO_DST = os.path.join(PX4_GZ_MODELS_DIR, "x500_cabo")
CABO_DST = os.path.join(PX4_GZ_MODELS_DIR, "cabo")

# Airframe customizado (precisa existir e ter sido compilado)
AUTOSTART_ID = "4022"
AIRFRAME_FILE = os.path.join(
    PX4_DIR, "ROMFS", "px4fmu_common", "init.d-posix", "airframes",
    f"{AUTOSTART_ID}_gz_x500_cabo",
)
MAKE_TARGET = "gz_x500_cabo"
COMANDO_MANUAL = f"cd {PX4_DIR} && PX4_SYS_AUTOSTART={AUTOSTART_ID} make px4_sitl {MAKE_TARGET}"


def sincronizar_pasta(origem, destino):
    """Espelha 'origem' em 'destino': apaga o que existir (link, arquivo ou pasta) e copia de novo."""
    if os.path.islink(destino) or os.path.isfile(destino):
        os.unlink(destino)
    elif os.path.isdir(destino):
        shutil.rmtree(destino)
    shutil.copytree(origem, destino)


def abortar(acoes, mensagem):
    """Registra o erro e encerra o launch."""
    acoes.append(LogInfo(msg=f"[launch] ERRO: {mensagem}"))
    acoes.append(EmitEvent(event=Shutdown(reason=mensagem)))
    return acoes


def comando_terminal(terminal, script_bash):
    """Monta o comando que abre um terminal separado executando 'script_bash'."""
    if terminal == "xterm":
        return ["xterm", "-title", "PX4 SITL", "-e", "bash", "-c", script_bash]
    if terminal == "konsole":
        return ["konsole", "-e", "bash", "-c", script_bash]
    return ["gnome-terminal", "--title=PX4 SITL", "--", "bash", "-c", script_bash]


def preparar_e_iniciar(context, *args, **kwargs):
    """
    1. Regenera o cabo (se gerar_cabo:=true, padrão)
    2. Confere os arquivos essenciais
    3. Copia x500_cabo e cabo para dentro do PX4
    4. Se so_sincronizar:=false (padrão), abre o PX4 em um terminal separado.
       Se so_sincronizar:=true, para aqui e você roda o make manualmente.
    """
    acoes = []

    # ---------- 1. Gerar o cabo ----------
    gerar = LaunchConfiguration("gerar_cabo").perform(context)

    if gerar.lower() == "true":
        acoes.append(LogInfo(msg="[launch] Gerando modelo do cabo (build_tether.py)..."))
        try:
            resultado = subprocess.run(
                ["python3", BUILD_TETHER_SCRIPT],
                capture_output=True,
                text=True,
                timeout=120,
            )
        except subprocess.TimeoutExpired:
            return abortar(acoes, "build_tether.py passou de 120 s e foi interrompido.")
        if resultado.returncode != 0:
            return abortar(acoes, f"falha ao gerar o cabo:\n{resultado.stderr}")
        acoes.append(LogInfo(msg="[launch] Cabo gerado com sucesso."))
    else:
        acoes.append(LogInfo(msg="[launch] Pulando geração do cabo (gerar_cabo:=false)."))

    # ---------- 2. Conferir arquivos essenciais ----------
    for caminho in (
        os.path.join(CABO_SRC, "model.sdf"),
        os.path.join(X500_CABO_SRC, "model.sdf"),
    ):
        if not os.path.isfile(caminho):
            return abortar(acoes, f"não encontrei {caminho}")

    if not os.path.isdir(PX4_GZ_MODELS_DIR):
        return abortar(acoes, f"pasta de modelos do PX4 não existe: {PX4_GZ_MODELS_DIR}")

    if not os.path.isfile(AIRFRAME_FILE):
        acoes.append(LogInfo(
            msg=f"[launch] AVISO: airframe não encontrado em {AIRFRAME_FILE}. "
                f"O alvo '{MAKE_TARGET}' pode não existir."
        ))

    # ---------- 3. Copiar modelos para dentro do PX4 ----------
    try:
        sincronizar_pasta(X500_CABO_SRC, X500_CABO_DST)
        sincronizar_pasta(CABO_SRC, CABO_DST)
    except OSError as e:
        return abortar(acoes, f"falha ao copiar modelos para o PX4: {e}")
    acoes.append(LogInfo(msg=f"[launch] Modelos copiados para {PX4_GZ_MODELS_DIR}."))

    # ---------- 4. Subir o PX4 (ou só sincronizar) ----------
    so_sync = LaunchConfiguration("so_sincronizar").perform(context).lower() == "true"
    if so_sync:
        acoes.append(LogInfo(msg="[launch] Modo so_sincronizar: nada será iniciado."))
        acoes.append(LogInfo(msg=f"[launch] Para rodar manualmente:\n    {COMANDO_MANUAL}"))
        return acoes

    terminal = LaunchConfiguration("terminal").perform(context)

    # As variáveis são exportadas DENTRO do bash: se já existir um servidor de
    # terminal aberto (comum no gnome-terminal), ele não herdaria o env do launch.
    gz_path_atual = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_path = f"{gz_path_atual}:{MODELS_SIM_DIR}" if gz_path_atual else MODELS_SIM_DIR
    script_bash = (
        f'export GZ_SIM_RESOURCE_PATH="{gz_path}"; '
        f'export PX4_SYS_AUTOSTART={AUTOSTART_ID}; '
        f'export PX4_SIM_MODEL={MAKE_TARGET}; '
        f'cd "{PX4_DIR}" && make px4_sitl {MAKE_TARGET}; '
        f'echo; echo "[PX4 encerrou. Feche esta janela ou use o shell abaixo]"; exec bash'
    )

    acoes.append(LogInfo(msg=f"[launch] Abrindo PX4 SITL em um terminal separado ({terminal})..."))
    acoes.append(
        ExecuteProcess(
            cmd=comando_terminal(terminal, script_bash),
            output="screen",
        )
    )
    return acoes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gerar_cabo",
            default_value="true",
            description="Se 'true', regenera o modelo do cabo antes de copiar.",
        ),
        DeclareLaunchArgument(
            "so_sincronizar",
            default_value="false",
            description="Se 'true', só cria/atualiza a pasta no PX4 e não inicia a simulação.",
        ),
        DeclareLaunchArgument(
            "terminal",
            default_value="gnome-terminal",
            description="Terminal usado para o PX4: gnome-terminal, xterm ou konsole.",
        ),
        OpaqueFunction(function=preparar_e_iniciar),
    ])