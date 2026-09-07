# Guia de execucao dos testes PX4/Gazebo

Comandos para reproduzir os testes ja executados no branch `shared`.

## Terminal 1 - preparar ambiente

```bash
cd /home/lima/codes/ic/drone-cabo
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
set +u
source /home/lima/codes/ic/drone-cabo/install/setup.bash
set -u
```

Build PX4 SITL:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
```

## Teste Etapa 0 - X500 puro

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

## Teste Etapa 3A - tether livre com 5 links

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 5 --length 0.30 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

## Teste Etapa 3B - tether livre com 10 links

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 10 --length 0.30 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

## Teste Etapa 4 - tether completo livre com 50 links

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander status
commander land
commander status
shutdown
```

## Teste Etapa 5 - tether completo ancorado

Estado atual: tentativa bloqueada por topologia de juntas no Gazebo/DART. O comando abaixo reproduz a falha registrada no plano de integracao; nao e um caso aprovado para voo.

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z --anchored --anchor-x 0 --anchor-y 0 --anchor-z -2.57
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo para observar o erro de startup:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

Erro esperado:

```text
Asked to create a joint between links [tether_anchor_link] as parent and [tether_link_50] as child,
but the child link already has a parent joint of type [BallJoint].
```

Depois de reproduzir essa tentativa, regenere a configuracao aprovada da Etapa 4 para deixar o modelo em estado executavel:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
```

## Teste de arquitetura A - tether ancorado independente

No Terminal 1, gere o tether independente:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_tether_anchor_chain.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf
```

Inicie o X500 puro:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No Terminal 2, insira o tether independente:

```bash
cd /home/lima/codes/ic/drone-cabo
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: true'
gz model -m tether_anchor_chain -l
gz topic -e -t /stats
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_attitude 1
commander land
shutdown
```

## Probe - ball joint entre modelos

Validar e executar o mundo minimo:

```bash
cd /home/lima/codes/ic/drone-cabo
gz sdf -k src/pacote_do_drone/worlds/inter_model_ball_probe.sdf
gz sim -s -r --iterations 1000 src/pacote_do_drone/worlds/inter_model_ball_probe.sdf
```

Resultado esperado: o mundo inicia sem erro bloqueante. Isso confirma `ball joint` inter-model predeclarado no `world`.

Criacao runtime de `<joint>` via `/world/create` nao e suportada nesta versao. O erro esperado e:

```text
Expected exactly one top-level <model>, <light> or <actor> on SDF.
```

## Experimento Etapa 5 - force-based com 5 links ancorados

No Terminal 1, compile o plugin e gere o tether:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf
```

Inicie o X500 puro com o caminho do plugin:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No Terminal 2, insira o tether com a ancora junto ao solo:

```bash
cd /home/lima/codes/ic/drone-cabo
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0.035}}'
gz topic -e -t /cabo/conexao/error
gz topic -e -t /cabo/conexao/force
gz topic -e -t /cabo/conexao/stats
./tools/collect_tether_force_stats.py --samples 300 --timeout 30
gz topic -e -t /stats
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander status
commander land
commander status
shutdown
```

## Terminal 2 - acompanhar Gazebo

```bash
cd /home/lima/codes/ic/drone-cabo
gz topic -l
gz topic -e -t /stats
gz model -m x500_tethered_0 -l
gz model -m x500_tethered_0 -j
```

Topicos uteis:

```bash
gz topic -e -t /world/default/pose/info
gz topic -e -t /world/default/dynamic_pose/info
gz topic -e -t /world/default/model/x500_tethered_0/link/base_link/sensor/imu_sensor/imu
gz topic -e -t /world/default/model/x500_tethered_0/link/base_link/sensor/air_pressure_sensor/air_pressure
```

Processos:

```bash
ps -o pid,ppid,stat,cmd -C px4 -C gz -C ruby
```

Logs PX4:

```bash
find /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/log -type f -name '*.ulg' | sort | tail
ulog_info /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/18_08_50.ulg
```

## Encerrar simulacao

Preferencialmente, no prompt `pxh>`:

```text
commander land
shutdown
```

Se sobrar processo:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
pkill -TERM -f 'gz sim'
```

## A0.2 - missao horizontal OFFBOARD reproduzivel (H0 / H1 / H2)

As sequencias manuais no prompt `pxh>` acima **nao** sustentam o modo OFFBOARD: o PX4 exige um stream continuo de setpoints antes e durante todo o voo. Foi essa a causa da falha do teste horizontal em A0.1. Para qualquer teste horizontal use a ferramenta versionada.

### Preparacao comum

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:${GZ_SIM_RESOURCE_PATH:-}
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
```

`GZ_SIM_RESOURCE_PATH` sem `src/pacote_do_drone/models` faz o startup de H1 falhar por modelo nao encontrado.

### H0 - X500 puro

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Em outro terminal:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/px4_offboard_horizontal_mission.py \
  --output-dir results/a0_2/h0_x500_puro --dx 0.5 --altitude 2.0 --rate 20
```

### H1 - tether livre

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

Mesma missao, com `--output-dir results/a0_2/h1_tether_livre`.

### H2 - tether ancorado com constraint force-based

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Em outro terminal, inserir o tether e conferir o sanity check estatico antes de voar:

```bash
cd /home/lima/codes/ic/drone-cabo
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0.035}}'

./tools/collect_tether_force_stats.py --samples 300 --timeout 40 --topic /cabo/conexao/stats
./tools/collect_tether_force_stats.py --samples 300 --timeout 40 --topic /cabo/anchor/stats
```

Criterio do sanity check: para `/cabo/conexao/stats` exigir `samples == requested_samples`, `invalid_messages == 0`, `timed_out == false`, `valid == true`. Para `/cabo/anchor/stats` o resultado esperado e `valid == false` com `invalid_messages == samples`: a medicao terrestre nao existe e o topico publica `NaN` justamente para nao ser confundida com forca zero.

Gravar as series e voar em paralelo:

```bash
./tools/record_tether_timeseries.py --duration 60 \
  --output-dir results/a0_2/h2_tether_ancorado --prefix h2 &
sleep 2
./tools/px4_offboard_horizontal_mission.py \
  --output-dir results/a0_2/h2_tether_ancorado --dx 0.5 --altitude 2.0 --rate 20
```

### Analise e comparacao

```bash
./tools/analyze_a0_2_h2.py --run-dir results/a0_2/h2_tether_ancorado
./tools/compare_a0_2_runs.py
```

`analyze_a0_2_h2.py` reconstroi `t_sim` a partir de `/stats`, verifica `F = -K e - C e_dot` componente a componente e escreve os PNG em `results/a0_2/h2_tether_ancorado/plots/`. Ele forca `sys.path` para o `dist-packages` do sistema porque o `matplotlib` instalado foi compilado contra numpy 1.x e o numpy 2.x local do usuario quebra a importacao.

### Encerrar

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'gz sim'
pkill -TERM -f 'make px4_sitl'
```

### Nao fazer

Nao reabilitar `Joint::EnableTransmittedWrenchCheck` no joint `anchor_world_fixed`: isso aborta o Gazebo Sim em `BallJoint::updateRelativeTransform` (backend DART) nesta topologia.

## A0.3 - baseline oficial com `tether_attach_link` fisico

A partir de A0.3 o endpoint do UAV e o link fisico `tether_attach_link` do modelo local `x500_tether_attach`, e nao mais `base_link` com offset virtual.

### Gerar os modelos

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_x500_tether_attach.py
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3

export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:${GZ_SIM_RESOURCE_PATH:-}
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
gz sdf -k src/pacote_do_drone/models/x500_tether_attach/model.sdf
gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf
```

`generate_x500_tether_attach.py` apenas **le** o modelo upstream do PX4; ele nunca escreve em `px4/PX4-Autopilot/`.

### Subir o cenario

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500_tether_attach make px4_sitl gz_x500
```

O `gz_bridge` compoe o nome de runtime como `${PX4_GZ_MODEL}_${instancia}`, ou seja **`x500_tether_attach_0`**, que e exatamente o `drone_model` configurado no plugin. Ao mudar `PX4_GZ_MODEL`, o `drone_model` do `tether_anchor_chain` tem de acompanhar.

Em outro terminal:

```bash
cd /home/lima/codes/ic/drone-cabo
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0.035}}'

./tools/collect_tether_force_stats.py --samples 300 --timeout 40 --topic /cabo/conexao/stats
```

### Voos e analise

```bash
# vertical
./tools/record_tether_timeseries.py --duration 60 --output-dir results/a0_3/vertical --prefix vertical &
sleep 2
./tools/px4_offboard_horizontal_mission.py --output-dir results/a0_3/vertical --dx 0.0 --altitude 2.0 --rate 20

# horizontal
./tools/record_tether_timeseries.py --duration 60 --output-dir results/a0_3/horizontal --prefix horizontal &
sleep 2
./tools/px4_offboard_horizontal_mission.py --output-dir results/a0_3/horizontal --dx 0.5 --altitude 2.0 --rate 20

./tools/analyze_a0_3_endpoint.py --run-dir results/a0_3/horizontal --prefix horizontal
./tools/compare_a0_2_a0_3.py
```

`analyze_a0_3_endpoint.py` calcula tambem `tau = r x F` offline, com `r = p_attach - CoM` extraido do SDF. O plugin nao aplica torque explicito: `Link::AddWorldForce(_ecm, F, p)` ja deriva o momento em torno do CoM do link.

### Reproduzir o endpoint antigo como controle

O endpoint `base_link + offset` esta depreciado, mas continua reproduzivel para comparacao controlada dentro da mesma sessao:

```bash
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3 \
  --drone-model x500_tether_attach_0 --drone-link base_link --drone-offset "0 0 -0.12"
```

Trocar a configuracao do plugin exige remover e reinserir o tether:

```bash
gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "tether_anchor_chain" type: MODEL'
```

As metricas de constraint variam de forma perceptivel **entre sessoes**, por causa do assentamento da cadeia `folded_ground`. Comparacoes finas devem ser feitas dentro da mesma sessao.

### Encerrar com verificacao

O `pkill` pode retornar antes de os processos morrerem. Um `gz sim` residual trava a proxima sessao com os topicos anunciados mas sem publicar, e o PX4 girando em lockstep. Sempre confirmar:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'gz sim'
pkill -TERM -f 'make px4_sitl'
sleep 5
ps -eo args | grep -E '[g]z sim|[b]in/px4'   # tem de nao retornar nada
```
