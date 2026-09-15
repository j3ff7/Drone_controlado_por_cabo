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
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0}}'
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
  --timeout 5000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0}}'

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
  --timeout 5000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0}}'

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

## A1 - ground station estatica com `tether_exit_point`

A ancora ideal `world -> anchor_link` deixou de existir. O modelo `tether_anchor_chain`
agora carrega a estacao e o cabo:

```text
world --fixed--> ground_station_base --fixed--> tether_exit_point --ball--> tether_link_1..5
```

### Gerar

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_x500_tether_attach.py
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3

export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:${GZ_SIM_RESOURCE_PATH:-}
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf
```

O spawn continua em `z=0.035`, o que coloca `tether_exit_point` exatamente na pose da
ancora validada em A0.3 e apoia o plinto no solo.

### NAO habilitar `--station-collision` nesta geometria

O plinto fica na origem do mundo, o mesmo ponto onde o PX4 spawna o X500 (`-p 0,0,0`).
Com colisao, ele interpenetra o trem de pouso, empurra o UAV para fora e para baixo,
estica o cabo ate saturar a constraint e a cadeia de ball joints diverge:

```text
gz sim: ./dart/dynamics/BallJoint.cpp:159:
  BallJoint::updateRelativeTransform(): Assertion `math::verifyTransform(mT)' failed.
Aborted
```

A flag existe para quando a estacao for afastada do ponto de decolagem. Ate la, a estacao
e somente visual.

### Verificar o endpoint terrestre

```bash
gz topic -e -t /cabo/estacao/exit_pose -n 1
```

Esperado: `position { z: 0.035 }`, `orientation { w: 1 }`, constante ao longo do tempo.
Forca e momento na estacao continuam `N/D`: `/cabo/anchor/stats` publica `NaN, NaN, 0` e o
coletor devolve `valid=false`, como em A0.2/A0.3.

### Testes e comparacao

Os testes A/B/C sao os mesmos de A0.3, trocando apenas o diretorio de saida para
`results/a1/{estatico,vertical,horizontal}`. Depois:

```bash
./tools/analyze_a0_3_endpoint.py --run-dir results/a1/horizontal --prefix horizontal
./tools/analyze_a0_3_endpoint.py --run-dir results/a1/vertical --prefix vertical
./tools/compare_stages.py --out results/a1/comparacao_a0_3_a1 \
  "A0.3 ancora ideal=results/a0_3/horizontal" "A1 ground station=results/a1/horizontal"
```

## A2 - reel passivo na ground station

O `tether_exit_point` deixou de ser montado direto na base:

```text
world --fixed--> ground_station_base --revolute(reel_joint)--> reel_link
      --fixed(tether_exit_fixed)--> tether_exit_point --ball--> tether_link_1..5
```

### Gerar

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_x500_tether_attach.py
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3
```

O reel vem ligado por padrao. Parametros: `--reel-radius` (0.0175 m), `--reel-width`
(0.05 m), `--reel-mass` (0.2 kg), `--reel-damping` (5e-3 N.m.s/rad), `--reel-friction`
(0 N.m). Use `--no-reel` para reproduzir a geometria de A1.

### Observar o reel

```bash
gz topic -e -t /cabo/estacao/reel_state -n 3
```

`x = theta [rad]`, `y = omega [rad/s]`, `z = 1` quando o estado da junta esta disponivel.
Sem estado disponivel o topico publica `NaN, NaN, 0` — nunca zero fabricado. O **esforco
da junta continua `N/D`**: a unica via na API e `TransmittedWrench`, que segue descartado.

### O endpoint gira ate o equilibrio, e isso e esperado

O `tether_exit_point` nasce no topo do tambor, na pose de A1, mas o topo e um equilibrio
instavel para um ponto da borda que sustenta carga. Ele gira ~173 deg e assenta embaixo,
descendo `2r = 35 mm` para `z ~ 0`. Comportamento fisico correto, sem efeito mensuravel na
constraint. Nao tente corrigir com tuning.

### Testes

Os testes sao os de A1 com saida em `results/a2/{isolado,vertical,horizontal}`. O teste
isolado e so gravacao, sem voo:

```bash
./tools/record_tether_timeseries.py --duration 25 --output-dir results/a2/isolado --prefix isolado
```

Espere `|omega|` limitado e decaindo de forma monotonica por janelas, com `damping > 0`.

### Comparar etapas

`tools/compare_stages.py` substituiu os comparadores por etapa. Recebe pares
`rotulo=diretorio` e inclui as colunas do reel quando a corrida gravou o topico:

```bash
./tools/compare_stages.py --out results/a2/comparacao_a1_a2_horizontal \
  "A1 estacao fixa=results/a1/horizontal" "A2 reel passivo=results/a2/horizontal"
```

## Geometria do TMS remodelada — spawn passa a ser `z: 0`

A estacao deixou de ser um plinto e virou um suporte de carretel com dimensoes reais:

```text
placa quadrada 0,30 x 0,30 x 0,02 m
  + 2 hastes verticais de 0,10 m, uma de cada lado, em y = +-0,09 m
  + eixo cilindrico coaxial (raio 0,008 m, comprimento 0,22 m) no topo das hastes, z = 0,12 m
  + tambor cilindrico raio 0,07 m, largura 0,16 m, montado ENTRE as hastes
  + tether_exit_point na tangente superior do tambor, z = 0,19 m acima da base
```

Duas consequencias praticas:

1. **A origem do modelo agora fica na face inferior da placa.** O spawn passa a ser
   `pose: {position: {z: 0}}`. O antigo `z: 0.035` levantava o plinto do solo; com a
   geometria nova ele deixaria a estacao flutuando. Todos os comandos deste guia ja
   foram atualizados.
2. **O ponto de saida subiu de `z = 0,035 m` para `z = 0,19 m`.** O suporte tem altura
   fisica real, entao a equivalencia geometrica com a ancora ideal de A0.3/A1 deixa de
   valer. Comparacoes de constraint com A1 passam a ter esse deslocamento embutido.

Parametros do tambor no gerador: `--reel-radius` (0.07 m), `--reel-width` (0.16 m),
`--reel-mass` (0.2 kg), `--reel-damping`, `--reel-friction`. A base, as hastes e o eixo
sao dimensionados a partir de `reel_width` para que o tambor sempre caiba entre as hastes.

## A3 - guia fixa de saida + reel passivo independente

A raiz do cabo saiu da borda do tambor. Reel e guia agora sao ramos irmaos da base:

```text
world --fixed--> ground_station_base
       |-- revolute(reel_joint) --> reel_link             (passivo, sem carga)
       +-- fixed(tether_exit_fixed) --> tether_exit_point (guia, z = 0,19 m)
                                          --ball--> tether_link_1..5
```

### Gerar

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_x500_tether_attach.py
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3
```

A guia fixa e o padrao. Para reproduzir a montagem de A2, com a raiz do cabo na borda do
tambor, use `--exit-on-reel` — util como controle experimental.

### Verificar o desacoplamento

```bash
gz topic -e -t /cabo/estacao/exit_pose  -n 1   # deve dar z: 0.19 e nao mudar nunca
gz topic -e -t /cabo/estacao/reel_state -n 3   # theta e omega em zero
```

O `reel_state` do tambor parado sai como `z: 1` apenas: o protobuf de texto **omite
campos zero**. Isso e uma mensagem valida com `theta = 0` e `omega = 0`, nao um dado
faltante — os parsers foram corrigidos para tratar assim (ver abaixo).

### O reel nao gira nesta etapa, e isso e o esperado

Com guia fixa e comprimento constante, nada aplica torque ao tambor: ele nao tem carga.
`theta` e `omega` ficam exatamente em zero. Que a junta esta livre e nao travada se
verifica reproduzindo A2 com `--exit-on-reel`, que usa a mesma junta e os mesmos damping
e friction e gira ate `|omega| = 3,5 rad/s`. O tambor so volta a ter dinamica propria
quando o payout o acoplar ao cabo.

### Regra de validade das mensagens (corrigida nesta etapa)

Campos ausentes valem **zero**; a serializacao de texto do protobuf omite zeros. Uma
mensagem so e invalida se ficou aberta no fim do stream. O coletor, que e o gate,
adicionalmente recusa valores nao finitos — e por isso que `/cabo/anchor/stats`, que
publica `NaN`, continua com `valid=false`. O gravador **preserva** NaN, que e como a
indisponibilidade de medida chega ao CSV.

### Comparar com A2

```bash
./tools/compare_stages.py --out results/a3/comparacao_a2_a3_horizontal \
  "A2 saida na borda do tambor=results/a2_tms/horizontal" "A3 guia fixa=results/a3/horizontal"
```

## A4 - bancada de torque do reel

Bancada isolada, sem PX4, sem cabo e sem ball joints, para medir o torque no `reel_joint`.
O `reel_link` e o `reel_joint` sao copias literais do modelo de producao, extraidas pelo
gerador — um teste garante que bancada e voo nao divergem.

### Rodar a bateria completa

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh          # compila tambem libReelTorqueBench.so
./tools/run_reel_torque_bench.py --output-dir results/a4 --duration 6 --repeats 3 \
  --cases 0.0 0.02 -0.02 0.005 0.05
```

Saida: `results/a4/reel_torque_bench.json`, um CSV por caso e plots em `results/a4/plots/`.

### Como o torque de referencia e imposto

Forca constante **no frame do corpo**, num braco igual ao raio do tambor:

```text
r_body = (0, 0, R)   F_body = (Fx, 0, 0)   ->   tau_ref = R * Fx   (paralelo ao eixo Y)
```

A forca e rotacionada para o mundo a cada passo. Se fosse aplicada direto no frame do
mundo, o torque variaria com a rotacao, como gravidade num pendulo.

### Estimador adotado

```text
tau_est = I*alpha + b*omega
```

`I` (inercia em torno do eixo) e `b` (damping) sao lidos de `tether_anchor_chain/model.sdf`
por `tools/reel_bench_io.py`; `alpha` vem de diferencas centradas de `omega(t)` sobre o
**tempo simulado carimbado no header** da mensagem. Erro maximo medido: 1,75e-05 N.m.

Verificacao rapida de sanidade: em regime, `omega` tem de convergir para `tau/b`
(0,02 N.m com b = 5e-3 da 4 rad/s).

### Sonda de TransmittedWrench

```bash
./tools/run_reel_torque_bench.py --output-dir results/a4/wrench_probe \
  --probe-transmitted-wrench --cases 0.0 0.02 -0.02 0.005 0.05
```

Desligada por padrao. Nesta bancada ela **nao** derruba o simulador e acerta a magnitude
com vies de -0,89% e sinal invertido (e a reacao da junta). Isso vale **so aqui**: nao
reintroduza a API no modelo completo, onde ela abortou o DART em A0.2/A1.

### Abrir a bancada para ver

```bash
./tools/generate_reel_bench.py --torque 0.02
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins
gz sim --render-engine ogre src/pacote_do_drone/worlds/reel_torque_bench.sdf
```

## A5 - tensao no lado terrestre

`T_est` e estimada por corpo livre de **todo** o cabo, sem tocar em nenhuma ball joint:

```text
sum(m_i a_i) = F_exit + F_c + sum(m_i g)
F_exit       = sum(m_i a_i) - F_c - sum(m_i g)
t_hat        = R_link1 * (1, 0, 0)
T_est        = |F_exit . t_hat|
```

`F_c` vem da propria constraint, `m_i` dos componentes de inercia do modelo e `g` do
componente `Gravity` do mundo. **Nao** se usa `TransmittedWrench` (a junta da guia e ball,
e foi esse par que abortou o DART em A0.2/A1) nem `tau_reel/R` — a tensao tem de ser
independente do torque do reel para poder validar o atuador de A6.

### Topicos

```bash
gz topic -e -t /cabo/estacao/tensao       -n 3   # x=T_est completo, y=T_est quase-estatico, z=flag
gz topic -e -t /cabo/estacao/exit_force   -n 1   # vetor F_exit
gz topic -e -t /cabo/estacao/exit_tangent -n 1   # vetor t_hat (unitario)
```

Sem estado disponivel, `x` e `y` saem `NaN` e `z = 0` — nunca zero fabricado.

### Rodar S0, S1, S2

Cenario identico ao de A3 (guia fixa, reel passivo). Estatico:

```bash
./tools/record_tether_timeseries.py --duration 20 --output-dir results/a5/estatico --prefix estatico
./tools/analyze_a5_tension.py --run-dir results/a5/estatico --prefix estatico
```

Vertical e horizontal usam a mesma missao OFFBOARD de sempre, com saida em
`results/a5/{vertical,horizontal}` e `--dx 0.0` / `--dx 0.5`, seguidos de
`analyze_a5_tension.py` com o prefixo correspondente.

### Qual estimador usar

Use o **quase-estatico** (`y`). O completo (`x`) inclui `sum(m a)` com a aceleracao obtida
por diferenciacao de um passo de `WorldLinearVelocity`, sem filtro: o ruido passo a passo
e ~3x maior e os transientes sao amplificados. A parcela inercial descartada vale 2–5% da
media em regime. `Link::WorldLinearAcceleration` **nao** e populada para estes elos nesta
versao do gz-sim — retorna `nullopt` mesmo com `EnableAccelerationChecks`.

### Sanidade esperada

- fracao axial `T_est / |F_exit|` proxima de 1 (medida: 0,997–0,998): o cabo so transmite
  forca ao longo de si mesmo;
- `T_est` **abaixo** de `|F_uav|`, pela parcela de peso suspenso entre as duas pontas;
- `|t_hat|` = 1 exato, sem `NaN`.

### Atencao ao habilitar colisoes dos segmentos

O balanco de corpo livre supoe que as unicas forcas externas sobre o cabo sao a guia, a
constraint e o peso. Ao ligar contato do cabo com o solo ou com o UGV, o balanco deixa de
fechar e o estimador tem de ser revisto.

## A6 - reel atuado em malha aberta

O reel recebe comando de **torque** no eixo, ja depois da reducao. Torque e nao velocidade
porque e a grandeza que o motor real entrega, e porque so contra ela o estimador
`tau_est = I*alpha + b*omega` de A4 pode ser conferido sem circularidade.

```text
/cabo/tms/reel_cmd        gz.msgs.Double   comando de torque [N.m]
/cabo/tms/reel_actuator   gz.msgs.Vector3d x=comando, y=torque aplicado,
                                           z=codigo de limite (0 nenhum, 1 tau_max,
                                                               2 omega_max, 3 ambos)
```

Limites padrao (valores de **teste**, o atuador definitivo nao foi dimensionado):
`tau_max = 0.05 N.m`, `ramp_rate = 0.05 N.m/s`, `omega_max = 12 rad/s`.

### Bancada M0-M4

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh          # compila tambem libReelActuator.so
./tools/run_reel_actuator_bench.py --output-dir results/a6
```

Saida: `results/a6/reel_actuator_bench.json`, um CSV e um plot por caso. Sanidade
esperada: `omega` de regime = `tau/b` (0.03 N.m com b=5e-3 da 6 rad/s), rampa de
`tau_max/ramp_rate` segundos, e `tau_est` sobreposto ao torque aplicado.

A guarda de `omega_max` **nao dispara** com os limites padrao, porque `tau_max` ja limita
`omega` a 10 rad/s. Para exercita-la, baixe o limite:

```bash
./tools/run_reel_actuator_bench.py --output-dir results/a6/guarda_omega --omega-max 4.0
```

Ela nao freia, so para de empurrar: `omega` fica limitada com ~0,7% de ultrapassagem, mas
o torque passa a alternar liga-desliga.

### Comandar o reel no cenario completo

O atuador ja vem no modelo de producao (`--no-reel-actuator` gera sem ele). Sem comando
publicado o torque e zero e a dinamica fica identica a de A3/A5.

```bash
gz topic -t /cabo/tms/reel_cmd -m gz.msgs.Double -p "data: 0.01"    # gira num sentido
gz topic -t /cabo/tms/reel_cmd -m gz.msgs.Double -p "data: -0.01"   # e no outro
gz topic -t /cabo/tms/reel_cmd -m gz.msgs.Double -p "data: 0"       # para
gz topic -e -t /cabo/estacao/reel_state -n 2                        # theta, omega
```

### O reel ainda NAO libera cabo

Girar o tambor **nao** altera o comprimento efetivo do tether — o acoplamento e objeto de
A7. Variacao de `T_est` durante comandos do reel nesta etapa e acomodacao do cabo, nao
payout. Nao interprete como tal.

## A7 - payout/retraction (FAIL, mecanismo disponivel por flag)

**A etapa nao passou.** O modelo de producao esta na baseline validada de A6, sem payout.
O mecanismo continua disponivel por flags para retomar o trabalho.

### Gerar com payout

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh          # compila tambem libTetherPayout.so
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3 \
  --payout-kp 5 --payout-kd 0.5 --payout-force-max 3 --payout-damping 0.5
```

Sem `--payout-*` o padrao ja inclui o payout; use `--no-payout` para voltar a A6, que e como
o modelo esta versionado agora.

Parametros: `--payout-min/--payout-max` (extensao `s`, padrao [0, 1] m -> `L` em
[2,50, 3,50] m), `--payout-r-eff` (0.07 m), `--payout-rate-max` (0.5 m/s),
`--payout-drive` (`force` padrao, `velocity` DIVERGE, `none` para diagnostico),
`--payout-kp/--payout-kd/--payout-force-max`, `--payout-mass`.

### Rodar P0-P4

```bash
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins
./tools/run_payout_bench.py --output-dir results/a7            # todos os casos
./tools/run_payout_bench.py --output-dir results/a7 --only P0 P1
```

A bancada `src/pacote_do_drone/worlds/payout_bench.sdf` roda estacao + reel + guia + cabo
**sem PX4**: a constraint nao resolve o lado do UAV e nao aplica forca. Saida em
`results/a7/payout_bench.json`, um CSV e um plot por caso.

### Topicos

```bash
gz topic -e -t /cabo/tms/payout     -n 3   # x=L [m], y=L_dot [m/s], z=flag (1 batente, 2 taxa)
gz topic -e -t /cabo/tms/payout_ref -n 3   # x=theta, y=L alvo ja saturado, z=R_eff
```

Os dois saem do mesmo `PreUpdate`, entao o erro de rastreamento nao carrega erro de
correlacao entre topicos.

### O que se sabe hoje

- Prismatica **livre** (`--payout-drive none`): estavel, `L` em [2,4999999, 2,7595] m.
- `--payout-drive velocity`: **diverge**, `L` chega a 1e+117 m ja com comando zero.
- `--payout-drive force` com `Kp <= 5`: estavel; P1/P2/P3 funcionam, erro medio de
  rastreamento 0,036 m em faixa de 1 m; P0 deriva 0,114 m; **P4 diverge**.
- Massa do elo de payout e ganhos do PD foram descartados como causa por medicao
  (testados 0,03 / 0,15 / 0,30 kg e varios pares Kp/Kd).
- **Limite fisico:** segurar a tracao medida de ~1,3 N exige `T*R_eff = 0,091 N.m`, acima
  do `tau_max = 0,05 N.m` de A6. O reel e back-driven pela carga.

Retomar por: (1) redimensionar torque/raio ou incluir freio; (2) trocar o PD de posicao
por acoplamento de transmissao, aplicando `tau_reel/R_eff` na prismatica com reacao no
tambor; (3) avaliar passo de fisica menor. **A8 fica bloqueada ate A7 passar.**

## B1 - sweep de discretizacao (FAIL, nao concluido)

O objetivo era variar N com `L = 2,5 m` fixo e `m_link = rho*L/N`. **Nao foi possivel medir:**
as formas iniciais N-independentes desestabilizam a cadeia de ball joints. Ver B1 no plano.

### Formas iniciais disponiveis

```text
--initial-axis folded_ground   zigue-zague aberto, SO N=5, validada de A0.1 a A6
--initial-axis coil            poligono regular fechado, horizontal   (INSTAVEL)
--initial-axis loop            poligono regular fechado, vertical     (INSTAVEL)
--initial-axis x | z           reta horizontal / vertical
```

`coil` e `loop` sao N-independentes por construcao — mesma circunferencia de perimetro `L`
para todo N, verificado em `test/test_coil_initial_shape.py`. O problema nao e a geracao:
as poses saem limpas, com giro constante de `2*pi/N` por junta.

### Reproduzir o sweep

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/run_b1_discretization.py --output-dir results/b1 --links 5 10 20 25
```

O runner regenera o modelo, sobe PX4/Gazebo headless, insere o cabo, deixa assentar
(`--settle`, padrao 25 s), grava (`--measure`, padrao 20 s) e encerra, por N. Saida:
`results/b1/b1_discretization.json` e `.md`, com os CSV por configuracao em `results/b1/nNN/`.

### Reproduzir o controle e os casos que abortam

```bash
# controle validado: sobrevive 50 s
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3

# casos que abortam o DART (coil N=5 sobrevive 15 s mas aborta em ~45 s)
./tools/generate_tether_anchor_chain.py --links 5  ... --initial-axis coil
./tools/generate_tether_anchor_chain.py --links 10 ... --initial-axis coil
./tools/generate_tether_anchor_chain.py --links 25 ... --initial-axis loop
```

Depois, em cada caso:

```bash
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins
cd px4/PX4-Autopilot && HEADLESS=1 PX4_GZ_MODEL=x500_tether_attach make px4_sitl gz_x500
# noutro terminal, apos "Ready for takeoff!":
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0}}'
sleep 50 && gz topic -e -t /cabo/conexao/stats -n 2   # vazio => abortou
```

O aborto aparece no log do PX4 como
`dart/dynamics/BallJoint.cpp:159: ... Assertion 'math::verifyTransform(mT)' failed.`

### Armadilha de limpeza de processos

**Nao** verifique processos residuais com `pgrep -f 'gz sim|bin/px4'`: o padrao casa com a
propria linha de comando de qualquer shell que o contenha, a lista nunca esvazia e o PX4 da
corrida anterior sobrevive para colidir com a proxima — foi o que contaminou as primeiras
tentativas de sweep. Use o nome do executavel:

```bash
ps -eo comm= | grep -E '^(px4|gz|ruby)$'
```

## B1 rodada 2 - discretizacao com cabo esticado (PASS)

A rodada 1 usava `coil`/`loop`, formas fechadas que viram laco folgado e derrubam o DART.
**Nao use `coil` nem `loop` como condicao inicial deste sweep.** A forma correta e `taut`.

### Condicao inicial `taut`

Os N elos sao distribuidos no arco circular que liga o `tether_exit_point` ao
`tether_attach_link`, com N cordas de comprimento exato `l = L/N`. O UAV e afastado para
`D = 2,38 m`, o que da 4,74% de folga e tracao estimada de ~1,4 N (46% de `Fmax`).

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_tether_anchor_chain.py --links 20 --length 2.50 --rho 0.06 --radius 0.003 \
  --initial-axis taut --taut-target "2.38 0 -0.083" \
  --force-constraint --stiffness 5 --damping 0.5 --max-force 3 --link-collisions
```

`--taut-target` e o attach do UAV **relativo ao ponto de saida** (a guia esta em z=0,19 m e
o attach em z~0,107 m, dai o -0,083). Omita `--link-collisions` para o caso sem colisoes.

### Sweep B1.1 (sem colisoes) e B1.2 (com colisoes)

```bash
# B1.1 - sem colisoes
./tools/run_b1_discretization.py --output-dir results/b1/sem_colisoes \
  --links 5 6 7 8 10 20 25 --settle 30 --measure 20

# B1.2 - com colisoes
./tools/run_b1_discretization.py --output-dir results/b1/com_colisoes \
  --links 5 8 10 20 25 --collisions --settle 30 --measure 20
```

O runner regenera o modelo, sobe PX4/Gazebo headless com `PX4_GZ_MODEL_POSE` afastando o
UAV, insere o cabo, assenta, grava e encerra, por N. Saida em
`results/b1/<serie>/b1_discretization.json` e `.md`, com CSV em `results/b1/<serie>/nNN/`.

### Teste vertical

```bash
# baseline N=5 sem colisoes, e depois a recomendada N=20 com colisoes
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins
export HEADLESS=1 PX4_GZ_MODEL=x500_tether_attach PX4_GZ_MODEL_POSE="2.38,0,0,0,0,0"
cd px4/PX4-Autopilot && make px4_sitl gz_x500
# noutro terminal, apos "Ready for takeoff!": inserir o cabo (comando de sempre), esperar
# 25 s de assentamento e entao
./tools/px4_offboard_horizontal_mission.py --output-dir results/b1/voo --dx 0.0 --altitude 2.0 --rate 20
```

### Resultado que importa

Sem colisoes o teto e `l = 0,3125 m` (N=8); acima disso o DART aborta. **Com colisoes dos
segmentos todos os N testados ate 25 sao estaveis, e o RTF nao muda** (0,9963–0,9965 nos
dois casos). As colisoes deixaram de ser um custo a adiar e passaram a ser um requisito de
robustez. Recomendado para B2: `l = 0,125 m` com colisoes.

## B2 - escalabilidade por comprimento e discretizacao (FAIL)

Baseline: `rho = 0,06 kg/m`, colisoes **habilitadas**, condicao inicial `taut` com
`D/L = 0,953`. **Nenhuma configuracao com `L > 2,5 m` ficou estavel.**

### Sweep estatico

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh

# L = 2,5 m, varredura de N (o runner escala D com L via --taut-ratio)
./tools/run_b1_discretization.py --output-dir results/b2/L2p5_fino \
  --links 5 8 10 20 25 30 40 --length 2.5 --collisions \
  --settle 25 --measure 15 --settle-wall-cap 150

# L maior, mesma resolucao nominal
./tools/run_b1_discretization.py --output-dir results/b2/L5_l0p1 \
  --links 50 --length 5.0 --collisions --settle 25 --measure 15
```

O assentamento agora e contado em **tempo simulado** (`--settle`), com teto de wall-clock
(`--settle-wall-cap`): dormir em wall-clock faria as configuracoes lentas assentarem menos
que as rapidas e a comparacao entre N deixaria de ser justa. O runner tambem detecta a
morte do `gz sim` durante o assentamento e nao espera o teto inteiro.

### Uma configuracao individual

```bash
./tools/generate_tether_anchor_chain.py --links 25 --length 5.0 --rho 0.06 --radius 0.003 \
  --initial-axis taut --taut-target "4.765 0 -0.083" \
  --force-constraint --stiffness 5 --damping 0.5 --max-force 3 --link-collisions

export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins
export HEADLESS=1 PX4_GZ_MODEL=x500_tether_attach PX4_GZ_MODEL_POSE="4.765,0,0,0,0,0"
cd px4/PX4-Autopilot && make px4_sitl gz_x500
# noutro terminal: inserir o cabo com o gz service de sempre e observar
gz topic -e -t /cabo/conexao/stats -n 2      # vazio => abortou
```

`--taut-target` = attach do UAV relativo a guia; o x deve ser `0,953 * L`.

### Tabelas e graficos

```bash
./tools/analyze_b2_scalability.py --results-dir results/b2
```

Gera `results/b2/b2_scalability.{md,json}` e os graficos em `results/b2/plots/`:
`rtf_vs_n.png`, `rtf_vs_length.png`, `rtf_vs_segment.png`, `tension_vs_segment.png`,
`force_vs_segment.png`, `error_vs_segment.png`, com uma curva por `L`.

### Resultados que importam

- RTF plano ate `N = 25` (1,003 s de parede por segundo simulado), joelho abrupto em
  `N = 30` (RTF 0,885), aborto em `N = 40`. Nao e degradacao suave.
- `L = 5 m` aborta com `l = 0,10`, com `l = 0,20` e tambem com `K/C/Fmax` dobrados.
- Peso do cabo atinge 98% de `Fmax = 3 N` em `L = 5 m` e 196% em `L = 10 m`: a constraint
  foi dimensionada para 2,5 m. Escalar `Fmax` sozinho **nao** resolve — ha segunda causa.
- **`l ~ 0,02 m` nao e viavel**: exigiria `N = 125` em `L = 2,5 m`, 4x acima da fronteira.
- `T_est` converge entre `N = 20` e `N = 25` (0,8% de diferenca).

Nao execute a matriz inteira as cegas: `L = 7,5` e `L = 10 m` foram deliberadamente
puladas depois que `L = 5 m` falhou na discretizacao grossa e na fina.

## C1-C4 - tether com revolutes alternadas (DESCARTADA, codigo removido)

A alternativa com `RevoluteJoint` de eixo alternado foi testada em C1-C4, apresentou
regressao (anisotropia de 9% a 25% em `|F_uav|` e de 56% a 68% no desvio lateral,
controlada por um angulo sem significado fisico) e **foi descartada**. As flags
`--joint-type` e `--revolute-roll` do gerador e do runner **nao existem mais**, assim como
`tools/analyze_c_revolute.py`, `tools/compare_anisotropy.py` e
`test/test_revolute_chain_model.py`. Resultados e analise no plano, secao **C1–C4**. Uma
copia do codigo removido, apenas como registro, fica fora do git em
`results/c/revolute_alt_removido/` (patches reversos + os tres arquivos).

Continuam disponiveis, por serem independentes da topologia de junta:

- `--initial-axis x` no runner (cabo reto; unica classe que constroi acima de `N ~ 34`);
- `--drone-y`, `--transient` e `--shape` no runner (perturbacao lateral, gravacao logo
  apos o spawn e forma do cabo via `tools/capture_tether_shape.py`);
- a sonda de construcao e o classificador de abortos abaixo;
- as duas correcoes do runner (timeout em `gz topic -e` e deteccao do servidor gz).

### Sonda de construcao (sem PX4, sem UAV)

Separa **construcao do modelo** de dinamica. Cada linha sobe um `gz sim` novo num mundo
vazio, insere o cabo e julga 4 s depois:

```bash
./tools/probe_model_construction.py --links 30 32 33 34 35 36 40
./tools/probe_model_construction.py --links 40 --no-collisions
```

Resultados e analise em `results/construcao/resumo.md`. Use esta sonda **antes** de gastar
um sweep completo: um aborto de construcao nao depende de PX4, de voo nem de tempo de
simulacao, e custa 15 s em vez de 4 minutos.

### Classificar os abortos

O resumo do sweep so diz `estavel = NAO`. A causa esta no `px4.log`, e a distincao muda o
que fazer a seguir:

```bash
./tools/classify_aborts.py --indir results/b2 results/c1 results/c2 results/c3 results/c4
```

- `Joint.cpp:537` com `ConstructSdfJoint` -> o modelo **nunca foi instanciado**; reproduza
  em 15 s com a sonda acima, sem PX4.
- `BallJoint.cpp:159` ou `RevoluteJoint.cpp:186` com `PhysicsPrivate::Step` -> **divergiu
  simulando**; so aparece rodando, e depende de condicao inicial, `l`, `L` e topologia.

### Duas armadilhas do runner corrigidas nesta rodada

- **`gz topic -e -n 1` nao retorna se o servidor morreu.** Sem teto, um aborto durante a
  construcao trava o sweep inteiro no primeiro `sim_time_now()`; foi o que consumiu a
  primeira tentativa de C2 e o que deixou os sweeps abortados de B2 sem `sweep.json`.
  Agora ha timeout e o relatorio e escrito **a cada configuracao**, nao so no fim.
- **`ps -eo comm=` nao identifica o servidor gz.** O executavel do `gz` e um wrapper ruby:
  o servidor aparece como `ruby`, exatamente como todo `gz topic` auxiliar. Com o criterio
  antigo (`'gz' in comm`) o `sim_alive()` dava falso negativo, o assentamento era pulado
  em 8 ms e — depois que passei a condicionar a gravacao a ele — as medidas saiam
  **vazias**. O criterio agora e a linha de comando do servidor
  (`gz sim ... -s ... .sdf`), coberto por `test/test_sim_alive_detection.py`.

## Restauracao pos-C - checkpoint BallJoint

Revalidacao da baseline depois de remover as revolutes. Build, estatico e voo:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins

# SDF da configuracao N=20 (ball, colisoes, taut) e validacao
./tools/generate_tether_anchor_chain.py --links 20 --length 2.5 --rho 0.06 --radius 0.003 \
  --initial-axis taut --taut-target "2.3825 0 -0.083" --link-collisions \
  --force-constraint --stiffness 5 --damping 0.5 --max-force 3
gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf

# estatico
./tools/run_b1_discretization.py --output-dir results/restauracao/estatico --links 20 \
  --length 2.5 --rho 0.06 --collisions --shape --settle-wall-cap 240
```

Voo vertical curto: suba PX4 como no teste vertical de B1, insira o cabo, espere 25 s e rode
`./tools/px4_offboard_horizontal_mission.py --output-dir <dir> --dx 0.0 --altitude 2.0 --rate 20`
com `./tools/record_tether_timeseries.py --duration 75 --output-dir <dir> --prefix vertical`
gravando em paralelo.

**Atencao — a ferramenta de missao nao detecta a morte do simulador.** Se o `gz sim` abortar
durante o voo, ela reporta `failed = false` e o UAV aparece congelado no ar. Confira sempre:

```bash
python3 -c "import json; m=json.load(open('<dir>/vertical_record_manifest.json')); print(m['sim_time_span_s'], 'de', m['duration_s'])"
grep -c "Assertion" <dir>/px4.log      # tem que ser 0
```

Resultado da restauracao: `N = 20` com colisoes aborta nas 3 corridas, com 7 a 14 s simulados
cobertos; `N = 5` sem colisoes cobre 74,6 de 75 s. A corrida N=20 de B1 tambem tinha
abortado (9,9 s de 60), sem ser percebida.

O `model.sdf` de producao fica no estado anterior a C (`N = 5`, `folded_ground`, sem
colisoes); regenere com o comando acima para usar `N = 20`.

## Teste UniversalJoint - N=10

Troca apenas o bloco da junta entre elos: `type="universal"` com `axis = (0,1,0)` e
`axis2 = (0,0,1)`, ambos no frame do elo filho (`expressed_in="tether_link_i"`), sem limite,
sem damping. Elos, massas, inercias, colisoes, poses, constraint, PX4, `tether_attach_link`
e `tether_exit_point` identicos. `--joint-type ball` continua o padrao do gerador; a saida
em `ball` e byte a byte igual a de antes desta rodada.

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
export GZ_SIM_RESOURCE_PATH=$PWD/src/pacote_do_drone/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/gz_plugins

# gerar e validar o SDF com universal
./tools/generate_tether_anchor_chain.py --links 10 --length 2.5 --rho 0.06 --radius 0.003 \
  --initial-axis taut --taut-target "2.3825 0 -0.083" --link-collisions \
  --force-constraint --stiffness 5 --damping 0.5 --max-force 3 --joint-type universal
gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf

B="--links 10 --length 2.5 --rho 0.06 --collisions --shape --settle-wall-cap 240"
# estatico, ball x universal
./tools/run_b1_discretization.py --output-dir results/universal/estatico_ball      $B --joint-type ball
./tools/run_b1_discretization.py --output-dir results/universal/estatico_universal $B --joint-type universal
# flexibilidade 3D: UAV deslocado 0,15 m em +y (fora do plano do sag)
./tools/run_b1_discretization.py --output-dir results/universal/lateral_ball \
  $B --transient 12 --drone-y 0.15 --joint-type ball
./tools/run_b1_discretization.py --output-dir results/universal/lateral_universal_r0 \
  $B --transient 12 --drone-y 0.15 --joint-type universal --universal-roll 0
./tools/run_b1_discretization.py --output-dir results/universal/lateral_universal_r45 \
  $B --transient 12 --drone-y 0.15 --joint-type universal --universal-roll 45
```

`--universal-roll 45` gira o par de eixos em torno do proprio cabo sem mudar mais nada: se a
resposta lateral mudar com esse angulo, os dois eixos estao introduzindo comportamento
artificial (foi o que reprovou as revolutes alternadas em C3). Para visualizar, use o
procedimento de sempre com o SDF gerado acima e **nao decole** — ver a nota de voo no plano.

Resultado (duas corridas por caso lateral; cada caso se repete com ate 0,4%): em repouso,
universal e ball sao identicas ate a quarta casa. Sob o deslocamento lateral, a universal da
`|F_uav|` 0,935 N com eixos a 0 graus e 1,084 N com eixos a 45 graus (15,8%), contra 0,968-0,972 N
da ball. **FAIL** por anisotropia; detalhes no plano, secao **Teste UniversalJoint - N=10**.
Os picos do transitorio variam entre repeticoes e nao servem para comparar casos.

**Nota de voo.** Com o UAV a 2,38 m da guia e 2,5 m de cabo, o cabo fica reto quando o
drone sobe ~0,84 m, mas a missao vertical sobe 2 m (exigiria 3,08 m de cabo). As quedas em
voo acontecem nessa subida. Ate a geometria do teste de voo ser corrigida, uma "falha em
voo" nao diz nada sobre o tipo de junta ou o numero de elos.

## Tether ligado ao X500 por juntas fisicas (arquitetura da `dev`)

Analise e decisao em `docs/TETHER_X500_CONNECTION_ANALYSIS.md`. Aqui nao ha
`TetherForceConstraint`: a ponta do cabo e ligada ao `base_link` do X500 por uma junta
`ball` de mundo, com pivo no `tether_attach_link`, e a raiz do cabo fica na guia fixa da
estacao. Juntas internas `ball`, colisoes, reel estatico sem atuador, sem prismatica.

### Gerar o mundo

```bash
cd /home/lima/codes/ic/drone-cabo
# X500 a 0,5 m da estacao: com 2,5 m de cabo, 2 m de subida e 0,5 m de deslocamento cabem
./tools/generate_x500_tether_world.py --drone-x 0.5
export GZ_SIM_RESOURCE_PATH=$PWD/src/pacote_do_drone/models:$PWD/px4/PX4-Autopilot/Tools/simulation/gz/models
gz sdf -k src/pacote_do_drone/worlds/x500_tether_joint.sdf
```

O gerador recusa uma posicao em que o cabo nao alcance o drone, mas **nao** confere se a
missao cabe no alcance do cabo. Confira antes de voar: com o pivo do drone em
`(D, 0, 0,107)` e a guia em `(0, 0, 0,19)`, a distancia no hover e
`sqrt(D^2 + dy^2 + (h + 0,107 - 0,19)^2)`, onde `h` e a subida real — some ~0,2 m ao alvo da
missao, porque a estimativa do PX4 costuma nascer deslocada. Mantenha abaixo de ~90% de `L`.

### Estatico (sem PX4)

```bash
gz sim -s -r src/pacote_do_drone/worlds/x500_tether_joint.sdf &       # tire o -s para ver
./tools/record_tether_connection.py --duration 45 --output-dir results/x500_joint/estatico \
  --links 10 --segment-length 0.25 --settle 15
```

### Voo: o PX4 se liga ao X500 que ja esta no mundo

```bash
# terminal 1: mundo (com interface grafica)
gz sim -r src/pacote_do_drone/worlds/x500_tether_joint.sdf

# terminal 2: PX4 sem spawnar outro drone
cd px4/PX4-Autopilot
PX4_GZ_MODEL_NAME=x500_tether_attach_0 HEADLESS=1 make px4_sitl gz_x500
# esperar "Ready for takeoff!"

# terminal 3: gravacao + missao
cd /home/lima/codes/ic/drone-cabo
./tools/record_tether_connection.py --duration 75 --output-dir results/x500_joint/vertical \
  --links 10 --segment-length 0.25 --settle 5 &
./tools/px4_offboard_horizontal_mission.py --output-dir results/x500_joint/vertical \
  --dx 0.0 --altitude 2.0 --rate 20
# horizontal: mesmo procedimento com --dx 0.5 (x local do PX4 = norte = +y do Gazebo)
```

**Nao confie so no `failed` da missao.** A ferramenta nao percebe se o simulador morreu.
Confira `sim_time_covered_s` no `conexao_summary.json` (tem de cobrir a gravacao),
`grep -c Assertion` no log do Gazebo e `grep -i failsafe` no log do PX4.

`tools/run_x500_joint_campaign.sh [D]` encadeia estatico, vertical e horizontal com esses
criterios e so executa o horizontal se o vertical passar. Resultados em
`results/x500_joint/D<D>/`.

Metricas: `pivot_gap_m` e o erro de conexao de uma junta (distancia ponta do cabo → pivo;
integra ≈ 0). `|F_uav|` e `T_est` **nao existem** nesta arquitetura: eram calculados pelo
plugin de forca, e ler o wrench da junta via `TransmittedWrench` ja derrubou o DART nesta stack.

### Resultado e cuidados (rodada X1)

- Estatico **PASS** com X500 a 0,5 e a 1,2 m; vertical **FAIL** nas duas geometrias, e o
  controle com a constraint de forca na mesma geometria tambem aborta. O gatilho comum e o
  cabo **com colisoes** sendo arrancado do solo durante a subida. Detalhes e linha do tempo em
  `docs/TETHER_X500_CONNECTION_ANALYSIS.md`.
- **Sem colisoes nos elos o mundo nem se sustenta estaticamente** com essa folga: o laco inicial
  cai livre atraves do solo e o DART aborta (`GenericJoint::addChildBiasForceToDynamic`).
- **Spawn pelo PX4 afunda o X500**: `PX4_GZ_MODEL_POSE` com z = 0 sobrescreve a pose do modelo e
  o `base_link` repousa a z ≈ 0,02 m (contra 0,227 m quando o X500 e incluido no mundo). Nos voos
  com a constraint de forca isso poe a ponta do cabo abaixo do solo desde o inicio.
- **`pkill -f` com um padrao que aparece no proprio comando mata o shell que o executa.** Rode a
  limpeza de dentro de um script (como `tools/run_x500_joint_campaign.sh`) ou mate por nome de
  executavel (`ps -eo pid,comm`).
- A campanha recusa iniciar o PX4 se o mundo ja morreu: sem isso o PX4 sobe o proprio `default`,
  sem o X500, e a missao so expira.

## Angulos do tether junto ao UAV (conexao por plugin de forca)

O `TetherForceConstraint` publica, so leitura, a direcao local do cabo junto ao drone:

| Topico | Conteudo |
| --- | --- |
| `/cabo/conexao/tangent_body` | tangente que sai do drone ao longo do ultimo elo, frame do drone (x frente, y esquerda, z cima) |
| `/cabo/conexao/angles` | `x` = azimute, `y` = elevacao (-90 = cabo reto abaixo), `z` = angulo entre a forca e a tangente, em graus |
| `/cabo/conexao/force_body` | forca da conexao sobre o drone, frame do drone |

```bash
./tools/build_tether_force_plugin.sh
tools/run_tether_angle_check.sh plugin_only        # estatico 30 s + voo vertical curto
gz topic -e -t /cabo/conexao/angles                 # com a simulacao rodando
```

O `tools/record_tether_timeseries.py` grava os tres topicos (`<prefixo>_angles.csv` etc.), e
`tools/record_tether_connection.py` calcula os mesmos angulos a partir das poses do Gazebo, como
checagem independente. Com o cabo quase vertical o azimute e mal condicionado e nao deve ser
interpretado. Analise: `docs/TETHER_CONNECTION_FORCE_VS_BALLJOINT.md`.

Duas armadilhas corrigidas nesta rodada: o gravador escrevia so uma lista fixa de topicos (os
novos eram recebidos e descartados), e um `wait` sem PID no script esperava tambem o PX4, que
nao termina.

## X500 + tether por `BallJoint` via wrapper no launch (sem modificar o PX4)

Arquitetura, resultados e comparacao em `docs/X500_TETHER_BALL_JOINT_INTEGRATION.md`. O PX4 spawna
um modelo wrapper gerado a cada execucao: `<include merge="true">` do `x500` original + `include`
do `model://tether_cable` (portado da `dev`, em `src/pacote_do_drone/tether_package/models/tether_cable`) com pose
`0 0 0.2` relativa ao `base_link` + junta `ball` `base_link` → `cabo_anexado::raiz_cabo`. Nenhum
arquivo do PX4 e editado e nao ha overlay permanente. Sem carretel, sem ancora, sem plugin de
forca; a integracao por plugin continua como estava.

```bash
cd /home/lima/codes/ic/drone-cabo
unset GZ_SIM_RESOURCE_PATH PX4_GZ_MODEL_NAME PX4_GZ_MODEL_POSE   # o lancador monta o proprio caminho
./tools/launch_x500_tether_ball.py --only-generate --print-env    # gera e valida (gz sdf -k)
./tools/launch_x500_tether_ball.py                                # PX4 + Gazebo com GUI
./tools/launch_x500_tether_ball.py --headless                     # sem GUI
```

Em outro terminal, depois de `Ready for takeoff!`:

```bash
gz model --list                                         # ground_plane, x500_tether_ball_0 (sem cabo duplicado)
gz model -m x500_tether_ball_0 | grep -A4 drone_cabo_joint     # Type: ball, Parent Link: base_link
./tools/record_x500_tether_ball.py --duration 30 --output-dir results/x500_tether_ball_shared/manual/estatico
```

Campanha (estrutura, estatico com captura da GUI, vertical, horizontal so se o vertical passar):

```bash
TIME_SCALE=3 tools/run_x500_tether_ball_campaign.sh ts3   # results/x500_tether_ball_shared/ts3/
cat results/x500_tether_ball_shared/ts3/vertical/flight_metrics.json
```

Metricas: `dist_root_base_m` (integridade da junta, 0,2 m), `azimuth_deg`/`elevation_deg` do 1º
segmento no frame do drone, `z_min_cable_m` (penetracao no solo), `sim_time_covered_s`, RTF; na
analise do voo, dx realizado, erro de retorno, RMS XY/Z, roll/pitch e os angulos por fase.

Cuidados:

- **`TIME_SCALE`**: 72 elos com colisao deixam o RTF em 0,3–0,5, e a missao OFFBOARD conta tempo
  de parede. Com `TIME_SCALE=1` a subida invade a janela de hover e o RMS Z nao significa nada.
- **Penetracao no solo**: o cabo arrastado atravessa o solo (colisao de 1,5 mm); a analise reprova
  `z_min < -5 mm`. Um voo com o cabo enterrado termina em chicote e aborto do DART ao arranca-lo.
- Poses por `/world/default/dynamic_pose/info` (o `pose/info` nao atualiza links aninhados);
  `gz sdf -p` omite os includes do wrapper sem erro.
- GUI invisivel a partir de processo em segundo plano: `--headless` + `gz sim -g --render-engine-gui ogre`.

### Resultado (rodada de 2026-09-14)

- **Estrutura e estatico: PASS** nas duas campanhas. Modelo unico `x500_tether_ball_0`, junta `ball`
  `base_link` → `cabo_anexado::raiz_cabo`, raiz ↔ `base_link` 0,2000008 m, elevacao do 1º segmento
  ~-67 graus, `z_min` 1,4 mm, RTF 0,30–0,35. PX4 com 0 mudancas.
- **Vertical (`ts3`): FAIL** so pela penetracao do cabo. Sem failsafe nem aborto, RMS XY 0,047 m,
  RMS Z em hover 0,034 m, roll/pitch ≤ 1,9 graus, junta integra; `z_min` -173 mm no pouso.
- **Horizontal: nao executado em `ts3`** (bloqueado pelo vertical). Na `baseline`, executado antes
  do criterio de penetracao, partiu com o cabo enterrado e abortou
  (`collision_space.cpp:460`, `aabbBound`).
- A baseline de voo continua sendo o plugin de forca; o wrapper so e usado quando chamado.

## Baseline oficial — tether por forca + angulos no frame do drone (guia do operador)

Baseline consolidada em 2026-09-15. Documentacao tecnica (links, lei da forca, `t_hat_world`,
`R_BW`, `t_hat_body`, convencoes, mascaras, requisito de 1 ms, limitacoes) em
`docs/TETHER_FORCE_AND_ANGLE_MEASUREMENT.md`.

| Item | Valor |
| --- | --- |
| Conexao drone–tether | `TetherForceConstraint` (forca na ponta de `tether_link_5`, reacao em `tether_attach_link`) |
| Angulos | tangente local `t_hat_world` transformada `world → body` (`t_hat_body = R_BW t_hat_world`) |
| Janela da tangente | 0,15 m (4 pontos) |
| Cabo | N = 5, L = 2,5 m, `folded_ground`, colisao nos elos (raio 2,25 mm) |
| Constraint | K = 5 N/m, C = 0,5 N·s/m, Fmax = 3 N |
| Mascaras de colisao | X500 = 1, tether = 2, solo = 65535 (padrao) |
| Passo de fisica | `max_step_size = 0.001 s` (copia do mundo `default` do PX4) |
| Spawn do X500 | `PX4_GZ_MODEL=x500_tether_attach_mask`, `PX4_GZ_MODEL_POSE=0,0,0.24,0,0,0` |
| Missoes | `--relative-altitude` |

> **Nao use `make px4_sitl gz_x500` sozinho como procedimento.** Sem um mundo ja rodando, o PX4
> inicia o `default.sdf` com passo de 4 ms, spawna o X500 sem mascara e na pose padrao — nao e a
> configuracao validada (a 4 ms o cabo penetra 2,9 mm no pouso). O `make` so e usado depois que o
> mundo de 1 ms esta de pe, e o PX4 se liga a ele (`gazebo already running world: default`).

> **Atencao:** `tools/run_tether_attitude_bench.sh` e `tools/run_tether_angle_collision_campaign.sh`
> encerram **todos** os processos `px4`, `gz` e `ruby` ao comecar e ao terminar. Nao rode uma
> campanha com uma sessao manual aberta.

### E0 — build e preparacao

```bash
cd /home/lima/codes/ic/drone-cabo

./tools/build_tether_force_plugin.sh          # build/gz_plugins/libTetherForceConstraint.so (+ 3 plugins)

(
  cd px4/PX4-Autopilot
  make px4_sitl                               # so compila; nao inicia a simulacao
)

python3 -m pytest -q -p no:cacheprovider test/
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/pacote_do_drone:src/cabo_avaliacao \
  python3 -m pytest -p no:cacheprovider -q \
  src/pacote_do_drone/test/test_angulos_cabo.py src/cabo_avaliacao/test/test_cenarios.py

# alguns testes regeneram modelos versionados: restaurar e conferir
./tools/generate_x500_tether_world.py --drone-x 0.5
./tools/generate_x500_tether_attach.py
git status --short src/pacote_do_drone/models src/pacote_do_drone/worlds     # deve ficar vazio
```

### E1 — bancada de atitude (T0/T1, sem PX4)

Corpo estatico no lugar do drone, cabo pendurado, uma sessao do Gazebo por atitude (~4 min):

```bash
tools/run_tether_attitude_bench.sh t1

python3 tools/analyze_tether_attitude_bench.py \
  --bench-dir results/tether_angles/bench/t1
```

Casos: `level`, `roll_p15`, `roll_m15`, `pitch_p15`, `pitch_m15`, `yaw_45`, `combined` (roll 10°,
pitch -10°, yaw 30°). Um caso avulso: `tools/run_tether_attitude_bench.sh roll20 roll20:20:0:0`.
Saidas em `results/tether_angles/bench/t1/<caso>/` (`plugin_*.csv`, `connection.csv`, `case.txt`,
`aborts.txt`) e `t1_analysis.json`. Esperado: `T1 PASS`; `t_hat_world` igual entre atitudes
(< 0,05°), `t_hat_body` = `R_BW t_hat_world` (< 0,01°).

### E2 a E5 — campanhas automatizadas

**B. Campanha de colisao** — C0 repouso, C1 arraste lento de 1 m a 0,1 m/s a 0,8 m, C2 voo e pouso:

```bash
STEP=0.001 \
DRONE_MASK=1 \
CABLE_MASK=2 \
tools/run_tether_angle_collision_campaign.sh collision step1ms
```

**C. Campanha integrada** — **S0** estatico (30 s pousado), **S1** vertical (arm → takeoff → hover
a 1,8 m → land → disarm), **S2** horizontal (hover → dx = 0,5 m → retorno → pouso):

```bash
STEP=0.001 \
DRONE_MASK=1 \
CABLE_MASK=2 \
tools/run_tether_angle_collision_campaign.sh integrated step1ms
```

Cada etapa so roda se a anterior passou (`verdict.txt`). Os nomes antigos `colisao`/`integrado`
continuam aceitos. Estrutura de `results/tether_angles/<collision|integrated>/<rotulo>/`:

| Arquivo | Conteudo |
| --- | --- |
| `px4.log`, `gz.log` | logs do PX4 e do servidor do Gazebo |
| `world/default.sdf` | mundo de 1 ms usado |
| `drone_models/x500_tether_attach_mask/`, `model/` | X500 com mascara 1 e cabo com mascara 2 |
| `verdict.txt`, `status.txt` | veredito por etapa; abortos, failsafe e passo |
| `<etapa>/plugin_*.csv` | topicos `/cabo/*` gravados |
| `<etapa>/connection_w{0.15,0.5,1.0}.csv` | poses, tangente Python por janela, penetracao no solo |
| `<etapa>/px4_offboard_horizontal_mission.{csv,json}` | missao OFFBOARD |
| `<etapa>/metrics.json` | metricas e veredito da etapa |

Etapas: `C0_rest`, `C1_drag`, `C2_landing`, `S0_static`, `S1_vertical`, `S2_horizontal`.

### E6 — analise e graficos

```bash
R=results/tether_angles/integrated/step1ms

python3 tools/analyze_tether_angle_flight.py \
  --run-dir "$R/S2_horizontal" \
  --px4-log "$R/px4.log" \
  "$R/gz.log"                                   # -> metrics.json (use --static no S0)

python3 tools/analyze_tether_tangent_windows.py \
  --run-dir "$R/S2_horizontal"                  # -> t2_windows.json (ultimo elo x janelas)

python3 tools/plot_tether_attitude_series.py \
  --run-dir "$R/S2_horizontal"                  # -> s3_series.svg, s3_series.csv, s3_metrics.json

xdg-open "$R/S2_horizontal/s3_series.svg"       # roll/pitch/yaw, azimute e elevacao world x body
```

### Execucao manual com a GUI do Gazebo

Tres terminais. Os resultados ficam em `results/tether_angles/manual/`.

**Terminal 1 — modelos, mundo de 1 ms e Gazebo com GUI**

```bash
cd /home/lima/codes/ic/drone-cabo
OUT=results/tether_angles/manual
rm -rf $OUT && mkdir -p $OUT

# 1. modelos com collision masks (copias em results/; os modelos versionados nao mudam)
./tools/generate_x500_tether_attach.py --model-name x500_tether_attach_mask --collide-bitmask 1 \
  --output-dir $OUT/drone_models/x500_tether_attach_mask
./tools/generate_tether_anchor_chain.py --links 5 --length 2.5 --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3 \
  --no-reel-actuator --link-collisions --collide-bitmask 2 --tangent-window 0.15 \
  --drone-model x500_tether_attach_mask_0 --output-dir $OUT/model

# 2. mundo default do PX4 com max_step_size = 0.001 s
./tools/generate_px4_world_step.py --step 0.001 --output $OUT/world/default.sdf

# 3. Gazebo com GUI (o servidor precisa enxergar o plugin, o X500 com mascara e os modelos do PX4)
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/gz_plugins
export GZ_SIM_RESOURCE_PATH=$PWD/$OUT/drone_models:$PWD/src/pacote_do_drone/models:$PWD/px4/PX4-Autopilot/Tools/simulation/gz/models:$PWD/px4/PX4-Autopilot/Tools/simulation/gz/worlds
gz sim -r $OUT/world/default.sdf 2>&1 | tee $OUT/gz.log
```

Se a janela nao abrir, rode o servidor com `gz sim -s -r $OUT/world/default.sdf` e a GUI a parte,
noutro terminal com o mesmo `GZ_SIM_RESOURCE_PATH`: `gz sim -g --render-engine-gui ogre`.

**Terminal 2 — PX4 ligado ao mundo ja rodando**

```bash
cd /home/lima/codes/ic/drone-cabo
OUT=$PWD/results/tether_angles/manual
cd px4/PX4-Autopilot
PX4_GZ_MODEL=x500_tether_attach_mask PX4_GZ_MODEL_POSE=0,0,0.24,0,0,0 \
  make px4_sitl gz_x500 2>&1 | tee $OUT/px4.log
# conferir no log: "gazebo already running world: default",
# "Requested Model Position: 0,0,0.24,0,0,0" e depois "Ready for takeoff!"
```

**Terminal 3 — cabo, gravacao e missoes**

```bash
cd /home/lima/codes/ic/drone-cabo
OUT=results/tether_angles/manual

# spawn do cabo (estacao na origem, sob o drone)
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 \
  --req "sdf_filename: \"$PWD/$OUT/model/model.sdf\" name: \"tether_anchor_chain\" allow_renaming: false"

# topicos ao vivo
gz topic -e -t /cabo/conexao/angles_body     # azimute_body, elevacao_body, angulo forca x tangente
gz topic -e -t /cabo/conexao/drone_rpy       # roll, pitch, yaw usados na transformacao

# gravacao: topicos do plugin + poses com 3 janelas de tangente e penetracao no solo
record() {  # $1 = diretorio, $2 = duracao [s]
  mkdir -p "$1"
  ./tools/record_tether_timeseries.py --duration "$2" --output-dir "$1" --prefix plugin > "$1/ts.log" 2>&1 &
  for w in 0.15 0.5 1.0; do
    ./tools/record_tether_connection.py --duration "$2" --output-dir "$1" --prefix "connection_w$w" \
      --links 5 --segment-length 0.5 --settle 5 --tangent-window "$w" --collision-radius 0.00225 \
      --drone-model x500_tether_attach_mask_0 > "$1/pose_w$w.log" 2>&1 &
  done
}

# E2 — estatico (drone pousado, 30 s)
record $OUT/static 30; wait
python3 tools/analyze_tether_angle_flight.py --run-dir $OUT/static --px4-log $OUT/px4.log $OUT/gz.log --static

# E3 — contato/arraste: 1 m a 0,1 m/s, 0,8 m acima do ponto de decolagem
record $OUT/drag 80; sleep 3
./tools/px4_offboard_horizontal_mission.py --output-dir $OUT/drag --rate 20 \
  --dx 1.0 --altitude 0.8 --relative-altitude --xy-speed 0.1 \
  --takeoff-hover 12 --move-hold 18 --return-hold 18 --land-stream 10
wait
python3 tools/analyze_tether_angle_flight.py --run-dir $OUT/drag --px4-log $OUT/px4.log $OUT/gz.log

# E4 — voo vertical
record $OUT/vertical 70; sleep 3
./tools/px4_offboard_horizontal_mission.py --output-dir $OUT/vertical --rate 20 \
  --dx 0.0 --altitude 1.8 --relative-altitude
wait
python3 tools/analyze_tether_angle_flight.py --run-dir $OUT/vertical --px4-log $OUT/px4.log $OUT/gz.log

# E5 — voo horizontal (x local do PX4 = norte = +y do Gazebo)
record $OUT/horizontal 70; sleep 3
./tools/px4_offboard_horizontal_mission.py --output-dir $OUT/horizontal --rate 20 \
  --dx 0.5 --altitude 1.8 --relative-altitude
wait
python3 tools/analyze_tether_angle_flight.py --run-dir $OUT/horizontal --px4-log $OUT/px4.log $OUT/gz.log

# E6 — graficos e janelas
python3 tools/plot_tether_attitude_series.py --run-dir $OUT/horizontal
python3 tools/analyze_tether_tangent_windows.py --run-dir $OUT/horizontal
```

Cada missao arma, decola, pousa e desarma; espere `Disarmed by landing` no terminal 2 antes da
proxima. Resultados: `results/tether_angles/manual/{static,drag,vertical,horizontal}/`
(`metrics.json`, `s3_series.svg`, `t2_windows.json`, CSVs). Para encerrar: `Ctrl+C` nos terminais 2
e 1.

### Checklist do operador

| Item | Como conferir (sessao manual; na campanha use `results/tether_angles/<modo>/<rotulo>/`) |
| --- | --- |
| [ ] PX4 spawn em z = 0,24 m | `grep "Requested Model Position" $OUT/px4.log` → `0,0,0.24,0,0,0`; `drone_z_inicial_m` ≈ 0,227 no `metrics.json` |
| [ ] dt = 1 ms | `grep -o "max_step_size>[^<]*" $OUT/world/default.sdf` → `0.001`; `gz topic -e -n 1 -t /world/default/stats` → ~1000 `iterations` por segundo de `sim_time` |
| [ ] X500 collision mask = 1 | `grep -c "<collide_bitmask>1<" $OUT/drone_models/x500_tether_attach_mask/model.sdf` → 9 |
| [ ] tether collision mask = 2 | `grep -c "<collide_bitmask>2<" $OUT/model/model.sdf` → 5 |
| [ ] solo collision mask = 65535 | `grep -c collide_bitmask $OUT/world/default.sdf` → 0 (padrao 65535) |
| [ ] altitude relativa habilitada | `--relative-altitude` na linha da missao; `drone_z_max_m` ≈ 2,0–2,3 m nos voos de 1,8 m |
| [ ] `/cabo/conexao/force` disponivel | `gz topic -l \| grep -x /cabo/conexao/force` |
| [ ] `/cabo/conexao/t_hat_world` disponivel | `gz topic -l \| grep -x /cabo/conexao/t_hat_world` |
| [ ] `/cabo/conexao/t_hat_body` disponivel | `gz topic -l \| grep -x /cabo/conexao/t_hat_body` |
| [ ] `/cabo/conexao/angles_body` disponivel | `gz topic -l \| grep -x /cabo/conexao/angles_body` |
| [ ] cabo nao colide com o proprio X500 | em repouso a ponta do cabo fica no solo: `python3 -c "import csv; print(list(csv.DictReader(open('$OUT/static/connection_w0.15.csv')))[-1]['tip_z'])"` ≈ 0,002 (e nao 0,26, que e a ponta apoiada na placa do X500); e `grep -c "<collide_bitmask>1<" .../x500_tether_attach_mask/model.sdf` = 9 |
| [ ] cabo permanece sobre o solo | `penetracao_max_m` < 0,001 no `metrics.json` (criterio de PASS: ≤ 0,002) |
| [ ] sem failsafe | `grep -ci failsafe $OUT/px4.log` → 0 |
| [ ] sem crash DART | `grep -cE "Assertion\|Aborted" $OUT/px4.log $OUT/gz.log` → 0 |

### Valores de referencia

Da campanha que validou a baseline (`integrated/step1ms`, 2026-09-14). Sao referencia para detectar
regressao, nao igualdade exata entre sessoes (a variacao entre sessoes ja observada e de decimos de
milimetro na penetracao e alguns centimetros no rastreio).

| Etapa | Grandeza | Referencia |
| --- | --- | --- |
| S0 | RTF | ≈ 0,982 |
| S0 | penetracao no solo | ≈ 0 (0,02 µm) |
| S1 | RMS XY | ≈ 0,047 m |
| S1 | RMS Z em hover | ≈ 0,123 m |
| S1 | roll / pitch max | ≈ 1,1° / 1,9° |
| S1 | penetracao no solo | < 1 mm (0,74 mm) |
| S1 | RTF | ≈ 0,979 |
| S2 | dx comandado / realizado | 0,50 m / ≈ 0,469 m |
| S2 | erro de retorno | ≈ 0,028 m |
| S2 | roll / pitch max | ≈ 4,2° / 1,1° |
| S2 | penetracao no solo | < 1 mm (0,47 mm) |
| S2 | RTF | ≈ 0,978 |
| S3 (S1, S2) | residuo `R_BW` p95 / plugin x poses p95 | 0° / ≈ 0,06° |
| C1 / C2 | penetracao no solo | 0,59 mm / 0,56 mm |

**Revalidacao na consolidacao (2026-09-15, rotulo `consolidated`, codigo e nomes do commit da
baseline)** — `collision` e `integrated` a 1 ms, todas as etapas PASS, 0 aborto, 0 failsafe, 0 NaN,
sem saturacao, PX4 com 0 mudancas antes e depois:

| Etapa | Referencia (2026-09-14) | Revalidacao (2026-09-15) |
| --- | --- | --- |
| C1 arraste: penetracao / dx realizado / retorno | 0,59 mm / 0,94 m / 0,03 m | 0,40 mm / 0,97 m / 0,04 m |
| C2 pouso: penetracao / RMS Z hover | 0,56 mm / 0,127 m | 0,88 mm / 0,129 m |
| S0: RTF / penetracao | 0,982 / 0,02 µm | 0,982 / 0,02 µm |
| S1: RMS XY / RMS Z hover | 0,047 / 0,123 m | 0,045 / 0,136 m |
| S1: roll / pitch max / penetracao / RTF | 1,1° / 1,9° / 0,74 mm / 0,979 | 2,6° / 2,1° / 0,65 mm / 0,977 |
| S2: dx comandado / realizado / retorno | 0,50 / 0,469 / 0,028 m | 0,50 / 0,538 / 0,060 m |
| S2: RMS XY / RMS Z hover | 0,153 / 0,107 m | 0,153 / 0,109 m |
| S2: roll / pitch max / penetracao / RTF | 4,2° / 1,1° / 0,47 mm / 0,978 | 4,4° / 2,7° / 0,81 mm / 0,978 |
| `\|F\|` mediana (S1 / S2) | 1,04 / 0,37 N | 0,51 / 1,00 N |
| S3: residuo `R_BW` p95 / plugin x poses p95 (S1; S2) | 0 / 0,06°; 0 / 0,06° | 0 / 0,07°; 0 / 0,05° |

Entre sessoes a penetracao ficou sempre abaixo de 1 mm (0,40–0,88 mm), o dx realizado variou de
0,47 a 0,54 m e o roll maximo de 1 a 4,4°; `|F|` mediano muda com o quanto do cabo fica suspenso em
cada voo. Use a faixa, nao o valor exato.

### Historico que levou a baseline (rodada de 2026-09-14)

| Configuracao | C0 | C1 arraste | C2 pouso | S0 | S1 vertical | S2 horizontal |
| --- | --- | --- | --- | --- | --- | --- |
| sem filtro, 4 ms (`collision/baseline`) | PASS | **FAIL** — contato cabo x drone vira o X500, aborto DART | nao exec. | — | — | — |
| filtro, 4 ms (`collision/mask`) | PASS | FAIL — penetracao 2,89 mm no pouso | nao exec. | — | — | — |
| filtro, 2 ms (`step2ms`) | PASS | PASS (0,87 mm) | PASS (1,54 mm) | PASS | FAIL (2,02 mm, pousado) | nao exec. |
| **filtro, 1 ms (`step1ms`)** | **PASS** | **PASS (0,59 mm)** | **PASS (0,56 mm)** | **PASS** | **PASS (0,74 mm)** | **PASS (0,47 mm)** |

Cuidados que ficaram da investigacao:

- **Spawn:** sem `PX4_GZ_MODEL_POSE=0,0,0.24` o PX4 spawna em z = 0 e o X500 assenta afundado, com o
  attach abaixo do solo.
- **Contato cabo x drone:** com colisao nos elos e sem mascaras, a folga da constraint deixa a ponta
  do cabo sobre a placa do X500 e o ultimo elo cruza as caixas de colisao dos rotores girando.
- **Contato no dartsim:** `kp`, `kd`, `max_vel` e `min_depth` nao sao lidos; as alavancas sao passo
  de fisica e raio de colisao.
- **Origem do EKF:** ja nasceu 1,14 m deslocada; por isso `--relative-altitude`.
- **Direcao da forca:** com colisao ela fica a 24–160° da tangente; use `t_hat`, nao a forca, como
  angulo do cabo.
- **Gravacao:** um `Vector3d` todo zero chega vazio ao gravador de texto (`drone_rpy` nivelado em
  bancada).
