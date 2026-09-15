# Inicializacao da simulacao na `dev`: PX4 + X500 + tether

Investigacao de 2026-09-14 no branch `dev` (`76be39a`, igual a `origin/dev`). Checkpoint da
`shared` feito antes, em `43d6826` (local, sem push).

## 1. Resultado principal

**A `dev` nao contem nenhum procedimento que inicialize PX4 SITL + X500 + tether.**

| Busca | Resultado |
| --- | --- |
| Arvore da `dev` (`Gazebo/`, `src/pacote_do_drone`, com ignorados) por `PX4_GZ_MODEL`, `gz_x500`, `px4_sitl`, `x500`, `make px4`, `MicroXRCE` | nenhuma ocorrencia |
| Historico completo da `dev` (`git log -G`) pelos mesmos padroes | nenhum commit |
| Historico da `dev` por `px4`/`xrce`/`fmu/` | so `76be39a` (2026-09-11), que adiciona `mission_test1.py` |
| `origin/main`, `origin/tests` | 0 arquivos com X500/PX4 no Gazebo |
| Shell scripts na `dev` | nenhum |

A integracao PX4 + X500 + tether **nasceu na `shared`**, no commit `309e6eb` (2026-09-03), depois
da separacao das branches (`80d3994`, 2026-06-05, sem nenhum arquivo X500).

O que a `dev` tem sao **dois fluxos independentes**:

1. uma simulacao Gazebo **drone proprio + cabo + carretel**, sem PX4;
2. uma missao ROS 2 para PX4 (`mission_test1.py`), sem modelo, sem mundo e sem cabo.

## 2. Arquivos relevantes

| Arquivo | Papel |
| --- | --- |
| `src/pacote_do_drone/launch/start_sim.launch.py` | ponto de entrada: sobe o mundo pelo `ros_gz_sim` e a bridge ROS |
| `src/pacote_do_drone/worlds/my_world.sdf` | mundo `mundo_ic`: inclui carretel, cabo e drone e declara as juntas de ligacao |
| `src/pacote_do_drone/models/build_world.py` | gera `models/cabo.sdf` e `worlds/my_world.sdf` a partir do JSON |
| `src/pacote_do_drone/tether_package/parameters/tether_parameters.json` | parametros do cabo (70 elos x 0,05 m) |
| `src/pacote_do_drone/models/cabo.sdf` | cabo usado: `raiz_cabo` + 70 segmentos (juntas `universal` ±30°) + `ponta_cabo` |
| `src/pacote_do_drone/models/carretel/carretel.sdf` | estacao: base fixa ao mundo + tambor em revolute com `JointController` |
| `src/pacote_do_drone/models/meu_drone/meu_drone.sdf` | drone proprio com `MulticopterVelocityControl` e `OdometryPublisher` |
| `src/pacote_do_drone/launch/controller_no_tether*.launch.py` | mesmo drone sem cabo, para o controlador |
| `src/pacote_do_drone/pacote_do_drone/mission_test1.py` | missao OFFBOARD via `px4_msgs` |
| `Gazebo/` | versao antiga (`meu_drone`, `cabo.sdf` com `DetachableJoint`, URDF), com caminhos absolutos de outra maquina (`/home/joseubu/IC`); fora do fluxo ativo |

## 3. Comando original de inicializacao (drone + cabo + carretel)

Reconstruido do codigo e do `commands.sh` local (que ainda cita `gerar_cabo.py`, ja
substituido por `build_world.py`):

```bash
cd ~/codes/ic/drone-cabo
source /opt/ros/humble/setup.bash
python3 src/pacote_do_drone/models/build_world.py     # opcional: regenera cabo e mundo
colcon build --symlink-install --packages-select pacote_do_drone
source install/setup.bash
ros2 launch pacote_do_drone start_sim.launch.py
# a simulacao abre PAUSADA: clicar em Play no Gazebo
```

Ambiente: ROS 2 Humble, `ros-humble-ros-gz` (0.244.25). O launch acrescenta
`GZ_SIM_RESOURCE_PATH=<share>/models`. O `ros_gz_sim` do Humble roda o **Ignition Gazebo 6
(Fortress)**, entao a inspecao por linha de comando e com `ign topic` / `ign model`, nao `gz`.
Apos trocar de branch, restos em `build/` e `install/` quebram o build; limpar
`build/pacote_do_drone install/pacote_do_drone install/cabo_avaliacao` resolve.

## 4. Fluxo de startup

```text
ros2 launch pacote_do_drone start_sim.launch.py
  ├─ ros_gz_sim/gz_sim.launch.py  → ign gazebo worlds/my_world.sdf -v4   (com interface grafica)
  │     world "mundo_ic" (passo 0,5 ms)
  │       ├─ include model://carretel  → meu_carretel   (base fixa ao mundo, tambor revolute)
  │       ├─ include model://cabo.sdf  → cabo_dinamico  (pose na ancora 0 0,18 0,335)
  │       ├─ joint ancora_carretel_cabo  (ball)  meu_carretel::cilindro_carretel → cabo_dinamico::raiz_cabo
  │       ├─ include model://meu_drone → meu_drone      (pose = ponta do cabo - offset)
  │       └─ joint cabo_drone_joint      (ball)  cabo_dinamico::ponta_cabo → meu_drone::base_link
  └─ ros_gz_bridge parameter_bridge
        /tensao_cabo       (WrenchStamped ← gz.msgs.Wrench)
        /angulos_cabo      (JointState    ← gz.msgs.Model)
        /meu_drone/cmd_vel (Twist         → gz.msgs.Twist)
```

- **Spawn do drone:** estatico, no SDF do mundo. `build_world.py` calcula a pose a partir da
  ponta do cabo: `spawn = ponta - offset_conexao_drone` (padrao `z = -0,01`) e yaw alinhado ao cabo.
- **Spawn do tether:** estatico, include no mesmo mundo.
- **Conexao cabo–drone:** junta `ball` de nivel de mundo com filho `meu_drone::base_link`. Sem
  `<pose>`, o pivo fica na origem do `base_link` e a posicao relativa inicial da ponta fica
  congelada. O link `tether_attach` do drone existe mas nao e usado.
- **Nada acontece em runtime:** nao ha servico de spawn, `DetachableJoint` ativo nem script de attach.

Topicos/plugins do lado Gazebo: `MulticopterVelocityControl` (`/meu_drone/cmd_vel`),
`OdometryPublisher` (`/meu_drone/odom`), `JointController` do tambor (`/carretel/velocidade`),
`JointStatePublisher` do cabo, sensores `force_torque`.

## 5. O lado PX4 da `dev`

`mission_test1.py` publica em `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
`/fmu/in/vehicle_command` e le `/fmu/out/vehicle_local_position`, ou seja espera PX4 SITL +
agente uXRCE-DDS rodando a parte. A `dev` nao fornece nenhum dos dois, nem diz qual veiculo
simular. Os waypoints formam um retangulo de 40 x 30 m a 5 m de altura: um voo livre,
incompativel com qualquer cabo. Nesta maquina `px4_msgs` e `MicroXRCEAgent` nao estao
instalados (`src/px4_msgs` e ignorado pelo `.gitignore` da `dev`), entao a missao nao executa.

## 6. Execucao

`ros2 launch pacote_do_drone start_sim.launch.py` com o `install/` existente, sem alterar nada.

| Verificacao | Resultado |
| --- | --- |
| Gazebo abriu | sim (Ignition Gazebo 6, janela grafica) |
| Entidades | `ground_plane`, `meu_carretel`, `cabo_dinamico`, `meu_drone`, `sun` — sem duplicatas |
| X500 | **ausente** (o drone e `meu_drone`) |
| PX4 | **nao inicializado** (nao faz parte do procedimento) |
| Cabo presente | sim, 70 segmentos, visivel do topo do tambor ate o drone |
| Conectado ao drone | sim: distancia ponta → `base_link` constante em 0,2076 m (variacao 1,1e-4 m) |
| Conectado no carretel | sim: distancia raiz → tambor constante em 0,2486 m (variacao 1,6e-4 m) |
| Interpenetracao | nenhuma evidente: cabo apoiado no solo (`z_min` ≈ -0,001 m), drone pousado (`z` = 0,059 m) |
| Estabilidade | sem erros no launch; RTF 0,04–0,11 (passo de 0,5 ms com 70 elos) |
| Bridge ROS | `/tensao_cabo`, `/angulos_cabo`, `/meu_drone/cmd_vel` presentes |

Captura: `results/dev_startup/gazebo_dev_tether.png`.

## 7. Diferencas relevantes em relacao a `shared`

| Aspecto | `dev` | `shared` (`43d6826`) |
| --- | --- | --- |
| Drone | `meu_drone`, controle de velocidade do Gazebo | X500 do PX4 (`x500_tether_attach`) |
| PX4 no loop | nao | sim, `gz_bridge` |
| Simulador | Ignition Gazebo 6 via `ros2 launch` | Gazebo Sim 7 via `make px4_sitl gz_x500` |
| Mundo | `mundo_ic`, passo 0,5 ms | `default` do PX4, passo 4 ms |
| Conexao no drone | junta `ball` de mundo → `base_link` | junta `ball` de mundo → `base_link`, pivo no `tether_attach_link` (na baseline anterior, constraint de forca) |
| Juntas do cabo | `universal` ±30°, 70 × 0,05 m | `ball`, 10 × 0,25 m |
| Ancoragem | tambor (revolute com controlador de velocidade) | guia fixa na estacao |
| Instrumentacao | bridge ROS (tensao, angulos) | topicos `/cabo/*`, registrador de poses |

Conclusao: nao ha procedimento original PX4 + X500 + tether a recuperar da `dev`. O unico
procedimento visual completo da `dev` e o de drone proprio + cabo + carretel, e ele funciona
como descrito.
