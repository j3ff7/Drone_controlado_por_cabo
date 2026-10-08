# Plano incremental de integracao X500/PX4 com tether

Este documento define uma estrategia tecnica para incorporar o tether atual ao modelo `gz_x500` usado pelo PX4/Gazebo, preservando o X500 upstream e isolando problemas por etapa.

## 1. Estado atual inspecionado

### PX4/X500

O PX4 foi adicionado como clone Git independente, ignorado pelo Git do projeto principal:

```text
px4/PX4-Autopilot/
```

Versao validada:

```text
remote: https://github.com/PX4/PX4-Autopilot.git
tag:    v1.14.4
commit: 1555f2bd2229544c43966ab5f94879c41d8e1e01
estado: detached HEAD
```

O X500 e lancado atualmente com:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

ou, sem GUI:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Arquivos relevantes do X500:

```text
px4/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf
px4/PX4-Autopilot/Tools/simulation/gz/models/x500/model.config
px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500
px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator
```

No `model.sdf` do X500, o link principal e:

```text
x500::base_link
```

com massa:

```text
2.0 kg
```

O `px4-rc.simulator` inicia o `gz_bridge` usando `PX4_GZ_MODEL`; portanto uma variante futura devera ser descoberta pelo `GZ_SIM_RESOURCE_PATH` e ser compativel com a logica de spawn do PX4.

### Tether atual

Fonte de parametros:

```text
src/pacote_do_drone/tether_parameters.json
```

Gerador:

```text
src/pacote_do_drone/models/gerar_cabo.py
```

SDF gerado:

```text
src/pacote_do_drone/models/cabo.sdf
```

World gerado legado:

```text
src/pacote_do_drone/worlds/my_world.sdf
```

Parametros atuais principais:

```text
num_links                50
comprimento_total_m      2.5
densidade_linear_kg_m    0.06
length                   0.05
radius                   0.002
mass                     0.003
dummy_mass               0.0001
root_mass                0.0005
tip_mass                 0.0005
joint_damping            0.08
joint_friction           0.002
joint_spring_stiffness   0.02
segment_collision        true
connection_type          ball
anchor                   (0.0, 0.0, 0.33) m
initial_shape            sine_slack
initial_end              (2.0, 0.0, 0.33) m
```

O cabo e discretizado por uma cadeia de links:

```text
raiz_cabo
dummy_i
segment_i
final_segment
ponta_cabo
```

Cada segmento e orientado ao longo de seu eixo local `+x`. A ponta visual/fisica auxiliar e ligada ao `final_segment` por:

```text
joint_ponta_cabo
```

O gerador calcula:

- comprimento de cada segmento;
- massa por segmento, preferencialmente a partir de `densidade_linear_kg_m`;
- inercias;
- poses iniciais;
- `initial_position` das juntas;
- limites das juntas;
- sensores de force/torque nas extremidades.

### Conexao ball atual

No launch principal atual:

```text
src/pacote_do_drone/launch/start_sim.launch.py
```

a conexao com o drone e criada por uma junta:

```text
cabo_drone_joint
```

Quando `connection_type = ball`, a forma atual e:

```xml
<joint name="cabo_drone_joint" type="ball">
  <pose relative_to="cabo_dinamico::final_segment">...</pose>
  <parent>cabo_dinamico::final_segment</parent>
  <child>meu_drone::cabo_sensor_link</child>
</joint>
```

A pose da junta e colocada no fim do `final_segment`, usando o offset extraido de `joint_ponta_cabo`. Essa escolha evita usar `ponta_cabo` como parent direto e reduz inconsistencias de base de junta.

### Sensor angular atual

Funcoes principais:

```text
src/pacote_do_drone/pacote_do_drone/cabo_angulos.py
src/pacote_do_drone/pacote_do_drone/sensores.py
```

Convencao:

```text
frame local do drone/sensor:
  x = frente
  y = esquerda
  z = cima

azimuth   = atan2(y, x)
elevation = atan2(-z, sqrt(x^2 + y^2))
```

Metodo principal recomendado:

```text
tangente local do cabo perto do drone
  -> vetor expresso no frame do drone
  -> azimuth/elevation
```

O calculo ja suporta uma janela fisica:

```text
janela_tangente_metros = 0.15
```

Essa abordagem deve ser preservada, porque nao depende diretamente do numero de links.

### Ancora estatica atual

A ancora atual e modelada no mundo/launch como um link fixo:

```text
ancora_cabo
```

preso ao mundo por:

```text
fixa_ancora_cabo_mundo
```

e conectado ao cabo por:

```text
ancora_carretel_cabo
```

O modelo visual do carretel atual existe em:

```text
src/pacote_do_drone/models/carretel/carretel.sdf
```

mas, para a integracao X500/PX4, a primeira ancora deve continuar ideal/fixa.

## 2. Componentes reutilizaveis sem alteracoes imediatas

Podem ser reutilizados:

- `src/pacote_do_drone/tether_parameters.json`;
- `src/pacote_do_drone/models/gerar_cabo.py`;
- `src/pacote_do_drone/models/cabo.sdf`, como artefato gerado;
- calculos de massa/segmentos/initial shape do tether;
- conexao `ball` como conceito;
- force/torque sensors nas extremidades do cabo;
- `src/pacote_do_drone/pacote_do_drone/cabo_angulos.py`;
- convencao de azimuth/elevation;
- `janela_tangente_metros`;
- `cabo_monitor.py`, depois que os topicos forem ligados ao mundo PX4;
- infra de metricas, depois de adaptar nomes de topicos/frame do X500;
- casos estaticos de postes em `cabo_avaliacao`;
- documentacao PX4 ja criada em `docs/PX4_INTEGRATION.md` e `docs/PX4_SIMULATION_GUIDE.md`.

Devem ser substituidos ou adaptados na trilha PX4:

- `meu_drone.sdf`;
- `MulticopterVelocityControl`;
- controle por `/meu_drone/cmd_vel`;
- nomes de modelo/link assumidos como `meu_drone::*`;
- topicos de odometria atuais `/meu_drone/odom`.

## 3. Preservar o X500 upstream

Nao modificar diretamente:

```text
px4/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf
```

A estrategia recomendada e criar uma variante externa do projeto, por exemplo:

```text
src/pacote_do_drone/models/x500_tethered/
```

ou, se a separacao PX4 crescer:

```text
px4_overlay/models/x500_tethered/
```

Mecanismo preferencial no Gazebo/SDF:

```text
modelo composto por include do x500 upstream + link extra
```

Forma conceitual:

```xml
<model name="x500_tethered">
  <include>
    <uri>model://x500</uri>
    <merge>true</merge>
  </include>

  <link name="tether_attach_link">...</link>

  <joint name="tether_attach_fixed" type="fixed">
    <parent>base_link</parent>
    <child>tether_attach_link</child>
  </joint>
</model>
```

Vantagens:

- preserva o X500 original;
- permite comparar `x500` e `x500_tethered`;
- reduz conflitos ao atualizar PX4;
- mantem customizacoes do tether no repositorio do projeto;
- evita patch no PX4 enquanto nao houver alteracao inevitavel.

Risco tecnico:

O suporte a `<merge>true</merge>` e a resolucao de `model://x500` devem ser validados no Gazebo Sim usado. Se houver incompatibilidade, o fallback e copiar o `model.sdf` do X500 para uma variante `x500_tethered`, mantendo essa copia no projeto como overlay versionado. Esse fallback aumenta manutencao, mas ainda evita editar o upstream.

Para o PX4 encontrar o modelo externo, sera necessario ajustar ambiente antes do launch:

```bash
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
```

e iniciar com um modelo futuro:

```bash
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

Se o autostart `gz_x500` exigir semanticamente o modelo `x500`, usar `PX4_GZ_MODEL=x500_tethered` mantendo `PX4_SIM_MODEL=gz_x500` pode ser investigado. Se isso nao for aceito pelo fluxo PX4 v1.14.4, a alternativa e criar um target/airframe minimo no PX4 como patch pequeno, mas apenas depois de validar que o overlay externo nao basta.

## 4. Etapas incrementais

### Etapa 0 - X500 puro

Objetivo:
validar o PX4/Gazebo sem tether.

Comando:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Comandos no `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

PASS:

- PX4 inicia sem erro relevante;
- Gazebo carrega `x500`;
- `commander arm` funciona;
- `commander takeoff` detecta decolagem;
- `vehicle_attitude` mostra roll/pitch pequenos em hover;
- `commander land` pousa e desarma;
- RTF registrado.

FAIL:

- falha de spawn;
- falha de sensores/estimador;
- arming recusado;
- failsafe ativo;
- hover instavel sem tether.

Status atual da validacao:

```text
data: 2026-09-03
status final: ETAPA 0: PASS
```

Baseline validada:

```text
PX4 remote: https://github.com/PX4/PX4-Autopilot.git
PX4 tag:    v1.14.4
PX4 commit: 1555f2bd2229544c43966ab5f94879c41d8e1e01
estado PX4: detached HEAD
modelo:     x500
```

Comando efetivamente usado:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Testes executados no `pxh>`:

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

Resultados observados:

- PX4 SITL iniciou sem erro bloqueante;
- Gazebo carregou o mundo `default` e o modelo `x500_0`;
- sensores/estimador convergiram, com `Ready for takeoff!` no startup;
- `commander status` antes do voo indicou `Arm state: Standby`, `navigation mode: AUTO_LOITER` e `in failsafe: no`;
- `commander arm` foi aceito, com `Armed by internal command`;
- `commander takeoff` foi aceito, com `Takeoff detected`;
- `vehicle_local_position` em hover apresentou posicao coerente, com `xy_valid=True`, `z_valid=True`, `v_xy_valid=True`, `v_z_valid=True`, `heading_good_for_control=True`;
- altitude local em hover: `z = -1.95 m` no frame NED do PX4, equivalente a aproximadamente `1.95 m` acima do ponto de referencia local;
- atitude em hover: roll aproximadamente `0.3 deg`, pitch aproximadamente `0.2 deg`, yaw aproximadamente `90.7 deg`;
- `commander status` durante hover indicou `Arm state: Armed`, `navigation mode: AUTO_LOITER` e `in failsafe: no`;
- `commander land` funcionou, com `Landing detected`;
- o veiculo desarmou automaticamente apos o pouso, com `Disarmed by landing`;
- `commander status` apos pouso indicou `Arm state: Standby` e `in failsafe: no`.

RTF observado:

```text
amostras durante o teste: aproximadamente 0.972 a 1.004
media aproximada:         0.992
```

Correcoes necessarias:

- nenhuma correcao bloqueante foi necessaria para aprovar a Etapa 0;
- ocorreram avisos nao bloqueantes de renderizacao EGL no Gazebo (`libEGL warning: egl: failed to create dri2 screen`);
- ocorreram avisos nao bloqueantes `Unknown message type [88]` e `Unknown message type [8]`, sem impacto observado em arming, takeoff, hover, land ou failsafe.

### Etapa 1 - Apenas ponto de conexao

Objetivo:
adicionar `tether_attach_link` sem cabo.

Mudanca minima:

```text
x500_tethered
  base_link
  tether_attach_link
  tether_attach_fixed
```

Sugestao inicial de posicao:

```text
relative_to="base_link"
pose: 0 0 -0.12 0 0 0
```

Essa posicao deve ficar abaixo do centro do drone e acima das colisoes inferiores, ajustada visualmente no Gazebo.

PASS:

- `x500_tethered` spawna;
- `tether_attach_link` aparece abaixo do drone;
- massa adicionada e pequena, por exemplo `1 g` a `10 g`;
- hover permanece praticamente igual ao X500 puro;
- roll/pitch e posicao nao mudam significativamente;
- RTF permanece semelhante.

FAIL:

- modelo nao e encontrado;
- link aparece deslocado;
- PX4 nao comunica com o modelo;
- hover muda muito sem cabo.

Registro da validacao atual:

```text
data: 2026-09-04
status final: ETAPA 1: PASS
```

Implementacao:

- criada a variante externa `x500_tethered`, sem editar o X500 upstream;
- a tentativa preferencial por `<include><merge>true</merge>` foi testada, mas o `gz sdf`/SDFormat desta instalacao nao aceitou a composicao de forma confiavel para referenciar `base_link`;
- aplicado o fallback previsto no plano: copia local do `model.sdf` do X500 como overlay do projeto, mantendo os assets por `model://x500`;
- adicionado `tether_attach_link` fixo em `base_link` por `tether_attach_fixed`.

Arquivos alterados:

```text
src/pacote_do_drone/models/x500_tethered/model.config
src/pacote_do_drone/models/x500_tethered/model.sdf
```

Attachment point:

```text
link:        tether_attach_link
joint:       tether_attach_fixed
parent:      base_link
pose local:  0 0 -0.12 0 0 0
frame:       base_link do X500
orientacao:  alinhada ao base_link
massa:       0.005 kg
```

Comando usado:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

Testes:

- `gz sdf -k` retornou `Valid.`;
- `gz_bridge` spawnou `x500_tethered_0`;
- `gz model -m x500_tethered_0 -l` confirmou `tether_attach_link` em `z=-0.120 m` relativo ao modelo;
- `gz model -m x500_tethered_0 -j` confirmou `tether_attach_fixed` entre `base_link` e `tether_attach_link`;
- `commander status`, `commander arm`, `commander takeoff`, `listener vehicle_local_position 1`, `listener vehicle_attitude 1`, `commander land`, `shutdown`.

Resultados:

- startup PX4/Gazebo: PASS;
- arming: PASS;
- takeoff: PASS;
- hover: PASS;
- land/disarm: PASS;
- failsafe observado: nao;
- atitude em hover via `listener vehicle_attitude`: roll `-0.1 deg`, pitch `0.3 deg`, yaw `90.3 deg`;
- maximos no ULog durante o voo: `|roll|max = 2.1 deg`, `|pitch|max = 0.7 deg`;
- altitude local observada em hover: aproximadamente `1.47 m` no PX4 NED (`z=-1.47 m`);
- RTF observado: `0.985` a `1.001`, media aproximada `0.993`.

### Etapa 2 - Ball joint sem cabo completo

Objetivo:
validar a interface mecanica sem a complexidade de 50 links.

Configuracoes candidatas:

1. `tether_mode=pendulo` com 1 link leve;
2. cabo gerado com `num_links=1`;
3. link minimo de teste conectado por `ball`.

Grandezas a medir:

```text
distancia entre ponto final do cabo e tether_attach_link
|F| na conexao
|M| na conexao
roll/pitch do X500
RTF
```

PASS:

- distancia entre pontos conectados aproximadamente zero;
- sem offset visual entre cabo e link;
- `|M|` transmitido pela junta ball aproximadamente zero;
- `|F|` pequeno e coerente com a massa de teste;
- hover permanece estavel.

FAIL:

- cabo aparece separado do attach point;
- junta aplica momento artificial;
- X500 inclina muito com carga minima;
- force/torque oscila sem causa fisica clara.

Registro da validacao atual:

```text
data: 2026-09-04
status final: ETAPA 2: PASS
```

Implementacao:

- mantido `tether_attach_link` da Etapa 1;
- adicionado `tether_test_link`, um link minimo de teste, conectado ao attachment point por `tether_test_ball_joint`;
- o frame de `tether_test_link` foi colocado exatamente no ponto de junta, com geometria e centro de massa deslocados para baixo. Assim, o ponto conectado do link coincide com o `tether_attach_link`.

Joint/link de teste:

```text
joint:              tether_test_ball_joint
tipo:               ball
parent:             tether_attach_link
child:              tether_test_link
pose da junta:      0 0 0 0 0 0
link de teste:      cilindro vertical de 0.10 m
raio visual:        0.008 m
massa:              0.005 kg
pose do link:       0 0 0 0 0 0 relativo a tether_attach_link
pose inertial/COM:  0 0 -0.05 0 0 0
inercia:            ixx=4.18e-6, iyy=4.18e-6, izz=1.60e-7 kg.m^2
```

Offset medido:

```text
P_attach = pose de tether_attach_link
P_link   = pose do frame de tether_test_link
d        = ||P_attach - P_link||
d inicial/hover ~= 0.000 m
```

Forca/momento:

- foi adicionada uma declaracao SDF de sensor `force_torque` em `tether_test_ball_joint`;
- nesta execucao, o Gazebo Sim 7.9.0 nao publicou um topico de force/torque ou wrench correspondente;
- portanto `Fx`, `Fy`, `Fz`, `Mx`, `My`, `Mz`, `|F|` e `|M|` ficaram indisponiveis como medicao direta nesta etapa;
- nao foi observado indicio dinamico de momento artificial relevante: o hover permaneceu estavel e os maximos de roll/pitch ficaram pequenos.

Testes:

- `gz sdf -k` retornou `Valid.`;
- `gz_bridge` spawnou `x500_tethered_0`;
- `gz model -m x500_tethered_0 -l` confirmou `tether_attach_link` e `tether_test_link`;
- `gz model -m x500_tethered_0 -j` confirmou `tether_test_ball_joint` do tipo `ball`;
- `commander status`, `commander arm`, `commander takeoff`, `listener vehicle_local_position 1`, `listener vehicle_attitude 1`, `commander land`, `shutdown`.

Resultados:

- startup PX4/Gazebo: PASS;
- arming: PASS;
- takeoff: PASS;
- hover: PASS;
- land/disarm: PASS;
- failsafe observado: nao;
- atitude em hover via `listener vehicle_attitude`: roll `0.2 deg`, pitch `0.2 deg`, yaw `91.5 deg`;
- maximos no ULog durante o voo: `|roll|max = 0.6 deg`, `|pitch|max = 1.2 deg`;
- altitude local observada em hover: aproximadamente `1.64 m` no PX4 NED (`z=-1.64 m`);
- RTF observado: `0.973` a `1.002`, media aproximada `0.991`;
- link conectado visual/cinematicamente sem offset relevante entre os frames conectados.

Comparacao das etapas validadas:

| Configuracao | hover | roll/pitch | offset conexao | `|M|max` | RTF | status |
| --- | --- | --- | --- | --- | --- | --- |
| Etapa 0 - X500 puro | estavel | amostra hover `0.3/0.2 deg` | nao aplicavel | nao aplicavel | media `0.992` | PASS |
| Etapa 1 - attach point | estavel | max ULog `2.1/0.7 deg` | fixo em `z=-0.12 m` | nao aplicavel | media `0.993` | PASS |
| Etapa 2 - ball + link minimo | estavel | max ULog `0.6/1.2 deg` | `d ~= 0.000 m` | indisponivel como topico | media `0.991` | PASS |

### Etapa 3 - Tether curto e simples

Objetivo:
introduzir dinamica de cabo com poucos graus de liberdade.

Configuracao:

```text
num_links = 5 ou 10
comprimento_total_m coerente com o teste
densidade_linear_kg_m mantida
raiz livre
conexao no drone via ball
```

PASS:

- X500 mantem hover;
- juntas nao saturam;
- cabo acompanha a dinamica sem explodir;
- forca na conexao e pequena/moderada;
- RTF aceitavel.

FAIL:

- queda brusca de RTF;
- instabilidade das juntas;
- cabo atravessa o drone;
- contato com solo domina a dinamica;
- PX4 entra em failsafe.

Registro da validacao atual:

```text
data: 2026-09-04
status final: ETAPA 3: PASS
```

Arquitetura implementada:

```text
X500/PX4
  base_link
    tether_attach_link
      tether_joint_1 (ball)
        tether_link_1
          tether_joint_2 (ball)
            tether_link_2
              ...
                extremidade livre
```

Geracao parametrizavel:

```text
script: tools/generate_x500_tethered.py
modelo gerado: src/pacote_do_drone/models/x500_tethered/model.sdf
config gerado: src/pacote_do_drone/models/x500_tethered/model.config
```

Comando final usado para gerar o caso aprovado mais discretizado:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 10 --length 0.30 --rho 0.06 --radius 0.003 --initial-axis z
```

Escolha fisica:

- comprimento curto escolhido: `0.30 m`;
- densidade linear: `0.06 kg/m`;
- massa total do tether curto: `0.018 kg`;
- raio visual dos elos: `0.003 m`;
- extremidade oposta: livre, sem ancora, carretel ou tether completo;
- inicializacao final aprovada: vertical, abaixo de `tether_attach_link`;
- colisoes dos elos curtos: desativadas nesta etapa para isolar a dinamica de juntas/massa e evitar crash do detector DART/ODE.

Tentativas e correcoes:

- tentativa inicial com `L=0.10 m`, `N=5`, inicializacao horizontal: `FAIL`, com abort no backend DART/ODE em `collide()`;
- tentativa com `L=0.10 m`, `N=5`, elos sem colisao: `FAIL`, ainda com abort no backend DART/ODE;
- tentativa com `L=0.30 m`, `N=5`, inicializacao horizontal: `FAIL`, com abort no backend DART/ODE logo apos startup;
- correcao aplicada: inicializacao vertical para remover torque gravitacional inicial alto e reduzir energia inicial da cadeia;
- resultado apos correcao: `L=0.30 m`, inicializacao vertical, elos sem colisao, `N=5` e `N=10` aprovados.

Parametros do caso `N=5`:

```text
comprimento_total_teste: 0.300000 m
comprimento_por_link:   0.060000 m
rho_linear:             0.060000 kg/m
massa_total:            0.018000 kg
massa_por_link:         0.003600 kg
raio:                   0.003000 m
inercia_transversal:    1.0881e-06 kg.m^2
inercia_axial:          1.62e-08 kg.m^2
```

Parametros do caso `N=10`:

```text
comprimento_total_teste: 0.300000 m
comprimento_por_link:   0.030000 m
rho_linear:             0.060000 kg/m
massa_total:            0.018000 kg
massa_por_link:         0.001800 kg
raio:                   0.003000 m
inercia_transversal:    1.3905e-07 kg.m^2
inercia_axial:          8.1e-09 kg.m^2
```

Assentamento desarmado:

- `N=5`: PASS, cadeia continua e estavel; RTF medio durante assentamento `0.999`;
- `N=10`: PASS, cadeia continua e estavel; RTF medio durante assentamento `0.993`;
- nao houve explosao de constraints, divergencia visivel ou separacao dos links nos casos verticais aprovados.

Resultados `N=5` (`ETAPA 3A`):

- startup PX4/Gazebo: PASS;
- arming: PASS;
- takeoff: PASS;
- hover: PASS;
- land/disarm: PASS;
- failsafe observado: nao;
- atitude em hover via `listener vehicle_attitude`: roll `0.2 deg`, pitch `-0.3 deg`, yaw `88.7 deg`;
- maximos no ULog durante o voo: `|roll|max = 3.9 deg`, `|pitch|max = 1.3 deg`;
- altitude local em hover: aproximadamente `2.00 m` no PX4 NED (`z=-2.00 m`);
- RTF em hover: `0.978` a `0.999`, media aproximada `0.994`;
- cadeia se manteve conectada e com movimento passivo.

Resultados `N=10` (`ETAPA 3B`):

- startup PX4/Gazebo: PASS;
- arming: PASS;
- takeoff: PASS;
- hover: PASS;
- land/disarm: PASS;
- failsafe observado: nao;
- atitude em hover via `listener vehicle_attitude`: roll `0.4 deg`, pitch `-0.4 deg`, yaw `90.3 deg`;
- maximos no ULog durante o voo: `|roll|max = 0.7 deg`, `|pitch|max = 0.7 deg`;
- altitude local em hover: aproximadamente `2.00 m` no PX4 NED (`z=-2.00 m`);
- RTF em hover: `0.983` a `1.002`, media aproximada `0.994`;
- cadeia se manteve conectada e com movimento passivo.

Forca/momento:

- foi mantida a declaracao SDF de `force_torque` na primeira junta (`tether_joint_1`);
- o Gazebo Sim 7.9.0 desta instalacao nao publicou topico `force`, `wrench` ou equivalente para essa junta;
- portanto `Fx`, `Fy`, `Fz`, `Mx`, `My`, `Mz`, `|F|max` e `|M|max` nao ficaram disponiveis como medicao direta nesta etapa;
- a ausencia de momento artificial relevante foi inferida indiretamente pelo hover estavel e pelos baixos valores de roll/pitch.

Comparacao:

| Configuracao | N links | massa total | hover | roll/pitch | `|F|max` | `|M|max` | RTF | status |
| --- | ---: | ---: | --- | --- | --- | --- | --- | --- |
| Etapa 0 - X500 puro | 0 | 0 kg | estavel | amostra hover `0.3/0.2 deg` | nao aplicavel | nao aplicavel | media `0.992` | PASS |
| Etapa 2 - ball + link minimo | 1 | `0.005 kg` | estavel | max ULog `0.6/1.2 deg` | indisponivel | indisponivel | media `0.991` | PASS |
| Etapa 3A - tether livre | 5 | `0.018 kg` | estavel | max ULog `3.9/1.3 deg` | indisponivel | indisponivel | media `0.994` | PASS |
| Etapa 3B - tether livre | 10 | `0.018 kg` | estavel | max ULog `0.7/0.7 deg` | indisponivel | indisponivel | media `0.994` | PASS |

Pendencias antes da Etapa 4:

- instrumentar forca/momento da conexao por um mecanismo que de fato publique `wrench` no Gazebo Sim 7.9.0;
- reavaliar colisoes dos elos em uma etapa propria, pois as colisoes curtas acionaram crash DART/ODE;
- manter a inicializacao vertical como baseline de baixa energia para os proximos testes.

### Etapa 4 - Tether completo com extremidade livre

Objetivo:
validar o custo computacional e a massa total do tether completo sem restricao da ancora.

Configuracao:

```text
num_links = 50
comprimento_total_m = 2.5
densidade_linear_kg_m = 0.06
comprimento_por_link = 0.05 m
massa_total_cabo = 0.150 kg
massa_por_link = 0.003 kg
raio_link = 0.003 m
inicializacao = vertical, eixo z
colisoes_links = false
conexao drone = ball joint em tether_attach_link
extremidade oposta = livre
```

PASS:

- X500 mantem hover;
- forca media na conexao e coerente com a massa do cabo;
- RTF medido e documentado;
- cabo nao cria momento artificial no drone.

FAIL:

- hover falha apenas pela massa do cabo;
- RTF inviavel;
- instabilidade numerica do cabo.

Status atual da validacao:

```text
data: 2026-09-05
status final: ETAPA 4: PASS
```

Comando de geracao usado:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Comando de simulacao usado:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

Testes executados no `pxh>`:

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

Resultados observados:

- startup PX4/Gazebo: PASS;
- spawn do modelo `x500_tethered_0`: PASS;
- `commander arm`: PASS;
- `commander takeoff`: PASS;
- hover: PASS;
- `commander land`: PASS;
- disarm automatico apos pouso: PASS;
- failsafe observado: nao;
- atitude em hover via `listener vehicle_attitude`: roll `0.4 deg`, pitch `-0.1 deg`, yaw `88.9 deg`;
- maximos no ULog durante o voo: `|roll|max = 0.75 deg`, `|pitch|max = 1.09 deg`;
- altitude maxima local: aproximadamente `2.02 m` no eixo vertical positivo;
- RTF em hover: media aproximada `0.988`, minimo `0.965`, maximo `1.001`;
- cadeia com 50 links permaneceu conectada ao `tether_attach_link`;
- nao foi observado offset visual na conexao durante o hover.

Forca, momento e tensao:

- a primeira junta (`tether_joint_1`) contem declaracao SDF de sensor `force_torque`;
- nesta instalacao, o Gazebo Sim 7.9.0 nao publicou topico `force`, `wrench` ou equivalente para a junta;
- portanto forca na conexao, momento na conexao e tensao do tether ficaram indisponiveis como medicao direta;
- a ausencia de momento artificial significativo foi avaliada indiretamente por hover estavel, roll/pitch baixos e conexao visualmente coerente.

Comparacao com a Etapa 3:

| Configuracao | N links | massa total | comprimento | hover | roll/pitch max | RTF medio | status |
| --- | ---: | ---: | ---: | --- | --- | ---: | --- |
| Etapa 3A - tether livre curto | 5 | `0.018 kg` | `0.30 m` | estavel | `3.9/1.3 deg` | `0.994` | PASS |
| Etapa 3B - tether livre curto | 10 | `0.018 kg` | `0.30 m` | estavel | `0.7/0.7 deg` | `0.994` | PASS |
| Etapa 4 - tether completo livre | 50 | `0.150 kg` | `2.50 m` | estavel | `0.75/1.09 deg` | `0.988` | PASS |

### Etapa 5 - Tether completo com ancora fixa

Objetivo:
primeira configuracao representativa do drone cabeado.

Arquitetura:

```text
PX4/X500
  -> tether_attach_link
  -> ball joint
  -> tether completo
  -> fixed anchor
```

Testes:

- hover acima/proximo da ancora;
- pequena subida;
- pequeno deslocamento horizontal;
- retorno;
- pouso.

PASS:

- tether nasce em configuracao geometricamente compativel;
- tensao inicial baixa/moderada;
- sem pico de tensao incompatível;
- X500 mantem hover;
- PX4 nao entra em failsafe;
- RTF documentado;
- cabo permanece conectado visual e fisicamente.

FAIL:

- tether nasce esticado/tensionado;
- pico de tensao derruba o controle;
- offset visual reaparece;
- ancora ou cabo cria momento artificial;
- contato com solo domina o resultado.

Status atual da validacao:

```text
data: 2026-09-05
status final: ETAPA 5: FAIL
motivo: topologia ancorada atual fecha a cadeia de joints dentro do mesmo modelo.
```

Tentativa executada:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z --anchored --anchor-x 0 --anchor-y 0 --anchor-z -2.57
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Geometria inicial tentada:

```text
posicao da ancora no modelo = (0.0, 0.0, -2.57) m
posicao do tether_attach_link = (0.0, 0.0, -0.12) m relativa a base_link
comprimento do tether = 2.50 m
distancia geometrica attach-ancora = 2.45 m
slack geometrico aproximado = 0.05 m
```

Resultado:

- `gz sdf -k` aceitou o arquivo como SDF valido;
- durante o startup do Gazebo/DART, o modelo gerou erro estrutural:

```text
Asked to create a joint between links [tether_anchor_link] as parent and [tether_link_50] as child,
but the child link already has a parent joint of type [BallJoint].
```

Diagnostico:

- a cadeia livre da Etapa 4 e montada como arvore `tether_attach_link -> tether_link_1 -> ... -> tether_link_50`;
- ao adicionar uma ancora fixa na outra extremidade dentro do mesmo modelo, `tether_link_50` passa a receber uma segunda junta pai;
- isso cria um loop cinematico fechado entre drone, cabo e mundo;
- o backend DART usado pelo Gazebo Sim nesta configuracao nao aceita essa topologia como uma arvore de articulacoes valida;
- a falha ocorre antes de `arm` e `takeoff`, portanto nao ha metricas validas de hover, subida, deslocamento horizontal ou retorno.

Subtestes da Etapa 5:

| Teste | Resultado | Observacao |
| --- | --- | --- |
| A - hover | nao executado | bloqueado pelo erro de topologia no startup |
| B - pequena subida vertical | nao executado | depende do Teste A |
| C - pequeno deslocamento horizontal | nao executado | depende do Teste A |
| D - retorno ao ponto inicial | nao executado | depende do Teste C |

Proxima correcao tecnica recomendada:

- nao tentar fechar a cadeia dentro do mesmo modelo `x500_tethered`;
- representar o cabo ancorado como uma topologia suportada pelo Gazebo, por exemplo com mundo/modelos separados e junta criada no nivel do mundo, ou reestruturar a arvore para que exista apenas um caminho pai-filho entre ancora, cabo e drone;
- validar essa arquitetura primeiro com o X500 parado, antes de repetir `arm/takeoff`.

### Investigacao do loop cinematico - modelos independentes

Status da rodada:

```text
data: 2026-09-05
objetivo: encontrar arquitetura sem closed kinematic loop para refazer a Etapa 5
```

Por que a topologia anterior falha:

```text
tether_attach_link -> tether_link_1 -> ... -> tether_link_50
                                  e
tether_anchor_link -> tether_link_50
```

O link `tether_link_50` passa a ter dois parent joints. O DART, usado pelo Gazebo Sim 7.9.0 nesta instalacao, rejeita essa estrutura porque ela deixa de ser uma arvore de articulacoes.

#### Teste A - tether ancorado como modelo independente

Foi criado um modelo separado:

```text
src/pacote_do_drone/models/tether_anchor_chain/
```

Gerador:

```text
tools/generate_tether_anchor_chain.py
```

Arquitetura:

```text
WORLD
|-- x500_0
|   `-- base_link
`-- tether_anchor_chain
    |-- anchor_link
    |-- tether_link_1
    |-- ...
    `-- tether_link_N
```

O tether fica enraizado em `anchor_link`, preso ao mundo por `anchor_world_fixed`, e a cadeia segue como arvore:

```text
world -> anchor_link -> tether_link_1 -> ... -> tether_link_N
```

Teste executado:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_tether_anchor_chain.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
```

Com o PX4/X500 puro rodando, o modelo foi inserido por:

```bash
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: true'
```

Resultado:

- Gazebo iniciou: PASS;
- DART nao abortou: PASS;
- tether ancorado independente assentou normalmente: PASS;
- X500 separado armou, decolou e manteve hover: PASS;
- atitude em hover do X500 separado: roll `0.2 deg`, pitch `0.2 deg`, yaw `90.7 deg`;
- RTF observado: aproximadamente `0.99` a `1.00`.

#### Ball joint entre modelos

Foi criado um mundo minimo:

```text
src/pacote_do_drone/worlds/inter_model_ball_probe.sdf
```

Arquitetura do teste:

```text
world
|-- model_a::link
|-- model_b::link
`-- joint inter_model_ball_joint, type=ball
```

Resultado:

- `gz sdf -k` aceitou o SDF: PASS;
- `gz sim -s -r --iterations 1000` executou sem erro bloqueante: PASS;
- conclusao: `ball joint` entre modelos e suportado quando declarado no nivel do `world` antes do startup.

Teste de criacao em runtime:

```bash
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf: "...<joint name=\"runtime_inter_model_ball_joint\" type=\"ball\">...</joint>..."'
```

Resultado:

```text
Expected exactly one top-level <model>, <light> or <actor> on SDF.
```

Conclusao:

- o servico `/world/<world>/create` nao cria uma junta de topo em runtime;
- portanto a solucao `inter-model ball` e promissora apenas se o mundo for montado antes do startup contendo X500, tether e junta;
- no fluxo PX4 atual, o X500 e inserido pelo `gz_bridge` apos o startup, entao a junta inter-model predeclarada ainda nao e uma solucao pronta.

#### Prototipo force-based

Foi implementado um plugin minimo:

```text
src/pacote_do_drone/gz_plugins/TetherForceConstraint.cc
tools/build_tether_force_plugin.sh
```

O plugin aplica:

```text
e = P_tether - P_drone
F_tether = -K e - C e_dot
F_drone  = -F_tether
```

As forcas sao aplicadas nos pontos de conexao, sem impor orientacao relativa. Topicos publicados:

```text
/cabo/conexao/error
/cabo/conexao/force
```

Primeiro sweep:

```text
N=5, L=2.50 m, K=20 N/m, C=4 N.s/m, Fmax=20 N
```

Resultado:

- topicos foram publicados;
- erro inicial pequeno, mas velocidades relativas saturaram a forca em `20 N`;
- DART abortou em `BallJoint::updateRelativeTransform`;
- diagnostico: parametros rigidos demais para a cadeia multibody com ball joints.

Segundo sweep:

```text
N=5, L=2.50 m, K=2 N/m, C=0.2 N.s/m, Fmax=2 N
ancora do prototipo inserida em z=2.38 m
offset no drone: base_link + (0, 0, -0.12) m
offset no tether: ponta do ultimo link
```

Comandos principais:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z --force-constraint --stiffness 2 --damping 0.2 --max-force 2
```

Spawn:

```bash
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 2.38}}'
```

Resultado:

- startup: PASS;
- spawn tether independente com plugin: PASS;
- topicos `/cabo/conexao/error` e `/cabo/conexao/force`: PASS;
- assentamento inicial: erro `~0.034 m`, forca `~0.068 N`;
- `commander arm`: PASS;
- `commander takeoff`: PASS;
- hover: PASS;
- `commander land`: PASS;
- desarme automatico apos pouso: PASS;
- failsafe observado: nao;
- atitude maxima via ULog: `|roll|max = 1.15 deg`, `|pitch|max = 0.94 deg`;
- altitude maxima local: `2.83 m`;
- RTF observado durante hover: aproximadamente `0.95` a `1.00`;
- erro de conexao em hover: aproximadamente `0.20 m`;
- forca de conexao em hover: aproximadamente `0.40 N`.

Interpretacao:

- o prototipo force-based e numericamente estavel com ganhos baixos;
- ele transmite forca e nao impoe orientacao relativa;
- ainda nao mantem a conexao suficientemente proxima de zero para substituir uma junta ideal;
- aumentar `K`/`C` sem cuidado pode tornar a simulacao stiff e abortar o DART.

Comparacao das alternativas:

| Arquitetura | inicia DART | conexao ~=0 | orientacao livre | momento artificial | hover | RTF | status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| tether independente sem conexao | sim | nao aplicavel | nao aplicavel | nao aplicavel | sim | `~0.99` | PASS |
| inter-model ball predeclarado no world | sim | sim no teste minimo | sim | esperado baixo | nao testado com PX4 | nao medido com PX4 | promissor, mas nao runtime |
| inter-model ball via `/world/create` runtime | nao | nao | nao | nao | nao | nao | FAIL |
| force constraint `K=20`, `C=4` | inicia, depois aborta | instavel | sim | sem torque direto | nao | nao | FAIL |
| force constraint `K=2`, `C=0.2` | sim | erro `~0.20 m` em hover | sim | sem torque direto | sim | `~0.95-1.00` | PASS parcial |

Recomendacao atual:

- manter X500/PX4 e tether como modelos independentes;
- enraizar o tether na ancora/estacao terrestre;
- nao usar a topologia com dois parents no ultimo link;
- para refazer a Etapa 5, a opcao mais viavel no fluxo PX4 atual e evoluir a conexao force-based, com sweep controlado de `K`, `C` e geometria inicial;
- a opcao `inter-model ball` deve ser preservada como alternativa preferencial mecanicamente, mas exige resolver como predeclarar o X500 no mundo sem conflitar com o spawn feito pelo PX4 `gz_bridge`, ou adicionar uma ferramenta/plugin nativo de criacao de joints em runtime.

Status final desta investigacao:

```text
PROTOTIPO MINIMO: PASS parcial
ARQUITETURA PRONTA PARA RETESTAR A ETAPA 5: NAO
```

### Etapa 5 - experimento minimo force-based aprovado

Este experimento e um novo contorno para a falha de closed kinematic loop da Etapa 5. Ele nao substitui uma junta ideal, mas valida que o X500/PX4 e o tether podem permanecer como modelos independentes e ainda trocar forcas sem criar segundo parent joint no DART.

Arquitetura implementada:

```text
WORLD
|-- x500_0
|   `-- base_link + attachment point em (0, 0, -0.12) m
`-- tether_anchor_chain
    |-- anchor_link fixo ao world
    `-- tether_link_1 ... tether_link_5
```

O modelo `tether_anchor_chain` e gerado por:

```text
tools/generate_tether_anchor_chain.py
```

A conexao translacional e implementada pelo plugin:

```text
src/pacote_do_drone/gz_plugins/TetherForceConstraint.cc
tools/build_tether_force_plugin.sh
```

Equacao aplicada a cada passo de simulacao:

```text
e     = p_tether - p_drone
e_dot = v_tether - v_drone

F_tether = -K e - C e_dot
F_drone  = -F_tether
|F| <= Fmax
```

Nao ha torque corretivo de orientacao. As forcas sao aplicadas em pontos deslocados nos corpos por `AddWorldForce`, permitindo que a dinamica resultante gere momentos fisicos quando houver braco de alavanca.

Topicos de instrumentacao:

```text
/cabo/conexao/error  vetor e = p_tether - p_drone [m]
/cabo/conexao/force  forca aplicada ao tether [N]
/cabo/conexao/stats  x=||e|| [m], y=||F|| [N], z=saturacao [0/1]
```

Configuracao geometrica validada:

```text
N_links = 5
L       = 2.50 m
rho     = 0.06 kg/m
m_total = 0.150 kg
m_link  = 0.030 kg
raio    = 0.003 m
anchor_chain spawn pose: z = 2.38 m
drone attachment offset: base_link + (0, 0, -0.12) m
```

Sweep executado em 2026-09-05:

| K [N/m] | C [N.s/m] | Fmax [N] | DART | arm/takeoff | hover | erro RMS [m] | erro max [m] | forca RMS [N] | forca max [N] | saturacao | roll max [deg] | pitch max [deg] | RTF | status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2.0 | 0.2 | 2.0 | PASS | PASS | PASS | 0.223 | 0.228 | 0.446 | 0.460 | 0% | 0.79 | 1.02 | ~1.00 | PASS |
| 3.0 | 0.3 | 2.0 | PASS | PASS | PASS | 0.145 | 0.146 | 0.435 | 0.439 | 0% | 1.04 | 1.02 | ~1.00 | PASS |
| 5.0 | 0.5 | 3.0 | PASS | PASS | PASS | 0.084 | 0.088 | 0.417 | 0.449 | 0% | 0.92 | 0.87 | ~0.99 | PASS |

Configuracao escolhida para os proximos testes minimos:

```text
K     = 5.0 N/m
C     = 0.5 N.s/m
Fmax  = 3.0 N
```

Justificativa: foi a menor combinacao testada que manteve o erro maximo de conexao abaixo de `0.10 m`, sem saturacao persistente, sem failsafe e com RTF proximo de 1.

Validacoes executadas:

```text
Gazebo + tether ancorado + X500 parado: PASS
DART sem abort: PASS
PX4 arm: PASS
PX4 takeoff: PASS
hover: PASS
land/disarm: PASS
failsafe: nao observado
```

Limitacoes observadas:

- a conexao force-based ainda permite erro finito entre o attachment point e a ponta do tether;
- `K=20`, `C=4`, `Fmax=20` ja havia produzido instabilidade numerica e abort do DART;
- este experimento valida a topologia e a troca de forcas, mas ainda nao valida tether completo com 50 links, trajetorias horizontais, sensor angular definitivo, carretel ou controle de comprimento.

Status:

```text
ETAPA 5 - EXPERIMENTO MINIMO FORCE-BASED: PASS
PROXIMO PASSO: validar gradualmente aumento de discretizacao, sem criar segundo parent joint.
```

#### Correcao de geometria da ancora suspensa

Foi identificada uma inconsistencia visual/fisica no experimento acima: a ancora do `tether_anchor_chain` aparecia suspensa no ar.

Causa:

```text
anchor_link pose local:       (0, 0, 0)
tether_anchor_chain spawn:    (0, 0, 2.38)
initial_axis anterior:        z, com links descendo no eixo -z
```

Ou seja, a posicao `z=2.38` era definida no frame do mundo como pose do modelo `tether_anchor_chain`. Como `anchor_link` tem pose local nula, isso colocava tambem a propria ancora em `z=2.38 m`. Esse deslocamento tinha sido usado para fazer a ponta inferior do cabo coincidir aproximadamente com o attachment do drone, mas fisicamente invertia o papel da ancora.

Correcao implementada:

- mantida a arquitetura force-based com X500/PX4 e tether como modelos independentes;
- mantidos `N=5`, `L=2.50 m`, `rho=0.06 kg/m`, `K=5`, `C=0.5`, `Fmax=3`;
- adicionado `--initial-axis folded_ground` em `tools/generate_tether_anchor_chain.py`;
- a cadeia de 5 segmentos e inicializada dobrada perto do solo, com cada segmento ainda medindo `0.50 m`;
- o offset da ponta usado pelo plugin passa a ser `tether_link_5 + (0.50, 0, 0)` no frame local do ultimo link;
- o modelo passa a ser inserido com `pose z=0.035`, deixando a esfera visual da ancora, de raio `0.035 m`, tangente ao solo.

Geometria corrigida:

```text
anchor model pose:       (0, 0, 0.035) no world
anchor_link pose local:  (0, 0, 0)
P_anchor fisico:         (0, 0, 0.035) m
X500 initial pose:       spawn PX4 default em torno da origem
drone attachment offset: base_link + (0, 0, -0.12) m
P_attach inicial usado:  aproximadamente (0, 0, 0.120) m
d inicial:               aproximadamente 0.085 m
L_tether:                2.50 m
slack inicial:           aproximadamente 2.415 m
```

Validacao apos correcao, em 2026-09-05:

```text
comando de geracao:
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3

spawn:
pose: {position: {z: 0.035}}
```

Resultados:

```text
Gazebo/DART startup: PASS
PX4 arm:             PASS
PX4 takeoff:         PASS
hover:               PASS
land/disarm:         PASS
failsafe:            nao observado
erro RMS hover:      0.199 m
erro max hover:      0.210 m
forca RMS hover:     0.993 N
forca max hover:     1.061 N
saturacao Fmax:      0%
roll max:            0.90 deg
pitch max:           0.90 deg
RTF:                 ~0.99
```

Observacao: a geometria corrigida remove a ancora suspensa, mas aumenta o erro de conexao em hover em relacao ao caso com ancora elevada. Isso e esperado para uma conexao spring-damper complacente com cabo inicialmente dobrado e ancorado no solo. O experimento continua aprovado como validacao topologica e dinamica minima da arquitetura force-based, mas ainda nao deve ser usado como tether fisicamente definitivo.

### Etapa 5 - reavaliacao pela referencia marsupial

Fonte principal lida integralmente:

```text
refs/README.md
refs/MARSUPIAL_TETHER_HANDOFF.md
refs/MARSUPIAL_TETHER_REFERENCE_ANALYSIS.md
```

Referencia analisada:

```text
repositorio: https://github.com/robotics-upo/marsupial_simulator_ros2
commit: d9046774cada2f0b679fb0dfdc1857516fc36936
engine: Gazebo Classic + ODE
modelos principais: rs_robot, tether, sjtu_drone
```

#### Causa confirmada da falha atual

**CONFIRMADO PELA REFERENCIA / CONFIRMADO NO PROJETO ATUAL:**

A Etapa 5 falhou porque a tentativa anterior fechou uma cadeia dentro do mesmo modelo:

```text
X500/base_link
  -> tether_attach_link
  -> tether_link_1
  -> ...
  -> tether_link_50
  -> tether_anchor_link
  -> world
```

Isso cria um segundo parent joint para `tether_link_50`. O DART usado pelo Gazebo Sim rejeitou essa topologia com:

```text
child link already has a parent joint
```

#### Estrategia observada na referencia

**CONFIRMADO PELA REFERENCIA:**

A referencia usa tres modelos independentes:

```text
rs_robot       # UGV + winch
tether         # cadeia articulada
sjtu_drone     # UAV
```

O tether interno e uma cadeia:

```text
tether/link_0 -> link_1 -> ... -> link_123 -> link_final
```

A conexao aos veiculos e feita apos o spawn por um WorldPlugin externo (`gazebo_ros_link_attacher`), chamado por `scripts/attach_tether.py`. Esse plugin cria joints runtime do Gazebo Classic/ODE:

```text
sjtu_drone/base_link -> tether/link_final
rs_robot/box_central -> tether/link_0
```

**CONFIRMADO PELA REFERENCIA:**

O attach criado nao e `ball`; e um `revolute` travado em zero. Tambem nao ha uso de `gz::sim::systems::DetachableJoint`, nem aplicacao direta de forcas para conectar endpoints. A referencia tambem nao mede tensao do cabo; as reacoes ficam dentro do solver.

#### Como a referencia evita o loop

**CONFIRMADO PELA REFERENCIA:**

A referencia nao evita o loop por uma arvore SDF/DART valida. Ela depende de constraints do Gazebo Classic/ODE. O proprio UGV contem uma estrutura nao-arvore: dois joints revolute com o mesmo child `box_central`.

**INFERENCIA:**

Portanto, a referencia nao oferece uma solucao diretamente portavel para o erro do DART. Ela mostra uma arquitetura de modelos separados e conexao runtime, mas o mecanismo concreto (`gazebo_ros_link_attacher`, `CreateJoint`, `SetModel`, ODE) e incompativel com Gazebo Sim/DART.

#### Comparacao direta

| Diferenca | Problema atual | Como a referencia resolve | Transferivel? | Adaptacao necessaria |
| --- | --- | --- | --- | --- |
| Raiz/topologia | Tether, drone e ancora foram fechados no mesmo modelo | Usa modelos separados e joints runtime Classic | PARCIALMENTE | Manter modelos separados, mas usar constraint suportada por Gazebo Sim/DART |
| Engine | Gazebo Sim + DART exige arvore de joints | Gazebo Classic + ODE tolera constraints/loops | NAO | Nao copiar attach Classic nem duplo parent |
| Conexao drone-cabo | `ball` dentro da mesma arvore funcionou so com extremidade livre | Runtime attach `revolute` travado | NAO diretamente | Preservar conceito de endpoint, nao o tipo/API da junta |
| Ancora/ground station | Ancora fixa ao mundo criou fechamento rigido | Ground station e UGV movel, nao fixed-to-world | PARCIALMENTE | Comecar com ancora ideal, mas sem dar segundo parent a um link |
| Comprimento variavel | Ainda nao implementado no X500 | Tambor gira; cadeia inteira permanece simulada | PARCIALMENTE | Futuro: um unico joint estrutural no tambor, sem duplo rolamento |
| Tensao | Sensor de force_torque ainda indisponivel | Nao mede tensao | NAO | Instrumentar wrench/forca no projeto atual |
| Forca direta | Prototipo local existe, mas ainda frouxo | Nao existe na referencia | ADAPTACAO PROPOSTA | Evoluir spring-damper/constraint unilateral com metricas |

#### Alternativas avaliadas para refazer a Etapa 5

1. **Modelos independentes + inter-model ball predeclarado no world**

   **ADAPTACAO PROPOSTA / HIPOTESE A VALIDAR.**

   Vantagens: preserva orientacao livre, transmite forca como constraint estrutural e se aproxima do requisito fisico. Um mundo minimo local ja confirmou que `ball joint` entre modelos funciona quando declarado no `world` antes do startup.

   Limite: no fluxo PX4 atual, o X500 e criado em runtime pelo `gz_bridge`; ainda nao ha mecanismo confirmado para predeclarar a junta entre `x500_0` e `tether_anchor_chain` sem conflitar com o spawn do PX4.

2. **Modelos independentes + conexao por forca spring-damper**

   **ADAPTACAO PROPOSTA.**

   Vantagens: preserva X500/PX4 independente, evita loop de joints, permite publicar diretamente `/cabo/conexao/error` e `/cabo/conexao/force`, e nao impoe orientacao relativa. Um prototipo 5-link local passou em startup, arm, takeoff, hover, land e disarm com ganhos baixos.

   Limite: erro de conexao em hover ficou em torno de `0.20 m`; ganhos maiores podem tornar o sistema numericamente stiff e abortar o DART. Ainda nao substitui uma junta ideal.

3. **Unica arvore orientada estacao -> tether -> X500**

   **INFERENCIA / HIPOTESE A VALIDAR.**

   Vantagens: satisfaz a exigencia de uma arvore DART.

   Limite: pode alterar canonical link, pose do modelo, pressupostos do PX4/Gazebo e comportamento dos rotores. A referencia explicitamente nao demonstra que isso funcione com PX4.

4. **DetachableJoint do Gazebo Sim**

   **HIPOTESE A VALIDAR.**

   A referencia nao usa esse recurso. Documentacao analisada na referencia indica restricoes de arvore e contato; portanto nao deve ser assumido como equivalente ao `gazebo_ros_link_attacher`.

#### Solucao recomendada nesta fase

**ADAPTACAO PROPOSTA:**

Refazer a Etapa 5 com dois modelos independentes:

```text
WORLD
|-- X500/PX4
|   `-- base_link + ponto de attachment
`-- tether_anchor_chain
    |-- anchor_link fixo ao world
    `-- tether_link_1 ... tether_link_N
```

Primeira tentativa recomendada: evoluir a conexao por forca com 5 links, porque ela ja foi validada minimamente no fluxo real PX4/Gazebo Sim sem fechar loop.

Manter em paralelo a alternativa `inter-model ball` predeclarada no `world`, mas so avancar nela depois de validar um fluxo em que o PX4 use um X500 ja existente no mundo ou permita declarar a junta apos spawn por outro mecanismo nativo.

#### Experimento minimo proposto

**HIPOTESE A VALIDAR:**

```text
X500 puro do PX4
tether_anchor_chain com 5 links
anchor_link fixo
conexao force-based entre:
  P_drone  = x500_0::base_link + offset (0, 0, -0.12)
  P_tether = ponta do ultimo link
```

Geometria inicial:

- escolher pose da ancora para que `||P_tether - P_drone|| < 5 cm` antes do arm;
- manter comprimento total suficiente para o takeoff nao tensionar imediatamente o drone;
- iniciar com `K` e `C` baixos e fazer sweep controlado.

Variaveis a registrar:

```text
/cabo/conexao/error
/cabo/conexao/force
vehicle_local_position
vehicle_attitude
RTF
failsafe/status PX4
```

Criterios PASS:

- Gazebo inicia sem erro;
- DART nao aborta;
- X500 permanece como modelo independente;
- tether permanece ancorado e conectado por forca;
- erro de conexao estacionario menor que valor acordado para o prototipo;
- roll/pitch baixos em hover;
- sem failsafe;
- RTF utilizavel;
- forca limitada, sem saturacao persistente;
- sem torque direto de orientacao aplicado pelo plugin.

Criterios FAIL:

- DART aborta;
- PX4 nao arma/decola;
- erro de conexao cresce sem limite;
- forca satura continuamente;
- roll/pitch altos ou failsafe;
- RTF inviavel;
- comportamento visual/fisico nao plausivel.

## 5. Sensor angular apos validar dinamica

O sensor deve ser incorporado somente depois que a Etapa 5 estiver estavel.

Metodo recomendado:

```text
poses Gazebo do X500 e segmentos do tether
  -> tangente local em uma janela fisica perto do drone
  -> vetor no frame do X500
  -> azimuth/elevation
```

Pontos a adaptar:

- substituir referencias a `meu_drone` por `x500_0` ou pelo nome real do modelo instanciado;
- identificar o frame/link correto do corpo do X500, provavelmente `x500_0::base_link`;
- manter aliases de topicos explicitos para lado do drone e lado da ancora;
- manter `janela_tangente_metros` como parametro.

PASS:

- cabo vertical com drone nivelado produz elevacao proxima de `90 deg`;
- inclinacao do drone altera os angulos no frame do drone;
- N/S/E/W produzem sinais coerentes;
- medicao por tangente local e comparacao geometrica independente sao registradas separadamente.

FAIL:

- angulos calculados no frame global por engano;
- comparacao direta com reta ancora-drone quando o cabo esta frouxo;
- dependencia fixa do indice do segmento em vez de janela fisica;
- saturacao artificial de elevation.

## 6. Validacao N/S/E/W do sensor

Depois da dinamica estavel, reproduzir:

```text
N
S
E
W
```

Critica metodologica:
nao depurar controle, tether e sensor ao mesmo tempo.

Sequencia por caso:

1. iniciar de uma configuracao slack conhecida;
2. atingir waypoint estacionario;
3. esperar acomodacao;
4. medir tangente local no lado do drone;
5. medir tangente local no lado da ancora;
6. calcular referencia geometrica independente;
7. comparar apenas na janela estacionaria.

## 7. Carretel: integracao posterior

### Primeiro: ancora ideal

Antes do carretel, manter:

```text
fixed anchor
```

Isso separa problemas do drone/tether dos problemas do mecanismo do carretel.

### Plataforma estatica

Depois criar:

```text
ground_station
```

Conteudo inicial:

- base visual;
- ponto de saida do cabo;
- frame do carretel;
- ancora fixa equivalente a ja validada.

### Carretel A - Geometria visual

Adicionar apenas visual do carretel, sem mudar a fisica do tether.

PASS:

- visual coerente;
- ponto de saida coincide com ancora validada;
- dinamica do cabo nao muda.

### Carretel B - Tambor rotativo

Adicionar:

```text
revolute joint
```

para o tambor, ainda sem alterar comprimento efetivo do cabo.

PASS:

- tambor gira;
- junta publica estado;
- tether continua preso ao ponto fixo.

### Carretel C - Sensor de comprimento

Relacionar:

```text
comprimento = angulo_tambor * raio_efetivo
```

e publicar estimativa de comprimento desenrolado.

PASS:

- leitura monotônica e coerente;
- topico publicado;
- sem efeito fisico no tether ainda.

### Carretel D - Comprimento variavel

Esta e a etapa mais dificil.

Abordagens viaveis a estudar:

1. Regerar/reinstanciar cabo com outro comprimento entre ensaios, nao em tempo real.
2. Manter comprimento fisico fixo e modelar apenas comprimento desenrolado como variavel de sensor/controle.
3. Usar segmentos "armazenados" no carretel e ativar/desativar restricoes, se o Gazebo permitir com estabilidade.
4. Implementar plugin Gazebo especifico para aplicar forca/tensao equivalente sem discretizar todo o cabo ativo.
5. Hibridizar: cabo discretizado perto do drone + modelo analitico de cabo/carretel no restante.

Nao implementar essa etapa antes de validar o tether ancorado com comprimento fixo.

## 8. Arquitetura recomendada de software

Estrutura sugerida a partir do repositorio atual:

```text
drone-cabo/
  src/
    pacote_do_drone/
      config/
        px4/
          tether_x500_short.json
          tether_x500_full_free.json
          tether_x500_full_anchor.json
      launch/
        start_sim.launch.py
        start_px4_tether.launch.py          # futuro
      models/
        cabo.sdf                            # gerado
        gerar_cabo.py
        x500_tethered/                      # overlay do projeto
          model.config
          model.sdf
        ground_station/                     # futuro
          model.config
          model.sdf
      pacote_do_drone/
        cabo_angulos.py
        sensores.py
        cabo_monitor.py
        experimento_tracking.py
  px4/
    README.md
    PX4-Autopilot/                          # clone local ignorado
  docs/
    PX4_INTEGRATION.md
    PX4_SIMULATION_GUIDE.md
    X500_TETHER_INTEGRATION_PLAN.md
```

Separacao de responsabilidades:

```text
PX4-Autopilot:
  manter upstream intacto sempre que possivel
  usar v1.14.4 como base atual
  evitar airframe customizado ate ser necessario

projeto drone-cabo:
  modelos de tether
  overlay x500_tethered
  mundos de teste
  launch ROS/Gazebo
  sensores de angulo
  metricas
  documentacao
```

## 9. Ordem recomendada de implementacao

1. Criar `x500_tethered` externo com apenas `tether_attach_link`.
2. Validar spawn/hover do `x500_tethered` sem cabo.
3. Adicionar teste de carga minima/pendulo por `ball`.
4. Adaptar gerador do cabo para perfis PX4 curtos (`N=5`, `N=10`).
5. Conectar tether curto com raiz livre.
6. Conectar tether completo com raiz livre.
7. Conectar tether completo a ancora fixa.
8. Adaptar sensores para nomes/frames do X500.
9. Reproduzir N/S/E/W.
10. Criar `ground_station` visual com ancora ideal.
11. Evoluir carretel em A/B/C/D.

## 10. Decisao tecnica

Decisao:

```text
Nao modificar o X500 upstream nesta fase.
Criar uma variante/overlay x500_tethered no projeto tethered drone.
Reutilizar o tether atual, mas integrando em etapas com PASS/FAIL claros.
Validar dinamica antes de validar sensor angular.
Validar ancora fixa antes de modelar carretel.
```

Justificativa:
essa estrategia minimiza risco, mantem reprodutibilidade com PX4 v1.14.4, preserva a comparacao com o X500 original e evita misturar problemas de controle, junta, cabo, sensor e carretel em uma unica etapa.

## 11. Decisao posterior: MuJoCo de referencia e cabo nodal no Gazebo

Decisao consolidada em 2026-10-06: usar o MuJoCo `N=70` como referencia fisica
offline e manter Gazebo/PX4 como plataforma operacional. Os testes posteriores
mostraram que aumentar a cadeia articulada nao e uma rota de producao: `N=20`
passa em estatico, mas aborta em voo, enquanto cadeias maiores encontram limites
de construcao/transformadas do DART. O proximo desenvolvimento e um cabo nodal em
plugin proprio, sem joints estruturais entre os nos.

Plano, metricas e gates atuais: `docs/SIMULATION_STRATEGY.md`.
