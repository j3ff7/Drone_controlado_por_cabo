# Conexao tether ↔ drone: `dev` x `shared`

Analise feita em 2026-09-14 lendo o codigo, sem assumir a arquitetura. A `dev` foi
inspecionada num snapshot de `origin/dev` (`76be39a`, extraido com `git archive`, sem
checkout); a `shared` no working tree (`6909ff0` + ajustes desta rodada).

## 1. Branch `dev`

### 1.1 Achado principal: nao ha X500 na `dev`

Nenhum arquivo da `dev` referencia `x500`, `PX4_GZ_MODEL` ou `gz_x500`, e o historico
tambem nao (`git log origin/dev -S x500` vazio). O drone usado no Gazebo da `dev` e um
modelo proprio, **`meu_drone`** (nome interno do SDF: `meu_drone_v2`), controlado pelo
proprio Gazebo:

| Plugin em `meu_drone.sdf` | Funcao |
| --- | --- |
| `gz::sim::systems::MulticopterVelocityControl` | controle de velocidade em `/meu_drone/cmd_vel` |
| `gz::sim::systems::MulticopterMotorModel` (x4) | rotores |
| `gz::sim::systems::OdometryPublisher` | `/meu_drone/odom` |
| `gz::sim::systems::ForceTorque` | sensores de forca/torque |

PX4 aparece na `dev` apenas em `src/pacote_do_drone/pacote_do_drone/mission_test1.py`
(e na copia `PX4_missions_test/mission_test1.py`): um no ROS 2 que usa `px4_msgs` em
`/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` e
`/fmu/out/vehicle_local_position`. Ele nao referencia modelo, mundo nem cabo — nao ha
integracao PX4 ↔ Gazebo ↔ tether na `dev`.

### 1.2 Arquivos relevantes

| Arquivo | Papel |
| --- | --- |
| `src/pacote_do_drone/worlds/my_world.sdf` | **define a conexao**: inclui carretel, cabo e drone e declara os dois joints de ligacao |
| `src/pacote_do_drone/models/build_world.py` | gera `models/cabo.sdf` e `worlds/my_world.sdf` a partir do JSON |
| `src/pacote_do_drone/models/cabo.sdf` | cabo usado pelo mundo: `raiz_cabo` + 70 segmentos + `ponta_cabo` |
| `src/pacote_do_drone/models/meu_drone/meu_drone.sdf` | drone; tem `tether_attach`, mas ele **nao e usado** na conexao |
| `src/pacote_do_drone/models/carretel/carretel.sdf` | estacao: base fixa ao mundo + tambor em junta revolute com `JointController` |
| `src/pacote_do_drone/tether_package/parameters/tether_parameters.json` | parametros lidos por `build_world.py` |
| `src/pacote_do_drone/tether_package/build_tether.py` | gerador alternativo de um modelo de cabo avulso |
| `src/pacote_do_drone/tether_package/models/cabo/model.sdf` | saida desse gerador — **nao incluida por nenhum mundo** |
| `src/pacote_do_drone/launch/start_sim.launch.py` | sobe `my_world.sdf` via `ros_gz_sim` + bridge ROS |
| `src/pacote_do_drone/models/Gazebo/cabo.sdf` | cabo legado com `DetachableJoint` (`segment_35` → `ground_plane`); so referenciado por um SDF antigo com caminho absoluto de outra maquina — **fora da arquitetura ativa** |

### 1.3 Topologia

```text
world
  └─(fixed: ancora_mundo)──────────── meu_carretel::base_suporte
        └─(revolute: junta_eixo, JointController /carretel/velocidade)
              meu_carretel::cilindro_carretel
                 └─(BALL, junta de MUNDO: ancora_carretel_cabo)
                       cabo_dinamico::raiz_cabo
                          └─(universal ±30°) segment_1 ─ ... ─ segment_70
                                └─(universal ±30°: joint_ponta) cabo_dinamico::ponta_cabo
                                      └─(BALL, junta de MUNDO: cabo_drone_joint)
                                            meu_drone::base_link   ← raiz da arvore do drone
                                               └─(fixed) tether_attach   (definido, nao usado)
```

A arvore e **aberta**: o drone vira filho da ponta do cabo pelo seu `base_link`, que e a raiz
da propria arvore do drone. Por isso o `DART` a aceita; ligar o cabo a `tether_attach`, que ja
tem pai (`base_link`), criaria um laco cinematico fechado.

### 1.4 Trecho do SDF responsavel pela conexao (`worlds/my_world.sdf`)

```xml
<include>
  <uri>model://carretel</uri>
  <name>meu_carretel</name>
  <pose>0 0 0 0 0 0</pose>
</include>

<include>
  <uri>model://cabo.sdf</uri>
  <name>cabo_dinamico</name>
  <pose>0.0 0.18 0.335 0 0 0</pose>
  <static>false</static>
</include>

<joint name="ancora_carretel_cabo" type="ball">
  <parent>meu_carretel::cilindro_carretel</parent>
  <child>cabo_dinamico::raiz_cabo</child>
  <pose>0 0 0 0 0 0</pose>
</joint>

<include>
  <uri>model://meu_drone</uri>
  <name>meu_drone</name>
  <pose>2.097274 0.180000 0.235039 0 0 0.000000</pose>
</include>

<joint name="cabo_drone_joint" type="ball">
  <parent>cabo_dinamico::ponta_cabo</parent>
  <child>meu_drone::base_link</child>
</joint>
```

### 1.5 Respostas as perguntas da rodada

| Pergunta | Resposta pela `dev` |
| --- | --- |
| A conexao e definida no SDF do drone? | **Nao.** E definida no SDF do **mundo**, por joints de nivel de mundo. |
| Existe link/joint/frame especifico? | O drone tem `tether_attach` (esfera, `fixed` a `base_link`, z = -0,007 m), mas o joint de ligacao usa **`meu_drone::base_link`**. |
| O `tether_package` so fornece o cabo? | Fornece **parametros** (JSON lido por `build_world.py`) e um gerador de cabo avulso cuja saida nao e usada. **Nao modifica o drone** nem define a conexao. |
| Ha dependencia entre modelo do cabo e do drone? | So pelo mundo: os nomes `cabo_dinamico::ponta_cabo` e `meu_drone::base_link` e a pose do drone, calculada por `build_world.py` para coincidir com a ponta. |
| Estatica no SDF ou em runtime por plugin? | **Estatica no SDF.** O unico `DetachableJoint` esta no cabo legado, fora do mundo ativo. |
| Juntas internas do cabo | `universal` com limite de ±30° (`<dynamics>` fora de `<axis>`, entao damping/atrito sao ignorados). |
| Carretel | revolute com `JointController` de velocidade; sem comando, fica parado. Sem payout. |

## 2. Branch `shared` antes do ajuste (`6909ff0`)

### 2.1 Arquivos relevantes

| Arquivo | Papel |
| --- | --- |
| `src/pacote_do_drone/models/x500_tether_attach/model.sdf` | X500 do PX4 + `tether_attach_link` (`fixed` a `base_link`, `0 0 -0.12`), sem cabo; gerado por `tools/generate_x500_tether_attach.py` |
| `src/pacote_do_drone/models/tether_anchor_chain/model.sdf` | estacao + guia de saida + reel + cabo, **modelo separado**; gerado por `tools/generate_tether_anchor_chain.py` |
| `src/pacote_do_drone/gz_plugins/TetherForceConstraint.cc` | **define a conexao com o drone**: constraint de forca, nao junta |
| `src/pacote_do_drone/gz_plugins/ReelActuator.cc` | torque no `reel_joint` (zero sem comando) |
| `src/pacote_do_drone/models/x500_tethered/model.sdf` | variante legada: X500 com cabo livre embutido (ball), sem ancora |

### 2.2 Topologia

```text
PX4 gz_bridge spawna:   x500_tether_attach_0
                           base_link ─(fixed)─ tether_attach_link  ◄──┐
                                                                     │ forca F = -K e - C e'
servico /world/default/create spawna:                                 │ (K=5, C=0,5, Fmax=3 N)
   world ─(fixed)─ ground_station_base ─(fixed)─ tether_exit_point    │ NENHUMA junta
                        └─(revolute, desacoplado) reel_link           │
         tether_exit_point ─ tether_joint_1 ─ ... ─ tether_link_10 ───┘
                           (ball na baseline validada;
                            universal no model.sdf commitado em 6909ff0 — erro)
```

Trecho responsavel pela conexao (`tether_anchor_chain/model.sdf`, `6909ff0`):

```xml
<plugin filename="libTetherForceConstraint.so" name="drone_cabo::TetherForceConstraint">
  <drone_model>x500_tether_attach_0</drone_model>
  <drone_link>tether_attach_link</drone_link>
  <tether_model>tether_anchor_chain</tether_model>
  <tether_link>tether_link_10</tether_link>
  <drone_offset>0 0 0</drone_offset>
  <tether_offset>0.25 0 0</tether_offset>
  <stiffness>5</stiffness>
  <damping>0.5</damping>
  <max_force>3</max_force>
</plugin>
```

Por que a `shared` foi para forca: o plano registra que dar um segundo pai ao ultimo elo
(`--anchored` sobre o `x500_tethered`, cujo cabo ja nasce filho do drone) criava um laco
fechado e derrubava o DART, e a constraint de forca manteve as duas arvores independentes.

## 3. Comparacao `dev` x `shared`

| Aspecto | `dev` | `shared` antes do ajuste |
| --- | --- | --- |
| Drone | `meu_drone` proprio, controle de velocidade do Gazebo; **sem X500 e sem PX4 no Gazebo** | X500 do PX4 (`x500_tether_attach`), spawnado pelo `gz_bridge` |
| Onde a conexao e definida | **SDF do mundo** (`my_world.sdf`) | plugin `TetherForceConstraint` dentro do modelo do tether |
| Conexao no drone | **junta `ball`** ponta do cabo → `meu_drone::base_link` | **forca** mola-amortecedor ponta → `tether_attach_link` (K=5, C=0,5, Fmax=3 N) — nenhuma junta |
| Conexao no solo | junta `ball` tambor → raiz do cabo | junta interna do cabo na guia `tether_exit_point`, fixa a base da estacao |
| Juntas internas do cabo | `universal` ±30° | `ball` na baseline validada (`universal` no `model.sdf` de `6909ff0`, por erro) |
| Carretel | revolute + `JointController` de velocidade, parado sem comando | revolute desacoplado do cabo + `ReelActuator` (torque zero sem comando) |
| Montagem | tudo num unico mundo estatico | dois modelos separados, cabo inserido em runtime pelo servico `create` |
| Instrumentacao do cabo | sensores `force_torque` nas juntas | `/cabo/conexao/*`, `/cabo/estacao/tensao` (`T_est`) calculados pelo plugin |

A diferenca que importa e o **lado do drone**: fisico (junta) na `dev`, forca na `shared`.
O lado do solo e fisico nas duas; a `shared` usa uma guia fixa em vez do tambor (decisao de
A3, que mostrou que prender o cabo na borda do tambor o transforma em manivela), o que, com o
carretel estatico, e equivalente a ancorar no tambor parado.

## 4. Ajuste feito na `shared` (esta rodada)

Reproduzida a arquitetura da `dev` com o X500, sem copiar arquivos da `dev`:

- `tools/generate_x500_tether_world.py` gera `src/pacote_do_drone/worlds/x500_tether_joint.sdf`:
  le o mundo `default` do PX4 (sem modifica-lo) e acrescenta o `tether_anchor_chain`, o
  `x500_tether_attach` com nome `x500_tether_attach_0` e a **junta de mundo `ball`**
  `tether_anchor_chain::tether_link_N` → `x500_tether_attach_0::base_link`, com o pivo em
  `0 0 -0.12` (o `tether_attach_link`). O filho e o `base_link` pela mesma razao da `dev`: e a
  raiz da arvore do X500, entao nao ha laco fechado.
- O tether e gerado **sem** `TetherForceConstraint`, sem `ReelActuator`, com juntas `ball`,
  colisoes e sem prismatica (corrige tambem o `universal` commitado por engano).
- O PX4 se liga ao X500 ja existente com `PX4_GZ_MODEL_NAME=x500_tether_attach_0`; o clone do
  PX4 nao e alterado.
- `--taut-bulge up` no gerador do tether: o arco inicial nasce acima do solo e cai sobre ele.
  Com a folga necessaria para subir 2 m, o arco para baixo nasceria ~0,6 m dentro do solo, e
  elos que nascem abaixo do plano ficam presos la (era a origem do `z_min` negativo de B1/B2).
  O padrao continua `down`, com saida byte a byte igual.

Trecho gerado responsavel pela conexao no drone:

```xml
<include>
  <uri>model://tether_anchor_chain</uri>
  <name>tether_anchor_chain</name>
  <pose>0 0 0 0 0 0</pose>
</include>
<include>
  <uri>model://x500_tether_attach</uri>
  <name>x500_tether_attach_0</name>
  <pose>1.200000 0.000000 0.227000 0 0 0</pose>
</include>
<joint name="tether_drone_ball" type="ball">
  <parent>tether_anchor_chain::tether_link_10</parent>
  <child>x500_tether_attach_0::base_link</child>
  <pose>0 0 -0.120000 0 0 0</pose>
</joint>
```

Topologia resultante:

```text
world ─(fixed)─ ground_station_base ─(fixed)─ tether_exit_point
                      └─(revolute, desacoplado, sem atuador) reel_link
      tether_exit_point ─(ball)─ tether_link_1 ─(ball)─ ... ─(ball)─ tether_link_10
            └─(BALL, junta de mundo: tether_drone_ball, pivo = tether_attach_link)
                  x500_tether_attach_0::base_link ─(fixed)─ tether_attach_link
                                                  └─(revolute x4) rotores
```

Consequencia na instrumentacao: sem a constraint de forca nao existem mais `|F_uav|` nem
`T_est` do plugin (`T_est` usava a forca da constraint no balanco de corpo livre, e ler o
wrench da junta via `TransmittedWrench` ja derrubou o DART nesta stack). O erro de conexao
passa a ser medido cinematicamente por `tools/record_tether_connection.py`: distancia entre a
ponta do ultimo elo e o pivo no drone, que numa junta integra deve ficar em ~0.

## 5. Validacao

### 5.1 Primeira geometria: X500 a 1,2 m da estacao

**Estatico (sem PX4), 44,8 s simulados — PASS.** Spawn dos 3 modelos; folga no pivo do drone
de 1,2e-6 m (max) e na guia de 1e-16 m; cabo assentado apoiado no solo (`z_min` = +2,2 mm, o
raio de colisao); 0 NaN; RTF 0,996; sem aborto. O log do Gazebo emite `No joint named
[tether_drone_ball] for modelID [1]`, mas o voo abaixo prova que a junta existe: a ponta do
cabo acompanhou o drone por 1,9 m de subida com folga de 1e-9 m.

**Vertical — FAIL.** Sequencia observada nas poses do Gazebo:

| t simulado | altura do `base_link` | posicao XY | guia→ponta do cabo | roll / pitch |
| --- | --- | --- | --- | --- |
| 10 s (decolagem) | 0,24 m | (1,18, 0,00) | 1,18 m | 0 / 0 |
| 12 s | 1,52 m | (1,56, -0,20) | 1,99 m | 6 / -5 graus |
| 13 s | 2,13 m | (0,96, -1,11) | 2,33 m | 11 / -4 graus |
| 13,5–13,9 s | cai de 1,85 a 0,27 m | circula a estacao a ~2,5 m | **2,48 m (cabo reto)** | ate 44 / 41 graus |
| 13,96 s | — | — | — | roll 105 → -160 graus; **aborto** |

Assinatura: `dart/dynamics/BallJoint.cpp:159: BallJoint::updateRelativeTransform(): Assertion
'math::verifyTransform(mT)'`, em `PhysicsPrivate::Step`. O PX4 entrou em failsafe (`blind
descent`, `blind land`) depois que o simulador parou; a ferramenta de missao, que nao checa o
simulador, reportou `failed = false`.

Leitura: a conexao nao falhou (folga de 1e-9 m ate o aborto). O que falhou foi a geometria.
A estimativa do PX4 ja estava em z = +0,19 m antes de decolar, entao o alvo de 2 m virou 2,2 m
de subida real; somado ao arrasto lateral de mais de 2 m, o cabo de 2,5 m ficou **reto**. Com
uma junta rigida nas duas pontas, cabo reto significa o drone preso a um raio fixo em torno da
estacao: ele foi chicoteado, virou, e a cadeia divergiu. E a mesma classe de falha de B1/C
(cabo curto para a geometria do teste), agora sem a complacencia da constraint de forca.

### 5.2 Segunda geometria: X500 a 0,5 m da estacao

Refeita para tirar a hipotese de cabo curto: com 0,5 m de distancia, subir 2,2 m usa ~87% do
cabo. Script: `tools/run_x500_joint_campaign.sh 0.5`.

**Estatico (sem PX4), 44,8 s simulados — PASS.** Folga no pivo 2,9e-7 m (max), na guia
1e-16 m; o arco inicial (espelhado para cima, corda 0,5 m) caiu e o cabo assentou apoiado no
solo (`z_min` = +2,2 mm); drone parado (roll/pitch <= 0,3 graus); 0 NaN; RTF 0,997; sem aborto.

**Vertical — FAIL**, praticamente no mesmo instante da primeira geometria:

| t simulado | altura | vz | XY | guia→ponta | cabo no solo? | roll / pitch |
| --- | --- | --- | --- | --- | --- | --- |
| 10,0 s (decolagem) | 0,25 m | 0,3 m/s | (0,49, 0,00) | 0,50 m | sim | 1 / 0 |
| 11,6 s | 1,03 m | 0,5 m/s | (0,61, -0,22) | 0,96 m (38%) | sim | 1 / 0 |
| 11,9 s | 1,19 m | 0,8 m/s | (0,65, -0,32) | 1,15 m (46%) | sim | -3 / -5 |
| 12,1 s | 1,42 m | 0,9 m/s | (0,68, -0,38) | 1,34 m (54%) | sim | 2 / **16** |
| 12,4 s | 1,68 m | 1,4 m/s | (0,77, -0,53) | 1,68 m (67%) | ultimos elos saindo | **-26** / 5 |
| 12,9 s | 2,18 m | 0,5 m/s | (1,25, 0,26) | 2,28 m (91%) | nao | -18 / -6 |
| 13,2 s | 2,08 m | -0,3 m/s | (1,43, 0,90) | 2,44 m (97%) | nao | 1 / 14 |
| 13,7 s | 0,23 m | -1,3 m/s | (1,94, 1,66) | 2,44 m | nao | **-75** / 6 |

Aborto em ~13,8 s (`BallJoint::updateRelativeTransform`), folga no pivo de 2,4e-7 m ate o
fim. PX4: `Takeoff detected`, `Imbalanced propeller detected`, `Failsafe activated`,
`blind descent/land`; Gazebo registra `Aliasing on motor` (comandos de motor mudando rapido
demais para o passo de 4 ms). A estimativa do PX4 nasceu em z = -0,42 m no solo.

Leitura: **esta falha nao e cabo curto.** A instabilidade comeca a ~1,2 m de altura com o cabo
a 46% do comprimento e parte dele ainda no solo, e so depois o drone deriva, o cabo fica reto e
o chicote derruba a simulacao. O candidato principal e o **acoplamento rigido**: o cabo sendo
descolado do solo elo a elo, puxando o drone por um ponto 0,12 m abaixo do centro de massa sem
nenhuma complacencia. A secao 5.3 testa exatamente isso com um controle.

### 5.3 Controle: mesma geometria com a constraint de forca original

Para separar "junta rigida" de "o que o cabo faz na subida", o mesmo voo foi repetido com a
conexao original da `shared` (`TetherForceConstraint`, K=5, C=0,5, Fmax=3 N): X500 a 0,5 m, o
mesmo cabo, o mesmo arco inicial, colisoes, mesma missao. O X500 foi spawnado pelo PX4, como
nas rodadas A.

Ressalva: o PX4 spawna em z = 0 e sobrescreve a pose propria do modelo, entao o X500 repousa
afundado (`base_link` a z ≈ 0,02 m, contra 0,227 m no mundo com juntas) e a ponta do cabo nasce
abaixo do solo. Nao e um controle perfeito — os elos proximos do drone ja partem com `z_min` de
-0,04 m.

**Resultado — o simulador tambem abortou**, com a mesma assinatura
(`BallJoint::updateRelativeTransform`), a 14,2 s simulados e 1,88 m de altura:

| t simulado | altura | erro da constraint | guia→ponta | `z_min` do cabo | roll / pitch |
| --- | --- | --- | --- | --- | --- |
| 10,0 s | 0,06 m | 0,03 m | 0,58 m | -0,029 m | 0 / 0 |
| 11,3 s | 1,11 m | 0,11 m | 0,87 m | -0,027 m | -1 / 0 |
| 12,6 s | 1,58 m | 0,12 m | 1,25 m | -0,026 m | 1 / 0 |
| 13,9 s | 1,84 m | 0,15 m | 1,47 m | **+0,002 m** (sai do solo) | 0 / 1 |
| 14,2 s | 1,87 m | **0,94 m** | 1,51 m | **+0,171 m** | 2 / -1 |

Plugin: `|e|` RMS 0,154 m e max 2,54 m; `|F_uav|` RMS 0,76 N e max 3,00 N (saturado em 2,5%
das amostras); `T_est` RMS 1,07 N e max 3,25 N. A ferramenta de missao voltou a dizer
`failed = false`.

Leitura: com a forca limitada a 3 N o drone **nao inclinou** (roll/pitch <= 2 graus), mas o
simulador morreu do mesmo jeito, e no mesmo evento — o instante em que o ultimo trecho do cabo
sai do solo (`z_min` de -0,025 para +0,17 m em 0,25 s, com o erro saltando de 0,15 para
0,94 m). Na execucao com junta rigida a instabilidade tambem comecou quando o cabo descolava do
solo; la, sem complacencia, a carga passou inteira ao drone e o virou antes do aborto.

O fator comum as tres quedas e o **cabo com colisoes sendo arrancado do solo**, nao o tipo de
conexao. Isso casa com o historico: os voos que passaram nas rodadas A eram **sem colisoes nos
elos**, e todos os voos com colisoes (B1 com N=20, a visualizacao com N=10, e esta rodada)
cairam. A secao 5.4 testa essa hipotese diretamente.

### 5.4 Diagnostico: juntas fisicas sem colisoes nos elos

Tentativa de isolar o contato cabo-solo: mesma arquitetura e geometria (0,5 m), elos **sem**
`<collision>` (`tools/run_x500_joint_campaign.sh 0.5 D0p5_sem_colisao --no-collisions`).

- **Estatico: aborto na dinamica logo apos carregar**, antes de qualquer amostra:
  `dart/dynamics/detail/GenericJoint.hpp:2002: GenericJoint::addChildBiasForceToDynamic`, em
  `Skeleton::computeForwardDynamics` → `BodyNode::updateBiasForce`. Sem colisao, o laco inicial
  (corda de 0,5 m para 2,5 m de cabo) cai livre atraves do solo e chicoteia a cadeia presa
  rigidamente ao X500 parado. E a mesma classe de falha do "laco folgado sem colisoes" de B1
  rodada 1.
- **Vertical: nao executado de fato.** O `gz sim` morreu antes do PX4; o PX4 subiu o proprio
  mundo `default`, sem o X500, e a missao expirou (`local position or attitude timeout`). O
  script de campanha agora recusa iniciar o PX4 sem o mundo vivo.

O diagnostico e **inconclusivo** para a hipotese do contato com o solo. O que ele mostra e que,
sem colisoes, a folga que a missao exige nem se sustenta estaticamente.

## 6. Conclusao

| Item | Resultado |
| --- | --- |
| Arquitetura da `dev` | juntas `ball` **no SDF do mundo** nas duas pontas; no drone, filho = `base_link` (raiz da arvore) |
| X500 na `dev` | **nao existe**; o drone e `meu_drone`, controlado pelo Gazebo, sem PX4 no lazo |
| Papel do `tether_package` | parametros do cabo + gerador avulso nao usado; nao toca no drone |
| `shared` antes | conexao no drone por **forca** (`TetherForceConstraint`), sem junta; `model.sdf` commitado com juntas `universal` por erro |
| Ajuste na `shared` | mundo gerado com junta `ball` de mundo ponta → `x500_tether_attach_0::base_link` (pivo no `tether_attach_link`), cabo `ball` sem plugin, reel estatico, PX4 ligado por `PX4_GZ_MODEL_NAME` |
| Estatico | **PASS** nas duas geometrias (folga no pivo <= 1,2e-6 m, cabo apoiado no solo, sem aborto) |
| Vertical | **FAIL** nas duas geometrias; a junta fica integra (~1e-7 m) ate o aborto do DART |
| Horizontal | **NAO EXECUTADO** (vertical falhou) |
| Controle com forca, mesma geometria | **tambem aborta**, no mesmo evento |
| **Status** | **FAIL** |

A arquitetura de conexao da `dev` foi reproduzida com o X500 e **se sustenta estaticamente**,
mas **nao voa** nesta stack. A queda nao e exclusiva da junta fisica: a constraint de forca cai
no mesmo instante, o do ultimo trecho do cabo com colisoes saindo do solo durante a subida. A
diferenca e que a junta rigida transmite essa carga inteira e vira o drone antes do aborto,
enquanto a forca limitada a 3 N o mantem nivelado ate o fim.

Proximos passos recomendados (nao executados nesta rodada):

1. **Voo sem cabo no solo**: X500 decolando logo acima da guia, com o cabo pendurado reto e sem
   contato com o solo, como nos voos das rodadas A que passaram, agora com a junta fisica. Separa
   "junta rigida em voo" de "descolamento do solo".
2. **Subida mais lenta** (limite de velocidade vertical do PX4) para reduzir o tranco do
   descolamento, e **passo de fisica menor** que 4 ms nesse caso (o Gazebo ja avisa `Aliasing on
   motor`).
3. **Tornar a ferramenta de missao consciente do simulador**: ela reportou `failed = false` em
   todas as quedas desta rodada.
