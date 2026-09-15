# X500 + tether por `BallJoint` na `shared`: wrapper composto no launch

Rodada de 2026-09-14. Integracao validada estaticamente no branch `dev` (`33cd20b`) e portada para
a `shared` (checkpoint `38ecb60`) **sem modificar o PX4**: `px4/PX4-Autopilot` (fora dos branches,
HEAD `1555f2bd22`) teve 0 mudancas antes e depois de cada campanha.

> **Nomes (2026-09-15).** Arquivos e saidas foram renomeados para ingles: `tools/*_cabo_ball*` →
> `tools/*_tether_ball*`, modelo wrapper `x500_cabo` → `x500_tether_ball`, diretorio do cabo
> `tether_package/models/cabo` → `tether_package/models/tether_cable` (conteudo de `model.sdf` e
> `model.config` identico ao `cabo` da `dev`; so o nome do diretorio mudou), resultados em
> `results/x500_tether_ball_shared/<rotulo>/{structure,static,vertical,horizontal}`. As corridas de
> 2026-09-14 descritas abaixo foram feitas com os nomes antigos; os valores nao mudam.

## 1. Diagnostico dev x shared (corrigido)

- O `README.md` da `dev` documenta **PX4 SITL + Gazebo + X500 + cabo + `BallJoint`**, mas a
  integracao nao esta no codigo versionado: o fluxo e **editar o `x500/model.sdf` dentro do PX4**
  (fora do Git) e rodar `make px4_sitl gz_x500`. O diagnostico anterior ("a `dev` nao tem PX4 +
  X500 + tether") valia para o codigo, nao para o procedimento.
- A `shared` tinha duas conexoes: forca (`TetherForceConstraint`, baseline das rodadas A) e junta
  `ball` de mundo com o cabo ancorado na estacao (X1, estatico PASS, voo FAIL).
- Na `dev`, a mesma conexao foi reproduzida sem tocar no PX4 de dois jeitos (overlay do SDF e
  wrapper no launch); os dois deram o mesmo estatico e o wrapper foi o recomendado. Esta rodada
  porta o wrapper.

## 2. Arquitetura

O gz-sim 7.9 nao cria uma junta `ball` entre modelos ja spawnados: o `UserCommands` so tem
`/create`, `/create_multiple`, `/remove`, `/set_pose`, `/set_pose_vector`, e o `DetachableJoint` so
cria juntas `fixed`. A conexao e composta **antes do spawn**, num modelo wrapper que o proprio PX4
spawna com `PX4_GZ_MODEL=x500_tether_ball`:

```xml
<model name="x500_tether_ball">
  <include merge="true">
    <uri>model://x500</uri>                      <!-- X500 original do PX4, sem copia -->
  </include>
  <include>
    <name>cabo_anexado</name>
    <uri>model://tether_cable</uri>
    <pose relative_to="base_link">0 0 0.2 0 0 0</pose>
  </include>
  <joint name="drone_cabo_joint" type="ball">
    <parent>base_link</parent>
    <child>cabo_anexado::raiz_cabo</child>
  </joint>
</model>
```

- `merge="true"`: `base_link`, IMU, barometro e plugins de motor ficam no topo do wrapper, entao o
  PX4 le os mesmos topicos (`/world/default/model/x500_tether_ball_0/link/base_link/sensor/imu_sensor/imu`).
- Pose **relativa ao `base_link`**: o `x500` mesclado traz `<pose>0 0 .24</pose>`; relativa ao
  wrapper, a raiz fica 4 cm abaixo do `base_link` em vez de 0,2 m acima.
- `GZ_SIM_RESOURCE_PATH` **nao e herdado**: o lancador monta `wrapper:tether_package/models`. Com
  um overlay de `x500` ja modificado no ambiente, o include pegaria um X500 que ja tem a junta
  (`joint with name[drone_cabo_joint] already exists`). O PX4 acrescenta os proprios modelos
  depois (`gz_env.sh`: `$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS`), entao
  `model://x500` e o original.
- O wrapper e gerado em `build/x500_tether_ball/models/` (ignorado pelo Git) a cada execucao e
  validado com `gz sdf -k` antes de iniciar o PX4. Nao ha overlay permanente.

### Por que nao editar o PX4

O PX4 fica fora do repositorio: uma edicao no `x500/model.sdf` nao e versionada, se perde ou
conflita a cada atualizacao do PX4 e prende o projeto a um arquivo de terceiros. O wrapper
acompanha o `x500` do PX4 automaticamente e vive no repositorio.

## 3. Arquivos

| Arquivo | Papel |
| --- | --- |
| `tools/launch_x500_tether_ball.py` | gera e valida o wrapper, monta o ambiente e executa `make px4_sitl gz_x500` |
| `src/pacote_do_drone/tether_package/models/tether_cable/{model.sdf,model.config}` | modelo `cabo`, copiado byte a byte de `origin/dev` |
| `src/pacote_do_drone/tether_package/build_tether.py` + `parameters/tether_parameters.json` | gerador e parametros do cabo, copiados de `origin/dev` |
| `tools/record_x500_tether_ball.py` | geometria em runtime por `dynamic_pose/info` (raiz, distancia raiz↔`base_link`, angulos do 1º segmento, `z_min`, RTF) |
| `tools/analyze_x500_tether_ball_flight.py` | metricas de voo (dx realizado, erro de retorno, RMS, angulos por fase) e veredito |
| `tools/run_x500_tether_ball_campaign.sh` | estrutura → estatico (+ captura) → vertical → horizontal (so se o vertical passar) |
| `test/test_record_x500_tether_ball.py` | testes da composicao de poses e dos angulos |

O modelo do cabo fica em `tether_package/models/` e nao em `src/pacote_do_drone/models/`, que ja
tem um arquivo `cabo.sdf` de outra geracao; `model://tether_cable` so resolve diretorios, mas manter os
dois separados evita confusao. A integracao por plugin de forca nao foi alterada.

### Modelo do cabo

| Item | Valor |
| --- | --- |
| Links | `raiz_cabo` (0,02 kg, sem colisao) → `segment_1..70` (0,05 m, 0,002 kg) → `ponta_cabo` |
| Colisao | cilindro de raio **1,5 mm** e 30 mm de comprimento por segmento |
| Juntas internas | 71 `universal`, limites ±30°; `<dynamics>` fora de `<axis>` (71 avisos, amortecimento e atrito ignorados) |
| Geometria inicial | reta subindo a 26,6° da raiz ate a ponta; cai e assenta no solo em ~5 s |
| Extremidade | **livre** (sem ancora no solo) |
| Sensores | 2 `force_torque` nas juntas das pontas, inativos (o mundo `default` nao carrega `ForceTorque`) |

Nao ha carretel nem ancora nesta integracao. Prender a ponta ao solo com `parent=base_link` fecharia
um laco cinematico (drone → cabo → mundo), que a DART desta stack nao suporta; a ancoragem continua
sendo o caminho da X1 (junta de mundo, cabo ancorado na estacao).

## 4. Comandos

```bash
cd /home/lima/codes/ic/drone-cabo
unset GZ_SIM_RESOURCE_PATH PX4_GZ_MODEL_NAME PX4_GZ_MODEL_POSE

# build do PX4 (uma vez; nao modifica arquivos versionados do PX4)
( cd px4/PX4-Autopilot && make px4_sitl )

# so gerar e validar o wrapper (imprime os exports usados)
./tools/launch_x500_tether_ball.py --only-generate --print-env

# simulacao com GUI do PX4 (terminal 1)
./tools/launch_x500_tether_ball.py
# sem GUI
./tools/launch_x500_tether_ball.py --headless
# outra pose da raiz (relativa ao base_link)
./tools/launch_x500_tether_ball.py --pose "0 0 -0.12 0 0 0"
```

Com a simulacao de pe, esperar `Ready for takeoff!` e, em outro terminal:

```bash
cd /home/lima/codes/ic/drone-cabo
gz model --list                                   # ground_plane, x500_tether_ball_0
gz model -m x500_tether_ball_0 | grep -A4 drone_cabo_joint   # Type: ball, Parent Link: base_link

# estatico: 30 s de geometria
./tools/record_x500_tether_ball.py --duration 30 --output-dir results/x500_tether_ball_shared/manual/static

# vertical (a missao conta tempo de PAREDE: com RTF ~0,3-0,5 multiplique as fases por ~3)
OUT=results/x500_tether_ball_shared/manual/vertical
./tools/record_x500_tether_ball.py --duration 225 --output-dir $OUT &
./tools/px4_offboard_horizontal_mission.py --output-dir $OUT --dx 0.0 --altitude 2.0 --rate 20 \
  --prestream 6 --offboard-settle 3 --takeoff-hover 36 --move-hold 30 --return-hold 30 --land-stream 24
wait %1
./tools/analyze_x500_tether_ball_flight.py --run-dir $OUT --px4-log <log do PX4>

# horizontal: mesmo procedimento com --dx 0.5 (x local do PX4 = norte = +y do Gazebo)
```

Campanha completa (estrutura, estatico com captura, vertical, horizontal condicionado):

```bash
TIME_SCALE=3 tools/run_x500_tether_ball_campaign.sh ts3     # results/x500_tether_ball_shared/ts3/
```

Visualizacao: a GUI que o PX4 abre (ogre2) funciona em um terminal normal. Se ela nao aparecer
(visto a partir de processos em segundo plano: `libEGL warning: egl: failed to create dri2
screen`), rode o PX4 com `--headless` e abra a GUI a parte:

```bash
gz sim -g --render-engine-gui ogre
gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'data: "x500_tether_ball_0"'
```

Cuidados de medicao e execucao:

- Use `/world/default/dynamic_pose/info`; `pose/info` nao atualiza os links do modelo aninhado.
- `gz sdf -p` omite os includes mesclado e aninhado sem erro; confira em runtime.
- A missao nao percebe se o simulador morreu: confira `sim_time_covered_s`, `Assertion|Aborted`
  e `failsafe` no log do PX4 (a analise faz isso).
- Nao use `pkill -f` com padrao que aparece no proprio comando; mate por nome de executavel
  (`ps -eo pid=,comm=`: `px4`, `gz`, `ruby`), como a campanha faz.

## 5. Validacao estrutural em runtime (campanha `baseline`)

| Verificacao | Resultado |
| --- | --- |
| `gz sdf -k` do wrapper | `Valid.` (so os 71 avisos de `<dynamics>` do cabo) |
| PX4 | `Ready for takeoff`, 0 `Preflight Fail` |
| Modelos | `ground_plane`, `x500_tether_ball_0` — modelo composto unico, cabo aninhado, sem duplicata |
| `base_link` | pai `x500_tether_ball_0`, 2,0 kg |
| Junta | `drone_cabo_joint`, `Type: ball`, `Parent Link: base_link` |
| Topico de IMU do PX4 | presente |
| Raiz ↔ `base_link` | 0,2000008 m |
| Crash DART no estatico | nao |
| PX4 local | 0 mudancas antes e depois |

Captura em `results/x500_tether_ball_shared/baseline/static/gazebo.png`: arvore com um unico
`x500_tether_ball_0`; o cabo sai do topo do corpo e se estende ~3 m pelo solo a frente do drone (+x),
com a ponta ondulada pelos limites de ±30 graus das juntas. Coerente com as poses.

## 6. Estatico (30 s de parede, drone pousado, PX4 de pe)

| Grandeza | Resultado |
| --- | --- |
| Pose inicial da raiz | (0,00; 0,00; 0,427) m — 0,2 m acima do `base_link` (z = 0,227) |
| Raiz ↔ `base_link` | 0,2000008 m (min = max ate 10⁻⁹) |
| 1º segmento, elevacao (mediana / p05 / p95) | -66,9 / -69,6 / -64,1 graus |
| 1º segmento, azimute (mediana / p05 / p95) | -1,3 / -2,9 / 0,3 graus |
| `z_min` do cabo | 1,35–1,46 mm (raio de colisao; sem penetracao) |
| Roll / pitch do drone | 0 / 0 graus |
| RTF (media / min) | **0,30 / 0,19** |
| Tempo simulado coberto | 8,9 s em 30 s de parede (coerente com o RTF: servidor vivo) |
| Aborto / failsafe | nao / nao |

Nota: o azimute e medido na direcao que **sai** do drone ao longo do 1º segmento (+x local do
segmento); com o cabo gerado ao longo de +x do modelo, azimute ≈ 0 e elevacao ≈ -67 graus
significam que o cabo desce da raiz para a frente do drone ate o solo. **Estatico: PASS.**

**RTF.** O servidor do Gazebo fica em 100% de um nucleo (fisica monothread) com 72 elos com
colisao; em voo, com o cabo parcialmente suspenso, o RTF sobe para ~0,5. O PX4 esta em lockstep,
entao o controle nao sofre, mas a missao OFFBOARD conta tempo de parede (secao 7).

## 7. Voos da campanha `baseline` (fases da missao em tempo de parede, `TIME_SCALE=1`)

### 7.1 Vertical — FAIL (penetracao do cabo no solo)

| Grandeza | Resultado |
| --- | --- |
| Sequencia PX4 | `Takeoff detected` → `Landing detected` → `Disarmed by landing`, sem failsafe, sem aborto |
| Tempo simulado coberto | 37,1 s (RTF medio 0,50) |
| Altura maxima do `base_link` | 2,035 m |
| RMS XY / RMS Z (missao) / RMS Z em hover | 0,074 m / 1,87 m* / 1,005 m* |
| Roll / pitch max | 1,1 / 2,6 graus |
| Raiz ↔ `base_link` | 0,2000000–0,2000008 m durante todo o voo |
| Elevacao do 1º segmento (mediana por fase) | solo -68; subida -73; hover -76; pouso -84 graus |
| **`z_min` do cabo** | **-0,070 m em voo, -0,111 m no pouso** |

\* Com RTF 0,5 o `climb_hover` de 12 s de parede virou ~6 s simulados e o drone ainda estava no
solo no fim da fase (z local -0,16 m); a subida caiu dentro de `translate_out`. O RMS Z "de hover"
dessa corrida mede o transitorio de subida e nao deve ser usado.

O criterio original (sem aborto, sem failsafe, cobertura, atitude, junta integra) deu PASS. A
analise das poses mostrou que, a partir de t = 33 s, com o drone subindo e arrastando o trecho
apoiado, os segmentos **atravessam o plano do solo** (`z_min` de -5 mm a -70 mm) e terminam
enterrados 11 cm no pouso. A colisao de 1,5 mm de raio nao segura o cabo arrastado. O criterio
`cabo_sem_penetrar_solo` (`z_min >= -5 mm`) foi adicionado a analise e o vertical passa a **FAIL**.

### 7.2 Horizontal — FAIL (executado antes da correcao do criterio)

Na mesma sessao, logo apos o vertical, com o cabo ainda enterrado (`z_min` = -0,066 m):

| t simulado | Evento |
| --- | --- |
| 67,4 s | decolagem; cabo ainda sob o solo |
| 68,5 s | drone a 1,0 m; o cabo se solta do solo |
| 68,6–69,1 s | a ponta percorre ~3 m em 0,5 s; o 1º segmento passa para **acima** do drone (elevacao +28 graus); roll -20 graus |
| ~69,2 s | PX4: `invalid setpoints` → `Failsafe: blind land` → `blind descent`; Gazebo: `assertion "aabbBound >= dMinIntExact && aabbBound < dMaxIntExact" failed in collide() [collision_space.cpp:460]` em `dart::constraint::ConstraintSolver::solve()` |

dx comandado 0,5 m; dx realizado 0,17 m (antes do colapso); erro de retorno nao aplicavel. A
junta ficou integra (0,2000003 m) ate a ultima amostra: a falha e do cabo chicoteando, nao da
`BallJoint`. Mesmo gatilho da X1: **cabo com colisoes saindo do solo**.

## 8. Campanha `ts3` (fases da missao x3, sessao nova)

`TIME_SCALE=3 tools/run_x500_tether_ball_campaign.sh ts3`, ja com o criterio de penetracao.

### 8.1 Estrutura e estatico — PASS

Identicos a `baseline`: `ground_plane` + `x500_tether_ball_0`, junta `ball` `base_link` →
`cabo_anexado::raiz_cabo`, raiz em (0,00; 0,00; 0,427), raiz ↔ `base_link` 0,2000008 m, elevacao
do 1º segmento -67,0 (p05 -69,4 / p95 -64,4) graus, azimute -1,4 grau, `z_min` 1,35 mm, RTF
0,35, sem aborto. Captura em `results/x500_tether_ball_shared/ts3/static/gazebo.png`.

### 8.2 Vertical — FAIL (so pela penetracao no pouso)

| Grandeza | Resultado |
| --- | --- |
| Sequencia PX4 | `Takeoff detected` → `Landing detected` → `Disarmed by landing` |
| Failsafe / aborto DART | 0 / 0 |
| Tempo simulado coberto | 117,6 s (RTF medio 0,54) |
| Altitude | hover a ~2,0 m (`base_link` max 2,16 m) |
| RMS XY (rastreio) | 0,047 m |
| RMS Z em hover (`rms_z_hold_m`, sem a subida) | **0,034 m** |
| RMS Z da missao (`rms_z_hover_m`, inclui a subida) | 1,01 m |
| Roll / pitch max | 1,9 / 1,9 graus |
| Raiz ↔ `base_link` | 0,2000 m em todas as 3290 amostras |
| Erro de retorno (dx = 0) | 0,004 m |
| 1º segmento, elevacao (mediana por fase) | solo -68; subida/hover -77; hover -80 a -81; pouso -75 graus |
| 1º segmento, azimute (mediana por fase) | -2 a -5 graus em voo; -112 graus pousado (cabo quase vertical: mal condicionado) |
| `z_min` do cabo | solo +1,4 mm; subida **-5,3 mm** por ~2 s (t = 35,4 s, trecho arrastado), volta a +1,4 mm em voo; **pouso -173 mm** |

Com as fases em escala, o voo e limpo: sobe, estabiliza, pousa e desarma sem failsafe, com o drone
nivelado e a junta integra. O que reprova e o cabo: no pouso (t ≈ 122 s) os segmentos que pendem
sob o drone chegam ao solo e atravessam o plano, ficando ate 17 cm enterrados. Hipotese
compativel com os numeros, **nao verificada**: tunelamento — descendo a ~0,7 m/s, um segmento
anda ~2,8 mm por passo de 4 ms, da ordem do diametro de colisao de 3 mm. Na subida o mesmo efeito
aparece fraco (-5,3 mm) enquanto o trecho apoiado e arrastado.

### 8.3 Horizontal — NAO EXECUTADO

Bloqueado pelo vertical, como manda o protocolo. A unica corrida horizontal e a da `baseline`
(secao 7.2), que partiu com o cabo enterrado pelo pouso anterior e mostra a consequencia: ao
arrancar o cabo do solo ele chicoteia e o DART aborta.

### 8.4 Visual

Nas capturas do estatico (`baseline` e `ts3`) o cabo sai do topo do X500 e se estende pelo solo
em +x, sem segundo modelo na arvore. A orientacao natural do cabo confere com as poses (elevacao
~-67 graus na raiz, cabo apoiado a partir de ~0,2 m). Nao ha captura em voo: a GUI em processo
separado reduz ainda mais o RTF e nao foi aberta durante as missoes.

## 9. Comparacao: plugin de forca x `BallJoint` via wrapper

| Criterio | Plugin de forca (`TetherForceConstraint`, X2) | `BallJoint` via wrapper (esta rodada) |
| --- | --- | --- |
| Estabilidade | voo vertical valido: 69,7 de 70 s, sem aborto nem failsafe, RTF 0,996 | estatico estavel; vertical com fases em escala sem aborto nem failsafe (RMS Z 0,034 m), mas o cabo atravessa o solo no pouso; horizontal com o cabo enterrado aborta (secoes 7–8) |
| Fidelidade mecanica | complacente (`e` ≈ 0,15 m), `Fmax` limita cargas, sem momento | articulacao rigida sem folga, sem momento, carga ilimitada |
| Disponibilidade de angulos | topicos `/cabo/conexao/{tangent_body,angles}` publicados pelo plugin | calculados das poses (`record_x500_tether_ball.py`); a junta nao expoe angulos |
| Forca na conexao | direta (`/cabo/conexao/force_body`) | nenhuma; exigiria `TransmittedWrench`, que ja derrubou o DART nesta stack |
| Instrumentacao | nenhuma extra | gravador de poses fora do Gazebo (Python, `dynamic_pose/info`) |
| Impacto no voo | drone nivelado (roll max 2,9 graus) | drone carrega o cabo inteiro sem limite; roll -20 graus no chicote |
| Integracao com o PX4 | mundo proprio + plugin compilado | PX4 spawna o wrapper normalmente (`PX4_GZ_MODEL`), sem mundo proprio nem plugin |
| Manutencao | plugin C++ (4 bibliotecas) acoplado a gz-sim 7 | um script Python e um modelo SDF; acompanha o `x500` do PX4 |
| Modelo do cabo | cadeia `ball` da `shared` (N=5–20, 0,125–0,5 m por elo) | cadeia `universal` da `dev` (70 × 0,05 m, colisao 1,5 mm) |
| RTF | ~1,0 | 0,30 (solo) a 0,50 (voo) |

Os dois nao usam o mesmo cabo, entao a comparacao de estabilidade em voo mistura conexao e
discretizacao. O que e atribuivel a conexao: a junta nunca abriu (0,2000 m em todas as amostras,
inclusive no colapso) e o plugin continua sendo a unica fonte de forca.

## 10. Limitacoes

- **Cabo atravessa o solo quando arrastado**: colisao de 1,5 mm em cilindros de 30 mm com passo de
  50 mm; nao ha contato entre segmentos consecutivos e o solo segura mal o cabo em movimento.
- **RTF baixo** (0,3–0,5) com 72 elos com colisao; missoes em tempo de parede precisam de
  `TIME_SCALE`.
- **Sem forca na conexao** e **sem colisao cabo–drone** (mesmo modelo); a raiz fica 0,2 m acima
  do `base_link` e o cabo pode atravessar o frame.
- **Cabo semirrigido**: juntas `universal` com ±30 graus e amortecimento ignorado.
- **Extremidade livre**: sem ancora nem carretel; ancorar com `parent=base_link` fecharia laco.
- **Azimute mal condicionado** com o cabo quase vertical (elevacao < -80 graus), com qualquer metodo.
- **Logs por sessao**: o `px4.log` cobre a sessao inteira; a analise do vertical feita depois do
  horizontal conta tambem o aborto do horizontal. O veredito vale na hora em que a campanha o grava.

## 11. Recomendacao

**Integracao:** o wrapper no launch e a forma certa de levar a `BallJoint` da `dev` para a
`shared`. Nao toca no PX4, e versionavel, gera um unico modelo com a junta onde deveria estar e
mantem os sensores do PX4. Estrutura e estatico passaram duas vezes, e a junta nunca abriu, nem
no colapso da horizontal.

**Voo:** a integracao **ainda nao esta pronta para campanhas de voo**. O limitante nao e a
`BallJoint` nem o PX4, e o modelo de cabo da `dev`: a colisao de 1,5 mm deixa o cabo atravessar o
solo no pouso (-173 mm) e quando arrastado, e um cabo enterrado derruba o DART na decolagem
seguinte.

**Baseline de voo:** continua sendo a conexao por plugin de forca, a unica com voo valido e medida
direta de forca. A integracao por wrapper fica disponivel, desligada por padrao: so e usada quando
`tools/launch_x500_tether_ball.py` e chamado, e nada da integracao por forca foi alterado.

Antes de voar o horizontal com a `BallJoint` sera preciso resolver o contato cabo–solo (raio e
comprimento da colisao, parametros de contato ou passo de fisica menor) e repetir o vertical. A
decisao fica para a proxima instrucao.

## Apendice — rodada na `dev` (overlay x wrapper)

Resultados da `dev` (`33cd20b`), estaticos, sem voo: com o SDF do X500 modificado por overlay e
com o wrapper, raiz ↔ pivo no `base_link` constante em 0,200001 m, `z_min` 1,4 mm, elevacao do 1º
segmento -64 a -68 graus, azimute ±3 graus, um unico modelo de veiculo, PX4 `Ready for takeoff`,
sem aborto. O overlay reproduz o README da `dev` sem tocar no PX4, mas exige manter uma copia do
`x500`; o wrapper produz a mesma conexao fisica e foi o escolhido.
