# Plugin de forca + medicao angular do tether no frame do drone

Rodada de 2026-09-14 na `shared` (checkpoint `38ecb60`, sem commit nesta rodada). A conexao
drone–tether continua sendo a **constraint de forca** (`TetherForceConstraint`); esta rodada
acrescenta a direcao local do cabo por tangente suavizada, sua transformacao `world → body` com a
atitude real do X500, e valida a colisao tether–solo e os voos. O PX4 local nao foi modificado.

## 0. Baseline oficial (consolidada em 2026-09-15)

| Item | Valor |
| --- | --- |
| Conexao drone–tether | `TetherForceConstraint` |
| Angulos | tangente local `t_hat_world` transformada `world → body` com a atitude atual |
| Janela da tangente | 0,15 m, 4 pontos |
| Colisao cabo–solo | habilitada (cilindro de raio 2,25 mm por elo) |
| Collision masks | X500 = 1, tether = 2, solo = 65535 |
| `max_step_size` | **0,001 s** |
| Spawn | `PX4_GZ_MODEL=x500_tether_attach_mask`, `PX4_GZ_MODEL_POSE=0,0,0.24,0,0,0` |
| Missoes | altitude relativa (`--relative-altitude`) |
| Cabo e constraint | N = 5, L = 2,5 m, K = 5 N/m, C = 0,5 N·s/m, Fmax = 3 N |

**Requisito de `dt = 1 ms`.** A colisao cabo–solo so e robusta com passo de 1 ms: a 4 ms o cabo
penetra 2,9 mm no pouso e a 2 ms a penetracao oscila entre 1,5 e 2,0 mm entre sessoes, no limite do
criterio de 2 mm (secoes 5 e 9). O mundo `default` do PX4 usa 4 ms; a baseline serve uma copia com
1 ms (`tools/generate_px4_world_step.py`) antes de iniciar o PX4, que se liga a ela. O arquivo do PX4
nao e editado.

## 1. Arquitetura confirmada no codigo

### 1.1 Links

| Papel | Nome real | Onde |
| --- | --- | --- |
| Estacao (fixa ao mundo) | `tether_anchor_chain::ground_station_base` | junta `ground_station_world_fixed` (world → base) |
| Tambor passivo, desacoplado | `tether_anchor_chain::reel_link` | junta `reel_joint` (revolute, eixo Y); sem atuador na baseline |
| Guia de saida (fixa) | `tether_anchor_chain::tether_exit_point` | junta `tether_exit_fixed`, z = 0,19 m |
| Primeiro elo (lado do solo) | `tether_anchor_chain::tether_link_1` | junta `tether_joint_1` (ball) com a guia |
| Elos intermediarios | `tether_link_2`, `tether_link_3`, `tether_link_4` | juntas `tether_joint_2..4` (ball) |
| **Elo mais proximo do drone** | `tether_anchor_chain::tether_link_5` | junta `tether_joint_5` (ball); a ponta livre recebe a forca |
| **Attachment no UAV** | `x500_tether_attach_0::tether_attach_link` | junta `tether_attach_fixed` (fixed) com `base_link`, pose `0 0 -0.12` |
| Corpo do UAV | `x500_tether_attach_0::base_link` | IMU/barometro lidos pelo PX4; mesma orientacao do `tether_attach_link` |

Baseline: N = 5, L = 2,5 m, elos de 0,5 m, `rho` = 0,06 kg/m (0,03 kg por elo), raio visual
3 mm, `folded_ground`. Cada elo nasce na sua junta e se estende no **+x local** ate a proxima.
`x500_tether_attach` e o `x500` do PX4 byte a byte mais o `tether_attach_link` (0,005 kg, sem
colisao); o PX4 continua lendo os mesmos sensores. Nas campanhas com filtro de colisao (secao 5.2)
o drone e uma copia gerada em `results/`, `x500_tether_attach_mask` (instancia
`x500_tether_attach_mask_0`), identica exceto pelo `<collide_bitmask>` 1 nas 9 colisoes; o plugin e
os gravadores recebem esse nome (`--drone-model`). A variante versionada nao muda.

### 1.2 Forca e reacao

`TetherForceConstraint` (em `tether_anchor_chain`, `PreUpdate`, a cada passo):

```text
p_drone  = pose(tether_attach_link) ⊕ drone_offset        drone_offset  = (0, 0, 0)
p_tether = pose(tether_link_5)      ⊕ tether_offset       tether_offset = (0,5, 0, 0)  (ponta do elo)
e        = p_tether - p_drone
F        = -K e - C (v_tether - v_drone),   |F| <= Fmax    K = 5 N/m, C = 0,5 N.s/m, Fmax = 3 N
tether_link_5.AddWorldForce(+F, tether_offset)      forca sobre o cabo, na ponta
tether_attach_link.AddWorldForce(-F, drone_offset)  reacao sobre o drone, no attach
```

`AddWorldForce(F, p)` aplica no ponto `p` do link e gera sozinho o momento `r × F` em torno do CoM.
Nao ha junta entre o cabo e o drone: a conexao e complacente (`|e| = |F|/K`).

### 1.3 Topicos (gz.msgs.Vector3d, frames do Gazebo)

| Topico | Conteudo | Frame |
| --- | --- | --- |
| `/cabo/conexao/force` | `F` sobre o cabo (o drone recebe `-F`) | mundo |
| `/cabo/conexao/force_body` | `-F`, forca sobre o drone | drone |
| `/cabo/conexao/error` | `e` | mundo |
| `/cabo/conexao/stats` | (`|e|`, `|F|`, saturado) | — |
| `/cabo/conexao/tangent_body`, `/cabo/conexao/angles` | medida antiga pelo ultimo elo: tangente no corpo; (azimute, elevacao, forca x tangente) | drone |
| **`/cabo/conexao/t_hat_world`** | tangente suavizada `t_hat_world` | mundo |
| **`/cabo/conexao/t_hat_body`** | `t_hat_body = R_BW t_hat_world` | drone |
| **`/cabo/conexao/angles_body`** | (`azimuth_body`, `elevation_body`, angulo entre `-F` e `t_hat`) [graus] | drone |
| **`/cabo/conexao/angles_world`** | (`azimuth_world`, `elevation_world`, janela usada [m]) | mundo |
| **`/cabo/conexao/drone_rpy`** | (roll, pitch, yaw) do `tether_attach_link` usados na transformacao [graus] | — |
| **`/cabo/conexao/t_hat_world_last_link`** | `-R_5 (1,0,0)`: so o ultimo elo, para comparacao | mundo |
| `/cabo/estacao/{tensao,exit_force,exit_tangent,exit_pose,reel_state}` | lado da estacao (inalterados) | mundo |
| `/world/default/pose/info` | poses de todos os links (verificacao independente em Python) | — |

Posicao/orientacao do drone para o PX4 vem dos sensores do proprio X500 (nenhuma mudanca).

## 2. Frames e convencoes

```text
W  mundo do Gazebo, ENU: x leste, y norte, z cima
B  tether_attach_link (= orientacao do base_link): x frente, y esquerda, z cima (FLU)
q_WB  orientacao do link do drone no mundo (Pose3d::Rot()), leva vetores de B para W
R_BW = R_WB^T  (mundo → corpo);   v_B = R_BW v_W = q_WB^-1 v_W
roll, pitch, yaw: R_WB = Rz(yaw) Ry(pitch) Rx(roll)  (convencao do gz-math)
```

Atitude usada: a pose **atual** do `tether_attach_link` a cada passo, sem supor roll/pitch/yaw
nulos. Sinais no gz: pitch positivo abaixa o nariz; roll positivo sobe o lado esquerdo.

Para o PX4 (NED/FRD): `t_FRD = (tx, -ty, -tz)`, logo `azimute_FRD = -azimute_FLU` (positivo para a
direita) e o angulo "para baixo" do PX4 e `-elevacao`.

## 3. Tangente local

Poligonal do cabo, da ponta para a estacao:

```text
P0 = ponta de tether_link_5 (p_tether)
P1 = origem de tether_link_5, P2 = origem de tether_link_4, ..., P5 = origem de tether_link_1
```

Regressao linear de `P(s)` contra o comprimento de arco `s`, com `n` pontos igualmente espacados em
`[0, w]` a partir da ponta (`w` limitado ao comprimento do cabo):

```text
s_i = i w / (n - 1),   p_i = P(s_i)        (interpolacao linear na poligonal)
d   = Σ (s_i - s̄)(p_i - p̄)
t_hat_world = d / |d|                       aponta do drone para longe dele, ao longo do cabo
t_hat_body  = R_BW t_hat_world
azimuth     = atan2(ty, tx)                 0 = frente, +90 = esquerda, ±180 = tras
elevation   = atan2(tz, sqrt(tx² + ty²))    0 = horizontal, -90 = cabo reto abaixo
```

Parametros (SDF do plugin, gerador `--tangent-window` / `--tangent-samples`): `w = 0,15 m`,
`n = 4`. Com elos de 0,5 m a janela de 0,15 m fica **inteira dentro do ultimo elo**, e um elo e
rigido: a tangente suavizada coincide com a do ultimo elo (secao T2). A regressao passa a mediar
elos quando `w` cruza juntas — com esta discretizacao, `w ≥ 0,5 m`. Implementacao C++ em
`src/pacote_do_drone/gz_plugins/TetherGeometry.hh` (testada em `test/cpp/test_tether_geometry.cc`)
e implementacao Python independente em `tools/record_tether_connection.py`
(`test/test_tether_tangent_python.py`).

Limitacao de gravacao: um `Vector3d` exatamente zero e serializado vazio pelo `gz topic -e`, entao
`drone_rpy = (0,0,0)` (corpo perfeitamente nivelado, so em bancada) nao aparece no CSV; a analise
usa o roll/pitch/yaw das poses nesse caso.

## 4. T0/T1 — bancada de compensacao de atitude (sem PX4)

`tools/run_tether_attitude_bench.sh t1` + `tools/analyze_tether_attitude_bench.py`. Um corpo
**estatico** `attitude_body` com o link `tether_attach_link` na origem (1,5; 0; 1,2) m faz o papel do
drone; o cabo da baseline (N = 5, K = 5, C = 0,5, Fmax = 3) nasce esticado da guia ate o corpo e
assenta pendurado. Uma sessao do Gazebo por atitude, 15 s de assentamento e 10 s de gravacao. Como
o ponto de conexao e a origem do corpo, **girar o corpo nao muda a geometria do cabo**: o
`t_hat_world` tem de ficar igual e o `t_hat_body` tem de mudar exatamente como `R_BW`.

Para o corpo estatico o plugin passou a compor a pose pela arvore (`gz::sim::worldPose`) e usar
velocidade zero quando a fisica nao publica `WorldPose`/velocidade; com o X500 dinamico os
componentes existem e nada muda.

| Caso (roll, pitch, yaw) | az/el world | az/el body medido | az/el body previsto | invariancia world | residuo `R_BW` | erro vetor | plugin x Python | ultimo elo x suavizada |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| level (0, 0, 0) — **T0** | 179,93 / -75,05 | 179,94 / -75,05 | 179,93 / -75,05 | 0 | 0,0015° | 0,0015° | 0,003° | 0,002° |
| roll +15 | 179,93 / -75,04 | -135,94 / -68,95 | -135,93 / -68,95 | 0,001° | 0,0005° | 0,0013° | 0,002° | 0,001° |
| roll -15 | 179,94 / -75,05 | 135,86 / -68,93 | 135,87 / -68,93 | 0,005° | 0,0000° | 0,0045° | 0,002° | 0,000° |
| pitch +15 | 179,94 / -75,05 | 16,31 / -89,95 | 20,33 / -89,95 | 0,007° | 0,0005° | 0,0069° | 0,001° | 0,001° |
| pitch -15 | 179,94 / -75,05 | 179,97 / -60,05 | 179,97 / -60,05 | 0,004° | 0,0000° | 0,0039° | 0,000° | 0,000° |
| yaw 45 | 179,94 / -75,05 | 134,94 / -75,05 | 134,93 / -75,05 | 0,001° | 0,0005° | 0,0012° | 0,003° | 0,000° |
| combined (10, -10, 30) | 179,94 / -75,05 | -175,40 / -67,11 | -175,40 / -67,11 | 0,004° | 0,0005° | 0,0033° | 0,002° | 0,000° |

- **T0 PASS.** Nivelado: `t_hat_world` = `t_hat_body` (0,0015°), elevacao -75,05°, azimute 179,9°
  (o cabo sai do corpo para tras, em direcao a estacao, e desce).
- **T1 PASS.** `t_hat_world` fica a menos de 0,007° do caso nivelado em todas as atitudes: a medida
  de direcao fisica nao se confunde com a atitude. `t_hat_body` coincide com `R_BW t_hat_world` a
  0,0015° e com a previsao feita so a partir do caso nivelado e da atitude a 0,007°. Exemplos: yaw
  45 desloca o azimute em exatamente -45°; pitch -15 (nariz para cima) leva a elevacao de -75 a -60.
- **pitch +15:** a atitude leva o cabo a -89,95° de elevacao no corpo, a 0,05° da vertical, onde o
  azimute nao e definido. O azimute medido difere 4° do previsto com erro vetorial de 0,007°. Por
  isso a analise verifica o vetor e so compara azimute com |elevacao| < 85°; o criterio inicial
  (azimute sempre) reprovava este caso por uma singularidade da representacao, nao da medida.
- Espalhamento temporal de `t_hat_world` (p95): ~1,0° — o cabo pendurado oscila lentamente.
- Verificacao independente: plugin (C++) x calculo Python pelas poses: ≤ 0,003°. Ultimo elo x
  tangente suavizada: ≤ 0,002° (janela de 0,15 m dentro do elo de 0,5 m).
- 0 abortos, 0 NaN, RTF ~0,99.

**Repeticao (2026-09-15).** A bancada foi rodada de novo (`tools/run_tether_attitude_bench.sh t1`,
sessoes novas) e passou outra vez, com numeros um pouco diferentes: invariancia de `t_hat_world`
≤ 0,021°, residuo `R_BW` ≤ 0,002°, erro vetorial da previsao ≤ 0,023°, plugin x Python ≤ 0,004°,
ultimo elo x suavizada ≤ 0,002°. Os dados locais em `results/tether_angles/bench/t1/` sao os dessa
repeticao; a tabela acima e da rodada de 2026-09-14. Os casos passaram a se chamar `level` e
`combined` (antes `nivelado` e `combinado`).

## 5. Colisao tether–solo: parametros confirmados

| Item | Valor na baseline de forca |
| --- | --- |
| Colisao dos elos | **desligada** (`colisoes_links=False`; o cabo atravessava o solo, `z_min` -1,23 m no voo da X2) |
| Com `--link-collisions` | cilindro por elo, raio `0,75 · r` = **2,25 mm**, comprimento = elo (0,5 m), pose no centro do elo |
| `<surface>` dos elos | nenhum (padrao) |
| Contato no dartsim | so `friction`/`bounce`; **`kp`, `kd`, `max_vel`, `min_depth` nao sao lidos** pelo gz-physics-dartsim desta stack (conferido nos simbolos do plugin) |
| Filtro de colisao | `<collide_bitmask>` suportado (`CollisionFilterMaskFeature` / `BitmaskContactFilter`); padrao 65535 |
| X500 | 9 colisoes (placa 0,354 × 0,354 × 0,05 m em `base_link` + 0,007; 2 hastes; 2 esquis; 4 rotores 0,279 × 0,017 × 0,0008 m), `min_depth` 0,001 |
| Solo | `ground_plane` do mundo `default` do PX4, plano, mascara padrao |
| `max_step_size` | 0,004 s (mundo `default` do PX4, 250 Hz) |
| Spawn do X500 | `PX4_GZ_MODEL_POSE=0,0,0.24`: sem ele o PX4 spawna em z = 0, sobrescreve a `<pose>0 0 .24` e o drone assenta afundado (modelo a z = 0,043, attach a -0,077, abaixo do solo) |

### 5.1 C0/C1 sem filtro (campanha `collision/baseline`)

`tools/run_tether_angle_collision_campaign.sh collision baseline`: colisoes nos elos, spawn corrigido,
mascaras padrao.

- **C0 PASS.** Cabo em repouso por 30 s: penetracao maxima 0,28 µm, sem NaN, RTF 0,996, `|F|` 0,77 N,
  drone a z = 0,227 m e attach a 0,107 m.
- **C1 FAIL — nao por tunelamento no solo.** Antes da queda a superficie mais baixa do cabo nunca
  passou de -0,2 mm. Linha do tempo (t simulado): decolagem em 53,5 s; cabo sai inteiro do solo e
  fica pendurado da guia; hover estavel com `|F|` ≈ 1,1 N, sem saturacao, roll/pitch < 2°; durante a
  translacao, em 65,8 s, o drone gira para roll -140° em 0,3 s, `|e|` salta para 2,3 m, a forca satura
  e a DART aborta em `BallJoint.cpp:159` (`updateRelativeTransform`), precedida de failsafe do PX4.
- **Causa: contato cabo–drone.** A complacencia da constraint (`|e|` = 0,15–0,23 m) deixa a ponta do
  cabo ~0,10 m acima do `base_link`: em repouso ela ficava apoiada **sobre a placa do X500**
  (ponta a z = 0,261 m; topo da placa a 0,259 m), e em voo o `tether_link_5` sai da ponta a ~-30° e
  atravessa a placa e o plano dos rotores (+0,06 m). Com colisao nos elos, a placa e as caixas de
  colisao dos rotores — que giram na velocidade do motor — passam a empurrar o cabo. Na baseline
  sem colisao isso nao existia.
- **Achado de ferramenta:** com `--altitude 1.0` absoluto o drone subiu a ~2,2 m, porque a origem do
  EKF nasceu 1,14 m deslocada; o cabo nunca foi arrastado. A missao ganhou `--relative-altitude`.

### 5.2 Filtro de colisao cabo x drone (campanha `collision/mask`)

Uma mudanca: `<collide_bitmask>` 1 em todas as colisoes do X500 (copia `x500_tether_attach_mask`
gerada em `results/` por `tools/generate_x500_tether_attach.py --collide-bitmask 1`; a variante
versionada nao muda) e 2 nas colisoes dos elos (`generate_tether_anchor_chain.py --collide-bitmask 2`).
O solo continua em 65535 e colide com os dois; cabo e drone deixam de se tocar. Voos com
`--relative-altitude`.

- **C0 PASS.** Penetracao 0,30 µm. A ponta do cabo agora repousa **no solo** (z = 2,2 mm, o raio de
  colisao) e nao mais sobre a placa do X500 (0,261 m); `|e|` em repouso cai de 0,154 para 0,106 m.
- **C1 — voo limpo, FAIL so por penetracao no pouso.** Arraste de 1 m a 0,1 m/s, 0,8 m acima do ponto
  de decolagem: sem aborto nem failsafe, dx realizado 0,94 m (rampa de 1,0 m), erro de retorno
  0,05 m, RMS XY 0,11 m, RMS Z em hover 0,098 m, roll/pitch ≤ 2,1°, `|F|` mediana 0,61 N (max 1,87 N),
  sem saturacao, RTF 0,994, maior buraco nos topicos 0,10 s. **Durante todo o arraste a superficie
  mais baixa do cabo ficou ≥ -0,1 mm.** Uma unica excursao passou do limite de 2 mm: t = 98,9–99,7 s,
  no **pouso** (drone descendo de 0,70 a 0,29 m), minimo **-2,9 mm**, recuperada em 0,77 s (sem cabo
  enterrado depois).
- A profundidade bate com o deslocamento por passo: descendo a ~0,7 m/s, um elo anda ~2,8 mm em
  4 ms. Como `kp`/`kd` de contato nao sao lidos pelo dartsim, a variavel testada a seguir e o passo
  de fisica (4 → 2 ms), sozinha; o raio de colisao fica como proxima alavanca.
- C2 nao executado (bloqueado por C1).

## 6. T2 — ultimo elo x tangente suavizada (voo C1 com filtro)

`tools/analyze_tether_tangent_windows.py`, poses de `/world/default/pose/info` (3985 amostras, mesma
gravacao para as tres janelas):

| Metodo | Jitter RMS por amostra | Variacao total | Elevacao no corpo (mediana / desvio) | x ultimo elo (mediana / p95) |
| --- | --- | --- | --- | --- |
| Ultimo elo (`-R_5 x`) | 0,386° | 10,3 °/s | -59,0° / 30,6° | — |
| Suavizada, w = 0,15 m (plugin) | 0,386° | 10,3 °/s | -59,0° / 30,6° | 0,000° / 0,000° |
| Suavizada, w = 0,5 m | 0,386° | 10,3 °/s | -59,0° / 30,6° | 0,000° / 0,000° |
| Suavizada, w = 1,0 m | 0,177° | 4,1 °/s | -50,5° / 28,5° | **15,9° / 78,7°** |

- Com elos de 0,5 m, qualquer janela ≤ 0,5 m cai inteira no ultimo elo e **e** o ultimo elo: sem
  diferenca de ruido nem de valor. A variacao medida (0,39° por amostra) e movimento real do cabo,
  nao ruido de medida — as poses do Gazebo sao exatas.
- A janela de 1,0 m cruza a junta 5 e mistura o elo 4: o jitter cai pela metade, mas a direcao passa
  a ter vies de 16° (mediana) e ate 79° quando o cabo dobra, e deixa de ser local. Em repouso, com o
  `folded_ground`, a diferenca chegou a 72,8°.
- **Decisao:** manter `w = 0,15 m` (local, igual ao ultimo elo nesta discretizacao). A suavizacao so
  traz beneficio com elos menores que a janela; o codigo ja suporta isso sem mudanca.

## 7. S3 preliminar — compensacao de atitude em voo (C1 com filtro)

`tools/plot_tether_attitude_series.py` (`s3_series.svg`, `s3_series.csv`, `s3_metrics.json`):

| Grandeza | Valor |
| --- | --- |
| Amostras alinhadas (drone_rpy, t_hat, angulos; tolerancia 10 ms) | 17 293 |
| Atitude no voo | roll -2,3…1,8°, pitch -2,1…0,6°, yaw -0,8…2,9° |
| Residuo `angulo(t_hat_body, R_BW(rpy) t_hat_world)` | mediana 0,000°, p95 0,000°, max 0,10° |
| \|elevacao_body - elevacao_world\| | mediana 0,20°, p95 0,89°, max 2,3° (da ordem da inclinacao do drone) |
| \|azimute_body - (azimute_world - yaw)\| (elevacao > -80°) | mediana 0,13°, p95 1,31° |
| Plugin x Python pelas poses: `t_hat_world` / `t_hat_body` | mediana 0,012° / 0,013°; p95 0,18° / 0,18° (n = 3980) |

O residuo zero mostra que os topicos do plugin sao coerentes entre si; a comparacao com as poses e
a verificacao independente. Com roll/pitch de ~2° neste voo a compensacao e pequena em modulo — a
bancada T1 (±15°) e a evidencia forte de que a atitude esta sendo removida corretamente.

### 5.3 Passo de fisica 2 ms (campanha `collision/step2ms`) — C0/C1/C2 PASS

Uma mudanca em relacao a 5.2: `STEP=0.002`. A campanha serve uma copia do mundo `default` do PX4
(`results/.../world/default.sdf`, gerada por `tools/generate_px4_world_step.py`, `max_step_size` 0,002, `real_time_update_rate` 500) antes do PX4,
que se liga a ela (`gazebo already running world: default`). Conferido em `/world/default/stats`:
19 404 iteracoes em 38,808 s simulados e 1050 iteracoes em 2,10 s — 500 Hz. Mesmos filtro de
colisao, cabo, constraint e missoes de 5.2.

| Etapa | 4 ms (`mask`) | 2 ms (`step2ms`) |
| --- | --- | --- |
| C0 repouso, penetracao | 0,30 µm — PASS | 0,10 µm — PASS |
| C1 arraste, penetracao max | **2,89 mm** no pouso — FAIL | **0,87 mm** (descida, drone a 0,63 m) — PASS |
| C1 dx realizado / retorno | 0,94 / 0,05 m | 0,95 / 0,07 m |
| C1 RMS XY / RMS Z hover | 0,11 / 0,098 m | 0,105 / 0,089 m |
| C1 roll / pitch max | 2,1 / 2,0° | 2,7 / 1,0° |
| C1 `\|F\|` mediana / max, saturacao | 0,61 / 1,87 N, 0 | 0,52 / 0,84 N, 0 |
| C2 pouso, penetracao max | nao executado | **1,54 mm** — PASS |
| C2 hover / RMS XY / RMS Z hover | — | ~2,1 m / 0,063 m / 0,112 m |
| C2 roll / pitch max, `\|F\|` mediana / max | — | 2,1 / 1,2°, 0,65 / 2,08 N, sem saturacao |
| RTF | 0,994 | 0,99 |
| Aborto / failsafe / NaN | 0 / 0 / 0 | 0 / 0 / 0 |

- **O passo reduziu a penetracao 3,3× no mesmo cenario** (2,89 → 0,87 mm), mais que a proporcao do
  passo; o RTF nao mudou (N = 5).
- C2: uma unica excursao abaixo de -1 mm, t = 171,39–171,75 s (0,36 s), minimo -1,54 mm, com o drone
  ainda a 1,78 m descendo — o trecho de cabo apoiado no solo sendo movido, nao o impacto do pouso.
  Folga de 0,46 mm ate o limite de 2 mm: **PASS com margem pequena**.
- Com o cabo pendurado quase na vertical sob o drone em hover (elevacao no corpo -88 a -89°), o
  azimute nao e interpretavel nessas fases.
- Raio de colisao e 1 ms nao foram necessarios e nao foram testados.

## 8. Sequencia integrada a 2 ms (campanha `integrated/step2ms`)

`STEP=0.002 DRONE_MASK=1 CABLE_MASK=2 tools/run_tether_angle_collision_campaign.sh integrated step2ms`,
sessao nova depois de C0–C2 passarem.

- **S0 PASS.** Estatico 30 s: penetracao 0,10 µm, RTF 0,989, sem NaN nem buracos.
- **S1 FAIL — so por penetracao (2,02 mm contra 2 mm).** Voo limpo: sem aborto nem failsafe, hover a
  ~2,0 m, RMS XY 0,047 m, RMS Z em hover 0,118 m, roll/pitch max 1,1 / 1,7°, erro de retorno 0,02 m,
  `|F|` mediana 0,95 N (max 1,85 N) sem saturacao, RTF 0,99, maior buraco nos topicos 0,10 s.
  Elevacao no corpo: -79° na subida, -89° em hover (cabo quase na vertical; azimute nao
  interpretavel), -30° no pouso.
- **A penetracao nao e impacto de descida:** a excursao dura 2,4 s (t = 100,54–102,92 s) com o drone
  **ja pousado** (z = 0,227 m), enquanto a forca da conexao sobe para 1,59 N (mediana no pouso). E um
  elo mantido ~2 mm dentro do solo sob carga depois do toque. O mesmo perfil de voo em C2 deu
  1,54 mm: **a 2 ms a profundidade no pouso oscila entre 1,5 e 2,0 mm entre sessoes, no limite do
  criterio** — nao e robusto.
- **S2 nao executado** (bloqueado por S1). O criterio de 2 mm nao foi relaxado.
- Proxima mudanca unica: passo de 1 ms (secao 9).

## 9. Passo de fisica 1 ms

Uma mudanca em relacao a 8: `STEP=0.001` (copia do mundo `default` com `max_step_size` 0,001 e
`real_time_update_rate` 1000; conferido: 39 465 iteracoes em 39,465 s simulados e 3049 iteracoes em
3,049 s — 1000 Hz; RTF 0,97–0,99). Filtro de colisao, cabo, constraint e missoes iguais.

### 9.1 C0/C1/C2 a 1 ms (campanha `collision/step1ms`) — PASS

| Metrica | 4 ms | 2 ms | **1 ms** |
| --- | --- | --- | --- |
| C0 penetracao | 0,30 µm | 0,10 µm | **0,02 µm** |
| C1 penetracao max (onde) | 2,89 mm (pouso) — FAIL | 0,87 mm (descida) | **0,59 mm** (descida, drone a 0,51 m) |
| C2 penetracao max (onde) | — | 1,54 mm | **0,56 mm** (descida, drone a 0,69 m) |
| C1 dx realizado / retorno | 0,94 / 0,05 m | 0,95 / 0,07 m | 0,94 / 0,03 m |
| C1 RMS XY / RMS Z hover | 0,11 / 0,098 m | 0,105 / 0,089 m | 0,107 / 0,092 m |
| C1 roll / pitch max | 2,1 / 2,0° | 2,7 / 1,0° | 2,5 / 0,7° |
| C2 RMS XY / RMS Z hover | — | 0,063 / 0,112 m | 0,060 / 0,127 m |
| C2 roll / pitch max | — | 2,1 / 1,2° | 2,0 / 3,0° |
| `\|F\|` mediana / max (C1; C2) | 0,61 / 1,87 N; — | 0,52 / 0,84 N; 0,65 / 2,08 N | 0,36 / 0,80 N; 0,45 / 1,61 N |
| Saturacao / aborto / failsafe / NaN | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| RTF | 0,994 | 0,99 | 0,98 |

Com 1 ms a profundidade maxima no pouso cai para ~0,6 mm (margem de 1,4 mm ao limite) sem custo de
RTF relevante nesta discretizacao (N = 5).

### 9.2 S0/S1/S2 a 1 ms (campanha `integrated/step1ms`) — PASS

`STEP=0.001 DRONE_MASK=1 CABLE_MASK=2 tools/run_tether_angle_collision_campaign.sh integrated step1ms`,
sessao nova depois de C0–C2 a 1 ms passarem.

| Metrica | S0 estatico | S1 vertical | S2 horizontal |
| --- | --- | --- | --- |
| Veredito | **PASS** | **PASS** | **PASS** |
| dx comandado / realizado | — | 0 / -0,03 m | **0,50 / 0,469 m** |
| Erro de retorno | — | 0,021 m | 0,028 m |
| RMS XY (rastreio) | — | 0,047 m | 0,153 m (inclui o transitorio do degrau) |
| RMS Z em hover | — | 0,123 m | 0,107 m |
| Roll / pitch max | — | 1,1 / 1,9° | 4,2 / 1,1° |
| Altura maxima do drone | 0,227 m (pousado) | 2,26 m | 2,29 m |
| `\|F_conexao\|` mediana / p95 / max | 0,034 N constante | 1,04 / 1,61 / 1,91 N | 0,37 / 1,16 / 2,19 N |
| Saturacao da constraint | 0 | 0 | 0 |
| Elevacao no corpo por fase (mediana) | 0° (cabo no solo) | subida -80,5°; hover -88,3/-88,9°; pouso -34,6° | subida -76,0°; ida -86,2°; volta -86,9°; pouso -53,4° |
| Penetracao max no solo | 0,02 µm | **0,74 mm** (drone pousado, t = 110,35 s) | **0,47 mm** (drone a 1,82 m) |
| NaN / maior buraco nos topicos | 0 / 0,08 s | 0 / 0,06 s | 0 / 0,10 s |
| RTF | 0,982 | 0,979 | 0,978 |
| Aborto DART / failsafe | 0 / 0 | 0 / 0 | 0 / 0 |

Com o cabo pendurado quase na vertical em hover (elevacao < -85° no corpo), o azimute nao e
interpretavel nessas fases, com qualquer metodo.

### 9.3 T2 nos voos a 1 ms

| Voo | Metodo | Jitter RMS / amostra | Elevacao no corpo (mediana / desvio) | x ultimo elo (mediana / p95) |
| --- | --- | --- | --- | --- |
| S1 | ultimo elo | 0,470° | -30,4° / 35,1° | — |
| S1 | w = 0,15 m / 0,5 m | 0,470° / 0,470° | -30,4° / 35,1° | 0,000° / 0,000° |
| S1 | w = 1,0 m | 0,267° | -15,2° / 37,9° | 15,4° / 78,6° |
| S2 | ultimo elo | 0,445° | -11,8° / 36,7° | — |
| S2 | w = 0,15 m / 0,5 m | 0,445° / 0,445° | -11,8° / 36,7° | 0,000° / 0,000° |
| S2 | w = 1,0 m | 0,296° | -17,1° / 34,4° | 48,3° / 70,6° |

Mesma conclusao de 6: nesta discretizacao a janela local e o ultimo elo; alargar a janela reduz o
jitter ao custo de um vies grande.

### 9.4 S3 — compensacao de atitude nos voos a 1 ms — PASS

Series em `results/tether_angles/integrated/step1ms/S{1_vertical,2_horizontal}/s3_series.{svg,csv}`
(paineis: roll/pitch/yaw; azimute world x body; elevacao world x body; residuo).

| Grandeza | S1 vertical | S2 horizontal |
| --- | --- | --- |
| Amostras alinhadas | 59 604 | 68 521 |
| Roll / pitch / yaw no voo | -1,2…1,1° / -2,1…0,9° / -3,9…1,4° | -4,5…3,9° / -0,8…1,2° / -4,4…-2,5° |
| Residuo `angulo(t_hat_body, R_BW t_hat_world)` mediana / p95 / max | 0,000 / 0,000 / 1,87°* | 0,000 / 0,000 / 0,23° |
| \|elevacao_body - elevacao_world\| mediana / p95 / max | 0,03 / 0,56 / 2,18° | 0,00 / 0,85 / 3,30° |
| \|azimute_body - (azimute_world - yaw)\| mediana / p95 (elev. > -80°) | 0,00 / 0,61° | 0,00 / 0,57° |
| Plugin x Python pelas poses (`t_hat_world`; `t_hat_body`) mediana / p95 | 0,0016 / 0,059°; 0,0017 / 0,058° | 0,0010 / 0,060°; 0,0011 / 0,060° |

\* amostra isolada de desalinhamento temporal entre topicos gravados separadamente; p95 = 0.

No S2 o drone rola ate ±4,5° e a elevacao no corpo difere da do mundo em ate 3,3°, na mesma ordem;
o azimute no corpo acompanha `azimute_world - yaw` a 0,6° (p95). A medida no corpo muda com a
atitude exatamente como a transformacao preve, e o calculo independente pelas poses confirma a
direcao a 0,06° (p95). Junto com a bancada T1 (±15°), isso demonstra a compensacao explicita.

## 10. Forca x tangente com colisao e filtro

O terceiro campo de `/cabo/conexao/angles_body` (angulo entre a forca sobre o drone e `t_hat`) ficou
grande nesta configuracao: S0 160°, S1 63° (mediana), S2 24° (mediana), contra 0,5–0,7° na X2 (sem
colisao, ponta do cabo livre para atravessar o solo). Com o cabo apoiado no solo e a folga da
constraint (`|e|` = 0,1–0,2 m) da ordem da geometria local, a forca aponta da ponta do cabo para o
attach, e nao ao longo do cabo. **A direcao da forca nao serve como proxy da direcao do tether
nesta configuracao; a tangente geometrica sim.** O modulo da forca continua sendo a medida direta de
carga na conexao.

## 11. Criterio de PASS

| Criterio | Resultado |
| --- | --- |
| Links do tether confirmados e documentados | sim (1.1) |
| Forca via plugin preservada | sim: mesma lei, mesmos topicos; `|F|` publicado em todas as etapas |
| `t_hat_world` valida | sim: invariante a atitude na bancada (≤ 0,007°), plugin x Python ≤ 0,06° (p95) em voo |
| `t_hat_body` corretamente transformada | sim: residuo `R_BW` ≤ 0,0015° (bancada), p95 0 em voo |
| Angulos body coerentes | sim: previsao na bancada ≤ 0,007° (vetor); em voo acompanham roll/pitch/yaw |
| Sem NaN / buracos | sim: 0 NaN; maior buraco 0,10 s (gravacao por `gz topic`, tolerancia 0,25 s) |
| Colisao com solo robusta | **sim com passo de 1 ms e filtro cabo x drone**; nao a 4 ms (2,9 mm) nem de forma robusta a 2 ms (1,5–2,0 mm) |
| Voo vertical | PASS (1 ms) |
| Voo horizontal | PASS (1 ms) |
| Sem crash DART / sem failsafe | sim nas campanhas com filtro (o crash de C1 sem filtro foi contato cabo–drone) |
| RTF aceitavel | 0,98 a 1 ms (N = 5) |

**Status: PASS** na configuracao `DRONE_MASK=1 CABLE_MASK=2 STEP=0.001`, spawn com
`PX4_GZ_MODEL_POSE=0,0,0.24` e missoes com `--relative-altitude`.

## 12. Limitacoes

- **Configuracao validada, nao padrao:** o filtro de colisao usa uma copia do X500 gerada em
  `results/`, e o passo de 1 ms usa uma copia do mundo `default` servida antes do PX4. O fluxo
  `make px4_sitl gz_x500` puro continua a 4 ms.
- **Cabo e drone nao colidem** por construcao (mascaras): o cabo pode atravessar o X500. Foi a forma
  de remover o contato nao fisico com as caixas de colisao dos rotores girando.
- **Discretizacao grossa (N = 5, elos de 0,5 m):** a tangente local e o ultimo elo; a janela de
  0,15 m nao suaviza nada aqui. Com elos menores a mesma implementacao passa a mediar juntas.
- **Azimute singular** com o cabo proximo da vertical do drone (hover sobre a estacao).
- **Direcao da forca ≠ direcao do cabo** com colisao (secao 10).
- **Uma sessao por configuracao:** a variacao entre sessoes a 2 ms (1,54 x 2,02 mm no mesmo perfil)
  mostra que as margens devem ser lidas com essa dispersao; a 1 ms foram 0,56 / 0,74 mm.
- **Custo de 1 ms com N maior nao medido.** Com N = 5 o RTF foi 0,98; cabos mais finos devem custar
  mais.
- Contato no dartsim: `kp`/`kd` nao disponiveis; raio de colisao maior nao foi necessario nem testado.
- Gravacao por `gz topic -e` em texto: `Vector3d` todo zero chega vazio; alinhamento entre topicos
  por relogio de parede (residuos isolados de ate 1,9° no S3).

## 13. Recomendacao

Manter o plugin de forca como baseline, com a medicao angular por tangente geometrica no frame do
drone (`/cabo/conexao/angles_body`, `/cabo/conexao/t_hat_body`) e `w = 0,15 m`. Para voos com cabo
com colisao, usar a configuracao validada: filtro cabo x drone, passo de 1 ms, spawn a z = 0,24 e
altitude relativa. A direcao da forca nao deve ser usada como angulo do cabo.

## 14. Revalidacao na consolidacao (2026-09-15)

Antes do commit da baseline, os arquivos e as saidas das ferramentas foram renomeados para ingles
(campanhas `collision`/`integrated`, etapas `C0_rest`, `C1_drag`, `C2_landing`, `S0_static`,
`S1_vertical`, `S2_horizontal`, arquivos `connection_w*.csv`, `verdict.txt`, `status.txt`,
`world/`, `drone_models/`; os nomes antigos `colisao`/`integrado` continuam aceitos como alias) e o
mundo de 1 ms passou a ser gerado por `tools/generate_px4_world_step.py`. As campanhas foram
rodadas de novo com esse codigo (rotulo `consolidated`):

| Etapa | Resultado |
| --- | --- |
| C0 / C1 / C2 | PASS / PASS (0,40 mm, dx 0,97 m) / PASS (0,88 mm) |
| S0 | PASS (RTF 0,982) |
| S1 vertical | PASS: RMS XY 0,045 m, RMS Z hover 0,136 m, roll/pitch 2,6/2,1°, 0,65 mm, RTF 0,977 |
| S2 horizontal | PASS: dx 0,50 → 0,538 m, retorno 0,060 m, roll/pitch 4,4/2,7°, 0,81 mm, RTF 0,978 |
| S3 (S1; S2) | residuo `R_BW` p95 = 0; plugin x poses p95 0,066° / 0,051° |
| Aborto / failsafe / NaN / saturacao | 0 / 0 / 0 / 0 |

As diferencas em relacao a secao 9 (dx realizado 0,47 → 0,54 m, roll max 1,1 → 2,6° no S1) sao
variacao entre sessoes; todos os criterios continuam atendidos com a mesma margem de penetracao
(< 1 mm). A tabela de referencia do operador esta em `docs/TEST_EXECUTION_GUIDE.md`.

## Comandos

Os procedimentos do operador (build, bancada de atitude, campanhas de colisao e integrada, analise,
graficos, execucao manual com GUI, checklist e valores de referencia) estao em
`docs/TEST_EXECUTION_GUIDE.md`, secao **Baseline oficial — tether por forca + angulos no frame do
drone**. Este documento fica com a parte tecnica.
