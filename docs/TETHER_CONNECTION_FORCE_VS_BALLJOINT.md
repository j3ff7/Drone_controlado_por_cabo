# Conexao tether–drone: plugin de forca x `BallJoint`

Rodada de 2026-09-14, na `shared` a partir do checkpoint `43d6826`. Pergunta: vale introduzir
uma `BallJoint` na conexao tether–drone para medir a orientacao local do cabo, ou os angulos
podem ser obtidos mantendo a conexao por forca?

## 1. O que existe hoje (confirmado no codigo)

A `shared` tem **as duas** conexoes:

- **A — plugin de forca** (`src/pacote_do_drone/gz_plugins/TetherForceConstraint.cc`): a
  baseline das rodadas A, ativada no gerador do tether com `--force-constraint`.
- **B — junta fisica** (`worlds/x500_tether_joint.sdf`, rodada X1): junta `ball` de mundo da
  ponta do cabo ao `base_link` do X500, sem plugin.

### Conexao A, em detalhe

```text
e    = p_ponta - p_attach             p_ponta  = tether_link_N + R_N (l, 0, 0)
                                      p_attach = tether_attach_link + R_d (0, 0, 0)
F    = -K e - C e'                    K = 5 N/m, C = 0,5 N.s/m (baseline)
|F| <= Fmax = 3 N                     saturacao sinalizada

tether_link_N.AddWorldForce( F, (l, 0, 0))      reacao no cabo, na ponta
tether_attach_link.AddWorldForce(-F, (0, 0, 0)) forca no drone, no link de attach
```

- A forca e expressa no **mundo** e aplicada num ponto dado no **frame do link**;
  `AddWorldForce` deriva sozinho o momento `r x F` em torno do CoM de cada link. Como o
  `tether_attach_link` fica 0,12 m abaixo do `base_link`, a forca produz torque no X500.
- Nao ha junta entre cabo e drone: sao duas arvores independentes ligadas por uma mola-amortecedor.

Topicos ja existentes:

| Topico | Conteudo |
| --- | --- |
| `/cabo/conexao/force` | `F` sobre o cabo, frame do mundo (o drone recebe `-F`) |
| `/cabo/conexao/error` | vetor `e`, frame do mundo |
| `/cabo/conexao/stats` | (\|e\|, \|F\|, saturado) |
| `/cabo/estacao/tensao` | `T_est` no solo (completo, quase-estatico, disponibilidade) |
| `/cabo/estacao/exit_force` | forca na saida do cabo, mundo |
| `/cabo/estacao/exit_tangent` | tangente do **primeiro** elo (lado do solo), mundo |

**Nao existia tangente nem angulos junto ao drone.** A unica tangente publicada era a da
saida na estacao.

## 2. Os angulos podem ser medidos sem junta?

Sim, e a informacao ja esta no estado da simulacao. O ultimo elo (`tether_link_N`) e o
segmento ligado ao drone; sua orientacao e a tangente local do cabo ali. Nenhuma junta a mais e
necessaria para conhecer essa orientacao — uma `BallJoint` so **restringiria** a ponta a
coincidir com o drone; os graus de liberdade de rotacao do ultimo elo ja existem hoje.

Medicao implementada (so leitura, publicada pelo plugin a cada passo):

```text
t_world  = -R_N (1, 0, 0)              direcao que SAI do drone ao longo do cabo
t_body   = R_drone^-1 t_world          frame do tether_attach_link = base_link (x frente, y esquerda, z cima)
azimute  = atan2(t_y, t_x)             0 = frente, +90 = esquerda, +-180 = tras
elevacao = atan2(t_z, hypot(t_x,t_y))  0 = horizontal, -90 = cabo pendurado reto abaixo
F_body   = R_drone^-1 (-F)             forca da conexao sobre o drone, no frame do drone
desalinhamento = angulo(t_body, F_body)
```

| Topico novo | Conteudo |
| --- | --- |
| `/cabo/conexao/tangent_body` | `t_body` (unitario) |
| `/cabo/conexao/angles` | `(azimute, elevacao, desalinhamento forca–tangente)` em graus |
| `/cabo/conexao/force_body` | `F` sobre o drone, frame do drone |

Validacao cruzada: `tools/record_tether_connection.py` calcula os mesmos angulos de forma
independente, a partir das poses do Gazebo (`pose/info`), sem passar pelo plugin.

Frame: o `base_link` do Gazebo e FLU. Para usar no PX4 (FRD): `y` e `z` trocam de sinal, o
azimute troca de sinal e a elevacao tambem.

## 3. Comparacao conceitual

| Criterio | A — plugin de forca | B — `BallJoint` |
| --- | --- | --- |
| Fidelidade mecanica | complacente: a ponta se afasta `e = F/K` (~0,15 m na baseline); nao transmite momento, como uma articulacao ideal; `Fmax` limita cargas | rigida e sem folga; tambem nao transmite momento; nao ha limite de carga |
| Forca transmitida | **direta**: `F` sai da lei de controle a cada passo, sem sensor | **indireta**: exigiria `TransmittedWrench` / sensor `force_torque`, que ja derrubou o DART nesta stack (A0.2) |
| Orientacao/angulos do cabo | pela orientacao do ultimo elo (implementado); a junta nao e necessaria | a mesma orientacao do ultimo elo; posicoes de junta `ball` nao sao expostas como angulos no gz-sim, entao o calculo seria o mesmo |
| Estabilidade numerica | a forca limitada amortece picos; o drone permaneceu nivelado no controle da X1 | carga ilimitada passa inteira ao drone; na X1 o drone virou antes do aborto |
| Impacto no DART | duas arvores independentes; nao cria laco | junta de mundo com filho = raiz do drone: aceita, mas adiciona mais uma `BallJoint` a cadeia |
| Integracao | ja e a baseline das rodadas A; o PX4 spawna o X500 normalmente | exige mundo proprio e ligar o PX4 por `PX4_GZ_MODEL_NAME` |
| Risco de reintroduzir problemas de `BallJoint` | nenhum novo | alto: `BallJoint::updateRelativeTransform` foi a assinatura das quedas em voo da X1 |

Evidencia experimental ja disponivel (rodada X1, `docs/TETHER_X500_CONNECTION_ANALYSIS.md`):
com `BallJoint` o estatico passou em duas geometrias, mas o vertical abortou nas duas; com
forca, na mesma geometria com colisoes nos elos, o vertical tambem abortou, com o drone
nivelado. O experimento "plugin + `BallJoint`" nao faz sentido fisico: com a junta, `e` fica em
~0 e o plugin deixa de transmitir e de medir forca.

## 4. Experimento: plugin de forca com a medicao nova

Script: `tools/run_tether_angle_check.sh plugin_only` (resultados em
`results/angles/plugin_only/`). Baseline de voo das rodadas A: X500 spawnado pelo PX4 na
origem, tether N = 5 (`l` = 0,5 m, L = 2,5 m) em `folded_ground`, sem colisoes nos elos,
K = 5 N/m, C = 0,5 N.s/m, Fmax = 3 N, reel sem atuador. Estatico de 30 s com o PX4 de pe e voo
vertical curto (alvo 2 m) na mesma sessao.

Ressalvas da geometria herdada: o spawn do PX4 afunda o X500 (`base_link` a z ≈ 0,01–0,04 m) e,
sem colisoes, o cabo atravessa o solo abaixo da estacao. Nada disso afeta a medicao junto ao
drone, que usa so o ultimo elo e o link de attach.

### 4.1 A medicao esta correta

Comparacao entre o que o plugin publica e o calculo independente pelas poses do Gazebo,
alinhados por tempo simulado (plugin a 250 Hz, poses a 60 Hz):

| Grandeza | Estatico (1485 pares) | Vertical (3479 pares) |
| --- | --- | --- |
| \|Δ elevacao\| mediana / p95 / max | 0,044 / 0,274 / 0,856 graus | 0,015 / 0,080 / 0,348 graus |
| \|Δ azimute\| mediana / p95 / max (componente horizontal > 0,2) | 0,078 / 0,516 / 0,939 graus (n=394) | 0,188 / 0,647 / 1,509 graus (n=58) |
| Angulo entre as tangentes mediana / max | 0,061 / 0,883 graus | 0,024 / 0,485 graus |

O residuo e do alinhamento temporal entre os dois fluxos; as duas contas concordam.

### 4.2 Voo valido

Tempo simulado coberto: 69,7 de 70 s; sem aborto do DART; PX4: `Takeoff detected` →
`Landing detected` → `Disarmed by landing`, sem failsafe. Subida real pelas poses de 1,43 m
(o spawn afundado e o deslocamento da estimativa do PX4 comem parte do alvo de 2 m); roll max
2,85 graus, pitch max 0,90 graus; RTF 0,996.

### 4.3 O que as medidas mostram

| Grandeza | Estatico | Vertical (voo inteiro) |
| --- | --- | --- |
| Elevacao (mediana / p05 / p95) | -82,0 / -88,1 / -65,5 graus | -84,6 / -87,9 / -81,0 graus |
| **Desalinhamento forca x tangente** (mediana / p95) | **0,70 / 2,52 graus** | **0,58 / 1,29 graus** |
| \|F\| sobre o drone (mediana / p95) | 0,77 / 0,87 N | 0,79 / 1,08 N |
| `F_body` z (mediana) | -0,76 N (para baixo) | -0,78 N |
| \|e\| (mediana) / saturacao | 0,15 m / 0 | 0,16 m / 0 |

Por fase do voo: elevacao entre -84 e -86 graus em todas as fases; desalinhamento com mediana
0,46–0,53 grau de solo a hover e 0,99 grau (p95 2,34) no pouso; \|F\| de 0,79 N no solo para
1,05 N em hover, quando o drone passa a carregar mais cabo suspenso.

Duas conclusoes:

1. **A forca da conexao ja aponta ao longo do cabo** dentro de ~1 grau (mediana) e ~2,5 graus
   (p95) neste regime. `F_body` sozinho da quase a mesma direcao que a tangente geometrica; a
   tangente do ultimo elo continua sendo a medida preferivel porque nao depende da complacencia.
2. **O azimute e mal condicionado com o cabo quase vertical.** Com elevacao perto de -85 graus
   a componente horizontal da tangente e pequena (so 58 de 3479 amostras do voo passam de 0,2)
   e o azimute oscila muito. Isso e geometria, nao metodo: uma `BallJoint` teria exatamente o
   mesmo problema. O azimute so e util com o cabo inclinado (drone deslocado lateralmente).

### 4.4 `BallJoint` e "plugin + `BallJoint`"

Nao foram rodados de novo nesta rodada. A analise nao encontrou justificativa tecnica: a junta
nao acrescenta nenhuma informacao de orientacao (o ultimo elo ja a carrega) e tira a medida
direta de forca. A evidencia experimental de `BallJoint` ja existe na rodada X1 (estatico PASS,
vertical FAIL nas duas geometrias). "Plugin + `BallJoint`" nao e fisicamente coerente: com a
junta, `e` ≈ 0 e a forca do plugin some.

## 5. Recomendacao

**Manter a conexao por plugin de forca**, com a medicao geometrica agora publicada
(`/cabo/conexao/tangent_body`, `/cabo/conexao/angles`, `/cabo/conexao/force_body`).

- A `BallJoint` duplicaria graus de liberdade ja representados pelo ultimo elo e nao adicionaria
  informacao de angulo.
- A forca transmitida, que o plugin da de graca, passaria a exigir `TransmittedWrench`, que ja
  derrubou o DART nesta stack.
- O risco numerico da `BallJoint` na conexao ja foi observado (X1): carga ilimitada vira o drone
  e a cadeia aborta em `BallJoint::updateRelativeTransform`.
- Limitacoes que continuam valendo: a conexao e complacente (`e` ≈ 0,15 m) e o azimute so e
  interpretavel com o cabo inclinado.
