# Relatorio de testes PX4 SITL + MuJoCo

Resultados consolidados em 2026-10-05. Este documento registra a baseline
fisica usada para comparar e calibrar o desenvolvimento no Gazebo/PX4. Os
arquivos CSV, logs SITL, ambiente virtual e runtime PX4 permanecem locais e nao
sao versionados.

## Ambiente e configuracao

```text
PX4                 v1.14.4, commit 1555f2bd2229544c43966ab5f94879c41d8e1e01
MuJoCo              3.14.0
Bridge              WKoishi/mujoco_px4_sitl, commit 67630ff462c0bf84239b0315126d44415cef6a33
Integracao PX4      HIL por TCP, strict lockstep
Taxa da fisica      1000 Hz (dt = 0.001 s)
Massa do X500       2.064307692 kg
Comprimento tether  2.5 m
Densidade linear    0.06 kg/m
Massa tether        0.15 kg
```

O mundo MuJoCo usa ENU e o corpo do X500 usa FLU. O tether e uma cadeia de
corpos com ball joints, ancorada no mundo em uma extremidade. A outra
extremidade e ligada ao ponto de fixacao do X500 por uma constraint nativa
`connect`, que permite fechar a restricao sem adicionar um segundo parent joint.

## Matriz de testes

| Caso | Configuracao | Resultado | Evidencia principal |
| --- | --- | --- | --- |
| M0 | X500 sem tether | PASS | arm, takeoff, hover e land; RTF 1.000; RMS XY/Z 0.058/0.065 m |
| M1 estatico | tether N=30 | PASS | RTF 0.617; sem NaN; ancora sem drift |
| M1 voo | X500 + tether N=30 | PASS | erro de conexao max 0.50 mm; tensao max 1.267 N; RTF 0.429 |
| M2 | tether N=70 | PASS estatico | RTF 0.0470; sem NaN ou drift da ancora |
| M2 | tether N=100 | PASS estatico | RTF 0.0247; sem NaN ou drift da ancora |
| M3 | voo vertical, N=70 | PASS | tensao max 0.544 N; roll/pitch max 1.50/0.60 deg; RTF 0.0578 |
| M4 | deslocamento horizontal, N=70 | PASS | 0.433 m para referencia de 0.5 m; erro de retorno 0.040 m; RTF 0.0623 |
| Contato parcial | descida em patamares, N=70 | PASS | 15.6 a 53.2 elos no solo; RTF 0.111 a 0.078; correlacao -0.812 |
| Contato completo | airborne ate proximo do solo, N=70 | PASS | 0 a 63.8 elos no solo; RTF 0.102 a 0.038; correlacao -0.879 |

O caso M3 foi gravado antes da correcao do sinal do eixo longitudinal das
capsulas MuJoCo. Recalculada offline, a elevacao estacionaria media foi 81.0
graus, no intervalo 74.6 a 88.7 graus. O codigo atual e o caso M4 ja usam o
sinal corrigido.

## Sweep de contato com o solo

O teste `run_rtf_full_contact_sweep_n70.sh` parte com o cabo completamente no
ar e reduz a altura do X500 em oito patamares. Cada medida usa uma janela
estacionaria e contabiliza elos distintos em contato com o piso.

| Altura media (m) | Elos em contato (media) | RTF | Tensao media (N) |
| ---: | ---: | ---: | ---: |
| 2.585 | 0.00 | 0.1023 | 1.4659 |
| 2.303 | 6.54 | 0.1553 | 1.2970 |
| 1.998 | 15.72 | 0.1474 | 1.0950 |
| 1.579 | 27.92 | 0.0948 | 0.8994 |
| 1.242 | 37.87 | 0.0726 | 0.6572 |
| 0.814 | 49.32 | 0.0520 | 0.3876 |
| 0.498 | 58.39 | 0.0428 | 0.1925 |
| 0.307 | 63.80 | 0.0380 | 0.0749 |

A correlacao entre numero medio de elos em contato e RTF foi -0.879. O
primeiro patamar, sem contato, opera proximo da extensao total do cabo e nao
forma uma sequencia monotona com o segundo; depois do inicio do contato, o RTF
cai consistentemente com o aumento do numero de contatos. Um pico transitorio
de 15.50 N ocorreu durante a subida de condicionamento, quando o tether ficou
quase esticado. Esse pico foi excluido das janelas estacionarias.

## Reproducao

Na raiz do repositorio:

```bash
cd /home/lima/codes/ic/drone-cabo
bash experiments/mujoco_px4/scripts/prepare.sh

bash experiments/mujoco_px4/scripts/run_m0.sh
bash experiments/mujoco_px4/scripts/run_m1_n30.sh
bash experiments/mujoco_px4/scripts/run_m3_n70.sh
bash experiments/mujoco_px4/scripts/run_m4_n70.sh
bash experiments/mujoco_px4/scripts/run_rtf_contact_n70.sh
bash experiments/mujoco_px4/scripts/run_rtf_full_contact_sweep_n70.sh
```

Para executar o X500 graficamente:

```bash
bash experiments/mujoco_px4/scripts/run_sitl.sh --viewer
```

Os testes automatizados do experimento sao executados com:

```bash
PYTHONPATH=experiments/mujoco_px4/bridge \
  experiments/mujoco_px4/.venv/bin/pytest -q experiments/mujoco_px4/tests
```

O indice dos resultados validados e das falhas diagnosticas esta em
`experiments/mujoco_px4/results/README.md`.

## Conclusoes

- O MuJoCo manteve a cadeia N=70 numericamente estavel em voo e durante o
  contato progressivo com o solo.
- A cadeia articulada completa tem custo alto: o RTF ficou entre 0.038 e 0.155
  no sweep de contato, apesar de o X500 puro atingir RTF 1.0.
- O aumento de contatos com o piso e um custo dominante, mas a cadeia longa e
  a condicao quase esticada tambem afetam o desempenho.
- N=70 requer a taxa de fisica validada de 1000 Hz; um teste de isolamento a
  500 Hz divergiu em 0.022 s.
- MuJoCo deve permanecer como referencia fisica offline. Gazebo/PX4 deve usar
  um cabo reduzido/nodal calibrado contra estes resultados para manter RTF
  operacional.
