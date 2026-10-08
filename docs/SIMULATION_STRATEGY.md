# Estrategia de simulacao: MuJoCo de referencia e Gazebo/PX4 operacional

Decisao consolidada em 2026-10-06.

## Divisao de responsabilidades

### MuJoCo: referencia fisica offline

Usar `experiments/mujoco_px4` para:

- validar massa, comprimento, tensao, forma e tangente do tether;
- executar a discretizacao de referencia com `N=70`;
- quantificar o efeito de contato com o solo;
- produzir dados de referencia para calibrar modelos reduzidos.

O MuJoCo permaneceu estavel com `N=70`, inclusive no sweep de zero a cerca de
64 elos em contato com o solo. O custo e alto: RTF entre aproximadamente 0,038
e 0,155 conforme o contato. Por isso ele e referencia, nao backend de iteracao.

### Gazebo/PX4: desenvolvimento operacional

Usar Gazebo Sim + PX4 para:

- controle de voo e missoes OFFBOARD;
- integracao ROS 2, sensores e frames;
- sensor angular do tether;
- futura ground station e carretel;
- testes de sistema com RTF utilizavel.

A conexao entre drone e tether permanece force-based, com modelos independentes.
Isso evita o closed kinematic loop rejeitado pelo DART e fornece diretamente
forca, erro de conexao, tangente e angulos no frame do drone.

## O que nao deve virar baseline

- Cadeia Gazebo com 50-70 `BallJoint` e contato completo: RTF baixo, penetracao
  no solo e abortos DART durante movimento.
- `N=20` articulado apenas porque passou no estatico: tres voos abortaram durante
  a subida. O teste historico que parecia PASS usava tempo de parede e terminou
  depois de somente cerca de 10 s simulados.
- `UniversalJoint` ou revolutes alternadas: introduzem anisotropia dependente da
  orientacao arbitraria dos eixos.
- Aumentar apenas ganhos, `Fmax` ou reduzir timestep para mascarar uma topologia
  inadequada.

## Arquitetura alvo no Gazebo

```text
PX4/X500
  |
  +-- TetherForceConstraint (reacao no attachment point)
  |
  `-- cabo nodal do projeto
      +-- particulas com massa
      +-- restricoes isotropicas de distancia
      +-- gravidade e amortecimento
      +-- contato reduzido com o solo
      `-- geometria visual desacoplada da resolucao fisica
```

O cabo nodal deve ser implementado em plugin proprio, sem joints estruturais do
DART entre os nos. A primeira versao nao inclui carretel nem comprimento variavel.

## Sequencia de desenvolvimento

### D0 - infraestrutura temporal

Status: PASS.

- Missoes OFFBOARD usam tempo simulado por padrao.
- Congelamento do relogio simulado reprova a missao.
- O gravador aceita `--sim-duration` e possui timeout de parede.
- `--phase-clock wall` e `--duration` existem apenas para reproducao historica.

### D1 - experimento nodal minimo

Implementar mundo vazio com:

```text
ancora fixa + 5 nos + extremidade movel passiva
```

Validar conservacao aproximada do comprimento, ausencia de NaN, isotropia e RTF.
Comparar forma e tensao com um caso equivalente do MuJoCo.

### D2 - X500 estatico e hover

Conectar a extremidade nodal ao `tether_attach_link` pelo mecanismo force-based.
Executar, em ordem: X500 parado, arm, takeoff curto fisicamente viavel, hover,
land e disarm.

### D3 - convergencia

Testar `N=10`, `20`, `40` no modelo nodal. Escolher o menor N que reproduza a
referencia MuJoCo dentro das tolerancias abaixo, mantendo RTF operacional.

### D4 - contato reduzido com o solo

Adicionar contato no modelo nodal e repetir patamares de altura equivalentes ao
sweep MuJoCo. Evitar um par de colisao rigida por elo sempre que uma lei de
contato nodal ou comprimento apoiado for suficiente.

## Metricas e gates

| Grandeza | Gate inicial Gazebo vs MuJoCo |
| --- | --- |
| Tensao media estacionaria | erro <= 10% |
| Direcao da tangente no drone | erro <= 3 deg |
| Comprimento total | erro max <= 1% |
| Isotropia lateral | diferenca <= 5% entre eixos equivalentes |
| Erro da conexao force-based | RMS <= 0,10 m |
| Saturacao em `Fmax` | < 1% da janela estacionaria |
| Atitude em hover | roll/pitch max <= 5 deg |
| RTF operacional | alvo >= 0,5; preferencial >= 0,8 |
| Robustez | nenhum aborto, NaN ou relogio congelado |

Os gates podem ser refinados depois do primeiro experimento nodal, mas devem ser
alterados antes da corrida, nunca depois de observar o resultado.

## Comandos temporais corretos

```bash
./tools/record_tether_timeseries.py --sim-duration 75 --wall-timeout 1800 \
  --output-dir results/<caso> --prefix tether

./tools/px4_offboard_horizontal_mission.py --phase-clock sim \
  --output-dir results/<caso> --dx 0 --altitude 0.5 \
  --relative-altitude --rate 20
```

O comando de voo deve respeitar a geometria: distancia ancora-drone e deslocamento
solicitado nunca podem exceder o comprimento disponivel do tether.
