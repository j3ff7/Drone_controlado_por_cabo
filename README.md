# Drone Controlado por Cabo — PX4 + Gazebo + ROS 2

Este repositório contém o desenvolvimento de uma plataforma de simulação para um **UAV X500 controlado pelo PX4 Autopilot e conectado a um cabo físico modelado no Gazebo**.

O projeto tem como objetivo estudar a dinâmica e o controle de um sistema aéreo cabeado (*tethered UAV*), considerando as restrições impostas pelo cabo sobre o movimento do drone.

A simulação utiliza o **PX4 SITL (Software In The Loop)** integrado ao **Gazebo**, enquanto o **ROS 2** é utilizado para desenvolvimento de nós de controle, aquisição de dados e integração com algoritmos de maior nível.

---

## 📋 Descrição do Projeto

O sistema é composto principalmente por:

* **PX4 Autopilot**, responsável pelo controle e pela dinâmica de voo do UAV;
* **Gazebo**, utilizado para a simulação física do drone e do cabo;
* **modelo X500**, utilizado como plataforma aérea;
* **modelo físico do cabo**, composto por múltiplos elos articulados;
* **junta esférica (*ball joint*)**, responsável pela conexão entre o drone e o cabo;
* **ROS 2**, utilizado para controle externo, aquisição de dados e desenvolvimento de algoritmos;
* **QGroundControl**, utilizado para acompanhamento e interação com o veículo durante a simulação.

A arquitetura atual parte do modelo X500 disponibilizado pelo PX4. O modelo do cabo é incorporado diretamente ao arquivo SDF do X500, permitindo que o PX4 inicie o drone já conectado ao cabo.

---

## 🏗️ Arquitetura da Simulação

A arquitetura geral pode ser representada da seguinte forma:

```text
                    ┌──────────────────────┐
                    │      QGroundControl  │
                    └──────────┬───────────┘
                               │
                               │ MAVLink
                               │
                    ┌──────────▼───────────┐
                    │    PX4 Autopilot     │
                    │       SITL           │
                    └──────────┬───────────┘
                               │
                               │
                    ┌──────────▼───────────┐
                    │       Gazebo         │
                    │                      │
                    │  ┌────────────────┐  │
                    │  │      X500      │  │
                    │  │                │  │
                    │  │   base_link    │  │
                    │  └───────┬────────┘  │
                    │          │            │
                    │      Ball Joint       │
                    │          │            │
                    │  ┌───────▼────────┐  │
                    │  │      Cabo      │  │
                    │  │  elos flexíveis│  │
                    │  └────────────────┘  │
                    └──────────┬───────────┘
                               │
                               │ ROS 2 / DDS
                               │
                    ┌──────────▼───────────┐
                    │        ROS 2         │
                    │                      │
                    │ Controladores / Nós  │
                    │ Aquisição de dados   │
                    │ Algoritmos           │
                    └──────────────────────┘
```

O **PX4 continua sendo o controlador de voo do drone**. O cabo não substitui o controlador do PX4; ele é incorporado ao ambiente físico do Gazebo e passa a exercer forças e restrições sobre o veículo durante a simulação.

---

## 🔗 Integração do Cabo ao X500

A principal modificação em relação ao modelo X500 original do PX4 consiste na inclusão do modelo do cabo diretamente no arquivo SDF do veículo.

O cabo é incluído utilizando:

```xml
<include>
  <name>cabo_anexado</name>
  <uri>model://cabo</uri>
  <pose>0 0 0.2 0 0 0</pose>
</include>
```

Em seguida, é criada uma junta esférica entre o `base_link` do X500 e o `raiz_cabo`:

```xml
<joint name="drone_cabo_joint" type="ball">
  <parent>base_link</parent>
  <child>cabo_anexado::raiz_cabo</child>
</joint>
```

Dessa forma, o cabo acompanha o drone durante a simulação e pode exercer influência sobre sua dinâmica.

A **ball joint** permite rotação relativa entre o drone e o cabo, representando uma conexão articulada no ponto de acoplamento.

---

## 🪢 Modelo do Cabo

O cabo é representado por uma sequência de elos rígidos conectados por juntas articuladas.

Essa abordagem permite representar aproximadamente o comportamento de um cabo flexível utilizando um modelo discreto.

Cada elo possui propriedades físicas como:

* massa;
* comprimento;
* raio;
* inércia;
* amortecimento;
* atrito;
* limites articulares.

O modelo também pode ser configurado para representar diferentes comprimentos e geometrias do cabo.

O modelo do cabo está localizado em:

```text
models/
└── cabo_flexivel/
```

---

## 📁 Estrutura do Repositório

```text
Drone_controlado_por_cabo/
│
├── Chrono/
│   └── Arquivos relacionados às simulações utilizando Chrono
│
├── Coppelia/
│   └── Arquivos relacionados aos testes realizados no CoppeliaSim
│
├── Gazebo/
│   └── Arquivos relacionados ao ambiente e à configuração do Gazebo
│
├── models/
│   └── cabo_flexivel/
│       └── Modelo SDF do cabo
│
├── results/
│   └── controller_tests/
│       └── Resultados, logs e gráficos dos testes
│
├── src/
│   └── pacote_do_drone/
│       └── Nós e controladores ROS 2
│
└── README.md
```

> **Observação:** parte da estrutura pode variar conforme a versão atual do projeto.

---

# ⚙️ Pré-requisitos

Para executar a simulação, são necessários:

* Ubuntu;
* PX4 Autopilot;
* Gazebo compatível com a versão utilizada pelo PX4;
* ROS 2;
* QGroundControl (opcional, para acompanhamento do veículo);
* ferramentas de compilação utilizadas pelo PX4.

O PX4 fornece suporte nativo a simulações SITL e integração com Gazebo.

---

# 🚀 Executando a Simulação

## 1. Clonar o projeto

Clone este repositório:

```bash
git clone https://github.com/j3ff7/Drone_controlado_por_cabo.git
```

Entre no diretório:

```bash
cd Drone_controlado_por_cabo
```

---

## 2. Preparar o PX4 Autopilot

O projeto utiliza o **PX4-Autopilot como base para execução do SITL**.

Clone o PX4, caso ainda não esteja instalado:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Entre no diretório:

```bash
cd PX4-Autopilot
```

O PX4 recomenda a utilização do repositório com os submódulos (`--recursive`) para a configuração do ambiente de desenvolvimento.

---

## 3. Adicionar o modelo modificado do X500

O modelo X500 utilizado na simulação foi modificado para incorporar o cabo.

A modificação é realizada no arquivo SDF do X500, adicionando:

1. a inclusão do modelo do cabo;
2. a junta esférica entre o X500 e o cabo.

O trecho principal é:

```xml
<include>
  <name>cabo_anexado</name>
  <uri>model://cabo</uri>
  <pose>0 0 0.2 0 0 0</pose>
</include>

<joint name="drone_cabo_joint" type="ball">
  <parent>base_link</parent>
  <child>cabo_anexado::raiz_cabo</child>
</joint>
```

É importante garantir que o Gazebo consiga localizar o modelo:

```text
model://cabo
```

e seus arquivos correspondentes.

---

# ▶️ 4. Iniciar o PX4 SITL

A simulação é iniciada **diretamente pelo PX4**, utilizando o sistema de build `make`.

No diretório do PX4-Autopilot:

```bash
make px4_sitl gz_x500
```

Esse comando compila/inicia o PX4 SITL utilizando o veículo X500 e o Gazebo. O PX4 disponibiliza a execução de simulações SITL por meio de comandos `make` específicos para os simuladores suportados.

Com o modelo X500 modificado, o drone é iniciado já contendo a conexão com o cabo.

---

# 🧠 5. Executar o controlador ROS 2

Após iniciar a simulação, os nós desenvolvidos em ROS 2 podem ser executados separadamente.

Primeiro, carregue o ambiente do ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Depois, carregue o workspace do projeto:

```bash
source ~/ros2_ws/install/setup.bash
```

O nó de controle pode então ser iniciado, por exemplo:

```bash
ros2 run pacote_do_drone <nome_do_no>
```

O nome do e
