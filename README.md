Drone Controlado por Cabo — PX4 + Gazebo + ROS 2

Este repositório contém o desenvolvimento de uma plataforma de simulação para um UAV X500 controlado pelo PX4 Autopilot e conectado a um cabo físico modelado no Gazebo.

O projeto tem como objetivo estudar a dinâmica e o controle de um sistema aéreo cabeado (tethered UAV), considerando as restrições impostas pelo cabo sobre o movimento do drone.

A simulação utiliza o PX4 SITL (Software In The Loop) integrado ao Gazebo, enquanto o ROS 2 é utilizado para desenvolvimento de nós de controle, aquisição de dados e integração com algoritmos de maior nível.

📋 Descrição do Projeto

O sistema é composto principalmente por:

PX4 Autopilot, responsável pelo controle e pela dinâmica de voo do UAV;

Gazebo, utilizado para a simulação física do drone e do cabo;

modelo X500, utilizado como plataforma aérea;

modelo físico do cabo, composto por múltiplos elos articulados;

junta esférica (ball joint), responsável pela conexão entre o drone e o cabo;

ROS 2, utilizado para controle externo, aquisição de dados e desenvolvimento de algoritmos;

QGroundControl, utilizado para acompanhamento e interação com o veículo durante a simulação.

A arquitetura atual utiliza um modelo X500 modificado, denominado x500_cabo, que incorpora o modelo do cabo diretamente ao arquivo SDF do veículo. Dessa forma, o PX4 pode iniciar o drone já conectado ao cabo.

O projeto também possui um processo automatizado de preparação da simulação. O launch pode:

regenerar o modelo do cabo;

verificar os modelos necessários;

copiar os modelos x500_cabo e cabo para a pasta de modelos do PX4;

configurar as variáveis necessárias para o Gazebo;

iniciar o PX4 SITL com o airframe personalizado gz_x500_cabo.

🏗️ Arquitetura da Simulação

A arquitetura geral pode ser representada da seguinte forma:

                     ┌──────────────────────┐
                     │   QGroundControl     │
                     └──────────┬───────────┘
                                │
                              MAVLink
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
                     │  │  X500 + Cabo   │  │
                     │  │                │  │
                     │  │   base_link    │  │
                     │  └───────┬────────┘  │
                     │          │            │
                     │      Ball Joint       │
                     │          │            │
                     │  ┌───────▼────────┐  │
                     │  │      Cabo      │  │
                     │  │ elos flexíveis │  │
                     │  └────────────────┘  │
                     └──────────┬───────────┘
                                │
                            ROS 2 / DDS
                                │
                     ┌──────────▼───────────┐
                     │        ROS 2         │
                     │                      │
                     │ Controladores / Nós  │
                     │ Aquisição de dados   │
                     │ Algoritmos           │
                     └──────────────────────┘

O PX4 continua sendo o controlador de voo do drone. O cabo não substitui o controlador do PX4; ele é incorporado ao ambiente físico do Gazebo e passa a exercer forças e restrições sobre o veículo durante a simulação.

🔗 Integração do Cabo ao X500

A principal modificação em relação ao modelo X500 original do PX4 consiste na inclusão do modelo do cabo diretamente no arquivo SDF do veículo.

O cabo é incluído utilizando:

<include>
  <name>cabo_anexado</name>
  <uri>model://cabo</uri>
  <pose>0 0 0.2 0 0 0</pose>
</include>

Em seguida, é criada uma junta esférica entre o base_link do X500 e o raiz_cabo:

<joint name="drone_cabo_joint" type="ball">
  <parent>base_link</parent>
  <child>cabo_anexado::raiz_cabo</child>
</joint>

Dessa forma, o cabo acompanha o drone durante a simulação e pode exercer influência sobre sua dinâmica.

A ball joint permite rotação relativa entre o drone e o cabo, representando uma conexão articulada no ponto de acoplamento.

🪢 Modelo do Cabo

O cabo é representado por uma sequência de elos rígidos conectados por juntas articuladas.

Essa abordagem permite representar aproximadamente o comportamento de um cabo flexível utilizando um modelo discreto.

Cada elo possui propriedades físicas como:

massa;

comprimento;

raio;

inércia;

amortecimento;

atrito;

limites articulares.

O modelo também pode ser configurado para representar diferentes comprimentos e geometrias do cabo.

A estrutura utilizada atualmente pelo processo de geração é:

src/
└── pacote_do_drone/
    └── models/
        ├── build_tether.py
        └── models_sim/
            ├── cabo/
            │   └── model.sdf
            └── x500_cabo/
                └── model.sdf

O arquivo build_tether.py é responsável pela geração/atualização do modelo do cabo antes da simulação.

📁 Estrutura do Repositório

A estrutura geral do projeto inclui:

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
│   └── Modelos utilizados no projeto
│
├── results/
│   └── controller_tests/
│       └── Resultados, logs e gráficos dos testes
│
├── src/
│   └── pacote_do_drone/
│       ├── models/
│       │   ├── build_tether.py
│       │   └── models_sim/
│       │       ├── cabo/
│       │       └── x500_cabo/
│       │
│       ├── launch/
│       │   └── Launch files
│       │
│       └── Nós e controladores ROS 2
│
└── README.md

Observação: a estrutura pode variar conforme a versão atual do projeto. Os caminhos utilizados pelo launch devem seguir a estrutura definida no próprio arquivo de inicialização.

⚙️ Pré-requisitos

Para executar a simulação, são necessários:

Ubuntu;

PX4-Autopilot;

Gazebo compatível com a versão utilizada pelo PX4;

ROS 2;

Python 3;

ferramentas de compilação utilizadas pelo PX4;

QGroundControl (opcional, para acompanhamento do veículo).

O projeto atualmente utiliza caminhos padrão configurados no launch, portanto a instalação deve seguir a estrutura esperada.

Por padrão, o launch considera:

~/PX4-Autopilot
~/Drone_controlado_por_cabo

O pacote ROS 2 é esperado em:

~/Drone_controlado_por_cabo/src/pacote_do_drone

O PX4 fornece suporte nativo a simulações SITL e integração com Gazebo.

🚀 Executando a Simulação

1. Clonar o projeto

Clone este repositório:

git clone https://github.com/j3ff7/Drone_controlado_por_cabo.git

Entre no diretório:

cd Drone_controlado_por_cabo

2. Preparar o PX4 Autopilot

O projeto utiliza o PX4-Autopilot como base para execução do SITL.

Clone o PX4, caso ainda não esteja instalado:

git clone https://github.com/PX4/PX4-Autopilot.git --recursive

Entre no diretório:

cd PX4-Autopilot

O PX4 recomenda a utilização do repositório com os submódulos (--recursive) para a configuração do ambiente de desenvolvimento.

3. Preparar o workspace ROS 2

Entre no workspace do projeto e carregue o ambiente do ROS 2:

source /opt/ros/jazzy/setup.bash

Caso o pacote ROS 2 esteja em um workspace separado, carregue também o workspace compilado:

source ~/ros2_ws/install/setup.bash

Compile o workspace caso necessário:

cd ~/Drone_controlado_por_cabo
colcon build

Depois:

source install/setup.bash

🧩 4. Preparação automática dos modelos

O projeto possui um launch responsável por preparar os modelos antes de iniciar o PX4.

O arquivo utiliza:

src/pacote_do_drone/models/build_tether.py

para gerar o cabo.

Depois da geração, os modelos são copiados para a pasta de modelos do PX4:

~/PX4-Autopilot/Tools/simulation/gz/models/

Os modelos utilizados são:

x500_cabo/
cabo/

O processo remove a versão anterior dos modelos no destino e copia novamente os modelos atuais.

Isso permite atualizar o modelo do cabo sem precisar realizar manualmente a cópia dos arquivos para dentro do PX4.

▶️ 5. Iniciar a simulação utilizando o Launch

O launch automatizado possui três argumentos principais:

gerar_cabo
so_sincronizar
terminal

Execução padrão

Na configuração padrão:

o cabo é regenerado;

os modelos são sincronizados com o PX4;

o PX4 SITL é iniciado;

o PX4 é aberto em um terminal separado.

O comando geral é:

ros2 launch pacote_do_drone star_px4.launch.py

Substitua star_px4.launch.py pelo nome do arquivo .launch.py utilizado no pacote.

5.1. Não regenerar o cabo

Caso o modelo do cabo já esteja pronto e não seja necessário executá-lo novamente:

ros2 launch pacote_do_drone star_px4.launch.py gerar_cabo:=false

Nesse caso, o launch utiliza o modelo existente.

5.2. Apenas sincronizar os modelos

Para gerar o cabo e copiar os modelos para o PX4, sem iniciar a simulação:

ros2 launch pacote_do_drone star_px4.launch.py so_sincronizar:=true

O launch exibirá o comando que pode ser executado manualmente:

cd ~/PX4-Autopilot && \
PX4_SYS_AUTOSTART=4022 \
make px4_sitl gz_x500_cabo

Esse modo é útil para verificar se os modelos foram corretamente atualizados antes de iniciar o PX4.

5.3. Escolher o terminal

O launch permite selecionar o terminal utilizado para abrir o PX4:

ros2 launch pacote_do_drone star_px4.launch.py terminal:=gnome-terminal

Também são suportados:

terminal:=xterm

ou:

terminal:=konsole

Por padrão:

terminal = gnome-terminal

🛩️ 6. Airframe personalizado

A configuração atual utiliza o autostart:

4022

associado ao airframe:

4022_gz_x500_cabo

O arquivo esperado pelo launch é:

PX4-Autopilot/
└── ROMFS/
    └── px4fmu_common/
        └── init.d-posix/
            └── airframes/
                └── 4022_gz_x500_cabo

O alvo utilizado na compilação é:

gz_x500_cabo

Portanto, a inicialização manual correspondente é:

cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4022 make px4_sitl gz_x500_cabo

O launch também configura:

export PX4_SYS_AUTOSTART=4022
export PX4_SIM_MODEL=gz_x500_cabo

antes de iniciar o make.

Importante: o airframe personalizado 4022_gz_x500_cabo precisa existir no PX4. Caso contrário, o launch exibirá um aviso e o alvo gz_x500_cabo poderá não estar disponível.

🌍 7. Configuração do Gazebo

Para que o Gazebo consiga encontrar os modelos, o launch configura automaticamente a variável:

GZ_SIM_RESOURCE_PATH

O caminho dos modelos utilizados pelo projeto é adicionado à variável:

~/Drone_controlado_por_cabo/src/pacote_do_drone/models/models_sim

Isso permite que referências como:

<uri>model://cabo</uri>

sejam resolvidas pelo Gazebo.

Os modelos também são sincronizados com:

~/PX4-Autopilot/Tools/simulation/gz/models/

onde ficam disponíveis para a execução do PX4.

🧠 8. Executar o controlador ROS 2

Após iniciar a simulação, os nós desenvolvidos em ROS 2 podem ser executados separadamente.

Primeiro, carregue o ambiente do ROS 2:

source /opt/ros/jazzy/setup.bash

Depois, carregue o workspace:

source ~/ros2_ws/install/setup.bash

O nó de controle pode então ser iniciado, por exemplo:

ros2 run pacote_do_drone <nome_do_no>

O nome do executável deve ser substituído pelo controlador utilizado no experimento.

O ROS 2 é utilizado para desenvolver controladores e algoritmos que se comunicam com o PX4 durante a simulação.

🎮 Controle do Drone

O controle de voo é realizado pelo PX4 Autopilot.

Os comandos podem ser enviados por diferentes interfaces, incluindo:

QGroundControl;

ROS 2;

comandos internos do PX4;

outros sistemas compatíveis com a interface de comunicação utilizada.

Nos experimentos desenvolvidos neste projeto, o ROS 2 pode ser utilizado para implementar missões e controladores externos.

Exemplos de aplicações:

voo por waypoints;

controle de posição;

controle de velocidade;

controle em modo Offboard;

coleta de dados;

avaliação da influência do cabo sobre o movimento do drone.

📡 Integração PX4 + ROS 2

O PX4 possui suporte à integração com ROS 2 por meio de sua arquitetura de comunicação baseada em DDS.

A integração permite que nós ROS 2:

enviem comandos ao PX4;

recebam estados do veículo;

acompanhem posição e velocidade;

implementem controladores externos;

coletem dados da simulação.

Exemplos de mensagens utilizadas no desenvolvimento incluem:

OffboardControlMode
TrajectorySetpoint
VehicleCommand
VehicleStatus
VehicleLocalPosition

📊 Aquisição de Dados

Durante os experimentos podem ser registrados dados relacionados ao comportamento do veículo e do cabo.

Entre as variáveis de interesse estão:

posição do drone;

velocidade;

orientação;

trajetória desejada;

trajetória realizada;

comandos enviados ao PX4;

ângulos do cabo;

tensão no cabo;

comprimento do cabo.

Os resultados dos experimentos podem ser armazenados em:

results/controller_tests/

Esses dados podem posteriormente ser utilizados para gerar gráficos e avaliar o desempenho dos controladores.

🧪 Estado Atual do Projeto

Atualmente, o projeto possui uma arquitetura de simulação baseada em:

PX4 SITL
   +
Gazebo
   +
X500
   +
Cabo flexível
   +
Ball Joint
   +
ROS 2

O modelo do cabo está integrado ao modelo x500_cabo utilizado pelo PX4.

O processo de inicialização também possui automação para:

geração do modelo do cabo;

verificação dos arquivos SDF;

sincronização dos modelos com o diretório do PX4;

configuração do caminho de recursos do Gazebo;

configuração do autostart do PX4;

inicialização do alvo gz_x500_cabo.

Os principais esforços de desenvolvimento estão relacionados à:

modelagem física do cabo;

integração drone–cabo;

estabilidade da simulação;

controle do UAV considerando o cabo;

aquisição de dados;

desenvolvimento de controladores;

avaliação do comportamento dinâmico do sistema.

🔬 Objetivo de Pesquisa

O projeto está inserido no contexto de UAVs cabeados (tethered UAVs), nos quais um cabo físico conecta o veículo aéreo a uma estrutura externa.

A presença do cabo pode fornecer vantagens como:

operação prolongada;

fornecimento contínuo de energia;

comunicação por cabo;

redução da dependência de baterias.

Entretanto, o cabo também introduz restrições dinâmicas e geométricas que afetam:

mobilidade;

estabilidade;

planejamento de movimento;

controle;

segurança operacional.

Assim, a simulação desenvolvida neste projeto busca fornecer uma plataforma para estudar essas interações antes da implementação em sistemas físicos.

🔧 Fluxo resumido de execução

O fluxo recomendado para a versão atual é:

                ┌──────────────────────┐
                │  ROS 2 Launch File   │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ build_tether.py      │
                │ Geração do cabo      │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ Verificação dos SDF  │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ Sincronização        │
                │ x500_cabo + cabo     │
                │       → PX4          │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ PX4_SYS_AUTOSTART=4022│
                │ gz_x500_cabo         │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ PX4 SITL + Gazebo    │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ ROS 2 / QGroundControl│
                │ Controle e aquisição │
                └──────────────────────┘

📚 Referências

PX4 Autopilot

PX4 Autopilot — GitHub

Documentação PX4

PX4 User Guide

ROS 2

ROS 2 Documentation

📄 Licença

Este projeto está licenciado sob os termos da MIT License.