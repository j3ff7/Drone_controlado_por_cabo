Drone Controlado por Cabo — Simulação PX4 + Gazebo

Este repositório contém a simulação de um drone X500 controlado pelo PX4 Autopilot no Gazebo (Ignition/GZ), com um cabo flexível acoplado à base do drone por meio de uma junta esférica (ball joint). O objetivo é estudar o comportamento dinâmico do sistema drone–cabo e validar controladores em ambiente simulado.
📋 Descrição do Projeto

O drone é modelado a partir do modelo base x500_base do PX4 e recebe a inclusão do modelo cabo (definido em models/cabo_flexivel). A conexão entre o drone e o cabo é feita por uma junta do tipo ball entre o link base_link do drone e o link raiz_cabo do modelo do cabo.

A simulação é iniciada diretamente pelo ambiente de desenvolvimento do PX4, utilizando o comando make para compilar e executar o SITL (Software In The Loop) com o Gazebo.
🧩 Estrutura do Repositório
Diretório / Arquivo	Descrição
Chrono/	Arquivos relacionados à simulação com o motor Chrono
Coppelia/	Arquivos para simulação no CoppeliaSim
Gazebo/	Arquivos de mundo e configuração do Gazebo
models/cabo_flexivel/	Modelo SDF do cabo flexível
results/controller_tests/	Resultados de testes e missões do controlador
src/pacote_do_drone/	Pacote ROS/PX4 com nós e controladores do drone
.gitignore	Arquivos ignorados pelo Git
⚙️ Pré-requisitos

    PX4 Autopilot (versão compatível com Gazebo Ignition/GZ)

    Gazebo (Ignition/GZ) instalado

    ROS 2 (se aplicável ao seu pacote de controle)

    Ferramentas de compilação do PX4 (make, cmake, etc.)

🚀 Como Executar a Simulação

    Clone o repositório para o diretório de simulação do PX4 (ou ajuste os caminhos conforme necessário):
    bash

    git clone https://github.com/j3ff7/Drone_controlado_por_cabo.git
    cd Drone_controlado_por_cabo

    Compile e inicie o PX4 SITL utilizando o make (a partir do diretório raiz do PX4 Autopilot):
    bash

    make px4_sitl gz_x500

    O modelo do drone já inclui o cabo e a junta configurados no arquivo SDF do X500.

    Execute o controlador (se estiver usando um pacote ROS 2):
    bash

    ros2 run pacote_do_drone <nome_do_no>

        Ajuste o comando conforme a estrutura do seu pacote em src/pacote_do_drone.

🔗 Detalhes da Anexação do Cabo

A integração do cabo ao drone foi feita diretamente no arquivo SDF do modelo X500 (x500.sdf). Os trechos principais são:
xml

<!-- Inclusão do modelo do cabo -->
<include>
  <name>cabo_anexado</name>
  <uri>model://cabo</uri>
  <pose>0 0 0.2 0 0 0</pose>
</include>

<!-- Junta esférica conectando o drone ao cabo -->
<joint name="drone_cabo_joint" type="ball">
  <parent>base_link</parent>
  <child>cabo_anexado::raiz_cabo</child>
</joint>

Essa configuração permite que o cabo se mova livremente em todas as direções a partir do ponto de ancoragem, simulando uma conexão flexível.
🎮 Controlador

O controlador utilizado é o PX4, executado em modo SITL. A comunicação entre o PX4 e o Gazebo é feita nativamente pelo bridge do PX4. Caso haja um nó ROS 2 para controle adicional, ele pode ser encontrado em src/pacote_do_drone.
📊 Resultados e Testes

Os resultados dos testes de controlador estão armazenados em results/controller_tests/. Consulte os arquivos dessa pasta para visualizar logs, gráficos e métricas de desempenho.
📄 Licença

Este projeto está licenciado sob os termos da MIT License — sinta-se à vontade para usar e adaptar.
