
# pegador_de_bandeiras

*Projeto da disciplina SSC0712 - Programação de Robôs Móveis (ICMC / USP) *

Este projeto implementa um robô móvel autônomo capaz de operar em um ambiente simulado no Gazebo utilizando ROS 2. O robô é responsável por localizar uma bandeira azul em uma arena contendo obstáculos, realizar sua captura utilizando uma garra robótica e posteriormente retornar até a base para efetuar o depósito da bandeira.
A navegação é baseada em fusão de informações provenientes de sensores LiDAR, câmera RGB e odometria, sendo toda a lógica de comportamento estruturada por meio de uma Máquina de Estados Finitos (FSM) que se adapta automaticamente a dois cenários distintos de desafios: a Arena Cilindros (obstáculos esparsos) e a Arena Paredes (labirintos e estruturas lineares).

Funcionalidades Implementadas
- Identificação Automática do Tipo de Arena: Detecção computacional inicial por visão para classificar o cenário operacional.
- Navegação e Tangenciamento de Labirintos: Algoritmo dedicado para contorno de paredes na Arena Paredes.
- Exploração Autônoma e Delimitação Visual: Rastreamento de zonas de solo colorido para refinamento de busca.
- Desvio Dinâmico de Obstáculos: Sistema adaptativo anticolisão baseado em LiDAR e janelas de severidade angular.
- Manipulação Robótica Assíncrona: Controle não-bloqueante da garra para captura e liberação do mastro da bandeira.
- Estratégias Avançadas de Evasão e Recuperação: Mitigação de oscilações, quinas e travamentos físicos.

https://github.com/Andre-Murakami/pegador_de_bandeiras_mk1/blob/pegador_de_bandeiras_mk1/Diagrama_de_estados.png

Máquina de Estados (FSM)
A inteligência de tomada de decisão é organizada sob uma FSM dividida em fases lógicas compartilhadas e específicas por ambiente:
1. Inicialização e Classificação
IDENTIFICANDO_ARENA: Estado inicial de calibração ativa. O robô avalia a proporção da malha estrutural na câmera. Se a ocupação visual ultrapassar o limiar de 30%, o robô chaveia para a lógica de Paredes; caso contrário, assume o cenário de Cilindros.
2. Comportamentos Específicos (Arena Paredes)
APROXIMANDO_PAREDE: O robô avança até detectar uma parede estrutural através do LiDAR frontal dentro do raio de segurança (0.67 m), definindo por proximidade angular o lado ideal de tangenciamento.
GIRANDO_PARALELO_PAREDE: Executa uma rotação precisa de 90º no eixo oposto ao obstáculo linear detectado.
SEGUINDO_PAREDE: Navegação tangente paralela à parede. O robô utiliza controle proporcional para manter a distância lateral restrita entre 0.60 m (self.distancia_minima_seguindo_parede) e 0.67 m. Caso perca a referência, inicia o protocolo de busca ativa de quinas.
AP_AVANCO_PRE_GIRO_NOVA_PAREDE: Micro-avanço temporizado para ultrapassar a quina antes de iniciar curvas fechadas.
GIRANDO_90_NOVA_PAREDE & AVANCANDO_FIXO_NOVA_PAREDE: Loop sequencial em dois ciclos (65º e 42.5º) para mapear e dobrar quinas sem colisão traseira.
3. Exploração, Busca e Captura (Comum / Arena Cilindros)
EXPLORANDO: Estado padrão de busca livre em linha reta por assinaturas visuais do alvo.
NAVEGANDO_PARA_AREA_SOLO_AZUL: Direciona o robô ao quadrante da bandeira ao identificar a zona azul por segmentação de cor.
PERMANECENDO_AREA_SOLO_AZUL & GIRANDO_180_AREA_SOLO_AZUL: Protocolos de confinamento e varreduras angulares de até 700º para localização rápida do mastro da bandeira.
NAVEGANDO_PARA_BANDEIRA: Alinhamento angular proporcional via câmera acoplado à velocidade linear adaptativa.
POSICIONANDO_PARA_COLETA: Ajuste milimétrico final a exatos 0.67 m do mastro utilizando dados centralizados do LiDAR.
CAPTURANDO_BANDEIRA: Automação sequencial controlada por registradores de ciclos (abertura de dedos, extensão mecânica, acoplamento, retração e elevação para tráfego seguro).
4. Retorno e Depósito
RETORNANDO_PARA_BASE: Rastreamento da zona verde. Ativa o mascaramento do LiDAR (Zona Cega Frontal entre -15º e +15º) para evitar que o robô tente desviar da bandeira que está transportando.
POSICIONANDO_PARA_DEPOSITO: Alinhamento axial com o centro da base receptora. Possui travas de limite de oscilação.
AVANCANDO_PARA_DEPOSITO: Avanço linear curto controlado por odometria (0.51 m ou 0.11 m).
RESETANDO_GARRA_BASE & SOLTANDO_BANDEIRA: Alinhamento angular final do atuador e abertura controlada da garra para liberação estável da carga.
5. Manobras de Emergência e Evasão
DESVIANDO_DE_OBSTACULO: Desvio reativo padrão baseado no menor vetor de distância lido pelo LiDAR.
RE_PRE_DESVIO: Acionado em proximidades críticas (<= 0.4 m). Força uma marcha à ré linear de afastamento rápido.
DESVIANDO_DE_OBSTACULO_2: Protocolo colisao acionado após 5 oscilações redundantes. Alterna entre curvas senoidais inversas e giros forçados de 100º para liberar o robô de quinas e pontos cegos.
Fluxo Geral de Operação

Tecnologias Utilizadas
1. ROS 2 (Humble)
2. Gazebo (Simulação Física e de Ambientes)
3. Python 3 / OpenCV (Filtragem HSVs e Processamento de Contornos de Cor)
4. NumPy & SciPy (Tratamento de Matrizes de Odometria e Quaterniões Eulerianos)
5. LiDAR Scan Msgs (Sensoriamento de Proximidade Bidirecional)

Estrutura Geral do Sistema
- O arquivo principal controle_robo.py centraliza a inteligência do agente, dividindo-se internamente em:
- Módulo de Percepção: Callbacks assíncronos de LiDAR (/scan), Câmera (/camera/image_raw) e Odometria (/odom).
- Filtragem de Ruído de Leitura: Métodos dinâmicos para mascarar dados espúrios e desconsiderar a carga frontal durante o retorno.
- Controle Cinemático Diferencial: Gerador dinâmico de velocidades lineares e angulares publicado em /cmd_vel com amortecimento adaptativo (self.fator_velocidade_arena_paredes = 0.5).

Estrutura Geral do Sistema
- O sistema é organizado em módulos responsáveis por:
- Percepção (LiDAR, câmera e odometria);
- Tomada de decisão baseada em estados;
- Controle de movimento diferencial;
- Controle da garra robótica;
- Navegação e recuperação de falhas.
- Essa arquitetura permite que o robô execute missões completas de busca, captura e transporte de objetos de forma totalmente autônoma em ambiente simulado.

Diagrama de Estados:
![Diagrama de Estados](https://github.com/jp-lopes/pegador_de_bandeiras/blob/pegador_de_bandeiras_mk1/Diagrama_de_estados.png)

Arquivos para apresentação na feira de extensão:
- [pôster](https://docs.google.com/presentation/d/1d3UxarLUDG2wu3aqnuSmSDFOD7z9jMMMoiP7K6U-u2s/edit?slide=id.p#slide=id.p)
- [slides](https://drive.google.com/file/d/1fvQyAbL2nSSsEmluT0DadpGj-qvy-iIv/view?usp=sharing)

## Autores
- Andre Luiz de Souza Murakami - nUSP 5631500 - [@Andre-Murakami](https://github.com/Andre-Murakami)
- Caio Cesar Trentin de Assis - nUSP 15674233 - [@CaioCesarTA](https://github.com/CaioCesarTA)
- João Pedro Lopes de Melo - nUSP 15588950 - [@jp-lopes](https://github.com/jp-lopes)






## 🚀 Instruções para Execução Local

### 1. Clonar o repositório

Acesse a pasta `src` do workspace ROS 2 e clone o projeto:

```bash
cd ~/ros2_ws/src
git clone -b pegador_de_bandeiras_mk1 https://github.com/Andre-Murakami/pegador_de_bandeiras_mk1.git
```

### 2. Instalar dependências

```bash
cd ~/ros2_ws
sudo apt update
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compilar o pacote

```bash
cd ~/ros2_ws
colcon build --packages-select pegador_de_bandeiras_mk1
```

### 4. Iniciar a simulação

```bash
source install/setup.bash
ros2 launch pegador_de_bandeiras_mk1 inicia_simulacao.launch.py
```

### 5. Abrir mais dois terminais

#### Terminal 1 — Carregar o robô

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch pegador_de_bandeiras_mk1 carrega_robo.launch.py
```

#### Terminal 2 — Iniciar o controle autônomo

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run pegador_de_bandeiras_mk1 controle_robo
```

---

# 🐳 Instruções para Execução com Docker

### 1. Clonar o repositório

```bash
cd ~/ros2_ws/src
git clone -b pegador_de_bandeiras_mk1 https://github.com/Andre-Murakami/pegador_de_bandeiras_mk1.git
```

### 2. Iniciar o container

```bash
cd ~/ros2_ws/src/pegador_de_bandeiras_mk1/docker

xhost +local:root

docker compose up -d
```

### 3. Entrar no container e compilar

```bash
docker exec -it ros2_humble_env bash

colcon build

source ~/.bashrc
```

### 4. Iniciar a simulação

```bash
ros2 launch pegador_de_bandeiras_mk1 inicia_simulacao.launch.py
```

### 5. Abrir mais dois terminais

#### Terminal 1 — Carregar o robô

```bash
docker exec -it ros2_humble_env bash

ros2 launch pegador_de_bandeiras_mk1 carrega_robo.launch.py
```

#### Terminal 2 — Iniciar o controle autônomo

```bash
docker exec -it ros2_humble_env bash

ros2 run pegador_de_bandeiras_mk1 controle_robo
```


















## Instruções para execução localmente

1. Acessar a pasta `src` do seu workspace ROS2 Humble e clonar o repositório:

    cd ~/ros2_ws/src
    git clone -b pegador_de_bandeiras_mk1 https://github.com/Andre-Murakami/pegador_de_bandeiras_mk1.git


2. Instalar dependências com `rosdep`:

    cd ~/ros2_ws
    sudo apt update
    sudo rosdep init        
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y


3. Compilar o pacote:

    cd ~/ros2_ws
    colcon build --packages-select 


4. Iniciar a simulação do Gazebo:

    source install/setup.bash
    ros2 launch pegador_de_bandeiras_mk1 inicia_simulacao.launch.py


5. Abrir mais dois terminais:
- No primeiro, carregar o robô na simulação:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch pegador_de_bandeiras_mk1 carrega_robo.launch.py


- No segundo, iniciar o controle autonômo do robô:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run pegador_de_bandeiras_mk1 controle_robo



## Instruções para execução com Docker

1. Acessar a pasta `src` do seu workspace ROS2 Humble e clonar o repositório:

    cd ~/ros2_ws/src
    git clone -b pegador_de_bandeiras_mk1 https://github.com/Andre-Murakami/pegador_de_bandeiras_mk1.git


2. Garantir permissões gráficas e iniciar container:
   
    cd ~/ros2_ws/src/pegador_de_bandeiras_mk1/docker
    xhost +local:root
    docker compose up -d


3. Entrar no container, compilar projeto e carregar variáveis:

    docker exec -it ros2_humble_env bash
    colcon build
    source ~/.bashrc


4. Iniciar simulação do Gazebo (dentro do container):

    ros2 launch pegador_de_bandeiras_mk1 inicia_simulacao.launch.py


5. Abrir mais dois terminais:
   
- No primeiro, carregar robô na simulação:

    docker exec -it ros2_humble_env bash
    ros2 launch pegador_de_bandeiras_mk1 carrega_robo.launch.py

- No segundo, iniciar o controle autonômo do robô:

    docker exec -it ros2_humble_env bash
    ros2 run pegador_de_bandeiras_mk1 controle_robo

