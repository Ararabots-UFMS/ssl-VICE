# Guia para Calouros - Ararabots Software (SSL)

Bem-vindo ao grupo de software do time Ararabots! Este guia vai te ajudar a configurar o ambiente, entender a estrutura do projeto e rodar o sistema pela primeira vez.

---

## Indice

1. [O que e a RoboCup SSL?](#1-o-que-e-a-robocup-ssl)
2. [Visao geral dos repositorios](#2-visao-geral-dos-repositorios)
3. [Requisitos do sistema](#3-requisitos-do-sistema)
4. [Instalacao do Ubuntu](#4-instalacao-do-ubuntu)
5. [Instalacao do ROS2 Humble](#5-instalacao-do-ros2-humble)
6. [Instalacao do Docker](#6-instalacao-do-docker)
7. [Clonando os repositorios](#7-clonando-os-repositorios)
8. [Instalando o grSim](#8-instalando-o-grsim)
9. [Instalando o ssl-game-controller](#9-instalando-o-ssl-game-controller)
10. [Configurando e rodando o ssl-VICE](#10-configurando-e-rodando-o-ssl-vice)
11. [Rodando a interface web (ssl-gui-react)](#11-rodando-a-interface-web-ssl-gui-react)
12. [Modos de execucao](#12-modos-de-execucao)
13. [Arquitetura do sistema](#13-arquitetura-do-sistema)
14. [Comandos uteis do ROS2](#14-comandos-uteis-do-ros2)
15. [Ferramentas de desenvolvimento](#15-ferramentas-de-desenvolvimento)
16. [Solucao de problemas comuns](#16-solucao-de-problemas-comuns)
17. [Proximos passos](#17-proximos-passos)

---

## 1. O que e a RoboCup SSL?

A **Small Size League (SSL)** e uma categoria da RoboCup onde times de ate 11 robos pequenos (180mm de diametro) jogam futebol autonomamente. O sistema funciona assim:

- **Cameras no teto** (ssl-vision) detectam a posicao dos robos e da bola
- **Nosso software** processa essas posicoes, decide a estrategia e calcula os movimentos
- **Comandos sao enviados** via radio para os robos reais (ou via UDP para o simulador)
- **Um arbitro automatico** (ssl-game-controller) controla o jogo (HALT, STOP, FREE KICK, etc.)

Para desenvolvimento e testes, usamos o **grSim**, um simulador que substitui os robos reais e as cameras.

---

## 2. Visao geral dos repositorios

Todos os repositorios ficam dentro de uma mesma pasta `ararabots/`:

```
ararabots/
├── ssl-VICE/              # Sistema principal (ROS2 + Python)
├── ssl-gui-react/         # Interface web (React + TypeScript)
├── grSim/                 # Simulador dos robos
└── ssl-game-controller/   # Arbitro automatico
```

| Repositorio | Funcao | Voce vai mexer? |
|---|---|---|
| [**ssl-VICE**](https://github.com/Ararabots-UFMS/ssl-VICE) | Cerebro do time: visao, estrategia, controle, comunicacao | Sim, sempre |
| [**ssl-gui-react**](https://github.com/Ararabots-UFMS/ssl-gui-react) | Interface web para visualizar o campo e enviar comandos | Sim, se trabalhar com frontend |
| **grSim** | Simulador - substitui robos reais e cameras durante desenvolvimento | Precisa instalar, raramente mexe no codigo |
| **ssl-game-controller** | Arbitro automatico do jogo | Precisa instalar, nao mexe no codigo |

---

## 3. Requisitos do sistema

### Obrigatorio

| Requisito | Versao | Observacao |
|---|---|---|
| **Ubuntu** | 22.04 LTS (Jammy) | Dual boot ou WSL2. A versao importa! |
| **ROS2** | Humble Hawksbill | Unica versao compativel com o projeto |
| **Python** | 3.10+ | Ja vem com o Ubuntu 22.04 |
| **Docker** | 20.10+ | Para rodar grSim e game-controller |
| **Git** | 2.x | Para clonar os repositorios |

### Recomendado

| Requisito | Observacao |
|---|---|
| **VS Code** | Editor de codigo com extensoes uteis |
| **Node.js 20+** | Necessario para rodar a interface web (ssl-gui-react) |
| **8 GB RAM** | Minimo para rodar tudo junto |
| **20 GB de disco livre** | Docker images + dependencias |

> **IMPORTANTE:** O ROS2 Humble so funciona no Ubuntu 22.04. Nao use outras versoes do Ubuntu nem outras distros, a menos que saiba exatamente o que esta fazendo.

---

## 4. Instalacao do Ubuntu

### Opcao A: Dual Boot (recomendado para desempenho)

1. Baixe a ISO do Ubuntu 22.04 LTS: https://releases.ubuntu.com/22.04/
2. Crie um pendrive bootavel com Rufus (Windows) ou Etcher
3. Instale ao lado do Windows (dual boot)
4. Separe no minimo **50 GB** para a particao do Ubuntu

### Opcao B: WSL2 (mais pratico, mas pode ter limitacoes de rede)

No PowerShell como administrador:
```powershell
wsl --install -d Ubuntu-22.04
```

Depois de instalar, abra o Ubuntu pelo terminal e atualize:
```bash
sudo apt update && sudo apt upgrade -y
```

> **Nota sobre WSL2:** O grSim (interface grafica) nao roda nativamente no WSL2. Voce vai precisar instalar o grSim no Windows e o resto no WSL2. A comunicacao UDP entre Windows e WSL2 funciona, mas pode precisar de configuracao extra de rede.

---

## 5. Instalacao do ROS2 Humble

Siga estes passos no Ubuntu 22.04:

### 5.1 Configurar o repositorio do ROS2

```bash
# Garantir que o locale esta em UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adicionar repositorio do ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 5.2 Instalar o ROS2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 5.3 Instalar ferramentas de build

```bash
sudo apt install python3-colcon-common-extensions python3-pip python3-rosdep -y
```

### 5.4 Configurar o source automatico

Adicione ao final do seu `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.5 Verificar a instalacao

```bash
ros2 --help
```

Se aparecer a lista de comandos do ROS2, esta funcionando.

---

## 6. Instalacao do Docker

```bash
# Instalar Docker
sudo apt install docker.io docker-compose -y

# Adicionar seu usuario ao grupo docker (evita precisar de sudo)
sudo usermod -aG docker $USER

# Reinicie o terminal ou faca logout/login
newgrp docker

# Verificar
docker --version
docker compose version
```

---

## 7. Clonando os repositorios

### Com HTTPS

```bash
mkdir ~/ararabots
cd ~/ararabots
git clone https://github.com/Ararabots-UFMS/ssl-VICE.git
git clone https://github.com/Ararabots-UFMS/ssl-gui-react.git
git clone https://github.com/RoboCup-SSL/ssl-game-controller.git
git clone https://github.com/RoboCup-SSL/grSim.git
```

### Com SSH

```bash
mkdir ~/ararabots
cd ~/ararabots
git clone git@github.com:Ararabots-UFMS/ssl-VICE.git
git clone git@github.com:Ararabots-UFMS/ssl-gui-react.git
git clone git@github.com:RoboCup-SSL/ssl-game-controller.git
git clone git@github.com:RoboCup-SSL/grSim.git
```

---

## 8. Instalando o grSim

O grSim e o simulador dos robos. Ele substitui os robos reais e as cameras durante o desenvolvimento.

### Opcao A: Instalar do fonte (Ubuntu)

```bash
# Dependencias
sudo apt install git build-essential cmake pkg-config \
    libode-dev libboost-dev \
    qtbase5-dev libqt5opengl5-dev \
    libprotobuf-dev protobuf-compiler \
    libgl1-mesa-dev -y

# Buildar
cd ~/ararabots/grSim
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Opcao B: Windows (se usar WSL2)

Baixe o grSim pre-compilado para Windows e rode nativamente. O grSim no Windows envia pacotes UDP que o WSL2 consegue receber.

### Configuracao importante do grSim

Ao abrir o grSim, verifique em **Game > Settings**:

| Configuracao | Valor |
|---|---|
| Vision Multicast Address | `224.5.23.2` |
| Vision Multicast Port | `10020` |
| Command Listen Port | `20011` |

Esses valores devem bater com o que o ssl-VICE espera.

---

## 9. Instalando o ssl-game-controller

O game-controller e o arbitro automatico. Ele envia comandos como HALT, STOP, FREE KICK, etc.

### Via Docker (recomendado)

```bash
docker pull robocupssl/ssl-game-controller
docker run -p 8081:8081 -p 10003:10003 robocupssl/ssl-game-controller
```

Acesse a interface do arbitro em: http://localhost:8081

### Via binario

Baixe a versao mais recente em: https://github.com/RoboCup-SSL/ssl-game-controller/releases

```bash
chmod +x ssl-game-controller_*
./ssl-game-controller_*
```

---

## 10. Configurando e rodando o ssl-VICE

### 10.1 Instalar dependencias Python

```bash
cd ~/ararabots/ssl-VICE
pip install -r requirements.txt
```

> Se der erro com `evdev`, nao se preocupe — ele so e necessario em Linux nativo com joystick.

### 10.2 Buildar os pacotes ROS2

```bash
cd ~/ararabots/ssl-VICE

# Sempre faca o source do ROS2 antes de buildar
source /opt/ros/humble/setup.bash

# Buildar (primeira vez demora mais)
colcon build

# Ativar os pacotes buildados
source install/setup.bash
```

> **IMPORTANTE:** Toda vez que abrir um novo terminal, voce precisa fazer:
> ```bash
> cd ~/ararabots/ssl-VICE
> source /opt/ros/humble/setup.bash
> source install/setup.bash
> ```
> Para evitar repetir, adicione no `~/.bashrc`:
> ```bash
> echo "source ~/ararabots/ssl-VICE/install/setup.bash" >> ~/.bashrc
> ```

### 10.3 Rodar o sistema

```bash
# Certifique-se que o grSim esta aberto e rodando

# Rodar todos os nodes de simulacao
ros2 launch launch/sim_one.py
```

---

## 11. Rodando a interface web (ssl-gui-react)

A interface web permite visualizar o campo, posicao dos robos e enviar comandos.

### 11.1 Instalar Node.js (se ainda nao tem)

```bash
# Instalar via nvm (recomendado)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install 20
```

### 11.2 Instalar dependencias e rodar

```bash
cd ~/ararabots/ssl-gui-react
npm install
npm run dev
```

Acesse em: http://localhost:5173

> A GUI se comunica com o ssl-VICE via WebSocket (Socket.IO). O node `gui_api` do ssl-VICE precisa estar rodando para a GUI funcionar.

---

## 12. Modos de execucao

### Simulacao completa (todos os nodes)

```bash
ros2 launch launch/sim_one.py
```

Sobe: vision, gameWatcher, controller, driver, grsim_messenger, manual_command, gui_api, referee

### Controle manual (joystick/teclado)

Minimo necessario para mover robos manualmente:

```bash
# Terminal 1
ros2 run vision visionNode --ros-args -p port:=10020

# Terminal 2
ros2 run manual_command manual_node

# Terminal 3
ros2 run grsim_messenger grsim_publisher_node
```

### Strategy Command GUI (enviar comandos de estrategia)

Para testar movimentos e posicionamento:

```bash
# Terminal 1
ros2 run vision visionNode --ros-args -p port:=10020

# Terminal 2
ros2 run control_unit gameWatcher

# Terminal 3
ros2 run new_movement driver

# Terminal 4
ros2 run control controller

# Terminal 5
ros2 run grsim_messenger grsim_publisher_node

# Terminal 6
ros2 run strategy_command_gui strategy_gui
```

### Dois times (simulacao completa)

```bash
ros2 launch launch/launch.py
```

### Hardware real (competicao)

```bash
ros2 launch launch/real_one.py
```

> **Dica:** Lembre-se de fazer `source install/setup.bash` em **cada terminal** antes de rodar qualquer comando ROS2.

---

## 13. Arquitetura do sistema

### Fluxo de dados

```
grSim/Cameras --> [UDP] --> Vision (Kalman) --> [ROS2] --> GameWatcher --> Strategy --> PathDriver --> Controller --> [UDP] --> grSim/Robos
                                                  ^                                                      |
                                             Referee ---------------------------------------------------+
```

### Pacotes ROS2 do ssl-VICE

| Pacote | O que faz | Frequencia |
|---|---|---|
| **vision** | Recebe posicoes via UDP, aplica filtro de Kalman | 60 Hz |
| **referee** | Recebe comandos do arbitro | ~10 Hz |
| **control_unit** | Agrega visao + arbitro em GameState | 60 Hz |
| **strategy** | Decide o que cada robo faz (Behavior Tree) | 10 Hz |
| **new_movement** | Planeja trajetoria evitando obstaculos (RRT) | 10 Hz |
| **control** | Controle PID de velocidade | 100 Hz |
| **grsim_messenger** | Envia comandos pro simulador (protobuf/UDP) | 100 Hz |
| **hardware_messenger** | Envia comandos pros robos reais (serial) | 100 Hz |
| **gui_api** | API Flask/SocketIO que conecta a interface web ao ROS2 | - |
| **manual_command** | Controle manual via joystick/teclado | - |
| **strategy_command_gui** | GUI para enviar comandos de estrategia manualmente | - |
| **system_interfaces** | Mensagens customizadas do ROS2 (VisionMessage, GameState, etc.) | - |
| **utils** | Funcoes utilitarias compartilhadas | - |

### Topics ROS2 principais

| Topic | Tipo | Descricao |
|---|---|---|
| `/visionTopic` | VisionMessage | Posicoes dos robos e bola |
| `/geometryTopic` | VisionGeometry | Dimensoes do campo |
| `/refereeTopic` | RefereeMessage | Comandos do arbitro |
| `/commandTopic` | TeamCommand | Comandos de velocidade para os robos |

### Portas de rede

| Porta | Protocolo | Direcao | Descricao |
|---|---|---|---|
| `10020` | UDP multicast | grSim -> Vision | Dados de visao (posicoes) |
| `10003` | UDP | game-controller -> Referee | Comandos do arbitro |
| `20011` | UDP | grsim_messenger -> grSim | Comandos para os robos |
| `5173` | HTTP | Browser -> ssl-gui-react | Interface web |
| `8081` | HTTP | Browser -> game-controller | Interface do arbitro |

---

## 14. Comandos uteis do ROS2

```bash
# Listar todos os topics ativos
ros2 topic list

# Ver mensagens de um topic em tempo real
ros2 topic echo /visionTopic

# Ver a frequencia de publicacao de um topic
ros2 topic hz /visionTopic

# Listar todos os nodes rodando
ros2 node list

# Ver info de um node (topics que ele publica/assina)
ros2 node info /vision_node

# Ver a estrutura de uma mensagem
ros2 interface show system_interfaces/msg/VisionMessage

# Listar todos os pacotes buildados
ros2 pkg list | grep -E "vision|control|strategy|grsim"
```

---

## 15. Ferramentas de desenvolvimento

### VS Code

Extensoes recomendadas:

| Extensao | Funcao |
|---|---|
| **Python** (Microsoft) | Suporte a Python |
| **Ruff** (Charlie Marsh) | Linting e formatacao Python (o projeto ja tem `ruff.toml`) |
| **ROS** (Microsoft) | Suporte a ROS2 |
| **GitLens** | Historico de alteracoes no codigo |

### Ruff (linting)

O projeto usa Ruff para manter o codigo padronizado. No VS Code:
- `Ctrl + Shift + P` > "Ruff: Format Document" para formatar
- O VS Code sublinha automaticamente trechos fora do padrao

### Git - fluxo basico

```bash
# Criar uma branch para sua tarefa
git checkout -b minha-feature

# Fazer alteracoes, depois commitar
git add .
git commit -m "Descricao do que fiz"

# Subir para o GitHub
git push origin minha-feature

# Criar Pull Request no GitHub
```

---

## 16. Solucao de problemas comuns

### "No module named 'google'" ou "No module named 'system_interfaces'"

O build esta incompleto ou desatualizado. Faca um build limpo:

```bash
cd ~/ararabots/ssl-VICE
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
pip install protobuf numpy flask flask-socketio ruckig pyserial inputs
colcon build
source install/setup.bash
```

### "package 'X' not found"

Voce esqueceu de fazer `source install/setup.bash`. Rode o source e tente novamente.

### Vision node nao recebe dados do grSim

1. Verifique se o grSim esta rodando
2. Confira as configuracoes de rede no grSim:
   - Vision Multicast Address: `224.5.23.2`
   - Vision Multicast Port: `10020`
3. Se estiver usando WSL2, pode ser problema de rede multicast. Tente:
   ```bash
   sudo ip route add 224.0.0.0/4 dev eth0
   ```

### colcon build falha

- Certifique-se que fez `source /opt/ros/humble/setup.bash` **antes** de buildar
- Se erros de dependencia, instale as dependencias Python:
  ```bash
  pip install -r requirements.txt
  ```
- Se o `system_interfaces` nao builda, pode ser falta do `rosidl`:
  ```bash
  sudo apt install ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime -y
  ```

### "not found: /opt/ros/humble/local_setup.bash"

O `install/` foi buildado com outra versao do ROS2. Limpe e rebuilde:
```bash
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
```

### Docker: permissao negada

```bash
sudo usermod -aG docker $USER
# Depois faca logout e login novamente
```

---

## 17. Proximos passos

### Para entender o sistema

1. Leia o documento de visao geral: `ssl-VICE/docs/visao-geral-sistema.md`
2. Rode o `sim_one.py` e observe os topics com `ros2 topic echo`
3. Abra a interface web (ssl-gui-react) e veja o campo em tempo real
4. Leia o codigo do `vision` node — e o mais simples e bom para comecar

### Para comecar a contribuir

1. Escolha uma issue no GitHub do [ssl-VICE](https://github.com/Ararabots-UFMS/ssl-VICE)
2. Crie uma branch a partir da `main`
3. Faca suas alteracoes e teste localmente com o grSim
4. Abra um Pull Request para review

### Ordem sugerida de estudo dos pacotes

1. **vision** — simples, recebe UDP e publica no ROS2
2. **system_interfaces** — entenda as mensagens customizadas
3. **control_unit** — como os dados sao agregados
4. **manual_command** — como enviar comandos para os robos
5. **grsim_messenger** — como os comandos chegam no simulador
6. **control** — controle PID
7. **new_movement** — planejamento de trajetoria (RRT)
8. **strategy** — logica de decisao (Behavior Trees)

### Links uteis

- [ssl-VICE (GitHub)](https://github.com/Ararabots-UFMS/ssl-VICE)
- [ssl-gui-react (GitHub)](https://github.com/Ararabots-UFMS/ssl-gui-react)
- [Documentacao ROS2 Humble](https://docs.ros.org/en/humble/)
- [Tutoriais ROS2 (oficial)](https://docs.ros.org/en/humble/Tutorials.html)
- [Regras da SSL](https://robocup-ssl.github.io/ssl-rules/)
- [grSim GitHub](https://github.com/RoboCup-SSL/grSim)
- [ssl-game-controller GitHub](https://github.com/RoboCup-SSL/ssl-game-controller)

---

> **Duvidas?** Pergunte no grupo de software, mas sempre tente buscar a resposta antes — leia a documentacao, explore o codigo, teste no terminal. A melhor forma de aprender e correndo atras. Ninguem nasce sabendo, e o projeto e feito para aprender juntos.
