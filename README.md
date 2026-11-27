<div align="center">
<a href="https://quackfy.vercel.app/">
<img height="100" src="https://ararabots-ufms.github.io/img/arara_no_bg.png" alt="Arara">
</a>
</div>

<div align="center">
<img src="https://img.shields.io/badge/build-latest-blue">
<img src="https://img.shields.io/github/issues/Ararabots-UFMS/ssl-VICE">
</div>

## Configuração de Ambiente

**Importante:** Além do repositório `ssl-VICE`, é necessário clonar os repositórios `ssl-gui` e `ssl-game-controller` na **mesma pasta** para que o sistema funcione corretamente.

Então é recomendado criar uma pasta "ararabots" e dentro clonar os respectivos repositórios de desenvolvimento
Se for clonado repositórios posteriores clonar dentro da pasta.

Com HTTP:

```bash
mkdir ararabots
cd ararabots
git clone https://github.com/Ararabots-UFMS/ssl-gui.git # com HTTP
git clone https://github.com/Ararabots-UFMS/ssl-VICE.git # com HTTP
git clone https://github.com/RoboCup-SSL/ssl-game-controller.git # com HTTP
```

Com SSH:

```bash
mkdir ararabots
cd ararabots
git clone git@github.com:Ararabots-UFMS/ssl-gui.git
git clone git@github.com:Ararabots-UFMS/ssl-VICE.git
git clone git@github.com:RoboCup-SSL/ssl-game-controller.git
```

## Execução

### Usando o Vice CLI (recomendado para desenvolvimento)

Adicione o diretório `scripts/` ao seu PATH:

```bash
echo 'export PATH="<caminho-para-ssl-VICE>/scripts:$PATH"' >> ~/.bashrc
```

Comandos disponíveis:

| Comando                | Descrição                                      |
| ---------------------- | ---------------------------------------------- |
| `vice build`           | Compila o workspace ROS2                       |
| `vice launch`          | Seleciona e executa um launch file             |
| `vice reload`          | Recompila e relança o último launch file usado |
| `vice run <pkg> <node>`| Executa um node individual (menu interativo se sem args) |
| `vice attach`          | Abre um shell interativo no container          |
| `vice topics`          | Lista e escuta tópicos ROS2                    |
| `vice start`           | Inicia o container                             |
| `vice stop`            | Para o container                               |
| `vice restart`         | Reinicia o container                           |
| `vice destroy`         | Remove o container e a imagem completamente    |
| `vice clean`           | Remove build/, install/, log/                  |

Fluxo de desenvolvimento:

1. Edite o código no host (o repositório é montado no container via volume).
2. `Ctrl+C` no terminal do `vice launch` para encerrar os nodes.
3. `vice reload` para recompilar e relançar automaticamente.

### Usando Docker Compose

```bash
docker build -t ssl-vice -f Dockerfile .
```

Se não for sua primeira execução rode sem o _--build_

```bash
vice topics
```

Após isso, dois serviços serão criados:

- `ssl-vice`
- `ssl-gui`

A interface gráfica (GUI) estará disponível em: [http://localhost:5173](http://localhost:5173)

## Até agora temos o `ssl-VICE` e o `ssl-GUI` rodando.

Para escutar tópicos específicos do ROS2:

Entre no container `ssl-vice`:

```bash
docker exec -it ssl-vice bash
source /root/ssl-VICE/install/setup.bash
```

Ou use o Vice CLI:

```bash
vice topics
```

## Agora vamos rodar as aplicações externas

Para rodar o `ssl-game-controller`.

```bash
cd ssl-game-controller
```

Dentro da pasta do `ssl-game-controller`:

```bash
docker compose up --build ssl-game-controller
```

## Agora vamos instalar o `grSim` e o `ssl-vision-client`

Com HTTP:

```bash
cd ararabots
git clone https://github.com/RoboCup-SSL/grSim.git
git clone https://github.com/RoboCup-SSL/ssl-vision.git
```

Com SSH:

```bash
cd ararabots
git clone git@github.com:RoboCup-SSL/grSim.git
git clone git@github.com:RoboCup-SSL/ssl-vision.git
```

Agora entre na pasta do `grSim` e siga as instruções de instalação por lá.

**Importante:** Priorize a parte da instalação que cita realizar a instalação pelo Docker

Faça a mesma coisa com o `ssl-vision`.

**Usuários de Arch Linux:** Consulte o guia [Configuração no Arch Linux](docs/archlinux-setup.md) para instruções específicas, incluindo instalação do grSim via AUR e correções para Wayland.

---

## Modelo do robô e aceleração do planner

O controlador (`control_mode:=predictive`, o padrão) prevê o estado do robô no instante
em que o comando chega às rodas. Para isso usa três números do robô, medidos com
`identify_robot`:

| parâmetro | significado |
|---|---|
| `actuation_delay` (s) | do comando publicado até o robô reagir (serial, rádio, firmware) |
| `tau` (s) | constante de tempo da malha de velocidade do próprio robô |
| `max_accel` (m/s²) | aceleração que as rodas realmente entregam |

Em `launch/sim_one.py` e `launch/real_one.py` eles ficam no dicionário `ROBOT`, e
todos os parâmetros do controlador e do planner saem dele. Para medir:

    ros2 launch launch/identify_sim.py        # ou identify_real.py no campo
    ros2 run control identify_robot --ros-args -p confirm:=true -p robot_id:=0
    ros2 run control identify_robot --fit identify_robot.csv

**Invariante:** a aceleração do planner deve ficar em torno de
`0.7 × min(max_accel medido, slew_limit)`. Planejar no limite não deixa margem para
frear atrasado, e qualquer atraso chega ao alvo como overshoot.

Para voltar ao controlador anterior: `control_mode:=legacy`.

---

## Extras

- [Configuração do VS Code](docs/vscode-setup.md) — Dev Container, IntelliSense para ROS2, Ruff com auto-format ao salvar
- [Configuração no Arch Linux](docs/archlinux-setup.md) — grSim via AUR, correções para Wayland

