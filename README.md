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

## Extras

- [Configuração do VS Code](docs/vscode-setup.md) — Dev Container, IntelliSense para ROS2, Ruff com auto-format ao salvar
- [Configuração no Arch Linux](docs/archlinux-setup.md) — grSim via AUR, correções para Wayland

