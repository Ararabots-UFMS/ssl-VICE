# Configuracao do Ambiente no Arch Linux

Guia de instalacao e configuracao do ssl-VICE e ferramentas externas no Arch Linux.

## Pre-requisitos

### Docker

```bash
sudo pacman -S docker docker-compose
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
```

Reinicie a sessao (logout/login) para o grupo `docker` ter efeito.

### Vice CLI

Adicione o diretorio `scripts/` ao seu PATH para usar o comando `vice` globalmente:

```bash
echo 'export PATH="<caminho-para-ssl-VICE>/scripts:$PATH"' >> ~/.bashrc
```

Comandos disponiveis:

| Comando       | Descricao                                      |
| ------------- | ---------------------------------------------- |
| `vice build`  | Compila o workspace ROS2                       |
| `vice launch` | Seleciona e executa um launch file             |
| `vice reload` | Recompila e relanca o ultimo launch file usado |
| `vice attach` | Abre um shell interativo no container          |
| `vice topics` | Lista e escuta topicos ROS2                    |

### Fluxo de desenvolvimento

1. Edite o codigo no host (o repositorio e montado no container via volume).
2. `Ctrl+C` no terminal do `vice launch` para encerrar os nodes.
3. `vice reload` para recompilar e relancar automaticamente.

## grSim

### Instalacao via AUR

```bash
yay -S grsim-git
```

Se a build falhar com erro de `cmake_minimum_required` no `vartypes`, use:

```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 yay -S grsim-git
```

### Problemas de renderizacao no Wayland (Hyprland, Sway, etc.)

O menu lateral do grSim pode nao atualizar corretamente em compositors Wayland. Para corrigir, force o uso do XWayland:

```bash
QT_QPA_PLATFORM=xcb grSim
```

Para nao precisar digitar toda vez, crie um alias:

**Bash:**

```bash
echo 'alias grsim="QT_QPA_PLATFORM=xcb grSim -style Fusion"' >> ~/.bashrc
```

## ssl-game-controller

```bash
cd ararabots/ssl-game-controller
docker compose up --build ssl-game-controller
```

## ssl-vision

Siga as instrucoes de instalacao no repositorio:

```bash
cd ararabots
git clone https://github.com/RoboCup-SSL/ssl-vision.git
```
