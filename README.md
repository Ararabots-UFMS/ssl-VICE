
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

<!-- Para aprender como instalar os programas e pacotes necessários, confira o [README dos Requisitos](./requirements/README.MD) -->

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
Em seguida, execute dentro da pasta do `ssl-VICE`:

```bash
docker-compose up --build
```

Após isso, dois serviços serão criados:
- `ssl-vice`
- `ssl-gui`

A interface gráfica (GUI) estará disponível em:
http://localhost:5173/

Até agora temos o `ssl-VICE` e o `ssl-GUI` rodando.
---
Para escutar tópicos específicos do ROS2:

Entre no container `ssl-vice`:

```bash
docker exec -it ssl-vice bash
source /root/ssl-VICE/install/setup.bash
```

E para listar e escutar tópicos, use:

```bash
ros2 topic list          # lista todos os tópicos disponíveis
ros2 topic echo /refereeTopic  # escuta um tópico de exemplo
```

Agora vamos rodar as aplicações externas
---
Para rodar o `ssl-game-controller`.
```bash
cd ssl-game-controller
```

Dentro da pasta do `ssl-game-controller`:
```bash
docker compose up --build ssl-game-controller
```


Agora vamos instalar o `grSim` e o `ssl-vision-client`
---

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