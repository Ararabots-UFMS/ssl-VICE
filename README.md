
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
docker build -t ssl-vice -f Dockerfile .
```
Isso gera uma imagem chamada "ssl-vice". Essa imagem é como um _sistema operacional empacotado_ com tudo pronto para rodar ROS2.

Após isso, execute:

``` bash
./scripts/vice
```
Em seguida:
```bash
echo 'export PATH="$PWD/scripts:$PATH"' >> ~/.bashrc

```
PWD é o caminho do seu diretório atual.

Após isso, você conseguirá executar os comandos:

``` bash
vice build
```
Para compilar o ROS2 com colcon dentro do container.

``` bash
vice attach
```
Para abrir um terminal dentro do container.

``` bash
vice launch
```
Mostra os launch files e roda o que você escolher

```bash
vice topics
```
Lista os tópicos e permite executar cada um deles




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

--- 
## Extras:

### Análise de Código com Ruff no VSCode
- Ruff é uma ferramenta rápida e eficiente para linting (análise de estilo) e formatação de código Python.
- Este projeto já inclui um arquivo ruff.toml com as configurações necessárias.

### Como usar o Ruff no VSCode
- Instale a extensão oficial do Ruff:
    - “Ruff” na aba de extensões do VSCode e instale a extensão de Charles Marsh.

### Correção automática:
Use o atalho padrão (Ctrl + Shift + P) e selecione "Ruff: Format Document" para formatar um arquivo inteiro.

**Linting em tempo real:** 
- O VSCode irá sublinhar automaticamente os trechos que não seguem os padrões definidos.
- Passe o mouse sobre os avisos para ver sugestões ou explicações.
