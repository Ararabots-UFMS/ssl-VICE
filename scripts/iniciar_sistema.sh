#!/bin/bash

# ==========================================
# Script de Configuração e Instalação do Sistema
# ==========================================



# Uma breve explicação: DIRETORIO_PROJETO="/home/usuario/robo" Se precisar alterar o caminho do projeto no futuro, você altera em um só lugar.
#                       pois o caminho está contido dentro de uma variável
#
#                       O comando echo imprime uma mensagem no terminal.
#
#
#                       Permissão para outra pessoa executar no terminal esse arquivo (.sh):
#                       Antes de iniciar certifique-se de que não tenha nenhuma guia ou programa aberto
#                       Abra a pasta na qual está esse arquivo, eu deixo na area de trabalho então faço:
#                       copiar o comando (Ctrl + c), colar no terminal (Ctrl + Shift + v)
#                       --> cd Desktop
#                       --> chmod +x iniciar_sistema.sh
#                       --> ./iniciar_sistema.sh
# Garante que o script pare caso ocorra algum erro
#set -e   # REMOVIDO - Para permitir verificação de erros

echo "==========================================="
echo "     Baixando e Configurando o Sistema...  "
echo "==========================================="

# 1. Atualiza os pacotes do sistema
echo "-> Atualizando listas de pacotes..."
sudo apt update

# 2. Instala dependências comuns (Git, Python, pip, venv e curl)
# Adicionado 'wget' às dependências
echo "-> Instalando dependências..."
sudo apt install -y git python3 python3-pip python3-venv curl wget

echo "-> Navegando para a Área de Trabalho..."

DESKTOP=$(xdg-user-dir DESKTOP)

# Cria a área de trabalho caso não exista
mkdir -p "$DESKTOP"

cd "$DESKTOP"

echo "-> Verificando existência da pasta Arara_Bots..."
NOME_PASTA="Arara_Bots"
if [ -d "$NOME_PASTA" ]; then
    echo "-> A pasta Arara_Bots já existe."
    cd "$NOME_PASTA"
else
    mkdir -p "$NOME_PASTA"
    cd "$NOME_PASTA"
fi

# 3. Instalação de dependências do grSim (Compiladores e bibliotecas gráficas)
echo "==========================================="
echo "-> Instalando dependências para o grSim..."
echo "==========================================="

sudo apt install -y build-essential cmake libqt5opengl5-dev qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libgl1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-all-dev

# 4. Clonagem e Compilação do grSim
echo "==========================================="
echo "-> Baixando e instalando o grSim..."
echo "==========================================="
GRSIM_OK=false

if [ -d "grSim" ]; then
    echo "-> O grSim já existe. Verificando compilação..."
    cd grSim
    # [CORRIGIDO] Verifica no local correto: bin/grSim (local real do executável)
    if [ -f "bin/grSim" ] || [ -f "build/bin/grSim" ] || [ -f "grsim" ]; then
        echo "-> grSim já compilado anteriormente."
        GRSIM_OK=true
    else
        echo "-> grSim encontrado mas não compilado."
    fi
    cd ..
else
    echo "-> Clonando grSim..."
    git clone https://github.com/RoboCup-SSL/grSim.git
fi

if [ "$GRSIM_OK" = false ]; then
    cd grSim
    echo "-> Tentando compilar grSim (pode levar vários minutos)..."
    mkdir -p build
    cd build
    
    if cmake .. && make -j$(nproc); then
        echo "-> grSim compilado com sucesso!"
        GRSIM_OK=true
    else
        echo "ERRO: Falha na compilação do grSim com Qt padrão!"
        echo "-> Baixando binário pré-compilado como alternativa..."
        cd ..

        sudo apt install -y wget curl
        wget -O grsim.AppImage https://github.com/RoboCup-SSL/grSim/releases/download/v1.0.4/grSim-1.0.4.AppImage
        
        if [ ! -f "grsim.AppImage" ]; then
            curl -L -o grsim.AppImage https://github.com/RoboCup-SSL/grSim/releases/download/v1.0.4/grSim-1.0.4.AppImage
        fi
        
        if [ -f "grsim.AppImage" ]; then
            chmod +x grsim.AppImage
            echo "-> Alternativa baixada! Execute: ./grsim.AppImage"
            GRSIM_OK=true
        else
            echo "ERRO CRÍTICO: Falha no download do AppImage"
        fi
    fi
    cd ../..
fi
#  Removido bloco duplicado que causava erro

# 5. Instalação do Árbitro (Game Controller)
# NOTA: O ssl-referee (Java/.jar) foi descontinuado e substituído pelo ssl-game-controller
# Referência: https://github.com/RoboCup-SSL/ssl-game-controller
echo "-> Baixando e configurando o Game Controller (Árbitro)..."

GC_VERSION="v3.20.2"
GC_BINARY="ssl-game-controller_${GC_VERSION}_linux_amd64"
ARBITRO_DIR="ssl-game-controller"

mkdir -p "$ARBITRO_DIR"

# Verifica se já está instalado e funcional
if [ -f "$ARBITRO_DIR/ssl-game-controller" ] && [ -x "$ARBITRO_DIR/ssl-game-controller" ]; then
    echo "-> Game Controller já instalado."
else
    echo "-> Baixando ssl-game-controller ${GC_VERSION}..."
    GC_URL="https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/${GC_VERSION}/${GC_BINARY}"
    curl -L -o "$ARBITRO_DIR/ssl-game-controller" "$GC_URL"

    # Tenta wget se curl falhou
    if [ ! -f "$ARBITRO_DIR/ssl-game-controller" ] || [ $(stat -c%s "$ARBITRO_DIR/ssl-game-controller") -lt 10000 ]; then
        echo "-> Tentando download alternativo com wget..."
        wget -O "$ARBITRO_DIR/ssl-game-controller" "$GC_URL"
    fi

    if [ -f "$ARBITRO_DIR/ssl-game-controller" ] && [ $(stat -c%s "$ARBITRO_DIR/ssl-game-controller") -gt 10000 ]; then
        chmod +x "$ARBITRO_DIR/ssl-game-controller"
        echo "-> Game Controller baixado com sucesso!"
    else
        echo "ERRO: Falha no download do Game Controller."
    fi
fi

# 6. Baixa ou atualiza o repositório do projeto ssl-VICE
echo "-> Verificando o repositório do projeto..."
NOME_PASTA="ssl-VICE"

if [ -d "$NOME_PASTA" ]; then
    echo "-> O repositório já existe localmente. Atualizando..."
    cd "$NOME_PASTA"
    git pull
else
    echo "-> Clonando o repositório do projeto..."
    git clone https://github.com/Ararabots-UFMS/ssl-VICE.git
    cd "$NOME_PASTA"
fi

# 7. Configuração do ambiente virtual Python (sempre dentro de ssl-VICE)
echo "-> Configurando ambiente virtual Python..."
python3 -m venv venv
source venv/bin/activate

# 8. Instalação das dependências do projeto

pip install --upgrade pip setuptools wheel pyyaml # Garante que o pip, setuptools e pyyaml estejam atualizados 

if [ -f "requirements.txt" ]; then
    echo "-> Instalando pacotes do requirements.txt..."
    pip install -r requirements.txt # é um arquivo tipo bloco de notas com varias bibliotecas do python ex: numpy e flask
else
    echo "-> Arquivo requirements.txt não encontrado. Pulando etapa."
fi

# Volta para a pasta Arara_Bots onde os componentes foram instalados
cd ..

# Verificação final
echo ""
echo "==========================================="
echo "        VERIFICAÇÃO FINAL                  "
echo "==========================================="

# Verifica grSim nos locais corretos (incluindo bin/grSim)
if [ -f "grSim/bin/grSim" ] || [ -f "grSim/build/bin/grSim" ] || [ -f "grSim/grsim" ] || [ -f "grSim/grsim.AppImage" ]; then
    echo "✓ grSim: INSTALADO com sucesso"
else
    echo "✗ grSim: FALHA na instalação - necessário instalar manualmente"
fi

# Verifica Game Controller (substituto do Referee)
if [ -f "ssl-game-controller/ssl-game-controller" ] && [ -x "ssl-game-controller/ssl-game-controller" ]; then
    echo "✓ Game Controller (Árbitro): INSTALADO"
else
    echo "✗ Game Controller (Árbitro): NÃO ENCONTRADO ou download incompleto"
fi

# Verifica ssl-VICE (inclui venv)
if [ -d "ssl-VICE" ] && [ -f "ssl-VICE/requirements.txt" ]; then
    echo "✓ ssl-VICE: INSTALADO"
else
    echo "✗ ssl-VICE: INSTALAÇÃO INCOMPLETA"
fi

echo "==========================================="
echo "   Processo concluído com sucesso!         "
echo "==========================================="