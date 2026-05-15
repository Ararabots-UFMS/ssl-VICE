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
set -e

echo "==========================================="
echo "     Baixando e Configurando o Sistema...  "
echo "==========================================="

# 1. Atualiza os pacotes do sistema
echo "-> Atualizando listas de pacotes..."
sudo apt update

# 2. Instala dependências comuns (Git, Python, pip, venv e curl)
echo "-> Instalando dependências..."
sudo apt install -y git python3 python3-pip python3-venv curl

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
echo "-> Instalando dependências para o grSim..."
sudo apt install -y build-essential cmake libqt5opengl5-dev qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libgl1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-all-dev

# 4. Clonagem e Compilação do grSim
echo "-> Baixando e instalando o grSim..."
if [ -d "grSim" ]; then
    echo "-> O grSim já existe. Atualizando..."
    cd grSim
    git pull
    cd .. # Volta para a pasta "Arara_Bots"
else
    git clone https://github.com/RoboCup-SSL/grSim.git
    cd grSim
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    cd ../.. # Volta para a pasta "Arara_Bots"
fi

# 5. Instalação do Árbitro (Referee)
echo "-> Baixando e configurando o Árbitro (Referee)..."
sudo apt install -y default-jre

ARBITRO_DIR="ssl-referee"
if [ -d "$ARBITRO_DIR" ]; then
    echo "-> O Árbitro já está baixado."
else
    mkdir -p "$ARBITRO_DIR"
    cd "$ARBITRO_DIR"
    
    echo "-> Baixando o executável do Árbitro..."
    curl -L -o referee.jar https://github.com/RoboCup-SSL/ssl-referee/releases/latest/download/referee.jar
    
    cd .. # Volta para a pasta "Arara_Bots"
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

# 7. Configuração do ambiente virtual Python
echo "-> Configurando ambiente virtual Python..."
python3 -m venv venv
source venv/bin/activate

# 8. Instalação das dependências do projeto

pip install --upgrade pip setuptools wheel # Garante que o pip e o setuptools estejam atualizados 

if [ -f "requirements.txt" ]; then
    echo "-> Instalando pacotes do requirements.txt..."
    pip install -r requirements.txt # é um arquivo tipo bloco de notas com varias bibliotecas do python ex: numpy e flask
else
    echo "-> Arquivo requirements.txt não encontrado. Pulando etapa."
fi

# Volta para a pasta raiz onde o script foi executado originalmente
cd ../.. 

echo "==========================================="
echo "   Processo concluído com sucesso!         "
echo "==========================================="