#!/bin/bash
# ==============================================================================
#  ararabots.sh - TUDO que a equipe precisa rodar, num arquivo so.
# ==============================================================================
#
#      ./ararabots.sh                 menu de cenarios (sobe o ambiente antes)
#      ./ararabots.sh --headless      idem, sem janela do grSim (mais estavel)
#      ./ararabots.sh preparar        so monta o ambiente e confere a cadeia
#      ./ararabots.sh cenario <nome>  monta UM cenario e deixa pronto para gravar
#      ./ararabots.sh validar [n]     roda todos os cenarios n vezes -> validacao.csv
#      ./ararabots.sh limpar          mata os nodes ROS orfaos
#      ./ararabots.sh grsim           sobe so o simulador
#      ./ararabots.sh parar           derruba tudo
#      ./ararabots.sh instalar        instalacao do zero (deps, clones, build)
#
#  MAQUINA NOVA? Rode so ./ararabots.sh - ele detecta o que falta (Docker, grSim
#  compilado, repositorios clonados, workspace ROS compilado) e OFERECE instalar
#  na hora. Nao e preciso saber que existe um subcomando 'instalar'.
#
#  O que a instalacao faz: pacotes do apt, clona os quatro repositorios irmaos
#  (ssl-VICE, ssl-gui, ssl-game-controller, grSim), compila o grSim nativo
#  (10-20 min), constroi as imagens Docker e compila o workspace ROS. Leva de 20
#  a 40 minutos na primeira vez; nas seguintes ela pula o que ja existe.
#
#  O companheiro deste arquivo e o ararabots.py, que faz a parte de dentro do
#  container: monta cenario, grava, resume e diagnostica.
#
#  ANTES DE TESTAR, DUAS COISAS QUE JA CUSTARAM HORAS:
#
#  1. No modo com janela, MANTENHA A JANELA DO grSim VISIVEL. Ele so avanca a
#     fisica quando a janela e redesenhada (glwidget.cpp:392 chama step() dentro
#     de paintGL). Minimizada ou coberta = mundo parado, e ele continua
#     reenviando o ultimo quadro: parece tudo normal e nada acontece.
#
#  2. UMA EXECUCAO NAO E UM RESULTADO. O mesmo codigo ja mandou a bola para
#     dentro do gol numa rodada e para tras na seguinte. Use a opcao 'd' do menu
#     (roda N vezes e mede a dispersao) antes de concluir qualquer coisa.
#
#  E uma terceira, descoberta medindo: esta maquina tem 2 nucleos e so a pilha
#  ROS pede ~106% deles (strategyNode 25%, visionNode 24%, controller 20%,
#  driver 20%, gameWatcher 17%). Feche o que puder antes de testar, e prefira
#  --headless. Com a maquina carregada o laco de controle nao roda na hora e o
#  robo simplesmente nao chega na bola.
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAIZ="$SCRIPT_DIR"
VICE="$RAIZ/ssl-VICE"
PY="$SCRIPT_DIR/ararabots.py"

ros_d()   { docker exec -d vice bash -c "source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }
ros_run() { docker exec vice bash -c "source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }
vivo()    { docker exec vice pgrep -f "$1" >/dev/null 2>&1; }


# ============================================================================
cmd_limpar() {
    ALVOS=(
        "ros2 launch"
        "lib/control/controller"
        "lib/control_unit/gameWatcher"
        "lib/grsim_messenger/grsim_publisher_node"
        "lib/vision/visionNode"
        "lib/new_movement/driver"
        "lib/strategy/strategyNode"
        "lib/referee/referee_node"
        "lib/manual_command/manual_node"
        "lib/hardware_messenger/hardware"
        "lib/gui_interpreter/apiNode"
        "estrategia_wrapper"
    )

    for alvo in "${ALVOS[@]}"; do
        docker exec vice pkill -f "$alvo" 2>/dev/null || true
    done
    sleep 3
    for alvo in "${ALVOS[@]}"; do
        docker exec vice pkill -9 -f "$alvo" 2>/dev/null || true
    done
    sleep 2

    restantes=$(docker exec vice ps -eo cmd 2>/dev/null | grep -cE "lib/(control|control_unit|vision|new_movement|strategy|grsim_messenger|manual_command|referee)/" || true)
    echo "nodes ROS restantes: ${restantes:-0}"
}


# ============================================================================
cmd_grsim() {
    BIN="$SCRIPT_DIR/Arara_Bots/grSim/bin/grSim"

    MODO="janela"
    ARGS=()
    if [ "${1:-}" = "--headless" ] || [ "${1:-}" = "-H" ]; then
        MODO="headless"
        ARGS=(--headless)
    fi

    # pgrep -x casa o NOME do executavel. Nao use 'pgrep -f bin/grSim': isso casa
    # qualquer linha de comando que contenha esse texto - inclusive o shell que roda
    # este script - e da falso positivo, fazendo o simulador nunca subir.
    if pgrep -x grSim >/dev/null; then
        echo "grSim ja esta rodando (para trocar de modo: pkill -x grSim)."
        return 0
    fi

    if [ ! -x "$BIN" ]; then
        echo "grSim nao esta compilado. Rode ./ararabots.sh instalar antes."
        return 1
    fi

    setsid env QT_QPA_PLATFORM=xcb DISPLAY="${DISPLAY:-:0}" \
        "$BIN" "${ARGS[@]}" >/tmp/grsim.log 2>&1 < /dev/null &

    sleep 8
    if pgrep -x grSim >/dev/null; then
        echo "grSim iniciado (modo: $MODO)."
        if [ "$MODO" = "janela" ]; then
            echo "!! Mantenha a janela do grSim VISIVEL. Minimizada ou coberta,"
            echo "!! a simulacao congela e os testes gravam dados falsos."
        fi
    else
        echo "grSim NAO subiu. Veja /tmp/grsim.log"
        return 1
    fi
}


# ============================================================================
cmd_parar() {
    echo ">> Parando nodes ROS 2 dentro do container..."
    if [ -n "$(docker ps -q -f name=^vice$)" ]; then
        docker exec vice bash -c "pkill -f 'ros2 launch'; pkill -f apiNode; pkill -f referee_node" 2>/dev/null || true
        echo "   nodes encerrados"
    else
        echo "   container vice nao esta rodando"
    fi

    echo ">> Parando o grSim..."
    if pgrep -x grSim >/dev/null; then
        pkill -x grSim && echo "   grSim encerrado"
    else
        echo "   grSim nao estava rodando"
    fi

    echo ">> Parando containers..."
    for c in ssl-gc ssl-gui vice; do
        if [ -n "$(docker ps -q -f name=^${c}$)" ]; then
            docker stop "$c" >/dev/null && echo "   $c parado"
        else
            echo "   $c ja estava parado"
        fi
    done

    echo ""
    echo "Tudo parado. Para subir de novo: ./testar_e2e.sh"
}


# ============================================================================
cmd_preparar() {
    MODO_GRSIM="janela"
    SUBIR_ESTRATEGIA=1
    for arg in "$@"; do
        case "$arg" in
            --headless)        MODO_GRSIM="headless" ;;
            --sem-estrategia)  SUBIR_ESTRATEGIA=0 ;;
            -h|--help)         uso; return 0 ;;
        esac
    done

    FALHAS=0
    passo()  { echo ""; echo "[$1/10] $2"; }
    ok()     { echo "      ✓ $1"; }
    falha()  { echo "      ✗ $1"; FALHAS=$((FALHAS+1)); }

    ros_d() {
        docker exec -d vice bash -c \
            "source /opt/ros/humble/setup.bash && \
             source /root/ssl-VICE/install/local_setup.bash && $*"
    }
    vivo()  { docker exec vice pgrep -f "$1" >/dev/null 2>&1; }

    echo "==============================================================="
    echo "  PREPARANDO AMBIENTE DE TESTES - Ararabots"
    echo "  grSim: $MODO_GRSIM"
    echo "==============================================================="

    # ---------------------------------------------------------------- 1
    passo 1 "Verificando pré-requisitos"

    # Se faltar alguma peca, OFERECE instalar em vez de so reclamar.
    #
    # Antes o script apenas dizia "rode o instalador" e saia. Quem esta montando
    # a maquina pela primeira vez nao tem por que saber que existem dois
    # comandos - o certo e ele descobrir o que falta e resolver na hora.
    faltando=""
    command -v docker >/dev/null                || faltando="$faltando docker"
    [ -x "$RAIZ/grSim/bin/grSim" ]              || faltando="$faltando grSim"
    [ -d "$VICE" ]                              || faltando="$faltando ssl-VICE"

    if [ -n "$faltando" ]; then
        echo "      falta:$faltando"
        if [ -t 0 ]; then
            read -rp "      instalar agora? (leva 20-40 min na primeira vez) [S/n] " resp
            case "${resp:-s}" in
                [Ss]*) cmd_instalar || return 1 ;;
                *)     falha "faltando:$faltando  ->  ./ararabots.sh instalar"; return 1 ;;
            esac
        else
            falha "faltando:$faltando  ->  ./ararabots.sh instalar"
            return 1
        fi
    fi

    docker info >/dev/null 2>&1 || { falha "Sem permissao no Docker (usermod -aG docker \$USER + logout)"; return 1; }
    ok "Docker"
    [ -x "$RAIZ/grSim/bin/grSim" ] || { falha "grSim nao compilado - a instalacao nao terminou"; return 1; }
    ok "grSim compilado"
    [ -d "$VICE" ] || { falha "ssl-VICE nao encontrado em $RAIZ"; return 1; }
    ok "ssl-VICE presente"

    # O workspace precisa estar COMPILADO, nao so clonado. Sem isto o
    # 'ros2 run strategy strategyNode' falha com "package not found" e o
    # sintoma aparece cinco passos depois, como estrategia que nao sobe.
    if [ -d "$VICE" ] && [ -n "$(docker ps -q -f name=^vice$)" ]; then
        if ! docker exec vice test -d /root/ssl-VICE/install/strategy 2>/dev/null; then
            echo "      workspace ROS nao compilado - compilando (alguns minutos)..."
            docker exec vice bash -c "cd /root/ssl-VICE && source /opt/ros/humble/setup.bash && colcon build" \
                >/dev/null 2>&1 && ok "workspace compilado" || falha "colcon build falhou"
        fi
    fi

    # ---------------------------------------------------------------- 2
    passo 2 "Container ROS 2 (vice)"
    if [ -z "$(docker ps -q -f name=^vice$)" ]; then
        "$VICE/scripts/vice" start >/dev/null 2>&1 || "$VICE/scripts/vice" build
        sleep 3
    fi
    if [ -n "$(docker ps -q -f name=^vice$)" ]; then ok "no ar"; else falha "não subiu"; fi

    # ---------------------------------------------------------------- 3
    passo 3 "Árbitro (ssl-game-controller) - sempre recriado"
    docker rm -f ssl-gc >/dev/null 2>&1 || true
    docker run -d --name ssl-gc --network host \
        robocupssl/ssl-game-controller:latest \
        -address :8081 \
        -publishAddress 224.5.23.1:10003 \
        -visionAddress 224.5.23.2:10020 >/dev/null 2>&1
    sleep 5
    if curl -sf -o /dev/null http://localhost:8081/; then
        ok "UI em http://localhost:8081 (publica em 224.5.23.1:10003)"
    else
        falha "não responde na porta 8081"
    fi

    # ---------------------------------------------------------------- 4
    passo 4 "Simulador (grSim)"
    # O grSim REESCREVE ~/.grsim.xml ao sair (configwidget.cpp:215), e o seletor
    # "Division" no XML NAO e respeitado na leitura - ele volta a usar a Division A.
    # Com isso o campo vira 12000 mm em vez de 9000, o gol vai para x=+/-6000, o
    # limiar de chute sobe junto, e a bola em x=2500 deixa de estar no terco de
    # ataque. O chute simplesmente para de funcionar, sem nenhuma mensagem de erro.
    #
    # Foi a causa de o mesmo teste passar numa execucao e falhar na seguinte.
    #
    # Solucao: alinhar as medidas da Division A com as da B (linhas 30 e 33 do
    # arquivo), com o grSim PARADO. Assim, qualquer que seja a divisao que ele
    # resolva usar, o campo tem 9000 x 6000.
    if [ -f ~/.grsim.xml ]; then
        comp="$(sed -n '30p' ~/.grsim.xml | tr -d ' \t')"
        if [ "$comp" != "9,000000" ]; then
            pgrep -x grSim >/dev/null && { echo "      (parando o grSim para corrigir o campo)"; pkill -x grSim; sleep 3; }
            sed -i '30s/.*/\t\t\t\t\t9,000000/; 33s/.*/\t\t\t\t\t6,000000/' ~/.grsim.xml
            ok "campo forcado para 9000 x 6000 (Division B)"
        fi
    fi

    if pgrep -x grSim >/dev/null; then
        atual="janela"; ps -o args= -C grSim | grep -q headless && atual="headless"
        if [ "$atual" != "$MODO_GRSIM" ]; then
            echo "      (rodando em '$atual', trocando para '$MODO_GRSIM')"
            pkill -x grSim; sleep 3
        fi
    fi
    if ! pgrep -x grSim >/dev/null; then
        if [ "$MODO_GRSIM" = "headless" ]; then
            cmd_grsim --headless >/dev/null
        else
            cmd_grsim >/dev/null
        fi
    fi
    if pgrep -x grSim >/dev/null; then
        ok "no ar (modo $MODO_GRSIM)"
        [ "$MODO_GRSIM" = "janela" ] && echo "      ! mantenha a janela VISÍVEL: coberta ou minimizada, a física congela"
    else
        falha "não subiu (veja /tmp/grsim.log)"
    fi

    # ---------------------------------------------------------------- 5
    passo 5 "Nodes de simulação (sim_one.py)"
    # Limpa nodes orfaos de execucoes anteriores antes de subir.
    # Sem isso eles se acumulam: ja medimos nove controllers simultaneos brigando
    # por /commandTopic, com load average 80 e a visao caindo de 45 Hz para 3 Hz.
    cmd_limpar >/dev/null 2>&1 || true
    if ! vivo "launch/sim_one.py"; then
        ros_d "ros2 launch /root/ssl-VICE/launch/sim_one.py > /tmp/sim.log 2>&1"
        sleep 12
    fi
    vivo "launch/sim_one.py" && ok "no ar" || falha "não subiu (docker exec vice cat /tmp/sim.log)"
    echo "      (o node 'hardware' morre por falta de /dev/ttyUSB0 - isso é normal em simulação)"

    # ---------------------------------------------------------------- 6
    passo 6 "Removendo o manual_command"
    # Em laco: o launch pode demorar a criar o processo, e enquanto ele viver estara
    # publicando em /commandTopic a 60 Hz, movendo os robos por conta propria.
    for _ in $(seq 1 8); do
        docker exec vice pkill -f "manual_command/manual_node" 2>/dev/null || true
        sleep 1
    done
    ok "fora de cena (ele disputaria /commandTopic com o controlador)"

    # ---------------------------------------------------------------- 7
    passo 7 "new_movement/driver"
    if ! vivo "new_movement/driver"; then
        ros_d "ros2 run new_movement driver > /tmp/driver.log 2>&1"
        sleep 8
    fi
    vivo "new_movement/driver" && ok "no ar (fornece strategy_command e update_obstacles)" \
                               || falha "não subiu (docker exec vice cat /tmp/driver.log)"

    # ---------------------------------------------------------------- 8
    passo 8 "referee_node"
    if ! vivo "referee/referee_node"; then
        ros_d "ros2 run referee referee_node > /tmp/ref.log 2>&1"
        sleep 6
    fi
    vivo "referee/referee_node" && ok "escutando 224.5.23.1:10003" \
                                || falha "não subiu (docker exec vice cat /tmp/ref.log)"

    # ---------------------------------------------------------------- 9
    passo 9 "strategyNode"
    if [ "$SUBIR_ESTRATEGIA" = "1" ]; then
        # SEMPRE recriado, nunca reaproveitado.
        #
        # Os nodes da arvore resolvem a cor do time chamando 'get_game_config' na
        # inicializacao. Se o strategyNode subiu antes do gameWatcher estar pronto,
        # essa chamada fica pendurada e o node fica preso em RUNNING para sempre - a
        # arvore devolve acao None e NENHUM comando e gerado. O sintoma e mudo: os
        # robos simplesmente nao se mexem.
        docker exec vice pkill -f "strategy/strategyNode" 2>/dev/null || true
        sleep 2
        ros_d "ros2 run strategy strategyNode > /tmp/strategy.log 2>&1"
        sleep 12
        vivo "strategy/strategyNode" && ok "no ar" \
                                     || falha "não subiu (docker exec vice cat /tmp/strategy.log)"
    else
        echo "      (pulado por --sem-estrategia)"
    fi

    # ---------------------------------------------------------------- 10
    passo 10 "Verificando a cadeia"
    docker cp "$PY" vice:/tmp/ararabots.py >/dev/null 2>&1
    # Se o campo nao vier com 9000 mm, a estrategia calcula gol e limiar de chute
    # errados e nada funciona - por isso isto e verificado explicitamente.
        docker exec vice bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /root/ssl-VICE/install/local_setup.bash && \
         python3 /tmp/ararabots.py cadeia" 2>/dev/null || falha "verificação não rodou"

    echo ""
    echo "==============================================================="
    if [ "$FALHAS" -eq 0 ]; then
        echo "  AMBIENTE PRONTO"
        echo ""
        echo "  Agora rode:  ./ararabots.sh"
        echo "  (o menu de cenarios; ele ja faz esta montagem sozinho)"
    else
        echo "  $FALHAS PROBLEMA(S) - veja as linhas com ✗ acima"
    fi
    echo "  Para derrubar tudo:  ./ararabots.sh parar"
    echo "==============================================================="
    return $([ "$FALHAS" -eq 0 ] && echo 0 || echo 1)
}


# ============================================================================
cmd_cenario() {
    CENARIO="${1:?informe o cenario}"
    PERFIL="${2:-${CAMPO:-codigo}}"

    ros_d()   { docker exec -d vice bash -c "source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }
    ros_run() { docker exec vice bash -c "source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }

    # --------------------------------------------------------------- pre-checagem
    #
    # Antes de qualquer coisa, conferir que as tres pecas de fora do ROS estao de pe.
    # Sem isto, a falha aparece 40 s depois como um traceback de socket no meio do
    # teste, e nao ha como saber o que faltou. Ja aconteceu: o Docker reiniciou, o
    # ssl-gc morreu junto, e duas repeticoes inteiras foram perdidas.
    faltou=0
    if ! docker ps --format '{{.Names}}' | grep -qx vice; then
        echo "   XX o container 'vice' nao esta no ar   ->  ./ararabots.sh preparar"
        faltou=1
    fi
    if ! curl -sf -o /dev/null http://localhost:8081/ 2>/dev/null; then
        echo "   XX o arbitro (ssl-gc) nao responde na porta 8081   ->  ./ararabots.sh preparar"
        faltou=1
    fi
    if ! pgrep -x grSim >/dev/null; then
        echo "   XX o grSim nao esta rodando   ->  ./ararabots.sh preparar"
        faltou=1
    fi
    [ "$faltou" = "1" ] && { echo "   (o ./ararabots.sh preparar reconstroi tudo isso)"; return 1; }

    # Os scripts auxiliares vao junto sempre - assim uma correcao neles vale na hora,
    # sem depender de lembrar de copiar.
    docker cp "$PY" vice:/tmp/ararabots.py >/dev/null 2>&1

    # ---------------------------------------------------------------- 1
    ros_run "CAMPO=$PERFIL python3 /tmp/ararabots.py posicionar $CENARIO" || return 1

    # ---------------------------------------------------------------- 2 e 3
    cmd_limpar >/dev/null 2>&1
    ros_d "ros2 launch /root/ssl-VICE/launch/sim_one.py > /tmp/sim.log 2>&1"

    # ---------------------------------------------------------------- 4
    for _ in $(seq 1 14); do
        docker exec vice pkill -f "manual_command/manual_node" 2>/dev/null || true
        sleep 1
    done

    ros_d "ros2 run new_movement driver > /tmp/driver.log 2>&1";  sleep 7
    ros_d "ros2 run referee referee_node > /tmp/ref.log 2>&1";    sleep 5
    ros_d "ros2 run strategy strategyNode > /tmp/strategy.log 2>&1"; sleep 8

    # ---------------------------------------------------------------- 5
    if ! ros_run "python3 /tmp/ararabots.py esperar 60"; then
        echo "   !! a estrategia nao comandou em 60s - o resultado NAO vale"
        return 2
    fi
    return 0
}


# ============================================================================
cmd_validar() {
    N="${1:-3}"
    shift 2>/dev/null || true
    LISTA=("$@")
    SAIDA="$SCRIPT_DIR/validacao.csv"

    # Trava simples contra duas instancias simultaneas.
    TRAVA="/tmp/validar_cenarios.lock"
    if ! mkdir "$TRAVA" 2>/dev/null; then
        echo "XX ja existe uma validacao rodando (trava em $TRAVA)."
        echo "   Se tiver certeza de que nao ha, remova com: rmdir $TRAVA"
        return 1
    fi
    trap 'rmdir "$TRAVA" 2>/dev/null' EXIT

    ros_run() { docker exec vice bash -c "source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }

    mapfile -t CENARIOS < <(python3 "$PY" listar | cut -d"|" -f1)

    [ ${#LISTA[@]} -gt 0 ] && CENARIOS=("${LISTA[@]}")
    [ -f "$SAIDA" ] || echo "cenario,rep,gol,chute,bola_ini_x,bola_ini_y,bola_fim_x,bola_fim_y,andou,load" > "$SAIDA"
    for c in "${CENARIOS[@]}"; do
        echo "############ $c"
        for i in $(seq 1 "$N"); do
            cmd_cenario "$c" >/dev/null 2>&1
            st=$?
            [ $st -eq 1 ] && { echo "  rep$i: cenario nao montou"; echo "$c,$i,ERRO,,,,,,," >> "$SAIDA"; continue; }
            L=$(cut -d' ' -f1 /proc/loadavg)
            OUT=$(ros_run "BRANCH=val_${c}_$i python3 /tmp/ararabots.py rodar $c 25" 2>&1)
            GOL=$(echo "$OUT" | grep -oE "GOL A FAVOR|GOL CONTRA|sem gol" | head -1)
            KICK=$(echo "$OUT" | grep -oE "CHUTE: (ATIVADO|nao ativado)" | head -1 | sed 's/CHUTE: //')
            BOLA=$(echo "$OUT" | grep -oE "BOLA: \(-?[0-9]+,-?[0-9]+\) -> \(-?[0-9]+,-?[0-9]+\)  andou +[0-9]+" | head -1)
            BI=$(echo "$BOLA" | sed -E 's/BOLA: \((-?[0-9]+),(-?[0-9]+)\).*/\1,\2/')
            BF=$(echo "$BOLA" | sed -E 's/.*-> \((-?[0-9]+),(-?[0-9]+)\).*/\1,\2/')
            AN=$(echo "$BOLA" | grep -oE "andou +[0-9]+" | grep -oE "[0-9]+")
            echo "  rep$i: ${GOL:-?}  chute=${KICK:-?}  ${BOLA:-sem leitura}  (load $L)"
            echo "$c,$i,${GOL:-?},${KICK:-?},${BI:-,},${BF:-,},${AN:-},$L" >> "$SAIDA"
        done
    done
    echo
    echo "CSV em $SAIDA"
}


# ============================================================================
cmd_menu() {
    DURACAO=25
    MODO="janela"
    MONTAR=1
    for arg in "$@"; do
        case "$arg" in
            --headless) MODO="headless" ;;
            --so-menu)  MONTAR=0 ;;
            -h|--help)         uso; return 0 ;;
        esac
    done

    ros_run() {
        docker exec vice bash -c \
            "source /opt/ros/humble/setup.bash && \
             source /root/ssl-VICE/install/local_setup.bash && $*"
    }

    # ---------------------------------------------------------------- montagem
    if [ "$MONTAR" = "1" ]; then
        if [ "$MODO" = "headless" ]; then
            cmd_preparar --headless || {
                echo; echo "XX o ambiente nao subiu - veja as linhas com x acima"; return 1; }
        else
            cmd_preparar || {
                echo; echo "XX o ambiente nao subiu - veja as linhas com x acima"; return 1; }
        fi
    fi

    # ------------------------------------------------------- lista de cenarios
    mapfile -t CENARIOS < <(python3 "$PY" listar)
    [ ${#CENARIOS[@]} -eq 0 ] && { echo "XX nao consegui ler os cenarios"; return 1; }

    rodar_um() {
        local nome="$1"
        echo
        echo "=============================================================="
        echo "  CENARIO: $nome   (${DURACAO}s)"
        echo "=============================================================="
        # A receita de montagem vive so em cmd_cenario. Ela ja esteve
        # copiada em tres scripts e eles divergiram: uma correcao entrava num deles
        # e o teste do outro falhava sem explicacao nenhuma.
        cmd_cenario "$nome"
        local st=$?
        [ $st -eq 1 ] && { echo "   XX o cenario nao montou"; return 1; }
        [ $st -eq 2 ] && echo "   !! a estrategia nao comandou - o resultado abaixo NAO vale"
        ros_run "python3 /tmp/ararabots.py rodar $nome $DURACAO"
    }

    rodar_dispersao() {
        local nome="$1" n="$2"
        echo
        echo "=============================================================="
        echo "  DISPERSAO: $nome  x$n"
        echo "=============================================================="
        for i in $(seq 1 "$n"); do
            echo; echo "----- repeticao $i/$n -----"
            cmd_cenario "$nome" >/dev/null
            local st=$?
            [ $st -eq 1 ] && { echo "   XX o cenario nao montou"; continue; }
            [ $st -eq 2 ] && echo "   !! a estrategia nao comandou - resultado suspeito"
            ros_run "BRANCH=disp_$i python3 /tmp/ararabots.py rodar $nome $DURACAO" \
                | grep -E "GOL|sem gol|CHUTE|BOLA:" | sed 's/^/   /'
        done
        echo
        rm -rf "$SCRIPT_DIR/saida" && docker cp vice:/tmp/cenarios_freekick "$SCRIPT_DIR/saida" >/dev/null 2>&1
        python3 "$PY" resumo disp
    }

    # ------------------------------------------------------------------- menu
    while true; do
        echo
        echo "=============================================================="
        # Mostra o modo REAL do grSim, nao o que foi pedido na linha de comando.
        # Com --so-menu o simulador pode ja estar rodando no outro modo, e o rotulo
        # errado faz voce achar que esta assistindo quando a janela nem existe.
        if pgrep -x grSim >/dev/null; then
            real="janela"; ps -o args= -C grSim 2>/dev/null | grep -q headless && real="headless"
        else
            real="PARADO"
        fi
        echo "  CENARIOS  (duracao atual: ${DURACAO}s   grSim: $real)"
        echo "=============================================================="
        for i in "${!CENARIOS[@]}"; do
            printf "  %2d) %-26s %s\n" $((i+1)) "${CENARIOS[$i]%%|*}" "${CENARIOS[$i]#*|}"
        done
        echo
        echo "   t) rodar TODOS em sequencia"
        echo "   d) rodar UM cenario N vezes e medir a DISPERSAO  <- e esta que vale"
            echo "   s) mudar a duracao da gravacao"
        echo "   q) sair"
        echo
        read -rp "escolha: " opcao

        case "$opcao" in
            q|Q) break ;;
            s|S) read -rp "duracao em segundos: " nova
                 [[ "$nova" =~ ^[0-9]+$ ]] && DURACAO="$nova" || echo "   valor invalido" ;;
            t|T) for linha in "${CENARIOS[@]}"; do rodar_um "${linha%%|*}"; done
                 read -rp "[enter] para voltar ao menu " _ ;;
            d|D) read -rp "numero do cenario: " num
                 read -rp "quantas repeticoes: " n
                 if [[ "$num" =~ ^[0-9]+$ ]] && [ "$num" -ge 1 ] && [ "$num" -le ${#CENARIOS[@]} ] \
                    && [[ "$n" =~ ^[0-9]+$ ]]; then
                     rodar_dispersao "${CENARIOS[$((num-1))]%%|*}" "$n"
                 else
                     echo "   valores invalidos"
                 fi
                 read -rp "[enter] para voltar ao menu " _ ;;
                *)   if [[ "$opcao" =~ ^[0-9]+$ ]] && [ "$opcao" -ge 1 ] && [ "$opcao" -le ${#CENARIOS[@]} ]; then
                     rodar_um "${CENARIOS[$((opcao-1))]%%|*}"
                     read -rp "[enter] para voltar ao menu " _
                 else
                     echo "   opcao invalida"
                 fi ;;
        esac
    done
    echo
    echo "Para derrubar tudo:  ./ararabots.sh parar"
}


# ============================================================================
cmd_instalar() {
    # O trap e LOCAL a esta funcao: sem o 'trap - ERR' no fim ele continuaria
    # valendo para o resto do script, e um erro banal em outro subcomando
    # abortaria tudo com uma mensagem de instalacao sem sentido.
    trap 'echo ""; echo "XX ERRO na linha $LINENO. Abortando a instalacao."; trap - ERR; return 1' ERR

    titulo() {
        echo ""
        echo "==============================================================="
        echo "  $1"
        echo "==============================================================="
    }

    # Clona o repositorio se nao existir; se existir, apenas atualiza.
    clonar_ou_atualizar() {
        local url="$1" dir="$2"
        if [ -d "$RAIZ/$dir/.git" ]; then
            echo "-> $dir ja existe, atualizando..."
            git -C "$RAIZ/$dir" pull --ff-only || echo "   (aviso: pull falhou, seguindo com a versao local)"
        else
            echo "-> Clonando $dir..."
            git clone "$url" "$RAIZ/$dir"
        fi
    }

    # ------------------------------------------------------------------------------
    titulo "1/6  Verificando o sistema"
    # ------------------------------------------------------------------------------
    if ! command -v apt >/dev/null; then
        echo "XX Este script assume Ubuntu/Debian (apt nao encontrado)."
        return 1
    fi
    echo "-> $(. /etc/os-release && echo "$PRETTY_NAME")"

    if ! command -v docker >/dev/null; then
        echo "-> Docker nao encontrado. Instalando..."
        sudo apt update
        sudo apt install -y docker.io docker-compose-v2
        sudo systemctl enable --now docker
        sudo usermod -aG docker "$USER"
        echo ""
        echo "!! Voce foi adicionado ao grupo 'docker'. FACA LOGOUT E LOGIN"
        echo "!! (ou rode 'newgrp docker') e execute este script novamente."
        return 0
    fi
    echo "-> Docker $(docker --version | grep -oP '\d+\.\d+\.\d+' | head -1) OK"

    if ! docker info >/dev/null 2>&1; then
        echo "XX Sem permissao para falar com o Docker."
        echo "   Rode: sudo usermod -aG docker \$USER  e faca logout/login."
        return 1
    fi

    mkdir -p "$RAIZ"

    # ------------------------------------------------------------------------------
    titulo "2/6  Dependencias de compilacao do grSim"
    # ------------------------------------------------------------------------------
    # Lista oficial do INSTALL.md do grSim. pkg-config e libglu1-mesa-dev sao
    # obrigatorios - sem eles o cmake falha.
    # VarTypes NAO entra aqui: o CMakeLists do grSim baixa e compila sozinho.
    echo "-> Instalando pacotes (pode pedir sua senha)..."
    sudo apt update
    sudo apt install -y \
        git build-essential cmake pkg-config \
        qtbase5-dev libqt5opengl5-dev \
        libgl1-mesa-dev libglu1-mesa-dev \
        libprotobuf-dev protobuf-compiler \
        libode-dev libboost-dev

    # ------------------------------------------------------------------------------
    titulo "3/6  Baixando os repositorios"
    # ------------------------------------------------------------------------------
    clonar_ou_atualizar https://github.com/Ararabots-UFMS/ssl-VICE.git          ssl-VICE
    clonar_ou_atualizar https://github.com/Ararabots-UFMS/ssl-gui.git           ssl-gui
    clonar_ou_atualizar https://github.com/RoboCup-SSL/ssl-game-controller.git  ssl-game-controller
    clonar_ou_atualizar https://github.com/RoboCup-SSL/grSim.git                grSim

    # ------------------------------------------------------------------------------
    titulo "4/6  Compilando o grSim (nativo)"
    # ------------------------------------------------------------------------------
    # Nativo em vez de Docker: o grSim e OpenGL pesado e repassar X11 para container
    # costuma cair em software rendering, derrubando o FPS da simulacao.
    if [ -x "$RAIZ/grSim/bin/grSim" ]; then
        echo "-> grSim ja compilado. Pulando."
        echo "   (para recompilar: rm -rf '$RAIZ/grSim/build')"
    else
        echo "-> Compilando (10-20 min; o VarTypes e baixado automaticamente)..."
        mkdir -p "$RAIZ/grSim/build"
        cd "$RAIZ/grSim/build"
        cmake ..
        make -j"$(nproc)"
        cd "$SCRIPT_DIR"
    fi
    echo "-> Executavel: $RAIZ/grSim/bin/grSim"

    # ------------------------------------------------------------------------------
    titulo "5/6  Construindo as imagens Docker"
    # ------------------------------------------------------------------------------
    # ssl-VICE: o CLI 'vice' cria a imagem (base ros:humble-ros-base) e o container.
    echo "-> ssl-VICE + workspace ROS 2..."
    "$RAIZ/ssl-VICE/scripts/vice" build

    # ssl-gui: build do Vite/node20. O npm install as vezes cai com ECONNRESET
    # por instabilidade de rede, entao tentamos ate 3 vezes.
    echo ""
    echo "-> ssl-gui (interface web)..."
    cd "$RAIZ/ssl-VICE"
    for tentativa in 1 2 3; do
        if docker compose build ssl-gui; then
            break
        fi
        if [ "$tentativa" = 3 ]; then
            echo "XX Falhou 3 vezes. Verifique sua conexao e rode de novo."
            return 1
        fi
        echo "   Tentativa $tentativa falhou (comum: ECONNRESET no npm). Repetindo..."
        sleep 5
    done
    cd "$SCRIPT_DIR"

    # ssl-game-controller: imagem oficial pronta, evita um build Go+frontend longo.
    echo ""
    echo "-> ssl-game-controller (imagem oficial)..."
    docker pull robocupssl/ssl-game-controller:latest

    # ------------------------------------------------------------------------------
    titulo "6/6  Pronto"
    # ------------------------------------------------------------------------------
    echo ""
    echo "    Tudo instalado. Para validar:"
    echo ""
    echo "        ./ararabots.sh preparar     sobe o sistema e confere cada elo"
    echo "        ./ararabots.sh              menu de cenarios"
    echo ""
    echo "    Enderecos quando o sistema estiver no ar:"
    echo "        http://localhost:5173   interface (ssl-gui)"
    echo "        http://localhost:8081   arbitro (ssl-game-controller)"
    echo "        http://localhost:5000   API do apiNode"
    echo ""
    trap - ERR
}


# ==============================================================================
#  Despachante
# ==============================================================================
uso() { sed -n '2,40p' "$0"; }

case "${1:-menu}" in
    instalar)  shift; cmd_instalar "$@" ;;
    preparar)  shift; cmd_preparar "$@" ;;
    cenario)   shift; cmd_cenario  "$@" ;;
    validar)   shift; cmd_validar  "$@" ;;
    limpar)    shift; cmd_limpar   "$@" ;;
    grsim)     shift; cmd_grsim    "$@" ;;
    parar)     shift; cmd_parar    "$@" ;;
    menu)      cmd_menu ;;
    -h|--help) uso ;;
    # sem subcomando reconhecido: e o menu, e os argumentos sao dele
    *)         cmd_menu "$@" ;;
esac
