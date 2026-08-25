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
#      ./ararabots.sh ajustes on|off|status [tracker|filtro|tudo]
#                                     correcoes fora de src/strategy/
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
#  ONDE ELE MORA: em ssl-VICE/docs/. A pasta RAIZ (a que tem os repositorios
#  irmaos ssl-VICE, ssl-gui, ssl-game-controller e grSim, como o README exige) e
#  descoberta subindo a arvore - ver _descobrir_raiz. Nao ha caminho fixo, entao
#  mover este arquivo de lugar nao quebra nada.
#
#  POR QUE ELE EXISTE, se ja ha o Game Controller e a GUI: o arbitro manda
#  comandos mas nao POSICIONA bola e robos, e a GUI e para humanos olharem. Um
#  teste precisa de posicao exata, sequencia de arbitro exata, gravacao e
#  repeticao - e nada disso da para fazer clicando. Este script nao substitui
#  nenhum dos dois: ele usa os dois.
#
#  ELE NAO MEXE NO SISTEMA. Nao cria node de estrategia, nao substitui a visao,
#  nao altera codigo do time. O unico node que ele cria e um GRAVADOR passivo
#  ('gravador_cenarios'), que so assina /visionTopic e /commandTopic para anotar
#  o que aconteceu. A bola e os robos sao posicionados por UDP direto no grSim
#  (protocolo grSim_Replacement, porta 20011), fora do ROS.
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

# A RAIZ e a pasta que contem os repositorios IRMAOS (ssl-VICE, ssl-gui,
# ssl-game-controller, grSim) - o layout que o README do ssl-VICE exige, porque
# o docker-compose.yml referencia ../ssl-gui.
#
# Descobrimos essa pasta subindo a arvore, em vez de fixar o caminho: este
# script ja morou na pasta de trabalho (RAIZ = ao lado dele) e hoje mora em
# ssl-VICE/docs/ (RAIZ = dois niveis acima). Fixar o caminho quebrou uma vez -
# com RAIZ=SCRIPT_DIR dentro de docs/, o VICE apontava para
# ssl-VICE/docs/ssl-VICE, que nao existe, e todo comando falhava sem dizer
# porque. Assim funciona das duas formas, e de qualquer outra que apareca.
_descobrir_raiz() {
    local d="$SCRIPT_DIR"
    for _ in 1 2 3 4 5; do
        # a raiz e quem tem ssl-VICE como filho
        [ -d "$d/ssl-VICE/scripts" ] && { echo "$d"; return; }
        # ou estamos DENTRO do ssl-VICE: a raiz e o pai dele
        [ -f "$d/scripts/vice" ] && { dirname "$d"; return; }
        d="$(dirname "$d")"
        [ "$d" = "/" ] && break
    done
    echo "$SCRIPT_DIR"      # ultimo recurso: instalacao do zero cria tudo aqui
}

RAIZ="$(_descobrir_raiz)"
VICE="$RAIZ/ssl-VICE"
PY="$SCRIPT_DIR/ararabots.py"

# DESCOBERTA DO DDS EM LOOPBACK.
#
# ROS_LOCALHOST_ONLY=1 restringe a descoberta do DDS a interface de loopback.
#
# POR QUE: com o padrao (0) a descoberta sai por TODAS as interfaces, inclusive
# o Wi-Fi. Todos os nodes rodam nesta mesma maquina, entao isso nao adiciona
# nada e adiciona superficie de falha - o HANDOVER §15.7 ja registra sockets
# multicast presos em interfaces antigas depois que o Wi-Fi reconectou.
#
# SINTOMA QUE ISTO CORRIGE, medido: driver e referee_node VIVOS, consumindo CPU,
# com os logs limpos, e um node recem-criado enxergando ZERO publicadores nos
# topicos deles - estavel por 12 s, ou seja nao era atraso de descoberta. O
# gravador nasce, nao descobre /visionTopic e grava o nada: e o
# "XX A BOLA NAO CHEGOU AO LUGAR PEDIDO / nenhuma leitura".
#
# Nao afeta a visao do grSim (224.5.23.2:10020) nem o arbitro
# (224.5.23.1:10003): esses sao sockets da aplicacao, nao do DDS.
DDS_ENV="export ROS_LOCALHOST_ONLY=1 &&"
ros_d()   { docker exec -d vice bash -c "$DDS_ENV source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }
ros_run() { docker exec vice bash -c "$DDS_ENV source /opt/ros/humble/setup.bash && source /root/ssl-VICE/install/local_setup.bash && $*"; }
vivo()    { docker exec vice pgrep -f "$1" >/dev/null 2>&1; }

# Espera ATIVA: repete o teste ate passar, ou desiste no teto.
#
# Toda espera fixa deste script foi trocada por uma destas. Os numeros antigos
# (sleep 12, sleep 8, sleep 6...) tinham sido escolhidos para o pior caso e eram
# pagos inteiros em toda execucao, mesmo quando o node ficava pronto em um
# segundo. Aqui o teto continua existindo - so nao e mais o custo padrao.
# O grSim MORRE sozinho sob carga. Ja aconteceu no meio de um lote, e o
# resultado e traicoeiro: o teleporte vai para um socket morto, a visao para de
# atualizar e a gravacao registra a bola parada ou em (0,0) - que sao lidos como
# "a jogada falhou" quando na verdade nao houve jogada nenhuma. Numa das medicoes
# apareceram ate robos de outro cenario, porque o simulador tinha reiniciado com
# as posicoes padrao.
#
# Por isso todo resultado passa por aqui: se o simulador nao sobreviveu, o
# numero NAO vale e o teste diz isso em vez de contar como falha da estrategia.
grsim_sobreviveu() {
    if pgrep -x grSim >/dev/null; then
        # Em modo janela a fisica so avanca quando a janela e redesenhada
        # (glwidget.cpp:392 chama step() dentro de paintGL). Coberta ou
        # minimizada = mundo parado, e ele segue reenviando o ultimo quadro:
        # parece que a estrategia quebrou, quando o simulador e que esta imovel.
        if ! ps -o args= -C grSim 2>/dev/null | grep -q headless; then
            echo "   ! grSim em modo JANELA - se ela nao estiver visivel, a fisica"
            echo "     esta congelada e este resultado nao vale nada."
        fi
        return 0
    fi
    echo "   !! O grSim MORREU durante a execucao - este resultado NAO vale."
    echo "      (acontece sob carga alta; veja /tmp/grsim.log)"
    return 1
}

esperar_por() {          # esperar_por <teto_s> <descricao> <comando...>
    local teto="$1" desc="$2"; shift 2
    local ini=$SECONDS
    while [ $((SECONDS - ini)) -lt "$teto" ]; do
        if "$@" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.3
    done
    echo "      ! tempo esgotado esperando: $desc (${teto}s)"
    return 1
}


# ============================================================================
# PORTAO DE MEDICAO: o teste se RECUSA a rodar num ambiente que nao vale.
#
# Por que existe: durante um dia inteiro de medicoes, tudo que este script fazia
# era AVISAR e seguir em frente. Resultado: lotes gravados com o grSim em camera
# lenta, com a visao muda, com o kp que nunca foi aplicado, e com a maquina tao
# carregada que o laco de controle do driver entregava 19 Hz dos 100 que ele
# pede. Cada um desses produziu numeros que pareciam resultado da estrategia.
#
# O criterio NAO e "load", que e proxy ruim. E a taxa do /control_command, que
# e o sintoma direto: e ela que determina o erro de rastreio, e o erro de
# rastreio e a unica grandeza medida que separa gol de erro.
#
# Limites (ARARABOTS_MIN_HZ e ARARABOTS_MIN_FPS ajustam):
HZ_MIN_CONTROLE="${ARARABOTS_MIN_HZ:-60}"     # /control_command, timer e 100 Hz
FPS_MIN_GRSIM="${ARARABOTS_MIN_FPS:-45}"      # fisica do grSim
portao_medicao() {
    local motivo=""

    # 1. grSim de pe e em headless. Em modo janela a fisica so avanca quando a
    #    janela e redesenhada (glwidget.cpp:392): coberta = mundo parado.
    if ! pgrep -x grSim >/dev/null; then
        motivo="o grSim nao esta rodando"
    elif ! ps -o args= -C grSim 2>/dev/null | grep -q -- --headless; then
        echo "   ! grSim em modo JANELA: mantenha a janela visivel e sem nada"
        echo "     por cima, ou suba com ./ararabots.sh grsim --headless"
    fi

    # 2. Processos zumbis <defunct>: so REPORTA, nunca mata.
    #
    # A primeira versao desta checagem fazia
    #     pkill -9 -f "$(nome do zumbi)"
    # com o padrao derivado do nome do processo. Se o zumbi se chamasse 'bash'
    # ou 'python3', isso casava com o PID 1 do container e o matava: o 'vice'
    # caiu com ExitCode=137 (SIGKILL), OOMKilled=false, no meio de uma medicao.
    #
    # E era inutil de qualquer forma: um zumbi ja esta morto, esperando o pai
    # recolher. Matar o zumbi nao faz nada; quem precisaria ser reiniciado e o
    # pai. Entao aqui a gente so avisa - reiniciar a cadeia e decisao de quem le.
    # Zumbis: CONTA, nao lista, e nao bloqueia.
    #
    # O PID 1 do container e 'tail -f /dev/null', que nao recolhe orfaos - entao
    # todo node morto vira <defunct> permanente, com PPID=1. Sao inofensivos:
    # nao consomem CPU nem memoria, so uma entrada de PID.
    #
    # A primeira versao LISTAVA os nomes e enchia a tela com a mesma lista
    # repetida nove vezes. Agora e uma linha, e so aparece quando ja passou de
    # uma centena - que e quando valeria recriar o container.
    #
    # Correcao de raiz (arquivo do time, nao mexido): 'init: true' no serviço
    # vice do docker-compose.yml poe o tini como PID 1, que recolhe.
    local nz
    nz="$(docker exec vice ps -eo stat= 2>/dev/null | grep -c '^Z' || echo 0)"
    [ "${nz:-0}" -gt 100 ] 2>/dev/null && \
        echo "   ! ${nz} processos zumbi (inofensivos, mas ja da para recriar o container)"

    # 3. Nodes duplicados: ja medimos NOVE controllers publicando comandos
    #    contraditorios, com a visao caindo de 45 para 3 Hz.
    local dup
    dup="$(docker exec vice ps -eo cmd 2>/dev/null | grep -oE "lib/[a-z_]+/[a-zA-Z_]+" \
           | sort | uniq -c | awk '$1 > 1 {print $2}' | tr '\n' ' ')"
    [ -n "$dup" ] && motivo="nodes duplicados: $dup"

    # 4. As taxas que importam, medidas de verdade.
    local relatorio hz_ctl fps
    relatorio="$(ros_run "python3 /tmp/ararabots.py cadeia" 2>/dev/null)"
    hz_ctl="$(printf '%s' "$relatorio" | grep -oE "/control_command +[0-9.]+" | grep -oE "[0-9.]+$" | cut -d. -f1)"
    fps="$(ros_run "python3 /tmp/ararabots.py fps" 2>/dev/null | grep -oE "[0-9]+ Hz" | grep -oE "^[0-9]+")"

    [ -n "$fps" ] && [ "$fps" -lt "$FPS_MIN_GRSIM" ] 2>/dev/null && \
        motivo="grSim a ${fps} Hz (minimo ${FPS_MIN_GRSIM})"
    [ -n "$hz_ctl" ] && [ "$hz_ctl" -lt "$HZ_MIN_CONTROLE" ] 2>/dev/null && \
        motivo="/control_command a ${hz_ctl} Hz (minimo ${HZ_MIN_CONTROLE})"
    printf '%s' "$relatorio" | grep -q "SEM DADOS" && motivo="algum topico sem dados"

    if [ -n "$motivo" ]; then
        echo
        echo "   XX MEDICAO BLOQUEADA: $motivo"
        echo "      Este teste NAO rodaria um resultado valido, entao ele nao roda."
        echo
        echo "      O que costuma resolver, nesta ordem:"
        echo "        1. feche o que puder na maquina - navegador, IDE, este"
        echo "           terminal grafico. Sao 2 nucleos e so a pilha ROS pede ~127%."
        echo "        2. ./ararabots.sh limpar    (mata nodes orfaos)"
        echo "        3. ./ararabots.sh parar && ./ararabots.sh --headless"
        echo
        echo "      Para medir assim mesmo, ciente de que o numero nao vale:"
        echo "        ARARABOTS_MIN_HZ=0 ARARABOTS_MIN_FPS=0 ./ararabots.sh ..."
        return 1
    fi
    [ -n "$hz_ctl" ] && echo "   portao ok: /control_command ${hz_ctl} Hz, grSim ${fps:-?} Hz"
    return 0
}

# ============================================================================
# Traz os replays HTML de dentro do container para uma pasta OBVIA no host.
#
# Antes eles ficavam em /tmp/cenarios_freekick/ DENTRO do container 'vice' -
# tecnicamente disponiveis e na pratica invisiveis. Agora caem em
# <raiz>/replays/, com nome de cenario e hora, e o caminho e impresso.
# Fica ao lado dos repositorios (ssl-VICE, grSim, ssl-gui, ...), nao dentro de
# nenhum deles e nao em caminho de ninguem: RAIZ e descoberta subindo a arvore
# (ver _descobrir_raiz), entao isto funciona em qualquer maquina.
REPLAYS="$RAIZ/Replays_GrSim"
copiar_replays() {
    mkdir -p "$REPLAYS"
    local nomes
    nomes="$(docker exec vice sh -c 'ls -1 /tmp/cenarios_freekick/*.html 2>/dev/null' 2>/dev/null)" || return 0
    [ -z "$nomes" ] && return 0
    local carimbo; carimbo="$(date +%H%M%S)"
    local n destino ultimo=""
    for n in $nomes; do
        destino="$REPLAYS/$(basename "$n" .html)__$carimbo.html"
        docker cp "vice:$n" "$destino" >/dev/null 2>&1 && ultimo="$destino"
        docker exec vice rm -f "$n" >/dev/null 2>&1
    done
    [ -n "$ultimo" ] && echo "   REPLAY: $ultimo"
    return 0
}

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

    # MATA TAMBEM OS PAIS.
    #
    # Nao e para eliminar zumbis que ja existem - esses tem PPID=1 e so somem
    # recriando o container. E para nao CRIAR mais: matando so o executavel do
    # node, o 'ros2' e o 'bash' que o criaram continuam vivos, e a proxima
    # montagem sobe outro conjunto por cima.
    #
    # Os nodes sobem como  docker exec -d vice bash -c "... ros2 launch ..." ou
    # "... ros2 run ...". Matando so o executavel do node, o 'ros2' e o 'bash'
    # que o criaram continuam vivos e NAO recolhem o filho morto: ele vira
    # <defunct>. A cada montagem de cenario acumula mais um conjunto.
    #
    # Medido no terminal do Felipe, numa unica execucao:
    #   "processos zumbi no container: ros2 visionNode gameWatcher controller
    #    grsim_publisher" repetido NOVE vezes - nove launches deixados para tras.
    docker exec vice pkill -f "ros2 launch /root/ssl-VICE/launch" >/dev/null 2>&1 || true
    docker exec vice pkill -f "ros2 run new_movement driver" >/dev/null 2>&1 || true
    docker exec vice pkill -f "ros2 run referee referee_node" >/dev/null 2>&1 || true
    docker exec vice pkill -f "ros2 run strategy strategyNode" >/dev/null 2>&1 || true
    sleep 1
    docker exec vice pkill -9 -f "ros2 launch /root/ssl-VICE/launch" >/dev/null 2>&1 || true

    restantes=$(docker exec vice ps -eo cmd 2>/dev/null | grep -cE "lib/(control|control_unit|vision|new_movement|strategy|grsim_messenger|manual_command|referee)/" || true)
    zumbis=$(docker exec vice ps -eo stat= 2>/dev/null | grep -c "^Z" || true)
    echo "nodes ROS restantes: ${restantes:-0}   zumbis: ${zumbis:-0}"
}


# ============================================================================
cmd_grsim() {
    BIN="$RAIZ/grSim/bin/grSim"

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

    esperar_por 25 "grSim subir" pgrep -x grSim
    if pgrep -x grSim >/dev/null; then
        # Registra o modo em disco. Sem isto, quem for reerguer o simulador
        # depois nao tem como saber se ele estava com janela ou headless - e
        # subir no modo errado tem consequencia silenciosa: em modo janela, com
        # a janela coberta ou minimizada, a fisica CONGELA e os robos ficam
        # parados como se o codigo tivesse quebrado.
        echo "$MODO" > /tmp/ararabots_modo_grsim
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
    echo "Tudo parado. Para subir de novo: ./ararabots.sh --headless"
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
    # Liga as correcoes fora da estrategia enquanto os testes rodarem.
    # Liga so o grupo 'tracker' - o dt do Kalman, que e o ajuste VALIDADO
    # (yy de 68-143 para 29-71, HANDOVER §25).
    #
    # O grupo 'filtro' (parametros do proprio Kalman) fica de fora de proposito:
    # o defeito que ele corrige esta medido, mas o ganho de RESULTADO ainda nao
    # esta - 3 gols em 9 contra 1 em 6, dentro da dispersao. Ligar por padrao
    # seria empilhar uma camada nao comprovada, que e como a tatica virou uma
    # maquina que nao funcionava (§17). Para medi-lo: ajustes on filtro.
    cmd_ajustes on tracker

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
    esperar_por 25 "arbitro responder na 8081" curl -sf -o /dev/null http://localhost:8081/
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
    # Numero de robos: 11 por time custam FPS a toa. Todos os cenarios cabem em
    # 3, e com a maquina apertada (2 nucleos) essa diferenca decide se o teste
    # roda ou trava. O grSim reescreve o XML ao sair, entao reforcamos aqui.
    if [ -f ~/.grsim.xml ] && grep -q "Robots Count" ~/.grsim.xml; then
        n_rob="$(grep -A1 'Robots Count' ~/.grsim.xml | tail -1 | tr -d ' \t')"
        if [ "$n_rob" != "3" ]; then
            pgrep -x grSim >/dev/null && { pkill -x grSim; sleep 2; }
            perl -0pi -e 's|(<Var name="Robots Count"[^>]*>\s*\n\s*)\d+|${1}3|' ~/.grsim.xml
            ok "robos por time forcado para 3 (eram $n_rob) - alivia o FPS"
        fi
    fi

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
    # Multicast utilizavel? Depois de a maquina dormir ou trocar de rede, o
    # envio para 224.5.23.2 falha e o grSim enche o log com
    # "Sending UDP datagram failed (maybe too large?)" - mensagem enganosa: o
    # tamanho nao tem nada a ver, e a rota que sumiu. Sem multicast, a visao
    # nao chega em ninguem e os robos ficam parados sem explicacao.
    if ! python3 -c "
import socket,sys
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
try: s.sendto(b'x',('224.5.23.2',10020))
except OSError: sys.exit(1)
" 2>/dev/null; then
        echo "   XX o multicast 224.5.23.2 esta inacessivel."
        echo "      A maquina dormiu ou trocou de rede? Reconecte e tente de novo,"
        echo "      ou adicione a rota:  sudo ip route add 224.0.0.0/4 dev \$(ip route show default | awk '{print \$5; exit}')"
        faltou=1
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
        # o gameWatcher e quem publica /game_state; sem ele a estrategia fica
        # pendurada no get_game_config e a arvore nunca comanda.
        esperar_por 40 "sim_one subir" vivo "control_unit/gameWatcher"
    fi
    vivo "launch/sim_one.py" && ok "no ar" || falha "não subiu (docker exec vice cat /tmp/sim.log)"
    echo "      (o node 'hardware' morre por falta de /dev/ttyUSB0 - isso é normal em simulação)"

    # ---------------------------------------------------------------- 6
    passo 6 "Removendo o manual_command"
    # Em laco: o launch pode demorar a criar o processo, e enquanto ele viver estara
    # publicando em /commandTopic a 60 Hz, movendo os robos por conta propria.
    ausencias=0
    for _ in $(seq 1 40); do
        if docker exec vice pgrep -f "manual_command/manual_node" >/dev/null 2>&1; then
            docker exec vice pkill -f "manual_command/manual_node" 2>/dev/null || true
            ausencias=0
        else
            ausencias=$((ausencias + 1))
            [ "$ausencias" -ge 6 ] && break
        fi
        sleep 0.25
    done
    ok "fora de cena (ele disputaria /commandTopic com o controlador)"

    # ---------------------------------------------------------------- 7
    passo 7 "new_movement/driver"
    if ! vivo "new_movement/driver"; then
        ros_d "ros2 run new_movement driver > /tmp/driver.log 2>&1"
        esperar_por 30 "driver subir" vivo "new_movement/driver"
    fi
    vivo "new_movement/driver" && ok "no ar (fornece strategy_command e update_obstacles)" \
                               || falha "não subiu (docker exec vice cat /tmp/driver.log)"

    # ---------------------------------------------------------------- 8
    passo 8 "referee_node"
    if ! vivo "referee/referee_node"; then
        ros_d "ros2 run referee referee_node > /tmp/ref.log 2>&1"
        esperar_por 25 "referee_node subir" vivo "referee/referee_node"
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
        esperar_por 10 "strategyNode antigo morrer" bash -c '! docker exec vice pgrep -f "strategy/strategyNode" >/dev/null 2>&1'
        ros_d "ros2 run strategy strategyNode > /tmp/strategy.log 2>&1"
        esperar_por 30 "strategyNode subir" vivo "strategy/strategyNode"
        vivo "strategy/strategyNode" && ok "no ar" \
                                     || falha "não subiu (docker exec vice cat /tmp/strategy.log)"
    else
        echo "      (pulado por --sem-estrategia)"
    fi

    # ---------------------------------------------------------------- 10
    passo 10 "Verificando a cadeia"
    # A velocidade do simulador vem PRIMEIRO: com ele em camera lenta, todo o
    # resto do diagnostico parece saudavel e mesmo assim o teste nao vale.
    docker cp "$PY" vice:/tmp/ararabots.py >/dev/null 2>&1
    ros_run "python3 /tmp/ararabots.py fps" || FALHAS=$((FALHAS+1))
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

    # (ros_d/ros_run vem do topo do arquivo - eram redefinidos aqui, sem o
    #  ROS_LOCALHOST_ONLY, e a copia local vencia a global)

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
        # Tenta reerguer antes de desistir: ele morre sozinho sob carga, e
        # exigir intervencao manual a cada queda torna qualquer lote inviavel.
        # Sobe no MESMO modo de antes. Antes esta linha subia sempre com
        # janela, porque a variavel que ela consultava nunca era definida - e
        # uma janela que ninguem esta olhando congela a fisica. Era isso que
        # fazia os robos "travarem" sem explicacao no meio de um lote.
        modo_antes="$(cat /tmp/ararabots_modo_grsim 2>/dev/null || echo headless)"
        echo "      (grSim caiu - subindo de novo em modo $modo_antes)"
        if [ "$modo_antes" = "headless" ]; then
            cmd_grsim --headless >/dev/null 2>&1
        else
            cmd_grsim >/dev/null 2>&1
        fi
        pgrep -x grSim >/dev/null || { echo "   XX o grSim nao subiu   ->  ./ararabots.sh preparar"; faltou=1; }
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
    # ESPERA ATIVA, nao tempo fixo.
    #
    # Isto era 'mate o manual_command 14 vezes, uma por segundo' seguido de
    # 'sleep 7', 'sleep 5' e 'sleep 8' - 34 segundos escolhidos para o pior
    # caso, pagos INTEIROS em toda execucao, mesmo quando tudo ficava pronto em
    # dois segundos. Numa validacao de 42 execucoes isso e meia hora de relogio.
    #
    # O manual_command continua sendo morto em laco porque o launch pode demorar
    # a cria-lo, e enquanto ele viver publica em /commandTopic a 60 Hz e move os
    # robos sozinho (medimos 2 m de deriva antes de o teste comecar). Mas agora
    # o laco PARA assim que ele fica ausente por tempo suficiente.
    local ausencias=0
    for _ in $(seq 1 40); do
        if docker exec vice pgrep -f "manual_command/manual_node" >/dev/null 2>&1; then
            docker exec vice pkill -f "manual_command/manual_node" 2>/dev/null || true
            ausencias=0
        else
            ausencias=$((ausencias + 1))
            [ "$ausencias" -ge 6 ] && break
        fi
        sleep 0.25
    done

    # Os tres sobem JUNTOS: nenhum depende do outro para iniciar (a estrategia
    # espera os servicos por conta propria, no seu wait_for_service).
    ros_d "ros2 run new_movement driver > /tmp/driver.log 2>&1"
    ros_d "ros2 run referee referee_node > /tmp/ref.log 2>&1"
    ros_d "ros2 run strategy strategyNode > /tmp/strategy.log 2>&1"

    # ---------------------------------------------------------------- 5
    # Uma unica chamada confere servicos, arbitro e estrategia comandando.
    # Ver 'pronto tudo': cada docker exec com rclpy custa ~4 s so para subir,
    # entao fazer uma chamada por elo desperdicaria o que estamos economizando.
    # GANHO DO CONTROLADOR, ajustado em tempo de execucao.
    #
    # control.py vem com kp=2,3. Com os passos curtos que a tatica usa (150 mm),
    # isso pede 2,3 * 0,15 = 0,345 m/s - abaixo do que o robo precisa para sair
    # do lugar. O resultado e o robo nao completar a aproximacao.
    #
    # Medido, tres pontos de operacao no mesmo cenario:
    #   kp=2,3 kd=0,1 (padrao) -> aproximou 34-72 mm, xx=70-93  (passa da bola)
    #   kp=3,0 kd=0,8          -> aproximou 136-361 mm          (nem chega)
    #   kp=5,0 kd=0,3          -> aproximou 111-130 mm, xx=18-44 (contato certo)
    #
    # kp=1.5, e nao 5: com o alvo do empurrao fixado 350 mm ALEM da bola, o
    # erro nunca fica pequeno, entao a zona morta deixa de ser um problema - e
    # ganho alto passa a ser um. Com kp=5 o comando pedia 1,75 m/s e o
    # controlador saturava em 1,5; nessa velocidade as rodas escorregam
    # (WheelPerpendicularFriction=0,05 na config do robo) e o robo GUINA.
    # Medimos o erro angular crescendo de 0,02 para 0,66 rad com alvo fixo.
    # 1,5 * 0,35 m = 0,52 m/s: firme, sem saturar, sem escorregar.
    #
    # (kp=5 era o unico dos tres testados que punha o robo na distancia de contato
    # (111 mm = raio do robo 90 + raio da bola 21,5). Nao resolve o alinhamento
    # lateral, mas e estritamente melhor que o padrao - e nao exige tocar em
    # arquivo nenhum, so o servico que o proprio control.py expoe.
    if ! ros_run "python3 /tmp/ararabots.py pronto tudo 70"; then
        echo "   !! a cadeia nao ficou pronta - o resultado NAO vale"
        return 2
    fi

    # O ganho e aplicado DEPOIS da cadeia estar pronta, e CONFERIDO.
    #
    # Antes esta chamada vinha antes do 'pronto tudo' e mandava a saida inteira
    # para /dev/null. Duas falhas silenciosas cabiam ai: o controller ainda nao
    # ter criado o servico (o timeout de 10 s estourava e ninguem via), ou a
    # chamada falhar por qualquer outro motivo. Nos dois casos a execucao seguia
    # com o kp=2,3 do repositorio - e seria registrada como se estivesse em
    # kp=1,5, o ponto de operacao que o HANDOVER §16.5 chama de "o que funciona".
    #
    # E exatamente o erro do §14: atribuir resultado de teste a uma mudanca que
    # nao chegou a existir. Ali foi um assert de script que falhou calado; aqui
    # seria uma chamada de servico. Agora conferimos a resposta.
    local saida_pid
    saida_pid="$(ros_run "timeout 15 ros2 service call /update_pid system_interfaces/srv/ControlParams '{id: 0, kp: 1.5, ki: 0.0, kd: 0.3}'" 2>&1)"
    if ! printf '%s' "$saida_pid" | grep -q "success=True"; then
        echo "   !! o ganho kp=1.5 NAO foi aplicado - o resultado NAO vale"
        echo "      (a execucao rodaria com o kp=2,3 do repositorio)"
        printf '%s\n' "$saida_pid" | tail -3 | sed 's/^/      /'
        return 2
    fi

    # Ultimo portao antes de gravar: com a cadeia de pe e o ganho aplicado,
    # confere se as TAXAS permitem um resultado valido. Ver portao_medicao.
    portao_medicao || return 3
    return 0
}


# ============================================================================
cmd_validar() {
    N="${1:-3}"
    shift 2>/dev/null || true
    LISTA=("$@")
    SAIDA="$SCRIPT_DIR/validacao.csv"

    # Trava simples contra duas instancias simultaneas.
    TRAVA="/tmp/ararabots_validar.lock"
    if ! mkdir "$TRAVA" 2>/dev/null; then
        echo "XX ja existe uma validacao rodando (trava em $TRAVA)."
        echo "   Se tiver certeza de que nao ha, remova com: rmdir $TRAVA"
        return 1
    fi
    trap 'rmdir "$TRAVA" 2>/dev/null' EXIT

    # (ros_run vem do topo do arquivo)

    mapfile -t CENARIOS < <(python3 "$PY" listar | cut -d"|" -f1)

    [ ${#LISTA[@]} -gt 0 ] && CENARIOS=("${LISTA[@]}")
    [ -f "$SAIDA" ] || echo "cenario,rep,gol,chute_pedido,disparou,hz_controle,rastreio_med,rastreio_p90,janela_abriu,janela_armada,v_saida_mms,xx,yy,percorreu,bola_ini_x,bola_ini_y,bola_fim_x,bola_fim_y,andou,load" > "$SAIDA"
    for c in "${CENARIOS[@]}"; do
        echo "############ $c"
        for i in $(seq 1 "$N"); do
            cmd_cenario "$c" >/dev/null 2>&1
            st=$?
            [ $st -eq 1 ] && { echo "  rep$i: cenario nao montou"; echo "$c,$i,ERRO,,,,,,," >> "$SAIDA"; continue; }
            [ $st -eq 3 ] && { echo "  rep$i: BLOQUEADO pelo portao de medicao"; echo "$c,$i,BLOQUEADO,,,,,,," >> "$SAIDA"; continue; }
            L=$(cut -d' ' -f1 /proc/loadavg)
            OUT=$(ros_run "BRANCH=val_${c}_$i python3 /tmp/ararabots.py rodar $c 25" 2>&1)
            GOL=$(echo "$OUT" | grep -oE "GOL A FAVOR|GOL CONTRA|sem gol" | head -1)
        grsim_sobreviveu || GOL="GRSIM-CAIU"
            KICK=$(echo "$OUT" | grep -oE "CHUTE: (ATIVADO|nao ativado)" | head -1 | sed 's/CHUTE: //')
            BOLA=$(echo "$OUT" | grep -oE "BOLA: \(-?[0-9]+,-?[0-9]+\) -> \(-?[0-9]+,-?[0-9]+\)  andou +[0-9]+" | head -1)
            BI=$(echo "$BOLA" | sed -E 's/BOLA: \((-?[0-9]+),(-?[0-9]+)\).*/\1,\2/')
            BF=$(echo "$BOLA" | sed -E 's/.*-> \((-?[0-9]+),(-?[0-9]+)\).*/\1,\2/')
            AN=$(echo "$BOLA" | grep -oE "andou +[0-9]+" | grep -oE "[0-9]+")
            # Medidas novas: velocidade REAL de saida da bola (visao crua do
            # grSim) e a geometria do chutador no ponto de maior aproximacao.
            # Sem elas, uma execucao sem gol nao distingue "chegou torto",
            # "nao chegou" e "chutou mas nao teve alcance".
            DP=$(echo "$OUT" | grep -oE "DISPARO \(grSim\): (SIM|NAO)" | grep -oE "(SIM|NAO)")
            HZ=$(echo "$OUT" | grep -oE "laco do driver a [0-9]+ Hz" | grep -oE "[0-9]+" | head -1)
            RM=$(echo "$OUT" | grep -oE "mediana [0-9]+ mm" | head -1 | grep -oE "[0-9]+")
            RP=$(echo "$OUT" | grep -oE "p90 [0-9]+" | head -1 | grep -oE "[0-9]+$")
            JA=$(echo "$OUT" | grep -oE "abriu em [0-9]+ quadros" | grep -oE "[0-9]+")
            JR=$(echo "$OUT" | grep -oE "com o chute ARMADO: [0-9]+" | grep -oE "[0-9]+$")
            [ -z "$JA" ] && JA=0
            [ -z "$JR" ] && JR=0
            VS=$(echo "$OUT" | grep -oE "VELOCIDADE DE SAIDA \(janela 50 ms\): [0-9]+" | grep -oE "[0-9]+$")
            XX=$(echo "$OUT" | grep -oE "xx=[ ]*[0-9.]+" | head -1 | grep -oE "[0-9.]+")
            YY=$(echo "$OUT" | grep -oE "yy=[ ]*[0-9.]+" | head -1 | grep -oE "[0-9.]+")
            PC=$(echo "$OUT" | grep -oE "a bola percorreu [0-9]+" | grep -oE "[0-9]+")
            echo "  rep$i: ${GOL:-?}  disparou=${DP:-?}  laco=${HZ:-?}Hz  rastreio=${RM:-?}/${RP:-?}mm  janela=${JA}q/armada=${JR}q  saida=${VS:-?}mm/s  xx=${XX:-?} yy=${YY:-?}  ${BOLA:-sem leitura}  (load $L)"
            copiar_replays
            echo "$c,$i,${GOL:-?},${KICK:-?},${DP:-?},${HZ:-},${RM:-},${RP:-},${JA},${JR},${VS:-},${XX:-},${YY:-},${PC:-},${BI:-,},${BF:-,},${AN:-},$L" >> "$SAIDA"
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
        [ $st -eq 3 ] && { echo "   XX bloqueado pelo portao de medicao (acima)"; return 1; }
        [ $st -eq 2 ] && echo "   !! a estrategia nao comandou - o resultado abaixo NAO vale"
        ros_run "python3 /tmp/ararabots.py rodar $nome $DURACAO"
    grsim_sobreviveu
    copiar_replays
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
            [ $st -eq 3 ] && { echo "   XX bloqueado pelo portao de medicao"; continue; }
            [ $st -eq 2 ] && echo "   !! a estrategia nao comandou - resultado suspeito"
            ros_run "BRANCH=disp_$i python3 /tmp/ararabots.py rodar $nome $DURACAO" \
                | grep -E "GOL|sem gol|DISPARO|RASTREIO|acima de 200|laco do driver|abriu em|nunca abriu|robo [0-9]:|BOLA:" | sed 's/^/   /'
            grsim_sobreviveu
            copiar_replays
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
        echo "   a) estado dos ajustes fora da estrategia"
        echo "   q) sair (desliga os ajustes)"
        echo
        read -rp "escolha: " opcao

        case "$opcao" in
            q|Q) break ;;
            a|A) cmd_ajustes status; read -rp "[enter] " _ ;;
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
    # Ao sair, devolve o repositorio ao estado limpo: os ajustes nao ficam
    # ligados sem alguem estar olhando. Foi exatamente isso que nos custou uma
    # sessao inteira de conclusoes invalidas.
    cmd_ajustes off
    echo "Para derrubar tudo:  ./ararabots.sh parar"
}


# ============================================================================
cmd_instalar() {
    # ==========================================================================
    #  Instalacao POR PECA, nao em bloco.
    # ==========================================================================
    #
    # Cada peca e verificada e instalada por conta propria. Rodar de novo depois
    # de uma falha continua de onde parou, em vez de recomecar - e rodar com tudo
    # pronto nao faz nada.
    #
    # Antes era um roteiro linear de 6 passos que abortava no primeiro tropeco, e
    # tinha um caso pior: ao adicionar o usuario ao grupo 'docker' ele DESISTIA
    # do resto e pedia para rodar tudo de novo.
    #
    # O layout de pastas e o que o README do ssl-VICE exige: os repositorios
    # precisam ser IRMAOS, porque o docker-compose.yml referencia ../ssl-gui.
    #
    #     <RAIZ>/ssl-VICE  ssl-gui  ssl-game-controller  grSim
    local feitos=() pulados=() falhos=()

    titulo() { echo ""; echo "-- $1"; }
    _ok()    { echo "   ok: $1";   feitos+=("$1"); }
    _pulo()  { echo "   ja ok: $1"; pulados+=("$1"); }
    _falha() { echo "   XX $1";    falhos+=("$1"); }

    # ---------------------------------------------------------------- pacotes
    # Ubuntu/Debian e Arch. O repositorio documenta os dois (ver
    # docs/archlinux-setup.md), entao o instalador nao pode assumir apt.
    local GER=""
    command -v apt    >/dev/null && GER="apt"
    command -v pacman >/dev/null && GER="pacman"

    titulo "Pacotes do sistema"
    if [ -z "$GER" ]; then
        _falha "gerenciador de pacotes nao reconhecido (nem apt nem pacman) - instale as dependencias a mao"
    elif [ -x "$RAIZ/grSim/bin/grSim" ] && command -v docker >/dev/null && command -v cmake >/dev/null; then
        _pulo "dependencias ja presentes"
    else
        echo "   instalando (pode pedir sua senha)..."
        if [ "$GER" = "apt" ]; then
            # Lista do INSTALL.md do grSim. pkg-config e libglu1-mesa-dev sao
            # obrigatorios: sem eles o cmake falha. VarTypes NAO entra - o
            # CMakeLists do grSim baixa e compila sozinho.
            sudo apt update && sudo apt install -y \
                git build-essential cmake pkg-config \
                qtbase5-dev libqt5opengl5-dev \
                libgl1-mesa-dev libglu1-mesa-dev \
                libprotobuf-dev protobuf-compiler \
                libode-dev libboost-dev docker.io docker-compose-v2 \
                && _ok "pacotes" || _falha "apt install falhou"
        else
            sudo pacman -S --needed --noconfirm \
                git base-devel cmake pkgconf qt5-base glu \
                protobuf ode boost docker docker-compose \
                && _ok "pacotes" || _falha "pacman falhou"
        fi
    fi

    # ---------------------------------------------------------------- docker
    titulo "Docker"
    if ! command -v docker >/dev/null; then
        _falha "docker nao instalado"
    else
        sudo systemctl enable --now docker >/dev/null 2>&1 || true
        if docker info >/dev/null 2>&1; then
            _pulo "docker acessivel"
        else
            # Adiciona ao grupo e SEGUE. Antes o script desistia aqui e mandava
            # rodar tudo de novo - o resto da instalacao nao depende disto.
            sudo usermod -aG docker "$USER" 2>/dev/null || true
            _falha "sem permissao no docker - faca logout/login (ou 'newgrp docker') e rode de novo"
        fi
    fi

    # ---------------------------------------------------------------- repos
    titulo "Repositorios (irmaos em $RAIZ)"
    mkdir -p "$RAIZ"
    local repos="ssl-VICE|https://github.com/Ararabots-UFMS/ssl-VICE.git
ssl-gui|https://github.com/Ararabots-UFMS/ssl-gui.git
ssl-game-controller|https://github.com/RoboCup-SSL/ssl-game-controller.git
grSim|https://github.com/RoboCup-SSL/grSim.git"
    local linha dir url
    while IFS='|' read -r dir url; do
        [ -z "$dir" ] && continue
        if [ -d "$RAIZ/$dir/.git" ]; then
            _pulo "$dir"
        else
            echo "   clonando $dir..."
            git clone --quiet "$url" "$RAIZ/$dir" && _ok "$dir" || _falha "clone de $dir"
        fi
    done <<< "$repos"

    # ---------------------------------------------------------------- grSim
    titulo "grSim (compilacao nativa)"
    # Nativo e nao em Docker: o grSim e OpenGL pesado, e repassar X11 para o
    # container costuma cair em software rendering e derrubar o FPS.
    if [ -x "$RAIZ/grSim/bin/grSim" ]; then
        _pulo "grSim compilado"
    elif [ ! -d "$RAIZ/grSim" ]; then
        _falha "grSim nao foi clonado"
    else
        echo "   compilando (10-20 min; o VarTypes e baixado sozinho)..."
        ( mkdir -p "$RAIZ/grSim/build" && cd "$RAIZ/grSim/build" \
          && cmake .. >/dev/null && make -j"$(nproc)" >/dev/null ) \
            && _ok "grSim" || _falha "compilacao do grSim"
    fi

    # ---------------------------------------------------------------- imagens
    titulo "Imagens Docker"
    if ! docker info >/dev/null 2>&1; then
        _falha "docker inacessivel - pulei as imagens"
    else
        if docker image inspect robocupssl/ssl-game-controller:latest >/dev/null 2>&1; then
            _pulo "imagem do arbitro"
        else
            # Imagem oficial pronta: evita um build Go + frontend demorado.
            docker pull -q robocupssl/ssl-game-controller:latest >/dev/null \
                && _ok "imagem do arbitro" || _falha "pull do arbitro"
        fi

        if [ -x "$VICE/scripts/vice" ]; then
            if [ -n "$(docker images -q ssl-vice 2>/dev/null)" ]; then
                _pulo "imagem do ssl-VICE"
            else
                echo "   construindo a imagem do ssl-VICE (via CLI 'vice')..."
                "$VICE/scripts/vice" build && _ok "imagem do ssl-VICE" \
                    || _falha "vice build"
            fi
        else
            _falha "scripts/vice nao encontrado em $VICE"
        fi
    fi

    # ---------------------------------------------------------------- workspace
    titulo "Workspace ROS 2"
    # Clonar nao basta: sem colcon build o 'ros2 run strategy strategyNode'
    # falha com "package not found", e o sintoma so aparece cinco passos depois
    # como "a estrategia nao sobe".
    if [ -z "$(docker ps -q -f name=^vice$)" ]; then
        "$VICE/scripts/vice" start >/dev/null 2>&1 || true
        sleep 3
    fi
    if [ -z "$(docker ps -q -f name=^vice$)" ]; then
        _falha "container 'vice' nao subiu - workspace nao compilado"
    elif docker exec vice test -d /root/ssl-VICE/install/strategy 2>/dev/null; then
        _pulo "workspace compilado"
    else
        echo "   colcon build (alguns minutos)..."
        docker exec vice bash -c \
            "cd /root/ssl-VICE && source /opt/ros/humble/setup.bash && colcon build" \
            >/dev/null 2>&1 && _ok "workspace" || _falha "colcon build"
    fi

    # ---------------------------------------------------------------- resumo
    echo ""
    echo "==============================================================="
    echo "  RESUMO DA INSTALACAO"
    echo "==============================================================="
    [ ${#pulados[@]} -gt 0 ] && printf "  ja estava pronto: %s\n" "$(IFS=, ; echo "${pulados[*]}")"
    [ ${#feitos[@]}  -gt 0 ] && printf "  instalado agora : %s\n" "$(IFS=, ; echo "${feitos[*]}")"
    if [ ${#falhos[@]} -gt 0 ]; then
        printf "  FALTOU          : %s\n" "$(IFS=, ; echo "${falhos[*]}")"
        echo ""
        echo "  Rode ./ararabots.sh instalar de novo - ele continua de onde parou."
        return 1
    fi
    echo ""
    echo "  Tudo pronto. Agora:  ./ararabots.sh"
    echo ""
    echo "  Interface   http://localhost:5173     Arbitro  http://localhost:8081"
    echo ""
    echo "  Dica do README: coloque $VICE/scripts no PATH para usar o CLI 'vice'."
    return 0
}



# ==============================================================================
#  Ajustes fora de src/strategy/
# ==============================================================================
#
#  Ha correcoes que ajudam os testes mas moram FORA da estrategia - hoje so a do
#  dt do filtro de Kalman, em src/vision/vision/tracker.py. Adota-las de vez e
#  decisao do time, entao ficam DESLIGADAS no repositorio e o script liga apenas
#  enquanto os testes rodam, devolvendo tudo ao estado limpo ao sair.
#
#  Cada trecho e delimitado por >>> ARARABOTS_AJUSTE ... <<< ARARABOTS_AJUSTE.
#  Dentro dele, as linhas do ajuste levam o prefixo '#AJUSTE#' e as originais
#  levam '#ORIG#' quando desativadas.
#
#  POR QUE ISTO EXISTE: rodamos uma sessao inteira de investigacao com essa
#  correcao comentada sem perceber. O robo estava sendo guiado por um filtro que
#  ignorava as medicoes, e as conclusoes daquele dia nao valiam nada.
cmd_ajustes() {
    local acao="${1:-status}"
    # QUAL ajuste. Os dois moram em src/vision/, mas nao tem o mesmo aval:
    #   tracker  - o dt do Kalman. Medido e VALIDADO (yy de 68-143 para 29-71).
    #   filtro   - os parametros do proprio filtro. O defeito e real e esta
    #              medido, mas em 6 execucoes contra 6 NAO melhorou o resultado
    #              (0 gols e 0 disparos com, 1 e 1 sem). Fica separado para nao
    #              entrar de carona num lote que quer medir so o tracker.
    # Sem argumento vale 'tudo', como antes.
    local alvo="${2:-tudo}"
    rm -f /tmp/ararabots_ajuste_mudou
    local -a ARQS=()
    case "$alvo" in
        tracker) ARQS=("$VICE/src/vision/vision/tracker.py") ;;
        filtro)  ARQS=("$VICE/src/vision/vision/kalman_filter.py") ;;
        driver)  ARQS=("$VICE/src/new_movement/new_movement/driver.py") ;;
        tudo|*)  ARQS=("$VICE/src/vision/vision/tracker.py"
                       "$VICE/src/vision/vision/kalman_filter.py"
                       "$VICE/src/new_movement/new_movement/driver.py") ;;
    esac
    python3 - "$acao" "${ARQS[@]}" <<'PY'
import re, sys
# Agora sao VARIOS arquivos: o dt do Kalman mora no tracker.py e os parametros
# do proprio filtro moram no kalman_filter.py. Um liga/desliga so, para os dois,
# porque testar com metade das correcoes ligadas nao mede nada.
acao, arqs = sys.argv[1], sys.argv[2:]
mudou = 0
ativo = False
for arq in arqs:
  linhas = open(arq, encoding="utf-8").read().split("\n")
  # IDEMPOTENCIA - defeito real, que ja destruiu um arquivo.
  #
  # Rodar 'off' duas vezes seguidas comentava TAMBEM a linha original: na
  # segunda passada ela e uma linha de codigo comum dentro do bloco, entao
  # ganhava o prefixo #AJUSTE#. Resultado: as duas versoes comentadas e a
  # classe sem __init__ nenhum - IndentationError ao importar. O mesmo vale
  # para 'on' repetido, e para o tracker.py.
  #
  # Um arquivo esta LIGADO se guarda alguma linha como #ORIG#. Se ja esta no
  # estado pedido, nao se mexe nele.
  ja_ligado = any(l.strip().startswith("#ORIG#") for l in linhas)
  if (acao == "on" and ja_ligado) or (acao == "off" and not ja_ligado):
      ativo = ativo or ja_ligado
      continue
  dentro = False
  for i, l in enumerate(linhas):
    if ">>> ARARABOTS_AJUSTE" in l:
        dentro = True
        continue
    if "<<< ARARABOTS_AJUSTE" in l:
        dentro = False
        continue
    if not dentro or not l.strip():
        continue
    eh_ajuste = "#AJUSTE#" in l
    eh_orig_off = l.strip().startswith("#ORIG#")
    eh_comentario = l.strip().startswith("#") and not eh_ajuste and not eh_orig_off
    if eh_comentario:
        continue
    if acao == "on":
        if eh_ajuste:
            linhas[i] = l.replace("#AJUSTE# ", "", 1); mudou += 1
        elif not eh_orig_off:
            linhas[i] = re.sub(r"^(\s*)", r"\1#ORIG# ", l); mudou += 1
    elif acao == "off":
        if eh_orig_off:
            linhas[i] = l.replace("#ORIG# ", "", 1); mudou += 1
        elif not eh_ajuste:
            linhas[i] = re.sub(r"^(\s*)", r"\1#AJUSTE# ", l); mudou += 1
    else:
        # LIGADO = existe pelo menos uma linha original guardada como #ORIG#.
        # Antes isto procurava a string "self.dt = dt", presa a UMA correcao do
        # tracker.py: qualquer ajuste novo nasceria invisivel para o 'status'.
        ativo = ativo or eh_orig_off
  if acao in ("on", "off"):
    open(arq, "w", encoding="utf-8").write("\n".join(linhas))
if acao in ("on", "off"):
    print("   ajustes %s (%d linhas em %d arquivos)"
          % (acao.upper(), mudou, len(arqs)))
    # o shell precisa saber se mexeu em arquivo: se mexeu, os nodes que ja
    # carregaram o modulo antigo tem de cair.
    open("/tmp/ararabots_ajuste_mudou", "w").write("1" if mudou else "0")
else:
    print("   ajustes fora da estrategia: %s" % ("LIGADOS" if ativo else "desligados"))
PY
    # Com --symlink-install o node le o arquivo do repositorio, entao basta
    # reinicia-lo; nao ha rebuild.
    ARARABOTS_MUDOU="$(cat /tmp/ararabots_ajuste_mudou 2>/dev/null || echo 0)"

    # DERRUBA A CADEIA INTEIRA quando alguma linha mudou de verdade.
    #
    # BUG QUE ISTO CORRIGE (visto no terminal do Felipe):
    #   1a execucao: "ajustes ON (0 linhas)"  -> nada mudou, nada foi morto, tudo funcionou
    #   2a execucao: "ajustes ON (5 linhas)"  -> visao 0 Hz, arbitro 0 Hz, teste invalido
    #
    # A versao anterior matava so o visionNode (e o driver) para que o node
    # relesse o arquivo. Mas o 'preparar' logo em seguida ve o LAUNCH ainda de
    # pe, conclui "nodes de simulacao: no ar" e nao sobe ninguem - sobrava um
    # launch vivo sem visionNode dentro. A cadeia ficava mutilada e todo teste
    # depois disso media o nada.
    #
    # Agora, se mudou linha, derruba tudo pelo caminho do executavel. O passo
    # [5/10] do preparar passa a nao encontrar nada de pe e sobe a cadeia
    # inteira, ja com o arquivo novo.
    if [ "$acao" != "status" ] && [ "${ARARABOTS_MUDOU:-0}" != "0" ]; then
        cmd_limpar >/dev/null 2>&1 || true
    fi
    return 0
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
    ajustes)   shift; cmd_ajustes "$@" ;;
    menu)      cmd_menu ;;
    -h|--help) uso ;;
    # sem subcomando reconhecido: e o menu, e os argumentos sao dele
    *)         cmd_menu "$@" ;;
esac
