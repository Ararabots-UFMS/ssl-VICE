#!/usr/bin/env python3
"""ararabots.py - toda a parte Python dos testes, num arquivo so.

Roda DENTRO do container 'vice' (menos 'resumo', que le os JSON no host).
Normalmente voce nao chama este arquivo direto: quem chama e o ararabots.sh.

    python3 ararabots.py listar                 nomes e titulos dos cenarios
    python3 ararabots.py posicionar <cenario>   monta o cenario no grSim
    python3 ararabots.py rodar <cenario> [s]    dispara a falta e grava
    python3 ararabots.py cadeia                 diagnostico dos topicos ROS
    python3 ararabots.py fps                    taxa real de quadros do grSim
    python3 ararabots.py pronto <elo> [s]       espera um elo ficar pronto
    python3 ararabots.py esperar [s]            espera a estrategia comandar
    python3 ararabots.py resumo [rotulo]        resume a dispersao das execucoes
    python3 ararabots.py atrito [v1 v2 ...]     mede o alcance da bola
    python3 ararabots.py decisao                confere a decisao da jogada

POR QUE 'posicionar' e 'rodar' sao separados: entre os dois o shell REINICIA os
nodes ROS. O filtro de Kalman leva dezenas de segundos para aceitar um teleporte
de 5 m (e faz bem: bola nenhuma pula assim), entao subimos tudo depois de a bola
ja estar no lugar - cada node nasce vendo o mundo na posicao do cenario.
"""
import base64
import glob
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from system_interfaces.msg import (
        VisionMessage, GameState, ControlCommand, TeamCommand, RefereeMessage,
    )
except ImportError:
    # Fora do container so funcionam as ferramentas que nao usam ROS ('resumo').
    rclpy = None



GRSIM_HOST = "127.0.0.1"
GRSIM_PORT = 20011          # porta de comandos/replacement do grSim
GC_HOST = "127.0.0.1"
GC_PORT = 8081
SAIDA_DIR = "/tmp/cenarios_freekick"

# Segundos de HALT apos o teleporte, para o driver se reancorar.
ESPERA_HALT = 8.0

# Gol: bola cruzando a linha com |y| dentro da largura do gol (Division B: 1 m).
GOL_X = 4500.0
GOL_MEIA_LARGURA = 500.0

# ===============================================================
# CONFIGURAÇÕES E FLUXOS DO ÁRBITRO
# ===============================================================
FLUXOS_ARBITRAGEM = {
    "freekick": ["STOP", "DIRECT_FREE_KICK_YELLOW"],
    "kickoff":  ["STOP", "PREPARE_KICKOFF_YELLOW", "NORMAL_START"]
}

# ==========================================================================
#  Cenarios
# ==========================================================================
# Coordenadas em MILIMETROS (mesma unidade da visao). A conversao para metros,
# que e o que o grSim espera no replacement, acontece em posicionar().
#
# Somos AZUIS e defendemos o gol de x negativo (on_positive_half=False),
# entao atacamos o lado positivo.
#
# O robo 0 e sempre o goleiro na logica do time.

# FUNCOES fixas em todos os cenarios (o codigo so trata o robo 0 como goleiro;
# as demais funcoes sao intencao do teste, para o relatorio ficar interpretavel):
#
#   NOSSOS (azul)          ADVERSARIOS (amarelo)
#   0 goleiro (nunca cobra)  0 goleiro
#   1 cobrador               1 barreira (>= 500 mm da bola, regra 5.3.3)
#   2 apoio / linha de passe 2 marcador
#   3 cobertura / recuo
CENARIOS = {
    "ataque": {
        "titulo": "Nosso freekick no ataque",
        "descricao": (
            "Bola no terco de ataque, alem do limiar. Esperado: o cobrador (1) "
            "posiciona atras da bola, empurra rumo ao gol adversario e ATIVA o "
            "chute; o goleiro (0) permanece na meta; apoio (2) da linha de passe "
            "e cobertura (3) segura o contra-ataque."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1900, 150, 0),
                  (2, 1700, 1400, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180), (2, 2900, 1100, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "meio": {
        "titulo": "Nosso freekick no meio-campo",
        "descricao": (
            "Bola no centro, aquem do limiar. Mesmo posicionamento do cobrador, "
            "porem o chute deve ficar DESATIVADO."
        ),
        "bola": (500.0, 800.0),
        "azuis": [(0, -4300, 0, 0), (1, -100, 900, 0),
                  (2, -300, 2000, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 1050, 800, 180), (2, 1400, 1900, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "passe": {
        "titulo": "Falta longe do gol: tem de sair PASSE, nao chute",
        "descricao": (
            "Bola no meio-campo, aquem do limiar de chute, com um companheiro "
            "ADIANTADO e com a linha livre de adversarios. O esperado e o "
            "cobrador tocar para ele com forca de passe (~2,5 m/s), e nao "
            "empurrar a bola sozinho ate a jogada expirar. Complementa o "
            "cenario 'meio', em que os companheiros estao ATRAS da bola e o "
            "certo e justamente NAO passar."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -400, 0, 0), (2, 1600, 400, 0)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "defesa": {
        "titulo": "Nosso freekick no campo de defesa",
        "descricao": (
            "Bola perto do nosso gol. O cobrador deve afastar a bola do perigo "
            "com o chute DESATIVADO; o goleiro nao pode deixar a meta."
        ),
        "bola": (-2800.0, -600.0),
        "azuis": [(0, -4300, 0, 0), (1, -3200, -750, 0),
                  (2, -1800, 900, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, -2250, -800, 180), (2, -1400, 200, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "limiar": {
        "titulo": "Bola ENTRE os dois limiares de chute",
        "descricao": (
            "Bola em x=1600. Com as medidas do repositorio (limiar 1125) o chute "
            "LIGA; com as da Division B (limiar 2250) NAO liga. E o caso que "
            "separa os dois perfis de campo."
        ),
        "bola": (1600.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1000, 100, 0),
                  (2, 900, 1500, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 2150, 0, 180), (2, 2400, 1200, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "lateral": {
        "titulo": "Bola junto a linha lateral",
        "descricao": (
            "Bola quase na lateral (y=2700 de 3000). Caso de borda: o ponto de "
            "aproximacao de _go_to_goal pode cair fora do campo."
        ),
        "bola": (2000.0, 2700.0),
        "azuis": [(0, -4300, 0, 0), (1, 1400, 2500, 0),
                  (2, 1600, 1100, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 2500, 2450, 180), (2, 2900, 1500, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "canto": {
        "titulo": "Bola no canto do campo de ataque",
        "descricao": (
            "Bola no canto ofensivo. A linha bola->gol fica bem inclinada e o "
            "ponto de aproximacao vai para perto da quina."
        ),
        "bola": (4100.0, 2700.0),
        "azuis": [(0, -4300, 0, 0), (1, 3550, 2500, 0),
                  (2, 3200, 1000, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3950, 2050, 180), (2, 3900, 900, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "deles_meio": {
        "titulo": "Freekick DELES no meio-campo",
        "descricao": (
            "Cobranca amarela no centro, com cobrador a 600 mm da bola e um "
            "companheiro avancado. Esperado: nossos robos recuam para a linha "
            "entre a bola e o NOSSO gol formando bloqueio, e o goleiro fica na "
            "meta (classe TheirFreekick)."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -900, 0, 180),
                  (2, -1300, 900, 180)],
        "amarelos": [(0, 4300, 0, 180), (1, 600, 0, 180), (2, 1400, 900, 180)],
        "comando": ("DIRECT", "YELLOW"),
    },
    "deles_perto_gol": {
        "titulo": "Freekick DELES perto do nosso gol",
        "descricao": (
            "Perigo maximo: cobranca rente a nossa area. Esperado: bloqueio "
            "concentrado entre a bola e o gol, e o goleiro na linha."
        ),
        "bola": (-3200.0, 500.0),
        "azuis": [(0, -4300, 0, 0), (1, -3750, 450, 180),
                  (2, -3600, -300, 180)],
        "amarelos": [(0, 4300, 0, 180), (1, -2600, 500, 180), (2, -2400, -400, 180)],
        "comando": ("DIRECT", "YELLOW"),
    },

    # ---------------------------------------------------------------------
    #  Contagem de robos: casos de robustez.
    #
    #  A logica trata o robo 0 como goleiro (if 0 in self.ally_robots) e itera
    #  sobre o resto. Variar quem esta em campo exercita caminhos que os
    #  cenarios cheios nunca tocam - inclusive o de lista vazia.
    # ---------------------------------------------------------------------
    "dois_goleiro_e_cobrador": {
        "titulo": "Dois em campo: goleiro + cobrador",
        "descricao": (
            "So o robo 0 (goleiro) e o robo 1 (cobrador). Esperado: goleiro na "
            "meta e o cobrador executando a falta sozinho, com chute ativo por "
            "estar alem do limiar. Se o goleiro sair da meta aqui, o problema e "
            "do alvo dele, nao de disputa com companheiros."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1900, 150, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "dois_sem_goleiro": {
        "titulo": "Dois em campo, SEM goleiro",
        "descricao": (
            "Robos 1 e 2, sem o robo 0. Testa o caminho em que "
            "'if 0 in self.ally_robots' e falso: nenhum comando de goleiro deve "
            "ser gerado, e os dois de linha devem tratar a bola. Revela se os "
            "dois recebem o MESMO alvo e disputam a mesma posicao."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(1, 1900, 150, 0), (2, 1700, 1400, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "um_so_cobrador": {
        "titulo": "Um em campo: apenas o cobrador",
        "descricao": (
            "Somente o robo 1. Sem goleiro e sem apoio: e o caso minimo em que a "
            "cobranca ainda deve acontecer. Se falhar aqui, o problema esta na "
            "propria execucao da falta, nao no time."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(1, 1900, 150, 0)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "um_so_goleiro": {
        "titulo": "Um em campo: apenas o goleiro",
        "descricao": (
            "Somente o robo 0. Nao ha quem cobre a falta. Esperado: um unico "
            "comando, mandando o goleiro para a meta, e nenhuma tentativa de "
            "chute. Serve para confirmar que o goleiro NAO assume a cobranca."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(0, -4300, 0, 0)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "campo_vazio": {
        "titulo": "Ninguem em campo do nosso time",
        "descricao": (
            "Nenhum robo aliado. Caso limite: com ally_robots vazio a acao "
            "devolve TaskStatus.RUNNING e nada e publicado. Serve para verificar "
            "que o sistema nao quebra nem gera comando para robo inexistente."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    # NOVOS CENÁRIOS DE KICKOFF
    "kickoff_favor": {
        "titulo": "Nosso Kickoff",
        "descricao": "Kickoff a favor no centro do campo",
        "bola": (0.0, 0.0),
        "comando": ("KICKOFF", "BLUE"),
        "azuis": [
            (0, -2200.0, 0.0, 0.0),     # Robo 0: Goleiro na nossa meta
            (1, -300.0, 0.0, 0.0),      # Robo 1: Cobrador atras da bola
            (2, -1000.0, 600.0, 0.0),   # Robo 2: Apoio
        ],
        "amarelos": [
            (0, 2200.0, 0.0, 3.14),     # Robo 0: Goleiro deles
            (1, 1000.0, -500.0, 3.14),  # Robo 1: Defesa amarela
            (2, 1000.0, 500.0, 3.14),   # Robo 2: Defesa amarela
        ],
    },
    "kickoff_contra": {
        "titulo": "Kickoff Adversario",
        "descricao": "Kickoff deles no centro do campo",
        "bola": (0.0, 0.0),
        "comando": ("KICKOFF", "YELLOW"),
        "azuis": [
            (0, -2200.0, 0.0, 0.0),     # Robo 0: Goleiro na nossa meta
            (1, -750.0, -400.0, 0.0),   # Robo 1: Defesa azul (fora do raio central)
            (2, -750.0, 400.0, 0.0),    # Robo 2: Defesa azul
        ],
        "amarelos": [
            (0, 2200.0, 0.0, 3.14),     # Robo 0: Goleiro deles
            (1, 300.0, 0.0, 3.14),      # Robo 1: Cobrador amarelo
            (2, 1000.0, 600.0, 3.14),   # Robo 2: Apoio amarelo
        ],
    },
}


# ==========================================================================
#  grSim - teleporte de bola e robos
# ==========================================================================
def _carregar_protobuf():
    """Importa o grSim_Packet_pb2 que ja vem gerado no ssl-VICE."""
    base = "/root/ssl-VICE/install/grsim_messenger/lib/python3.10/site-packages"
    caminho = os.path.join(base, "grsim_messenger", "protobuf")
    if caminho not in sys.path:
        sys.path.insert(0, caminho)
    import grSim_Packet_pb2

    return grSim_Packet_pb2


ROBOS_POR_TIME = 11   # "Robots Count" do grSim

FONTE_ROS = (
    "source /opt/ros/humble/setup.bash && "
    "source /root/ssl-VICE/install/local_setup.bash"
)


# NOTA sobre reiniciar o visionNode
#
# Isso e feito pelo ararabots.sh (cmd_cenario), de FORA deste processo, entre
# o "posicionar" e o "gravar". Nao da para fazer aqui dentro: se este script
# subir o visionNode como processo filho, o DDS nao consegue descobri-lo a partir
# do proprio pai - a assinatura fica muda (0 mensagens), embora qualquer outro
# processo leia o mesmo topico normalmente.
#
# Por que reiniciar: o filtro de Kalman absorve um teleporte de 5 m bem devagar
# (e faz bem - bola nenhuma pula assim na vida real). Sem o node novo, ele
# seguiria publicando a posicao ANTIGA por dezenas de segundos e a estrategia
# decidiria em cima dela. Com o rastreador zerado, o objeto nasce ja na posicao
# certa. Matar o visionNode nao derruba o resto: o sim_one.py nao tem on_exit.


_SOCK_GRSIM = None


def _sock_grsim():
    """Um socket UDP so, reaproveitado. Abrir e fechar a 10 Hz nao e de graca."""
    global _SOCK_GRSIM
    if _SOCK_GRSIM is None:
        _SOCK_GRSIM = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return _SOCK_GRSIM


def _enviar_agora(pacote):
    """Envia UMA vez, sem dormir. Para comandos que se repetem sozinhos.

    POR QUE ISTO EXISTE - defeito do INSTRUMENTO, nao da estrategia
    ---------------------------------------------------------------
    O envio antigo era sempre 3 datagramas com sleep(0,05) entre eles: 150 ms
    de bloqueio por chamada. Isso e correto para um TELEPORTE (acontece uma vez
    e nao pode se perder), mas o laco de 'rodar' chama comandar_amarelos a
    10 Hz - e durante esses 150 ms o processo NAO faz spin no rclpy.

    Consequencia medida no proprio laco: cada volta custava ~0,25 s (0,10 de
    spin + 0,15 dormindo), entao ficavamos ~60% do tempo surdos. As amostras de
    visao que chegavam nesse intervalo se acumulavam na fila do DDS e eram
    carimbadas com o time.time() do momento em que finalmente foram tratadas.

    Isso envenena exatamente a medicao que queremos fazer: a velocidade da bola
    logo apos o disparo depende de dt entre quadros consecutivos, e o dt estava
    sendo inventado pelo atraso do nosso proprio laco.

    Um comando de velocidade perdido nao custa nada - vem outro em 100 ms.
    """
    _sock_grsim().sendto(pacote.SerializeToString(), (GRSIM_HOST, GRSIM_PORT))


def _enviar(pacote):
    """Envia repetido, para o que NAO pode se perder (teleporte, replacement).

    UDP nao garante entrega e o grSim aplica o ultimo que chegar. Aqui o
    bloqueio de 150 ms nao atrapalha: so acontece entre etapas, nunca durante
    a gravacao.
    """
    for _ in range(3):
        _sock_grsim().sendto(pacote.SerializeToString(), (GRSIM_HOST, GRSIM_PORT))
        time.sleep(0.05)


def _cmd_amarelo(c, rid, ori, vx_campo, vy_campo, girar=0.0, chute=0.0):
    """Adiciona ao pacote um comando de velocidade para UM robo amarelo.

    CONVERSAO DE REFERENCIAL - o erro que isto corrige
    -------------------------------------------------
    grSim espera veltangent/velnormal no referencial DO ROBO (eixo do corpo),
    nao no do campo. O controlador anterior mandava a direcao de campo direto, e
    como os amarelos sao posicionados com angulo 180 graus, todo comando saia
    invertido: o adversario andava para tras do que deveria.
    """
    ca, sa = math.cos(ori), math.sin(ori)
    rc = c.robot_commands.add()
    rc.id = int(rid)
    rc.wheelsspeed = False
    rc.veltangent = float(vx_campo * ca + vy_campo * sa)
    rc.velnormal = float(-vx_campo * sa + vy_campo * ca)
    rc.velangular = float(girar)
    rc.kickspeedx = float(chute)
    rc.kickspeedz = 0.0
    rc.spinner = False


def _ir_para(rx, ry, ax, ay, vel=1.2, freio=400.0):
    """Vetor de velocidade de (rx,ry) ate (ax,ay), freando na chegada.

    O freio proporcional existe pelo mesmo motivo da tatica: sem
    desaceleracao o robo passa do alvo e volta, oscilando para sempre.
    """
    dx, dy = ax - rx, ay - ry
    d = math.hypot(dx, dy)
    if d < 20.0:
        return 0.0, 0.0
    v = vel * min(1.0, d / freio)
    return dx / d * v, dy / d * v


def comandar_amarelos(bola, amarelos, azuis=None, modo="nossa_falta", bola_vel=None):
    """Adversario com PAPEIS, em vez de um robo solto atacando a bola."""
    if not amarelos:
        return
    azuis = azuis or {}
    pb = _carregar_protobuf()
    bx, by = bola
    pacote = pb.grSim_Packet()
    c = pacote.commands
    c.timestamp = 0.0
    c.isteamyellow = True

    # a meta DELES fica em +x (atacamos +x), a nossa em -x
    meta_deles_x, nossa_meta_x = GOL_X, -GOL_X
    ids = sorted(amarelos)

    # ------------------------------------------------------------- goleiro
    if 0 in amarelos:
        gx, gy, gori = amarelos[0]
        alvo_x = meta_deles_x - 120.0

        bvx, bvy = bola_vel if bola_vel else (0.0, 0.0)
        alvo_y = by
        if bvx > 300.0:
            t = max(0.0, (alvo_x - bx) / bvx)
            if t < 3.0:
                alvo_y = by + bvy * t
        alvo_y = max(-GOL_MEIA_LARGURA + 60.0, min(GOL_MEIA_LARGURA - 60.0, alvo_y))

        vx, vy = _ir_para(gx, gy, alvo_x, alvo_y, vel=2.5, freio=120.0)
        _cmd_amarelo(c, 0, gori, vx, vy)

    linha = [r for r in ids if r != 0]

    # ------------------------------------------------------------- nosso KICKOFF (falta azul)
    if modo == "nosso_kickoff" and linha:
        # Robôs amarelos defendem: mantêm-se no campo deles (x > 750 mm) fora do círculo central
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            lado = 1.0 if ordem % 2 == 0 else -1.0
            ax = 850.0 + (ordem // 2) * 350.0
            ay = lado * (600.0 + (ordem // 2) * 450.0)
            vx, vy = _ir_para(rx, ry, ax, ay, vel=1.2)
            _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- KICKOFF deles (falta amarela)
    if modo == "kickoff_deles" and linha:
        cobrador = min(linha, key=lambda r: math.hypot(amarelos[r][0] - bx, amarelos[r][1] - by))
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            if rid == cobrador:
                # Cobrador se posiciona em +x (atrás da bola) apontando para o nosso gol (-x)
                dist_bola = math.hypot(rx - bx, ry - by)
                atras_x, atras_y = bx + 220.0, by
                perto = dist_bola < 140.0
                alvo = (bx, by) if math.hypot(rx - atras_x, ry - atras_y) < 150.0 else (atras_x, atras_y)
                vx, vy = _ir_para(rx, ry, alvo[0], alvo[1], vel=0.9, freio=250.0)
                _cmd_amarelo(c, rid, rori, vx, vy, chute=5.0 if perto else 0.0)
            else:
                # Demais robôs aguardam o passe na metade do seu campo
                lado = 1.0 if ordem % 2 == 0 else -1.0
                ax = 1100.0
                ay = lado * (750.0 + ordem * 300.0)
                vx, vy = _ir_para(rx, ry, ax, ay, vel=1.0)
                _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- falta DELES
    if modo == "falta_deles" and linha:
        cobrador = min(linha, key=lambda r: math.hypot(amarelos[r][0] - bx,
                                                       amarelos[r][1] - by))
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            if rid == cobrador:
                dx, dy = nossa_meta_x - bx, 0.0 - by
                n = math.hypot(dx, dy) or 1.0
                ux, uy = dx / n, dy / n
                atras_x, atras_y = bx - ux * 260.0, by - uy * 260.0
                dist_bola = math.hypot(rx - bx, ry - by)
                proj = (rx - bx) * ux + (ry - by) * uy
                if proj > -80.0:
                    ang = math.atan2(ry - by, rx - bx) + 0.7
                    raio = max(600.0, dist_bola)
                    alvo = (bx + raio * math.cos(ang), by + raio * math.sin(ang))
                    vx, vy = _ir_para(rx, ry, alvo[0], alvo[1], vel=1.1)
                    _cmd_amarelo(c, rid, rori, vx, vy)
                else:
                    perto = dist_bola < 140.0
                    alvo = (bx, by) if math.hypot(rx - atras_x, ry - atras_y) < 150.0 \
                        else (atras_x, atras_y)
                    vx, vy = _ir_para(rx, ry, alvo[0], alvo[1], vel=0.9, freio=250.0)
                    _cmd_amarelo(c, rid, rori, vx, vy, chute=5.0 if perto else 0.0)
            else:
                lado = 1.0 if ordem % 2 == 0 else -1.0
                ax = bx - 900.0
                ay = max(-2400.0, min(2400.0, by + lado * 900.0))
                vx, vy = _ir_para(rx, ry, ax, ay, vel=1.0)
                _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- nossa falta
    if modo == "nossa_falta" and linha:
        dx, dy = meta_deles_x - bx, 0.0 - by
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        px, py = -uy, ux
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            lado = (ordem // 2 + 1) * (1.0 if ordem % 2 == 0 else -1.0)
            ax = bx + ux * 700.0 + px * lado * 180.0
            ay = by + uy * 700.0 + py * lado * 180.0
            if math.hypot(ax - bx, ay - by) < 550.0:
                ax, ay = bx + ux * 600.0, by + uy * 600.0
            vx, vy = _ir_para(rx, ry, ax, ay, vel=1.2)
            _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- jogo normal
    for ordem, rid in enumerate(linha):
        rx, ry, rori = amarelos[rid]
        candidatos = sorted(azuis.items(), key=lambda kv: abs(meta_deles_x - kv[1][0]))
        if ordem < len(candidatos):
            azul = candidatos[ordem][1]
            dx, dy = meta_deles_x - azul[0], 0.0 - azul[1]
            n = math.hypot(dx, dy) or 1.0
            ax, ay = azul[0] + dx / n * 350.0, azul[1] + dy / n * 350.0
        else:
            ax, ay = meta_deles_x - 1500.0, 0.0
        vx, vy = _ir_para(rx, ry, ax, ay, vel=1.2)
        _cmd_amarelo(c, rid, rori, vx, vy)
    _enviar_agora(pacote)


def mover_bola(x, y):
    """Move so a bola, sem tocar nos robos."""
    pb = _carregar_protobuf()
    pacote = pb.grSim_Packet()
    pacote.replacement.ball.x = x / 1000.0
    pacote.replacement.ball.y = y / 1000.0
    pacote.replacement.ball.vx = 0.0
    pacote.replacement.ball.vy = 0.0
    _enviar(pacote)


def posicionar(cenario):
    """Monta o cenario no grSim: bola, robos usados e o resto DESLIGADO.

    grSim espera METROS e angulo em GRAUS.

    Todo robo que o cenario nao lista entra com turnon=False e estacionado fora
    do campo. Isso importa: sslworld.cpp:1102 pula os robos desligados ao montar
    o pacote de visao, entao eles somem do jogo - nao aparecem no /visionTopic,
    nao viram aliados nem obstaculos, e nao poluem o relatorio. Sem isso os 22
    robos ficam em campo e fica impossivel enxergar o que a jogada fez.
    """
    pb = _carregar_protobuf()
    pacote = pb.grSim_Packet()
    rep = pacote.replacement

    bx, by = cenario["bola"]
    rep.ball.x = bx / 1000.0
    rep.ball.y = by / 1000.0
    rep.ball.vx = 0.0
    rep.ball.vy = 0.0

    for time_amarelo, chave in ((False, "azuis"), (True, "amarelos")):
        usados = {rid: (x, y, d) for rid, x, y, d in cenario.get(chave, [])}
        for rid in range(ROBOS_POR_TIME):
            r = rep.robots.add()
            r.id = rid
            r.yellowteam = time_amarelo
            if rid in usados:
                x, y, direcao = usados[rid]
                r.x = x / 1000.0
                r.y = y / 1000.0
                r.dir = float(direcao)
                r.turnon = True
            else:
                # Estacionado atras da linha de fundo e desligado.
                r.x = 6.0 if time_amarelo else -6.0
                r.y = -3.6 + rid * 0.35
                r.dir = 0.0
                r.turnon = False

    _enviar(pacote)


# ==========================================================================
#  game-controller - cliente WebSocket minimo (sem dependencias externas)
# ==========================================================================
def enviar_comando_arbitro(tipo, time_cor="UNKNOWN"):
    sock = socket.create_connection((GC_HOST, GC_PORT), timeout=5)
    try:
        chave = base64.b64encode(os.urandom(16)).decode()
        req = (
            f"GET /api/control HTTP/1.1\r\n"
            f"Host: {GC_HOST}:{GC_PORT}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {chave}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            pedaco = sock.recv(4096)
            if not pedaco:
                raise RuntimeError("conexao fechada no handshake")
            resp += pedaco

        msg = json.dumps(
            {
                "change": {
                    "origin": "cenarios-freekick",
                    "newCommandChange": {
                        "command": {"type": tipo, "forTeam": time_cor}
                    },
                }
            }
        ).encode()

        cabecalho = bytearray([0x81])
        n = len(msg)
        if n < 126:
            cabecalho.append(0x80 | n)
        else:
            cabecalho.append(0x80 | 126)
            cabecalho += struct.pack(">H", n)
        mascara = os.urandom(4)
        cabecalho += mascara
        sock.sendall(
            bytes(cabecalho) + bytes(b ^ mascara[i % 4] for i, b in enumerate(msg))
        )
        sock.settimeout(1.0)
        try:
            sock.recv(4096)
        except socket.timeout:
            pass
    finally:
        sock.close()


# ==========================================================================
#  Gravacao via ROS 2
# ==========================================================================
PORTA_STATUS_AZUL = 30011   # "Blue Team status send port" do ~/.grsim.xml


def _ler_varint(buf, i):
    """Le um varint do protobuf. Devolve (valor, proximo indice)."""
    val = 0
    desloc = 0
    while i < len(buf):
        b = buf[i]
        i += 1
        val |= (b & 0x7F) << desloc
        if not (b & 0x80):
            return val, i
        desloc += 7
    return val, i


def _decodificar_status(buf):
    """Decodifica um Robots_Status do grSim, sem depender do protoc.

    POR QUE DECODIFICAR NA MAO
    --------------------------
    O grSim_Robotstatus.proto existe no fonte do grSim, mas o ssl-VICE so
    compila Commands, Packet e Replacement - nao ha modulo pronto para importar
    dentro do container. A mensagem tem quatro campos escalares, entao o parser
    manual cabe em vinte linhas e evita adicionar uma dependencia de build ao
    ambiente de testes de todo mundo.

    Robots_Status { repeated Robot_Status robots_status = 1 }
    Robot_Status  { int32 robot_id=1; bool infrared=2; bool flat_kick=3;
                    bool chip_kick=4 }

    Devolve: [{"id":n, "infrared":bool, "flat_kick":bool, "chip_kick":bool}, ...]
    """
    saida = []
    i = 0
    while i < len(buf):
        chave, i = _ler_varint(buf, i)
        campo, tipo = chave >> 3, chave & 0x07
        if campo != 1 or tipo != 2:
            break
        tam, i = _ler_varint(buf, i)
        sub, i = buf[i:i + tam], i + tam
        r = {"id": -1, "infrared": False, "flat_kick": False, "chip_kick": False}
        j = 0
        while j < len(sub):
            k, j = _ler_varint(sub, j)
            c, t = k >> 3, k & 0x07
            if t != 0:
                break
            v, j = _ler_varint(sub, j)
            if c == 1:
                r["id"] = v
            elif c == 2:
                r["infrared"] = bool(v)
            elif c == 3:
                r["flat_kick"] = bool(v)
            elif c == 4:
                r["chip_kick"] = bool(v)
        saida.append(r)
    return saida


IP_VISAO_GRSIM = "224.5.23.2"   # mesmo grupo que o sim_one.py passa ao visionNode
PORTA_VISAO_GRSIM = 10020


def _abrir_escuta_visao_crua():
    """Escuta a visao do grSim DIRETO, por fora do tracker do time.

    POR QUE ISTO E NECESSARIO
    -------------------------
    O /visionTopic nao carrega o que o grSim viu: carrega o que o filtro de
    Kalman do tracker ACREDITA. E medimos que, no voo da bola, essas duas
    coisas sao muito diferentes - a bola ficou parada em (2500,0) no topico
    por 1,2 s DEPOIS de o grSim confirmar o disparo.

    Qualquer medicao de velocidade de saida feita em cima do topico mede o
    filtro, nao a bola. Aqui pegamos o SSL_DetectionFrame antes de qualquer
    filtragem, com o t_capture do proprio simulador - que e o unico relogio
    confiavel que temos, ja que o nosso e o do laco de gravacao.

    Isto e so leitura de um grupo multicast que ja esta sendo transmitido:
    nao muda nada no ambiente e nao concorre com o visionNode (os dois
    recebem, e para isso serve o SO_REUSEADDR).

    O HANDOVER §12 ja recomendava isto em outras palavras: "confirme lendo a
    verdade direto do multicast do grSim antes de concluir qualquer coisa".
    """
    try:
        from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
    except ImportError as e:
        print(f"   (sem visao crua: {e})")
        return None, None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", PORTA_VISAO_GRSIM))
        mreq = struct.pack("4sl", socket.inet_aton(IP_VISAO_GRSIM),
                           socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        s.setblocking(False)
        return s, SSL_WrapperPacket
    except OSError as e:
        print(f"   (sem visao crua: porta {PORTA_VISAO_GRSIM} {e})")
        return None, None


def _abrir_escuta_status():
    """Escuta o estado REAL do chutador que o grSim devolve.

    POR QUE ISTO VALE MAIS QUE QUALQUER INFERENCIA
    ----------------------------------------------
    Ate aqui o teste so sabia que a ESTRATEGIA pediu chute (campo 'kick' no
    /commandTopic). Se o disparo aconteceu de fato, e quando, era deduzido do
    pico de velocidade da bola - o mesmo pico que ja nos enganou uma vez, quando
    26584 mm/s de uma colisao foram lidos como chute (HANDOVER §6.8).

    O grSim responde a verdade. Em sslworld.cpp:678-692, sempre que 'infrared'
    (bola encostada na placa) ou o KickStatus mudam, ele monta um Robots_Status
    e manda para o endereco de quem enviou o comando, na porta fixa
    BlueStatusSendPort (30011 no nosso XML). Quem envia os comandos azuis e o
    grsim_publisher_node, de 127.0.0.1 - e o container 'vice' usa rede 'host',
    entao o datagrama chega em 127.0.0.1:30011 e podemos simplesmente escutar.

    Nao interferimos em nada: so lemos. Nao mandamos comando azul nenhum, o que
    pararia o robo (sslworld.cpp:613 aplica setSpeed do pacote recebido).

    Se a porta estiver ocupada, o teste segue sem esse dado, apenas com a
    medicao por visao.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", PORTA_STATUS_AZUL))
        s.setblocking(False)
        return s
    except OSError as e:
        print(f"   (sem status do chutador: porta {PORTA_STATUS_AZUL} {e})")
        return None


def _criar_gravador():
    """Cria o node que escuta visao e comandos. rclpy ja deve estar inicializado."""
    from rclpy.node import Node
    from system_interfaces.msg import VisionMessage, TeamCommand

    class Gravador(Node):
        def __init__(self):
            super().__init__("gravador_cenarios")
            self.amostras = []
            self.kick_por_robo = {}
            self.gravando = False
            self.bola = None
            self.amarelos = {}
            self.gol = None          # "nosso", "contra" ou None
            # velocidade de cada aliado, usada por esperar_assentar
            self.velocidades = {}
            self.bola_vel = (0.0, 0.0)
            self.contato = {}
            # Relogio MONOTONICO: o dt entre quadros e a base da medicao de
            # velocidade da bola, e time.time() pode dar passo para tras se o
            # NTP ajustar o relogio no meio da gravacao.
            self.t0 = time.monotonic()
            # Instante em que a ESTRATEGIA pediu chute pela primeira vez.
            self.t_pedido_chute = {}
            # Eventos vindos do proprio grSim (verdade do simulador).
            self.eventos_chutador = []
            self.sock_status = _abrir_escuta_status()
            self.estado_chutador = {}
            # Visao crua do grSim, por fora do tracker: [(t_capture, x, y, t_nosso)]
            self.sock_visao, self._WrapperPacket = _abrir_escuta_visao_crua()
            self.visao_crua = []
            self.robos_crus = []
            # Todas as transicoes do pedido de chute, nao so a primeira.
            self.janelas_kick = []
            self._kick_anterior = {}
            self.create_subscription(VisionMessage, "visionTopic", self._visao, 10)
            self.create_subscription(VisionMessage, "visionTopic", self._contato, 10)
            self.create_subscription(TeamCommand, "commandTopic", self._comando, 10)
            # setpoint que o driver esta perseguindo, em mm. E o que diz se a
            # ancora dele ja voltou a coincidir com a realidade.
            self.setpoints = {}
            self.alvos = []
            self.create_subscription(ControlCommand, "control_command",
                                     self._setpoint, 10)

        def _contato(self, msg):
            """Mede a geometria do CHUTADOR no ponto de maior aproximacao.

            Reproduz robot.cpp:120-128: a bola precisa estar a menos de 31,5 mm
            da placa (xx) e 40 mm do eixo do corpo (yy). Sem medir isto, um
            teste que falha nao distingue "chegou torto" de "nao chegou".
            """
            if not msg.balls or not msg.blue_robots:
                return
            b = msg.balls[0]
            for r in msg.blue_robots:
                d = math.hypot(b.position_x - r.position_x,
                               b.position_y - r.position_y)
                if d >= self.contato.get(r.id, (1e9,))[0]:
                    continue
                dx, dy = math.cos(r.orientation), math.sin(r.orientation)
                kx = r.position_x + 75.5 * dx
                ky = r.position_y + 75.5 * dy
                ex, ey = kx - b.position_x, ky - b.position_y
                xx = abs(ex * dx + ey * dy)
                yy = abs(-ex * dy + ey * dx)
                self.contato[r.id] = (d, xx, yy)

        def _setpoint(self, msg):
            for c in msg.command:
                alvo = (c.position_x * 1000.0, c.position_y * 1000.0)
                self.setpoints[c.id] = alvo
                # SERIE TEMPORAL DO ALVO.
                #
                # Sem ela nao da para separar as duas explicacoes do vai-e-vem:
                # a estrategia mandando o robo para lugares diferentes a cada
                # ciclo, ou o robo nao conseguindo seguir um alvo estavel. Uma
                # e problema de estrategia, a outra da cadeia de controle - e
                # ate agora escolhemos entre elas no palpite.
                if self.gravando:
                    # Guarda tambem a VELOCIDADE do setpoint. control.py usa ela
                    # como feedforward direto (output = feedforward + kp*erro),
                    # entao ela pode dominar o comando: se a trajetoria carrega
                    # uma velocidade "errada", o robo anda naquela direcao
                    # independente do erro de posicao. Sem gravar isto nao da
                    # para separar as duas hipoteses.
                    self.alvos.append((round(time.monotonic() - self.t0, 4),
                                       int(c.id), alvo[0], alvo[1],
                                       float(c.velocity_x) * 1000.0,
                                       float(c.velocity_y) * 1000.0))

        def _drenar_visao_crua(self):
            """Le os quadros de visao do grSim sem passar pelo tracker."""
            if self.sock_visao is None:
                return
            while True:
                try:
                    dados, _ = self.sock_visao.recvfrom(65536)
                except (BlockingIOError, OSError):
                    return
                if not self.gravando:
                    continue
                try:
                    pkt = self._WrapperPacket()
                    pkt.ParseFromString(dados)
                except Exception:
                    continue
                if not pkt.HasField("detection") or not pkt.detection.balls:
                    continue
                b = pkt.detection.balls[0]
                tc = round(float(pkt.detection.t_capture), 6)
                self.visao_crua.append((
                    tc, float(b.x), float(b.y),
                    round(time.monotonic() - self.t0, 4),
                ))
                # Os NOSSOS robos, tambem crus. Sem a orientacao verdadeira nao
                # da para saber se a janela do chutador esteve aberta - e a
                # orientacao do topico e justamente a que atrasa 17-23 graus.
                for r in pkt.detection.robots_blue:
                    self.robos_crus.append((tc, int(r.robot_id), float(r.x),
                                            float(r.y), float(r.orientation)))

        def _drenar_status(self):
            """Le tudo que o grSim mandou desde a ultima vez, sem bloquear."""
            self._drenar_visao_crua()
            if self.sock_status is None:
                return
            while True:
                try:
                    dados, _ = self.sock_status.recvfrom(4096)
                except (BlockingIOError, OSError):
                    return
                agora = time.monotonic() - self.t0
                for r in _decodificar_status(dados):
                    antes = self.estado_chutador.get(r["id"])
                    novo = (r["infrared"], r["flat_kick"], r["chip_kick"])
                    self.estado_chutador[r["id"]] = novo
                    if not self.gravando or antes == novo:
                        continue
                    self.eventos_chutador.append({
                        "t": round(agora, 4),
                        "id": r["id"],
                        "infrared": r["infrared"],
                        "flat_kick": r["flat_kick"],
                        "chip_kick": r["chip_kick"],
                    })

        def _visao(self, msg):
            self._drenar_status()
            if msg.balls:
                b = msg.balls[0]
                self.bola = (b.position_x, b.position_y)
                # o goleiro precisa da VELOCIDADE para ter reflexo: so a
                # posicao o faz correr atras da bola em vez de interceptar
                self.bola_vel = (float(getattr(b, "velocity_x", 0.0) or 0.0),
                                 float(getattr(b, "velocity_y", 0.0) or 0.0))
                # Gol: a bola cruzou a linha dentro da largura da meta?
                if self.gravando and self.gol is None and abs(b.position_y) <= GOL_MEIA_LARGURA:
                    if b.position_x >= GOL_X:
                        self.gol = "nosso"
                    elif b.position_x <= -GOL_X:
                        self.gol = "contra"
            # guarda tambem a ORIENTACAO: os comandos de velocidade do grSim
            # sao no referencial DO ROBO, entao sem o angulo nao da para
            # converter uma direcao de campo em veltangent/velnormal.
            self.amarelos = {r.id: (r.position_x, r.position_y, r.orientation)
                             for r in msg.yellow_robots}
            self.azuis_pos = {r.id: (r.position_x, r.position_y)
                              for r in msg.blue_robots}
            self.velocidades = {
                r.id: math.hypot(r.velocity_x, r.velocity_y)
                for r in msg.blue_robots
            }
            if not self.gravando:
                return
            self.amostras.append(
                {
                    # 4 casas: com a bola a 6 m/s, 1 ms vale 6 mm. Arredondar em
                    # 3 casas ja era ruido comparavel ao que queremos medir.
                    "t": round(time.monotonic() - self.t0, 4),
                    "bola": [
                        # vx/vy vem do Kalman do tracker: suavizado, atrasa no
                        # degrau do disparo. Fica gravado para comparacao, mas
                        # quem manda na medicao e a derivada das posicoes.
                        {"x": b.position_x, "y": b.position_y,
                         "vx": float(getattr(b, "velocity_x", 0.0) or 0.0),
                         "vy": float(getattr(b, "velocity_y", 0.0) or 0.0)}
                        for b in msg.balls
                    ],
                    "azuis": [
                        {
                            "id": r.id,
                            "x": r.position_x,
                            "y": r.position_y,
                            "ang": r.orientation,
                        }
                        for r in msg.blue_robots
                    ],
                    "amarelos": [
                        {"id": r.id, "x": r.position_x, "y": r.position_y}
                        for r in msg.yellow_robots
                    ],
                }
            )

        def _comando(self, msg):
            if not self.gravando:
                return
            for r in msg.robots:
                anterior = self.kick_por_robo.get(r.robot_id, 0.0)
                self.kick_por_robo[r.robot_id] = max(anterior, float(r.kick))
                # Instante do PRIMEIRO pedido de chute. E a referencia temporal
                # para procurar a partida da bola: sem ela, achar "o quadro
                # seguinte ao disparo" seria adivinhacao.
                if float(r.kick) > 0.0 and r.robot_id not in self.t_pedido_chute:
                    self.t_pedido_chute[r.robot_id] = round(
                        time.monotonic() - self.t0, 4)
                # e TODAS as trocas liga/desliga, para saber em que intervalos a
                # estrategia estava de fato armada
                armado = float(r.kick) > 0.0
                if self._kick_anterior.get(r.robot_id) != armado:
                    self._kick_anterior[r.robot_id] = armado
                    self.janelas_kick.append(
                        (round(time.monotonic() - self.t0, 4), r.robot_id, armado))

    return Gravador()


#  Limiares de leitura da velocidade da bola (HANDOVER §6.8, medidos)
CENTRO_ATE_PLACA_MM = 75.5   # mesmo valor usado no _contato do gravador
LIM_XX_GRSIM = 31.5          # KickerThickness*2 + BallRadius (robot.cpp:128)
LIM_YY_GRSIM = 40.0          # KickerWidth/2

VEL_EMPURRAO = 2500.0    # ate aqui e empurrao de corpo, nao chute
VEL_CHUTE_MIN = 4000.0   # dai para cima houve disparo de verdade
VEL_ABSURDA = 9000.0     # acima do teto fisico do grSim: colisao ou glitch
VEL_PARTIU = 1000.0      # a partir daqui consideramos que a bola partiu


def energia_do_chute(amostras, eventos, t_pedido, visao_crua=None):
    """Mede a velocidade com que a bola REALMENTE sai, quadro a quadro.

    A PERGUNTA QUE ISTO RESPONDE (HANDOVER §24, pendencia 1)
    --------------------------------------------------------
    Com o chute disparando e a bola bem centrada, ela percorre 1306-1679 mm
    quando 6,4 m/s deveriam render 2362 mm. Faltam ~40% e nunca soubemos de
    onde: o handover registra que o passo decisivo - "registrar a velocidade da
    bola no quadro seguinte ao disparo" - nunca foi dado, e que estivemos
    inferindo pela distancia percorrida.

    Sao duas explicacoes com previsoes numericas OPOSTAS:

      a) a bola sai a ~6,4 m/s e perde energia no caminho -> o alcance e
         fisico, e mexer na forca nao resolve nada;
      b) a bola sai a ~5,3 m/s -> nao e atrito, e o disparo que ja nasce fraco,
         e a correcao e de forca.

    Uma execucao separa as duas.

    POR QUE NAO USAR O vx/vy DA VISAO COMO MEDIDA PRINCIPAL
    -------------------------------------------------------
    Aquilo vem do filtro de Kalman do tracker, que existe justamente para
    suavizar degraus - e um chute e o degrau mais brusco que a bola sofre. Ele
    vai reportar a subida com atraso e amortecida. A derivada das POSICOES
    entre quadros consecutivos nao tem esse vies. Gravamos os dois e mostramos
    lado a lado; se discordarem muito, e o filtro atrasando, nao a medida.

    JANELA CURTA, DE PROPOSITO
    --------------------------
    A desaceleracao medida em alta velocidade e ~8,9 m/s² (§19), ou seja
    ~0,15 m/s perdidos por quadro a 60 Hz. Uma media longa subestimaria a
    saida; por isso a estimativa principal usa ~50 ms logo apos a partida.
    """
    # FONTE. Preferimos sempre a visao crua do grSim: ela tem o t_capture do
    # proprio simulador (relogio confiavel) e nao passou pelo filtro de Kalman
    # do tracker, que mede-se atrasar o voo da bola em centenas de milimetros.
    # O /visionTopic fica como reserva, e o relatorio diz qual foi usada.
    if visao_crua and len(visao_crua) >= 3:
        fonte = "visao crua do grSim (sem tracker, relogio t_capture)"
        # O grSim emite um quadro por camera. A bola aparece na camera cuja
        # regiao a contem, e ao cruzar a fronteira ela pode ser reportada por
        # DUAS no mesmo t_capture. Ordenar e ficar com uma leitura por instante
        # evita ler essa duplicata como um salto de posicao.
        vistos, limpo = set(), []
        for tc, x, y, _tn in sorted(visao_crua):
            if tc in vistos:
                continue
            vistos.add(tc)
            limpo.append((tc, {"x": x, "y": y}))
        faixa = limpo
        # Referencia temporal, no NOSSO relogio, traduzida para t_capture pelo
        # quadro cru mais proximo. Preferimos o instante em que o grSim disse
        # que DISPAROU (flat_kick) ao instante em que a estrategia PEDIU: entre
        # um e outro pode haver segundos de aproximacao, e o primeiro pedido
        # costuma cair antes de qualquer contato.
        disparo = next((x["t"] for x in (eventos or []) if x.get("flat_kick")), None)
        alvo = disparo if disparo is not None else (
            min(t_pedido.values()) if t_pedido else None)
        if alvo is not None:
            tc_ref = min(visao_crua, key=lambda r: abs(r[3] - alvo))[0]
            t_pedido = {"ref": tc_ref}
    else:
        fonte = "/visionTopic (filtrado pelo tracker - leia com desconfianca)"
        faixa = [(a["t"], a["bola"][0]) for a in amostras if a.get("bola")]
    if len(faixa) < 3:
        return None

    # velocidade quadro a quadro, pela derivada das posicoes
    quadros = []
    for (t0, b0), (t1, b1) in zip(faixa, faixa[1:]):
        dt = t1 - t0
        if dt <= 1e-4:
            continue
        d = math.hypot(b1["x"] - b0["x"], b1["y"] - b0["y"])
        quadros.append((t1, d / dt, d))

    if not quadros:
        return None

    # a partida da bola: primeiro quadro claramente em movimento, nao antes do
    # pedido de chute (se houve pedido)
    t_ref = min(t_pedido.values()) if t_pedido else 0.0
    partida = None
    for i, (t, v, _) in enumerate(quadros):
        if t >= t_ref - 0.05 and v >= VEL_PARTIU:
            partida = i
            break
    if partida is None:
        return None

    primeiros = [round(v, 1) for _, v, _ in quadros[partida:partida + 4]]

    # estimativa principal: deslocamento acumulado nos ~50 ms apos a partida
    # ATENCAO ao indice: quadros[i] e o intervalo faixa[i] -> faixa[i+1], e o
    # 't' guardado nele e o do FIM do intervalo. Entao a janela comeca em
    # faixa[partida][0], nao em quadros[partida][0]. Somar a distancia de todos
    # os intervalos e dividir pela duracao medida a partir do fim do primeiro
    # conta um intervalo a mais de espaco do que de tempo - um teste sintetico
    # com 6400 mm/s injetados devolvia 8237.
    t_ini = faixa[partida][0]
    janela = [q for q in quadros[partida:] if q[0] - t_ini <= 0.05] \
        or [quadros[partida]]
    dist_janela = sum(d for _, _, d in janela)
    dur_janela = janela[-1][0] - t_ini
    v_janela = dist_janela / dur_janela if dur_janela > 1e-4 else quadros[partida][1]

    v_pico = max(v for _, v, _ in quadros)
    com_bola = [a for a in amostras if a.get("bola")]
    v_kalman = max(
        (math.hypot(a["bola"][0].get("vx", 0.0), a["bola"][0].get("vy", 0.0))
         for a in com_bola), default=0.0)

    # quanto a bola andou da partida ate parar
    # Percurso do chute: da partida ate a bola PARAR (ou ate o fim da
    # gravacao). Medir ate o ultimo quadro incluiria qualquer toque posterior
    # - o robo alcancando a bola de novo, um adversario rebatendo - e o numero
    # deixaria de ser "o alcance daquele chute".
    p_ini = faixa[partida][1]
    p_fim, t_parou = faixa[-1][1], None
    lentos = 0
    for t, v, _ in quadros[partida:]:
        if v < 150.0:
            lentos += 1
            if lentos >= 5:
                t_parou = t
                p_fim = next(b for tt, b in faixa if tt >= t)
                break
        else:
            lentos = 0
    percorrido = math.hypot(p_fim["x"] - p_ini["x"], p_fim["y"] - p_ini["y"])

    if v_pico > VEL_ABSURDA:
        veredito = "COLISAO/GLITCH (acima do teto fisico do grSim)"
    elif v_pico >= VEL_CHUTE_MIN:
        veredito = "CHUTE de verdade"
    elif v_pico >= VEL_EMPURRAO:
        veredito = "chute fraco ou raspao"
    else:
        veredito = "EMPURRAO de corpo, nao chute"

    return {
        "fonte": fonte,
        "t_pedido_chute": t_ref if t_pedido else None,
        "t_partida": round(t_ini, 4),
        "v_primeiros_quadros": primeiros,
        "v_saida_janela_50ms": round(v_janela, 1),
        "v_pico_quadro": round(v_pico, 1),
        "v_pico_kalman": round(v_kalman, 1),
        "percorrido_mm": round(percorrido, 1),
        "veredito": veredito,
        "eventos_chutador": eventos,
    }


def janela_do_chutador(visao_crua, robos_crus, janelas_kick, eventos):
    """Quantos quadros a janela do chutador esteve ABERTA, e o que faziamos neles.

    A PERGUNTA QUE ISTO RESPONDE
    ---------------------------
    Uma execucao sem gol nao distingue duas falhas opostas:

      a) o robo nunca chegou a uma geometria que permitisse chutar
         -> o problema e a APROXIMACAO;
      b) a geometria esteve boa e a estrategia nao estava armada naquele
         instante -> o problema e a DECISAO (ou a visao em que ela se apoia).

    As duas pedem correcoes em lugares diferentes, e ate agora estivemos
    escolhendo no palpite.

    Aqui reproduzimos o teste do grSim (robot.cpp:120-128) quadro a quadro, com
    posicao e orientacao CRUAS - sem o tracker, cuja orientacao atrasa 17-23
    graus, o que sozinho vale 31-42 mm de yy.

    Devolve as contagens e, quando a janela abriu sem estarmos armados, o
    tamanho da maior oportunidade perdida.
    """
    if not visao_crua or not robos_crus:
        return None
    bolas = {}
    for tc, x, y, tn in visao_crua:
        bolas.setdefault(tc, (x, y, tn))

    # intervalos em que a estrategia pediu chute, no NOSSO relogio
    armado_ate = []
    aberto = {}
    for t, rid, ligou in sorted(janelas_kick or []):
        if ligou:
            aberto[rid] = t
        elif rid in aberto:
            armado_ate.append((aberto.pop(rid), t))
    for rid, t in aberto.items():
        armado_ate.append((t, float("inf")))

    def estava_armado(tn):
        return any(a <= tn <= b for a, b in armado_ate)

    abertos, abertos_armados, seq, melhor_seq = 0, 0, 0, 0
    melhor = None
    for tc, rid, rx, ry, ori in sorted(robos_crus):
        if tc not in bolas:
            continue
        bx, by, tn = bolas[tc]
        dx, dy = math.cos(ori), math.sin(ori)
        kx = rx + CENTRO_ATE_PLACA_MM * dx
        ky = ry + CENTRO_ATE_PLACA_MM * dy
        ex, ey = kx - bx, ky - by
        xx = abs(ex * dx + ey * dy)
        yy = abs(-ex * dy + ey * dx)
        if xx < LIM_XX_GRSIM and yy < LIM_YY_GRSIM:
            abertos += 1
            seq += 1
            melhor_seq = max(melhor_seq, seq)
            if estava_armado(tn):
                abertos_armados += 1
            elif melhor is None or seq > melhor[0]:
                melhor = (seq, round(tc, 3), round(xx, 1), round(yy, 1))
        else:
            seq = 0
    return {
        "quadros_com_janela_aberta": abertos,
        "desses_com_chute_armado": abertos_armados,
        "maior_sequencia_aberta": melhor_seq,
        "maior_perdida": melhor,
        "disparou": any(x.get("flat_kick") for x in (eventos or [])),
    }


def imprimir_janela(j):
    if not j:
        print("   JANELA DO CHUTADOR: sem dados crus para avaliar")
        return
    a, ar = j["quadros_com_janela_aberta"], j["desses_com_chute_armado"]
    print("   JANELA DO CHUTADOR (teste do grSim refeito na visao crua):")
    if a == 0:
        print("     nunca abriu - a geometria nunca permitiu chutar")
        print("     -> o gargalo e a APROXIMACAO")
        return
    print(f"     abriu em {a} quadros (maior sequencia: {j['maior_sequencia_aberta']})")
    print(f"     desses, com o chute ARMADO: {ar}")
    if ar == 0:
        p = j["maior_perdida"]
        print("     -> OPORTUNIDADE PERDIDA: a geometria permitia e nao estavamos armados")
        if p:
            print(f"        melhor momento: t_capture={p[1]}  xx={p[2]}  yy={p[3]}")
        print("     -> o gargalo e a DECISAO / a visao em que ela se apoia")


_MODELO_REPLAY = r"""<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<title>Replay da cobranca</title><style>
:root{--bg:#11151a;--fg:#e8eef5;--dim:#8b98a8;--linha:#39485a;--ok:#3ddc84;--ruim:#ff6b6b;--alvo:#ffd166}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);
font:14px/1.5 ui-monospace,Menlo,Consolas,monospace}
.top{padding:10px 14px;border-bottom:1px solid var(--linha);display:flex;gap:18px;flex-wrap:wrap;align-items:center}
.badge{padding:2px 8px;border-radius:4px;font-weight:600}
.g-sim{background:var(--ok);color:#062}.g-nao{background:#2a3340;color:var(--dim)}
.wrap{padding:12px}svg{width:100%;height:auto;display:block;background:#0d2818;border-radius:6px}
.ctl{display:flex;gap:12px;align-items:center;padding:10px 14px;flex-wrap:wrap}
input[type=range]{flex:1;min-width:220px}button{background:#22303f;color:var(--fg);border:1px solid var(--linha);
border-radius:5px;padding:6px 14px;cursor:pointer;font:inherit}button:hover{background:#2c3d50}
.leg{display:flex;gap:16px;flex-wrap:wrap;padding:0 14px 12px;color:var(--dim);font-size:12px}
.leg i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:5px;vertical-align:-1px}
.num{font-variant-numeric:tabular-nums}
</style></head><body>
<div class="top">
  <b id="cen"></b>
  <span id="gol" class="badge"></span>
  <span id="disp" class="badge"></span>
  <span class="num">t = <b id="t">0.00</b> s</span>
  <span class="num">erro de rastreio: <b id="err">-</b> mm</span>
</div>
<div class="top" id="resumo" style="font-size:13px">
</div>
<div class="wrap"><svg id="campo" viewBox="-5000 -3400 10000 6800"></svg></div>
<div class="ctl">
  <button id="play">reproduzir</button>
  <button id="lento">0,25x</button>
  <input type="range" id="sl" min="0" value="0">
</div>
<div class="leg">
  <span><i style="background:#ff9f1c"></i>bola</span>
  <span><i style="background:#4da3ff"></i>nosso robo (a seta e a direcao do chutador)</span>
  <span><i style="background:var(--alvo)"></i>setpoint que o driver comandou</span>
  <span><i style="background:var(--ok)"></i>disparo do chutador (verdade do grSim)</span>
  <span>a linha amarela e o ERRO DE RASTREIO</span>
</div>
<script>
const D = /*DADOS*/;
const svg = document.getElementById('campo');
const NS = 'http://www.w3.org/2000/svg';
function el(n, at){const e=document.createElementNS(NS,n);for(const k in at)e.setAttribute(k,at[k]);return e;}
// campo Division B: 9000 x 6000, gols em x=+-4500 com 1000 de largura
svg.appendChild(el('rect',{x:-4500,y:-3000,width:9000,height:6000,fill:'none',stroke:'#3a6b52','stroke-width':20}));
svg.appendChild(el('line',{x1:0,y1:-3000,x2:0,y2:3000,stroke:'#3a6b52','stroke-width':14}));
svg.appendChild(el('circle',{cx:0,cy:0,r:500,fill:'none',stroke:'#3a6b52','stroke-width':14}));
for(const s of [-1,1]){
  svg.appendChild(el('rect',{x:s>0?4500:-4680,y:-500,width:180,height:1000,fill:'none',stroke:'#5b8f74','stroke-width':16}));
  svg.appendChild(el('rect',{x:s>0?3500:-4500,y:-1000,width:1000,height:2000,fill:'none',stroke:'#3a6b52','stroke-width':12}));
}
const trilhaB = el('path',{fill:'none',stroke:'#ff9f1c','stroke-width':16,opacity:.45}); svg.appendChild(trilhaB);
const trilhaR = el('path',{fill:'none',stroke:'#4da3ff','stroke-width':14,opacity:.35}); svg.appendChild(trilhaR);
const linhaErro = el('line',{stroke:'var(--alvo)','stroke-width':12,'stroke-dasharray':'40 30',opacity:.9}); svg.appendChild(linhaErro);
const alvo = el('g'); svg.appendChild(alvo);
alvo.appendChild(el('circle',{r:70,fill:'none',stroke:'#ffd166','stroke-width':14}));
alvo.appendChild(el('line',{x1:-110,y1:0,x2:110,y2:0,stroke:'#ffd166','stroke-width':10}));
alvo.appendChild(el('line',{x1:0,y1:-110,x2:0,y2:110,stroke:'#ffd166','stroke-width':10}));
const robo = el('g'); svg.appendChild(robo);
robo.appendChild(el('circle',{r:90,fill:'#4da3ff'}));
const seta = el('line',{x1:0,y1:0,x2:170,y2:0,stroke:'#fff','stroke-width':22,'stroke-linecap':'round'});
robo.appendChild(seta);
const bola = el('circle',{r:45,fill:'#ff9f1c'}); svg.appendChild(bola);
const marcaChute = el('circle',{r:0,fill:'none',stroke:'var(--ok)','stroke-width':22}); svg.appendChild(marcaChute);

document.getElementById('cen').textContent = D.cenario || 'execucao';
// resumo do erro de rastreio - aparece sem precisar clicar em nada
(function(){
  // usa D.quadros direto: este bloco roda ANTES da declaracao 'const Q',
  // e tocar em Q aqui levantava ReferenceError (zona morta temporal do
  // const) - o script abortava e o replay nunca comecava a tocar.
  const es=D.quadros.filter(q=>q.a&&q.r.length)
            .map(q=>Math.hypot(q.a[1]-q.r[0][1], q.a[2]-q.r[0][2])).sort((x,y)=>x-y);
  if(!es.length) return;
  const med=es[es.length>>1], p90=es[Math.floor(.9*es.length)];
  const acima=100*es.filter(x=>x>200).length/es.length;
  const d=document.getElementById('resumo');
  d.innerHTML='rastreio: mediana <b>'+med.toFixed(0)+'</b> mm &nbsp; p90 <b>'
    +p90.toFixed(0)+'</b> mm &nbsp; acima de 200 mm em <b>'+acima.toFixed(0)+'%</b> do tempo';
  d.style.color = med>150 ? 'var(--ruim)' : 'var(--ok)';
})();
const gb=document.getElementById('gol');
gb.textContent = D.gol==='nosso' ? 'GOL' : (D.gol==='contra' ? 'GOL CONTRA' : 'sem gol');
gb.className = 'badge ' + (D.gol==='nosso' ? 'g-sim':'g-nao');
const db=document.getElementById('disp');
db.textContent = D.disparou ? 'chutador DISPAROU' : 'nao disparou';
db.className = 'badge ' + (D.disparou ? 'g-sim':'g-nao');

const Q = D.quadros;
const sl = document.getElementById('sl'); sl.max = Q.length-1;
let pB='', pR='';
function desenha(i){
  const q = Q[i];
  document.getElementById('t').textContent = q.t.toFixed(2);
  bola.setAttribute('cx', q.b[0]); bola.setAttribute('cy', q.b[1]);
  pB = (i===0?'M':'L') + q.b[0] + ' ' + q.b[1] + (i===0?'':' ');
  const meu = q.r && q.r.length ? q.r[0] : null;
  if (meu){
    robo.setAttribute('transform', 'translate('+meu[1]+','+meu[2]+') rotate('+(meu[3]*180/Math.PI)+')');
    robo.style.display='';
  } else robo.style.display='none';
  if (q.a && meu){
    alvo.setAttribute('transform','translate('+q.a[1]+','+q.a[2]+')'); alvo.style.display='';
    linhaErro.setAttribute('x1',meu[1]); linhaErro.setAttribute('y1',meu[2]);
    linhaErro.setAttribute('x2',q.a[1]); linhaErro.setAttribute('y2',q.a[2]);
    linhaErro.style.display='';
    const e = Math.hypot(q.a[1]-meu[1], q.a[2]-meu[2]);
    const ee = document.getElementById('err');
    ee.textContent = e.toFixed(0);
    ee.style.color = e>200 ? 'var(--ruim)' : 'var(--ok)';
  } else { alvo.style.display='none'; linhaErro.style.display='none'; }
  const ev = D.eventos.find(x => x.flat_kick && Math.abs(x.t - q.t) < 0.25);
  marcaChute.setAttribute('r', ev ? 260 : 0);
  if (ev){ marcaChute.setAttribute('cx', q.b[0]); marcaChute.setAttribute('cy', q.b[1]); }
}
// trilhas completas, desenhadas de uma vez
trilhaB.setAttribute('d', Q.map((q,i)=>(i?'L':'M')+q.b[0]+' '+q.b[1]).join(' '));
trilhaR.setAttribute('d', Q.filter(q=>q.r&&q.r.length).map((q,i)=>(i?'L':'M')+q.r[0][1]+' '+q.r[0][2]).join(' '));

// setInterval, e nao requestAnimationFrame: o rAF nao dispara quando a pagina
// esta em aba oculta ou num painel que a renderiza sem foco - o botao alternava
// e nada acontecia na tela.
let vel=1, timer=null;
const botao=document.getElementById('play');
function avanca(){
  let i=+sl.value+1; if(i>=Q.length) i=0;
  sl.value=i; desenha(i);
}
function toca(){
  para();
  timer=setInterval(avanca, 16/vel);
  botao.textContent='pausar';
}
function para(){ if(timer){clearInterval(timer); timer=null;} botao.textContent='reproduzir'; }
botao.onclick = () => timer ? para() : toca();
document.getElementById('lento').onclick = e => {
  vel = vel===1?0.25:1; e.target.textContent = vel===1?'0,25x':'1x';
  if(timer) toca();
};
sl.onmousedown = para;
sl.oninput = () => desenha(+sl.value);
desenha(0);
toca();   // comeca tocando: nada de clicar para ver a execucao
</script></body></html>"""


def erro_de_rastreio(resultado):
    """Distancia entre o robo e o setpoint que o driver esta comandando.

    E a unica grandeza medida que separa gol de erro: as execucoes que marcaram
    tiveram mediana de 76 e 153 mm, a que falhou teve 306 mm. Como sai de UMA
    execucao, serve para iterar onde contar gols e caro demais.

    O setpoint vem do /control_command (driver.py:33, trajectory.get_state) e a
    posicao do robo vem da visao CRUA do grSim, sem passar pelo tracker.
    """
    alvos = sorted(resultado.get("alvos") or [])
    rc = sorted(resultado.get("robos_crus") or [])
    vc = sorted(resultado.get("visao_crua") or [])
    if not alvos or not rc or not vc:
        return None
    pares = [(r[3], r[0]) for r in vc]          # (t_nosso, t_capture)
    por_tc = {}
    for tc, rid, x, y, _ori in rc:
        por_tc.setdefault(rid, []).append((tc, x, y))
    erros = []
    for reg in alvos:
        tn, rid, ax, ay = reg[0], reg[1], reg[2], reg[3]
        serie = por_tc.get(rid)
        if not serie:
            continue
        tc = min(pares, key=lambda p: abs(p[0] - tn))[1]
        _t, rx, ry = min(serie, key=lambda r: abs(r[0] - tc))
        erros.append(math.hypot(ax - rx, ay - ry))
    if not erros:
        return None
    erros.sort()
    # Taxa real do laco do driver DURANTE esta execucao. Vai junto do resultado
    # de proposito: um erro de rastreio so quer dizer alguma coisa se a gente
    # souber a que taxa o laco estava rodando quando ele foi medido. O timer do
    # driver e 100 Hz; ja medimos 19 Hz com a maquina carregada e 93 Hz com ela
    # livre, e comparar numeros dessas duas situacoes nao significa nada.
    ts = [x[0] for x in alvos]
    hz = (len(ts) / (ts[-1] - ts[0])) if len(ts) > 2 and ts[-1] > ts[0] else 0.0
    return {
        "hz_controle": round(hz, 1),
        "n": len(erros),
        "mediana": round(erros[len(erros) // 2], 1),
        "p90": round(erros[int(0.9 * len(erros))], 1),
        "maximo": round(erros[-1], 1),
        "pct_acima_200": round(100.0 * sum(1 for e in erros if e > 200) / len(erros), 1),
    }


def gerar_replay(resultado, destino):
    """Reconstroi a execucao a partir da visao CRUA, num HTML que roda sozinho.

    POR QUE ISTO EXISTE
    -------------------
    Ver a execucao ao vivo e assistir por um dos dois caminhos que mentem:

      - a janela do grSim congela a fisica (glwidget.cpp:392 chama step() dentro
        do paintGL), entao o que se ve nao e o que a estrategia enfrentaria;
      - a ssl-gui desenha o /visionTopic, que e a saida do filtro de Kalman - o
        mesmo que manteve a bola parada em (2500,0) por 1,2 s DEPOIS do disparo.

    Aqui a fonte e o multicast do grSim, sem filtro nenhum, com o t_capture do
    proprio simulador. E mostra o que nenhuma visualizacao ao vivo mostra: o
    SETPOINT que o driver estava comandando, desenhado junto do robo. A distancia
    entre os dois e o erro de rastreio, que e o que separa gol de erro.
    """
    import json as _json
    # uma leitura de bola por instante (o grSim emite um quadro por camera)
    _vistos = set()
    vc = []
    for r in sorted(resultado.get("visao_crua") or []):
        if r[0] in _vistos:
            continue
        _vistos.add(r[0])
        vc.append(r)
    rc = sorted(resultado.get("robos_crus") or [])
    if not vc:
        return None

    t0 = vc[0][0]
    # mapa t_nosso -> t_capture, para trazer alvos e eventos ao mesmo relogio
    pares = [(r[3], r[0]) for r in vc]

    def para_tc(tn):
        melhor = min(pares, key=lambda p: abs(p[0] - tn))
        return melhor[1]

    # CASAR POR PROXIMIDADE, nao por igualdade de instante.
    #
    # O grSim emite um quadro por camera: a bola aparece na camera que a contem
    # e os robos nas suas. Os t_capture sao diferentes, entao casar por chave
    # exata devolve zero quadros completos - foi o que aconteceu na primeira
    # versao (1050 quadros, nenhum com robo E setpoint juntos).
    import bisect
    rob_t = sorted({t for t, _, _, _, _ in rc})
    rob_por_t = {}
    for t, rid, x, y, ori in rc:
        rob_por_t.setdefault(t, []).append([rid, round(x), round(y), round(ori, 4)])

    def robos_perto(t, tol=0.03):
        # (o mesmo robo pode vir em mais de uma camera; um por id basta)
        if not rob_t:
            return []
        i = bisect.bisect_left(rob_t, t)
        cand = [rob_t[j] for j in (i - 1, i) if 0 <= j < len(rob_t)]
        if not cand:
            return []
        melhor = min(cand, key=lambda x: abs(x - t))
        if abs(melhor - t) > tol:
            return []
        unicos = {}
        for r in rob_por_t[melhor]:
            unicos.setdefault(r[0], r)
        return list(unicos.values())

    alvos_t = sorted((para_tc(tn), [rid, round(ax), round(ay)])
                     for tn, rid, ax, ay in
                     ((r[0], r[1], r[2], r[3]) for r in (resultado.get("alvos") or [])))
    eventos = [{"t": round(para_tc(e["t"]) - t0, 4), **e}
               for e in (resultado.get("eventos_chutador") or [])]

    quadros = []
    ia, ultimo_alvo = 0, None
    for t, x, y, _tn in vc:
        while ia < len(alvos_t) and alvos_t[ia][0] <= t:
            ultimo_alvo = alvos_t[ia][1]
            ia += 1
        quadros.append({
            "t": round(t - t0, 4),
            "b": [round(x), round(y)],
            "r": robos_perto(t),
            "a": ultimo_alvo,
        })

    dados = _json.dumps({
        "quadros": quadros,
        "eventos": eventos,
        "cenario": resultado.get("cenario"),
        "gol": resultado.get("gol"),
        "gol_em": resultado.get("gol_em"),
        "disparou": resultado.get("disparou"),
        "bola_inicial": resultado.get("bola_inicial"),
    }, separators=(",", ":"))

    with open(destino, "w") as f:
        f.write(_MODELO_REPLAY.replace("/*DADOS*/", dados))
    return destino


def gol_pela_visao_crua(visao_crua):
    """Decide o gol pela visao CRUA, nao pelo topico filtrado.

    POR QUE ISTO IMPORTA PARA O CRITERIO DE APROVACAO
    -------------------------------------------------
    O criterio do Felipe e "bola cruzando x>4500 com |y|<500". Ate aqui isso
    era conferido no /visionTopic - ou seja, na saida do filtro de Kalman.

    Medimos o filtro ULTRAPASSAR a posicao real em ate 431 mm ao frear a bola
    (ele estima velocidade e continua projetando depois que ela ja parou). Com
    431 mm de ultrapassagem, uma bola que morre em x=4300 pode ser publicada
    cruzando x=4500 - e contada como gol que nao houve. O erro tambem funciona
    ao contrario, escondendo um gol de raspao.

    Contando na fonte crua, gol e gol.

    Devolve ("nosso"|"contra"|None, x, y) do primeiro cruzamento.
    """
    for tc, x, y, _tn in sorted(visao_crua or []):
        if abs(y) <= GOL_MEIA_LARGURA:
            if x >= GOL_X:
                return "nosso", x, y
            if x <= -GOL_X:
                return "contra", x, y
    return None, 0.0, 0.0


def imprimir_energia(e):
    if not e:
        print("   ENERGIA DO CHUTE: a bola nao se moveu o bastante para medir")
        return
    print("   ENERGIA DO CHUTE (derivada das posicoes):")
    print(f"     fonte: {e['fonte']}")
    if e["t_pedido_chute"] is not None:
        print(f"     estrategia pediu chute em t={e['t_pedido_chute']:.3f}s"
              f"   bola partiu em t={e['t_partida']:.3f}s")
    seq = "  ".join(f"{v:.0f}" for v in e["v_primeiros_quadros"])
    print(f"     primeiros quadros apos a partida: {seq} mm/s")
    print(f"     VELOCIDADE DE SAIDA (janela 50 ms): {e['v_saida_janela_50ms']:.0f} mm/s"
          f"  = {e['v_saida_janela_50ms']/1000.0:.2f} m/s")
    print(f"     pico quadro a quadro: {e['v_pico_quadro']:.0f} mm/s"
          f"   pico pelo Kalman: {e['v_pico_kalman']:.0f} mm/s")
    print(f"     a bola percorreu {e['percorrido_mm']:.0f} mm")
    print(f"     -> {e['veredito']}")
    ev = e.get("eventos_chutador") or []
    if ev:
        print("   CHUTADOR, pelo proprio grSim (verdade, nao inferencia):")
        for x in ev[:12]:
            marcas = []
            if x["infrared"]:
                marcas.append("bola encostada")
            if x["flat_kick"]:
                marcas.append("DISPAROU")
            if x["chip_kick"]:
                marcas.append("chip")
            print(f"     t={x['t']:6.3f}s  robo {x['id']}: "
                  f"{', '.join(marcas) if marcas else 'solto'}")
        if len(ev) > 12:
            print(f"     ... e mais {len(ev) - 12} eventos (veja o JSON)")
    else:
        print("   CHUTADOR: nenhum evento recebido do grSim "
              f"(porta {PORTA_STATUS_AZUL})")


def _girar(no, segundos):
    import rclpy

    fim = time.time() + segundos
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.05)


def esperar_assentar(no, limite=8.0, vel_max=25.0, piso=0.0, erro_max=120.0):
    """Espera o driver REANCORAR, em vez de dormir um tempo fixo.

    POR QUE ESPERA ATIVA
    --------------------
    Isto era 'durma 8 segundos'. O numero foi escolhido com folga, para o pior
    caso - e o pior caso e raro. Na maioria das execucoes os robos ja estao
    parados em 1 ou 2 segundos, e os 6 restantes eram desperdicio puro,
    multiplicado por cada repeticao de cada cenario.

    O QUE ESTA ESPERANDO DE VERDADE
    -------------------------------
    O driver cria a trajetoria de um robo uma vez e dali em diante replaneja a
    partir do setpoint do plano anterior, NUNCA da posicao medida. Depois de um
    teleporte ele fica planejando a partir de uma origem fantasma. Sob HALT, a
    tatica manda cada robo para a PROPRIA posicao lida da visao: o driver
    persegue esse alvo e, quando o setpoint dele alcanca a posicao real, a
    ancora esta corrigida.

    ERRO QUE ISTO CORRIGE
    ---------------------
    A primeira versao esperava os robos PARAREM. So que, logo depois de um
    teleporte, eles ja estao parados - nunca se moveram. A condicao era
    satisfeita em 0,3 s, o driver nao recebia o tempo de reancorar, e o
    resultado era o robo andando 112 mm em 20 s: a ancora continuava na posicao
    anterior ao teleporte. O sintoma parecia "os robos travaram".

    A condicao certa nao e o robo estar parado, e o SETPOINT DO DRIVER ter
    convergido para a posicao medida. E isso que medimos agora.
    """
    import rclpy
    inicio = time.time()
    fim = inicio + limite
    ok_desde = None
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.05)
        if time.time() - inicio < piso:
            continue
        sp = getattr(no, "setpoints", None)
        pos = getattr(no, "azuis_pos", None)
        vels = getattr(no, "velocidades", None)
        if not sp or not pos or not vels:
            continue
        comuns = [i for i in sp if i in pos]
        if not comuns:
            continue
        erro = max(math.hypot(sp[i][0] - pos[i][0], sp[i][1] - pos[i][1])
                   for i in comuns)
        parado = max(vels.values()) < vel_max
        if erro < erro_max and parado:
            # exige estabilidade por 3 decimos: uma leitura isolada boa
            # acontece a toa quando um quadro de visao se repete.
            ok_desde = ok_desde or time.time()
            if time.time() - ok_desde > 0.3:
                return time.time() - inicio
        else:
            ok_desde = None
    return limite


def conferir_teleporte(no, alvo, tolerancia=200.0, limite=45.0):
    """Espera a bola aparecer no alvo em /visionTopic. Devolve (ok, pos, atraso).

    Serve de controle positivo E de medida do atraso da visao.

    O limite e generoso de proposito: o visionNode do ssl-VICE le UM datagrama
    por callback do timer (vision_node.py:74 + vision_client.py:76). Com ~21
    pacotes/s chegando do grSim e o callback nao acompanhando, o socket acumula
    e o atraso cresce sem parar - ja medimos mais de 20s. Para comparacao, o
    teleporte medido direto contra o grSim, sem ROS no meio, leva 0,2s.
    """
    alvo_x, alvo_y = alvo
    t0 = time.time()
    fim = t0 + limite
    while time.time() < fim:
        _girar(no, 0.3)
        if no.bola is None:
            continue
        if math.hypot(no.bola[0] - alvo_x, no.bola[1] - alvo_y) <= tolerancia:
            return True, no.bola, time.time() - t0
    return False, no.bola, time.time() - t0


# ==========================================================================
#  Execucao de um cenario
# ==========================================================================
def rodar(nome, duracao=12.0):
    if nome not in CENARIOS:
        print(f"Cenario desconhecido: {nome}", file=sys.stderr)
        return 2

    cen = CENARIOS[nome]
    perfil = os.environ.get("CAMPO", "original")

    print(f"\n>> {cen['titulo']}")
    print(f"   {cen['descricao']}")
    print(f"   bola em x={cen['bola'][0]:.0f} y={cen['bola'][1]:.0f} mm")

    import rclpy

    alvo = cen["bola"]

    rclpy.init()
    no = _criar_gravador()

    try:
        pedido = alvo
        ok, onde, atraso = conferir_teleporte(no, alvo, limite=25.0)
        if ok:
            print(f"   cenario confirmado pela visao em {atraso:.1f}s")
            print("   assentando sob HALT (driver reancora as trajetorias)...")
            enviar_comando_arbitro("HALT")
            gasto = esperar_assentar(no, limite=ESPERA_HALT, piso=1.5)
            print(f"   assentado em {gasto:.1f}s (teto era {ESPERA_HALT:.0f}s)")

        if not ok:
            visto = (
                f"({onde[0]:.0f}, {onde[1]:.0f})" if onde else "nenhuma leitura"
            )
            print()
            print("   XX A BOLA NAO CHEGOU AO LUGAR PEDIDO.")
            print(f"      pedido: ({pedido[0]:.0f}, {pedido[1]:.0f})"
                  f"   visto: {visto}")
            print()
            print("   Duas causas possiveis:")
            print()
            print("   1) SIMULACAO CONGELADA. O grSim so avanca a fisica")
            print("      quando a janela dele e redesenhada (glwidget.cpp:392")
            print("      chama step() dentro de paintGL). Janela minimizada,")
            print("      coberta ou tela bloqueada = mundo parado, e ele segue")
            print("      reenviando o ultimo quadro, entao parece normal.")
            print("      -> traga a janela do grSim para a frente, ou suba com")
            print("         ./ararabots.sh grsim --headless")
            print()
            print("   2) O visionNode nao esta publicando (ver HANDOVER §15).")
            print("      -> ./ararabots.sh parar  e depois  ./ararabots.sh preparar")
            print()
            return 3

        print(f"   bola confirmada em ({no.bola[0]:.0f}, {no.bola[1]:.0f})")

        # 1. Transição inicial obrigatória para STOP
        enviar_comando_arbitro("STOP")
        _girar(no, 1.5)

        tipo, cor = cen["comando"]
        tipo_cen = tipo.upper()
        cor_cen = cor.upper()

        # 2. SEÇÃO MODIFICADA: Envio do comando de arbitragem conforme a regra
        if tipo_cen in ("KICKOFF", "PREPARE_KICKOFF"):
            print(f"   comando do arbitro: PREPARE_KICKOFF {cor_cen} -> NORMAL_START")
            enviar_comando_arbitro("PREPARE_KICKOFF", cor_cen)
            _girar(no, 2.0)  # Tempo para o posicionamento de kickoff
            enviar_comando_arbitro("NORMAL_START")
        else:
            print(f"   comando do arbitro: {tipo} {cor}")
            enviar_comando_arbitro(tipo, cor)

        print(f"   gravando por {duracao:.0f}s (olhe a janela do grSim)...")
        no.t0 = time.monotonic()
        no.gravando = True

        # 3. SEÇÃO MODIFICADA: Mapeamento do comportamento do adversário
        if tipo_cen in ("DIRECT", "INDIRECT"):
            modo_adv = "nossa_falta" if cor_cen == "BLUE" else "falta_deles"
        elif tipo_cen in ("KICKOFF", "PREPARE_KICKOFF"):
            modo_adv = "nosso_kickoff" if cor_cen == "BLUE" else "kickoff_deles"
        else:
            modo_adv = "jogo"

        adversario_ligado = os.environ.get("ADVERSARIO", "1") != "0"
        print(f"   adversario: {'ativo (' + modo_adv + ')' if adversario_ligado else 'desligado'}")

        fim = time.time() + duracao
        proximo_cmd = 0.0
        while time.time() < fim:
            _girar(no, 0.1)
            if adversario_ligado and no.bola and time.time() >= proximo_cmd:
                proximo_cmd = time.time() + 0.1
                comandar_amarelos(no.bola, no.amarelos,
                                  getattr(no, "azuis_pos", {}), modo_adv,
                                  getattr(no, "bola_vel", (0.0, 0.0)))
        no.gravando = False

        enviar_comando_arbitro("HALT")
        amostras, kicks = no.amostras, no.kick_por_robo
        eventos_chutador = list(no.eventos_chutador)
        pedido_chute = dict(no.t_pedido_chute)
        visao_crua = list(no.visao_crua)
        alvos = list(no.alvos)
        robos_crus = list(no.robos_crus)
        janelas_kick = list(no.janelas_kick)
        for sk in (no.sock_status, no.sock_visao):
            if sk is not None:
                sk.close()
    finally:
        no.destroy_node()
        rclpy.shutdown()

    kick_ativado = any(v > 0 for v in kicks.values())
    energia = energia_do_chute(amostras, eventos_chutador, pedido_chute,
                               visao_crua)
    disparou = next((x for x in eventos_chutador if x.get("flat_kick")), None)
    janela = janela_do_chutador(visao_crua, robos_crus, janelas_kick,
                                eventos_chutador)
    _res_parcial = {"alvos": alvos, "robos_crus": robos_crus, "visao_crua": visao_crua}
    rastreio_calc = erro_de_rastreio(_res_parcial)
    gol_topico = no.gol
    gol_cru, gol_x, gol_y = gol_pela_visao_crua(visao_crua)
    gol_final = gol_cru if visao_crua else gol_topico

    perfis = {
        "original": {"goal_x": 2250.0, "kick_threshold": 1125.0},
        "divb": {"goal_x": 4500.0, "kick_threshold": 2250.0},
        "codigo": {"goal_x": 4500.0, "kick_threshold": 2250.0},
    }
    const = perfis.get(perfil, perfis["codigo"])
    resultado = {
        "cenario": nome,
        "branch": os.environ.get("BRANCH", "?"),
        "ordem": list(CENARIOS).index(nome),
        "titulo": cen["titulo"],
        "descricao": cen["descricao"],
        "perfil_campo": perfil,
        "goal_x": const["goal_x"],
        "kick_threshold": const["kick_threshold"],
        "bola_inicial": {"x": cen["bola"][0], "y": cen["bola"][1]},
        "comando": f"{tipo} {cor}",
        "duracao": duracao,
        "kick_por_robo": {str(k): v for k, v in kicks.items()},
        "kick_ativado": kick_ativado,
        "disparou": bool(disparou),
        "gol": gol_final,
        "gol_pelo_topico": gol_topico,
        "gol_pela_visao_crua": gol_cru,
        "gol_em": {"x": gol_x, "y": gol_y} if gol_cru else None,
        "amostras": amostras,
        "eventos_chutador": eventos_chutador,
        "visao_crua": visao_crua,
        "robos_crus": robos_crus,
        "alvos": alvos,
        "janela_chutador": janela,
        "janelas_kick": janelas_kick,
        "rastreio": rastreio_calc,
        "t_pedido_chute": {str(k): v for k, v in pedido_chute.items()},
        "energia": energia,
        "registrado_em": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    os.makedirs(SAIDA_DIR, exist_ok=True)
    branch = os.environ.get("BRANCH", perfil)
    destino = os.path.join(SAIDA_DIR, f"{nome}__{branch}.json")
    with open(destino, "w") as f:
        json.dump(resultado, f)

    # --- resumo no terminal ---
    print(f"   amostras gravadas: {len(amostras)}")
    if gol_final == "nosso":
        print(f"   *** GOL A FAVOR ***  (cruzou em x={gol_x:.0f} y={gol_y:.0f})")
    elif gol_final == "contra":
        print(f"   *** GOL CONTRA ***  (cruzou em x={gol_x:.0f} y={gol_y:.0f})")
    else:
        print("   sem gol")
    if visao_crua and gol_topico != gol_cru:
        print(f"   !! o /visionTopic dizia '{gol_topico}' e a visao crua diz "
              f"'{gol_cru}' - vale a crua")
    if getattr(no, "contato", None):
        print("   CONTATO (limites do grSim: xx<31,5  yy<40):")
        for rid, (d, xx, yy) in sorted(no.contato.items()):
            veredito = "PODIA CHUTAR" if (xx < 31.5 and yy < 40.0) else (
                "torto (yy=%.0f)" % yy if yy >= 40.0 else "longe (xx=%.0f)" % xx)
            print("      robo %d: aproximou %4.0f mm  xx=%5.1f  yy=%5.1f  -> %s"
                  % (rid, d, xx, yy, veredito))
    print(f"   CHUTE: {'ATIVADO' if kick_ativado else 'nao ativado'}", end="")
    if kicks:
        detalhe = ", ".join(f"r{k}={v:.1f}" for k, v in sorted(kicks.items()))
        print(f"  ({detalhe})")
    else:
        print()

    if disparou:
        print(f"   DISPARO (grSim): SIM, robo {disparou['id']} em t={disparou['t']:.3f}s")
    else:
        print("   DISPARO (grSim): NAO - a estrategia pediu, o chutador nao disparou")

    rastreio = rastreio_calc
    if rastreio:
        print("   RASTREIO (robo x setpoint do driver): mediana %.0f mm  p90 %.0f  max %.0f"
              % (rastreio["mediana"], rastreio["p90"], rastreio["maximo"]))
        print("      acima de 200 mm em %.0f%% do tempo  (alvo: mediana <100, p90 <250)"
              % rastreio["pct_acima_200"])
        print("      laco do driver a %.0f Hz durante esta execucao (timer e 100)"
              % rastreio["hz_controle"])
    imprimir_janela(janela)
    imprimir_energia(energia)

    if amostras:
        prim, ult = amostras[0], amostras[-1]
        print("   deslocamento dos nossos robos (mm):")
        pos_ini = {r["id"]: r for r in prim["azuis"]}
        for r in ult["azuis"]:
            ini = pos_ini.get(r["id"])
            if not ini:
                continue
            d = math.hypot(r["x"] - ini["x"], r["y"] - ini["y"])
            papel = " (goleiro)" if r["id"] == 0 else ""
            print(
                f"      robo {r['id']}{papel}: "
                f"({ini['x']:7.0f},{ini['y']:7.0f}) -> "
                f"({r['x']:7.0f},{r['y']:7.0f})   andou {d:6.0f}"
            )
        if prim["bola"] and ult["bola"]:
            b0, b1 = prim["bola"][0], ult["bola"][0]
            db = math.hypot(b1["x"] - b0["x"], b1["y"] - b0["y"])
            print(f"      BOLA: ({b0['x']:.0f},{b0['y']:.0f}) -> "
                  f"({b1['x']:.0f},{b1['y']:.0f})   andou {db:.0f}")

    print(f"   salvo em {destino}")
    try:
        html = destino[:-5] + ".html"
        if gerar_replay(resultado, html):
            print(f"   REPLAY VISUAL: {html}")
            print("      (visao CRUA do grSim, com o setpoint do driver desenhado)")
    except Exception as exc:
        print(f"   (replay nao gerado: {exc})")
    return 0
    
def _ferramenta_cadeia():
    """Conta mensagens em cada topico e diz o que esta vivo na cadeia."""



    JANELA = 5.0


    class Verificador(Node):
        def __init__(self):
            super().__init__("verificador_cadeia")
            self.n = {"visao": 0, "estado": 0, "arbitro": 0, "controle": 0, "time": 0}
            self.visao = None
            self.estado = None
            self.comando_arbitro = None

            self.topicos = ["visionTopic", "game_state", "refereeTopic",
                            "control_command", "commandTopic"]
            self.subs = [
                self.create_subscription(VisionMessage, "visionTopic", self._visao, 10),
                self.create_subscription(GameState, "game_state", self._estado, 10),
                self.create_subscription(RefereeMessage, "refereeTopic", self._arbitro, 10),
                self.create_subscription(ControlCommand, "control_command", self._controle, 10),
                self.create_subscription(TeamCommand, "commandTopic", self._time, 10),
            ]

        def _visao(self, m):
            self.n["visao"] += 1
            self.visao = (len(m.blue_robots), len(m.yellow_robots), len(m.balls))

        def _estado(self, m):
            self.n["estado"] += 1
            self.estado = (len(m.ally_robots), len(m.enemy_robots), len(m.balls),
                           m.geometry.field_length)
            self.comando_arbitro = m.referee.command

        def _arbitro(self, m):
            self.n["arbitro"] += 1

        def _controle(self, m):
            self.n["controle"] += 1

        def _time(self, m):
            self.n["time"] += 1


    def main():
        rclpy.init()
        no = Verificador()

        # ESPERAR A DESCOBERTA ANTES DE CONTAR.
        #
        # BUG QUE ISTO CORRIGE: este node nascia e comecava a contar na mesma
        # hora. Mas um node recem-criado ainda nao descobriu os publicadores -
        # e sob carga isso leva segundos. Topicos publicando normalmente
        # apareciam como "SEM DADOS  0.0 Hz", e o diagnostico mandava reiniciar
        # o grSim e recriar o container do arbitro sem que houvesse problema
        # nenhum. Confirmado: com /visionTopic acusando 0.0 Hz aqui, um
        # 'ros2 topic echo /visionTopic --once' devolvia dados na hora.
        #
        # Agora esperamos cada assinatura enxergar pelo menos um publicador,
        # e so entao a janela de contagem comeca.
        limite = time.time() + 15.0
        while time.time() < limite:
            rclpy.spin_once(no, timeout_sec=0.05)
            # rclpy Humble: a contagem de publicadores vem do NODE, por topico
            # (a Subscription nao expoe get_publisher_count).
            if all(no.count_publishers(t) > 0 for t in no.topicos):
                break
        # zera o que tiver chegado durante a espera: a janela tem de ser limpa
        for k in no.n:
            no.n[k] = 0

        fim = time.time() + JANELA
        while time.time() < fim:
            rclpy.spin_once(no, timeout_sec=0.02)

        def linha(nome, chave, detalhe=""):
            qtd = no.n[chave]
            marca = "✓" if qtd > 0 else "✗"
            hz = qtd / JANELA
            print(f"      {marca} {nome:<24} {hz:6.1f} Hz  {detalhe}")

        v = no.visao
        linha("/visionTopic", "visao",
              f"azuis={v[0]} amarelos={v[1]} bolas={v[2]}" if v else "SEM DADOS")

        e = no.estado
        if e:
            campo = f"campo={e[3]}mm" if e[3] else "campo=? (geometria nao chegou)"
            linha("/game_state", "estado", f"aliados={e[0]} inimigos={e[1]} {campo}")
        else:
            linha("/game_state", "estado", "SEM DADOS")

        linha("/refereeTopic", "arbitro", f"comando={no.comando_arbitro!r}")
        linha("/control_command", "controle", "(alvos da estrategia -> driver)")
        linha("/commandTopic", "time", "(velocidades -> grSim)")

        print()
        if no.n["visao"] == 0:
            print("      ! Visao parada. Causas comuns:")
            print("        - grSim com a janela coberta/minimizada (a fisica congela)")
            print("        - grSim precisa ser reiniciado apos suspensao da maquina")
        if no.n["arbitro"] == 0:
            print("      ! Arbitro nao chega ao ROS 2. Recrie o container:")
            print("        docker rm -f ssl-gc  (o ararabots.sh preparar ja faz isso)")
        if e and e[0] == 0:
            print("      ! game_state sem aliados: a estrategia nao tem sobre quem agir.")

        no.destroy_node()
        rclpy.shutdown()

    # O arquivo original chamava isto no seu 'if __name__'; aqui a chamada
    # precisa ser explicita, senao o subcomando define tudo e nao executa nada.
    main()

def _ferramenta_esperar():
    """Bloqueia ate a estrategia COMANDAR de verdade (nao apenas existir)."""

    LIMITE = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0


    class Espera(Node):
        def __init__(self):
            super().__init__("espera_estrategia")
            self.comandos = 0
            self.create_subscription(ControlCommand, "control_command", self._cb, 10)

        def _cb(self, m):
            if m.command:
                self.comandos += 1


    rclpy.init()
    no = Espera()
    fim = time.time() + LIMITE
    while time.time() < fim and no.comandos < 5:
        rclpy.spin_once(no, timeout_sec=0.05)
    ok = no.comandos >= 5
    print(f"   estrategia {'ATIVA' if ok else 'INATIVA'} "
          f"({no.comandos} comandos em {LIMITE - (fim - time.time()):.0f}s)")
    no.destroy_node()
    sys.exit(0 if ok else 1)


def _ferramenta_resumo():
    """Resume N execucoes do mesmo cenario: avanco, y final, pico, gols."""

    DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saida")
    ROTULO = sys.argv[2] if len(sys.argv) > 2 else "disp"
    GOL_X, MEIA_TRAVE = 4500.0, 500.0


    def cruzamento_y(amostras):
        """Onde a bola cruzou x=4500 (interpolado), ou None se nunca cruzou."""
        ant = None
        for a in amostras:
            if not a.get("bola"):
                continue
            b = a["bola"][0]
            if ant is not None and ant["x"] < GOL_X <= b["x"]:
                f = (GOL_X - ant["x"]) / (b["x"] - ant["x"] or 1.0)
                return ant["y"] + f * (b["y"] - ant["y"])
            ant = b
        return None


    arqs = sorted(glob.glob(os.path.join(DIR, "**", "*__%s_*.json" % ROTULO), recursive=True))
    if not arqs:
        print("nenhum resultado com o rotulo '%s' em %s" % (ROTULO, DIR))
        sys.exit(1)

    print("%-4s %10s %10s %9s %10s %8s %8s" %
          ("rep", "bola_x", "bola_y", "avanco", "pico_v", "cruz_y", "gol"))
    print("-" * 66)

    linhas = []
    for arq in arqs:
        d = json.load(open(arq))
        am = d.get("amostras", [])
        com_bola = [a for a in am if a.get("bola")]
        if not com_bola:
            continue
        b0, b1 = com_bola[0]["bola"][0], com_bola[-1]["bola"][0]
        pico = 0.0
        for i in range(1, len(com_bola)):
            p, q = com_bola[i - 1], com_bola[i]
            dt = (q.get("t", 0) - p.get("t", 0)) or (1 / 30.0)
            v = math.hypot(q["bola"][0]["x"] - p["bola"][0]["x"],
                           q["bola"][0]["y"] - p["bola"][0]["y"]) / dt
            # picos absurdos sao teleporte/glitch de visao, nao fisica
            if v < 12000.0:
                pico = max(pico, v)
        cy = cruzamento_y(com_bola)
        rep = os.path.basename(arq).split("__")[1].replace(".json", "")
        gol = d.get("gol") or ("-" if cy is None else
                               ("SIM" if abs(cy) < MEIA_TRAVE else "larga"))
        linhas.append((rep, b1["x"], b1["y"], b1["x"] - b0["x"], pico, cy, gol))
        print("%-4s %10.0f %10.0f %9.0f %10.0f %8s %8s" %
              (rep.split("_")[-1], b1["x"], b1["y"], b1["x"] - b0["x"], pico,
               "-" if cy is None else "%.0f" % cy, gol))

    if not linhas:
        sys.exit(1)


    def espalha(vals, nome, unid="mm"):
        if not vals:
            return
        m = sum(vals) / len(vals)
        dp = (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5
        print("  %-16s media %8.0f   desvio %7.0f   min %8.0f   max %8.0f  %s"
              % (nome, m, dp, min(vals), max(vals), unid))


    print()
    print("DISPERSAO entre %d execucoes IDENTICAS:" % len(linhas))
    espalha([l[3] for l in linhas], "avanco da bola")
    espalha([l[2] for l in linhas], "y final")
    espalha([l[4] for l in linhas], "pico de velocidade", "mm/s")
    gols = sum(1 for l in linhas if l[6] == "SIM" or l[6] == "nosso")
    print()
    print("  GOLS: %d de %d" % (gols, len(linhas)))
    print("  criterio do Felipe: 2 de 3 -> %s"
          % ("ATINGIDO" if gols * 3 >= 2 * len(linhas) else "NAO atingido"))


def _ferramenta_atrito():
    """Mede o alcance real da bola no grSim, isolado de robo e estrategia."""



    def carregar():
        import importlib
        for mod in ("grSim_Packet_pb2",):
            try:
                return importlib.import_module(mod)
            except ImportError:
                pass
        sys.path.insert(0, "/root/ssl-VICE/install/grsim_messenger/lib/python3.10/site-packages/grsim_messenger/protobuf")
        return importlib.import_module("grSim_Packet_pb2")


    pb = carregar()
    SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    DEST = ("127.0.0.1", 20011)


    def lancar(vx, vy):
        p = pb.grSim_Packet()
        p.replacement.ball.x = 0.0
        p.replacement.ball.y = 0.0
        p.replacement.ball.vx = vx
        p.replacement.ball.vy = vy
        SOCK.sendto(p.SerializeToString(), DEST)


    class Obs(Node):
        def __init__(self):
            super().__init__("atrito_bola")
            self.b = None
            self.create_subscription(GameState, "game_state", self.a, 10)

        def a(self, m):
            if m.balls:
                self.b = (m.balls[0].position_x, m.balls[0].position_y)


    rclpy.init()
    n = Obs()
    vels = [float(x) for x in sys.argv[2:]] or [3.0, 4.0, 5.0, 6.0, 6.5]

    print("%8s %12s %14s" % ("v(m/s)", "alcance(mm)", "desaceleracao"))
    print("-" * 38)
    for v in vels:
        lancar(0.0, 0.0)
        t = time.time()
        while time.time() - t < 1.5:
            rclpy.spin_once(n, timeout_sec=0.02)
        lancar(v, 0.0)
        t0 = time.time()
        xmax = 0.0
        parado_desde = None
        ant = None
        while time.time() - t0 < 12.0:
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.b is None:
                continue
            xmax = max(xmax, n.b[0])
            if ant is not None and abs(n.b[0] - ant) < 1.0:
                if parado_desde is None:
                    parado_desde = time.time()
                elif time.time() - parado_desde > 1.0:
                    break
            else:
                parado_desde = None
            ant = n.b[0]
        a = (v * v) / (2 * (xmax / 1000.0)) if xmax > 1 else float("nan")
        print("%8.1f %12.0f %10.2f m/s2" % (v, xmax, a))

    print()
    print("Para a falta em x=2500 chegar ao gol em x=4500 sao 2000 mm de rolagem.")
    rclpy.shutdown()


def _ferramenta_fps():
    """Mede a taxa REAL de quadros que o grSim esta emitindo.

    POR QUE ISTO E OBRIGATORIO ANTES DE QUALQUER TESTE
    --------------------------------------------------
    Em modo janela o grSim so avanca a fisica quando a janela e redesenhada
    (glwidget.cpp:392 chama step() dentro de paintGL). Com a janela coberta,
    minimizada, ou apenas disputando CPU, ele nao para - ele fica LENTO. Medimos
    4,1 Hz contra os 147,5 Hz do modo headless: 1/36 do tempo real.
    
    O sintoma e traicoeiro: nada acusa erro. A gravacao de 25 s passa a cobrir
    menos de 2 segundos simulados, o robo "nao chega na bola", e a conclusao
    natural e que a estrategia esta errada. Perdemos horas assim - comparando
    versoes de codigo cujas diferencas eram, na verdade, velocidade de simulacao.
    """
    porta = 10020
    grupo = "224.5.23.2"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", porta))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4sl", socket.inet_aton(grupo), socket.INADDR_ANY))
    s.settimeout(1.0)
    n, t0 = 0, time.time()
    while time.time() - t0 < 5.0:
        try:
            s.recv(4096); n += 1
        except socket.timeout:
            pass
    hz = n / 5.0
    if hz >= 45.0:
        print("      ok  grSim a %.0f Hz" % hz)
        return 0
    print("      !!  grSim a apenas %.1f Hz (esperado 60+)." % hz)
    print("          A simulacao esta rodando em camera lenta e QUALQUER teste")
    print("          feito assim nao vale. Use --headless, ou deixe a janela")
    print("          do grSim visivel e sem nada por cima.")
    return 1


def _ferramenta_pronto():
    """Espera um elo da cadeia ficar PRONTO de verdade, e sai assim que estiver.

    Substitui os 'sleep 7' / 'sleep 5' / 'sleep 8' do shell. Aqueles numeros
    foram escolhidos para o pior caso; na maioria das execucoes o node responde
    em menos de um segundo, e o resto era espera jogada fora - multiplicada por
    cada repeticao de cada cenario.

    A espera acontece TODA aqui dentro, num unico 'docker exec'. Fazer o laco no
    shell custaria um exec por tentativa, e o custo do proprio exec passaria a
    dominar o tempo que estamos tentando economizar.

        pronto servicos [s]   os servicos que a estrategia precisa existirem
        pronto arbitro  [s]   /refereeTopic entregando mensagem
        pronto visao    [s]   /game_state com robos e geometria
        pronto tudo     [s]   os tres acima + a estrategia ja comandando
    """
    alvo = sys.argv[2] if len(sys.argv) > 2 else "servicos"
    limite = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

    rclpy.init()
    no = Node("pronto")
    t0 = time.time()
    achou = False
    try:
        if alvo == "servicos":
            # Sem estes tres a estrategia fica presa esperando o driver, e o
            # sintoma e mudo: nenhum comando e gerado, os robos nao se mexem.
            precisa = {"/strategy_command", "/update_obstacles", "/set_orientation"}
            while time.time() - t0 < limite:
                nomes = {n for n, _ in no.get_service_names_and_types()}
                if precisa.issubset(nomes):
                    achou = True
                    break
                time.sleep(0.2)

        elif alvo == "arbitro":
            estado = {"ok": False}
            no.create_subscription(RefereeMessage, "refereeTopic",
                                   lambda _m: estado.__setitem__("ok", True), 10)
            while time.time() - t0 < limite and not estado["ok"]:
                rclpy.spin_once(no, timeout_sec=0.1)
            achou = estado["ok"]

        elif alvo == "visao":
            estado = {"ok": False}

            def cb(m):
                if m.ally_robots and getattr(m.geometry, "field_length", 0):
                    estado["ok"] = True

            no.create_subscription(GameState, "game_state", cb, 10)
            while time.time() - t0 < limite and not estado["ok"]:
                rclpy.spin_once(no, timeout_sec=0.1)
            achou = estado["ok"]
        elif alvo == "tudo":
            # TODAS as checagens num processo so.
            #
            # Cada 'docker exec' com rclpy custa ~4 s so para subir. Fazendo uma
            # chamada por elo, esse custo passaria a dominar justamente o tempo
            # que estamos tentando economizar. Aqui pagamos uma vez.
            precisa = {"/strategy_command", "/update_obstacles", "/set_orientation"}
            estado = {"arbitro": False, "comandos": 0}
            no.create_subscription(RefereeMessage, "refereeTopic",
                                   lambda _m: estado.__setitem__("arbitro", True), 10)
            no.create_subscription(
                ControlCommand, "control_command",
                lambda m: estado.__setitem__("comandos", estado["comandos"] + 1) if m.command else None, 10)
            servicos = False
            while time.time() - t0 < limite:
                rclpy.spin_once(no, timeout_sec=0.05)
                if not servicos:
                    servicos = precisa.issubset(
                        {n for n, _ in no.get_service_names_and_types()})
                # a estrategia precisa COMANDAR, nao so existir: a arvore leva
                # dezenas de segundos para resolver a cor do time, e gravar
                # antes disso mede o nada.
                if servicos and estado["arbitro"] and estado["comandos"] >= 5:
                    achou = True
                    break
            if not achou:
                print("   ! faltou: %s%s%s" % (
                    "" if servicos else "servicos ",
                    "" if estado["arbitro"] else "arbitro ",
                    "" if estado["comandos"] >= 5 else "estrategia-comandando"))

        else:
            print("alvo desconhecido: %s" % alvo, file=sys.stderr)
    finally:
        no.destroy_node()
        rclpy.shutdown()

    gasto = time.time() - t0
    if achou:
        print("   %s pronto em %.1fs" % (alvo, gasto))
        sys.exit(0)
    print("   ! %s NAO ficou pronto em %.0fs" % (alvo, limite))
    sys.exit(1)


def _ferramenta_decisao():
    """Confere a DECISAO da jogada sem simulador algum: chuta, passa, ou nada.

    Deterministico. Quando estiver em duvida se a logica esta certa, use isto e
    nao a simulacao - metade dos bugs que cacamos na estrategia estavam no
    ambiente, nao no codigo.
    """
    sys.path.insert(0, "/root/ssl-VICE/install/strategy/lib/python3.10/site-packages")
    from strategy.tatics.freekick import OurFreekick

    class R:
        def __init__(s, i, x, y):
            s.id = i; s.position_x = x; s.position_y = y
            s.velocity_x = 0.0; s.velocity_y = 0.0
            s.orientation = 0.0; s.velocity_orientation = 0.0

    class B:
        def __init__(s, x, y):
            s.position_x = x; s.position_y = y
            s.velocity_x = 0.0; s.velocity_y = 0.0

    casos = [
        ("ataque livre -> CHUTA no canto",      B(2500, 0),     {1: R(1, 2200, 0), 2: R(2, 3500, 600)},       {0: R(0, 4300, 900)}),
        ("linha de tiro bloqueada -> nao arma", B(2500, 0),     {1: R(1, 2200, 0), 2: R(2, 3500, -600)},      {0: R(0, 3200, -150)}),
        ("defesa, apoio a frente -> PASSA",     B(-2800, -600), {1: R(1, -3100, -600), 2: R(2, -1200, -200)}, {0: R(0, 4300, 0)}),
        ("apoio so ATRAS -> passa para tras",   B(500, 800),    {1: R(1, -100, 900), 2: R(2, -300, 2000)},    {0: R(0, 4300, 0)}),
        ("encurralado -> passa ao goleiro",     B(-3500, 1200), {0: R(0, -4300, 0), 1: R(1, -3800, 1200)},    {0: R(0, -2500, 1200), 1: R(1, -2600, 600)}),
        ("sozinho em campo -> nao arma",        B(500, 800),    {1: R(1, 200, 800)},                          {0: R(0, 4300, 0)}),
    ]
    for nome, bola, aliados, inimigos in casos:
        fk = OurFreekick(aliados, bola, False, enemy_robots=inimigos)
        x, y, tipo = fk._alvo_da_jogada(1)
        print("%-38s -> %-5s alvo=(%7.0f,%6.0f)  forca=%.2f m/s  arma=%s"
              % (nome, tipo.upper(), x, y, fk._forca_do_chute(1), fk._tem_alvo_valido()))


if __name__ == "__main__":
    acao = sys.argv[1] if len(sys.argv) > 1 else ""

    if acao == "listar":
        for nome, c in CENARIOS.items():
            print("%s|%s" % (nome, c["titulo"]))

    elif acao == "posicionar":
        if len(sys.argv) < 3 or sys.argv[2] not in CENARIOS:
            print("cenario invalido", file=sys.stderr); sys.exit(2)
        cen = CENARIOS[sys.argv[2]]
        print(">> %s" % cen["titulo"])
        print("   %s" % cen["descricao"])
        print("   bola em x=%.0f y=%.0f mm" % (cen["bola"][0], cen["bola"][1]))
        enviar_comando_arbitro("HALT")
        time.sleep(1.5)
        print("   montando o cenario (robos nao usados sao desligados)...")
        posicionar(cen)
        print("   pronto - o shell vai reiniciar os nodes agora")

    elif acao == "rodar":
        if len(sys.argv) < 3 or sys.argv[2] not in CENARIOS:
            print("cenario invalido", file=sys.stderr); sys.exit(2)
        dur = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
        sys.exit(rodar(sys.argv[2], dur))

    elif acao == "fps":      sys.exit(_ferramenta_fps())
    elif acao == "pronto":   _ferramenta_pronto()
    elif acao == "cadeia":   _ferramenta_cadeia()
    elif acao == "esperar":  _ferramenta_esperar()
    elif acao == "resumo":   _ferramenta_resumo()
    elif acao == "atrito":   _ferramenta_atrito()
    elif acao == "decisao":  _ferramenta_decisao()
    else:
        print(__doc__); sys.exit(2)
