#!/usr/bin/env python3
"""ararabots.py - toda a parte Python dos testes, num arquivo so.

Roda DENTRO do container 'vice' (menos 'resumo', que le os JSON no host).
Normalmente voce nao chama este arquivo direto: quem chama e o ararabots.sh.

    python3 ararabots.py listar                 nomes e titulos dos cenarios
    python3 ararabots.py posicionar <cenario>   monta o cenario no grSim
    python3 ararabots.py rodar <cenario> [s]    dispara a falta e grava
    python3 ararabots.py cadeia                 diagnostico dos topicos ROS
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


#!/usr/bin/env python3
"""Executa cenarios de freekick no grSim e grava o que a estrategia faz.

Roda DENTRO do container 'vice' (precisa do rclpy e dos protobuf do ssl-VICE).
Normalmente voce nao chama este arquivo direto - use ./testar_freekick.sh.

Para cada cenario:
  1. teleporta bola e robos no grSim (grSim_Replacement, UDP 20011)
  2. manda o comando de arbitro correspondente (WebSocket no game-controller)
  3. grava posicoes e o campo 'kick' por N segundos
  4. salva um JSON em /tmp/cenarios_freekick/

Uso:
    python3 cenarios_freekick.py listar
    python3 cenarios_freekick.py rodar <nome_do_cenario> [duracao_s]
"""
import base64
import json
import math
import os
import socket
import struct
import sys
import time

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
                  (2, 1700, 1400, 0), (3, 0, -800, 0)],
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
                  (2, -300, 2000, 0), (3, -1600, -400, 0)],
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
                  (2, -1800, 900, 0), (3, 200, -500, 0)],
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
                  (2, 900, 1500, 0), (3, -800, -700, 0)],
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
                  (2, 1600, 1100, 0), (3, 0, 400, 0)],
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
                  (2, 3200, 1000, 0), (3, 1800, 0, 0)],
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
                  (2, -1300, 900, 180), (3, -2400, -600, 180)],
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
                  (2, -3600, -300, 180), (3, -2500, 900, 180)],
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
# Isso e feito pelo testar_estrategia_freekick.sh, de FORA deste processo, entre
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


def _enviar(pacote):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Repete: UDP nao garante entrega e o grSim aplica o ultimo que chegar.
    for _ in range(3):
        sock.sendto(pacote.SerializeToString(), (GRSIM_HOST, GRSIM_PORT))
        time.sleep(0.05)
    sock.close()


def comandar_amarelos(bola, robos_amarelos):
    """Move o adversario: o amarelo mais proximo ataca a bola e chuta ao NOSSO gol.

    POR QUE EXISTE: o banco de testes so TELEPORTAVA os amarelos, deixando-os
    parados. Com adversario estatico nao da para saber se a nossa defesa
    (TheirFreekick) esta bloqueando de verdade - os robos bloqueiam alguem que
    nunca ataca.

    Comportamento simples e deterministico de proposito: um cobrador vai a bola e
    chuta para -x. Os demais ficam parados. Isso basta para exercitar o bloqueio.
    (Evolucao futura combinada com o Felipe: rodar a propria estrategia no time
    amarelo - opcao 3.)
    """
    if not robos_amarelos:
        return
    pb = _carregar_protobuf()
    bx, by = bola

    # cobrador = amarelo mais proximo da bola
    alvo_id = min(robos_amarelos,
                  key=lambda r: math.hypot(robos_amarelos[r][0] - bx,
                                           robos_amarelos[r][1] - by))
    rx, ry = robos_amarelos[alvo_id]

    # aponta da bola para o NOSSO gol (x negativo): e para la que ele chuta
    dx, dy = -GOL_X - bx, 0.0 - by
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx / n, dy / n

    # ponto atras da bola, do lado oposto ao nosso gol
    ax = bx - ux * 200.0
    ay = by - uy * 200.0
    vx, vy = ax - rx, ay - ry
    dist = math.hypot(vx, vy) or 1.0
    perto = math.hypot(rx - bx, ry - by) < 200.0

    velocidade = 1.2 if not perto else 0.6
    pacote = pb.grSim_Packet()
    c = pacote.commands
    c.timestamp = 0.0
    c.isteamyellow = True
    rc = c.robot_commands.add()
    rc.id = int(alvo_id)
    rc.wheelsspeed = False
    rc.veltangent = float(vx / dist * velocidade)
    rc.velnormal = float(vy / dist * velocidade)
    rc.velangular = 0.0
    rc.kickspeedx = 4.0 if perto else 0.0
    rc.kickspeedz = 0.0
    rc.spinner = False
    _enviar(pacote)


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
            self.t0 = time.time()
            self.create_subscription(VisionMessage, "visionTopic", self._visao, 10)
            self.create_subscription(TeamCommand, "commandTopic", self._comando, 10)

        def _visao(self, msg):
            if msg.balls:
                b = msg.balls[0]
                self.bola = (b.position_x, b.position_y)
                # Gol: a bola cruzou a linha dentro da largura da meta?
                if self.gravando and self.gol is None and abs(b.position_y) <= GOL_MEIA_LARGURA:
                    if b.position_x >= GOL_X:
                        self.gol = "nosso"
                    elif b.position_x <= -GOL_X:
                        self.gol = "contra"
            self.amarelos = {r.id: (r.position_x, r.position_y)
                             for r in msg.yellow_robots}
            self.velocidades = {
                r.id: math.hypot(r.velocity_x, r.velocity_y)
                for r in msg.blue_robots
            }
            if not self.gravando:
                return
            self.amostras.append(
                {
                    "t": round(time.time() - self.t0, 3),
                    "bola": [
                        {"x": b.position_x, "y": b.position_y} for b in msg.balls
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

    return Gravador()


def _girar(no, segundos):
    import rclpy

    fim = time.time() + segundos
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.05)


def esperar_assentar(no, limite=8.0, vel_max=25.0):
    """Espera os robos PARAREM, em vez de dormir um tempo fixo.

    POR QUE ESPERA ATIVA
    --------------------
    Isto era 'durma 8 segundos'. O numero foi escolhido com folga, para o pior
    caso - e o pior caso e raro. Na maioria das execucoes os robos ja estao
    parados em 1 ou 2 segundos, e os 6 restantes eram desperdicio puro,
    multiplicado por cada repeticao de cada cenario.

    O QUE ESTA ESPERANDO DE VERDADE
    -------------------------------
    O driver cria a trajetoria de um robo uma vez e dali em diante replaneja a
    partir do setpoint do plano anterior, nunca da posicao medida. Depois de um
    teleporte ele ficaria planejando a partir de uma origem fantasma. Sob HALT,
    a tatica manda cada robo para a PROPRIA posicao lida da visao: quando essas
    trajetorias terminam, a ancora do driver volta a coincidir com a realidade.

    Terminaram quando os robos param de se mexer. E isso que medimos aqui, em
    vez de cronometrar. O limite continua existindo como teto de seguranca.
    """
    import rclpy
    fim = time.time() + limite
    quietos_desde = None
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.05)
        vels = getattr(no, "velocidades", None)
        if not vels:
            continue
        if max(vels.values()) < vel_max:
            # exige estabilidade por 3 decimos: uma leitura isolada de zero
            # acontece a toa quando um quadro de visao se repete.
            quietos_desde = quietos_desde or time.time()
            if time.time() - quietos_desde > 0.3:
                return time.time() - (fim - limite)
        else:
            quietos_desde = None
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

            # ---------------------------------------------------------------
            #  Assentamento sob HALT: deixa o driver se reancorar sozinho.
            #
            #  O driver cria a trajetoria de um robo UMA unica vez (driver.py:90)
            #  e dali em diante replaneja a partir do setpoint do plano anterior
            #  (driver.py:257), nunca da posicao medida. Depois de um teleporte
            #  ele ficaria planejando a partir de uma origem fantasma.
            #
            #  Mas tatics/halt.py manda cada robo para a PROPRIA posicao lida da
            #  visao. Entao, mantendo HALT por alguns segundos, o driver replaneja
            #  ate a posicao real; quando essa trajetoria termina, get_state()
            #  passa a devolver o ponto final - que e a posicao verdadeira. A
            #  ancora se corrige sozinha, sem reiniciar node nenhum e sem tocar
            #  no codigo do time.
            #
            #  E o mesmo que acontece numa partida de verdade: sempre ha HALT
            #  antes de uma falta.
            print("   assentando sob HALT (driver reancora as trajetorias)...")
            enviar_comando_arbitro("HALT")
            gasto = esperar_assentar(no, limite=ESPERA_HALT)
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
            print("         ./iniciar_grsim.sh --headless")
            print()
            print("   2) VISAO ATRASADA DEMAIS (bug do visionNode, ver LEIAME).")
            print("      -> reinicie os nodes: ./parar_tudo.sh && ./testar_e2e.sh")
            print()
            return 3

        print(f"   bola confirmada em ({no.bola[0]:.0f}, {no.bola[1]:.0f})")

        # Um freekick so vale a partir do STOP.
        # STOP: espera o comando CHEGAR na visao do time, em vez de contar 2s.
        # O que importa nao e o tempo, e a estrategia ja estar vendo o STOP
        # quando o DIRECT for enviado logo em seguida.
        enviar_comando_arbitro("STOP")
        esperar_assentar(no, limite=2.0)

        tipo, cor = cen["comando"]
        print(f"   comando do arbitro: {tipo} {cor}")
        enviar_comando_arbitro(tipo, cor)

        print(f"   gravando por {duracao:.0f}s (olhe a janela do grSim)...")
        no.t0 = time.time()
        no.gravando = True
        # Enquanto grava, o adversario ATACA - senao a defesa nao e testada.
        fim = time.time() + duracao
        while time.time() < fim:
            _girar(no, 0.1)
            # Adversario movel DESATIVADO por decisao do Felipe: ele introduzia
            # uma variavel nova no meio da cacada do bug do chute. A funcao
            # comandar_amarelos() continua pronta para religar depois.
            pass
        no.gravando = False

        enviar_comando_arbitro("HALT")
        amostras, kicks = no.amostras, no.kick_por_robo
    finally:
        no.destroy_node()
        rclpy.shutdown()

    kick_ativado = any(v > 0 for v in kicks.values())
    # Guardamos as constantes do perfil para o relatorio poder DESENHAR onde a
    # estrategia acha que fica o gol - e a diferenca contra o gol real de 4500
    # explica de imediato o goleiro fora de posicao.
    # Medidas que a estrategia esta REALMENTE usando, para o relatorio desenhar
    # a referencia certa.
    #
    # Antes o padrao caia em "original" (gol +/-2250) sempre que o perfil nao
    # fosse reconhecido - inclusive com CAMPO=codigo, que e o modo normal. O
    # relatorio entao acusava "o goleiro abandona a meta" comparando com uma
    # medida que o codigo nao usa mais. Agora o padrao e Division B, que e o que
    # o codigo assume quando nao ha geometria da visao.
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
        "gol": no.gol,
        "amostras": amostras,
        "registrado_em": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    os.makedirs(SAIDA_DIR, exist_ok=True)
    branch = os.environ.get("BRANCH", perfil)
    destino = os.path.join(SAIDA_DIR, f"{nome}__{branch}.json")
    with open(destino, "w") as f:
        json.dump(resultado, f)

    # --- resumo no terminal ---
    print(f"   amostras gravadas: {len(amostras)}")
    if no.gol == "nosso":
        print("   *** GOL A FAVOR ***")
    elif no.gol == "contra":
        print("   *** GOL CONTRA ***")
    else:
        print("   sem gol")
    print(f"   CHUTE: {'ATIVADO' if kick_ativado else 'nao ativado'}", end="")
    if kicks:
        detalhe = ", ".join(f"r{k}={v:.1f}" for k, v in sorted(kicks.items()))
        print(f"  ({detalhe})")
    else:
        print()

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
                f"     robo {r['id']}{papel}: "
                f"({ini['x']:7.0f},{ini['y']:7.0f}) -> "
                f"({r['x']:7.0f},{r['y']:7.0f})  andou {d:6.0f}"
            )
        if prim["bola"] and ult["bola"]:
            b0, b1 = prim["bola"][0], ult["bola"][0]
            db = math.hypot(b1["x"] - b0["x"], b1["y"] - b0["y"])
            print(f"     BOLA: ({b0['x']:.0f},{b0['y']:.0f}) -> "
                  f"({b1['x']:.0f},{b1['y']:.0f})  andou {db:.0f}")

    print(f"   salvo em {destino}")
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

            self.create_subscription(VisionMessage, "visionTopic", self._visao, 10)
            self.create_subscription(GameState, "game_state", self._estado, 10)
            self.create_subscription(RefereeMessage, "refereeTopic", self._arbitro, 10)
            self.create_subscription(ControlCommand, "control_command", self._controle, 10)
            self.create_subscription(TeamCommand, "commandTopic", self._time, 10)

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
            print("        docker rm -f ssl-gc  (o preparar_testes.sh ja faz isso)")
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

    elif acao == "pronto":   _ferramenta_pronto()
    elif acao == "cadeia":   _ferramenta_cadeia()
    elif acao == "esperar":  _ferramenta_esperar()
    elif acao == "resumo":   _ferramenta_resumo()
    elif acao == "atrito":   _ferramenta_atrito()
    elif acao == "decisao":  _ferramenta_decisao()
    else:
        print(__doc__); sys.exit(2)
