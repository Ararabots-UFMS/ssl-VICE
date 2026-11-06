#!/usr/bin/env python3
"""Shot campaign: random ball/robot placements, one CSV row per attempt.

Runs INSIDE the vice container with the chain already up (ararabots.sh campanha
does that). It never restarts a node: each attempt is HALT, teleport,
FORCE_START, record, so an attempt costs seconds instead of a full relaunch.

    python3 campanha_chute.py --n 25

The plan of attempt i depends only on (--seed, i), so a run can be resumed or
split in blocks: attempts already in the CSV are skipped.

Blue attacks +x. Lengths in mm, angles in degrees.
"""
import argparse
import csv
import json
import math
import os
import random
import socket
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ararabots as A   # noqa: E402  placement, referee and protobuf helpers

GOL_X = A.GOL_X
GOL_MEIA = A.GOL_MEIA_LARGURA
MEIO_CAMPO_X, MEIO_CAMPO_Y = 4500.0, 3000.0
AREA_FUNDO, AREA_MEIA = 1000.0, 1000.0      # penalty area: depth, half width
PORTADOR = 1                                # id 0 is the goalkeeper for the tactic
ARQUIVO_MIRA = "/tmp/ararabots_mira_y"      # read by strategy/skills/experimento.py
VEL_CHUTE = 1500.0                          # ball faster than this was kicked, not pushed
SALTO = 400.0                               # more than this in one frame is not motion
VEL_MAX_BOLA = 9000.0                       # mm/s; nothing in grSim moves the ball faster
GOLEIRO_ID = 0                              # their keeper: yellow 0; other yellows from 1
RAIO_ROBO = 90.0
DEFESA_RAIO = 200.0                         # a shot that passes this close to the keeper hit it
CENTRO_ATE_PLACA = 73.0                     # robot centre to the kicker plate (grSim)
MOVEU = 25.0                                # the ball has been touched once it moved this far
CONTATO = 130.0                             # robot centre to ball centre when they touch (90 + 21 + slack)
POSSE_YY = 60.0                             # ball this far off the kicker axis is a graze, not possession
SALTO_CHUTE = 800.0                         # a kick adds at least this much speed in 100 ms
VEL_CHUTE_MIN = 2500.0                      # below this a take-off is a collision, not grSim's kicker
# Desaceleracao da bola no grSim, medida 28/10 (600/1200/1800 mm/s: 1346/1359/1699).
DESACEL_BOLA = 1350.0
# DOIS numeros, e a diferenca entre eles e o defeito que estamos medindo:
#
#   539 mm/s  distancia / tempo ate ENCOSTAR na bola (a reta, 32 tentativas)
#   161 mm/s  distancia / tempo ate TER a bola       (24 tentativas pareadas)
#
# O segundo e 3,3x pior porque ele encosta, perde, e volta - mediana de 2
# investidas por tentativa. Reduzir essa diferenca e o alvo do trabalho.
#
# O SORTEIO usa o primeiro, de proposito. Ele e um botao de PROJETO DE
# EXPERIMENTO ("de que velocidade a bola precisa para ainda estar rolando
# quando ele chegar"), nao uma afirmacao sobre o robo. Usar os 161 tornaria a
# grade impossivel: exigiria bola a 4400 mm/s rolando 7 m num campo de 9x6, e
# o sorteio rejeitaria tudo. Quem diz a verdade sobre o contato e a coluna
# medida 'vel_bola_toque', nao esta constante.
VEL_ROBO_APROX = 539.0
VEL_ROBO_POSSE = 161.0    # so para o relatorio; ver acima
MARGEM_ROLANDO = 1.3                        # a bola tem de chegar ao contato ainda andando
VEL_BOLA_MAX = 5000.0                       # nada no jogo chega com mais que isto

CAMPOS = [
    "tentativa", "seed", "bola_x", "bola_y", "bola_vx", "bola_vy", "geometria",
    "robo_x", "robo_y", "robo_dir",
    "dist_robo_bola", "dist_bola_gol", "angulo_bola_gol", "angulo_contorno",
    "mira_y", "inimigos", "resultado", "chutou", "t_chute", "bola_chute_x",
    "bola_chute_y", "dist_chute_gol", "arrasto", "recuo", "vel_saida",
    "erro_corpo", "erro_direcao", "y_na_linha", "erro_y", "bola_fim_x",
    "bola_fim_y", "duracao", "trombadas", "fantasmas", "motivo", "goleiro_x", "goleiro_y",
    "toque_xx", "toque_yy", "toque_armado", "toque_foi_chute",
    "alcancou", "encostou", "t_toque", "t_encosto", "toque_x", "toque_y",
    "investidas", "t_1a_investida", "vel_bola_toque", "bola_parada_no_toque",
    "rolagem_prevista", "cond", "chaves_off",
]


def na_area(x, y, folga):
    return abs(x) > MEIO_CAMPO_X - AREA_FUNDO - folga and abs(y) < AREA_MEIA + folga


def rolagem(v):
    """Quanto a bola anda sozinha, largada a v mm/s (atrito medido)."""
    return v * v / (2.0 * DESACEL_BOLA)


def rolagem_livre(bx, by, ux, uy):
    """Quanto a bola pode rolar nesta direcao SEM sair do campo nem entrar numa area.

    POR QUE ISTO E OBRIGATORIO, e nao refinamento: nas 12 tentativas de 28/10,
    os TRES 'nao_alcancou' foram isto e nada mais -
        cruza #1       a bola sai do campo depois de 1760 mm
        foge_gol #1    entra na area DELES depois de 2188 mm
        foge_nosso #0  entra na NOSSA area depois de 2245 mm
    O portador nao entra na area (e nao deve), e bola fora e jogada morta: as
    tres tentativas nao mediram eficiencia de chute nenhuma, mediram para onde
    o sorteio jogou a bola. Agora a colocacao so passa se a bola couber.
    """
    passo = 50.0
    d = 0.0
    while d < 7000.0:
        d += passo
        x, y = bx + ux * d, by + uy * d
        if not no_campo(x, y, 300.0) or na_area(x, y, 100.0):
            return d - passo
    return 7000.0


# OS LIMITES SAO OS DO PLANEJADOR, nao uma folga escolhida a mao.
#
# skills/geometria.no_campo SATURA todo alvo em 4300 x 2800, e o portador pede
# alvos por ali. Se o posto de chute cai fora disso, o alvo dele e recortado e
# ele para a alguns centimetros do lugar de onde chutaria - para sempre.
#
# MEDIDO em 03/11/2026, t26 do lote de 40 da seed 12 (a unica falha em 40): a
# bola para em (-977, 2802), 198 mm da linha lateral. O posto de chute fica em
# (-1093, 2860) - 60 mm acima de LIMITE_Y_ALVO. O portador ficou 16 s oscilando
# a ~260 mm da bola, com 1 investida e nenhum chute. A versao anterior desta
# checagem usava folga 100 (ou seja, 2900) e deixou passar.
LIMITE_X_ALVO, LIMITE_Y_ALVO = 4300.0, 2800.0


# VISAO DE OUTRO grSim NA REDE INVALIDA A TENTATIVA - E BLOQUEIA O LOTE.
#
# O grSim multicasta visao em 224.5.23.2:10020 e NAO ha dono do grupo: duas
# maquinas na mesma rede publicam as duas, e o no de visao do ROS consome as
# duas. Cada robo e a bola passam a aparecer em DOIS lugares, alternando a
# cada quadro.
#
# ISTO NAO E AVISO, E BLOQUEIO. Esta verificacao existia e so IMPRIMIA
# "(ignorado)" - e o '(ignorado)' valia para o sniffer desta campanha, nunca
# para a estrategia, que e quem estava sendo medida.
#
# MEDIDO em 03/11/2026: o mesmo codigo, a mesma grade e as MESMAS 40
# colocacoes (conferidas campo a campo) deram 38/40 com a rede limpa e 16/40
# com outro grSim no ar - 1163 pacotes dele contra 1177 nossos em 5 s. O lote
# sujo trouxe nao_alcancou de 0 para 7, erro de corpo p90 de 3,9 para 38,6
# graus, e derrubou inclusive a mira 0,0 (7/7 -> 4/7), que nenhuma mudanca
# daquele dia tocava. Eu quase atribui isso a uma alteracao de estrategia.
#
# O limite nao e zero: um datagrama perdido na troca de grupo nao condena um
# lote de 2 h. Dez num intervalo de tentativa e um grSim transmitindo.
LIMITE_ESTRANHOS = 10


def posto_de_chute(bx, by, mira):
    """Onde o portador tem de ficar para chutar a bola em (bx,by) rumo ao gol."""
    a = math.atan2((mira if mira is not None else 0.0) - by, GOL_X - bx)
    return bx - math.cos(a) * CONTATO, by - math.sin(a) * CONTATO


def posto_legal(bx, by, mira):
    """O portador CABE atras da bola, com a bola ali, para chutar no gol?

    POR QUE ISTO E OBRIGATORIO, e nao refinamento: a tentativa 13 de
    docs/campanha_gatilho (03/11/2026) falhou nos DOIS bracos do pareado - e
    o sinal de que o defeito e a colocacao, nao a estrategia. A bola rolou
    livre e parou perto de (-3377, 1019), fora da nossa caixa de area por 3 mm.
    Mas o posto de chute fica 130 mm ATRAS dela, em x ~ -3507: DENTRO da
    caixa (x < -3380, |y| < 1120). O portador nao entra; o recorte joga o alvo
    dele para a borda, ele chega de lado, encosta na bola e a empurra para
    dentro. Bola final (-3659, 992), 14,8 s sem chute.

    Nao ha jogada legal ali com tiro no gol, nos dois bracos: a tentativa nao
    mede eficiencia de chute, mede onde o sorteio parou a bola.
    """
    px, py = posto_de_chute(bx, by, mira)
    return (abs(px) <= LIMITE_X_ALVO and abs(py) <= LIMITE_Y_ALVO
            and not na_area(px, py, 120.0))


def no_campo(x, y, folga):
    return abs(x) < MEIO_CAMPO_X - folga and abs(y) < MEIO_CAMPO_Y - folga


def plano(seed, i, args):
    """A colocacao da tentativa i. Amostragem por rejeicao, deterministica em (seed, i)."""
    # crc32, e NAO hash(): hash() de string muda a cada processo
    # (PYTHONHASHSEED), e a campanha precisa poder ser retomada.
    #
    # A GRADE ENTRA NA SEMENTE. Antes o sorteio era so (seed, i) e a geometria
    # saia DEPOIS da posicao: rodando uma geometria por vez, as quatro pastas
    # recebiam EXATAMENTE as mesmas colocacoes - "a bola sempre vem no mesmo
    # canto". Misturando a grade, cada cenario sorteia o seu.
    marca = zlib.crc32(("|".join(sorted(args.geometrias)) + "#"
                        + ("auto" if args.vel_auto else
                           ",".join(f"{v:.0f}" for v in sorted(args.bola_vel)))).encode())
    rng = random.Random(seed * 1000003 + i * 7919 + marca)

    # O SORTEIO INTEIRO E UM LACO. As duas condicoes da bola em movimento
    # (caber no campo, chegar andando) podem reprovar a colocacao, e ai ela e
    # refeita do zero - posicao da bola inclusive.
    for _ in range(5000):
        while True:
            bx = rng.uniform(args.x_min, args.x_max)
            by = rng.uniform(-args.y_max, args.y_max)
            if no_campo(bx, by, 300.0) and not na_area(bx, by, 300.0):
                break
        while True:
            d = rng.uniform(args.dist_min, args.dist_max)
            a = rng.uniform(-math.pi, math.pi)
            rx, ry = bx + d * math.cos(a), by + d * math.sin(a)
            if no_campo(rx, ry, 200.0) and not na_area(rx, ry, 200.0):
                break
        rdir = rng.uniform(-180.0, 180.0)
        mira = rng.choice(args.miras) if args.miras else None

        # BOLA JA ROLANDO: a direcao sai da geometria da tentativa, nao do
        # campo. "Cruzando na frente do atacante" so quer dizer algo em
        # relacao a onde o atacante esta.
        geo = rng.choice(args.geometrias)
        vel = rng.choice(args.bola_vel) if args.bola_vel else 0.0
        if geo == "parada" or (vel <= 0.0 and not args.vel_auto):
            geo, bvx, bvy = "parada", 0.0, 0.0
        else:
            para_robo = math.atan2(ry - by, rx - bx)
            if geo == "cruza":
                rumo = para_robo + rng.choice((-math.pi / 2.0, math.pi / 2.0))
            elif geo == "de_frente":
                rumo = para_robo
            elif geo == "foge_gol":
                rumo = math.atan2((mira if mira is not None else 0.0) - by, GOL_X - bx)
            elif geo == "foge_nosso":
                rumo = math.atan2(-by, -GOL_X - bx)
            else:
                rumo = rng.uniform(-math.pi, math.pi)
            ux, uy = math.cos(rumo), math.sin(rumo)
            livre = rolagem_livre(bx, by, ux, uy)

            # TETO: a bola nao pode sair do campo nem entrar numa area rolando
            # sozinha. Os TRES 'nao_alcancou' de 28/10 foram exatamente isso -
            # fora do campo a 1760 mm, area deles a 2188, nossa area a 2245 - e
            # nenhum deles mediu eficiencia de chute: mediram para onde o
            # sorteio jogou a bola.
            teto = min(math.sqrt(2.0 * DESACEL_BOLA * livre), VEL_BOLA_MAX)

            if args.vel_auto:
                # PISO: a bola tem de chegar ao contato AINDA ANDANDO. Com
                # atrito de 1350 mm/s2 e o portador se aproximando a 539 mm/s
                # (medido), lancar 2500 fixo deu bola parada em 6 dos 9
                # contatos de 28/10 - o teste dizia "bola andando" e media bola
                # parada. O piso sai da geometria: v > a * (d / v_robo) * margem.
                piso = DESACEL_BOLA * (d / VEL_ROBO_APROX) * MARGEM_ROLANDO
                if piso > teto:
                    continue                    # geometria impossivel
                vel = rng.uniform(piso, teto)
            elif vel > teto:
                continue                        # a velocidade pedida joga a bola fora
            bvx, bvy = vel * math.cos(rumo), vel * math.sin(rumo)

            # A bola tem de PARAR num lugar onde exista posto de chute legal.
            # t13 de campanha_gatilho falhou nos dois bracos por isto: ver
            # posto_legal.
            andou = rolagem(vel)
            if not posto_legal(bx + ux * andou, by + uy * andou, mira):
                continue

        inimigos = []
        if args.goleiro == "gol":
            inimigos.append((GOLEIRO_ID, GOL_X - RAIO_ROBO - 10.0,
                             rng.uniform(-GOL_MEIA + RAIO_ROBO, GOL_MEIA - RAIO_ROBO), 180.0))
        elif args.goleiro == "area":
            inimigos.append((GOLEIRO_ID,
                             rng.uniform(MEIO_CAMPO_X - AREA_FUNDO + RAIO_ROBO, GOL_X - RAIO_ROBO - 10.0),
                             rng.uniform(-AREA_MEIA + RAIO_ROBO, AREA_MEIA - RAIO_ROBO), 180.0))
        primeiro = 1 if args.goleiro != "nenhum" else 0
        for rid in range(primeiro, primeiro + args.inimigos):
            while True:
                ex = rng.uniform(max(bx, -1000.0), MEIO_CAMPO_X - 300.0)
                ey = rng.uniform(-MEIO_CAMPO_Y + 300.0, MEIO_CAMPO_Y - 300.0)
                if all(math.hypot(ex - px, ey - py) > 500.0
                       for px, py in [(bx, by), (rx, ry)] + [(e[1], e[2]) for e in inimigos]):
                    break
            inimigos.append((rid, ex, ey, 180.0))
        return dict(bola=(bx, by), bola_vel=(bvx, bvy), geometria=geo,
                    robo=(rx, ry, rdir), mira=mira, inimigos=inimigos,
                    goleiro=args.goleiro != "nenhum")

    raise SystemExit(
        "nao consegui sortear a tentativa %d: nenhuma colocacao satisfaz "
        "'a bola cabe no campo' e 'chega andando' com dist %.0f-%.0f mm. "
        "Diminua --dist-max (o portador se aproxima a ~%.0f mm/s)."
        % (i, args.dist_min, args.dist_max, VEL_ROBO_APROX))


def criar_no():
    from rclpy.node import Node
    from system_interfaces.msg import TeamCommand

    class Gravador(Node):
        """Ball and robot straight from grSim's detection frames, not from visionTopic:
        the topic is the tracker's belief, and it lags and jumps while the ball flies."""

        def __init__(self):
            super().__init__("gravador_campanha")
            self.bola = None
            self.robo = None
            self.kick = 0.0
            self.gravando = False
            self.quadros = []       # (t_capture, bx, by, rx, ry, rth, kick)
            self.fantasmas = 0      # ball detections far from where the ball was
            self.estranhos = set()  # other machines multicasting vision
            self.estranhos_n = 0    # quantos pacotes deles, desde o ultimo zero
            self._local = {}
            self.sock, self._Pacote = A._abrir_escuta_visao_crua()
            if self.sock is None:
                raise RuntimeError("sem acesso a visao crua do grSim")
            self.create_subscription(TeamCommand, "commandTopic", self._comando, 20)

        def _desta_maquina(self, ip):
            """Only a local address can be bound to."""
            if ip not in self._local:
                sonda = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sonda.bind((ip, 0))
                    self._local[ip] = True
                except OSError:
                    self._local[ip] = False
                finally:
                    sonda.close()
            return self._local[ip]

        def drenar(self):
            while True:
                try:
                    dados, origem = self.sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    return
                if not self._desta_maquina(origem[0]):
                    self.estranhos.add(origem[0])   # another grSim on the network
                    self.estranhos_n += 1
                    continue
                pacote = self._Pacote()
                try:
                    pacote.ParseFromString(dados)
                except Exception:
                    continue            # a truncated or foreign datagram on the group
                if not pacote.HasField("detection"):
                    continue
                quadro = pacote.detection
                for r in quadro.robots_blue:
                    if r.robot_id == PORTADOR:
                        self.robo = (r.x, r.y, r.orientation)
                if not quadro.balls:
                    continue
                # Cameras overlap, and one of them may report a second ball: keep the
                # detection nearest to where the ball already was.
                bolas = [(b.x, b.y) for b in quadro.balls]
                longe = False
                if self.bola is not None:
                    bolas.sort(key=lambda b: math.hypot(b[0] - self.bola[0], b[1] - self.bola[1]))
                    longe = math.hypot(bolas[0][0] - self.bola[0], bolas[0][1] - self.bola[1]) > 1000.0
                if longe and self.gravando:
                    self.fantasmas += 1
                    continue            # a ghost: keep it out of the trajectory too
                self.bola = bolas[0]
                if self.gravando and self.robo:
                    self.quadros.append((quadro.t_capture,) + self.bola + self.robo + (self.kick,))

        def _comando(self, msg):
            for r in msg.robots:
                if r.robot_id == PORTADOR:
                    self.kick = float(r.kick)

    return Gravador()


def girar(no, segundos):
    import rclpy
    fim = time.monotonic() + segundos
    while time.monotonic() < fim:
        rclpy.spin_once(no, timeout_sec=0.01)
        no.drenar()


def esperar_teleporte(no, p, limite=10.0):
    fim = time.monotonic() + limite
    while time.monotonic() < fim:
        girar(no, 0.1)
        if no.bola and no.robo and (
                math.hypot(no.bola[0] - p["bola"][0], no.bola[1] - p["bola"][1]) < 150.0
                and math.hypot(no.robo[0] - p["robo"][0], no.robo[1] - p["robo"][1]) < 150.0):
            return True
    return False


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def limpar(q):
    """Drop frames where the ball is somewhere it could not have got to.

    visionTopic now and then reports the ball metres away for a frame or two and
    then back. A real ball tops out near 6.5 m/s, so a frame further than that
    from the last good one is dropped, unless the new place holds for 0.75 s (then it is the ball, and the earlier ones were wrong).
    """
    def perto(a, b):
        return math.hypot(b[1] - a[1], b[2] - a[2]) <= VEL_MAX_BOLA * abs(b[0] - a[0]) + 60.0

    bons, espera = [], []
    for f in q:
        if not bons or perto(bons[-1], f):
            bons.append(f)
            espera = []
            continue
        espera = [g for g in espera if perto(g, f)] + [f]
        if len(espera) >= 20 and espera[-1][0] - espera[0][0] > 0.75:
            bons.extend(espera)
            espera = []
    return bons


def toque_em(q, rolando):
    """Index of the first frame where OUR robot has the ball, or None.

    TWO CRITERIA, and the reason is the whole moving-ball measurement.

    With the ball at REST, "it moved 25 mm" is contact: nothing else in the
    scenario can shift it. That is what the campaign has measured so far, and
    it stays bit-identical here so the earlier CSVs remain comparable.

    With the ball ROLLING, that test fires on frame 2 of every attempt - the
    ball is moving because WE launched it. Then everything downstream lies:
    the kick is "detected" at take-off, the drag is measured from a point the
    robot never visited, and an attempt where the attacker never got near the
    ball reads as a clean shot. So for a rolling ball contact is PROXIMITY:
    the first frame with the ball inside CONTATO of the robot centre.
    """
    if not rolando:
        bx0, by0 = q[0][1], q[0][2]
        for i, f in enumerate(q):
            if math.hypot(f[1] - bx0, f[2] - by0) >= MOVEU:
                return i
        return None
    for i, f in enumerate(q):
        xx, yy = _na_frente(f)
        if xx is not None and xx > 0.0 and abs(yy) <= POSSE_YY:
            return i
    return None


def _na_frente(f):
    """Ball in the robot's own frame: (forward, sideways) in mm, or (None, None).

    POR QUE: distance alone makes a GRAZE count as possession. Measured on the
    20 attempts of 28/10: 7 of 19 first touches sat outside the kicker plate
    (|yy| > 40 mm) and one was at 115 mm - the ball went PAST the side of the
    robot and the tool called it "reached". grSim only fires with the ball
    within 40 mm of the plate centre, so a ball 115 mm to the side is not a
    ball the attacker has.
    """
    _, bx, by, rx, ry, rth, _ = f
    if math.hypot(bx - rx, by - ry) >= CONTATO:
        return None, None
    dx, dy = bx - rx, by - ry
    return (dx * math.cos(rth) + dy * math.sin(rth),
            -dx * math.sin(rth) + dy * math.cos(rth))


def investidas_em(q, ik):
    """Quantas vezes ele chegou na bola, e quando foi a primeira.

    UMA INVESTIDA e uma entrada no raio de CONTATO vinda de fora dele. Se o
    chute sai na primeira, investidas = 1; se ele erra a abordagem, a bola
    escapa e ele volta, cada volta conta mais uma.

    POR QUE ISTO VIRA COLUNA: nas 20 de 28/10, **13 de 19** primeiros toques
    NAO foram o chute - ele encostava, perdia, e so depois acertava. Isso
    aparecia disfarcado de "t_chute alto" e nao dava para contar. Agora da.
    """
    n, dentro, primeira = 0, False, None
    for f in q:
        perto = math.hypot(f[1] - f[3], f[2] - f[4]) < CONTATO
        if perto and not dentro:
            n += 1
            if primeira is None:
                primeira = f[0]
        dentro = perto
    return n, primeira


def encosto_em(q):
    """First frame where the ball is merely NEAR the robot - graze included.

    Kept apart from toque_em on purpose: "it got close" and "it had the ball"
    are different questions, and collapsing them is what inflated alcancou.
    """
    for i, f in enumerate(q):
        if math.hypot(f[1] - f[3], f[2] - f[4]) < CONTATO:
            return i
    return None


def chute_em(q, desde=0):
    """Index of the first frame where the ball takes off WITH the kicker armed, and
    how many times it was knocked that fast without it (a bump, not a kick).

    A TAKE-OFF IS A GAIN OF SPEED, NOT A HIGH SPEED. The old test was
    "faster than VEL_CHUTE"; with the ball launched at 1600-2400 mm/s that is
    true from the first frame, and the kicker is often already armed while the
    attacker closes in - so a shot got recorded on every attempt, whether or
    not the robot ever touched the ball. Requiring the speed to JUMP by
    SALTO_CHUTE over 100 ms keeps a kick a kick (0 -> 5000 passes either way)
    and refuses a ball that was merely already rolling fast.

    'desde' is the contact frame: nothing before the attacker has the ball can
    be its shot.
    """
    trombadas, rapido = 0, False
    for i in range(max(1, desde), len(q)):
        v = math.hypot(*velocidade(q, i))
        if v <= VEL_CHUTE:
            rapido = False
            continue
        j = i
        while j > 0 and q[i][0] - q[j][0] < 0.10:
            j -= 1
        if v - math.hypot(*velocidade(q, j)) <= SALTO_CHUTE:
            continue
        if rapido:
            continue
        rapido = True
        # ARMED NEAR THE TAKE-OFF, AND FAST ENOUGH TO BE THE KICKER.
        #
        # The window was 25 frames (~0.4 s) with no speed floor, and that let a
        # COLLISION through: foge_gol #1 of 28/10 left at 1780 mm/s, 20.5 deg
        # off the aim, kicker not armed at the touch - the robot ran into the
        # ball while the kicker happened to be armed somewhere nearby. grSim's
        # kicker puts the ball out at 4100-5900; nothing we hit by accident
        # reaches 2500.
        armado = any(f[6] > 0.0 for f in q[max(0, i - 8):i + 3])
        # O PISO OLHA O PICO DA DECOLAGEM, NAO O PRIMEIRO QUADRO.
        #
        # O quadro em que a bola cruza VEL_CHUTE esta no MEIO da rampa: a
        # janela de 50 ms da velocidade pega quadros de antes do contato, e o
        # valor sai baixo. Medido num chute sintetico de 5000 mm/s: o primeiro
        # quadro acima de 1500 marcou 1667, abaixo do piso - e como o latch
        # 'rapido' ja estava armado, o chute inteiro era perdido e virava
        # trombada. O pico nos 200 ms seguintes e o numero certo.
        pico = max((math.hypot(*velocidade(q, k)) for k in range(i, len(q))
                    if q[k][0] - q[i][0] <= 0.20), default=v)
        if armado and pico >= VEL_CHUTE_MIN:
            return i, trombadas
        trombadas += 1
    return None, trombadas


def velocidade(q, i, janela=0.05):
    """Ball velocity over the window ending at frame i."""
    j = i
    while j > 0 and q[i][0] - q[j][0] < janela:
        j -= 1
    dt = q[i][0] - q[j][0]
    if dt <= 0:
        return 0.0, 0.0
    # A single-frame jump is the tracker swapping detections, not the ball moving.
    if any(math.hypot(q[k + 1][1] - q[k][1], q[k + 1][2] - q[k][2]) > SALTO for k in range(j, i)):
        return 0.0, 0.0
    return (q[i][1] - q[j][1]) / dt, (q[i][2] - q[j][2]) / dt


def medir(q, p):
    """Everything the CSV says about one attempt, from its recorded frames."""
    bx0, by0 = p["bola"]
    # With the strategy choosing its own aim, errors are measured against the goal centre.
    alvo = (GOL_X, p["mira"] if p["mira"] is not None else 0.0)
    m = dict(chutou=0, resultado="sem_chute")
    q = limpar(q)
    if len(q) < 5:
        m["resultado"] = "ERRO"
        m["motivo"] = "sem quadros"
        return m
    t0 = q[0][0]
    # A robot that never moved means the chain was down (network change, dead node),
    # not that the strategy chose not to kick.
    if max(math.hypot(f[3] - q[0][3], f[4] - q[0][4]) for f in q) < 100.0:
        m["resultado"] = "ERRO"
        m["motivo"] = "robo parado: a cadeia caiu?"
        return m
    m["duracao"] = round(q[-1][0] - t0, 2)
    m["bola_fim_x"], m["bola_fim_y"] = round(q[-1][1]), round(q[-1][2])

    rolando = any(p.get("bola_vel", (0.0, 0.0)))
    it = toque_em(q, rolando)
    m["alcancou"] = int(it is not None)
    ie = encosto_em(q)
    m["encostou"] = int(ie is not None)
    if ie is not None:
        m["t_encosto"] = round(q[ie][0] - t0, 2)
    # A BOLA AINDA ESTAVA ANDANDO QUANDO ELE CHEGOU?
    #
    # MEDIDO em 28/10: a desaceleracao da bola no grSim e ~1350 mm/s2, entao
    # uma bola lancada a 1200 mm/s PARA em 1,2 s depois de rolar 530 mm. O
    # atacante leva de 1 a 4,5 s para alcancar. Sem esta coluna, metade de uma
    # campanha "com a bola andando" e, no instante que importa, bola parada -
    # e o numero sai diluido sem ninguem ver. Agora da para condicionar a
    # analise a "estava mesmo rolando no contato".
    if it is not None:
        vt = math.hypot(*velocidade(q, it))
        m["vel_bola_toque"] = round(vt)
        m["bola_parada_no_toque"] = int(vt < 150.0)
    n_inv, t_inv = investidas_em(q, None)
    m["investidas"] = n_inv
    if t_inv is not None:
        m["t_1a_investida"] = round(t_inv - t0, 2)
    if it is not None:
        m["t_toque"] = round(q[it][0] - t0, 2)
        m["toque_x"], m["toque_y"] = round(q[it][1]), round(q[it][2])
        # WHERE THE BALL WAS WHEN WE GOT IT is the only origin that means
        # anything once it rolls: drag and recoil measured from the plan's
        # (bx0, by0) would be counting the ball's own travel as the tactic's
        # doing. With the ball at rest the two points are the same place.
        bx0, by0 = (q[it][1], q[it][2]) if rolando else (bx0, by0)
    m["recuo"] = round(max(0.0, bx0 - min(f[1] for f in q[it or 0:])))

    ik, m["trombadas"] = chute_em(q, it or 0)
    cruzou = None
    for a, b in zip(q, q[1:]):
        if a[1] < GOL_X <= b[1]:
            s = (GOL_X - a[1]) / (b[1] - a[1])
            cruzou = a[2] + s * (b[2] - a[2])
            break
    contra = any(f[1] <= -GOL_X and abs(f[2]) <= GOL_MEIA for f in q)

    if ik is not None:
        # The state just before the ball took off: 60 ms back, clear of the kick itself.
        ia = ik
        while ia > 0 and q[ik][0] - q[ia][0] < 0.06:
            ia -= 1
        _, kx, ky, rx, ry, rth, _ = q[ia]
        m.update(chutou=1, t_chute=round(q[ik][0] - t0, 2), bola_chute_x=round(kx),
                 bola_chute_y=round(ky), dist_chute_gol=round(math.hypot(GOL_X - kx, alvo[1] - ky)),
                 arrasto=round(math.hypot(kx - bx0, ky - by0)))
        ideal = math.atan2(alvo[1] - ky, alvo[0] - kx)
        m["erro_corpo"] = round(math.degrees(norm(rth - ideal)), 1)
        # Direction and speed once the ball is free: 80 to 200 ms after take-off.
        depois = [f for f in q[ik:] if 0.08 <= f[0] - q[ik][0] <= 0.20]
        if len(depois) >= 2:
            dt = depois[-1][0] - depois[0][0]
            vx, vy = (depois[-1][1] - depois[0][1]) / dt, (depois[-1][2] - depois[0][2]) / dt
            m["vel_saida"] = round(math.hypot(vx, vy))
            m["erro_direcao"] = round(math.degrees(norm(math.atan2(vy, vx) - ideal)), 1)
            if vx > 1.0:
                # Where the shot was heading on the goal line, even if it died short.
                m["y_na_linha"] = round(depois[0][2] + vy / vx * (GOL_X - depois[0][1]))
        if cruzou is not None:
            m["y_na_linha"] = round(cruzou)
        if "y_na_linha" in m:
            m["erro_y"] = round(m["y_na_linha"] - alvo[1])

    goleiro = next((e for e in p["inimigos"] if e[0] == GOLEIRO_ID), None) \
        if p.get("goleiro") else None
    defendido = ik is not None and goleiro is not None and any(
        math.hypot(f[1] - goleiro[1], f[2] - goleiro[2]) < DEFESA_RAIO for f in q[ik:])
    m["_it"], m["_ik"], m["_quadros"] = it, ik, q
    m.update(primeiro_toque(q, ik, it))
    # A BALL WE NEVER TOUCHED SCORES NOTHING, either way. Launched at the
    # mouth it rolls in by itself ("gol"), launched at our half it rolls into
    # our own net ("gol_contra") - both would be the throw being credited to
    # the tactic. Crossing a line untouched is the attacker not getting there.
    entrou = cruzou is not None and abs(cruzou) <= GOL_MEIA
    if it is None:
        # "encostou de raspao mas nunca teve a bola" e diferente de "nem chegou
        # perto"; as duas contam como nao alcancou, mas o CSV guarda qual foi.
        m["resultado"] = "nao_alcancou"
    elif contra:
        m["resultado"] = "gol_contra"
    elif entrou and ik is None:
        # GOL EMPURRADO NAO E GOL DE CHUTE.
        #
        # de_frente #0 de 28/10 empurrou a bola para dentro ao longo de 14,9 s,
        # sem disparo nenhum, e entrava na mesma conta de um chute colocado.
        # Numa campanha chamada "eficiencia de CHUTE" isso estraga o numero:
        # a taxa de gol sobe por um comportamento que nao e o medido.
        m["resultado"] = "gol_empurrado"
    elif entrou:
        m["resultado"] = "gol"
    elif ik is None:
        m["resultado"] = "sem_chute"
    elif defendido:
        m["resultado"] = "defendido"
    elif cruzou is not None or not no_campo(q[-1][1], q[-1][2], 0.0):
        m["resultado"] = "fora"
    else:
        m["resultado"] = "curto"       # kicked, and the ball died inside the field
    return m


def primeiro_toque(q, ik, it=None):
    """Where the ball sat on the robot's front at the first touch, and if it was the kick.

    grSim fires only with the ball on the plate: 0-31.5 mm in front of it and within
    40 mm of its centre. A first touch outside that is a push.
    """
    if it is None:
        return {}
    _, bx, by, rx, ry, rth, kick = q[max(0, it - 1)]
    if math.hypot(bx - rx, by - ry) > 250.0:
        return {}                       # something else moved it
    dx, dy = bx - rx, by - ry
    xx = dx * math.cos(rth) + dy * math.sin(rth) - CENTRO_ATE_PLACA
    yy = -dx * math.sin(rth) + dy * math.cos(rth)
    foi_chute = ik is not None and abs(q[ik][0] - q[it][0]) < 0.15
    return dict(toque_xx=round(xx), toque_yy=round(yy), toque_armado=int(kick > 0.0),
                toque_foi_chute=int(foi_chute))


REPLAY_HTML = """<!doctype html><meta charset=utf-8><title>tentativa %(i)d - %(geo)s</title>
<style>
 body{background:#11151c;color:#cdd6e4;font:13px/1.5 system-ui,sans-serif;margin:0;padding:14px}
 h1{font-size:15px;margin:0 0 2px} .sub{color:#8d99ae;margin:0 0 10px}
 canvas{background:#16331f;border-radius:6px;max-width:100%%;height:auto;display:block}
 .bar{display:flex;gap:10px;align-items:center;margin:10px 0}
 button{background:#2a9d8f;border:0;color:#06231f;font-weight:600;padding:5px 14px;border-radius:5px;cursor:pointer}
 input[type=range]{flex:1} table{border-collapse:collapse;margin-top:10px;font-size:12px}
 td{padding:2px 12px 2px 0} td:first-child{color:#8d99ae}
 b.ok{color:#2a9d8f} b.ruim{color:#e76f51}
</style>
<h1>tentativa %(i)d &middot; %(geo)s &middot; bola a %(vel)d mm/s</h1>
<p class=sub>%(resumo)s</p>
<canvas id=c width=900 height=600></canvas>
<div class=bar><button id=b>play</button><input id=s type=range min=0 value=0><span id=t></span></div>
<table>%(tabela)s</table>
<script>
const Q=%(quadros)s, META=%(meta)s;
const c=document.getElementById('c'), g=c.getContext('2d');
const W=9000, H=6000, S=c.width/W;
const X=x=>(x+W/2)*S, Y=y=>(H/2-y)*S;      // campo em mm -> pixel, y para cima
function campo(){
  g.fillStyle='#16331f'; g.fillRect(0,0,c.width,c.height);
  g.strokeStyle='#4a7a57'; g.lineWidth=1.5;
  g.strokeRect(X(-4500),Y(3000),9000*S,6000*S);
  g.beginPath(); g.moveTo(X(0),Y(3000)); g.lineTo(X(0),Y(-3000)); g.stroke();
  g.beginPath(); g.arc(X(0),Y(0),500*S,0,7); g.stroke();
  for(const sx of [-1,1]){                   // area de penalti 1000 x 2000
    g.strokeRect(X(sx*4500)-(sx>0?1000*S:0),Y(1000),1000*S,2000*S);
    g.strokeStyle='#cdd6e4';                 // boca do gol
    g.beginPath(); g.moveTo(X(sx*4500),Y(500)); g.lineTo(X(sx*4500),Y(-500)); g.stroke();
    g.strokeStyle='#4a7a57';
  }
  if(META.mira!==null){                      // linha de mira: contato -> alvo
    g.setLineDash([6,5]); g.strokeStyle='#e9c46a'; g.lineWidth=1;
    g.beginPath(); g.moveTo(X(META.mx),Y(META.my)); g.lineTo(X(4500),Y(META.mira)); g.stroke();
    g.setLineDash([]);
  }
}
function trilha(ate){
  g.lineWidth=2; g.strokeStyle='#8d99ae';     // caminho do robo
  g.beginPath(); Q.slice(0,ate+1).forEach((q,k)=>k?g.lineTo(X(q[3]),Y(q[4])):g.moveTo(X(q[3]),Y(q[4]))); g.stroke();
  g.strokeStyle='#ff9f1c'; g.lineWidth=2.5;   // caminho da bola
  g.beginPath(); Q.slice(0,ate+1).forEach((q,k)=>k?g.lineTo(X(q[1]),Y(q[2])):g.moveTo(X(q[1]),Y(q[2]))); g.stroke();
  for(const [idx,cor,rot] of [[META.it,'#3a86ff','toque'],[META.ik,'#e76f51','chute']]){
    if(idx===null||idx>ate) continue;
    g.fillStyle=cor; g.beginPath(); g.arc(X(Q[idx][1]),Y(Q[idx][2]),5,0,7); g.fill();
    g.fillText(rot,X(Q[idx][1])+8,Y(Q[idx][2])-6);
  }
}
function desenha(k){
  const q=Q[k]; campo(); trilha(k);
  g.save(); g.translate(X(q[3]),Y(q[4])); g.rotate(-q[5]);   // robo: casco 90 mm
  g.fillStyle=q[6]>0?'#2a9d8f':'#4f6ef7'; g.beginPath(); g.arc(0,0,90*S,0,7); g.fill();
  g.fillStyle='#fff'; g.fillRect(73*S,-40*S,6,80*S);          // placa do chutador
  g.restore();
  g.fillStyle='#ff9f1c'; g.beginPath(); g.arc(X(q[1]),Y(q[2]),Math.max(3,21*S),0,7); g.fill();
  document.getElementById('t').textContent=(q[0]-Q[0][0]).toFixed(2)+' s';
}
const sl=document.getElementById('s'); sl.max=Q.length-1;
sl.oninput=()=>desenha(+sl.value);
let tocando=null;
document.getElementById('b').onclick=function(){
  if(tocando){clearInterval(tocando);tocando=null;this.textContent='play';return;}
  this.textContent='pause';
  tocando=setInterval(()=>{
    let k=+sl.value+1; if(k>=Q.length){clearInterval(tocando);tocando=null;
      document.getElementById('b').textContent='play';return;}
    sl.value=k; desenha(k);
  },16);
};
desenha(0);
</script>
"""


def escrever_replay(q, m, p, i, caminho):
    """One self-contained HTML per attempt, from the campaign's own raw frames.

    The campaign recorded ball, robot pose and kick at full rate and threw all of
    it away except a 150-point line in trajetorias.jsonl. That is enough for a
    scatter plot and not enough to see WHY an attempt failed - where the ball sat
    on the plate, whether the body was turned, when the kicker armed. The scenario
    path has had gerar_replay() for exactly this reason; the campaign had nothing.
    """
    if len(q) < 3:
        return None
    it = m.get("_it")
    ik = m.get("_ik")
    vel = round(math.hypot(*p["bola_vel"]))
    linhas = [("resultado", m.get("resultado", "")), ("alcancou", m.get("alcancou", "")),
              ("chutou", m.get("chutou", "")), ("t_toque (s)", m.get("t_toque", "")),
              ("t_chute (s)", m.get("t_chute", "")), ("erro_y (mm)", m.get("erro_y", "")),
              ("vel_saida (mm/s)", m.get("vel_saida", "")),
              ("erro_corpo (graus)", m.get("erro_corpo", "")),
              ("toque xx/yy (mm)", f"{m.get('toque_xx','')} / {m.get('toque_yy','')}"),
              ("1o toque foi chute", m.get("toque_foi_chute", "")),
              ("trombadas", m.get("trombadas", "")), ("fantasmas", m.get("fantasmas", ""))]
    ok = m.get("resultado") == "gol"
    with open(caminho, "w") as arquivo:
        arquivo.write(REPLAY_HTML % dict(
            i=i, geo=p["geometria"], vel=vel,
            resumo=f'robo a {round(math.hypot(p["robo"][0]-p["bola"][0], p["robo"][1]-p["bola"][1]))} mm '
                   f'da bola, contorno {m.get("_contorno", "?")} graus, mira '
                   f'{"auto" if p["mira"] is None else round(p["mira"])}',
            tabela="".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in linhas),
            quadros=json.dumps([[round(f[0], 3), round(f[1], 1), round(f[2], 1),
                                 round(f[3], 1), round(f[4], 1), round(f[5], 4), f[6]] for f in q]),
            meta=json.dumps(dict(it=it, ik=ik, mira=p["mira"],
                                 mx=q[it][1] if it is not None else p["bola"][0],
                                 my=q[it][2] if it is not None else p["bola"][1])),
        ))
    return caminho


def chaves_desligadas():
    """As chaves de experimento DESLIGADAS agora, lidas do disco.

    POR QUE ISTO VAI PARA O CSV: em 06/11/2026 um lote de 105 tentativas rodou
    inteiro com INTERCEPTAR_ATE_A_PARADA desligada sem ninguem pedir - a
    bandeira sobrou do pareado anterior (ver 'por_chave': com --par a ULTIMA
    condicao de cada colocacao e 'antes', que CRIA o arquivo, e nada o
    apagava). O lote parecia ser do codigo novo e nao era. Uma linha de CSV
    tem de provar em que condicao ela foi medida, e nao depender de memoria.
    """
    import glob
    return sorted(os.path.basename(c)[len("ararabots_sem_"):].upper()
                  for c in glob.glob("/tmp/ararabots_sem_*"))


def por_chave(chave, ligada):
    """Liga/desliga uma chave de experimento SEM reiniciar o strategyNode.

    skills/experimento.py testa os.path.exists("/tmp/ararabots_sem_<chave>") a
    CADA chamada, entao a bandeira de arquivo troca o comportamento ao vivo. E
    isto que torna o par intercalado possivel.
    """
    caminho = "/tmp/ararabots_sem_" + chave.lower()
    if ligada:
        try:
            os.remove(caminho)
        except OSError:
            pass
    else:
        open(caminho, "w").close()


def tentativa(no, i, p, args):
    if p["mira"] is None:
        try:
            os.remove(ARQUIVO_MIRA)       # the strategy picks its own aim
        except OSError:
            pass
    else:
        with open(ARQUIVO_MIRA, "w") as arquivo:
            arquivo.write(str(p["mira"]))
    cen = dict(bola=p["bola"], azuis=[(PORTADOR,) + p["robo"]], amarelos=p["inimigos"])

    A.enviar_comando_arbitro("HALT")
    A.posicionar(cen)
    no.bola = no.robo = None
    if not esperar_teleporte(no, p):
        return dict(resultado="ERRO", motivo="teleporte nao confirmado pela visao")
    girar(no, 1.5)                    # the movement chain re-anchors under HALT
    A.enviar_comando_arbitro("STOP")
    girar(no, 1.0)
    A.posicionar(cen)                 # the geometry has to hold at t = 0
    girar(no, 0.3)

    no.quadros = []
    no.fantasmas = 0
    no.estranhos_n = 0          # a contagem e POR TENTATIVA (ver LIMITE_ESTRANHOS)
    no.gravando = True
    # THE BALL IS LAUNCHED HERE, not in posicionar().
    #
    # Everything above runs with the ball AT REST, because esperar_teleporte
    # needs to read it at the place the plan asked for. Setup costs seconds
    # (HALT, 1.5 s re-anchor, STOP, 1.0 s, re-teleport, 0.3 s): at 2400 mm/s a
    # ball launched up there would be seven metres away - off the field - by
    # the time FORCE_START arrived, and every attempt would record an empty
    # pitch. Same lesson the scenario path already learnt (ararabots.py, the
    # comment on bola_em_movimento in rodar()).
    #
    # So: last thing before the whistle, teleport it back to its place WITH
    # velocity. t = 0 of the recording is the instant the ball starts rolling.
    if any(p["bola_vel"]):
        A.mover_bola(p["bola"][0], p["bola"][1], *p["bola_vel"])
    A.enviar_comando_arbitro("FORCE_START", "BLUE")
    inicio = time.monotonic()
    chutado_em = parada_desde = None
    while time.monotonic() - inicio < args.duracao:
        girar(no, 0.05)
        q = limpar(no.quadros)
        if len(q) < 3:
            continue
        agora, bx, by = q[-1][0], q[-1][1], q[-1][2]
        vel = math.hypot(*velocidade(q, len(q) - 1))
        if chutado_em is None:
            ik, _ = chute_em(q)
            if ik is not None:
                chutado_em = q[ik][0]
        if not no_campo(bx, by, -150.0):                      # in the goal or off the field
            girar(no, 0.2)
            break
        if chutado_em is not None and agora - chutado_em > 0.5:
            parada_desde = (parada_desde or agora) if vel < 50.0 else None
            if parada_desde is not None and agora - parada_desde > 0.5:
                break
    no.gravando = False
    A.enviar_comando_arbitro("HALT")
    m = medir(list(no.quadros), p)
    m["fantasmas"] = no.fantasmas
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=25, help="attempts to run now")
    ap.add_argument("--total", type=int, default=None, help="stop once the CSV has this many")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--saida", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "campanha"))
    ap.add_argument("--duracao", type=float, default=20.0, help="seconds before giving up")
    ap.add_argument("--x-min", type=float, default=-1500.0)
    ap.add_argument("--x-max", type=float, default=3400.0)
    ap.add_argument("--y-max", type=float, default=2700.0)
    ap.add_argument("--dist-min", type=float, default=300.0, help="robot to ball")
    ap.add_argument("--dist-max", type=float, default=2500.0)
    ap.add_argument("--miras", default="-350,-175,0,175,350",
                    help="aim points inside the goal (y, mm), or 'auto' to let the strategy choose")
    ap.add_argument("--inimigos", type=int, default=0, help="other static yellow robots")
    ap.add_argument("--goleiro", choices=["nenhum", "gol", "area"], default="nenhum",
                    help="static yellow keeper: on the goal line, or anywhere in their area")
    ap.add_argument("--tentativas", default="",
                    help="run exactly these attempt numbers (e.g. 67,115), done or not")
    ap.add_argument("--bola-vel", default="0",
                    help="ball speeds to draw from, mm/s (e.g. 0,800,1600,2400); 0 = at rest; "
                         "'auto' picks a speed per attempt so the ball is STILL ROLLING at "
                         "contact and still fits inside the field")
    ap.add_argument("--geometrias", default="cruza,de_frente,foge_gol,foge_nosso",
                    help="which way a rolling ball goes, relative to the attempt: "
                         "cruza, de_frente, foge_gol, foge_nosso, aleatoria, parada")
    ap.add_argument("--par", default="",
                    help="roda cada colocacao DUAS vezes em sequencia, com a chave de "
                         "experimento ligada e desligada (ex.: --par ATRITO_PREVISAO). "
                         "Intercalado de proposito: lotes separados derivam.")
    ap.add_argument("--sem-replay", action="store_true",
                    help="skip the per-attempt HTML replay (mass runs: one file per attempt)")
    ap.add_argument("--plano", action="store_true", help="print the placements and exit")
    args = ap.parse_args()
    args.miras = [] if args.miras == "auto" else [float(v) for v in args.miras.split(",")]
    args.vel_auto = args.bola_vel.strip().lower() == "auto"
    args.bola_vel = [] if args.vel_auto else [float(v) for v in args.bola_vel.split(",")]
    args.geometrias = [g.strip() for g in args.geometrias.split(",") if g.strip()]
    conhecidas = {"cruza", "de_frente", "foge_gol", "foge_nosso", "aleatoria", "parada"}
    if not set(args.geometrias) <= conhecidas:
        ap.error("geometria desconhecida: %s" % ", ".join(sorted(set(args.geometrias) - conhecidas)))
    # THE SEED IS NOT ENOUGH TO IDENTIFY AN ATTEMPT ANY MORE. plano() draws the
    # speed and the geometry from the same rng, so attempt 7 of seed 1 is a
    # different placement under --bola-vel 0 and under --bola-vel 1600. Mixing
    # them in one CSV would put two different attempts on the same (seed,
    # tentativa) and the resume logic would skip the one never run. One output
    # directory per grid, and the grid is recorded next to the rows.

    caminho = os.path.join(args.saida, "tentativas.csv")
    feitas = set()
    if os.path.exists(caminho):
        with open(caminho) as arquivo:
            linhas = [l for l in csv.DictReader(arquivo)
                      if int(l["seed"]) == args.seed and l["resultado"] != "ERRO"]
        if args.par:
            # So conta pronta a colocacao que tem AS DUAS condicoes; meia
            # tentativa nao serve para par nenhum.
            conds = {}
            for l in linhas:
                conds.setdefault(int(l["tentativa"]), set()).add(l.get("cond", ""))
            feitas = {t for t, c in conds.items() if {"antes", "depois"} <= c}
        else:
            feitas = {int(l["tentativa"]) for l in linhas}

    fila, i = [], 0
    if args.tentativas:
        fila = [int(v) for v in args.tentativas.split(",")]
    while not args.tentativas and len(fila) < args.n and (args.total is None or i < args.total):
        if i not in feitas:
            fila.append(i)
        i += 1
    if args.plano:
        for i in fila:
            print(i, plano(args.seed, i, args))
        return 0
    if not fila:
        print("campanha completa: nada a rodar")
        return 0

    # PORTAO: BANDEIRA SOBRANDO BLOQUEIA O LOTE.
    #
    # Medido em 06/11/2026: 105 tentativas rodaram com
    # INTERCEPTAR_ATE_A_PARADA desligada porque a bandeira sobrou do pareado
    # anterior. O lote tinha pasta nova, pre-voo de rede limpo e portao de
    # grSim de pe - e media outro codigo. Nada apontava para isso.
    #
    # Nao APAGO a bandeira: 'touch /tmp/ararabots_sem_<chave>' e um caminho
    # documentado em skills/experimento.py, e apagar em silencio tiraria uma
    # escolha deliberada de quem a fez. Paro e digo qual e.
    sobrando = chaves_desligadas()
    if sobrando and not args.par:
        print("XX CHAVE DE EXPERIMENTO DESLIGADA:", " ".join(sobrando))
        print("   Este lote mediria o codigo ANTIGO dessas chaves sem dizer.")
        print("   Se foi de proposito, rode com a chave no nome da pasta.")
        print("   Se sobrou de um pareado, limpe:")
        for c in sobrando:
            print("     rm /tmp/ararabots_sem_%s" % c.lower())
        return 1

    import rclpy
    rclpy.init()
    no = criar_no()
    os.makedirs(args.saida, exist_ok=True)
    with open(os.path.join(args.saida, "grade.json"), "w") as arquivo:
        json.dump(dict(seed=args.seed, bola_vel="auto" if args.vel_auto else args.bola_vel,
                       geometrias=args.geometrias,
                       miras=args.miras, inimigos=args.inimigos, goleiro=args.goleiro,
                       x_min=args.x_min, x_max=args.x_max, y_max=args.y_max,
                       dist_min=args.dist_min, dist_max=args.dist_max), arquivo, indent=2)
    novo = not os.path.exists(caminho)
    erros_seguidos = 0
    try:
        with open(caminho, "a", newline="") as arquivo, \
                open(os.path.join(args.saida, "trajetorias.jsonl"), "a") as trajetorias:
            escritor = csv.DictWriter(arquivo, fieldnames=CAMPOS, restval="")
            if novo:
                escritor.writeheader()
            for i in fila:
              p = plano(args.seed, i, args)
              # PAR INTERCALADO. Com --par CHAVE a MESMA colocacao roda duas
              # vezes em sequencia, 'depois' e 'antes', no mesmo processo e com
              # a cadeia no mesmo estado.
              #
              # POR QUE INTERCALAR, e nao rodar dois lotes: em 28/10/2026 rodei
              # os dois bracos como campanhas separadas e a MESMA condicao de
              # base marcou 4/12 num lote e 13/16 no outro. Com uma oscilacao
              # dessas entre lotes, nenhuma diferenca de 2 ou 3 gols significa
              # coisa alguma - e foi assim que um lote disse p=0,06 e o
              # seguinte p=0,89 para a mesma mudanca. Intercalando, a deriva de
              # ambiente cai igual nos dois lados.
              for cond in (["depois", "antes"] if args.par else [""]):
                  if args.par:
                      por_chave(args.par, cond == "depois")
                  (bx, by), (rx, ry, rdir) = p["bola"], p["robo"]
                  m = tentativa(no, i, p, args)
                  # With the strategy choosing its aim, the goal centre stands in for it.
                  a_gol = math.atan2((p["mira"] or 0.0) - by, GOL_X - bx)
                  linha = dict(
                      tentativa=i, seed=args.seed, bola_x=round(bx), bola_y=round(by),
                      robo_x=round(rx), robo_y=round(ry), robo_dir=round(rdir),
                      dist_robo_bola=round(math.hypot(rx - bx, ry - by)),
                      dist_bola_gol=round(math.hypot(GOL_X - bx, by)),
                      # 0 = straight in front of the goal, 90 = on the goal line.
                      angulo_bola_gol=round(math.degrees(math.atan2(by, GOL_X - bx)), 1),
                      # 0 = robot already behind the ball, 180 = ball between it and the goal.
                      angulo_contorno=round(abs(math.degrees(norm(math.atan2(by - ry, bx - rx) - a_gol))), 1),
                      mira_y=p["mira"], inimigos=len(p["inimigos"]),
                      bola_vx=round(p["bola_vel"][0]), bola_vy=round(p["bola_vel"][1]),
                      geometria=p["geometria"],
                      rolagem_prevista=round(rolagem(math.hypot(*p["bola_vel"]))))
                  gk = next((e for e in p["inimigos"] if e[0] == GOLEIRO_ID), None) \
                      if p.get("goleiro") else None
                  if gk is not None:
                      linha.update(goleiro_x=round(gk[1]), goleiro_y=round(gk[2]))
                  linha["cond"] = cond
                  linha["chaves_off"] = " ".join(chaves_desligadas())
                  linha.update(m)

                  # O LOTE PARA AQUI. Ver LIMITE_ESTRANHOS: a visao de outro
                  # grSim na rede nao e ruido, e um segundo robo e uma segunda
                  # bola em cima dos nossos.
                  if no.estranhos_n > LIMITE_ESTRANHOS:
                      motivo_rede = (
                          "outro grSim na rede transmitindo visao em 224.5.23.2: %s "
                          "(%d pacotes nesta tentativa). A estrategia ve dois robos e "
                          "duas bolas. Peca para fechar o grSim dele, ou isole a rede."
                          % (", ".join(sorted(no.estranhos)), no.estranhos_n))
                      linha["resultado"] = "ERRO"
                      linha["motivo"] = motivo_rede
                      escritor.writerow(linha)
                      arquivo.flush()
                      print("XX", motivo_rede, flush=True)
                      return 5
                  # The replay wants the frame indices and the frames; the CSV must
                  # not see them (DictWriter raises on a field it does not know).
                  quadros_limpos = linha.pop("_quadros", [])
                  it_idx, ik_idx = linha.pop("_it", None), linha.pop("_ik", None)
                  escritor.writerow(linha)
                  arquivo.flush()
                  if not args.sem_replay and quadros_limpos:
                      pasta_rep = os.path.join(args.saida, "replays", p["geometria"])
                      os.makedirs(pasta_rep, exist_ok=True)
                      escrever_replay(quadros_limpos,
                                      dict(m, _it=it_idx, _ik=ik_idx,
                                           _contorno=linha["angulo_contorno"]),
                                      p, i, os.path.join(pasta_rep, f"tentativa_{i:04d}.html"))
                  limpos = limpar(no.quadros)
                  passo = max(1, len(limpos) // 150)
                  trajetorias.write(json.dumps(dict(
                      tentativa=i, seed=args.seed, resultado=linha["resultado"],
                      bola=[[round(f[1]), round(f[2])] for f in limpos[::passo]],
                      robo=[[round(f[3]), round(f[4])] for f in limpos[::passo]])) + "\n")
                  trajetorias.flush()
                  mira_txt = "auto" if p["mira"] is None else f"{p['mira']:+5.0f}"
                  vel_txt = "parada" if p["geometria"] == "parada" else \
                      f"{math.hypot(*p['bola_vel']):4.0f} {p['geometria']}"
                  print(f"  #{i:4d} bola ({bx:5.0f},{by:5.0f}) {vel_txt:16s} "
                        f"robo a {linha['dist_robo_bola']:4d} mm "
                        f"contorno {linha['angulo_contorno']:5.1f} mira {mira_txt} -> "
                        f"{linha['resultado']:12s} erro_y={linha.get('erro_y', '')} "
                        f"t_toque={linha.get('t_toque', '')} t={linha.get('t_chute', '')}", flush=True)
                  erros_seguidos = erros_seguidos + 1 if linha["resultado"] == "ERRO" else 0
                  if erros_seguidos >= 3:
                      print("XX tres tentativas seguidas sem montar: a cadeia caiu?")
                      return 4
    finally:
        # O PAREADO DEVOLVE A CHAVE LIGADA. A ultima condicao de cada
        # colocacao e 'antes', que CRIA a bandeira; sem isto ela sobra para o
        # proximo lote. Foi o que contaminou 105 tentativas em 06/11/2026.
        if args.par:
            por_chave(args.par, True)
        if no.estranhos:
            print("!! outro grSim na rede transmitindo visao (ignorado):", ", ".join(sorted(no.estranhos)))
        try:
            os.remove(ARQUIVO_MIRA)
        except OSError:
            pass
        no.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
