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

CAMPOS = [
    "tentativa", "seed", "bola_x", "bola_y", "robo_x", "robo_y", "robo_dir",
    "dist_robo_bola", "dist_bola_gol", "angulo_bola_gol", "angulo_contorno",
    "mira_y", "inimigos", "resultado", "chutou", "t_chute", "bola_chute_x",
    "bola_chute_y", "dist_chute_gol", "arrasto", "recuo", "vel_saida",
    "erro_corpo", "erro_direcao", "y_na_linha", "erro_y", "bola_fim_x",
    "bola_fim_y", "duracao", "trombadas", "fantasmas", "motivo",
]


def na_area(x, y, folga):
    return abs(x) > MEIO_CAMPO_X - AREA_FUNDO - folga and abs(y) < AREA_MEIA + folga


def no_campo(x, y, folga):
    return abs(x) < MEIO_CAMPO_X - folga and abs(y) < MEIO_CAMPO_Y - folga


def plano(seed, i, args):
    """The placement of attempt i. Rejection sampling, deterministic in (seed, i)."""
    rng = random.Random(seed * 1000003 + i)
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
    mira = rng.choice(args.miras)

    inimigos = []
    for rid in range(args.inimigos):
        while True:
            ex = rng.uniform(max(bx, -1000.0), MEIO_CAMPO_X - 300.0)
            ey = rng.uniform(-MEIO_CAMPO_Y + 300.0, MEIO_CAMPO_Y - 300.0)
            livre = all(math.hypot(ex - px, ey - py) > 500.0
                        for px, py in [(bx, by), (rx, ry)] + [(e[1], e[2]) for e in inimigos])
            if livre:
                break
        inimigos.append((rid, ex, ey, 180.0))
    return dict(bola=(bx, by), robo=(rx, ry, rdir), mira=mira, inimigos=inimigos)


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
                else:
                    self.bola = bolas[0]
                if self.gravando and self.robo:
                    self.quadros.append((quadro.t_capture,) + bolas[0] + self.robo + (self.kick,))

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


def chute_em(q):
    """Index of the first frame where the ball takes off WITH the kicker armed, and
    how many times it was knocked that fast without it (a bump, not a kick)."""
    trombadas, rapido = 0, False
    for i in range(1, len(q)):
        if math.hypot(*velocidade(q, i)) <= VEL_CHUTE:
            rapido = False
            continue
        if rapido:
            continue
        rapido = True
        armado = any(f[6] > 0.0 for f in q[max(0, i - 25):i + 3])
        if armado:
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
    alvo = (GOL_X, p["mira"])
    m = dict(chutou=0, resultado="sem_chute")
    q = limpar(q)
    if len(q) < 5:
        m["resultado"] = "ERRO"
        m["motivo"] = "sem quadros"
        return m
    t0 = q[0][0]
    m["duracao"] = round(q[-1][0] - t0, 2)
    m["bola_fim_x"], m["bola_fim_y"] = round(q[-1][1]), round(q[-1][2])
    m["recuo"] = round(max(0.0, bx0 - min(f[1] for f in q)))

    ik, m["trombadas"] = chute_em(q)
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

    if contra:
        m["resultado"] = "gol_contra"
    elif cruzou is not None and abs(cruzou) <= GOL_MEIA:
        m["resultado"] = "gol"
    elif ik is None:
        m["resultado"] = "sem_chute"
    elif cruzou is not None or not no_campo(q[-1][1], q[-1][2], 0.0):
        m["resultado"] = "fora"
    else:
        m["resultado"] = "curto"       # kicked, and the ball died inside the field
    return m


def tentativa(no, i, p, args):
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
    no.gravando = True
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
                    help="aim points inside the goal (y, mm); '0' is the stock centre aim")
    ap.add_argument("--inimigos", type=int, default=0, help="static yellow robots (ids 0..n-1)")
    ap.add_argument("--plano", action="store_true", help="print the placements and exit")
    args = ap.parse_args()
    args.miras = [float(v) for v in args.miras.split(",")]

    caminho = os.path.join(args.saida, "tentativas.csv")
    feitas = set()
    if os.path.exists(caminho):
        with open(caminho) as arquivo:
            feitas = {int(l["tentativa"]) for l in csv.DictReader(arquivo)
                      if int(l["seed"]) == args.seed and l["resultado"] != "ERRO"}

    fila, i = [], 0
    while len(fila) < args.n and (args.total is None or i < args.total):
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

    import rclpy
    rclpy.init()
    no = criar_no()
    os.makedirs(args.saida, exist_ok=True)
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
                (bx, by), (rx, ry, rdir) = p["bola"], p["robo"]
                m = tentativa(no, i, p, args)
                a_gol = math.atan2(p["mira"] - by, GOL_X - bx)
                linha = dict(
                    tentativa=i, seed=args.seed, bola_x=round(bx), bola_y=round(by),
                    robo_x=round(rx), robo_y=round(ry), robo_dir=round(rdir),
                    dist_robo_bola=round(math.hypot(rx - bx, ry - by)),
                    dist_bola_gol=round(math.hypot(GOL_X - bx, by)),
                    # 0 = straight in front of the goal, 90 = on the goal line.
                    angulo_bola_gol=round(math.degrees(math.atan2(by, GOL_X - bx)), 1),
                    # 0 = robot already behind the ball, 180 = ball between it and the goal.
                    angulo_contorno=round(abs(math.degrees(norm(math.atan2(by - ry, bx - rx) - a_gol))), 1),
                    mira_y=p["mira"], inimigos=len(p["inimigos"]))
                linha.update(m)
                escritor.writerow(linha)
                arquivo.flush()
                limpos = limpar(no.quadros)
                passo = max(1, len(limpos) // 150)
                trajetorias.write(json.dumps(dict(
                    tentativa=i, seed=args.seed, resultado=linha["resultado"],
                    bola=[[round(f[1]), round(f[2])] for f in limpos[::passo]],
                    robo=[[round(f[3]), round(f[4])] for f in limpos[::passo]])) + "\n")
                trajetorias.flush()
                print(f"  #{i:4d} bola ({bx:5.0f},{by:5.0f}) robo a {linha['dist_robo_bola']:4d} mm "
                      f"contorno {linha['angulo_contorno']:5.1f} mira {p['mira']:+5.0f} -> "
                      f"{linha['resultado']:9s} erro_y={linha.get('erro_y', '')} "
                      f"t={linha.get('t_chute', '')}", flush=True)
                erros_seguidos = erros_seguidos + 1 if linha["resultado"] == "ERRO" else 0
                if erros_seguidos >= 3:
                    print("XX tres tentativas seguidas sem montar: a cadeia caiu?")
                    return 4
    finally:
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
