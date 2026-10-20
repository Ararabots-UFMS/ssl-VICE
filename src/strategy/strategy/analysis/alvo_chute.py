"""Where the ball should go this tick: the goal, a team-mate, a clearance, or nowhere.

Every direction the roles use (body orientation, approach side, push point) comes
from this one decision, so it is made once per tick and locked against flicker.
"""

from dataclasses import dataclass
from math import hypot
from typing import Dict, Optional, Tuple

from strategy import diagnostico
from strategy.analysis.mundo import Mundo
from strategy.roles.atribuicao import PAPEL_APOIO, PAPEL_COBERTURA, PAPEL_PORTADOR
from strategy.skills import avaliacao_chute, experimento, geometria, posicionamento

# The chosen aim holds for 2 s, like the roles. Switching goal<->pass every 10.9
# ticks spun our robots at twice the opponent's rate near the ball.
CICLOS_MIN_MIRA = 20
# Inside our defensive third (field 9000, goal at -4500) clearing beats building.
TERCO_DEFENSIVO = 3000.0
LIMITE_Y = 2600.0
# An opponent this close to the ball or to the carrier is pressing it.
PRESSAO_RAIO = 400.0
PRESSAO_MIN = 1

Ponto = Tuple[float, float]


@dataclass
class MemoriaMira:
    tipo: Optional[str] = None
    ponto: Optional[Ponto] = None
    ciclos: int = 0
    crua: Optional[str] = None          # this tick's choice before the lock
    segurada: bool = False              # the lock kept last tick's aim
    # Pass receiver frozen at decision time as (x, y, rid): a moving receiver made
    # the push point jump every tick and the carrier never lined up (0 of 1272 frames).
    alvo_passe: Optional[Tuple[float, float, int]] = None


@dataclass(frozen=True)
class AlvoChute:
    ponto: Optional[Ponto]
    tipo: Optional[str]          # gol | passe | saida | alivio | bloqueado
    # Aim borrowed from last tick over a 'bloqueado': keep pointing, never arm.
    emprestada: bool
    receptor: Optional[Tuple[float, float, int]]


def _cru(mundo: Mundo, papeis: Dict[int, str], memoria: MemoriaMira):
    """Goal if the line is clear, else the APOIO (or COBERTURA), else nothing.

    74% of our kicks had an opponent on the line within 2 m: kicking into them
    gives the ball back.
    """
    ball, aliados, inimigos = mundo.bola, mundo.aliados, mundo.inimigos
    bx, by = ball.position_x, ball.position_y
    gol_x, gol_y, nota_gol = avaliacao_chute.melhor_alvo(ball, mundo.gol_ataque, inimigos)
    if nota_gol > avaliacao_chute.NOTA_CHUTE_DIRETO:
        memoria.alvo_passe = None
        return gol_x, experimento.mira_y(gol_y), "gol"

    def _recebe(robo):
        return (posicionamento.pode_receber(robo, bx, by, mundo.gol_ataque)
                and geometria.linha_livre(bx, by, robo.position_x, robo.position_y,
                                          inimigos))

    congelado = memoria.alvo_passe
    if congelado is not None:
        atual = aliados.get(congelado[2])
        if (atual is not None and papeis.get(congelado[2]) in (PAPEL_APOIO, PAPEL_COBERTURA)
                and _recebe(atual)):
            return congelado[0], congelado[1], "passe"
        memoria.alvo_passe = None

    # The receiver must be ahead of the ball: a retreating APOIO made passes go back.
    for papel_receptor in (PAPEL_APOIO, PAPEL_COBERTURA):
        for rid, papel in papeis.items():
            if papel != papel_receptor or rid not in aliados:
                continue
            a = aliados[rid]
            if _recebe(a):
                memoria.alvo_passe = (a.position_x, a.position_y, rid)
                return a.position_x, a.position_y, "passe"

    if nota_gol > avaliacao_chute.NOTA_CHUTE_BAIXA_CHANCE:
        memoria.alvo_passe = None
        return gol_x, gol_y, "gol"
    return None, None, "bloqueado"


def _mira_existe(tipo, papeis, memoria: MemoriaMira) -> bool:
    """The goal never moves; a pass needs its receiver still in a receiving role."""
    if tipo != "passe":
        return True
    receptor = memoria.alvo_passe
    if receptor is not None and papeis.get(receptor[2]) == PAPEL_COBERTURA:
        return True
    return any(p == PAPEL_APOIO for p in papeis.values())


def _mira_firme(memoria: MemoriaMira, ax, ay, tipo, papeis):
    """Hold the previous valid aim for CICLOS_MIN_MIRA ticks, also over a 'bloqueado'.

    'bloqueado' is the absence of information, not news: alternating passe/bloqueado
    (132/148 ticks measured) left the carrier orbiting the ball without touching it.
    """
    ant_t, ant_p, n = memoria.tipo, memoria.ponto, memoria.ciclos
    ant_vale = ant_t not in (None, "bloqueado") and ant_p is not None

    if experimento.desligado("MIRA_FIRME"):
        segura = ant_vale and n < CICLOS_MIN_MIRA and tipo not in (None, "bloqueado")
    else:
        segura = ant_vale and n < CICLOS_MIN_MIRA and _mira_existe(ant_t, papeis, memoria)

    memoria.crua = tipo
    memoria.segurada = bool(segura)
    if segura:
        memoria.ciclos = n + 1
        return ant_p[0], ant_p[1], ant_t

    memoria.tipo = tipo
    memoria.ponto = None if ax is None else (ax, ay)
    memoria.ciclos = 0
    return ax, ay, tipo


def _saida_de_bola(mundo: Mundo):
    """In our third with nothing open: clear to the side, half-way up the field.

    The best available line, not a clear one (requiring a clear line fired 2 times
    in 170). Aimed at the touch line so it stays in play: Aimless Kick, Division B.
    """
    ball = mundo.bola
    sent = 1.0 if mundo.gol_ataque.x >= 0 else -1.0
    ly = LIMITE_Y if ball.position_y >= 0 else -LIMITE_Y
    candidatos = [(ball.position_x + 1800.0 * sent, ly * 0.7),
                  (ball.position_x + 2200.0 * sent, 0.0),
                  (ball.position_x + 1800.0 * sent, -ly * 0.7)]
    melhor, melhor_folga = None, -1.0
    for cx, cy in candidatos:
        dx, dy = cx - ball.position_x, cy - ball.position_y
        n = hypot(dx, dy) or 1.0
        folga = geometria.folga_lateral(ball.position_x, ball.position_y,
                                        dx / n, dy / n, n, mundo.inimigos)
        if folga > melhor_folga:
            melhor, melhor_folga = (cx, cy), folga
    return melhor


def _alivio(mundo: Mundo, portador):
    """Pressed with no line: protect the ball by kicking away from whoever presses.

    Pressure is measured on the ball as well as the robot: behind the ball, the
    carrier is too far from a frontal press for its own radius to see it.
    """
    ball, inimigos = mundo.bola, mundo.inimigos

    def _perto(e, alvo):
        return hypot(e.position_x - alvo.position_x, e.position_y - alvo.position_y) < PRESSAO_RAIO

    if experimento.desligado("PRESSAO_BOLA"):
        perto = sum(1 for e in inimigos.values() if _perto(e, portador))
    else:
        perto = sum(1 for e in inimigos.values() if _perto(e, ball) or _perto(e, portador))
    if perto < PRESSAO_MIN:
        return None

    ameaca = min(inimigos.values(),
                 key=lambda e: hypot(e.position_x - portador.position_x,
                                     e.position_y - portador.position_y))
    if experimento.desligado("PROTECAO"):
        lat_y = LIMITE_Y if ball.position_y >= 0 else -LIMITE_Y
        avanco = 800.0 if mundo.gol_ataque.x >= 0 else -800.0
        alvo = (ball.position_x + avanco, lat_y)
    else:
        alvo = posicionamento.saida_sob_pressao(
            ball.position_x, ball.position_y, inimigos, mundo.gol_ataque,
            mundo.nosso_gol, ameaca=ameaca)
    if alvo is not None:
        diagnostico.log("[JG] ALIVIO perto=%d -> %.0f,%.0f" % (perto, alvo[0], alvo[1]))
    return alvo


def alvo_do_chute(mundo: Mundo, papeis: Dict[int, str], memoria: MemoriaMira) -> AlvoChute:
    ax, ay, tipo = _cru(mundo, papeis, memoria)
    ax, ay, tipo = _mira_firme(memoria, ax, ay, tipo, papeis)
    ponto = None if ax is None else (ax, ay)
    emprestada = bool(memoria.segurada and memoria.crua in (None, "bloqueado"))

    ball = mundo.bola
    d_nosso_gol = hypot(ball.position_x - mundo.nosso_gol.x,
                        ball.position_y - mundo.nosso_gol.y)
    if d_nosso_gol < TERCO_DEFENSIVO and tipo in (None, "bloqueado"):
        saida = _saida_de_bola(mundo)
        if saida is not None:
            ponto, tipo = saida, "saida"
        diagnostico.log("[JG] SAIDA d_gol=%.0f -> %s" % (d_nosso_gol, tipo))

    if ponto is None and mundo.inimigos:
        rid_p = next((k for k, v in papeis.items() if v == PAPEL_PORTADOR), None)
        if rid_p is not None and rid_p in mundo.aliados:
            alivio = _alivio(mundo, mundo.aliados[rid_p])
            if alivio is not None:
                ponto, tipo = alivio, "alivio"

    diagnostico.log("[JG] alvo_chute=%s inimigos=%d situacao=%s"
                    % (tipo, len(mundo.inimigos or {}), mundo.situacao))
    return AlvoChute(ponto=ponto, tipo=tipo, emprestada=emprestada,
                     receptor=memoria.alvo_passe)
