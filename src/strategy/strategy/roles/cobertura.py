"""COBERTURA: the defender. Holds the ball's shadow on our goal and stays in our half.

It stands on the line ball -> our goal (posicionamento.cobertura_defensiva): first
onto that line at its nearest point, then forward along it to close the gap. Never
past midfield: the target is clamped and the planner gets the defensive_half
obstacle. It never arms the kick, except:

  BUSCADOR   with no carrier assigned (the FORCAR_COBERTURA test mode), the closest
             defender goes for the ball: from the shadow it kicks the ball away along
             it; from the attacking side of the ball it circles round first.
  NA SOMBRA  within contact range and on the shadow line, a defender may clear: a pass
             to the carrier or another defender within 20 deg of the line, else at
             their goal.
"""

from dataclasses import dataclass, field
from math import atan2, hypot
from typing import Dict

from strategy.analysis.alvo_chute import TERCO_DEFENSIVO
from strategy.analysis.jogo import AnaliseJogo
from strategy.context import GOALKEEPER_ID
from strategy.roles.atribuicao import PAPEL_COBERTURA, PAPEL_PORTADOR
from strategy.roles.base import Papel
from strategy.skills import aproximacao, chute, geometria, posicionamento

# The planner's half-field barrier wants the centre more than 90 mm from midfield.
# 150 left the ball off the kicker plate at midfield (contact needs < ~107 mm).
LIMITE_MEIO_CAMPO = 100.0
# Already in the shadow last tick: a wider band, so ball noise does not flip it.
TOL_SOMBRA_MANTIDA = 120.0
# How far past the ball the clearance aims, along the shadow line.
ALCANCE_SAIDA = 1800.0
# A pass from the shadow must leave within this of the shadow line, or the ball
# leaves the kicker plate: ~20 deg.
DESVIO_MAX_PASSE = 0.35


@dataclass
class MemoriaCobertura:
    # Robot id -> was in the shadow last tick (ball-chaser only).
    na_sombra: Dict[int, bool] = field(default_factory=dict)


def _versor_sombra(a: AnaliseJogo):
    bola, gol = a.bola, a.mundo.nosso_gol
    ux, uy, _ = geometria.versor(bola.position_x, bola.position_y, gol.x, gol.y)
    return ux, uy


def _passe_da_sombra(a: AnaliseJogo, rid: int, ux_sombra: float, uy_sombra: float):
    """Where a defender in the shadow kicks: a team-mate along the shadow line, or goal.

    The pass has to fit the direction the robot arrives from; aiming far to the side
    takes the ball off the plate, and moving the robot for it would leave the shadow.
    """
    mundo, bola = a.mundo, a.bola
    bx, by = bola.position_x, bola.position_y
    dir_x, dir_y = -ux_sombra, -uy_sombra
    candidatos = []
    for outro, papel in a.papeis.items():
        if (outro in (GOALKEEPER_ID, rid) or papel not in (PAPEL_PORTADOR, PAPEL_COBERTURA)
                or outro not in mundo.aliados):
            continue
        receptor = mundo.aliados[outro]
        if not posicionamento.pode_receber(receptor, bx, by, mundo.gol_ataque):
            continue
        if not geometria.linha_livre(bx, by, receptor.position_x, receptor.position_y,
                                     mundo.inimigos):
            continue
        vx, vy, distancia = geometria.versor(bx, by, receptor.position_x, receptor.position_y)
        desvio = atan2(abs(dir_x * vy - dir_y * vx), dir_x * vx + dir_y * vy)
        if desvio > DESVIO_MAX_PASSE:
            continue
        prioridade = 0 if papel == PAPEL_PORTADOR else 1
        candidatos.append(((prioridade, desvio, -distancia, outro),
                           (receptor.position_x, receptor.position_y)))
    if candidatos:
        return min(candidatos)[1], "passe"
    return (mundo.gol_ataque.x, mundo.gol_ataque.y), "gol"


class Cobertura(Papel):
    nome = PAPEL_COBERTURA

    def __init__(self):
        self.memoria = MemoriaCobertura()

    def _na_sombra(self, a: AnaliseJogo, rid: int) -> bool:
        """This tick's shadow flag for the ball-chaser, as decided in alvo()."""
        return rid == a.buscador and self.memoria.na_sombra.get(rid, False)

    def alvo(self, a: AnaliseJogo, rid: int, ordem: int):
        mundo, bola = a.mundo, a.bola
        bx, by = bola.position_x, bola.position_y
        r = mundo.aliados[rid]
        nosso = mundo.nosso_gol
        buscar = rid == a.buscador
        ux_s, uy_s = _versor_sombra(a)

        na_sombra = False
        if buscar:
            tol = (TOL_SOMBRA_MANTIDA if self.memoria.na_sombra.get(rid)
                   else posicionamento.TOL_CENTRO_SOMBRA)
            na_sombra = posicionamento.na_sombra_da_bola(r, bx, by, nosso, tolerancia=tol)
            self.memoria.na_sombra[rid] = na_sombra
        else:
            self.memoria.na_sombra.pop(rid, None)

        if na_sombra:
            # Through the ball along the shadow: the target past it keeps the tracker
            # active through the firing window.
            x, y = bx - aproximacao.EMPURRAO * ux_s, by - aproximacao.EMPURRAO * uy_s
        elif buscar and (r.position_x - bx) * mundo.sentido > 0:
            # On the attacking side of the ball: circle round to come in from behind.
            x, y = aproximacao.ponto_de_aproximacao(
                r.position_x, r.position_y, bx, by,
                (bx - ALCANCE_SAIDA * ux_s, by - ALCANCE_SAIDA * uy_s),
                aproximacao.AVANCO_PORTADOR)
        else:
            x, y = posicionamento.cobertura_defensiva(bx, by, nosso, ordem, r)

        # Never past midfield. The ball-chaser keeps to the shadow line while clamped.
        lado = 1.0 if mundo.on_positive_half else -1.0
        if lado * x < LIMITE_MEIO_CAMPO:
            x = lado * LIMITE_MEIO_CAMPO
            if buscar and abs(nosso.x - bx) > 1e-6:
                y = by + (x - bx) / (nosso.x - bx) * (nosso.y - by)
        return x, y

    def mira(self, a: AnaliseJogo, rid: int):
        bola, r = a.bola, a.mundo.aliados[rid]
        bx, by = bola.position_x, bola.position_y
        ux_s, uy_s = _versor_sombra(a)
        na_sombra = self._na_sombra(a, rid)
        if na_sombra:
            nosso = a.mundo.nosso_gol
            perigo = hypot(bx - nosso.x, by - nosso.y) < TERCO_DEFENSIVO
            ponto = (bx - ALCANCE_SAIDA * ux_s, by - ALCANCE_SAIDA * uy_s)
            tipo = "saida" if perigo else "gol"
        else:
            ponto, tipo = None, "bloqueado"
        perto = (hypot(r.position_x - bx, r.position_y - by) < chute.ALCANCE_CONTATO
                 and (na_sombra or posicionamento.na_sombra_da_bola(
                     r, bx, by, a.mundo.nosso_gol)))
        if perto:
            ponto, tipo = _passe_da_sombra(a, rid, ux_s, uy_s)
        return ponto, tipo, perto or na_sombra or not a.alvo.emprestada

    def evita_bola(self, a: AnaliseJogo, rid: int) -> bool:
        # The defender may touch the ball; it is never a route obstacle for it.
        return False

    def metade_defensiva(self, a: AnaliseJogo, rid: int) -> int:
        return 1 if a.mundo.on_positive_half else -1
