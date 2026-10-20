"""COBERTURA: between the ball and our goal, and second to the ball when it is theirs."""

from math import hypot

from strategy.analysis.jogo import AnaliseJogo
from strategy.analysis.mundo import SITUACAO_DELES, SITUACAO_DISPUTA
from strategy.roles.atribuicao import PAPEL_COBERTURA
from strategy.roles.base import Papel, versor_ataque
from strategy.skills import geometria, posicionamento

# With the ball this close to our goal the first cover goes ON the ball: a shot
# crosses the field in 0.8 s, so waiting on the line is too late. Our third.
COBERTURA_MATA = 2500.0


class Cobertura(Papel):
    nome = PAPEL_COBERTURA

    def alvo(self, a: AnaliseJogo, rid: int, ordem: int):
        mundo, ball = a.mundo, a.bola
        bx, by = ball.position_x, ball.position_y
        r = mundo.aliados[rid]

        # Chosen as the pass receiver: leave the line and offer the frozen point.
        receptor = a.alvo.receptor
        if receptor is not None and receptor[2] == rid:
            return geometria.no_campo(receptor[0], receptor[1])

        # Theirs or contested: second to the ball, from the other side. Without a
        # dribbler two angles of attack give two ways out instead of one.
        if mundo.situacao in (SITUACAO_DELES, SITUACAO_DISPUTA):
            ux, uy, px, py = versor_ataque(a)
            lado = 1.0 if (ordem % 2 == 0) else -1.0
            return geometria.no_campo(bx + px * 240.0 * lado + ux * 100.0,
                                      by + py * 240.0 * lado + uy * 100.0)

        if hypot(bx - mundo.nosso_gol.x, by - mundo.nosso_gol.y) < COBERTURA_MATA and ordem == 0:
            # On the ball and beyond it, so it does not arrive braking.
            dx, dy = bx - r.position_x, by - r.position_y
            n = hypot(dx, dy) or 1.0
            return geometria.no_campo(bx + dx / n * 400.0, by + dy / n * 400.0)

        # On the ball->goal line, the next ones closer to the goal: two bodies in the
        # same corridor cover the rebound, two side by side cover nothing.
        return posicionamento.cobertura_na_linha(bx, by, mundo.nosso_gol,
                                                 recuo_extra=500.0 * ordem)
