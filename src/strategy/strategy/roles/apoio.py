"""APOIO: helps without the ball, either attacking (offers a pass) or marking.

Directions here come from their goal, not from the kick target: the APOIO is
often the pass receiver, and positioning relative to itself produced the
degenerate backwards pass measured on 02/10.
"""

from math import hypot

from strategy.analysis.alvo_chute import TERCO_DEFENSIVO
from strategy.analysis.jogo import AnaliseJogo
from strategy.analysis.mundo import RAIO_POSSE, SITUACAO_DELES, SITUACAO_SOLTA, dist
from strategy.context import GOALKEEPER_ID
from strategy.roles.atribuicao import PAPEL_APOIO
from strategy.roles.base import Papel
from strategy.skills import aproximacao, geometria, posicionamento

# Inside this the robot fights for the ball whatever its label: the situation
# flickers with tracking and the role hysteresis lasts 2 s. Twice RAIO_POSSE.
ENGAJA_RAIO = 450.0


class Apoio(Papel):
    nome = PAPEL_APOIO

    def alvo(self, a: AnaliseJogo, rid: int, ordem: int):
        mundo, ball = a.mundo, a.bola
        r = mundo.aliados[rid]
        bx, by = ball.position_x, ball.position_y
        dgx, dgy = mundo.gol_ataque.x - bx, mundo.gol_ataque.y - by
        ng = hypot(dgx, dgy) or 1.0
        uax, uay = dgx / ng, dgy / ng
        pax, pay = -uay, uax

        # If the ball is mine, I fight for it: do not abandon a ball just won.
        meu = all(dist(r, ball) <= dist(o, ball) + 1.0
                  for rid_o, o in mundo.aliados.items()
                  if rid_o not in (GOALKEEPER_ID, rid))
        if dist(r, ball) < ENGAJA_RAIO and meu:
            if a.alvo_chute is not None:
                avx, avy = a.alvo_chute[0] - bx, a.alvo_chute[1] - by
                nav = hypot(avx, avy) or 1.0
                dx, dy = avx / nav, avy / nav
            else:
                dx, dy = uax, uay
            return geometria.no_campo(bx + dx * aproximacao.AVANCO_PORTADOR,
                                      by + dy * aproximacao.AVANCO_PORTADOR)

        # MARKING: theirs, or in our danger zone. DISPUTA stays out on purpose: it
        # is 51% of the time and nobody would be ahead when we win the ball back.
        perigo = hypot(bx - mundo.nosso_gol.x, by - mundo.nosso_gol.y) < TERCO_DEFENSIVO
        if mundo.situacao == SITUACAO_DELES or perigo:
            inimigos = mundo.inimigos
            if inimigos:
                dono = min(inimigos.values(), key=lambda e: dist(e, ball))
                if dist(dono, ball) < RAIO_POSSE:
                    # Lost it: block the shot line (they shot 5.4 m straight in).
                    return posicionamento.bloquear_linha(bx, by, mundo.nosso_gol)
                ameaca = posicionamento.ameaca_mais_perigosa(inimigos, mundo.nosso_gol)
                if ameaca is not None:
                    return posicionamento.marcar(ameaca, mundo.nosso_gol)
            return posicionamento.bloquear_linha(bx, by, mundo.nosso_gol)

        # LOOSE: someone must be ahead when it is recovered.
        if mundo.situacao == SITUACAO_SOLTA:
            return posicionamento.ocupar_espaco(bx, by, uax, uay)

        # ATTACK: offer a pass ahead and wide; drop behind the ball when blocked.
        lado = 1.0 if (ordem % 2 == 0) else -1.0
        if a.tipo_alvo == "bloqueado":
            return posicionamento.recuo_de_apoio(bx, by, uax, uay, pax, pay, lado)
        return posicionamento.oferta_de_passe(bx, by, uax, uay, pax, pay, lado)

    def evita_bola(self, a: AnaliseJogo, rid: int) -> bool:
        # Any robot may touch the ball; for the support it is never a route obstacle.
        return False
