"""O que a estrategia sabe sobre a bola: previsao e "ja foi chutada?".

CAMADA DE SKILLS - nivel 1.

ATENCAO A FONTE DOS NUMEROS: a velocidade da bola chega pelo filtro de Kalman,
que SUBNOTIFICA e atrasa. MEDIDO no /game_state durante um jogo, 526 amostras:
    p50 = 15    p90 = 36    max = 1321 mm/s
enquanto a visao crua mostra picos de 6000. Qualquer limiar daqui tem de vir do
que a ESTRATEGIA enxerga, nao da fisica.
"""

from math import hypot

# Acima desta velocidade a bola JA FOI CHUTADA - ninguem persegue, intercepta.
#
# POR QUE ISTO PRECISA EXISTIR: o contato SUBTRAI velocidade da bola
# (grSim robot.cpp:168, KickerDampFactor). Quem alcanca a bola em voo a FREIA -
# medimos a bola partindo a 5929 mm/s e andando 200 mm porque o proprio robo a
# alcancou e segurou.
#
# 250 mm/s, e nao 800: com o limiar em 800 a condicao so fechava no pico, por um
# ou dois quadros, quando o robo ja tinha alcancado a bola e freado. 250 fica
# muito acima do ruido (p90 = 36) e pega a bola ainda saindo.
VEL_BOLA_CHUTADA = 250.0


def velocidade(ball):
    vx = getattr(ball, "velocity_x", 0.0) or 0.0
    vy = getattr(ball, "velocity_y", 0.0) or 0.0
    return vx, vy


def bola_ja_saiu(ball):
    """A bola esta viajando? Entao nao ha o que empurrar."""
    vx, vy = velocidade(ball)
    return hypot(vx, vy) > VEL_BOLA_CHUTADA


def onde_a_bola_vai(ball, segundos=0.5):
    """Posicao prevista da bola daqui a 'segundos'.

    A previsao e conservadora POR CONSTRUCAO, porque a velocidade vem do Kalman
    (ver o cabecalho do modulo): erra para perto, nunca para longe. Serve para
    escolher ONDE ESPERAR, nao para cronometrar nada.
    """
    vx, vy = velocidade(ball)
    return ball.position_x + vx * segundos, ball.position_y + vy * segundos
