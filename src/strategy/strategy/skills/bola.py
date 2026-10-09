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


# PERSEGUIR A BOLA EM MOVIMENTO: quando, e por quanto.
#
# ATENCAO - ESTAS DUAS FUNCOES NAO ESTAO EM USO, E O MOTIVO IMPORTA.
#
# Elas nasceram para corrigir a "investida fantasma" trocando o limiar de 250
# por 600 mm/s e o avanco fixo de 1600 mm por meio segundo de bola. A MEDICAO
# REFUTOU (09/10/2026, mesmos cinco cenarios): o avanco grande nao e ruido, e o
# seguimento que atravessa a bola e faz o chute sair.
#
#     cenario       avanco de 1600        avanco proporcional
#     lado_esq      +4248 e CHUTE         +1112, sem chute
#     diagonal      +4745 e CHUTE         +1626, sem chute
#
# A correcao que ficou esta em tatics/running.py: o ramo de perseguicao passou a
# ser condicionado a FASE da aproximacao - persegue quem ja esta atras da bola.
# Ficam aqui porque o limiar de 600 mm/s para "bola realmente viajando" segue
# valido como medida (11% dos quadros passam de 250 com a bola quase parada), e
# porque apagar a medicao junto com o codigo e como a tatica virou uma maquina
# que ninguem sabia explicar.
#
# DEFEITO QUE ISTO CORRIGE - a "investida fantasma". O portador abandonava a
# aproximacao e ia para o ponto de interceptacao sempre que a bola passava de
# VEL_BOLA_CHUTADA (250 mm/s), mirando o ponto previsto MAIS 1600 mm alem dele
# (AVANCO_SOLTA). Duas coisas erradas de uma vez:
#
#   1. 250 mm/s nao e "a bola esta viajando", e ruido de contato. MEDIDO nos
#      replays de 'portador_longe_atras' e 'portador_bola_diagonal': 11% dos
#      quadros estao acima de 250 mm/s, com a bola praticamente parada no
#      campo. Em 1 de cada 9 ciclos o alvo saltava ~1,6 m;
#   2. o avanco de 1600 mm e fixo, independente de a bola estar a 300 ou a
#      5000 mm/s. Com a bola devagar, o alvo fica num lugar para onde ela nunca
#      vai - e o robo investe ali.
#
# 600 mm/s: acima do ruido de contato e abaixo de um passe fraco (medimos passe
# a 2159 mm/s). O avanco passa a ser proporcional - meio segundo de bola, que e
# o mesmo horizonte da previsao -, entao ele nunca aponta para alem de onde ela
# chega.
VEL_PERSEGUIR = 600.0


def bola_viajando(ball):
    """A bola esta REALMENTE viajando? (nao e ruido de contato)"""
    vx, vy = velocidade(ball)
    return hypot(vx, vy) > VEL_PERSEGUIR


def avanco_da_perseguicao(ball, teto):
    """Quanto mirar ALEM do ponto previsto: meio segundo de bola, no maximo."""
    vx, vy = velocidade(ball)
    return min(teto, hypot(vx, vy) * 0.5)


def onde_a_bola_vai(ball, segundos=0.5):
    """Posicao prevista da bola daqui a 'segundos'.

    A previsao e conservadora POR CONSTRUCAO, porque a velocidade vem do Kalman
    (ver o cabecalho do modulo): erra para perto, nunca para longe. Serve para
    escolher ONDE ESPERAR, nao para cronometrar nada.
    """
    vx, vy = velocidade(ball)
    return ball.position_x + vx * segundos, ball.position_y + vy * segundos
