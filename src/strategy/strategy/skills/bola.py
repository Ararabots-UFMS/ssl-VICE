"""O que a estrategia sabe sobre a bola: previsao e "ja foi chutada?".

CAMADA DE SKILLS - nivel 1.

ATENCAO A FONTE DOS NUMEROS: a velocidade da bola chega pelo filtro de Kalman,
que SUBNOTIFICA e atrasa. MEDIDO no /game_state durante um jogo, 526 amostras:
    p50 = 15    p90 = 36    max = 1321 mm/s
enquanto a visao crua mostra picos de 6000. Qualquer limiar daqui tem de vir do
que a ESTRATEGIA enxerga, nao da fisica.
"""

from math import atan2, hypot, pi

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


def _norm_ang(ang):
    return ((ang + pi) % (2.0 * pi)) - pi


def cruzamento_da_bola(ball, goal_x, goal_half_width=GOL_MEIA_LARGURA,
                       margem_mm=MARGEM_GOL):
    """Bola JÁ EM VOO: onde (y) ela cruza a linha do gol, ou None."""
    vx, vy = velocidade(ball)
    if hypot(vx, vy) <= VEL_BOLA_CHUTADA:
        return None
    bx = getattr(ball, "position_x", 0.0)
    by = getattr(ball, "position_y", 0.0)
    ate_o_gol = goal_x - bx
    if vx * ate_o_gol <= 1e-9:          # indo para longe do gol
        return None
    y_cruza = by + vy * ate_o_gol / vx
    if abs(y_cruza) > goal_half_width + margem_mm:
        return None
    return y_cruza


def prever_direcao_chute(shooter, ball, goal_x, goal_half_width=GOL_MEIA_LARGURA,
                         distancia_maxima_mm=550.0, angulo_limite_rad=0.6,
                         margem_mm=MARGEM_GOL):
    if shooter is None or ball is None:
        return None
    if bola_ja_saiu(ball):
        return None          # bola em voo: ver cruzamento_da_bola()

    bx = getattr(ball, "position_x", 0.0)
    by = getattr(ball, "position_y", 0.0)
    sx = getattr(shooter, "position_x", bx)
    sy = getattr(shooter, "position_y", by)
    if hypot(sx - bx, sy - by) > distancia_maxima_mm:
        return None

    heading = getattr(shooter, "orientation", None)
    if heading is None:
        dx = bx - sx
        dy = by - sy
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        heading = atan2(dy, dx)

    # a bola precisa estar À FRENTE do atacante
    erro_bola = abs(_norm_ang(heading - atan2(by - sy, bx - sx)))
    if erro_bola > angulo_limite_rad:
        return None

    # onde o raio da mira, saindo da bola, cruza a linha do gol
    ate_o_gol = goal_x - bx
    c = cos(heading)
    if c * ate_o_gol <= 1e-9:            # mirando para longe do gol
        return None
    y_cruza = by + sin(heading) * ate_o_gol / c
    if abs(y_cruza) > goal_half_width + margem_mm:
        return None

    tempo = max(0.15, hypot(ate_o_gol, y_cruza - by) / VEL_CHUTE_TIPICA)
    return {"tipo": "gol", "target_x": goal_x, "target_y": y_cruza,
            "tempo": tempo, "angulo": heading, "erro": erro_bola}