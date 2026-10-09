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


def prever_direcao_chute(shooter, ball, goal_x, goal_half_width=900.0,
                        distancia_maxima_mm=550.0, angulo_limite_rad=0.6):
    """Heuristica conservadora: o atacante esta mirando ao gol?

    A ideia e simples e robusta: a direcao do corpo do atacante, comparada com a
    linha bola->gol, define a chance de chute ao gol. Quando isso fica dentro de
    um angulo pequeno (0,6 rad ~= 35 graus), tratamos como chute em direcao ao
    gol. Sem orientacao confiavel, o fallback usa o vetor atacante->bola.
    """
    if shooter is None or ball is None:
        return None
    if bola_ja_saiu(ball):
        return None

    bx = getattr(ball, "position_x", 0.0)
    by = getattr(ball, "position_y", 0.0)
    sx = getattr(shooter, "position_x", bx)
    sy = getattr(shooter, "position_y", by)
    dist = hypot(sx - bx, sy - by)
    if dist > distancia_maxima_mm:
        return None

    heading = getattr(shooter, "orientation", None)
    if heading is None:
        dx = bx - sx
        dy = by - sy
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        heading = atan2(dy, dx)

    ang_meta = atan2(0.0 - by, goal_x - bx)
    if abs(_norm_ang(heading - ang_meta)) > angulo_limite_rad:
        # O atacante pode estar olhando para o lado errado ou para outro alvo.
        return None

    candidatos = [
        (goal_x, 0.0),
        (goal_x, goal_half_width),
        (goal_x, -goal_half_width),
    ]
    melhor = None
    melhor_erro = None
    for cx, cy in candidatos:
        ang_cand = atan2(cy - by, cx - bx)
        erro = abs(_norm_ang(heading - ang_cand))
        if melhor_erro is None or erro < melhor_erro:
            melhor_erro = erro
            melhor = (cx, cy)

    if melhor is None:
        return None

    alvo_x, alvo_y = melhor
    tempo = max(0.15, hypot(alvo_x - bx, alvo_y - by) / max(VEL_BOLA_CHUTADA, 1.0))
    return {
        "tipo": "gol",
        "target_x": alvo_x,
        "target_y": alvo_y,
        "tempo": tempo,
        "angulo": heading,
        "erro": melhor_erro,
    }
