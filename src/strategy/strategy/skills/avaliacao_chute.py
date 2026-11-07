"""Avaliacao geometrica de chute ao gol, sem dependencias de ROS."""

from math import atan2, pi, tan

from strategy.skills.geometria import linha_livre


LARGURA_GOL_MM = 1000.0
FOLGA_CHUTE_MM = 180.0
AMOSTRAS_ANGULARES = 81
ANGULO_ABERTURA_NOTA_MAX = 0.3
NOTA_CHUTE_DIRETO = 0.3
NOTA_CHUTE_BAIXA_CHANCE = 0.1

# O ULTIMO CALCULO DE VAO, para quem quiser VER a nota trabalhando.
#
# Nao e log: e um dicionario sobrescrito a cada chamada, custo de uma
# atribuicao. Quem grava e quem pediu diagnostico (ver analysis/alvo_chute.py),
# porque I/O dentro desta funcao entraria no caminho de decisao de todo ciclo.
#
# Por que existe: a nota e o vao escolhido NAO apareciam em lugar nenhum. Com
# goleiro adversario em campo, "ele chutou no canto errado" e "ele achou o
# unico vao que havia" produzem o mesmo CSV, e nao da para separar os dois sem
# ver os candidatos que ele descartou.
ULTIMO_VAO = {}


def _angulo_relativo(angulo, referencia):
    return (angulo - referencia + pi) % (2.0 * pi) - pi


def melhor_alvo(ball, gol, inimigos):
    """Devolve (x, y, nota) do maior vao livre da boca do gol.

    A nota e uma heuristica angular em [0, 1], nao uma probabilidade calibrada
    de gol. Cada raio amostrado e validado pela mesma margem usada pelo VICE
    para verificar linhas de chute e passe.
    """
    x, y, nota, _ = melhor_vao(ball, gol, inimigos)
    return x, y, nota


def melhor_vao(ball, gol, inimigos):
    """Como melhor_alvo, e tambem as bordas do vao: (x, y, nota, (a_min, a_max)).

    As bordas sao angulos absolutos vistos da bola; None quando nao ha vao.
    """
    bx, by = float(ball.position_x), float(ball.position_y)
    gx, gy = float(gol.x), float(gol.y)
    dx = gx - bx
    if abs(dx) < 1e-6:
        return None, None, 0.0, None

    half_width = LARGURA_GOL_MM / 2.0
    center_angle = atan2(gy - by, dx)
    angle_a = atan2(gy - half_width - by, dx)
    angle_b = atan2(gy + half_width - by, dx)
    offsets = sorted((
        _angulo_relativo(angle_a, center_angle),
        _angulo_relativo(angle_b, center_angle),
    ))
    lo, hi = offsets
    if hi - lo < 1e-9:
        return None, None, 0.0, None

    step = (hi - lo) / (AMOSTRAS_ANGULARES - 1)
    open_samples = []
    for i in range(AMOSTRAS_ANGULARES):
        offset = lo + i * step
        angle = center_angle + offset
        target_y = by + tan(angle) * dx
        target_y = max(gy - half_width, min(gy + half_width, target_y))
        is_open = linha_livre(bx, by, gx, target_y, inimigos, FOLGA_CHUTE_MM)
        open_samples.append(is_open)

    best = None
    i = 0
    while i < len(open_samples):
        if not open_samples[i]:
            i += 1
            continue
        start = i
        while i + 1 < len(open_samples) and open_samples[i + 1]:
            i += 1
        end = i
        gap_lo = max(lo, lo + (start - 0.5) * step)
        gap_hi = min(hi, lo + (end + 0.5) * step)
        width = max(0.0, gap_hi - gap_lo)
        if best is None or width > best[0]:
            best = (width, gap_lo, gap_hi)
        i += 1

    if best is None:
        # LINHA FECHADA tambem e informacao: sem isto o diagnostico fica em
        # branco exatamente no caso que se quer entender.
        ULTIMO_VAO.clear()
        ULTIMO_VAO.update(bola=(round(bx, 1), round(by, 1)),
                          amostras=AMOSTRAS_ANGULARES, abertas=0,
                          folga_mm=FOLGA_CHUTE_MM, vaos=[], mira=None, nota=0.0,
                          corte_direto=NOTA_CHUTE_DIRETO,
                          corte_baixa=NOTA_CHUTE_BAIXA_CHANCE)
        return None, None, 0.0, None

    width, gap_lo, gap_hi = best
    target_angle = center_angle + (gap_lo + gap_hi) / 2.0
    target_y = by + tan(target_angle) * dx
    target_y = max(gy - half_width, min(gy + half_width, target_y))
    score = min(width / ANGULO_ABERTURA_NOTA_MAX, 1.0)

    # TODOS os vaos, nao so o vencedor: o descartado e metade da evidencia.
    vaos = []
    i = 0
    while i < len(open_samples):
        if not open_samples[i]:
            i += 1
            continue
        start = i
        while i + 1 < len(open_samples) and open_samples[i + 1]:
            i += 1
        g_lo = max(lo, lo + (start - 0.5) * step)
        g_hi = min(hi, lo + (i + 0.5) * step)
        a_meio = center_angle + (g_lo + g_hi) / 2.0
        y_meio = max(gy - half_width,
                     min(gy + half_width, by + tan(a_meio) * dx))
        # LIMITES EM Y, nao so o centro: sem eles nao da para responder "a
        # mira travada ainda cai dentro de um vao aberto?", que e o teste da
        # troca favoravel em analysis/alvo_chute.py.
        y_lo = max(gy - half_width,
                   min(gy + half_width, by + tan(center_angle + g_lo) * dx))
        y_hi = max(gy - half_width,
                   min(gy + half_width, by + tan(center_angle + g_hi) * dx))
        vaos.append(dict(y=round(y_meio, 1),
                         y_lo=round(min(y_lo, y_hi), 1),
                         y_hi=round(max(y_lo, y_hi), 1),
                         largura=round(max(0.0, g_hi - g_lo), 4),
                         nota=round(min(max(0.0, g_hi - g_lo)
                                        / ANGULO_ABERTURA_NOTA_MAX, 1.0), 3),
                         escolhido=abs((g_lo + g_hi) / 2.0
                                       - (gap_lo + gap_hi) / 2.0) < 1e-9))
        i += 1

    ULTIMO_VAO.clear()
    ULTIMO_VAO.update(
        bola=(round(bx, 1), round(by, 1)),
        amostras=AMOSTRAS_ANGULARES,
        abertas=sum(1 for o in open_samples if o),
        folga_mm=FOLGA_CHUTE_MM,
        vaos=vaos,
        mira=round(target_y, 1),
        nota=round(score, 3),
        corte_direto=NOTA_CHUTE_DIRETO,
        corte_baixa=NOTA_CHUTE_BAIXA_CHANCE,
    )
    return gx, target_y, score, (center_angle + gap_lo, center_angle + gap_hi)