"""Avaliacao geometrica de chute ao gol, sem dependencias de ROS."""

from math import atan2, pi, tan

from strategy.skills.geometria import linha_livre


LARGURA_GOL_MM = 1000.0
FOLGA_CHUTE_MM = 180.0
AMOSTRAS_ANGULARES = 81
ANGULO_ABERTURA_NOTA_MAX = 0.3
NOTA_CHUTE_DIRETO = 0.3
NOTA_CHUTE_BAIXA_CHANCE = 0.1


def _angulo_relativo(angulo, referencia):
    return (angulo - referencia + pi) % (2.0 * pi) - pi


def melhor_alvo(ball, gol, inimigos):
    """Devolve (x, y, nota) do maior vao livre da boca do gol.

    A nota e uma heuristica angular em [0, 1], nao uma probabilidade calibrada
    de gol. Cada raio amostrado e validado pela mesma margem usada pelo VICE
    para verificar linhas de chute e passe.
    """
    bx, by = float(ball.position_x), float(ball.position_y)
    gx, gy = float(gol.x), float(gol.y)
    dx = gx - bx
    if abs(dx) < 1e-6:
        return None, None, 0.0

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
        return None, None, 0.0

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
        return None, None, 0.0

    width, gap_lo, gap_hi = best
    target_angle = center_angle + (gap_lo + gap_hi) / 2.0
    target_y = by + tan(target_angle) * dx
    target_y = max(gy - half_width, min(gy + half_width, target_y))
    score = min(width / ANGULO_ABERTURA_NOTA_MAX, 1.0)
    return gx, target_y, score