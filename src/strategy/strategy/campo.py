"""Field constants shared by the tactics. Division B: 9000 x 6000 mm."""

from utils.math_util import Vector2D


class CenterGoal:
    # Was 2250 (an SSL-EL half field) in three tactics, which sent the goalkeeper
    # to midfield and the attackers to an empty spot.
    GOAL_POSITIVE = Vector2D(4500.0, 0.0)
    GOAL_NEGATIVE = Vector2D(-4500.0, 0.0)


def nosso_gol(on_positive_half: bool) -> Vector2D:
    return CenterGoal.GOAL_POSITIVE if on_positive_half else CenterGoal.GOAL_NEGATIVE


def gol_ataque(on_positive_half: bool) -> Vector2D:
    return CenterGoal.GOAL_NEGATIVE if on_positive_half else CenterGoal.GOAL_POSITIVE


# Penalty areas: 1000 deep, 2000 wide. The planner keeps field robots 90 mm out of
# them and moves a target inside to the nearest edge, which is off the line of fire.
GOL_X = 4500.0
AREA_FUNDO = 1000.0
AREA_MEIA_LARGURA = 1000.0
# The planner's 90 mm, plus room so it never has to move our target.
MARGEM_AREA = 120.0


def _caixas_das_areas():
    for lado in (1.0, -1.0):
        x_borda = lado * (GOL_X - AREA_FUNDO - MARGEM_AREA)
        x_fundo = lado * (GOL_X + MARGEM_AREA)
        yield (min(x_borda, x_fundo), max(x_borda, x_fundo),
               -AREA_MEIA_LARGURA - MARGEM_AREA, AREA_MEIA_LARGURA + MARGEM_AREA)


def recortar_antes_das_areas(ox, oy, tx, ty):
    """The target, or where the segment origin->target first enters a penalty area.

    Keeps the point on the same line, so a push through the ball stays aimed. An
    origin already inside an area is left alone: nothing legal is left to cut.
    """
    dx, dy = tx - ox, ty - oy
    entrada = 1.0
    for x0, x1, y0, y1 in _caixas_das_areas():
        if x0 < ox < x1 and y0 < oy < y1:
            continue
        # Liang-Barsky: the parameter where the segment enters the box, if it does.
        t_in, t_out = 0.0, 1.0
        for p, q in ((-dx, ox - x0), (dx, x1 - ox), (-dy, oy - y0), (dy, y1 - oy)):
            if p == 0.0:
                if q < 0.0:
                    t_in, t_out = 1.0, 0.0
                continue
            t = q / p
            if p < 0.0:
                t_in = max(t_in, t)
            else:
                t_out = min(t_out, t)
        if t_in < t_out and t_in < entrada:
            entrada = t_in
    return ox + dx * entrada, oy + dy * entrada
