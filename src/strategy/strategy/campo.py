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
