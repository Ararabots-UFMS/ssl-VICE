"""The open-play view of one tick: who is where, which goal is whose, who has the ball."""

from dataclasses import dataclass
from math import hypot
from typing import Any, List, Mapping

from utils.math_util import Vector2D

from strategy import campo
from strategy.context import GOALKEEPER_ID, TickContext

# Physical contact is at 111 mm; 250 covers "has the ball".
RAIO_POSSE = 250.0

SITUACAO_NOSSA = "NOSSA"
SITUACAO_DELES = "DELES"
SITUACAO_DISPUTA = "DISPUTA"
SITUACAO_SOLTA = "SOLTA"


def dist(robo, ball) -> float:
    return hypot(robo.position_x - ball.position_x, robo.position_y - ball.position_y)


def situacao_de_jogo(ally_robots, enemy_robots, ball) -> str:
    """Who has the ball right now. Geometry only, no memory.

    Measured over 8802 frames: DISPUTA 51%, NOSSA 33%, DELES 8%, SOLTA 8%.
    """
    dn = min((dist(r, ball) for rid, r in ally_robots.items() if rid != GOALKEEPER_ID),
             default=9e9)
    dd = min((dist(r, ball) for r in enemy_robots.values()),
             default=9e9) if enemy_robots else 9e9
    if dn <= RAIO_POSSE and dd <= RAIO_POSSE:
        return SITUACAO_DISPUTA
    if dn <= RAIO_POSSE:
        return SITUACAO_NOSSA
    if dd <= RAIO_POSSE:
        return SITUACAO_DELES
    return SITUACAO_SOLTA


@dataclass(frozen=True)
class Mundo:
    aliados: Mapping[int, Any]
    inimigos: Mapping[int, Any]
    bola: Any
    on_positive_half: bool
    nosso_gol: Vector2D
    gol_ataque: Vector2D
    situacao: str

    @property
    def sentido(self) -> float:
        """+1 when we attack towards +x, -1 towards -x."""
        return -1.0 if self.on_positive_half else 1.0

    @property
    def linha(self) -> List[int]:
        """Field players (everyone but the goalkeeper), by id."""
        return sorted(rid for rid in self.aliados if rid != GOALKEEPER_ID)


def analisar_mundo(context: TickContext) -> Mundo:
    lado = context.on_positive_half
    return Mundo(
        aliados=context.ally_robots,
        inimigos=context.enemy_robots,
        bola=context.ball,
        on_positive_half=lado,
        nosso_gol=campo.nosso_gol(lado),
        gol_ataque=campo.gol_ataque(lado),
        situacao=situacao_de_jogo(context.ally_robots, context.enemy_robots, context.ball),
    )
