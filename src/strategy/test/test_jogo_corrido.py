"""Open play: role assignment, the role registry and the carrier's phases, without ROS."""

from math import pi
from types import SimpleNamespace

import pytest

from strategy.analysis.jogo import MemoriaAnalise, analisar
from strategy.context import GOALKEEPER_ID, GameConfig, TickContext
from strategy.roles import portador as portador_mod
from strategy.roles.atribuicao import CICLOS_MIN_PAPEL, PAPEL_PORTADOR
from strategy.tatics.running import JogoCorrido


def robo(rid, x, y, th=0.0):
    return SimpleNamespace(id=rid, position_x=x, position_y=y, orientation=th,
                           velocity_x=0.0, velocity_y=0.0)


def contexto(aliados, bola=(0.0, 0.0), inimigos=(), positivo=False):
    return TickContext(
        ally_robots={r.id: r for r in aliados},
        enemy_robots={r.id: r for r in inimigos},
        balls=(SimpleNamespace(position_x=bola[0], position_y=bola[1],
                               velocity_x=0.0, velocity_y=0.0),),
        referee_command="FORCE_START",
        config=GameConfig(on_positive_half=positivo),
    )


@pytest.mark.parametrize("n_linha", [1, 2, 3, 4, 5])
def test_every_role_handed_out_has_an_implementation(n_linha):
    jogo = JogoCorrido()
    ctx = contexto([robo(GOALKEEPER_ID, -4000, 0)]
                   + [robo(i, -1000 + 500 * i, 200 * i) for i in range(1, n_linha + 1)])
    papeis = analisar(ctx, MemoriaAnalise()).papeis
    assert set(papeis.values()) <= set(jogo.papeis)
    assert list(papeis.values()).count(PAPEL_PORTADOR) == 1


def test_one_command_per_robot_goalkeeper_first():
    ctx = contexto([robo(0, -4000, 0), robo(1, -500, 0), robo(2, 500, 800), robo(3, -2000, -500)])
    comandos = JogoCorrido().comandos(ctx)
    assert [c.robot_id for c in comandos] == [0, 1, 2, 3]
    assert all(c.target_x is not None for c in comandos)


def test_the_carrier_keeps_its_role_through_a_small_advantage():
    jogo = JogoCorrido()
    r1, r2 = robo(1, -300, 0), robo(2, -900, 0)
    ctx = contexto([r1, r2, robo(3, -2500, 0)])
    jogo.comandos(ctx)
    assert jogo.memoria.analise.papeis.portador == 1

    # r2 becomes slightly closer: not worth a swap, so r1 keeps the ball.
    r2.position_x = -250
    for _ in range(CICLOS_MIN_PAPEL + 5):
        jogo.comandos(ctx)
    assert jogo.memoria.analise.papeis.portador == 1


def test_carrier_in_front_of_the_ball_circles_with_the_ball_as_obstacle():
    jogo = JogoCorrido()
    # Attacking +x; the carrier is between the ball and their goal.
    ctx = contexto([robo(1, 400, 0, th=pi), robo(2, -3000, 1500), robo(3, -3000, -1500)])
    comandos = {c.robot_id: c for c in jogo.comandos(ctx)}
    portador = jogo.papeis[PAPEL_PORTADOR]
    assert portador.memoria.fase == portador_mod.CONTORNAR
    assert comandos[1].ball is True
    assert comandos[1].kick == 0.0


def test_carrier_lined_up_behind_the_ball_pushes_through_it():
    jogo = JogoCorrido()
    ctx = contexto([robo(1, -150, 0, th=0.0), robo(2, -3000, 1500), robo(3, -3000, -1500)])
    comandos = {c.robot_id: c for c in jogo.comandos(ctx)}
    assert jogo.papeis[PAPEL_PORTADOR].memoria.fase == portador_mod.EMPURRAR
    assert comandos[1].ball is False
    assert comandos[1].target_x > 0.0
