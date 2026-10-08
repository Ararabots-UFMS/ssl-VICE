from math import hypot, pi
from types import SimpleNamespace as S

import pytest

from strategy.skills import chute, posicionamento
from strategy.tatics import running


def robo(x, y=0.0, orientation=0.0):
    return S(position_x=x, position_y=y, orientation=orientation)


def abertura_coberta(bx, by, rx, ry, gol_x):
    """Fracao da largura real do gol ocultada pelo casco do zagueiro."""
    cobertos = 0
    for gy in range(-500, 501, 5):
        dx, dy = gol_x - bx, gy - by
        t = max(0.0, min(1.0, ((rx - bx) * dx + (ry - by) * dy)
                              / (dx * dx + dy * dy)))
        if hypot(bx + t * dx - rx, by + t * dy - ry) <= 90.0:
            cobertos += 1
    return cobertos / 201


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("bx,by", [(0, 0), (1500, 800), (2400, 1600)])
def test_cobertura_reduz_abertura_descoberta_do_gol(side, bx, by):
    gol = S(x=side * 4500.0, y=0.0)
    bx *= side
    x, y = posicionamento.cobertura_defensiva(bx, by, gol)
    antiga_x = bx + .45 * (gol.x - bx)
    antiga_y = by * .55
    assert abertura_coberta(bx, by, x, y, gol.x) > abertura_coberta(
        bx, by, antiga_x, antiga_y, gol.x)
    assert hypot(x - bx, y - by) >= posicionamento.DISTANCIA_SOMBRA_BOLA - 1e-6
    assert side * x >= 150


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("goalkeeper", [False, True])
@pytest.mark.parametrize("near_ball", [False, True])
def test_cobertura_so_posiciona_e_nunca_chuta(monkeypatch, side, goalkeeper, near_ball):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(side * 2600, 200)
    x = side * (2694 if near_ball else 3500)
    allies = {1: robo(x, 200, 0 if side < 0 else pi)}
    if goalkeeper:
        allies[0] = robo(side * 4300)
    goal = S(x=side * 4500.0, y=0.0)
    tt = S(ally_robots=allies, enemy_robots={}, ball=ball,
           on_positive_half=side > 0,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={"chute_armado": {1: True}})
    cmd = next(c for c in running.montar_comandos(tt) if c.robot_id == 1)
    expected = posicionamento.cobertura_defensiva(ball.position_x,
                                                   ball.position_y, goal)
    assert (cmd.target_x, cmd.target_y) == expected
    assert cmd.ball
    assert cmd.kick == 0
    assert 1 not in tt.estado["chute_armado"]


def test_kick_latch_survives_contact_noise_and_resets_when_ball_leaves():
    r = robo(0)
    ball = robo(240)
    state = {}
    assert chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)
    ball.position_x = 340
    assert chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)
    ball.position_x = 430
    assert not chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)
    assert not state[1]


def test_missing_target_clears_kick_latch():
    state = {1: True}
    assert not chute.armar_chute(robo(0), robo(94), None, 1, "saida", state, 1)
    assert not state[1]
