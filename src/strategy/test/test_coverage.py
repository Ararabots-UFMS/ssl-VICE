from types import SimpleNamespace

import pytest

from strategy.skills.posicionamento import cobertura_defensiva
from strategy.tatics.running import (
    PAPEL_COBERTURA, PAPEL_PORTADOR, SITUACAO_SOLTA, SITUACAO_DISPUTA,
    SITUACAO_DELES, SITUACAO_NOSSA, alvo_do_papel, distribuir_papeis,
)


@pytest.mark.parametrize("forced,expected", [("1", PAPEL_COBERTURA),
                                             ("0", PAPEL_PORTADOR),
                                             ("", PAPEL_PORTADOR)])
@pytest.mark.parametrize("situation", [SITUACAO_SOLTA, SITUACAO_DISPUTA,
                                       SITUACAO_DELES, SITUACAO_NOSSA])
def test_lone_defender_role_in_test_menu(monkeypatch, forced, expected, situation):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", forced)
    monkeypatch.delenv("ARARABOTS_PAPEIS_FIXOS", raising=False)
    monkeypatch.setattr("strategy.tatics.running.os.path.exists", lambda _: False)
    ball = SimpleNamespace(position_x=-2600.0, position_y=0.0)
    robots = {1: SimpleNamespace(position_x=-3000.0, position_y=600.0)}
    assert distribuir_papeis(robots, ball, situation) == {1: expected}


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("ball_x,ball_y", [(-4400, 2200), (0, 0), (2000, 800)])
def test_coverage_stays_inside_shot_cone_and_in_own_half(side, ball_x, ball_y):
    goal = SimpleNamespace(x=side * 4500.0, y=0.0)
    bx = side * ball_x
    x, y = cobertura_defensiva(bx, ball_y, goal)
    assert side * x >= 150.0 - 1e-6
    fraction = (x - bx) / (goal.x - bx)
    assert 0 <= fraction <= 1
    assert ball_y + (-500 - ball_y) * fraction <= y
    assert y <= ball_y + (500 - ball_y) * fraction


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("situation", [SITUACAO_SOLTA, SITUACAO_DISPUTA,
                                       SITUACAO_DELES, SITUACAO_NOSSA])
def test_coverage_does_not_chase_ball_in_attacking_half(side, situation):
    goal = SimpleNamespace(x=side * 4500.0, y=0.0)
    attack = SimpleNamespace(x=-goal.x, y=0.0)
    ball = SimpleNamespace(position_x=-side * 4400.0, position_y=0.0)
    robots = {2: SimpleNamespace(position_x=side * 1000.0, position_y=0.0)}
    x, y, kick = alvo_do_papel(PAPEL_COBERTURA, situation, 2, robots,
                               ball, attack, goal)
    assert side * x >= 150.0 - 1e-6
    assert y == 0
    assert not kick


@pytest.mark.parametrize("side", [-1, 1])
def test_ball_inside_penalty_area_keeps_coverage_outside(side):
    goal = SimpleNamespace(x=side * 4500.0, y=0.0)
    x, y = cobertura_defensiva(side * 4400.0, 0.0, goal)
    assert side * x <= 3400.0 or abs(y) >= 1100.0
