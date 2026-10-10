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
def test_zagueiro_busca_a_bola_e_mantem_chute_disponivel(monkeypatch, side, goalkeeper, near_ball):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(side * 2600, 200)
    x = side * (2694 if near_ball else 3500)
    allies = {1: robo(x, 200, 0 if side < 0 else pi)}
    if goalkeeper:
        allies[0] = robo(side * 4300)
    tt = S(ally_robots=allies, enemy_robots={}, ball=ball,
           on_positive_half=side > 0,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={"chute_armado": {}})
    cmd = next(c for c in running.montar_comandos(tt) if c.robot_id == 1)
    assert not cmd.ball
    assert cmd.defensive_half == side
    assert side * cmd.target_x < side * x
    if near_ball:
        assert abs(cmd.target_x - ball.position_x) < 300.0
        assert cmd.kick > 0
        assert tt.estado["chute_armado"][1]
    else:
        assert abs(cmd.target_x - ball.position_x) < 1700.0
        assert cmd.kick == 0


@pytest.mark.parametrize("side", [-1, 1])
def test_cobertura_nao_chuta_para_propria_meta(monkeypatch, side):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(side * 2600, 200)
    # O robo esta do lado do ataque: a bola fica atras dele.
    allies = {1: robo(side * 2500, 200, 0 if side < 0 else pi)}
    tt = S(ally_robots=allies, enemy_robots={}, ball=ball,
           on_positive_half=side > 0,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})
    cmd = running.montar_comandos(tt)[0]
    assert cmd.kick == 0
    assert not cmd.ball


def test_zagueiro_armado_mesmo_sem_linha_livre(monkeypatch):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    monkeypatch.setattr(running, "alvo_do_chute", lambda *args: (None, None, "bloqueado"))
    ball = robo(-2600.0)
    allies = {1: robo(-2800.0)}
    tt = S(ally_robots=allies, enemy_robots={}, ball=ball,
           on_positive_half=False,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    cmd = running.montar_comandos(tt)[0]
    assert cmd.target_x > ball.position_x
    assert cmd.kick > 0
    assert not cmd.ball


def test_zagueiro_entra_na_sombra_antes_de_aproximar(monkeypatch):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(-2600.0, 200.0)
    r = robo(-3500.0, 800.0)
    goal = S(x=-4500.0, y=0.0)
    tt = S(ally_robots={1: r}, enemy_robots={}, ball=ball,
           on_positive_half=False,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=goal),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    entrando = running.montar_comandos(tt)[0]
    sombra = posicionamento.cobertura_defensiva(
        ball.position_x, ball.position_y, goal, robo=r)
    assert (entrando.target_x, entrando.target_y) == sombra
    assert entrando.kick == 0
    assert tt.estado["zagueiro_na_sombra"][1] is False

    r.position_x, r.position_y = sombra
    aproximando = running.montar_comandos(tt)[0]
    assert tt.estado["zagueiro_na_sombra"][1] is True
    assert aproximando.target_x > entrando.target_x
    assert aproximando.defensive_half == -1
    assert not aproximando.ball


def test_zagueiro_contorna_bola_quando_esta_a_frente(monkeypatch):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(-2600.0, 200.0)
    tt = S(ally_robots={1: robo(-2500.0, 200.0)}, enemy_robots={}, ball=ball,
           on_positive_half=False,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    cmd = running.montar_comandos(tt)[0]
    assert tt.estado["zagueiro_na_sombra"][1] is False
    assert abs(cmd.target_y - ball.position_y) > 100.0
    assert cmd.kick == 0
    assert cmd.defensive_half == -1


@pytest.mark.parametrize("side", [-1, 1])
def test_aproximacao_do_zagueiro_permanece_no_eixo_da_sombra(monkeypatch, side):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(side * 2600.0, 700.0)
    goal = S(x=side * 4500.0, y=0.0)
    sx, sy = posicionamento.cobertura_defensiva(
        ball.position_x, ball.position_y, goal)
    r = robo(sx, sy)
    tt = S(ally_robots={1: r}, enemy_robots={}, ball=ball,
           on_positive_half=side > 0,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    cmd = running.montar_comandos(tt)[0]
    ux, uy, _ = posicionamento.versor(ball.position_x, ball.position_y,
                                     goal.x, goal.y)
    assert tt.estado["zagueiro_na_sombra"][1]
    assert abs((cmd.target_x - ball.position_x) * uy
               - (cmd.target_y - ball.position_y) * ux) < 1e-6
    assert (cmd.target_x - ball.position_x) * ux < 0
    assert cmd.defensive_half == side


def test_sombra_tolera_pequeno_deslocamento_da_bola(monkeypatch):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(-2600.0, 700.0)
    goal = S(x=-4500.0, y=0.0)
    sx, sy = posicionamento.cobertura_defensiva(
        ball.position_x, ball.position_y, goal)
    r = robo(sx, sy)
    tt = S(ally_robots={1: r}, enemy_robots={}, ball=ball,
           on_positive_half=False,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=goal),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    running.montar_comandos(tt)
    ball.position_y += 150.0
    assert not posicionamento.na_sombra_da_bola(
        r, ball.position_x, ball.position_y, goal)
    cmd = running.montar_comandos(tt)[0]
    assert tt.estado["zagueiro_na_sombra"][1]
    ux, uy, _ = posicionamento.versor(ball.position_x, ball.position_y,
                                     goal.x, goal.y)
    assert abs((cmd.target_x - ball.position_x) * uy
               - (cmd.target_y - ball.position_y) * ux) < 1e-6


def test_apenas_zagueiro_mais_proximo_busca_bola(monkeypatch):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(-2600.0)
    allies = {1: robo(-3500.0), 2: robo(-2900.0)}
    tt = S(ally_robots=allies, enemy_robots={}, ball=ball,
           on_positive_half=False,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    cmds = {c.robot_id: c for c in running.montar_comandos(tt)}
    assert cmds[2].target_x > ball.position_x
    assert (cmds[1].target_x, cmds[1].target_y) == posicionamento.cobertura_defensiva(
        ball.position_x, ball.position_y, tt.goal_center.GOAL_NEGATIVE,
        robo=allies[1])
    assert cmds[1].kick == 0
    assert not cmds[1].ball and not cmds[2].ball
    assert cmds[1].defensive_half == -1
    assert cmds[2].defensive_half == -1


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("distancia_bola", [0.0, 2000.0])
def test_zagueiro_respeita_limite_da_metade_defensiva(monkeypatch, side, distancia_bola):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(-side * distancia_bola, 700.0)
    goal = S(x=side * 4500.0, y=0.0)
    robo_x = side * 1000.0
    fracao = (robo_x - ball.position_x) / (goal.x - ball.position_x)
    allies = {1: robo(robo_x, ball.position_y * (1 - fracao))}
    tt = S(ally_robots=allies, enemy_robots={}, ball=ball,
           on_positive_half=side > 0,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    cmd = running.montar_comandos(tt)[0]
    assert cmd.defensive_half == side
    assert side * cmd.target_x >= 100.0
    if distancia_bola == 0.0:
        assert side * cmd.target_x == 100.0
    esperado_y = ball.position_y * (goal.x - cmd.target_x) / (goal.x - ball.position_x)
    assert cmd.target_y == pytest.approx(esperado_y)
    assert not cmd.ball


def test_cobertura_arma_chute_na_posicao_travada_do_replay(monkeypatch):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    ball = robo(-3200.0, 200.0)
    # Ultimo quadro de zg_perto_gol__original: 224 mm da bola, orientado
    # para frente, mas o comando antigo anulava kick por ser cobertura.
    r = robo(-3414.3069, 133.9566, 0.29894)
    tt = S(ally_robots={1: r}, enemy_robots={}, ball=ball,
           on_positive_half=False,
           goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                         GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
           skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
           estado={})

    cmd = running.montar_comandos(tt)[0]

    assert cmd.kick > 0
    assert not cmd.ball
    assert cmd.target_x > r.position_x


@pytest.mark.parametrize("side", [-1, 1])
def test_cobertura_entra_no_centro_antes_de_avancar(side):
    gol = S(x=side * 4500.0, y=0.0)
    bx, by = side * 1000.0, 0.0
    r = robo(side * 2500.0, 600.0)
    x, y = posicionamento.cobertura_defensiva(bx, by, gol, robo=r)
    assert x == pytest.approx(r.position_x)
    assert y == pytest.approx(0.0)
    assert (x, y) != posicionamento.cobertura_defensiva(bx, by, gol)

    # Uma vez no meio da largura, pode avancar para fechar a sombra.
    r.position_y = 0.0
    assert posicionamento.cobertura_defensiva(bx, by, gol, robo=r) == (
        posicionamento.cobertura_defensiva(bx, by, gol))


@pytest.mark.parametrize("side", [-1, 1])
def test_cobertura_respeita_area_ao_entrar_na_sombra(side):
    gol = S(x=side * 4500.0, y=0.0)
    r = robo(side * 4000.0, 500.0)
    x, y = posicionamento.cobertura_defensiva(side * 500.0, 0.0, gol, robo=r)
    assert side * x == pytest.approx(3350.0)
    assert y == pytest.approx(0.0)


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
