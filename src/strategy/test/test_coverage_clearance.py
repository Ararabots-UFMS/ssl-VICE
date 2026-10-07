from math import atan2, cos, hypot, pi, sin
from types import SimpleNamespace as S

import pytest

from strategy.skills import aproximacao, chute, geometria
from strategy.tatics import running


def robot(x, y=0.0, orientation=0.0):
    return S(position_x=x, position_y=y, orientation=orientation)


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_passes_to_nearest_teammate_regardless_of_role(side):
    ball = robot(side * 2600)
    allies = {1: robot(side * 2700), 2: robot(side * 2000, 400),
              3: robot(side * 1000)}
    goal = S(x=side * 4500, y=0.0)
    assert running.alvo_da_cobertura(1, ball, allies, {}, goal) == (
        side * 2000, 400, "passe")
    allies[0] = robot(side * 2800)
    assert running.alvo_da_cobertura(1, ball, allies, {}, goal) == (
        side * 2800, 0.0, "passe")


@pytest.mark.parametrize("side", [-1, 1])
def test_lone_coverage_clears_away_from_goal_and_avoids_enemy(side):
    ball = robot(side * 2600)
    goal = S(x=side * 4500, y=0.0)
    enemies = {1: robot(ball.position_x - side * 800)}
    x, y, kind = running.alvo_da_cobertura(
        1, ball, {1: robot(side * 2700)}, enemies, goal)
    assert kind == "saida"
    assert side * (x - ball.position_x) < 0
    assert hypot(x - goal.x, y - goal.y) > abs(ball.position_x - goal.x)
    assert geometria.linha_livre(ball.position_x, ball.position_y, x, y, enemies)
    assert geometria.livre_do_lado(x, y, enemies)


@pytest.mark.parametrize("side", [-1, 1])
def test_blocked_nearest_pass_uses_free_clearance(side):
    ball = robot(side * 2600)
    goal = S(x=side * 4500, y=0.0)
    allies = {1: robot(side * 2700), 2: robot(side * 1800)}
    enemies = {1: robot(side * 2200)}
    x, y, kind = running.alvo_da_cobertura(1, ball, allies, enemies, goal)
    assert kind == "saida"
    assert geometria.linha_livre(ball.position_x, ball.position_y, x, y, enemies)


@pytest.mark.parametrize("side", [-1, 1])
def test_surrounded_coverage_uses_best_clearance_instead_of_waiting(side):
    ball = robot(side * 2600)
    enemies = {i: robot(ball.position_x - side * 500 * cos(i * pi / 12),
                        500 * sin(i * pi / 12)) for i in range(-5, 6)}
    x, y, kind = running.alvo_da_cobertura(
        1, ball, {1: robot(side * 2700)}, enemies,
        S(x=side * 4500, y=0.0))
    assert kind == "saida"
    assert side * (x - ball.position_x) < 0
    dx, dy = x - ball.position_x, y - ball.position_y
    length = hypot(dx, dy)
    chosen_clearance = geometria.folga_lateral(
        ball.position_x, ball.position_y, dx / length, dy / length, length, enemies)
    for degrees in range(-75, 76, 15):
        ux, uy = -side * cos(degrees * pi / 180), sin(degrees * pi / 180)
        assert chosen_clearance + 1e-6 >= geometria.folga_lateral(
            ball.position_x, ball.position_y, ux, uy, 2200, enemies)


def tactic(monkeypatch, side, allies, enemies=None):
    monkeypatch.setenv("ARARABOTS_FORCAR_COBERTURA", "1")
    monkeypatch.setattr(running, "_gravar_papeis", lambda _: None)
    return S(ally_robots=allies, enemy_robots=enemies or {},
             ball=robot(side * 2600), on_positive_half=side > 0,
             goal_center=S(GOAL_POSITIVE=S(x=4500.0, y=0.0),
                           GOAL_NEGATIVE=S(x=-4500.0, y=0.0)),
             skills_factory=S(move_with_angle=lambda **kwargs: S(**kwargs)),
             estado={"mira_tipo": "gol", "mira_ponto": (-side * 4500, 0),
                     "mira_ciclos": 0})


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("pass_to_teammate", [False, True])
def test_coverage_command_uses_its_own_target_and_kick_strength(
        monkeypatch, side, pass_to_teammate):
    angle = 0.0 if side < 0 else pi
    allies = {1: robot(side * 2694, orientation=angle)}
    if pass_to_teammate:
        allies[2] = robot(side * 2100)
    tt = tactic(monkeypatch, side, allies)
    cmd = running.montar_comandos(tt)[0]
    assert cmd.kick == (chute.FORCA_PASSE if pass_to_teammate else chute.FORCA_SAIDA)
    assert not cmd.ball
    assert cmd.defensive_half == side
    assert side * cmd.target_x >= 150


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_waits_for_alignment_before_passing(monkeypatch, side):
    angle = 0.0 if side < 0 else pi
    allies = {1: robot(side * 2694, orientation=angle),
              2: robot(side * 2600, 500)}
    tt = tactic(monkeypatch, side, allies)
    assert running.montar_comandos(tt)[0].kick == 0
    # A bola na placa e o corpo alinhado ao receptor permitem o passe lateral.
    direction = atan2(500, 0)
    allies[1] = robot(tt.ball.position_x - 94 * cos(direction),
                      -94 * sin(direction), direction)
    assert running.montar_comandos(tt)[0].kick == chute.FORCA_PASSE


@pytest.mark.parametrize("side", [-1, 1])
def test_distant_ball_keeps_coverage_position_without_kicking(monkeypatch, side):
    tt = tactic(monkeypatch, side, {1: robot(side * 3500)},
                {1: robot(-side * 1000)})
    cmd = running.montar_comandos(tt)[0]
    assert cmd.kick == 0
    assert cmd.ball
    assert side * cmd.target_x >= 150


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("with_goalkeeper", [False, True])
def test_lone_coverage_fetches_free_ball_from_a_distance(monkeypatch, side,
                                                           with_goalkeeper):
    allies = {1: robot(side * 3200, -200)}
    if with_goalkeeper:
        allies[0] = robot(side * 4300)
    tt = tactic(monkeypatch, side, allies)
    tt.ball = robot(side * 2400, 1600)
    cmd = next(command for command in running.montar_comandos(tt)
               if command.robot_id == 1)
    assert hypot(cmd.target_x - tt.ball.position_x,
                 cmd.target_y - tt.ball.position_y) < 450
    assert not cmd.ball
    assert tt.estado["miras_cobertura"][1][2] == "saida"


@pytest.mark.parametrize("side", [-1, 1])
def test_nearby_ball_in_attacking_half_does_not_pull_coverage_forward(side):
    ball = robot(-side * 100)
    x, y, engaging = running.alvo_do_papel(
        running.PAPEL_COBERTURA, running.SITUACAO_NOSSA, 1,
        {1: robot(side * 150)}, ball, S(x=-side * 4500, y=0),
        S(x=side * 4500, y=0), alvo_chute=(-side * 1500, 0))
    assert side * x >= 150
    assert not engaging


@pytest.mark.parametrize("side", [-1, 1])
def test_normal_game_coverage_passes_without_changing_carrier_target(monkeypatch, side):
    angle = 0.0 if side < 0 else pi
    allies = {1: robot(side * 2800, orientation=angle),
              2: robot(side * 2500, orientation=angle + pi)}
    tt = tactic(monkeypatch, side, allies)
    monkeypatch.delenv("ARARABOTS_FORCAR_COBERTURA")
    monkeypatch.delenv("ARARABOTS_PAPEIS_FIXOS", raising=False)
    monkeypatch.setattr(running.os.path, "exists", lambda _: False)
    roles = {}
    monkeypatch.setattr(running, "_gravar_papeis", roles.update)
    cmds = running.montar_comandos(tt)
    assert roles == {1: running.PAPEL_COBERTURA, 2: running.PAPEL_PORTADOR}
    assert cmds[0].kick == chute.FORCA_PASSE
    assert not cmds[0].ball
    assert abs(geometria.norm_ang(cmds[1].angle - angle)) < 1e-6


@pytest.mark.parametrize("side", [-1, 1])
def test_enemy_in_body_direction_disarms_a_previously_armed_coverage(monkeypatch, side):
    angle = 0.0 if side < 0 else pi
    tt = tactic(monkeypatch, side, {1: robot(side * 2694, orientation=angle)})
    assert running.montar_comandos(tt)[0].kick == chute.FORCA_SAIDA
    tt.enemy_robots = {1: robot(tt.ball.position_x - side * 400)}
    assert running.montar_comandos(tt)[0].kick == 0
    assert not tt.estado["chute_armado"][1]


@pytest.mark.parametrize("side", [-1, 1])
def test_surrounded_coverage_kicks_through_the_best_available_corridor(monkeypatch, side):
    enemies = {i: robot(side * 2600 - side * 500 * cos(i * pi / 12),
                        500 * sin(i * pi / 12)) for i in range(-5, 6)}
    tt = tactic(monkeypatch, side, {1: robot(side * 2700)}, enemies)
    x, y, _ = running.alvo_da_cobertura(
        1, tt.ball, tt.ally_robots, enemies, S(x=side * 4500, y=0))
    angle = atan2(y, x - tt.ball.position_x)
    tt.ally_robots[1] = robot(tt.ball.position_x - 94 * cos(angle),
                              -94 * sin(angle), angle)
    assert running.montar_comandos(tt)[0].kick == chute.FORCA_SAIDA


@pytest.mark.parametrize("side", [-1, 1])
def test_recorded_contact_arms_clearance_without_exact_aim(monkeypatch, side):
    # First open kicker window in zv_mata: the old aim gate missed this contact.
    angle = -0.6684615 if side < 0 else pi + 0.6684615
    r = robot(side * 3259.4731, 292.5497, angle)
    tt = tactic(monkeypatch, side, {1: r}, {
        1: robot(-side * 429.9513, 964.4178),
        2: robot(-side * 506.8017, -584.9536),
        3: robot(side * 52.4488, 1321.5118)})
    tt.ball = robot(side * 3200, 200)
    xx, yy, _ = chute.geometria_do_chutador(r, tt.ball)
    assert chute.na_janela_de_disparo(xx, yy)
    cmd = running.montar_comandos(tt)[0]
    assert abs(geometria.norm_ang(r.orientation - cmd.angle)) > running.TOL_MIRA
    assert cmd.kick == chute.FORCA_SAIDA


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_does_not_clear_toward_own_goal(monkeypatch, side):
    angle = pi if side < 0 else 0
    tt = tactic(monkeypatch, side, {1: robot(side * 2506, orientation=angle)})
    assert running.montar_comandos(tt)[0].kick == 0


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_keeps_engaging_while_contouring_then_releases(monkeypatch, side):
    tt = tactic(monkeypatch, side, {1: robot(side * 2800)})
    assert not running.montar_comandos(tt)[0].ball
    tt.ally_robots[1] = robot(side * 3300, 200)
    assert not running.montar_comandos(tt)[0].ball
    tt.ball.position_x = -side * 100  # ball leaves the defensive half
    cmd = running.montar_comandos(tt)[0]
    assert cmd.ball
    assert cmd.kick == 0
    assert not tt.estado["miras_cobertura"]


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("pass_to_teammate", [False, True])
def test_coverage_holds_aim_during_small_ball_or_receiver_movements(side, pass_to_teammate):
    ball = robot(side * 2600)
    allies = {1: robot(side * 2700)}
    if pass_to_teammate:
        allies[2] = robot(side * 2000, 400)
    goal = S(x=side * 4500, y=0)
    state = {}
    first = running.alvo_da_cobertura(1, ball, allies, {}, goal, state)
    ball.position_y -= 1
    if pass_to_teammate:
        allies[2].position_y += 20
    assert running.alvo_da_cobertura(1, ball, allies, {}, goal, state) == first
    if pass_to_teammate:
        allies[2].position_y += 400
        assert running.alvo_da_cobertura(1, ball, allies, {}, goal, state) != first


def test_kick_latch_survives_contact_noise_and_resets_when_ball_leaves():
    r = robot(0)
    ball = robot(240)
    state = {}
    assert chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)
    ball.position_x = 340  # outside initial range, still inside the latch range
    assert chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)
    ball.position_x = 430
    assert not chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)
    assert not state[1]
    ball.position_x = 340
    assert not chute.armar_chute(r, ball, (2000, 0), 1, "saida", state, 1)


def test_missing_target_clears_kick_latch():
    state = {1: True}
    assert not chute.armar_chute(robot(0), robot(94), None, 1, "saida", state, 1)
    assert not state[1]


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_contours_without_pushing_ball_from_wrong_side(side):
    ball = robot(side * 2600)
    r = robot(ball.position_x - side * 160, 136, orientation=pi)
    target = (ball.position_x - side * 1000, 500)
    x, y = aproximacao.ponto_de_aproximacao_segura(r, ball.position_x, 0, target)
    # The movement segment must stay outside the combined robot/ball radius.
    dx, dy = x - r.position_x, y - r.position_y
    fraction = max(0, min(1, ((ball.position_x - r.position_x) * dx
                             - r.position_y * dy) / (dx * dx + dy * dy)))
    assert hypot(r.position_x + fraction * dx - ball.position_x,
                 r.position_y + fraction * dy) > 111.5


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_rotates_before_advancing_to_pass(side):
    angle = 0 if side < 0 else pi
    r = robot(side * 2820, orientation=angle + pi / 2)
    target = (side * 1800, 0)
    assert aproximacao.ponto_de_aproximacao_segura(r, side * 2600, 0, target) == (
        r.position_x, r.position_y)
    r.orientation = angle
    x, y = aproximacao.ponto_de_aproximacao_segura(r, side * 2600, 0, target)
    assert (x - r.position_x) * cos(angle) + (y - r.position_y) * sin(angle) > 0


@pytest.mark.parametrize("side", [-1, 1])
def test_coverage_clearance_contours_before_touching_ball(side):
    ball = robot(side * 3200, 200)
    r = robot(side * 3000, 600)
    goal = S(x=side * 4500, y=0)
    target = chute.alvo_de_afastamento(ball, goal, {})
    x, y, engaging = running.alvo_do_papel(
        running.PAPEL_COBERTURA, running.SITUACAO_SOLTA, 1,
        {1: r}, ball, S(x=-side * 4500, y=0), goal,
        alvo_chute=target, tipo_chute="saida")
    assert engaging
    assert (x, y) == aproximacao.ponto_de_aproximacao_segura(
        r, ball.position_x, ball.position_y, target)
