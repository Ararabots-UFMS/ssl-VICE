"""Open play: role assignment, the role registry and the carrier's phases, without ROS."""

from math import atan2, cos, hypot, pi, sin
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


def test_push_target_stops_at_their_area_on_the_line_of_fire(monkeypatch):
    """Inside the area the planner moved it to the nearest edge, off the line."""
    from strategy import campo

    bola = (3250.0, -300.0)
    # Lined up 380 mm behind the ball: from there the push point is ~500 mm past it.
    ang = atan2(-bola[1], campo.GOL_X - bola[0])

    def alvo():
        jogo = JogoCorrido()
        ctx = contexto([robo(1, bola[0] - 380 * cos(ang), bola[1] - 380 * sin(ang), th=ang),
                        robo(2, -3000, 1500), robo(3, -3000, -1500)], bola=bola)
        cmd = {c.robot_id: c for c in jogo.comandos(ctx)}[1]
        assert jogo.papeis[PAPEL_PORTADOR].memoria.fase == portador_mod.EMPURRAR
        return cmd.target_x, cmd.target_y

    monkeypatch.setenv("ARARABOTS_SEM_RECORTE_AREA", "1")
    antes = alvo()
    monkeypatch.delenv("ARARABOTS_SEM_RECORTE_AREA")
    depois = alvo()

    borda = campo.GOL_X - campo.AREA_FUNDO - campo.MARGEM_AREA
    assert antes[0] > borda                       # the old target was in the area
    assert depois[0] == pytest.approx(borda)      # now it stops at its edge
    # ... on the same line from the ball.
    cruz = ((antes[0] - bola[0]) * (depois[1] - bola[1])
            - (antes[1] - bola[1]) * (depois[0] - bola[0]))
    assert abs(cruz) < 1e-6 * hypot(antes[0] - bola[0], antes[1] - bola[1]) ** 2



def _fase_do_portador(jogo, aliados, bola=(0.0, 0.0), inimigos=()):
    jogo.comandos(contexto(aliados, bola=bola, inimigos=inimigos))
    return jogo.papeis[PAPEL_PORTADOR].memoria.fase


def _outros():
    return [robo(2, -3000, 1500), robo(3, -3000, -1500)]


def test_lined_up_but_far_goes_to_the_waiting_point_first():
    """Pushing from far left the last stretch to the planner, and it arrived sideways."""
    assert _fase_do_portador(JogoCorrido(), [robo(1, -1000, 0)] + _outros()) == portador_mod.CONTORNAR


def test_close_and_on_the_line_pushes():
    assert _fase_do_portador(JogoCorrido(), [robo(1, -300, 20)] + _outros()) == portador_mod.EMPURRAR


def test_close_but_off_the_line_waits_then_gives_in():
    """60 mm off: outside the kicker's 40 mm, so it settles first, but never forever."""
    from strategy.roles.portador import CICLOS_MAX_ALINHANDO

    jogo = JogoCorrido()
    aliados = [robo(1, -300, 60)] + _outros()
    fases = [_fase_do_portador(jogo, aliados) for _ in range(CICLOS_MAX_ALINHANDO)]
    assert fases[0] == portador_mod.CONTORNAR
    assert fases[-1] == portador_mod.EMPURRAR


def test_drifting_off_the_line_during_the_push_stops_it():
    jogo = JogoCorrido()
    r = robo(1, -300, 0)
    assert _fase_do_portador(jogo, [r] + _outros()) == portador_mod.EMPURRAR
    r.position_y = 120.0                 # past the 90 mm exit, inside the old 160
    assert _fase_do_portador(jogo, [r] + _outros()) == portador_mod.CONTORNAR


def test_kick_tolerance_follows_the_gap_left_by_the_keeper():
    """A keeper narrowing the goal narrows how far off the aim the body may fire."""
    from strategy.analysis.alvo_chute import TOL_MIN_VAO
    from strategy.skills.chute import TOL_CORPO_DISPARO

    def tolerancia(inimigos):
        jogo = JogoCorrido()
        jogo.comandos(contexto([robo(1, 1000, 0)] + _outros(), bola=(1500.0, 0.0),
                               inimigos=inimigos))
        return jogo.memoria.analise.mira.tolerancia

    aberto = tolerancia(())
    com_goleiro = tolerancia([robo(0, 4400, 120)])
    assert TOL_MIN_VAO <= com_goleiro < aberto <= TOL_CORPO_DISPARO


def test_a_robot_turning_out_of_the_tolerance_does_not_arm(monkeypatch):
    """Armed now, the kick fires up to a tick later: by then this body is 5 deg off."""
    from strategy.skills import chute

    monkeypatch.setenv("ARARABOTS_DISPARO_PREVISTO", "1")

    bola = SimpleNamespace(position_x=0.0, position_y=0.0, velocity_x=0.0, velocity_y=0.0)
    alvo = (4500.0, 0.0)
    parado = robo(1, -100, 0, th=0.0)
    girando = robo(1, -100, 0, th=0.0)
    girando.velocity_orientation = 0.9          # rad/s: 5 deg in 100 ms
    tol = 0.035                                  # 2 deg
    assert chute.armar_chute(parado, bola, alvo, 1.0, "gol", {}, 1, tol_corpo=tol)
    assert not chute.armar_chute(girando, bola, alvo, 1.0, "gol", {}, 1, tol_corpo=tol)

