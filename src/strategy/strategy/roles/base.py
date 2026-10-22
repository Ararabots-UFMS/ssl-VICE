"""What every open-play role is, and the one pipeline that turns it into a command.

A role answers "where do I go?" (``alvo``). Orientation, the kick gate and the
obstacle flags are the same for every role and live in ``comando``, so two roles
can never disagree about how a kick is armed. Override the small hooks only.
"""

from abc import ABC, abstractmethod
from math import hypot
from typing import Dict, Optional, Tuple

from strategy import diagnostico
from strategy.analysis.jogo import AnaliseJogo
from strategy.context import GOALKEEPER_ID
from strategy.skills import chute, experimento
from strategy.skills.skills import Skill, Skills

# Diagnostics only: who is actually on the ball.
RAIO_POSSE_PORTADOR = 200.0

_SKILLS = Skills("Movement")


def versor_ataque(a: AnaliseJogo) -> Tuple[float, float, float, float]:
    """Unit vector ball -> kick target (or their goal) and its perpendicular."""
    bx, by = a.bola.position_x, a.bola.position_y
    if a.alvo_chute is not None:
        dx, dy = a.alvo_chute[0] - bx, a.alvo_chute[1] - by
    else:
        dx, dy = a.mundo.gol_ataque.x - bx, a.mundo.gol_ataque.y - by
    n = hypot(dx, dy) or 1.0
    ux, uy = dx / n, dy / n
    return ux, uy, -uy, ux


class Papel(ABC):
    """One open-play role. Instances live as long as the play, so they may keep memory."""

    nome: str = ""

    @abstractmethod
    def alvo(self, a: AnaliseJogo, rid: int, ordem: int) -> Tuple[float, float]:
        """Where robot ``rid`` goes. ``ordem`` counts robots already in this role."""

    def fase(self) -> Optional[str]:
        """Phase handed to the kick gate (chute.chutar_em). None for most roles."""
        return None

    def velocidade_chegada(self, a: AnaliseJogo, rid: int, x: float, y: float):
        """Velocity to arrive with, in mm/s. Zero means stop on the target."""
        return 0.0, 0.0

    def evita_bola(self, a: AnaliseJogo, rid: int) -> bool:
        """Whether the planner routes around the ball for this robot."""
        return True

    def diagnosticar(self, a: AnaliseJogo, rid: int, cmd: Skill) -> None:
        """Extra DIAG_JOGO output for this role. Runs only with diagnostics on."""

    def comando(self, a: AnaliseJogo, rid: int, ordem: int,
                travas: Dict[int, object]) -> Skill:
        robo, ball = a.mundo.aliados[rid], a.bola
        alvo_x, alvo_y = self.alvo(a, rid, ordem)

        # Body orientation and kick in one call; see chute.chutar_em for why the
        # movement target must never be reachable.
        gol = a.mundo.gol_ataque
        ang, arma, forca = chute.chutar_em(
            robo, ball, a.alvo_chute, a.tipo_alvo, a.mundo.sentido, travas, rid,
            (gol.x, gol.y),
            so_lado_certo=not experimento.desligado("ORIENTACAO_LADO"),
            pode_armar=not a.alvo.emprestada,
            exigir_corpo_alinhado=not experimento.desligado("DISPARO_ALINHADO"),
            fase=self.fase(), tol_corpo=a.alvo.tolerancia)

        vel_x, vel_y = self.velocidade_chegada(a, rid, alvo_x, alvo_y)
        cmd = _SKILLS.move_with_angle(robot_id=rid, target_x=alvo_x, target_y=alvo_y,
                                      vel_x=vel_x, vel_y=vel_y, angle=ang)
        cmd.kick = forca                 # chutar_em already returns 0 when not armed
        cmd.ball = self.evita_bola(a, rid)
        cmd.field_border = True
        cmd.penalty_area = True

        if diagnostico.ativo():
            self._diagnostico_comum(a, rid, alvo_x, alvo_y, arma)
            self.diagnosticar(a, rid, cmd)
        return cmd

    def _diagnostico_comum(self, a, rid, alvo_x, alvo_y, arma):
        robo, ball = a.mundo.aliados[rid], a.bola
        d = hypot(robo.position_x - ball.position_x, robo.position_y - ball.position_y)
        diagnostico.log("[JG] r%d papel=%s pos=%.0f,%.0f alvo=%.0f,%.0f d=%.0f"
                        % (rid, self.nome, robo.position_x, robo.position_y, alvo_x,
                           alvo_y, hypot(alvo_x - robo.position_x, alvo_y - robo.position_y)))
        com_a_bola = d < RAIO_POSSE_PORTADOR and all(
            d <= hypot(o.position_x - ball.position_x, o.position_y - ball.position_y) + 1.0
            for rid_o, o in a.mundo.aliados.items()
            if rid_o not in (GOALKEEPER_ID, rid))
        if com_a_bola:
            diagnostico.log("[JG] COM_A_BOLA r%d papel=%s d=%.0f alvo=%s"
                            % (rid, self.nome, d, a.tipo_alvo))
        # In the robot's frame, which is what grSim judges (robot.cpp:120-128).
        xx, yy, _ = chute.geometria_do_chutador(robo, ball)
        frente = (a.alvo_chute is not None
                  and chute.direcao_para_frente(ball, a.alvo_chute, a.mundo.sentido))
        diagnostico.log("[JG] PLACA r%d papel=%s xx=%.0f yy=%.0f arma=%s dispara=%s"
                        % (rid, self.nome, xx, yy, arma, chute.na_janela_de_disparo(xx, yy)))
        diagnostico.log("[JG] arma=%s frente=%s d=%.0f tipo=%s" % (arma, frente, d, a.tipo_alvo))
