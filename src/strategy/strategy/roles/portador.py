"""PORTADOR: has the ball or goes for it.

Phases, decided every tick by geometry (skills/aproximacao.fase_de_aproximacao):

  CONTORNAR    go to a fixed point behind the ball; the ball is an obstacle, so the
               planner does the circling.
  MIRAR        hold that point and turn until the body is on the line of fire.
  EMPURRAR     go through the ball towards the kick target; this is where it kicks.
  DESENCOSTAR  inside the ball's keep-out: step radially out first.

A phase only holds for the tick that decided it. Branches that return early
(ball already kicked, legacy loose ball) leave it empty.
"""

from dataclasses import dataclass
import math
from math import atan2, cos, hypot, sin
from typing import Optional

from strategy import campo, diagnostico
from strategy.analysis.jogo import AnaliseJogo
from strategy.analysis.mundo import SITUACAO_SOLTA
from strategy.roles.atribuicao import PAPEL_PORTADOR
from strategy.roles.base import Papel, versor_ataque
from strategy.skills import aproximacao, experimento, geometria
from strategy.skills import bola as skill_bola

CONTORNAR = "contornar"
MIRAR = "mirar"
EMPURRAR = "empurrar"
DESENCOSTAR = "desencostar"

# Body error still worth stopping to aim for. 0.25 rad = 14 deg, inside the
# kick gate's 20 so it leaves the aim phase already within it.
TOL_CORPO_MIRA = 0.25
# Only worth stopping to aim close to the ball; far away it turns while driving.
RAIO_MIRA_PARADO = 600.0
# 2 s of aiming without turning, then push anyway: standing still loses the ball.
CICLOS_MAX_MIRANDO = 20
# Error drop per tick that still counts as turning (0.1 rad/s).
GIRO_MIN_MIRA = 0.01
# Arrival speed at the waiting point, towards the ball: turns the point into a
# state so the planner comes from behind (-121/-142 deg -> -6/-8 measured).
VEL_CHEGADA_CONTORNO = 400.0
# Arrival speed when pushing, only with the COM_CHEGADA_EM_MOVIMENTO experiment.
VEL_CHEGADA_EMPURRAO = 600.0
# Start the push only this close to the line of fire: grSim fires with the ball within
# 40 mm of the kicker centre, and entering at 90 made 16% of first touches pushes.
LATERAL_ENTRADA = 40.0
# Leave the push beyond this: the old entry limit, the hull's radius.
LATERAL_SAIDA = 90.0
# ... and only from near the waiting point, where the last stretch is straight.
RAIO_ENTRADA = aproximacao.APROXIMACAO_ATRAS + 150.0
# Not settled after 1.5 s that close: accept the old corridor rather than stall.
CICLOS_MAX_ALINHANDO = 15


@dataclass
class MemoriaPortador:
    fase: Optional[str] = None
    ciclos_mirando: int = 0
    erro_mira: Optional[float] = None
    ciclos_alinhando: int = 0


class Portador(Papel):
    nome = PAPEL_PORTADOR

    def __init__(self):
        self.memoria = MemoriaPortador()

    def fase(self) -> Optional[str]:
        if experimento.desligado("PLANEJADOR"):
            return None
        return self.memoria.fase

    def alvo(self, a: AnaliseJogo, rid: int, ordem: int):
        m = self.memoria
        fase_anterior, m.fase = m.fase, None
        ball, robo = a.bola, a.mundo.aliados[rid]
        bx, by = ball.position_x, ball.position_y
        rx, ry = robo.position_x, robo.position_y
        ux, uy, _, _ = versor_ataque(a)
        solta = a.mundo.situacao == SITUACAO_SOLTA

        # Legacy loose ball (planner switched off): run to where it will stop.
        if solta and experimento.desligado("PLANEJADOR"):
            fx, fy = skill_bola.onde_a_bola_vai(ball, 1.0)
            direcao = a.alvo_chute if a.alvo_chute is not None else (
                fx + ux * 1000.0, fy + uy * 1000.0)
            return aproximacao.ponto_de_aproximacao(
                rx, ry, fx, fy, direcao, aproximacao.AVANCO_SOLTA,
                contornar=not experimento.desligado("ORBITA"),
                empurrar=not experimento.desligado("EMPURRAO"))

        # Ball already kicked: intercept beyond it. Catching a ball in flight brakes
        # it (robot.cpp:168); the 1.6 m follow-through is what produces the kick.
        if skill_bola.bola_ja_saiu(ball):
            avanco = aproximacao.AVANCO_SOLTA if solta else aproximacao.AVANCO_PORTADOR
            return self._fora_das_areas(
                ball, aproximacao.ponto_de_interceptacao(rx, ry, ball, avanco))

        direcao = a.alvo_chute if a.alvo_chute is not None else (
            bx + ux * 1000.0, by + uy * 1000.0)
        avanco = aproximacao.AVANCO_SOLTA if solta else aproximacao.AVANCO_PORTADOR

        if not experimento.desligado("PLANEJADOR"):
            fase = aproximacao.fase_de_aproximacao(rx, ry, bx, by, direcao, fase_anterior)
            if experimento.ligado("SO_POSICIONAR") and fase == EMPURRAR:
                fase = CONTORNAR
            fase = self._so_na_linha(robo, ball, direcao, fase, fase_anterior)
            fase = self._talvez_mirar(robo, ball, direcao, fase)
            m.fase = fase
            if fase == MIRAR:
                return aproximacao.ponto_de_espera(bx, by, direcao)
            if fase == DESENCOSTAR:
                return aproximacao.ponto_de_desencoste(rx, ry, bx, by)
            if fase == CONTORNAR:
                return aproximacao.ponto_de_espera(bx, by, direcao)

        # EMPURRAR. The long-destination variant lost the ball (measured 09/10).
        if experimento.ligado("COM_DESTINO_LONGO") and not experimento.desligado("PLANEJADOR"):
            return self._fora_das_areas(ball, aproximacao.destino_de_empurrao(bx, by, direcao))
        ponto = aproximacao.ponto_de_aproximacao(
            rx, ry, bx, by, direcao, avanco,
            contornar=not experimento.desligado("ORBITA"),
            empurrar=not experimento.desligado("EMPURRAO"))
        return self._fora_das_areas(ball, ponto)

    @staticmethod
    def _fora_das_areas(ball, ponto):
        """A target in a penalty area, cut back along the line from the ball.

        The planner would move it to the area's nearest edge, off the line of fire:
        the robot then met the ball off-centre and pushed it into the area.
        """
        if experimento.desligado("RECORTE_AREA"):
            return ponto
        return campo.recortar_antes_das_areas(ball.position_x, ball.position_y,
                                              ponto[0], ponto[1])

    def _so_na_linha(self, robo, ball, direcao, fase, fase_anterior):
        """Push only from on the line of fire; until then keep going to the waiting point."""
        m = self.memoria
        if fase != EMPURRAR or experimento.desligado("CHEGADA_NA_LINHA"):
            m.ciclos_alinhando = 0
            return fase
        bx, by = ball.position_x, ball.position_y
        a = atan2(direcao[1] - by, direcao[0] - bx)
        dx, dy = robo.position_x - bx, robo.position_y - by
        lateral = abs(dx * sin(a) - dy * cos(a))
        if fase_anterior == EMPURRAR:
            return fase if lateral <= LATERAL_SAIDA else CONTORNAR
        if hypot(dx, dy) > RAIO_ENTRADA:
            m.ciclos_alinhando = 0
            return CONTORNAR
        if lateral <= LATERAL_ENTRADA:
            m.ciclos_alinhando = 0
            return EMPURRAR
        m.ciclos_alinhando += 1
        if m.ciclos_alinhando >= CICLOS_MAX_ALINHANDO:
            m.ciclos_alinhando = 0
            return EMPURRAR
        return CONTORNAR

    def _talvez_mirar(self, robo, ball, direcao, fase):
        """Stop and turn before pushing: it reached the ball before finishing the turn.

        The cap counts only ticks without turning, so a slow 180 deg turn is not cut.
        """
        m = self.memoria
        if fase != EMPURRAR or experimento.desligado("PARAR_E_MIRAR"):
            m.ciclos_mirando, m.erro_mira = 0, None
            return fase
        bx, by = ball.position_x, ball.position_y
        erro = abs(geometria.norm_ang(
            robo.orientation - atan2(direcao[1] - by, direcao[0] - bx)))
        d = hypot(robo.position_x - bx, robo.position_y - by)
        n = m.ciclos_mirando
        if m.erro_mira is not None and erro < m.erro_mira - GIRO_MIN_MIRA:
            n = 0
        if erro > TOL_CORPO_MIRA and d < RAIO_MIRA_PARADO and n < CICLOS_MAX_MIRANDO:
            m.ciclos_mirando, m.erro_mira = n + 1, erro
            return MIRAR
        m.ciclos_mirando, m.erro_mira = 0, None
        return fase

    def velocidade_chegada(self, a: AnaliseJogo, rid: int, x: float, y: float):
        vx = vy = 0.0
        if a.alvo_chute is None:
            return vx, vy
        bx, by = a.bola.position_x, a.bola.position_y
        if not experimento.desligado("CHEGADA_ALINHADA") and self.memoria.fase == CONTORNAR:
            cx, cy = bx - x, by - y
            n = hypot(cx, cy) or 1.0
            vx, vy = cx / n * VEL_CHEGADA_CONTORNO, cy / n * VEL_CHEGADA_CONTORNO
        if experimento.ligado("COM_CHEGADA_EM_MOVIMENTO") and self.memoria.fase == EMPURRAR:
            dx, dy = a.alvo_chute[0] - bx, a.alvo_chute[1] - by
            n = hypot(dx, dy) or 1.0
            vx, vy = dx / n * VEL_CHEGADA_EMPURRAO, dy / n * VEL_CHEGADA_EMPURRAO
        return vx, vy

    def evita_bola(self, a: AnaliseJogo, rid: int) -> bool:
        # The ball is an obstacle while circling and aiming, and only stops being
        # one to push through it. Not while stepping out: it is already inside it.
        if experimento.desligado("PLANEJADOR"):
            return False
        return self.memoria.fase in (CONTORNAR, MIRAR)

    def diagnosticar(self, a: AnaliseJogo, rid: int, cmd) -> None:
        robo = a.mundo.aliados[rid]
        if a.alvo_chute is not None:
            # Where the robot, its velocity and its target sit across the line of fire.
            bx, by = a.bola.position_x, a.bola.position_y
            ang = atan2(a.alvo_chute[1] - by, a.alvo_chute[0] - bx)
            nx, ny = -sin(ang), cos(ang)
            diagnostico.log(
                "[JG] LINHA r%d fase=%s lat=%.0f vlat=%.0f alvo_lat=%.0f ao_longo=%.0f "
                "corpo=%.1f" %
                (rid, self.memoria.fase,
                 (robo.position_x - bx) * nx + (robo.position_y - by) * ny,
                 (robo.velocity_x or 0.0) * nx + (robo.velocity_y or 0.0) * ny,
                 (cmd.target_x - bx) * nx + (cmd.target_y - by) * ny,
                 (robo.position_x - bx) * cos(ang) + (robo.position_y - by) * sin(ang),
                 math.degrees(geometria.norm_ang(robo.orientation - ang))))
        receptor = a.alvo.receptor[2] if a.alvo.receptor is not None else None
        diagnostico.log_e_arquivo(
            "[JG] MOV r%d fase=%s tipo=%s receptor=%s d_bola=%.0f "
            "alvo=%.0f,%.0f evita_bola=%s kick=%.1f" %
            (rid, self.memoria.fase, a.tipo_alvo, receptor,
             hypot(robo.position_x - a.bola.position_x, robo.position_y - a.bola.position_y),
             cmd.target_x, cmd.target_y, cmd.ball, cmd.kick))
