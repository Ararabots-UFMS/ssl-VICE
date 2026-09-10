from strategy.skills.skills import Skills
from utils.math_util import Vector2D
from math import atan2, hypot


class Goalkeeper:
    def __init__(self, gk_info, ball, on_positive_half):
        self.name = "Goalkeeper"
        self.skills_factory = Skills("Movement")
        self.gk = gk_info
        self.ball = ball
        self.padding = 100
        self.on_positive_half = on_positive_half

    def _get_ball_angle(self) -> float:
        dx = self.ball.position_x - self.gk.position_x
        dy = self.ball.position_y - self.gk.position_y
        angle = atan2(dy, dx)
        return angle

    def _ball_in_goal_area(self) -> bool:
        # deprecated: keep for compatibility but prefer using goal_position-aware check in execute
        if self.on_positive_half:
            if (
                self.ball.position_x > 1750.0 - self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            ):
                return True
        else:
            if (
                self.ball.position_x < -1750.0 + self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            ):
                return True

        return False

    def execute(self, goal_position: Vector2D, ball: Vector2D):
        """
        Quando a bola está na área do gol, o goleiro segue a lógica de ataque:
        - posiciona-se atrás da bola (considerando a direção de empurrar para fora do gol)
        - quando estiver próximo o suficiente, empurra a bola para fora da área

        Caso contrário, posiciona-se no centro da meta alinhado com a posição y da bola.
        """
        # ângulo do robô olhando para a bola (usado para orientação)
        angle = self._get_ball_angle()

        # Se a bola estiver na área do gol (uso goal_position para suportar ambos os lados)
        # definimos a área em função do lado do gol recebido.
        goal_x = goal_position.x
        if goal_x > 0:
            in_area = (
                self.ball.position_x > 1750.0 - self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            )
        else:
            in_area = (
                self.ball.position_x < -1750.0 + self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            )

        if in_area:
            bx, by = ball.position_x, ball.position_y

            # direção para empurrar: do centro da meta para a bola (ou seja, do gol para fora)
            dx = bx - goal_position.x
            dy = by - goal_position.y
            norm = hypot(dx, dy) or 1.0
            ux, uy = dx / norm, dy / norm

            # ponto "atrás" da bola para se posicionar antes de empurrar
            behind_dist = 70.0
            stage_x = bx - ux * behind_dist
            stage_y = by - uy * behind_dist

            # obter posição atual do goleiro
            rx = getattr(self.gk, "position_x", None)
            ry = getattr(self.gk, "position_y", None)

            dist_to_stage = hypot(rx - stage_x, ry - stage_y) if rx is not None else float("inf")
            dist_to_ball = hypot(rx - bx, ry - by) if rx is not None else float("inf")

            # se estiver próximo o suficiente, faça o push; senão aproxime-se e alinhe-se atrás
            if dist_to_stage < 230.0 or dist_to_ball < 230.0:
                push_dist = 220.0
                target_x = bx + ux * push_dist
                target_y = by + uy * push_dist

                robot_command = self.skills_factory.move_with_angle(
                    robot_id=0,
                    target_x=target_x,
                    target_y=target_y,
                    vel_x=0.0,
                    vel_y=0.0,
                    angle=angle,
                )

                # permitir interação com a bola para empurrar
                robot_command.field_border = True
                robot_command.ally_ids = []
                # kicker do goleiro deve ficar desativado por padrão aqui
                try:
                    robot_command.deactivate_kick()
                except Exception:
                    pass

                return robot_command
            else:
                # mover para o ponto de alinhamento atrás da bola
                robot_command = self.skills_factory.move_with_angle(
                    robot_id=0,
                    target_x=stage_x,
                    target_y=stage_y,
                    vel_x=0.0,
                    vel_y=0.0,
                    angle=angle,
                )
                robot_command.field_border = True
                robot_command.ally_ids = []
                try:
                    robot_command.deactivate_kick()
                except Exception:
                    pass

                return robot_command

        robot_command = self.skills_factory.move_with_angle(
            robot_id=0,
            target_x=goal_position.x,
            target_y=max(-400, min(400, ball.position_y)),
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )
        robot_command.field_border = True
        return robot_command
