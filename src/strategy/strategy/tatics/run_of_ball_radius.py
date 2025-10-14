import math

from strategy.behaviour import LeafNode, TaskStatus
from strategy.skills.skills import Skills


class RunOffBallRadius(LeafNode):
    """Tática simples: posiciona um robô ao redor da bola a uma certa distância (raio).
    Parâmetros:
      name: nome do node
      robot_id: id do robô que deve se mover
      ball_x, ball_y: posição da bola
      radius: distância desejada a partir da bola (padrão 585.0)
      angle: ângulo (rad) ao redor da bola onde o robô deve se posicionar (0 -> +x)
    """

    def __init__(self, name, robot_id: int, ball_x: float, ball_y: float, radius: float = 585.0, angle: float = 0.0):
        super().__init__(name)

        skills_factory = Skills("Movement")
        target_x = ball_x + radius * math.cos(angle)
        target_y = ball_y + radius * math.sin(angle)

        s = skills_factory.move_to(
            robot_id=robot_id,
            target_x=target_x,
            target_y=target_y,
            vel_x=0.0,
            vel_y=0.0,
        )


        s.ball = True  
        s.field_border = True

        self.robots_action = [s]

    def run(self):
        return TaskStatus.RUNNING, list(self.robots_action)