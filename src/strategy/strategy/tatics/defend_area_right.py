from strategy.behaviour import LeafNode, TaskStatus
from strategy.skills.skills import Skills
import math
class DefendAreaRight(LeafNode):
    """Tática que move um robô para defender uma área específica do campo, posicionando-se entre a bola e o gol.
    O robô se posiciona em uma linha reta entre a bola e o gol, a uma distância segura da bola.
    """

    def __init__(self, name, robot_id: int, robot_x: float, robot_y:float, ball_x: float, ball_y: float, goal_x: float = -2250, goal_y: float = 0, robot_radius: float = 85, distance: float = 5):
        super().__init__(name)

        self.robot_id = robot_id
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.ball_x = ball_x
        self.ball_y = ball_y
        self.robot_radius = robot_radius
        self.distance = distance
        skills_factory = Skills("Movement")

        dist_ball = (4000 - ball_x)/(goal_x - ball_x)
        target_x = 4000 - robot_radius * (goal_x - ball_x) / math.sqrt((goal_x - ball_x) ** 2 + ((goal_y - ball_y) ** 2))
        target_y = ball_y + dist_ball * (goal_y - ball_y) - robot_radius * (goal_y - ball_y) / math.sqrt((goal_x - ball_x) ** 2 + ((goal_y - ball_y) ** 2))
        angle_to_ball = math.atan2(ball_y - target_y, ball_x - target_x)

        s = skills_factory.move_with_angle(
            robot_id=robot_id,
            target_x=target_x,
            target_y=target_y,
            angle=angle_to_ball,
            vel_x=0.0,
            vel_y=0.0,
        )

        s.field_border = True
        s.penalty_area = True

        self.robots_action = [s]

    def run(self):
        return TaskStatus.RUNNING, list(self.robots_action)