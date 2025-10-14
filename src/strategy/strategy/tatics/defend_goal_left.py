from strategy.behaviour import LeafNode, TaskStatus
from strategy.skills.skills import Skills
import math

class DefendGoalLeft(LeafNode):
    """Tática que move um robô para defender uma área específica do campo, posicionando-se entre a bola e o gol.
    O robô se posiciona em uma linha reta entre a bola e o gol, a uma distância segura da bola.
    """

    def __init__(self, name, robot_id: int, robot_x: float, robot_y:float, ball_x: float, ball_y: float, goal_x: float = 2250, goal_y: float = 0,):
        super().__init__(name)

        self.robot_id = robot_id
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.ball_x = ball_x
        self.ball_y = ball_y
        skills_factory = Skills("Movement")

        target_x = goal_x
        target_y = goal_y
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
        s.ally_ids = [0,1]

        self.robots_action = [s]

    def run(self):
        return TaskStatus.RUNNING, list(self.robots_action)