from strategy.behaviour import LeafNode, TaskStatus
from strategy.skills.skills import Skills
import math

class GoForAimingGoalLeftKick(LeafNode):
    """Tática que move um robô para uma posição de ataque em direção ao gol adversário. 
    O robô se posiciona em uma linha reta entre a bola e o gol adversário, a uma distância segura da bola.
    """

    def __init__(self, name, robot_id: int, robot_x: float, robot_y:float, ball_x: float, ball_y: float, goal_x: float = 2250, goal_y: float = 0, robot_radius: float = 85):
        super().__init__(name)

        self.robot_id = robot_id
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.ball_x = ball_x
        self.ball_y = ball_y
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.robot_radius = robot_radius
        skills_factory = Skills("Movement")

        target_x = ball_x + robot_radius  * (ball_x - goal_x) / math.sqrt((ball_x - goal_x) ** 2 + ((ball_y - goal_y) ** 2))
        target_y = ball_y + robot_radius * (ball_y - goal_y) / math.sqrt((ball_x - goal_x) ** 2 + ((ball_y - goal_y)) ** 2)
        angle_to_goal = math.atan2(goal_y - target_y, goal_x - target_x)

        s = skills_factory.move_with_angle(
            robot_id=robot_id,
            target_x=target_x,
            target_y=target_y,
            angle=angle_to_goal,
            vel_x=0.0,
            vel_y=0.0,
        )

        s.activate_kick()

        s.field_border = True
        s.penalty_area = True

        self.robots_action = [s]

    def run(self):
        return TaskStatus.RUNNING, list(self.robots_action)