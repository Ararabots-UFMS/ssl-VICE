from strategy.strategy.skills.skills import Skills
from strategy.behaviour import LeafNode, TaskStatus
import math

class AimToGoalLeft(LeafNode):

    def __init__(self, name, robot_id: int, robot_x: float, robot_y:float, goal_x: float = 2250, goal_y: float = 0):
        super().__init__(name)

        self.robot_id = robot_id
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        skills_factory = Skills("Aiming")

        angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)

        s = skills_factory.set_orientation(
            robot_id=robot_id,
            angle=angle_to_goal
        )

        s.field_border = True
        s.penalty_area = True
        
        self.robots_action = [s]

    def run(self):
        return TaskStatus.RUNNING, list(self.robots_action)