from rclpy.node import Node

from time import time

from strategy.blackboard import Blackboard
from movement.move import Movement

from ruckig import Trajectory


class Robot(Node):
    def __init__(self, id, name) -> None:
        super().__init__(f"robot_{name}")
        self.blackboard = Blackboard()
        self.name = name
        self.id = id
        self.behaviour_tree = None

        self.move = Movement(self.id)
        self.trajectory = Trajectory(3)  # Degrees of freedom = 3
        self.trajectory_start_time = 0

        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        self.get_logger().info(f"Running robot {self.id}")
        # profile = self.behaviour_tree.run()
        # self.trajectory = self.move(profile)
        # self.trajectory_start_time = time()
