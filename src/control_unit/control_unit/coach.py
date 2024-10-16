import rclpy.executors
from rclpy.node import Node
import rclpy

from control_unit.robot import Robot
from strategy.blackboard import Blackboard


class Coach(Node):
    def __init__(self, _executor, behaviour_tree) -> None:
        super().__init__("coach")

        self._executor = _executor
        self.behaviour_tree = behaviour_tree

        # get blackboard
        self.blackboard = Blackboard()

        self.robots = {}

        # TODO: experiment with other timer rates
        self.timer = self.create_timer(0.5, self.update)
        self.timer = self.create_timer(0.1, self.run)

    def update(self):
        # If a robot is in the blackboard and it doens't exist, it is created
        for ally_robot in self.blackboard.ally_robots.values():
            try:
                self.robots[ally_robot.id]
            except:
                # TODO: implementar nome dos robos
                self.robots[ally_robot.id] = Robot(
                    ally_robot.id, f"robo{ally_robot.id}"
                )
                self._executor.add_node(self.robots[ally_robot.id])

        # If a robot is not in the blackboard, it is destroyed
        # The list is used to avoid "RuntimeError: dictionary changed size during iteration"
        for robot in list(self.robots.values()):
            if not any(
                robot.id == ally_robot.id
                for ally_robot in self.blackboard.ally_robots.values()
            ):
                self.get_logger().info(f"Destroying robot {robot.id}")
                self._executor.remove_node(self.robots[robot.id])
                self.robots[robot.id].destroy_node()
                self.robots.pop(robot.id)

    def run(self):
        self.get_logger().info(f"Running")
        # self.behaviour_tree.run(self.blackboard)


def main(args=None):
    rclpy.init(args=args)
    coach = Coach(None)
    rclpy.spin(coach)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
