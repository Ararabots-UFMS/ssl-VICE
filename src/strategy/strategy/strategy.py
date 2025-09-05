from rclpy.node import Node

from strategy.plays.root import RootTree
from strategy.robot_client import RobotClient

class Strategy(Node):
    def __init__(self):
        super().__init__('strategy_node')
        self.get_logger().info("Strategy node initialized")
        self.cli = RobotClient()
        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        # The code below just create a simple behaviour tree which is available in strategy

        status, action = RootTree("RootStrategy").run()

        if action is not None:
            self.cli.send_request(action)
        else:
            self.get_logger().info("No valid action found")

