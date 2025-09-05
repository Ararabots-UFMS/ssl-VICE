from rclpy.node import Node
from strategy.plays.root import RootTree
from typing import Iterable
from system_interfaces.srv import StrategyCommand


class Strategy(Node):
    def __init__(self, wait_for_service: bool = True):
        super().__init__('strategy_node')
        self.get_logger().info("Strategy node initialized")

        self.cli = self.create_client(StrategyCommand, "strategy_command")

        if wait_for_service:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service "strategy_command"...')

        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        status, action = RootTree("RootStrategy").run()

        if status == status.RUNNING and action is not None:
            self.send_request(action)
        else:
            self.get_logger().info("No valid action found")

    def send_request(self, skill_or_iter) -> None:
            if isinstance(skill_or_iter, Iterable) and not hasattr(skill_or_iter, "robot_id"):
                for s in skill_or_iter:
                    self.send_request(s)
                return

            skill = skill_or_iter
            if not hasattr(skill, "robot_id"):
                self.get_logger().warn("Ignoring skill without robot_id")
                return

            req = StrategyCommand.Request()
            req.id = int(skill.robot_id)
            req.position_x = float(getattr(skill, "target_x", 0.0))
            req.position_y = float(getattr(skill, "target_y", 0.0))
            req.velocity_x = float(getattr(skill, "vel_x", 0.0))
            req.velocity_y = float(getattr(skill, "vel_y", 0.0))

            fut = self.cli.call_async(req)
            fut.add_done_callback(lambda f, rid=req.id: self._handle_response(f, rid))

    def _handle_response(self, future, robot_id: int) -> None:
        try:
            resp = future.result()
            if getattr(resp, "success", False):
                self.get_logger().info(f"Robot {robot_id}: command accepted.")
            else:
                self.get_logger().warn(f"Robot {robot_id}: command rejected by path_driver.")
        except Exception as e:
            self.get_logger().error(f"Service call failed for robot {robot_id}: {e}")
