import rclpy
from rclpy.node import Node
from typing import Iterable
from strategy.skills.move import MoveSkill
from system_interfaces.srv import StrategyCommand
from typing import Iterable

class MockStrategy(Node):
    def __init__(self, wait_for_service: bool = True):
        super().__init__("mock_strategy_node")
        self.get_logger().info("Mock Strategy node initialized")
        self.cli = self.create_client(StrategyCommand, "strategy_command")

        if wait_for_service:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service "strategy_command"...')
        
        self.timer = self.create_timer(0.1, self.run)

    def run(self):
        cmds = unique_command()
        if cmds:
            self.send_request(cmds)
        else:
            self.get_logger().info("No valid action found")
        pass

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
                #fut.add_done_callback(lambda f, rid=req.id: self._handle_response(f, rid))


def main(args=None):
    rclpy.init(args=args)
    strategy = MockStrategy()
    rclpy.spin(strategy)
    rclpy.shutdown()


def unique_command(args=None):

    cmds = [
        MoveSkill(name="tri_r0", robot_id=2, target_x=-250.0, target_y=0.0),
        MoveSkill(name="tri_r1", robot_id=1, target_x=-1225.0, target_y=750.0),
        MoveSkill(name="tri_r2", robot_id=0, target_x=-1225.0, target_y=-750.0),
    ]

    return cmds

if __name__ == '__main__':
    main()