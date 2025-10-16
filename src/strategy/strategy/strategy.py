import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from strategy.root import RootTree
from typing import Iterable
from system_interfaces.srv import StrategyCommand, SetOrientation, UpdateObstacle, UpdateKick
from strategy.skills.skills import Skill


class Strategy(Node):
    def __init__(self, wait_for_service: bool = True):
        super().__init__("strategy_node")
        self.get_logger().info("Strategy node initialized")

        self.move_cli = self.create_client(StrategyCommand, "strategy_command")
        self.orientation_cli = self.create_client(SetOrientation, "set_orientation")
        self.obstacle_cli = self.create_client(UpdateObstacle, "update_obstacles")
        self.kick_cli = self.create_client(UpdateKick, "update_kick")

        if wait_for_service:
            while not self.move_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "strategy_command"...')
            while not self.orientation_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "set_orientation"...')
            while not self.obstacle_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "update_obstacles"...')
            while not self.kick_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "kick_command"...')

        self.timer = self.create_timer(0.1, self.run)
        self.root = RootTree("RootStrategy")

    def run(self):
        status, action = self.root.run()

        if action is None:
            return

        if isinstance(action, Iterable) and not isinstance(action, Skill):
            skill_list = [s for s in action if hasattr(s, "robot_id")]
        else:
            skill_list = [action] if hasattr(action, "robot_id") else []

        latest: dict[int, Skill] = {sk.robot_id: sk for sk in skill_list}

        for sk in latest.values():
            self._send_kick(sk)
            if sk.target_x is not None and sk.target_y is not None:
                self._send_move(sk)
            if sk.angle is not None:
                self._send_orientation(sk.robot_id, sk.angle)
            if any(
                field
                for field in [
                    sk.field_border,
                    sk.penalty_area,
                    sk.center_area,
                    sk.ball,
                    sk.enemy_ids,
                    sk.ally_ids,
                ]
            ):
                self._send_obstacles(sk)

    def _send_move(self, skill: Skill) -> None:
        req = StrategyCommand.Request()
        req.id = int(skill.robot_id)
        req.position_x = float(skill.target_x or 0.0)
        req.position_y = float(skill.target_y or 0.0)
        req.velocity_x = float(skill.vel_x)
        req.velocity_y = float(skill.vel_y)
        fut = self.move_cli.call_async(req)
        fut.add_done_callback(lambda f, rid=req.id: self._handle_move_response(f, rid))

    def _send_kick(self, skill: Skill) -> None:
        req = UpdateKick.Request()
        req.id = int(skill.robot_id)
        req.kick = float(skill.kick)
        fut = self.kick_cli.call_async(req)
        fut.add_done_callback(lambda f, rid=req.id: self._handle_kick_response(f, rid))

    def _send_orientation(self, robot_id: int, angle: float) -> None:
        try:
            req = SetOrientation.Request()
        except Exception:
            return
        req.robot_id = int(robot_id)
        req.orientation = float(angle)
        fut = self.orientation_cli.call_async(req)
        fut.add_done_callback(
            lambda f, rid=robot_id: self._handle_orientation_response(f, rid)
        )

    def _send_obstacles(self, skill: Skill) -> None:

        req = UpdateObstacle.Request()
        req.id = int(skill.robot_id)
        req.field_border = bool(skill.field_border)
        req.penalty_area = bool(skill.penalty_area)
        req.center_area = bool(skill.center_area)
        req.ball = bool(skill.ball)
        req.enemy_ids = list(skill.enemy_ids or [])
        req.ally_ids = list(skill.ally_ids or [])
        fut = self.obstacle_cli.call_async(req)
        fut.add_done_callback(
            lambda f, rid=req.id: self._handle_obstacle_response(f, rid, skill)
        )

    def _handle_kick_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Kick service failed for robot {robot_id}: {e}")

    def _handle_move_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Move service failed for robot {robot_id}: {e}")

    def _handle_orientation_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(
                f"Orientation service failed for robot {robot_id}: {e}"
            )

    def _handle_obstacle_response(self, future, robot_id: int, skill: Skill) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(
                f"Obstacle service failed for robot {robot_id}: {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    strategy_node = Strategy(wait_for_service=True)

    def _traverse_tree(node):
        nodes = [node]
        for child in getattr(node, "children", []):
            nodes.extend(_traverse_tree(child))
        return nodes

    executor = MultiThreadedExecutor()
    executor.add_node(strategy_node)

    bt_nodes = _traverse_tree(strategy_node.root)
    seen = set()
    unique_bt_nodes = []
    for n in bt_nodes:
        if id(n) not in seen:
            seen.add(id(n))
            unique_bt_nodes.append(n)

    for n in unique_bt_nodes:
        executor.add_node(n)

    try:
        executor.spin()
    finally:
        for n in unique_bt_nodes:
            executor.remove_node(n)
            n.destroy_node()
        executor.shutdown()
        strategy_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
