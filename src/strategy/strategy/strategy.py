import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from strategy.root import RootTree
from typing import Iterable
from movement_interfaces.msg import MovementCommand, MovementCommandArray
from system_interfaces.srv import SetOrientation, UpdateKick
from strategy.skills.skills import Skill


class Strategy(Node):
    def __init__(self, wait_for_service: bool = True):
        super().__init__("strategy_node")
        self.get_logger().info("Strategy node initialized")

        # Movement goes out as one command array per tick. The planner needs every
        # robot's target in the same message to plan them against each other, and a
        # per-robot service call could not express that.
        self.movement_pub = self.create_publisher(
            MovementCommandArray, "movement_manager/commands", 10
        )
        self.orientation_cli = self.create_client(SetOrientation, "set_orientation")
        self.kick_cli = self.create_client(UpdateKick, "update_kick")

        if wait_for_service:
            while not self.orientation_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "set_orientation"...')
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

        commands = []
        for sk in latest.values():
            self._send_kick(sk)
            if sk.target_x is not None and sk.target_y is not None:
                commands.append(self._movement_command(sk))
            if sk.angle is not None:
                self._send_orientation(sk.robot_id, sk.angle)

        self._send_movement(commands)

    def _movement_command(self, skill: Skill) -> MovementCommand:
        """
        One robot's target, with the obstacles it is allowed to ignore.

        The skill's enemy_ids/ally_ids are not carried over: the obstacle factory always
        builds an obstacle for every robot on the field, so there was never a way to
        opt out of one. field_border is likewise not optional — leaving the field is
        never legal.
        """
        cmd = MovementCommand()
        cmd.robot_id = int(skill.robot_id)
        cmd.target_pos.x = float(skill.target_x or 0.0)
        cmd.target_pos.y = float(skill.target_y or 0.0)
        cmd.target_vel.x = float(skill.vel_x)
        cmd.target_vel.y = float(skill.vel_y)
        cmd.planning_options.avoid_penalty_area = bool(skill.penalty_area)
        cmd.planning_options.avoid_center_area = bool(skill.center_area)
        cmd.planning_options.avoid_ball = bool(skill.ball)
        return cmd

    def _send_movement(self, commands: list[MovementCommand]) -> None:
        msg = MovementCommandArray()
        msg.commands = commands
        self.movement_pub.publish(msg)

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

    def _handle_kick_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Kick service failed for robot {robot_id}: {e}")

    def _handle_orientation_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(
                f"Orientation service failed for robot {robot_id}: {e}"
            )


def _traverse_tree(node):
    nodes = [node]
    for child in getattr(node, "children", []):
        nodes.extend(_traverse_tree(child))
    return nodes


def main(args=None):
    rclpy.init(args=args)
    strategy_node = Strategy(wait_for_service=True)

    executor = MultiThreadedExecutor()
    executor.add_node(strategy_node)

    bt_nodes = _traverse_tree(strategy_node.root)
    for n in bt_nodes:
        if isinstance(n, Node):
            executor.add_node(n)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in bt_nodes:
            if isinstance(n, Node):
                executor.remove_node(n)
                n.destroy_node()

        executor.remove_node(strategy_node)
        strategy_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
