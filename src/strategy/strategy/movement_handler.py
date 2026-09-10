from typing import Iterable
from movement_interfaces.msg import MovementCommand, MovementCommandArray
from system_interfaces.srv import SetOrientation, UpdateKick
from strategy.skills.skills import Skill


class MovementHandler:
    def __init__(self, node):

        self.node = node

        # Planner needs every robot's target in the same message to plan them against each other
        self.movement_pub = self.node.create_publisher(
            MovementCommandArray, "movement_manager/commands", 10
        )

        self.orientation_client = self.node.create_client(SetOrientation, "set_orientation")
        self.kick_client = self.node.create_client(UpdateKick, "update_kick")

        self._wait_for_services()

    def handle_movement(self, skills: Iterable[Skill]) -> None:
        commands = []
        for skill in skills:
            self._send_kick(skill)
            if skill.target_x is not None and skill.target_y is not None:
                commands.append(self._movement_command(skill))
            if skill.angle is not None:
                self._send_orientation(skill.robot_id, skill.angle)

        self._send_movement(commands)

    def _wait_for_services(self):
        while not self.orientation_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().info('Aguardando serviço "set_orientation"...')
        while not self.kick_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().info('Aguardando serviço "kick_command"...')
    
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
        fut = self.kick_client.call_async(req)
        fut.add_done_callback(lambda f, rid=req.id: self._handle_kick_response(f, rid))

    def _send_orientation(self, robot_id: int, angle: float) -> None:
        try:
            req = SetOrientation.Request()
        except Exception:
            return
        req.robot_id = int(robot_id)
        req.orientation = float(angle)
        fut = self.orientation_client.call_async(req)
        fut.add_done_callback(
            lambda f, rid=robot_id: self._handle_orientation_response(f, rid)
        )

    def _handle_kick_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.node.get_logger().error(f"Kick service failed for robot {robot_id}: {e}")

    def _handle_orientation_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.node.get_logger().error(
                f"Orientation service failed for robot {robot_id}: {e}"
            )