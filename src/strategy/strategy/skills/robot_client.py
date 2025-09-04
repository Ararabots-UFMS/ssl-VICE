from rclpy.node import Node
from typing import Optional, Dict

from system_interfaces.srv import StrategyCommand


class RobotClient(Node):
    """
    Single Node that manages multiple robots.
    state:
      - self.skills[robot_id] -> MoveTo instance (or any object with target_x/target_y)
      - self.last_response_success[robot_id] -> Optional[bool]
    """

    def __init__(self, tick_period: float = 0.1, wait_for_service: bool = True):
        super().__init__("robot_client")
        self.skills: Dict[int, object] = {}
        self.last_response_success: Dict[int, Optional[bool]] = {}
        self._wait_msg_emitted = False

        self.cli = self.create_client(StrategyCommand, "strategy_command")
        if wait_for_service:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                if not self._wait_msg_emitted:
                    self.get_logger().info('Serviço "strategy_command" não disponível, esperando...')
                    self._wait_msg_emitted = True

        self.get_logger().info("RobotClient (multi) criado e pronto.")
        self.create_timer(tick_period, self._timer_tick)

    def _timer_tick(self) -> None:
        try:
            self.tick()
        except Exception as e:
            self.get_logger().error(f"tick error: {e}")

    def set_skill(self, skill) -> None:
        """Register or replace a skill for a robot. skill must have .robot_id"""
        rid = int(skill.robot_id)
        self.skills[rid] = skill
        self.last_response_success[rid] = None
        self.get_logger().info(f"Skill set for robot {rid}: {getattr(skill, 'name', type(skill).__name__)}")

    def remove_skill(self, robot_id: int) -> None:
        self.skills.pop(robot_id, None)
        self.last_response_success.pop(robot_id, None)

    def tick(self) -> None:
        if not self.skills:
            return
        # iterate snapshot to allow modifications inside callbacks
        for rid, skill in list(self.skills.items()):
            # duck-typing: requires target_x / target_y
            if hasattr(skill, "target_x") and hasattr(skill, "target_y"):
                self.send_request(skill)

    def send_request(self, skill) -> None:
        req = StrategyCommand.Request()
        req.id = int(skill.robot_id)
        req.position_x = float(skill.target_x)
        req.position_y = float(skill.target_y)
        req.velocity_x = float(getattr(skill, "vel_x", 0.0))
        req.velocity_y = float(getattr(skill, "vel_y", 0.0))

        fut = self.cli.call_async(req)
        # capture robot_id for callback (avoid late-binding)
        robot_id = int(skill.robot_id)
        fut.add_done_callback(lambda fut, rid=robot_id: self._handle_response(fut, rid))

        self.get_logger().info(
            f"Sending strategy command: ID={req.id} Pos=({req.position_x},{req.position_y}) Vel=({req.velocity_x},{req.velocity_y})"
        )

    def _handle_response(self, future, robot_id: int) -> None:
        try:
            resp = future.result()
            ok = bool(getattr(resp, "success", False))
            self.last_response_success[robot_id] = ok
            if ok:
                self.get_logger().info(f"Robot {robot_id}: path_driver aceitou o comando.")
            else:
                self.get_logger().warning(f"Robot {robot_id}: path_driver recusou o comando.")
        except Exception as e:
            self.last_response_success[robot_id] = False
            self.get_logger().error(f"Chamada de serviço falhou para robot {robot_id}: {e}")
