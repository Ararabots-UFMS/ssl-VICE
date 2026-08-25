import rclpy
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg
from new_movement.entities.States import State, Vector2D
from rclpy.node import Node

from control.p_controller import PController
from control.pid_controller import RobotTrajectoryController
from system_interfaces.msg import GameState, RobotCommand, TeamCommand
from system_interfaces.srv import (
    ControlParams,
    GetGameConfig,
    SetKp,
    SetOrientation,
    UpdateKick,
)


class Controller(Node):
    """Simplified controller node.

    Consumes high-frequency GameState and ControlCommand messages, publishes TeamCommand.
    Fetches low-frequency configuration (team color) once via GetGameConfig service.
    """

    def __init__(self):
        super().__init__("controller")
        # information about the game state
        self.referee_command = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

        self._desired_states = ["TIMEOUT_BLUE", "TIMEOUT_YELLOW", "HALT"]

        # Caches
        self.ally_robots = {}
        self.control_references = {}
        self.is_halt = None
        # Low-frequency config (polled periodically)
        self.is_team_color_yellow = False
        self.is_field_side_left = False
        self._config_client = self.create_client(GetGameConfig, "get_game_config")
        self._config_call_inflight = False
        self._last_config = None
        # Kick cache por robô: mantém o último valor até nova atualização
        self.kick_cache: dict[int, float] = {}

        # Poll game config every 0.5s for robustness when values change
        self.create_timer(0.5, self._poll_game_config)

        # Controllers
        self.robot_controller = RobotTrajectoryController()
        self.orientation_controller = PController(kp=1, max_output=2)
        self.target_orientations = {}

        # ROS Interfaces
        self.create_subscription(
            TrajectoryPointMsg,
            "movement_tracker/control_reference",
            self.receive_control_reference,
            10,
        )
        self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)
        self.create_service(ControlParams, "update_pid", self.update_parameters)
        self.create_service(
            SetOrientation, "set_orientation", self.set_orientation_callback
        )
        self.create_service(SetKp, "update_kp_angular", self.update_kp_angular_callback)
        self.create_service(UpdateKick, "update_kick", self.update_kick_callback)

        # Timing
        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.timer_callback)

    def receive_control_reference(self, msg: TrajectoryPointMsg):
        self.control_references[msg.robot_id] = msg

    def _poll_game_config(self):
        if self._config_call_inflight:
            return
        if not self._config_client.service_is_ready():
            return
        self._config_call_inflight = True
        future = self._config_client.call_async(GetGameConfig.Request())

        def done(fut):
            self._config_call_inflight = False
            try:
                resp = fut.result()
                # Log only on changes
                changed = (
                    self._last_config is None
                    or self._last_config.is_team_color_yellow
                    != resp.is_team_color_yellow
                    or getattr(self._last_config, "is_field_side_left", None)
                    != getattr(resp, "is_field_side_left", None)
                )
                self.is_team_color_yellow = bool(resp.is_team_color_yellow)
                self.is_field_side_left = bool(
                    getattr(resp, "is_field_side_left", False)
                )
                self._last_config = resp
                if changed:
                    self.get_logger().info(
                        f"GameConfig updated: is_team_color_yellow={self.is_team_color_yellow}, is_field_side_left={self.is_field_side_left}"
                    )
            except Exception as e:
                self.get_logger().error(f"Failed to call get_game_config service: {e}")

        future.add_done_callback(done)

    def timer_callback(self):
        if not self.control_references:
            return

        # config is polled periodically in _poll_game_config

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        team_cmd = TeamCommand()
        team_cmd.is_team_color_yellow = self.is_team_color_yellow
        team_cmd.robots = []

        for rid, ref in self.control_references.items():
            if rid not in self.ally_robots:
                continue

            cur = self.ally_robots[rid]
            cur_state = State(
                Vector2D(cur.position_x / 1000.0, cur.position_y / 1000.0),
                Vector2D(cur.velocity_x / 1000.0, cur.velocity_y / 1000.0),
            )
            tgt_state = State(
                Vector2D(ref.pos.x / 1000.0, ref.pos.y / 1000.0),
                Vector2D(ref.vel.x / 1000.0, ref.vel.y / 1000.0),
            )

            vel_cmd = self.robot_controller.compute_trajectory_command(
                rid, tgt_state, cur_state, dt
            )

            tgt_orientation = self.target_orientations.get(rid, cur.orientation)

            vel_ang_cmd = self.orientation_controller.compute(
                target=tgt_orientation, current=cur.orientation
            )


            if self.is_halt:
                vel_cmd = Vector2D(0, 0)
                vel_ang_cmd = 0.0

            out = RobotCommand(robot_id=rid)
            out.linear_velocity_x = float(vel_cmd.x)
            out.linear_velocity_y = float(vel_cmd.y)
            out.angular_velocity = float(vel_ang_cmd)
            out.orientation = cur.orientation
            out.kick = float(self.kick_cache.get(rid, 0.0))

            team_cmd.robots.append(out)

        active = set(self.control_references.keys())
        self.robot_controller.cleanup_unused_robots(active)

        self.publisher.publish(team_cmd)

    def update_parameters(self, req, resp):
        self.robot_controller.update_params(req.kp, req.ki, req.kd)
        resp.success = True
        return resp

    def set_orientation_callback(self, req, resp):
        self.target_orientations[req.robot_id] = req.orientation
        resp.success = True
        return resp

    def update_kp_angular_callback(self, req, resp):
        if req.kp >= 0:
            self.orientation_controller.kp = req.kp
            resp.success = True
        else:
            resp.success = False
        return resp

    def update_kick_callback(self, req, resp):
        robot_id = int(req.id)
        kick = float(req.kick)

        self.kick_cache[robot_id] = kick

        resp.success = True
        return resp

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        self.referee_command = msg.referee.command
        self.is_halt = self.referee_command in self._desired_states


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
