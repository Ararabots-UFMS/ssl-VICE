import rclpy
from rclpy.node import Node

from movement.entities.motion import MotionState

from utils.math_util import Vector2D

from control.p_controller import PController
from control.pid_controller import RobotTrajectoryController
from control.cbf_osqp_core import CBFOsqpCore
from system_interfaces.msg import GameState, RobotCommand, TeamCommand
from system_interfaces.srv import (
    ControlParams,
    GetGameConfig,
    SetKp,
    SetOrientation,
    UpdateCbfParams,
    UpdateKick,
)
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg


# Cap on how far a measurement is carried forward. Beyond this vision has stopped
# arriving, and extrapolating a stale pose is worse than acting on it as it stands.
MAX_MEASUREMENT_AGE = 0.2


class Controller(Node):
    """Simplified controller node.

    Consumes high-frequency GameState and pmand messages, publishes TeamCommand.
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
        self.enemy_robots = {}
        self.vision_wall_stamp = 0.0
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

        # CBF safety filter (ASIF)
        self.gamma_field = 4.0
        self.gamma_prohibited = 4.0
        self.gamma_robot = 4.0
        self.d_min = 0.20
        self.robot_margin = 0.10
        self.rho = 100.0

        self.field_half_length = 4.5
        self.field_half_width = 3.0
        self.n_robots_max = self.declare_parameter("n_robots_max", 16).value

        self.prohibited_zones = [
            (-4.5, -3.5, -1.0, 1.0),
        ]

        self._cbf_core = CBFOsqpCore(
            gamma_field=self.gamma_field,
            gamma_prohibited=self.gamma_prohibited,
            gamma_robot=self.gamma_robot,
            d_min=self.d_min,
            robot_margin=self.robot_margin,
            rho=self.rho,
            field_half_length=self.field_half_length,
            field_half_width=self.field_half_width,
            prohibited_zones=self.prohibited_zones,
            n_robots_max=self.n_robots_max,
        )

        self.create_service(UpdateCbfParams, "update_cbf_params", self.update_cbf_params_callback)

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

    def update_cbf_params_callback(self, req, resp):
        self.gamma_field = req.gamma_field
        self.gamma_prohibited = req.gamma_prohibited
        self.gamma_robot = req.gamma_robot
        self.d_min = req.d_min
        self.robot_margin = req.robot_margin
        self.rho = req.rho

        self._cbf_core.update_params(
            gamma_field=self.gamma_field,
            gamma_prohibited=self.gamma_prohibited,
            gamma_robot=self.gamma_robot,
            d_min=self.d_min,
            robot_margin=self.robot_margin,
            rho=self.rho,
        )

        resp.success = True
        return resp

    def _other_robots(self, self_id):
        other_robots = []
        for rid, r in self.ally_robots.items():
            if rid == self_id:
                continue
            other_robots.append((
                r.position_x / 1000.0, r.position_y / 1000.0,
                r.velocity_x / 1000.0, r.velocity_y / 1000.0,
            ))
        for r in self.enemy_robots.values():
            other_robots.append((
                r.position_x / 1000.0, r.position_y / 1000.0,
                r.velocity_x / 1000.0, r.velocity_y / 1000.0,
            ))
        return other_robots

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

        # The reference is for right now, the measurement is from whenever vision last
        # saw the robot. Comparing them directly reports the travel in between as
        # position error, which always points forward and so always asks for more speed.
        measurement_age = (now.nanoseconds / 1e9) - self.vision_wall_stamp
        measurement_age = min(max(measurement_age, 0.0), MAX_MEASUREMENT_AGE)

        team_cmd = TeamCommand()
        team_cmd.is_team_color_yellow = self.is_team_color_yellow
        team_cmd.robots = []

        for rid, ref in self.control_references.items():
            if rid not in self.ally_robots:
                continue

            cur = self.ally_robots[rid]
            cur_state = MotionState(
                Vector2D(
                    (cur.position_x + cur.velocity_x * measurement_age) / 1000.0,
                    (cur.position_y + cur.velocity_y * measurement_age) / 1000.0,
                ),
                Vector2D(cur.velocity_x / 1000.0, cur.velocity_y / 1000.0),
            )
            tgt_state = MotionState(
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
                safe_vel = Vector2D(0.0, 0.0)
                vel_ang_cmd = 0.0
            else:
                other_robots = self._other_robots(rid)
                safe_x, safe_y = self._cbf_core.solve(
                    cur_state.position.x, cur_state.position.y,
                    vel_cmd.x, vel_cmd.y,
                    other_robots,
                )
                safe_vel = Vector2D(safe_x, safe_y)

            out = RobotCommand(robot_id=rid)
            out.linear_velocity_x = float(safe_vel.x)
            out.linear_velociity_y = float(safe_vel.y)
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
        self.enemy_robots = {r.id: r for r in msg.enemy_robots}
        self.vision_wall_stamp = msg.vision_wall_stamp
        self.referee_command = msg.referee.command
        self.is_halt = self.referee_command in self._desired_states


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
