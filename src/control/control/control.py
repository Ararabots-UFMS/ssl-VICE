import rclpy
from rclpy.node import Node

from new_movement.entities.States import State, Vector2D
from control.p_controller import PController
from control.pid_controller import RobotTrajectoryController

from system_interfaces.msg import ControlCommand, RobotCommand, TeamCommand, GameState
from system_interfaces.srv import ControlParams, SetKp, SetOrientation, GetGameConfig


class Controller(Node):
    """Simplified controller node.

    Consumes high-frequency GameState and ControlCommand messages, publishes TeamCommand.
    Fetches low-frequency configuration (team color) once via GetGameConfig service.
    """

    def __init__(self):
        super().__init__("controller")

        # Caches
        self.ally_robots = {}
        self.latest_command = None

        # Low-frequency config
        self.is_team_color_yellow = False
        self._config_client = self.create_client(GetGameConfig, "get_game_config")
        self._config_requested = False

        # Controllers
        self.robot_controller = RobotTrajectoryController()
        self.orientation_controller = PController(kp=2.5, max_output=4.0)
        self.target_orientations = {}

        # ROS Interfaces
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.create_subscription(
            ControlCommand, "control_command", self.receive_command, 10
        )
        self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)
        self.create_service(ControlParams, "update_pid", self.update_parameters)
        self.create_service(
            SetOrientation, "set_orientation", self.set_orientation_callback
        )
        self.create_service(SetKp, "update_kp_angular", self.update_kp_angular_callback)

        # Timing
        self.last_time = self.get_clock().now()
        self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}

    def receive_command(self, msg: ControlCommand):
        self.latest_command = msg

    def _request_config_once(self):
        if self._config_requested:
            return
        if not self._config_client.service_is_ready():
            return
        future = self._config_client.call_async(GetGameConfig.Request())

        def done(fut):
            try:
                resp = fut.result()
                self.is_team_color_yellow = resp.is_team_color_yellow
            except Exception:
                pass

        future.add_done_callback(done)
        self._config_requested = True

    def timer_callback(self):
        if self.latest_command is None:
            return

        self._request_config_once()

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        team_cmd = TeamCommand()
        team_cmd.is_team_color_yellow = self.is_team_color_yellow
        team_cmd.robots = []

        for desired in self.latest_command.command:
            rid = desired.id
            if rid not in self.ally_robots:
                continue

            cur = self.ally_robots[rid]
            cur_state = State(
                Vector2D(cur.position_x / 1000.0, cur.position_y / 1000.0),
                Vector2D(cur.velocity_x / 1000.0, cur.velocity_y / 1000.0),
            )
            tgt_state = State(
                Vector2D(desired.position_x, desired.position_y),
                Vector2D(desired.velocity_x, desired.velocity_y),
            )

            vel_cmd = self.robot_controller.compute_trajectory_command(
                rid, tgt_state, cur_state, dt
            )

            out = RobotCommand(robot_id=rid)
            out.linear_velocity_x = float(vel_cmd.x)
            out.linear_velocity_y = float(vel_cmd.y)
            target_orientation = self.target_orientations.get(rid, cur.orientation)
            out.angular_velocity = float(
                self.orientation_controller.compute(
                    target=target_orientation, current=cur.orientation
                )
            )
            out.orientation = cur.orientation
            out.kick = 0.0
            team_cmd.robots.append(out)

        active = {d.id for d in self.latest_command.command}
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


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
