import rclpy
from control.cbf_osqp_core import CBFOsqpCore
from rclpy.node import Node
from new_movement.entities.States import State, Vector2D
from system_interfaces.msg import GameState, FilterCommand, TeamCommand, RobotCommand
from system_interfaces.srv import GetGameConfig

class AsifFilter(Node):
        """Simplified ASIF filter node.

        Consumes high-frequency GameState and FilterCommand messages, publishes TeamCommand.
        """

        def __init__(self):
                super().__init__("asif_filter")

                self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

                self._desired_states = ["TIMEOUT_BLUE", "TIMEOUT_YELLOW", "HALT"]
                self.ally_robots = {}
                self.enemy_robots = {}
                self.is_team_color_yellow = False
                self.is_field_side_left = False
                self._config_client = self.create_client(GetGameConfig, "get_game_config")

                self._config_call_inflight = False
                self._last_config = None
                self._latest_command = None

                self.create_timer(0.5, self._poll_game_config)

                self.create_subscription(FilterCommand, "filter_command", self.receive_command, 10)
                self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)

                self.gamma_field = 4.0 #agressiviness close to field border
                self.gamma_prohibited = 4.0 #agressiviness close to penalty_area
                self.gamma_robot = 4.0 #agressivines close to other robot
                self.d_min = 0.20 #min distance between robot centers
                self.robot_margin = 0.10 #robot ray + gap to borders/areas
                self.rho = 100.0 #penalty weight

                self.field_half_length = 4.5
                self.field_half_width = 3.0


                self.prohibited_zones = [
                (-4.5, -3.5, -1.0, 1.0),
                ]

                self._cbf_core = CBFOsqpCore(
                        gamma_field=self.gamma_field,
                        gamma_prohibited=self.gamma_prohibited,
                        robot_margin=self.robot_margin,
                        rho=self.rho,
                        field_half_length=self.field_half_length,
                        field_half_width=self.field_half_width,
                        prohibited_zones=self.prohibited_zones,
                )

                self.last_time = self.get_clock().now()
                self.create_timer(0.01, self.timer_callback)

        def game_state_callback(self, msg: GameState):
                self.ally_robots = {r.id: r for r in msg.ally_robots}
                self.enemy_robots = {r.id: r for r in msg.enemy_robots}
                self.referee_command = msg.referee.command
                self.is_halt = self.referee_command in self._desired_states

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

                                changed = (
                                        self._last_config is None
                                        or self._last_config.is_team_color_yellow
                                        !=resp.is_team_color_yellow
                                        or getattr(self._last_config,"is_field_side_left", None)
                                        != getattr(resp, "is_field_side_left", None)
                                )
                                self.is_team_color_yellow = bool(resp.is_team_color_yellow)
                                self.is_field_side_left = bool(
                                        getattr(resp, "is_field_side_left", False)
                                )
                                self._last_config = resp
                                if changed:
                                        self.get_logger().info(
                                                f"GameConfig updated (filter): is_team_color_yellow={self.is_team_color_yellow}, is_field_side_left={self.is_field_side_left}"
                                        )
                        except Exception as e:
                                self.get_logger().error(f"Failed to call get_game_config service: {e}")

                future.add_done_callback(done)

        def receive_command(self, msg:FilterCommand):
                self._latest_command = msg


        def Filter(self, cur_state, u_des):
                px, py = cur_state.position.x, cur_state.position.y

                safe_x, safe_y = self._cbf_core.solve(px, py, u_des.x, u_des.y)
                return Vector2D(safe_x, safe_y)

        def timer_callback(self):
                if self._latest_command is None:
                        return

                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now

                team_cmd = TeamCommand()
                team_cmd.is_team_color_yellow = self.is_team_color_yellow
                team_cmd.robots = []

                for robot in self._latest_command.robots:
                        rid = robot.robot_id
                        if rid not in self.ally_robots:
                                continue

                        cur = self.ally_robots[rid]
                        cur_state = State(
                                Vector2D(cur.position_x / 1000.0, cur.position_y / 1000.0),
                                Vector2D(cur.velocity_x / 1000.0, cur.velocity_y / 1000.0),
                        )

                        u_des = Vector2D(robot.linear_velocity_x, robot.linear_velocity_y)

                        if self.is_halt:
                                safe_vel = Vector2D(0.0, 0.0)
                        else:
                                safe_vel = self.Filter(cur_state, u_des)

                        out = RobotCommand(robot_id=rid)
                        out.linear_velocity_x = safe_vel.x
                        out.linear_velocity_y = safe_vel.y
                        out.angular_velocity = robot.angular_velocity
                        out.orientation = robot.orientation
                        out.kick = robot.kick

                        team_cmd.robots.append(out)

                self.publisher.publish(team_cmd)

def main(args=None):
        rclpy.init(args=args)
        node = AsifFilter()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
        main()

