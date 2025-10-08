from rclpy.node import Node
from system_interfaces.msg import (
    VisionMessage,
    GUIMessage,
    RefereeMessage,
    VisionGeometry,
    Balls,
    GameState,
)
from threading import Lock

import rclpy
from std_srvs.srv import SetBool
from system_interfaces.srv import GetGameConfig


class GameWatcher(Node):
    def __init__(self) -> None:
        super().__init__("game_watcher")

        # Thread-safe data storage
        self._lock = Lock()

        # Game state data
        self.ally_robots = {}
        self.enemy_robots = {}
        self.balls = [Balls()]
        self.gui = GUIMessage()
        self.referee = RefereeMessage()
        self.referee_last_command = RefereeMessage()
        self.can_i_start = False
        self.geometry = VisionGeometry()
        self.can_i_kick = 0.0

        # Debug counters
        self._vision_count = 0
        self._gui_count = 0
        self._referee_count = 0
        self._geometry_count = 0

        # Subscribers
        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.update_from_vision, 10
        )
        self.gui_subscriber = self.create_subscription(
            GUIMessage, "guiTopic", self.update_from_gui, 10
        )
        self.referee_subscriber = self.create_subscription(
            RefereeMessage, "refereeTopic", self.update_from_gamecontroller, 10
        )
        self.geometry_subscriber = self.create_subscription(
            VisionGeometry, "geometryTopic", self.update_from_geometry, 10
        )

        # Services low-frequency
        self.set_team_color_srv = self.create_service(
            SetBool, "set_team_color", self.handle_set_team_color
        )
        self.get_game_config_srv = self.create_service(
            GetGameConfig, "get_game_config", self.handle_get_game_config
        )

        # Aggregated game state publisher
        self.game_state_pub = self.create_publisher(GameState, "game_state", 10)

        # Dirty flag to track state changes
        self._dirty = True
        self._last_publish_seq = 0

        # Fixed publication timer (60 Hz ~ 16.66ms)
        self._publish_timer = self.create_timer(1.0 / 60.0, self._timer_publish_state)

    def update_from_vision(self, message: VisionMessage):
        with self._lock:
            if message is None:
                return
            if self.gui.is_team_color_yellow:
                self.ally_robots = {ally.id: ally for ally in message.yellow_robots}
                self.enemy_robots = {enemy.id: enemy for enemy in message.blue_robots}
            else:
                self.ally_robots = {ally.id: ally for ally in message.blue_robots}
                self.enemy_robots = {enemy.id: enemy for enemy in message.yellow_robots}

            if message.balls:
                self.balls = message.balls
            self._vision_count += 1
        self._dirty = True
        if self._vision_count % 30 == 0:
            self.get_logger().debug(
                f"[VISION] msgs={self._vision_count} allies={len(self.ally_robots)} enemies={len(self.enemy_robots)} balls={len(self.balls)} team_yellow={self.gui.is_team_color_yellow}"
            )

    def update_from_gamecontroller(self, message: RefereeMessage):
        with self._lock:
            self.referee_last_command = self.referee
            self.referee = message
            self._referee_count += 1
        self._dirty = True
        if self._referee_count % 10 == 0:
            self.get_logger().debug(
                f"[REFEREE] msgs={self._referee_count} command={self.referee.command} last={self.referee_last_command.command}"
            )

    def update_from_gui(self, message: GUIMessage):
        with self._lock:
            self.gui = message
            self._gui_count += 1
        self._dirty = True
        if self._gui_count % 10 == 0:
            self.get_logger().debug(
                f"[GUI] msgs={self._gui_count} team_yellow={self.gui.is_team_color_yellow} field_left={self.gui.is_field_side_left}"
            )

    def update_from_geometry(self, message: VisionGeometry):
        with self._lock:
            self.geometry = message
            self._geometry_count += 1
        self._dirty = True
        if self._geometry_count % 5 == 0:
            try:
                lines = len(self.geometry.field_lines)
            except Exception:
                lines = -1
            self.get_logger().debug(
                f"[GEOMETRY] msgs={self._geometry_count} field_lines={lines}"
            )

    def update_referee_no_command(self, message):
        with self._lock:
            self.referee.command = message

    def update_referee_start(self):
        with self._lock:
            self.can_i_start = True

    def update_referee_not_start(self):
        with self._lock:
            self.can_i_start = False

    def activate_kick(self):
        with self._lock:
            self.can_i_kick = 1.0
            self._dirty = True
        self.get_logger().debug("[KICK] ativado")

    def deactivate_kick(self):
        with self._lock:
            self.can_i_kick = 0.0
            self._dirty = True
        self.get_logger().debug("[KICK] desativado")

    def handle_set_team_color(self, request, response):
        try:
            self.get_logger().info(
                f"Setting team color is_team_color_yellow={request.data}"
            )
            with self._lock:
                self.gui.is_team_color_yellow = bool(request.data)
                self._dirty = True
            response.success = True
            response.message = "Team color updated"
        except Exception as e:
            self.get_logger().error(f"Failed to set team color: {e}")
            response.success = False
            response.message = str(e)
        return response

    def handle_get_game_config(self, request, response):  # request unused
        with self._lock:
            response.is_team_color_yellow = self.gui.is_team_color_yellow
            response.is_field_side_left = self.gui.is_field_side_left
            response.is_play_pressed = self.gui.is_play_pressed
            response.robot_count = self.gui.robot_count
        return response

    def _snapshot_log(self):
        """Log periódico do estado (para diagnóstico)."""
        with self._lock:
            allies = len(self.ally_robots)
            enemies = len(self.enemy_robots)
            balls = len(self.balls)
            ref_cmd = self.referee.command
            team = "YELLOW" if self.gui.is_team_color_yellow else "BLUE"
        self.get_logger().info(
            f"[SNAPSHOT] allies={allies} enemies={enemies} balls={balls} referee={ref_cmd} team={team} vision_msgs={self._vision_count}"
        )

    def _timer_publish_state(self):
        if not self._dirty:
            return
        self._publish_state()
        self._dirty = False
        self._last_publish_seq += 1

    def _publish_state(self):
        """Monta e publica GameState (uso interno)."""
        msg = GameState()
        with self._lock:
            msg.ally_robots = list(self.ally_robots.values())
            msg.enemy_robots = list(self.enemy_robots.values())
            msg.balls = self.balls
            msg.gui = self.gui
            msg.referee = self.referee
            msg.referee_last_command = self.referee_last_command
            msg.geometry = self.geometry
            msg.can_i_start = self.can_i_start
            msg.can_i_kick = float(self.can_i_kick)
        self.game_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GameWatcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
