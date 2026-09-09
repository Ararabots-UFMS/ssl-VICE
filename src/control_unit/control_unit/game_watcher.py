from threading import Lock

import rclpy
from rclpy.node import Node

from system_interfaces.msg import (
    Balls,
    GameState,
    GUIMessage,
    RefereeMessage,
    VisionGeometry,
    VisionMessage,
)
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
        # Stamps of the vision packet the robot/ball estimates came from.
        self.vision_capture_stamp = 0.0
        self.vision_wall_stamp = 0.0
        self.is_team_color_yellow = None
        self.is_field_side_left = None
        self.on_positive_half = None

        # Track variability of on_positive_half (sticky-true if it ever toggles)
        self._on_pos_last_value = None
        self._on_pos_has_varied = False

        # Subscribers
        self.referee_subscriber = self.create_subscription(
            RefereeMessage, "refereeTopic", self.update_from_gamecontroller, 10
        )

        self.geometry_subscriber = self.create_subscription(
            VisionGeometry, "geometryTopic", self.update_from_geometry, 10
        )

        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.update_from_vision, 10
        )

        # Services
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
            # Só roda após sabermos nossa cor pelo referee
            # if self.is_team_color_yellow is None:
            #     return
            if message is None:
                return
            if self.is_team_color_yellow:
                self.ally_robots = {ally.id: ally for ally in message.yellow_robots}
                self.enemy_robots = {enemy.id: enemy for enemy in message.blue_robots}
            else:
                self.ally_robots = {ally.id: ally for ally in message.blue_robots}
                self.enemy_robots = {enemy.id: enemy for enemy in message.yellow_robots}

            if message.balls:
                self.balls = message.balls

            self.vision_capture_stamp = message.capture_stamp
            self.vision_wall_stamp = message.wall_stamp

        self._dirty = True

    def update_from_gamecontroller(self, message: RefereeMessage):
        with self._lock:
            try:
                self.referee = message
            except Exception:
                # fallback mínimo
                try:
                    self.referee.command = getattr(message, "command", "")
                except Exception:
                    pass

        for team in message.teams:
            if team.name == "Ararabots":
                with self._lock:
                    self.is_team_color_yellow = bool(team.color == "yellow")
                    self.is_field_side_left = not bool(team.on_positive_half)
                    self.on_positive_half = bool(team.on_positive_half)

    def update_from_geometry(self, message: VisionGeometry):
        with self._lock:
            self.geometry = message
        self._dirty = True

    def update_referee_no_command(self, message):
        with self._lock:
            self.referee.command = message

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

    def handle_get_game_config(self, request, response):
        with self._lock:
            response.is_team_color_yellow = bool(self.is_team_color_yellow)
            response.is_field_side_left = bool(self.is_field_side_left)
            response.on_positive_half = bool(self.on_positive_half)

            cmd = str(getattr(getattr(self, "referee", None), "command", "")).upper()
            response.is_play_pressed = cmd == "NORMAL_START" or cmd == "FORCE_START"

            try:
                response.robot_count = int(len(self.ally_robots))
            except Exception:
                response.robot_count = 3

        return response

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
            msg.vision_capture_stamp = self.vision_capture_stamp
            msg.vision_wall_stamp = self.vision_wall_stamp
        self.game_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GameWatcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
