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

        # Service to set team color
        self.set_team_color_srv = self.create_service(
            SetBool, "set_team_color", self.handle_set_team_color
        )

        # Aggregated game state publisher
        self.game_state_pub = self.create_publisher(GameState, "game_state", 10)

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
        if self._vision_count % 30 == 0:
            self.get_logger().debug(
                f"[VISION] msgs={self._vision_count} allies={len(self.ally_robots)} enemies={len(self.enemy_robots)} balls={len(self.balls)} team_yellow={self.gui.is_team_color_yellow}"
            )
        self._publish_state()

    def update_from_gamecontroller(self, message: RefereeMessage):
        with self._lock:
            self.referee_last_command = self.referee
            self.referee = message
            self._referee_count += 1
        if self._referee_count % 10 == 0:
            self.get_logger().debug(
                f"[REFEREE] msgs={self._referee_count} command={self.referee.command} last={self.referee_last_command.command}"
            )
        self._publish_state()

    def update_from_gui(self, message: GUIMessage):
        with self._lock:
            self.gui = message
            self._gui_count += 1
        if self._gui_count % 10 == 0:
            self.get_logger().debug(
                f"[GUI] msgs={self._gui_count} team_yellow={self.gui.is_team_color_yellow} field_left={self.gui.is_field_side_left}"
            )
        self._publish_state()

    def update_from_geometry(self, message: VisionGeometry):
        with self._lock:
            self.geometry = message
            self._geometry_count += 1
        if self._geometry_count % 5 == 0:
            try:
                lines = len(self.geometry.field_lines)
            except Exception:
                lines = -1
            self.get_logger().debug(
                f"[GEOMETRY] msgs={self._geometry_count} field_lines={lines}"
            )
        self._publish_state()

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
        self.get_logger().debug("[KICK] ativado")

    def desactivate_kick(self):
        with self._lock:
            self.can_i_kick = 0.0
        self.get_logger().debug("[KICK] desativado")

    def handle_set_team_color(self, request, response):
        try:
            self.get_logger().info(
                f"Setting team color is_team_color_yellow={request.data}"
            )
            with self._lock:
                self.gui.is_team_color_yellow = bool(request.data)
            response.success = True
            response.message = "Team color updated"
        except Exception as e:
            self.get_logger().error(f"Failed to set team color: {e}")
            response.success = False
            response.message = str(e)
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

    def _publish_state(self):
        """Publica o estado agregado em um único tópico."""
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

    # Getters
    def get_ally_robots(self):
        with self._lock:
            return self.ally_robots.copy()

    def get_enemy_robots(self):
        with self._lock:
            return self.enemy_robots.copy()

    def get_balls(self):
        with self._lock:
            return self.balls.copy()

    def get_gui(self):
        with self._lock:
            return self.gui

    def get_referee(self):
        with self._lock:
            return self.referee

    def get_referee_last_command(self):
        with self._lock:
            return self.referee_last_command

    def get_can_i_start(self):
        with self._lock:
            return self.can_i_start

    def get_geometry(self):
        with self._lock:
            return self.geometry

    def get_can_i_kick(self):
        with self._lock:
            return self.can_i_kick


def main(args=None):
    rclpy.init(args=args)
    node = GameWatcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
