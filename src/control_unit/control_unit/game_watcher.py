from rclpy.node import Node
from system_interfaces.msg import VisionMessage, GUIMessage, RefereeMessage, VisionGeometry
from strategy.blackboard import Blackboard

import rclpy
from std_srvs.srv import SetBool


class GameWatcher(Node):
    def __init__(self) -> None:
        super().__init__("game_watcher")
        self.blackboard = Blackboard()

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

        # Service to set team color (True => yellow, False => blue)
        self.set_team_color_srv = self.create_service(
            SetBool, 'set_team_color', self.handle_set_team_color
        )

    def update_from_vision(self, message: VisionMessage):
        self.blackboard.update_from_vision_message(message)

    def update_from_gamecontroller(self, message: RefereeMessage):
        self.blackboard.update_from_gamecontroller_message(message)

    def update_from_gui(self, message: GUIMessage):
        self.blackboard.update_from_gui_message(message)

    def update_from_geometry(self, message: VisionGeometry):
        self.blackboard.update_from_geometry(message)

    def handle_set_team_color(self, request, response):
        """Service handler for SetBool. Sets blackboard.gui.is_team_color_yellow."""
        try:
            # request.data is a bool; True -> yellow
            self.get_logger().info(f"Setting team color is_team_color_yellow={request.data}")
            self.blackboard.gui.is_team_color_yellow = bool(request.data)
            response.success = True
            response.message = "Team color updated"
        except Exception as e:
            self.get_logger().error(f"Failed to set team color: {e}")
            response.success = False
            response.message = str(e)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GameWatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()