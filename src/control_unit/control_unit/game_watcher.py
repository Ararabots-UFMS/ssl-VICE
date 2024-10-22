from rclpy.node import Node

from system_interfaces.msg import VisionMessage, GUIMessage, GameData, VisionGeometry
from strategy.blackboard import Blackboard


class GameWatcher(Node):
    def __init__(self) -> None:
        super().__init__("game_watcher")
        self.blackboard = Blackboard()

        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.update_from_vision, 10
        )

        self.gui_subscriber = self.create_subscription(
            GUIMessage, "guiTopic", self.update_from_gui, 10
        )

        self.referee_subscriber = self.create_subscription(
            GameData, "/referee_messages", self.update_from_gamecontroller, 10
        )

        self.geometry_subscriber = self.create_subscription(
            VisionGeometry, "geometryTopic", self.update_from_geometry, 10
        )

    def update_from_vision(self, message: VisionMessage):
        self.blackboard.update_from_vision_message(message)

    def update_from_gamecontroller(self, message: GameData):
        self.blackboard.update_from_gamecontroller_message(message)

    def update_from_gui(self, message: GUIMessage):
        self.blackboard.update_from_gui_message(message)

    def update_from_geometry(self, message: VisionGeometry):
        self.blackboard.update_from_geometry(message)
