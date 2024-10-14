from rclpy.node import Node

from flask import Flask
from flask_socketio import SocketIO, emit
from threading import Thread
import subprocess

from gui_interpreter.gui_publisher import GUIPublisher
from utils.vision_subscriber import VisionSubscriber
from utils.converter import todict
import rclpy

from system_interfaces.msg import VisionMessage, GUIMessage

app = Flask(__name__)
gui_socket = SocketIO(app, cors_allowed_origins="*")


class APINode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.publisher = self.create_publisher(GUIMessage, "guiTopic", 10)

        self.vision_running = False
        self.vision_subscriber = None

        self.is_field_side_left = False
        self.is_team_color_blue = False
        self.is_play_pressed = False

    def handle_connect(self):
        self.get_logger().info("Client connected")
        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.emit_vision_message, 10
        )

    def handle_disconnect(self):
        self.get_logger().info("Client disconneted")
        self.vision_subscriber = None

    def handle_field_side(self, is_field_side_left):
        self.get_logger().info(f"Is team field side? {is_field_side_left}")
        self.is_field_side_left = is_field_side_left
        self.publish_gui_data()

    def handle_team_color(self, is_team_color_blue):
        self.get_logger().info(f"Is team color blue? {is_team_color_blue}")
        self.is_team_color_blue = is_team_color_blue
        self.publish_gui_data()

    def emit_vision_message(self, msg: VisionMessage) -> None:
        data = todict(msg)
        gui_socket.emit("vision_msg", {"data": data})

    def create_message(self) -> GUIMessage:
        msg = GUIMessage()
        msg.is_field_side_left = self.is_field_side_left
        msg.is_team_color_blue = self.is_team_color_blue
        msg.is_play_pressed = self.is_play_pressed
        return msg

    def publish_gui_data(self) -> None:
        message = self.create_message()
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = APINode("api_node")
    gui_socket.on_event("connect", node.handle_connect, namespace="")
    gui_socket.on_event("disconnect", node.handle_disconnect, namespace="")
    gui_socket.on_event("fieldSide", node.handle_field_side, namespace="")
    gui_socket.on_event("teamColor", node.handle_team_color, namespace="")
    Thread(target=gui_socket.run, args=(app,)).start()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
