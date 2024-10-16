import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from flask import Flask
from flask_socketio import SocketIO, emit
import threading
from threading import Thread

from utils.converter import todict

from grsim_messenger.grsim_publisher import grSimPublisher

from system_interfaces.msg import VisionMessage, GUIMessage
from vision.vision_node import Vision

app = Flask(__name__)
gui_socket = SocketIO(app, cors_allowed_origins="*")

vision_running = threading.Event()

communication_running = threading.Event()


class APINode(Node):
    def __init__(self, name, executor, vision_event, communication_event):
        super().__init__(name)

        self.publisher = self.create_publisher(GUIMessage, "guiTopic", 10)

        self.executor = executor

        self.vision_running = vision_event
        self.vision_subscriber = None
        self.vision_node = Vision()

        self.communication_running = communication_event
        self.communication_node = grSimPublisher()

        self.is_field_side_left = True
        self.is_team_color_yellow = False
        self.is_play_pressed = False

    def handle_connect(self):
        self.get_logger().info("Client connected")
        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.emit_vision_message, 10
        )
        gui_socket.emit("visionStatus", {"status": self.vision_running.is_set()})

    def emit_vision_message(self, msg: VisionMessage) -> None:
        data = todict(msg)
        gui_socket.emit("vision_msg", {"data": data})

    def handle_disconnect(self):
        self.get_logger().info("Client disconneted")
        self.vision_subscriber = None

    def handle_field_side(self, is_field_side_left):
        self.get_logger().info(f"Is team field side left? {is_field_side_left}")
        self.is_field_side_left = is_field_side_left
        self.publish_gui_data()

    def handle_team_color(self, is_team_color_blue):
        self.get_logger().info(f"Is team color blue? {is_team_color_blue}")
        self.is_team_color_blue = is_team_color_blue
        self.publish_gui_data()

    def handler_vision(self):
        self.handle_button(self.vision_running, self.vision_node, module="vision") 

    def handler_communication(self):
        self.handle_button(
            self.communication_running, self.communication_node, module="communication"
        )     

    def handle_referee(self):
        self.handle_button(self.referee_running, self.referee_node, module="referee")

    def handle_button(self, running, node, module):

        if running.is_set():
            running.clear()
            self.executor.remove_node(node)
            gui_socket.emit("output", {"line": f"{module} node stopped"})
            gui_socket.emit("status", {"status": running.is_set()})
            self.get_logger().info("Node stopped")
        else:
            running.set()
            gui_socket.emit("output", {"line": f"Starting {module} node"})
            gui_socket.emit("status", {"status": running.is_set()})
            self.get_logger().info(f"Starting node")
            self.executor.add_node(node)

    def run_vision(self):
        while vision_running.is_set():
            rclpy.spin_once(self.vision_node)

    def create_message(self) -> GUIMessage:
        msg = GUIMessage()
        msg.is_field_side_left = self.is_field_side_left
        msg.is_team_color_yellow = self.is_team_color_yellow
        msg.is_play_pressed = self.is_play_pressed
        return msg

    def publish_gui_data(self) -> None:
        message = self.create_message()
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    node = APINode("api_node", executor, vision_running, communication_running)
    gui_socket.on_event("connect", node.handle_connect, namespace="")
    gui_socket.on_event("disconnect", node.handle_disconnect, namespace="")
    gui_socket.on_event("fieldSide", node.handle_field_side, namespace="")
    gui_socket.on_event("teamColor", node.handle_team_color, namespace="")
    gui_socket.on_event("visionButton", node.handler_vision, namespace="")
    gui_socket.on_event(
        "communicationButton", node.handler_communication, namespace=""
    )
    Thread(
        target=gui_socket.run, args=(app,), kwargs={"allow_unsafe_werkzeug": True}
    ).start()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
