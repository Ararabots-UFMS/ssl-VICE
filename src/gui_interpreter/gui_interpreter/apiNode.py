import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from flask import Flask
from flask_socketio import SocketIO, emit
import threading
from threading import Thread
import ctypes

from utils.converter import todict

from grsim_messenger.grsim_publisher import grSimPublisher
from hardware_messenger.hardware_publisher import HardwarePublisher

from system_interfaces.msg import VisionMessage, GUIMessage, GUIRobot, TrajectoryMessage
from vision.vision_node import Vision
from referee.referee_node import RefereeNode

app = Flask(__name__)
gui_socket = SocketIO(app, cors_allowed_origins="*")

vision_running = threading.Event()

communication_running = threading.Event()

referee_running = threading.Event()


class thread_with_exception(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.socket = socket

    def run(self):

        # target function of the thread class
        try:
            self.socket.run(app, allow_unsafe_werkzeug=True)
        finally:
            print("ended")

    def get_id(self):

        # returns id of the respective thread
        if hasattr(self, "_thread_id"):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            thread_id, ctypes.py_object(SystemExit)
        )
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print("Exception raise failure")


class APINode(Node):
    def __init__(
        self, name, executor, vision_event, communication_event, referee_event
    ):
        super().__init__(name)

        self.publisher = self.create_publisher(GUIMessage, "guiTopic", 10)

        self.executor = executor

        self.vision_running = vision_event
        self.vision_subscriber = None
        self.vision_node = Vision()
        
        self.trajectory_subscriber = None

        self.communication_running = communication_event
        self.communication_node = grSimPublisher()

        self.referee_running = referee_event
        self.referee_node = RefereeNode()

        self.robots = []
        self.robot_count = 0

        self.is_field_side_right = True
        self.is_team_color_yellow = True
        self.is_play_pressed = False

        self.get_logger().info("API Node started")

        self.create_timer(0.5, self.publish_gui_data)

    def handle_connect(self):
        self.get_logger().info("Client connected")
        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.emit_vision_message, 10
        )
        self.trajectory_subscriber = self.create_subscription(
            TrajectoryMessage, "trajectory_topic", self.emit_trajectory_message, 10
        )
        gui_socket.emit("visionStatus", {"status": self.vision_running.is_set()})
    
    def emit_vision_message(self, msg: VisionMessage) -> None:
        data = todict(msg)

        gui_socket.emit("vision_update", {
            "yellow": data["yellow_robots"],
            "blue":   data["blue_robots"],
            "balls":  data["balls"],
        })

    def emit_trajectory_message(self, msg: TrajectoryMessage) -> None:
        """Trajectory emission to GUI"""
        trajectories_data = []
        
        for robot_trajectory in msg.trajectories:
            robot_data = {
                "robot_id": robot_trajectory.robot_id,
                "total_duration": robot_trajectory.total_duration,
                "points": []
            }
            
            for point in robot_trajectory.points:
                point_data = {
                    "x": point.x,
                    "y": point.y,
                    "velocity_x": point.velocity_x,
                    "velocity_y": point.velocity_y,
                    "timestamp": point.timestamp
                }
                robot_data["points"].append(point_data)
            
            trajectories_data.append(robot_data)
        
        gui_socket.emit("trajectory_update", {
            "trajectories": trajectories_data
        })

    def handle_disconnect(self):
        self.get_logger().info("Client disconneted")
        self.vision_subscriber = None
        self.trajectory_subscriber = None

    def handle_field_side(self, is_field_side_right):
        # self.get_logger().info(f"Is team field side left? {is_field_side_left}")
        is_field_side_right
    
        self.get_logger().info(f"Is team field side left? {is_field_side_right}")

        self.is_field_side_right = is_field_side_right

    def handle_team_color(self, is_team_color_yellow):
        # self.get_logger().info(f"Is team color blue? {is_team_color_blue}")

        self.get_logger().info(f"Is team color blue? {is_team_color_yellow}")

        self.is_team_color_yellow = is_team_color_yellow

    def handle_simulation(self, is_simulation):
        self.get_logger().info(f"Is sumulation? {is_simulation}")
        self.is_simulation = is_simulation
        try:
            self.executor.remove_node(self.communication_node)
        except:
            pass
        self.communication_node.destroy_node()
        if is_simulation:
            self.communication_node = grSimPublisher()
        else:
            self.communication_node = HardwarePublisher()

    def handle_vision_button(self):
        if self.vision_running.is_set():
            self.vision_running.clear()

            self.executor.remove_node(self.vision_node)

            gui_socket.emit("visionOutput", {"line": "Vision node stopped"})
            gui_socket.emit("visionStatus", {"status": self.vision_running.is_set()})
            self.get_logger().info("Vision node stopped")
        else:
            self.vision_running.set()

            gui_socket.emit("visionOutput", {"line": "Starting vision node"})
            gui_socket.emit("visionStatus", {"status": self.vision_running.is_set()})
            self.get_logger().info("Starting vision node")

            self.executor.add_node(self.vision_node)

    def handle_communication_button(self):
        if self.communication_running.is_set():
            self.communication_running.clear()
            self.executor.remove_node(self.communication_node)
            gui_socket.emit(
                "communicationOutput", {"line": "Communication node stopped"}
            )
            gui_socket.emit(
                "communicationStatus", {"status": self.communication_running.is_set()}
            )
            self.get_logger().info("Communication node stopped")
        else:
            self.communication_running.set()
            gui_socket.emit(
                "communicationOutput", {"line": "Starting communication node"}
            )
            gui_socket.emit(
                "communicationStatus", {"status": self.communication_running.is_set()}
            )
            self.get_logger().info("Starting communication node")
            self.executor.add_node(self.communication_node)

    def handle_referee_button(self):
        if self.referee_running.is_set():
            self.referee_running.clear()
            self.executor.remove_node(self.referee_node)
            gui_socket.emit("refereeOutput", {"line": "Referee node stopped"})
            gui_socket.emit("refereeStatus", {"status": self.referee_running.is_set()})
            self.get_logger().info("Referee node stopped")
        else:
            self.referee_running.set()
            gui_socket.emit("referee", {"line": "Starting referee node"})
            gui_socket.emit("refereeStatus", {"status": self.referee_running.is_set()})
            self.get_logger().info("Starting referee node")
            self.executor.add_node(self.referee_node)

    def create_message(self) -> GUIMessage:
        msg = GUIMessage()
        msg.is_field_side_left = not self.is_field_side_right
        msg.is_team_color_yellow = self.is_team_color_yellow
        msg.is_play_pressed = self.is_play_pressed
        for gui_robot in self.robots:
            robot = GUIRobot()
            robot.id = gui_robot["id"]
            robot.name = gui_robot["name"]
            robot.address = [int(i) for i in gui_robot["address"].split(",")]
            robot.kp = float(gui_robot["kp"])
            robot.ki = float(gui_robot["ki"])
            robot.kd = float(gui_robot["kd"])

            msg.robots.append(robot)
        msg.robot_count = self.robot_count
        return msg

    def publish_gui_data(self) -> None:
        message = self.create_message()
        self.publisher.publish(message)

    def handle_config_button(self, msg):
        self.get_logger().info("Configuration saved")
        self.robot_count = len(msg)
        self.robots = msg
        self.publish_gui_data()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    node = APINode(
        "api_node", executor, vision_running, communication_running, referee_running
    )
    gui_socket.on_event("connect", node.handle_connect, namespace="")
    gui_socket.on_event("disconnect", node.handle_disconnect, namespace="")
    gui_socket.on_event("fieldSide", node.handle_field_side, namespace="")
    gui_socket.on_event("teamColor", node.handle_team_color, namespace="")
    gui_socket.on_event("fieldMode", node.handle_simulation, namespace="")
    gui_socket.on_event("visionButton", node.handle_vision_button, namespace="")
    gui_socket.on_event(
        "communicationButton", node.handle_communication_button, namespace=""
    )
    gui_socket.on_event("refereeButton", node.handle_referee_button, namespace="")
    gui_socket.on_event("configSaveButton", node.handle_config_button, namespace="")
    try:
        thread = thread_with_exception(gui_socket)
        thread.start()
        executor.add_node(node)
        executor.spin()
    except Exception:
        thread.raise_exception()
        rclpy.shutdown()


if __name__ == "__main__":
    main()