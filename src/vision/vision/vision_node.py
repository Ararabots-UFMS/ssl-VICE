from vision.vision_client import Client
from vision.tracker import ObjectTracker
from vision.world_message import wrap_geo_message, wrap_message

from typing import Optional

import rclpy
from rclpy.node import Node

from google.protobuf import text_format
from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from vision.proto.messages_robocup_ssl_geometry_pb2 import SSL_GeometryData

from system_interfaces.msg import VisionMessage, VisionGeometry


class Vision(Node):
    """VICE Vision Node, connects and receives data from ssl-vision"""

    def __init__(self):
        super().__init__("visionNode")

        # Parameters settings.
        self.declare_parameter("ip", "224.5.23.2")
        # Visão real
        #self.declare_parameter("port", 10006)
        # Grsim
        self.declare_parameter('port', 10020)
        self.declare_parameter("verbose", False)
        # Optional local interface IP to bind/join multicast (e.g., 192.168.0.10). If empty uses INADDR_ANY.
        self.declare_parameter("interface_ip", "")
        # Socket read timeout (0 => non-blocking) in seconds.
        self.declare_parameter("socket_timeout", 0.0)
        self.declare_parameter("num_cams", 4)
        self.declare_parameter("max_frame_skipped", 30)
        # Verbose prints in terminal all received data.
        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value
        self.interface_ip = (
            self.get_parameter("interface_ip").get_parameter_value().string_value
        )
        self.socket_timeout = (
            self.get_parameter("socket_timeout").get_parameter_value().double_value
        )
        self.num_cams = (
            self.get_parameter("num_cams").get_parameter_value().integer_value
        )
        self.max_frame_skipped = (
            self.get_parameter("max_frame_skipped").get_parameter_value().integer_value
        )

        self.client = Client(
            ip=self.ip,
            port=self.port,
            interface_ip=self.interface_ip if self.interface_ip else None,
            timeout=self.socket_timeout,
        )

        self.get_logger().info(f"Binding client on {self.ip}:{self.port}")
        self.client.connect()

        # Setting ROS publisher.
        # TODO: Find optimal queue size...
        self.publisher = self.create_publisher(VisionMessage, "visionTopic", 10)
        self.geometry_publisher = self.create_publisher(
            VisionGeometry, "geometryTopic", 10
        )

        self.tracker = ObjectTracker(max_frame_skipped=self.max_frame_skipped)

        # TODO: Find the optimal timer.
        self.unify_timer = self.create_timer(0.016, self.publish_vision)
        self.tracker_timer = self.create_timer(0.001, self.update_tracker)

    def update_tracker(self):
        try:
            # Orientation does not have a proper processing. Using raw orientantion and setting orientation velocity to 0.
            data = self.client.receive()

            if data is None:
                return  # no packet available now

            self.tracker.update(data)

            if data.HasField("geometry"):
                self.publish_geometry(data.geometry)

            if self.verbose:
                self.get_logger().info(text_format.MessageToString(data))

        except KeyboardInterrupt:
            self.get_logger().info(
                "Process finished successfully by user, terminating now..."
            )
        except Exception as exception:
            self.get_logger().warning(f"An unexpected error occurred: {exception}")

    def set_filter_param(
        self,
        x_sd: Optional[float] = None,
        y_sd: Optional[float] = None,
        a_sd: Optional[float] = None,
        u_x: Optional[float] = None,
        u_y: Optional[float] = None,
        u_a: Optional[float] = None,
        acceleration_sd_2d: Optional[float] = None,
        acceleration_sd_1d: Optional[float] = None,
    ):
        
        for object_ in self.tracker.objects:
            object_.KF.set_param(x_sd, y_sd, u_x, u_y, acceleration_sd_2d)
            if not object_.id.is_ball:
                object_.orientation_KF.set_param(a_sd, u_a, acceleration_sd_1d)

    def publish_vision(self):
        message = wrap_message(self.tracker.objects)

        # Validate message before publishing (catch garbage/overflow)
        if not self._is_valid_vision_message(message):
            self.get_logger().warn("Skipping invalid vision message")
            return

        if self.context.ok():
            self.publisher.publish(message)

    def _is_valid_vision_message(self, message: VisionMessage) -> bool:
        """Check if vision message contains reasonable data"""
        max_reasonable_pos = 10000.0  # 10m in mm
        
        for robot in list(message.yellow_robots) + list(message.blue_robots):
            if (
                abs(robot.position_x) > max_reasonable_pos
                or abs(robot.position_y) > max_reasonable_pos
            ):
                self.get_logger().warn(
                    f"Invalid robot position detected: ({robot.position_x}, {robot.position_y})"
                )
                return False
        
        for ball in message.balls:
            if abs(ball.position_x) > max_reasonable_pos or abs(ball.position_y) > max_reasonable_pos:
                self.get_logger().warn(f"Invalid ball position: ({ball.position_x}, {ball.position_y})")
                return False
        
        return True

    def publish_geometry(self, message: SSL_GeometryData):
        message: VisionGeometry = wrap_geo_message(message)

        if self.context.ok():
            self.geometry_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
