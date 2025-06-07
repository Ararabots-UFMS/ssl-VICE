import rclpy
from rclpy.node import Node
from system_interfaces.msg import RefereeMessage
from referee.referee_client import Client
from referee.proto.ssl_gc_referee_message_pb2 import Referee
from referee.referee_message_wrapper import MessageWrapping


class RefereeNode(Node):
    """ROS2 Node that listens to ssl-game-controller referee multicast messages."""

    def __init__(self):
        super().__init__("refereeNode")
        self._declare_parameters()
        self._setup_client()
        self._setup_publisher()
        self.last_message = RefereeMessage()
        self.get_logger().info(f"Listening for multicast messages on {self.ip}:{self.port}")

    def _declare_parameters(self):
        # In ROS2 Humble declare_parameter return the value directly
        self.ip = self.declare_parameter("ip", "224.5.23.1").value
        self.port = self.declare_parameter("port", 11003).value
        self.buffer_size = self.declare_parameter("buffer_size", 20240).value

    def _setup_client(self):
        self.client = Client(self.ip, self.port, self.buffer_size)
        self.client.connect()

    def _setup_publisher(self):
        self.publisher_ = self.create_publisher(RefereeMessage, "refereeTopic", 10)
        # with 0.01 seconds in timer, referee receive msgs by one second
        self.timer_ = self.create_timer(0.001, self._listen_to_multicast)

    def _listen_to_multicast(self):
        try:
            data = self.client.receive()
            message = self._parse_referee_message(data)
            if message and self.last_message != message:
                self.publisher_.publish(message)
                self.get_logger().info(f"Published new Referee message: {message}")
                self.last_message = message
        except Exception as e:
            self.get_logger().error(f"Error receiving multicast message: {e}")

    def _parse_referee_message(self, data) -> RefereeMessage | None:
        try:
            referee_proto = Referee()
            referee_proto.ParseFromString(data)
            return MessageWrapping(referee_proto).msg
        except Exception as e:
            self.get_logger().warning(f"Failed to parse Protobuf message: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
