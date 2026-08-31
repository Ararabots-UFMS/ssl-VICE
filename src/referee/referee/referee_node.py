from typing import Optional

import traceback

import rclpy
from rclpy.node import Node

from .referee_client import RefereeClient

from .referee_message_wrapper import RefereeMessageWrapper

from referee.proto.ssl_gc_referee_message_pb2 import Referee

from system_interfaces.msg import RefereeMessage


class RefereeNode(Node):
    """Listener node for SSL game-controller referee messages via multicast UDP."""

    def __init__(self):
        super().__init__("refereeNode")

        # Track last seen command to reduce noisy logs
        self._last_command = None
        self._last_command_counter = None
        # ROS publisher for the wrapped referee message
        self.publisher = self.create_publisher(RefereeMessage, "refereeTopic", 10)

        self.declare_params()
        self.get_params()
        self.setup_referee_client()

        # Polling timer to receive messages and publish. Short interval to be responsive.
        self.poll_timer = self.create_timer(0.01, self.receive_and_publish)

    def declare_params(self):
        """Declare ROS parameters for the node."""
        # Default multicast address used by ssl-game-controller for referee messages
        self.declare_parameter("ip", "224.5.23.1")
        self.declare_parameter("port", 11003)
        # Buffer size for UDP receive
        self.declare_parameter("buffer_size", 65536)
        self.declare_parameter("verbose", False)
        # Optional local interface IP to use for multicast membership (e.g. 127.0.0.1 or host iface)
        self.declare_parameter("interface", "")

    def get_params(self):
        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.buffer_size = (
            self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value
        self.interface = self.get_parameter("interface").get_parameter_value().string_value

    def setup_referee_client(self):
        # Setup UDP multicast client
        self.client = RefereeClient(ip=self.ip, port=self.port, buffer_size=self.buffer_size)

        self.get_logger().info(f"Binding referee client on {self.ip}:{self.port} (interface={self.interface or 'any'})")
        # Pass interface if explicitly set, otherwise let client use INADDR_ANY
        self.client.connect(interface_ip=self.interface or None)

    def receive_and_publish(self):

        self.get_logger().debug("Polling for referee message...")

        try:
            result = self.client.receive()
            if not result:
                self.get_logger().warn("No referee message received.")
                return

            # Parse protobuf Referee message
            referee_proto = Referee()
            referee_proto.ParseFromString(result[0])

            self.get_logger().debug(f"Received {len(result[0])} bytes from {result[1]}")

            self.publish_to_topic(self, referee_proto)

            self.log_changes(referee_proto, result)

        except KeyboardInterrupt:
            return
        except Exception as e:  # keep broad as network parsing can raise multiple kinds
            self.get_logger().error(
                f"Error receiving/parsing referee message: {e}, traceback: {traceback.format_exc()}")
            return

    def publish_to_topic(self, node: Node, referee_proto: Referee):
        """Wraps a Referee protobuf message into a ROS message and publishes it."""
        wrapped = RefereeMessageWrapper(referee_proto).msg
        node.publisher.publish(wrapped)

    def log_changes(self, referee_proto: Referee, result: Optional[tuple[bytes, tuple[str, int]]]):
        data, addr = result
        # Convert enum to name if possible
        try:
            cmd_name = Referee.Command.Name(referee_proto.command)
        except Exception:
            cmd_name = str(referee_proto.command)

        # Only show the receipt and detail lines when command or counter changes
        if (self.command_was_updated(referee_proto)):
            self._last_command = referee_proto.command
            self._last_command_counter = referee_proto.command_counter

            self.get_logger().info(f"Received {len(data)} bytes from {addr}")
            stage_name = Referee.Stage.Name(referee_proto.stage)
            self.get_logger().info(
                f"Referee changed: command={cmd_name} counter={referee_proto.command_counter} "
                f"stage={stage_name} time_left={referee_proto.stage_time_left}"
            )

    def command_was_updated(self, referee_proto: Referee):
        """Check if the command or command counter has changed since last seen."""
        return (
            referee_proto.command != self._last_command
            or referee_proto.command_counter != self._last_command_counter
        )

def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
