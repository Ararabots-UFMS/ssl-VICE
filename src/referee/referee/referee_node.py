from typing import Optional

import rclpy
from rclpy.node import Node

from .referee_client import Client

from .referee_message_wrapper import MessageWrapping

from referee.proto.ssl_gc_referee_message_pb2 import Referee

from system_interfaces.msg import RefereeMessage


class RefereeNode(Node):
    """Listener node for SSL game-controller referee messages via multicast UDP."""

    def __init__(self):
        super().__init__("refereeNode")

        # Parameters settings.
        # Default multicast address used by ssl-game-controller for referee messages
        self.declare_parameter("ip", "224.5.23.1")
        self.declare_parameter("port", 10003)
        # Buffer size for UDP receive
        self.declare_parameter("buffer_size", 65536)
        self.declare_parameter("verbose", False)
        # Optional local interface IP to use for multicast membership (e.g. 127.0.0.1 or host iface)
        self.declare_parameter("interface", "")

        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.buffer_size = (
            self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value
        self.interface = self.get_parameter("interface").get_parameter_value().string_value

        # Setup UDP multicast client
        self.client = Client(ip=self.ip, port=self.port, buffer_size=self.buffer_size)

        self.get_logger().info(f"Binding referee client on {self.ip}:{self.port} (interface={self.interface or 'any'})")
        # Pass interface if explicitly set, otherwise let client use INADDR_ANY
        iface = self.interface if self.interface else None
        self.client.connect(interface_ip=iface)

        # ROS publisher for the wrapped referee message
        self.publisher = self.create_publisher(RefereeMessage, "refereeTopic", 10)

        # Track last seen command to reduce noisy logs
        self._last_command = None
        self._last_command_counter = None

        # Polling timer to receive messages and publish. Short interval to be responsive.
        self.poll_timer = self.create_timer(0.01, self.receive_and_publish)

    def receive_and_publish(self):

        try:
            result = self.client.receive()
            if not result:
                return

            data, addr = result

            # Parse protobuf Referee message
            referee_proto = Referee()
            try:
                referee_proto.ParseFromString(data)

                # Wrap into ROS message and publish
                wrapped = MessageWrapping(referee_proto).msg
                if self.context.ok():
                    self.publisher.publish(wrapped)

                # Convert enum to name if possible
                try:
                    cmd_name = Referee.Command.Name(referee_proto.command)
                except Exception:
                    cmd_name = str(referee_proto.command)

                # Only show the receipt and detail lines when command or counter changes
                if (
                    referee_proto.command != self._last_command
                    or referee_proto.command_counter != self._last_command_counter
                ):
                    # update last seen values
                    self._last_command = referee_proto.command
                    self._last_command_counter = referee_proto.command_counter

                    # print the two lines the user wants
                    self.get_logger().info(f"Received {len(data)} bytes from {addr}")
                    self.get_logger().info(
                        f"Referee changed: command={cmd_name} counter={referee_proto.command_counter} "
                        f"stage={referee_proto.stage} time_left={referee_proto.stage_time_left}"
                    )

            except Exception as parse_exc:
                # keep parse failures quiet by default; log at debug for troubleshooting
                preview = data[:64]
                self.get_logger().debug(
                    f"Failed to parse Referee protobuf from {addr}: {parse_exc}; data-preview={preview.hex()}"
                )

        except KeyboardInterrupt:
            self.get_logger().info("Process finished by user, terminating...")
        except Exception as exc:  # keep broad as network parsing can raise multiple kinds
            self.get_logger().warning(f"Error receiving/parsing referee message: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
