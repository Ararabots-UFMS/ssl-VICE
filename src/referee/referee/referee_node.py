import rclpy
from rclpy.node import Node
from google.protobuf import json_format
from system_interfaces.msg import GameData
from referee.referee_client import Client  
from referee.proto.ssl_gc_referee_message_pb2 import Referee
from strategy.blackboard import Blackboard

class RefereeNode(Node):
    """ROS2 Node that listens to ssl-game-controller referee multicast messages."""

    def __init__(self):
        super().__init__('refereeNode')

        # Parameters settings.
        self.ip = self.declare_parameter('ip', '224.5.23.1').get_parameter_value().string_value
        self.port = self.declare_parameter('port', 10003).get_parameter_value().integer_value
        self.buffer_size = self.declare_parameter('buffer_size', 1024).get_parameter_value().integer_value

        # Initialize the client
        self.client = Client(self.ip, self.port, self.buffer_size)
        self.client.connect()

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(GameData, 'referee_messages', 10)
        self.timer_ = self.create_timer(0.001, self.listen_to_multicast)
        self.last_message = GameData()

        self.get_logger().info(f"Listening for multicast messages on {self.ip}:{self.port}")
        self.listen_to_multicast()

    def listen_to_multicast(self):
        """Listen to multicast messages and publish to ROS2 topic."""
        try:
            data = self.client.receive()
            referee_message = Referee()

            try:
                # Parse the Protobuf message
                referee_message.ParseFromString(data)

                # Create and populate GameData message
                msg = self.parse_referee_message(referee_message)

                # Publish only if command_counter has changed
                if self.last_message != msg:
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published new Referee message: {msg}")
                    self.last_message = msg

            except Exception as e:
                self.get_logger().warning(f"Failed to parse Protobuf message: {str(e)}")

        except Exception as e:
            self.get_logger().error(f"Error receiving multicast message: {str(e)}")

    def parse_referee_message(self, referee_message):
        """Converts the referee message into the GameData format."""
        msg = GameData()
        msg.stage = Referee.Stage.Name(referee_message.stage)
        msg.command = Referee.Command.Name(referee_message.command)
        msg.command_counter = referee_message.command_counter

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()