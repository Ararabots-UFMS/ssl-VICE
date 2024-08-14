import rclpy
from rclpy.node import Node

from typing import List

from communication import feedback_client

from system_interfaces.msg import Communication, Infos, Command

class CommunicationNode(Node):

    def __init__(self):
        super().__init__('communication_node')
        self.subscription = self.create_subscription(Communication, 'commandTopic', self.update_commands, 10)

        # Client to receive robots information.
        self.client = feedback_client()
        self.publisher = self.create_publisher(Infos, 'infoTopic', 10)
        
        # Receiving data at 120 Hz
        self.info_timer = self.create_timer(1/120, self.publish_infos)

        self.commands: List[Command] = []

        self.command_timer = self.create_timer(1/120, self.publish_command)

    def update_commands(self, msg: List[Command]):
        self.commands = msg
        return
    
    def publish_command(self, msgs: List[Command]):
        # Serialize commands and send to robots
        pass

    def publish_infos(self):
        # Publish 
        pass

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()