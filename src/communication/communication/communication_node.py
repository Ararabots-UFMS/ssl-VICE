import rclpy
from rclpy.node import Node

from communication import feedback_client

from system_interfaces.msg import Communication, Infos

class CommunicationNode(Node):

    def __init__(self):
        super().__init__('communication_node')
        self.subscription = self.create_subscription(Communication, 'commandTopic', self.send_command, 10)

        # Client to receive robots information.
        self.client = feedback_client()
        self.publisher = self.create_publisher(Infos, 'infoTopic', 10)
        
        # Receiving data at 120 Hz
        self.timer = self.create_timer(1/120, self.publish_infos)

    def send_command(self, msg):
        # Serialize message and send to hardware.
        if msg.commands:
            # Send command to robot
            pass

    def publish_infos(self):
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