import rclpy
from rclpy.node import Node

from system_interfaces.msg import Communication



class CommunicationNode(Node):

    def __init__(self):
        super().__init__('communication_node')
        self.subscription = self.create_subscription(Communication,'communicationTopic', self.forward, 10)
        self.subscription  # prevent unused variable warning

    def forward(self, msg):
        # Serialize message and send to hardware.
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