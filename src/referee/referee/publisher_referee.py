import rclpy
from rclpy.node import Node
from google.protobuf import text_format


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('referee_publisher')

        self.declare_parameter('ip', '224.5.23.1')
        self.declare_parameter('port', 10003)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.client = Client(ip = self.ip, port = self.port)


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
