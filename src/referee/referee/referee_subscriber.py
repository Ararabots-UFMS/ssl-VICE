import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RefereeSubscriber(Node):
    def __init__(self):
        super().__init__('referee_subscriber')
        self.subscription = self.create_subscription(
            String,
            'referee_messages',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received Referee message: {msg.data}")
        # Aqui você pode adicionar qualquer processamento adicional ou lógica de salvamento

def main(args=None):
    rclpy.init(args=args)
    node = RefereeSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()