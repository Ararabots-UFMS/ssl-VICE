from rclpy.node import Node
import rclpy

from system_interfaces.msg import VisionMessage

class VisionSubscriber(Node):
    '''Receives data from vice-vision package'''
    def __init__(self):
        super().__init__('vision_subscriber')

        self.data = VisionMessage()

        self.vision_subs = self.create_subscription(
            VisionMessage, 
            'visionTopic',
            self.update_message, 
            10)
        
    def update_message(self, msg: VisionMessage) -> None:
        self.data = msg

    def get_message(self) -> VisionMessage:
        return self.data
    
if __name__ == '__main__':
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()