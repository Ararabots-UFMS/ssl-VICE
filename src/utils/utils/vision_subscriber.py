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
    rclpy.init()

    vision_subscriber = VisionSubscriber()

    rclpy.spin(vision_subscriber)
    vision_subscriber.destroy_node()
    rclpy.shutdown()