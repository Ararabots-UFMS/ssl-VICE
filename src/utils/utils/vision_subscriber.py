from rclpy.node import Node
import rclpy

from system_interfaces.msg import VisionMessage

class VisionSubscriber(Node):
    '''Receives data from vice-vision package'''
    def __init__(self):
        super().__init__('visionNode')
        
        self.data = []

        self.vision_subs = self.create_subscription(
            VisionMessage, 
            'visionTopic',
            self.listener_callback, 
            10)
        
    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg}")
        self.data = msg

    def get_data(self):
        return self.data
    
if __name__ == '__main__':
    rclpy.init()
    vision_subs = VisionSubscriber()
    while True:
        data = vision_subs.get_data()
        print(f"Received update: {data}", flush=True)
        rclpy.spin_once(vision_subs)