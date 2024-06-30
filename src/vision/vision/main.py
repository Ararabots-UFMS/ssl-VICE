from vision.vision_client import Client
from typing import Optional

import rclpy
import rclpy.node.Node as Node

from system_interfaces.msg import visionMessage 

class Vision(Node):
    '''VICE Vision Node, connects and receives data from ssl-vision'''
    def __init__(self):
        super().__init__('visionNode')
        
        # Parameters settings.
        self.declare_parameter('ip', '224.5.23.2')
        self.declare_parameter('port', 10006)
        self.declare_parameter('verbose', False)
        
        ip = self.get_parameter('ip').value.as_str()
        port = self.get_parameter('port').value.as_int()
        # Verbose prints in terminal all received data.
        self.verbose = self.get_parameter('verbose').value.as_bool()
        
        self.client = Client(ip = ip, port = port)

        self.get_logger().info(f"Binding client on {self.ip}:{self.port}")
        self.client.connect()

        # Setting ROS publisher.
        # TODO: Find optimal queue size...
        self.publisher = self.create_publisher(visionMessage, 'visionTopic', queue_size=10)

        # TODO: Find the optimal timer.
        self.timer = self.create_timer(0.25, self.receive)

    def receive(self):
        try:
            data = client.receive()

            # TODO: apply filter to reduce noise

            if self.verbose:
                self.get_logger().info(data)

            message = visionMessage()
            
            # TODO: Process data...

            if self.context.ok():
                self.publisher.publish(message)

        except KeyboardInterrupt:
            print('Process finished successfully by user, terminating now...')
        except Exception as exception:
            print('An unexpected error occurred:')
            print(exception)

def main(args = None):
    rclpy.init(args=args)
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()