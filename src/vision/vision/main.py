from vision.vision_client import Client
from vision.tracker import ObjectTracker
from typing import Optional

import rclpy
from rclpy.node import Node

from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from vision.merge_trackers import merge_trackers

from system_interfaces.msg import VisionMessage, Robots, Balls, ObjectID

class Vision(Node):
    '''VICE Vision Node, connects and receives data from ssl-vision'''
    def __init__(self):
        super().__init__('visionNode')
        
        # Parameters settings.
        self.declare_parameter('ip', '224.5.23.2')
        self.declare_parameter('port', 10006)
        self.declare_parameter('verbose', False)
        self.declare_parameter('num_cams', 4)
        self.declare_parameter('max_frame_skipped', 5)
        
        # Verbose prints in terminal all received data.
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.num_cams = self.get_parameter('num_cams').get_parameter_value().integer_value
        self.max_frame_skipped = self.get_parameter('max_frame_skipped').get_parameter_value().integer_value
        
        self.client = Client(ip = self.ip, port = self.port)

        self.get_logger().info(f"Binding client on {self.ip}:{self.port}")
        self.client.connect()

        # Setting ROS publisher.
        # TODO: Find optimal queue size...
        self.publisher = self.create_publisher(VisionMessage, 'visionTopic', 10)

        self.trackers = []
        for cam in range(self.num_cams):
            self.trackers.append(ObjectTracker(cam_id = cam, max_frame_skipped = self.max_frame_skipped))

        # TODO: Find the optimal timer.
        self.unify_timer = self.create_timer(0.1, self.publish_vision)
        self.tracker_timer = self.create_timer(0.001, self.update_tracker)

    def update_tracker(self):
        try:
            # Orientation does not have a proper processing. Using raw orientantion and setting orientation velocity to 0.
            data: SSL_WrapperPacket = self.client.receive()

            data_cam_id = data.detection.camera_id

            self.trackers[data_cam_id].update(data)

            if self.verbose:
                self.get_logger().info(data)

        except KeyboardInterrupt:
            print('Process finished successfully by user, terminating now...')
        except Exception as exception:
            print('An unexpected error occurred:')
            print(exception)

    def publish_vision(self):
        message = merge_trackers(self.trackers)
        
        if self.context.ok():
                self.publisher.publish(message)

def main(args = None):
    rclpy.init(args = args)
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()