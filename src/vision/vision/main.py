from vision.vision_client import Client
from vision.tracker import ObjectTracker
from typing import Optional

import rclpy
from rclpy.node import Node

from system_interfaces.msg import VisionMessage, Robots, Balls

class Vision(Node):
    '''VICE Vision Node, connects and receives data from ssl-vision'''
    def __init__(self):
        super().__init__('visionNode')
        
        # Parameters settings.
        self.declare_parameter('ip', '224.5.23.2')
        self.declare_parameter('port', 10006)
        self.declare_parameter('verbose', False)
        
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        # Verbose prints in terminal all received data.
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        self.client = Client(ip = self.ip, port = self.port)

        self.get_logger().info(f"Binding client on {self.ip}:{self.port}")
        self.client.connect()

        # Setting ROS publisher.
        # TODO: Find optimal queue size...
        self.publisher = self.create_publisher(VisionMessage, 'visionTopic', 10)

        # TODO: Find the optimal timer.
        self.timer = self.create_timer(0.1, self.receive)

    def receive(self):
        # Set max_frame_skipped as a parameter, not a constant.
        tracker = ObjectTracker(max_frame_skipped = 10000)

        try:
            data = self.client.receive()

            if self.verbose:
                self.get_logger().info(data)

            message = VisionMessage()

            if data.detection.robots_yellow:
                for yellow_robot in data.detection.robots_yellow:
                    robot_msg = Robots()
                    robot_msg.id = yellow_robot.robot_id
                    robot_msg.position_x = yellow_robot.x
                    robot_msg.position_y = yellow_robot.y
                    robot_msg.orientation = yellow_robot.orientation

                    message.yellow_robots.append(robot_msg)
            
            if data.detection.robots_blue:
                for blue_robot in data.detection.robots_blue:
                    robot_msg = Robots()
                    robot_msg.id = blue_robot.robot_id
                    robot_msg.position_x = blue_robot.x
                    robot_msg.position_y = blue_robot.y
                    robot_msg.orientation = blue_robot.orientation

                    message.blue_robots.append(robot_msg)
            
            if data.detection.balls:
                for ball in data.detection.balls:
                    ball_msg = Balls()
                    ball_msg.position_x = ball.x
                    ball_msg.position_y = ball.y

                    message.balls.append(ball_msg)

            message = tracker.update(message)

            if self.context.ok():
                self.publisher.publish(message)

        except KeyboardInterrupt:
            print('Process finished successfully by user, terminating now...')
        except Exception as exception:
            print('An unexpected error occurred:')
            print(exception)

def main(args = None):
    rclpy.init(args = args)
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()