from system_interfaces.msg import ControlCommand, RobotControlCommand
from system_interfaces.msg import TeamCommand, RobotCommand
from system_interfaces.msg import PID

from strategy.blackboard import Blackboard

import rclpy
from rclpy.node import Node

class Controller(Node):
    def __init__(self):
        self.subscriber = self.create_subscription(
            ControlCommand, 'control_command', self.send_command, 10
        )
        self.publisher = self.create_publisher(TeamCommand, 'commandTopic', 10)

        # self.update_parameters_service = self.create_service(
        #     PID, 'update_pid', self.update_parameters
        # )

        self.blackboard = Blackboard()
        
    def send_command(self, msg):
        # TODO only doing feedforward, need feedback too
        team_command = TeamCommand()
        team_command.robots = []
        team_command.is_team_color_yellow = self.blackboard.gui.is_team_color_yellow

        for robot in msg.command:
            robot_command = RobotCommand()
            robot_command.robot_id = robot.id
            robot_command.linear_velocity_x = robot.velocity_x
            robot_command.linear_velocity_y = robot.velocity_y
            robot_command.angular_velocity = 0.0
            robot_command.kick = False

            team_command.robots.append(robot_command)

        self.publisher.publish(team_command)

    def update_parameters(self, request, response):
        pass
