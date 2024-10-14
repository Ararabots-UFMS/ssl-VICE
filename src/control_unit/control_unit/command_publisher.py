from rclpy.node import Node

from system_interfaces.msg import TeamCommand, RobotCommand
from time import time


class CommandPublisher(Node):
    def __init__(self, coach) -> None:
        super().__init__("command_publisher")

        self.coach = coach

        self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)

        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):

        msg = TeamCommand()

        msg.isteamyellow = self.coach.blackboard.gui.is_team_color_yellow

        current_time = time()
        for robot in self.coach.robots.values():
            elapsed_time = current_time - robot.trajectory_start_time
            _, velocities, _ = robot.trajectory.at_time(elapsed_time)
            command = RobotCommand()
            command.robot_id = robot.id
            command.linear_velocity_x = velocities[0]
            command.linear_velocity_y = velocities[1]
            command.angular_velocity = velocities[2]
            msg.robots.append(command)

        self.publisher.publish(msg)
