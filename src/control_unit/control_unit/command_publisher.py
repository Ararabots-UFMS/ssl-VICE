from rclpy.node import Node

from system_interfaces.msg import TeamCommand, RobotCommand
from time import time
from movement.path.path_profiles import MovementProfiles, DirectionProfiles

from math import cos, sin


def rotate_velocities_to_robot_frame(vx, vy, orientation):
    vel_norm = vx * cos(orientation) - vy * sin(orientation)
    vel_tan = vx * sin(orientation) + vy * cos(orientation)
    return float(vel_norm), float(vel_tan)


mm_to_m = 1 / 1000


class CommandPublisher(Node):
    def __init__(self, coach) -> None:
        super().__init__("command_publisher")

        self.coach = coach

        self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)

        self.timer = self.create_timer(1/60, self.publish_command)

    def publish_command(self):

        msg = TeamCommand()

        msg.is_team_color_yellow = self.coach.blackboard.gui.is_team_color_yellow
        # msg.is_team_color_yellow = True

        current_time = time()
        for robot in self.coach.robots.values():
            elapsed_time = current_time - robot.trajectory_start_time

            # position, velocities, _ = robot.path_trajectory.at_time(elapsed_time)
            # orientation, angular_velocity, _ = robot.orientation_trajectory.at_time(elapsed_time)
            velocities = robot.velocities
            orientation = robot.get_state()[0][2]

            vel_norm, vel_tan = rotate_velocities_to_robot_frame(
                velocities[0] * mm_to_m,
                velocities[1] * mm_to_m,
                -orientation,
            )

            vel_angular = float(velocities[2])

            if robot.current_command != None:
                if robot.current_command["path_profile"] == MovementProfiles.Break:
                    vel_norm, vel_tan = 0.0, 0.0
                if robot.current_command["orientation_profile"] == DirectionProfiles.Break:
                    vel_angular = 0.0


            command = RobotCommand()
            command.robot_id = robot.id
            command.linear_velocity_x = vel_norm
            command.linear_velocity_y = vel_tan
            command.angular_velocity = vel_angular
            command.kick = robot.kick*1.5 #1.5 m/s is our kick speed
            msg.robots.append(command)



        self.publisher.publish(msg)
