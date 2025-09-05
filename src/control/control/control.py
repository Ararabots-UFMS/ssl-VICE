from system_interfaces.msg import ControlCommand
from system_interfaces.msg import TeamCommand, RobotCommand
from system_interfaces.srv import ControlParams

from strategy.blackboard import Blackboard
from control.pid_controller import RobotTrajectoryController
from new_movement.entities.States import Vector2D, State

import rclpy
from rclpy.node import Node

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscriber = self.create_subscription(
            ControlCommand, 'control_command', self.send_command, 10
        )
        self.publisher = self.create_publisher(TeamCommand, 'commandTopic', 10)

        self.update_parameters_service = self.create_service(
            ControlParams, 'update_pid', self.update_parameters
        )

        self.blackboard = Blackboard()
        
        self.robot_controller = RobotTrajectoryController()
        
    def send_command(self, message):
        team_command = TeamCommand()
        team_command.robots = []
        team_command.is_team_color_yellow = self.blackboard.gui.is_team_color_yellow

        commanded_robot_ids = set()

        for robot in message.command:
            robot_command = RobotCommand()
            robot_command.robot_id = robot.id
            commanded_robot_ids.add(robot.id)
            
            try:
                if robot.id not in self.blackboard.ally_robots:
                    continue
                
                current_robot = self.blackboard.ally_robots[robot.id]
                
                current_position = Vector2D(current_robot.position_x / 1000, current_robot.position_y / 1000)
                current_velocity = Vector2D(current_robot.velocity_x / 1000, current_robot.velocity_y / 1000)
                current_state = State(current_position, current_velocity)
                
                target_position = Vector2D(robot.position_x, robot.position_y)
                target_velocity = Vector2D(robot.velocity_x, robot.velocity_y)
                target_state = State(target_position, target_velocity)
                
                velocity_command = self.robot_controller.compute_trajectory_command(
                    robot.id, target_state, current_state
                )
                
                robot_command.linear_velocity_x = float(velocity_command.x)
                robot_command.linear_velocity_y = float(velocity_command.y)
                robot_command.angular_velocity = 0.0  # TODO: Add orientation controller
                robot_command.orientation = current_robot.orientation
                robot_command.kick = 0.0
                
                team_command.robots.append(robot_command)
                
            except:
                self.get_logger().info("ERROR")

        # self.robot_controller.cleanup_unused_robots(commanded_robot_ids)

        self.publisher.publish(team_command)

    def update_parameters(self, request, response):
        self.robot_controller.update_params(request.kp, request.ki, request.kd)
        
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()