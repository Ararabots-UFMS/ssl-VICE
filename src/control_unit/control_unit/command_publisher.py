from rclpy.node import Node

from system_interfaces.msg import TeamCommand

#TODO: Implement the command publisher
class CommandPublisher(Node):
    def __init__(self, coach) -> None:
        super().__init__('command_publisher')
        self.coach = coach
        self.publisher = self.create_publisher(TeamCommand, 'commandTopic', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        
    def publish_command(self):
        self.get_logger().info(f"Number of robots: {len(self.coach.robots)}")
        # msg = TeamCommand()
        
        # msg.isteamyellow = self.coach.blackboard.gui.is_team_color_yellow
        
        # for id, robot in self.coach.robots.items():
        #     msg.robots.append(robot.trajectory)
        
        # msg.robots = self.blackboard.ally_robots
        # self.publisher.publish(msg)