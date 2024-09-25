import rclpy.executors
from rclpy.node import Node
import rclpy

from control_unit.robot import Robot
from utils.topic_subscriber import TopicSubscriber
from strategy.blackboard import Blackboard

class Coach(Node):
    def __init__(self, blackboard, behaviour_tree, max_robots: int) -> None:
        super().__init__('coach')
        
        #get coach behaviour tree
        self.behaviour_tree = behaviour_tree
        
        #get blackboard
        self.blackboard = blackboard
        
        #TODO: experiment with other timer rates
        self.robot_executor = rclpy.executors.MultiThreadedExecutor(max_threads=max_robots)
        
        self.robots = {}
        for ally_robot in blackboard.ally_robots:
            self.robots[ally_robot.id] = Robot(ally_robot)
            self.robot_executor.add_node(self.robots[ally_robot.id])
            
        self.timer = self.create_timer(0.5, self.update)
        self.timer = self.create_timer(0.1, self.run)
        
        self.robot_executor.spin()

    def update(self):
        for ally_robot in blackboard.ally_robots:
            try:
                self.robots[robot.id].update(ally_robot)
            except:
                self.robots[robot.id] = Robot(ally_robot)
                
        #TODO: isso aqui não roda, precisa ser implementado de vdd depois
        for robot in self.robots:
            if robot not in blackboard.ally_robots:
                self.robots[robot].destroy_node()
                self.robots.pop(robot)
        
    def run(self):
        self.behaviour_tree.run(self.blackboard)
        
def main(args=None):
    rclpy.init(args=args)
    coach = Coach(None)
    rclpy.spin(coach)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()