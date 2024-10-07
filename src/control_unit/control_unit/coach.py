import rclpy.executors
from rclpy.node import Node
import rclpy

from control_unit.robot import Robot
from utils.topic_subscriber import TopicSubscriber
from strategy.blackboard import Blackboard

class Coach(Node):
    def __init__(self, behaviour_tree, max_robots: int) -> None:
        super().__init__('coach')
        
        #get coach behaviour tree
        self.behaviour_tree = behaviour_tree
        
        #get blackboard
        self.blackboard = Blackboard()
        
        #TODO: experiment with other timer rates
        self.robot_executor = rclpy.executors.MultiThreadedExecutor(num_threads=max_robots)
        
        self.robots = {}
        for ally_robot in self.blackboard.ally_robots:
            self.robots[ally_robot.id] = Robot(ally_robot)
            self.robot_executor.add_node(self.robots[ally_robot.id])
            
        self.timer = self.create_timer(0.5, self.update)
        self.timer = self.create_timer(0.1, self.run)
        
        self.robot_executor.spin()

    def update(self):
        for ally_robot in self.blackboard.ally_robots:
            try:
                self.robots[ally_robot.id]
            except:
                #TODO: implementar nome dos robos
                self.robots[ally_robot.id] = Robot(ally_robot.id, f"robo{ally_robot.id}", None)
                
        #TODO: isso aqui não roda, precisa ser implementado de vdd depois
        for robot in self.robots:
            if robot not in self.blackboard.ally_robots:
                self.robots[robot].destroy_node()
                self.robots.pop(robot)
        
    def run(self):
        if self.behaviour_tree != None:
            self.behaviour_tree.run(self.blackboard)
        else:
            pass
        
def main(args=None):
    rclpy.init(args=args)
    coach = Coach(None)
    rclpy.spin(coach)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()