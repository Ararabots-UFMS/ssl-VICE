import rclpy.executors
from rclpy.node import Node
import rclpy
import threading

from coach.robot import Robot
from utils.vision_subscriber import VisionSubscriber
from strategy.blackboard import Blackboard
from utils.referee_subscriber import RefereeSubscriber

class Coach(Node):
    def __init__(self, behaviour_tree) -> None:
        super().__init__('coach')
        
        #get coach behaviour tree
        self.behaviour_tree = behaviour_tree
        
        self.vision_subscriber = VisionSubscriber()
        self.referee_subscriber = RefereeSubscriber()
        #self.gui_subscriber = GUISubscriber()
        
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.vision_subscriber)
        self.executor.add_node(self.referee_subscriber)
        #self.executor.add_node(self.gui_subscriber)
        
        #start listening for messages from interpreters
        thread = threading.Thread(target=self.spin_executor)
        thread.start()
        
        self.blackboard = Blackboard()
        
        #TODO: experiment with other timer rates
        self.update_timer = self.create_timer(0.1, self.spin)
        # self.run_timer = self.create_timer(0.1, self.run)
        
    def spin_executor(self):
        self.executor.spin()
    
    def update_from_interpreters(self):
        self.update_from_vision()
        self.update_from_gamecontroller()
        self.update_from_gui()
        
        self.update()
        
    def update_from_vision(self):
        message = self.vision_subscriber.get_message()
    
        self.blackboard.update_from_vision_message(message)
        
    def update_from_gamecontroller(self):
        message = self.referee_subscriber.get_message()
        self.blackboard.update_from_gamecontroller_message(message)
        pass
    
    def update_from_gui(self):
        #message = self.gui_subscriber.get_message()
        #self.blackboard.update_from_gui_message(message)
        pass
    
    def update(self):
        for robot in self.blackboard.get_robots():
            robot.run()
        
    def run(self):
        self.behaviour_tree.run(self.blackboard)
        
def main(args=None):
    rclpy.init(args=args)
    coach = Coach(None)
    rclpy.spin(coach)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()