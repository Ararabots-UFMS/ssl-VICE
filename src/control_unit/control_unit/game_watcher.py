from rclpy.node import Node
import rclpy

from utils.topic_subscriber import TopicSubscriber 
from system_interfaces.msg import VisionMessage, GUIMessage, GameData
from strategy.blackboard import Blackboard
from control_unit.coach import Coach

class GameWatcher(Node):
    def __init__(self):
        super().__init__('game_watcher')
        self.blackboard = Blackboard()
        
        # self.vision_subscriber = TopicSubscriber('vision_subs', VisionMessage, 'visionTopic')
        self.gui_subscriber = TopicSubscriber('gui_subs', GUIMessage, 'guiTopic')
        self.referee_subscriber = TopicSubscriber('referee_subs', GameData, '/referee_messages')
        
        self.coach = Coach(self.behaviour_tree, self.blackboard, max_robots=3)
        
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        # self.executor.add_node(self.vision_subscriber)
        self.executor.add_node(self.referee_subscriber)
        self.executor.add_node(self.gui_subscriber)
        self.executor.add_node(self.coach)
        
        self.timer = self.create_timer(0.001, self.update_from_interpreters)
        
        #start listening for messages from interpreters
        self.executor.spin()
    
    def update_from_interpreters(self):
        self.update_from_vision()
        self.update_from_gamecontroller()
        self.update_from_gui()
        
    # def update_from_vision(self):
    #     message = self.vision_subscriber.get_message()
    
    #     self.blackboard.update_from_vision_message(message)
        
    def update_from_gamecontroller(self):
        message = self.referee_subscriber.get_message()
        
        self.blackboard.update_from_gamecontroller_message(message)
        
    def update_from_gui(self):
        message = self.gui_subscriber.get_message()
        
        self.blackboard.update_from_gui_message(message)

def main(args=None):
    rclpy.init(args=args)
    game_watcher = GameWatcher()
    rclpy.spin(game_watcher)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()