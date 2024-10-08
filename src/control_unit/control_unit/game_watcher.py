from rclpy.node import Node
import rclpy
from utils.topic_subscriber import TopicSubscriber 
from system_interfaces.msg import VisionMessage, GUIMessage, GameData
from strategy.blackboard import Blackboard
from control_unit.coach import Coach
from strategy.play.strategy_node import make_bt
from py_trees import logging as log_tree

class GameWatcher(Node):
    def __init__(self, executor):
        super().__init__('game_watcher')
        self.blackboard = Blackboard()

        """ Made some changes to run just the referee subscriber """
        
        # self.vision_subscriber = TopicSubscriber('vision_subs', VisionMessage, 'visionTopic')
        # self.gui_subscriber = TopicSubscriber('gui_subs', GUIMessage, 'guiTopic')
        self.referee_subscriber = TopicSubscriber('referee_subs', GameData, '/referee_messages')

        # self.coach = Coach(behaviour_tree=self.behaviour_tree, max_robots=3)
        
        executor.add_node(self.referee_subscriber)
        # self.executor.add_node(self.vision_subscriber)
        # self.executor.add_node(self.gui_subscriber)
        # self.executor.add_node(self.coach)
        
        self.timer = self.create_timer(0.001, self.update_from_interpreters)
        self.timer = self.create_timer(0.001, self.update_behaviour_tree)

    def update_from_interpreters(self):
        self.update_from_gamecontroller()
        # self.update_from_vision()
        # self.update_from_gui()
        
    def update_from_gamecontroller(self):
        message = self.referee_subscriber.get_message()
        if message:
            self.blackboard.update_from_gamecontroller_message(message)
        else:
            print("No message received")
    
    def update_behaviour_tree(self):
        # Create the initial tree based on the blackboard state
        log_tree.level = log_tree.Level.DEBUG
        tree = make_bt(self.blackboard.referee.command)  # Update the tree with the new state
        tree.tick_once()

    
    # def update_from_vision(self):
    #     message = self.vision_subscriber.get_message()
    
    #     self.blackboard.update_from_vision_message(message)
        
        
    # def update_from_gui(self):
    #     message = self.gui_subscriber.get_message()
        
    #     self.blackboard.update_from_gui_message(message)

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the executor outside the class
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)

    game_watcher = GameWatcher(executor)
    executor.add_node(game_watcher)
    executor.spin()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
