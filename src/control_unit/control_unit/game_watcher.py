from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor

from utils.topic_subscriber import TopicSubscriber 
from system_interfaces.msg import VisionMessage, GUIMessage#, RefereeMessage
from strategy.blackboard import Blackboard


class GameWatcher(Node):
    def __init__(self, _executor) -> None:
        super().__init__('game_watcher')
        self.blackboard = Blackboard()
        
        self.vision_subscriber = TopicSubscriber('vision_subs', VisionMessage, 'visionTopic')
        self.gui_subscriber = TopicSubscriber('gui_subs', GUIMessage, 'guiTopic')
        self.referee_subscriber = TopicSubscriber('referee_subs', GUIMessage, 'refereeTopic')
        
        _executor.add_node(self.vision_subscriber)
        _executor.add_node(self.referee_subscriber)
        _executor.add_node(self.gui_subscriber)
        
        self.timer = self.create_timer(0.1, self.update_from_interpreters)
    
    def update_from_interpreters(self):
        self.update_from_vision()
        self.update_from_gamecontroller()
        self.update_from_gui()
        
    def update_from_vision(self):
        message = self.vision_subscriber.get_message()
    
        self.blackboard.update_from_vision_message(message)
        
    def update_from_gamecontroller(self):
        message = self.referee_subscriber.get_message()
        
        self.blackboard.update_from_gamecontroller_message(message)
        
    def update_from_gui(self):
        message = self.gui_subscriber.get_message()
        
        self.blackboard.update_from_gui_message(message)