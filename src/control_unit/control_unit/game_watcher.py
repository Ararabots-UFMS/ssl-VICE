from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor

from utils.topic_subscriber import TopicSubscriber 
from system_interfaces.msg import VisionMessage, GUIMessage#, RefereeMessage
from strategy.blackboard import Blackboard


class GameWatcher(Node):
    def __init__(self) -> None:
        super().__init__('game_watcher')
        self.blackboard = Blackboard()
        
        self.vision_subscriber = self.create_subscription(VisionMessage,
                                                        'visionTopic',
                                                        self.update_from_vision,
                                                        10)
        self.gui_subscriber = self.create_subscription(GUIMessage,
                                                        'guiTopic',
                                                        self.update_from_gui,
                                                        10)
        #TODO: fix referee topic
        self.referee_subscriber = self.create_subscription(GUIMessage,
                                                        'refereeTopic',
                                                        self.update_from_gamecontroller,
                                                        10)
        
    def update_from_vision(self, message):
        self.blackboard.update_from_vision_message(message)
        
    def update_from_gamecontroller(self, message):
        self.blackboard.update_from_gamecontroller_message(message)
        
    def update_from_gui(self, message):
        self.blackboard.update_from_gui_message(message)