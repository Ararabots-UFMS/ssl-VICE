from rclpy.node import Node

from control_unit.coach import Coach

class GameWatcher(Node):
    def __init__(self):
        super().__init__('game_watcher')
        self.blackboard = Blackboard()
        
        self.vision_subscriber = TopicSubscriber('vision_subs', VisionMessage, 'visionTopic')
        self.referee_subscriber = RefereeSubscriber('gui_subs', GUIMessage, 'guiTopic')
        self.gui_subscriber = GUISubscriber('referee_subs', RefereeMessage, 'refereeTopic')
        
        self.coach = Coach(self.behaviour_tree, self.blackboard, max_robots=3)
        
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.vision_subscriber)
        self.executor.add_node(self.referee_subscriber)
        self.executor.add_node(self.gui_subscriber)
        self.executor.add_node(self.coach)
        
        self.timer = self.create_timer(0.1, self.update_from_interpreters)
        
        #start listening for messages from interpreters
        self.executor.spin()
    
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