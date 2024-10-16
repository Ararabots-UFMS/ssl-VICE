from rclpy.node import Node

from system_interfaces.msg import GUIMessage

class GUIPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        
        # TODO: Find optimal queue size...
        self.publisher = self.create_publisher(GUIMessage, 'guiTopic', 10)
        self.unify_timer = self.create_timer(0.016, self.publish_gui_data)
        
        self.is_field_side_left = False
        self.is_team_color_yellow = False
        self.is_play_pressed = False
        
    def create_message(self) -> GUIMessage:
        msg = GUIMessage()
        msg.is_field_side_left = self.is_field_side_left
        msg.is_team_color_yellow = self.is_team_color_yellow
        msg.is_play_pressed = self.is_play_pressed
        return msg

    def publish_gui_data(self) -> None:
        message = self.create_message()
        self.publisher.publish(message)