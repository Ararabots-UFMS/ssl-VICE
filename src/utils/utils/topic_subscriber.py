import rclpy
from rclpy.node import Node

class TopicSubscriber(Node):
    def __init__(self, name: str, message_type, topic: str):
        super().__init__(name)
        self.data = None
        self.subscription = self.create_subscription(
            message_type,
            topic,
            self.update_message,
            10)

    def update_message(self, msg) -> None:
        self.data = msg

    def get_message(self):
        return self.data

def main(args=None):
    rclpy.init(args=args)
    node = TopicSubscriber('vision_subs', VisionMessage, 'visionTopic')
    # node = TopicSubscriber('gui_subs', GUIMessage, 'guiTopic')
    # node = TopicSubscriber('referee_subs', RefereeMessage, 'refereeTopic')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
