class Blackboard():
    def __init__(self) -> None:
        self.yellow_robots = []
        self.blue_robots = []
        self.balls = []
        
    def update_from_vision_message(self, message):
        pass