from rclpy.node import Node

from strategy.blackboard import Blackboard

from movement.move import Movement

class Robot(Node):
    def __init__(self, id, name) -> None:
        super().__init__(f'robot_{name}')
        self.blackboard = Blackboard()
        self.name = name
        self.id = id
        self.behaviour_tree = None
        self.address = None
        self.pid = None
        
        self.move = Movement(self.id)
        self.trajectory = None
        
        self.timer = self.create_timer(0.1, self.run)
    
    def get_position(self):
        return (self.blackboard.ally_robots[self.id].position_x,
                self.blackboard.ally_robots[self.id].position_y)
        
    def get_velocity(self):
        return (self.blackboard.ally_robots[self.id].velocity_x,
                self.blackboard.ally_robots[self.id].velocity_y)
        
    def get_orientation(self):
        return self.blackboard.ally_robots[self.id].orientation
    
    def run(self):
        self.get_logger().info(f"Running robot {self.id}")
        # self.behaviour_tree.run()
        # self.move()