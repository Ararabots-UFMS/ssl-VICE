import rclpy
from rclpy.node import Node
from movement.obstacles.static_obstacles import BoundaryObstacles, WallObstacles, PenaltyAreaObstacles

from system_interfaces.msg import VisionGeometry


class GeometryObstacleSubscriber(Node):

    def __init__(self):
        super().__init__('geometry_obstacle_subscriber')
        self.subscription = self.create_subscription(
            VisionGeometry,
            'geometryTopic',
            self.update_static_geometries,
            10)
        self.subscription  # prevent unused variable warning

        self.obstacles = []

    def update_static_geometries(self, msg):
        obstacles = []
        obstacles.append(BoundaryObstacles(msg))
        obstacles.append(WallObstacles(msg))
        obstacles.append(PenaltyAreaObstacles(msg))

        self.obstacles = obstacles
        

    def get_obstacles(self):
        return self.obstacles