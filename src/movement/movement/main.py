import rclpy

from movement.geo_subscriber import GeometryObstacleSubscriber
from movement.obstacles.static_obstacles import BoundaryObstacles, WallObstacles, PenaltyAreaObstacles

def main():
    rclpy.init()

    geo_subscriber = GeometryObstacleSubscriber()
    geo_subscriber.get_logger().info('comecando')

    rclpy.spin_once(geo_subscriber)
    geo_subscriber.get_logger().info('comecando')

    obstacles = geo_subscriber.get_obstacles()
    x = (-4489, -1089, 0, 0)
    for obstacle in obstacles:
        if obstacle.is_colission(x):
            geo_subscriber.get_logger().info(f'COLISÃO em {type(obstacle)}')
        else:
            geo_subscriber.get_logger().info(f'No colission on {type(obstacle)}')

if __name__ == '__main__':
    main()