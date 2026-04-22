import rclpy
from rclpy.node import Node

from new_movement.entities.States import State, Vector2D
from new_movement.planner import CollisionSolver
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from movement_interfaces.msg import (
    TargetArray,
    Trajectory,
    TrajectoryPoint,
)
from typing import Dict


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner')
        self.cur_targets: TargetArray = None
        self.cur_overhead_points: Dict[int, TrajectoryPoint] = {}
        self.solver = CollisionSolver(trys = 50)
        self.generator = TrajectoryGenerator()

        # Publisher
        self.trajectory_publisher = self.create_publisher(Trajectory, 'planner_topic', 10)

        # Subscribers
        self.target_subscriber = self.create_subscription(TargetArray, 'target_topic', self.target_callback, 10)
        self.overhead_subscriber = self.create_subscription(TrajectoryPoint, 'overhead_topic', self.overhead_callback, 10)

        # Timer
        self.planner_timer = self.create_timer(0.016, self.plan_trajectory)

    def plan_trajectory(self) -> None:
        # e publicar uma trajectoria
        for target in self.cur_targets.targets:
            target_point = State(
                                Vector2D(target.target_pos.x, target.target_pos.y), 
                                Vector2D(target.target_vel.x, target.target_vel.y)
            )
            if(target.robot_id in self.cur_overhead_points):
                overhead_point: TrajectoryPoint = self.cur_overhead_points[target.robot_id]
                initial_point = State(
                                    Vector2D(overhead_point.pos.x, overhead_point.pos.y), 
                                    Vector2D(overhead_point.vel.x, overhead_point.vel.y)
                )
            else:
                initial_point = State(
                                    Vector2D(target.initial_pos.x, target.initial_pos.y), 
                                    Vector2D(target.initial_vel.x, target.initial_vel.y)
                )
            
            trajectory = self.solver(initial_point, target_point, [], self.generator) # TODO Obstacles
            if trajectory is not None:
                self.trajectory_publisher.publish(trajectory.to_msg()) # TODO to_msg
            
    def target_callback(self, msg: TargetArray) -> None:
        self.cur_targets = msg

    def overhead_callback(self, msg: TrajectoryPoint) -> None:
        robot_id: int = msg.robot_id
        self.cur_overhead_points[robot_id] = msg

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin()
    .destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
