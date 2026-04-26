import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from concurrent.futures import ThreadPoolExecutor

from new_movement.entities.States import State, Vector2D
from new_movement.entities.Trajectory import Trajectory
from new_movement.planner import Planner
from new_movement.utilities.obstacle_factory import ObstacleFactory

from movement_interfaces.msg import (
    TargetArray,
    Trajectory as TrajectoryMsg,
    TrajectoryPoint as TrajectoryPointMsg,
)
from system_interfaces.msg import GameState

from typing import Dict, Optional


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # Parameters
        self.declare_parameter('planner_freq', 50.0)
        self.declare_parameter('max_threads', 8)
        
        # State
        self.cur_targets: Optional[TargetArray] = None
        self.cur_overhead_points: Dict[int, TrajectoryPointMsg] = {}
        self.game_state: Optional[GameState] = None
        self.last_planned_trajectories: Dict[int, Trajectory] = {}
        
        # Tools
        self.planner = Planner(bypass_trys=50)
        self.factory = ObstacleFactory()
        self.executor = ThreadPoolExecutor(max_workers=self.get_parameter('max_threads').value)
        
        # ROS Communication
        cb_group = ReentrantCallbackGroup()
        
        # Subscribers
        self.target_sub = self.create_subscription(
            TargetArray, 'target_topic', self.target_callback, 10, callback_group=cb_group)
        self.overhead_sub = self.create_subscription(
            TrajectoryPointMsg, 'overhead_topic', self.overhead_callback, 10, callback_group=cb_group)
            
        # Publisher
        self.trajectory_pub = self.create_publisher(TrajectoryMsg, 'planner_topic', 10)
        
        # Timer
        freq = self.get_parameter('planner_freq').value
        self.timer = self.create_timer(1.0 / freq, self.planning_loop, callback_group=cb_group)
        
        self.get_logger().info(f"Planner Node initialized at {freq}Hz")

    def target_callback(self, msg: TargetArray):
        self.cur_targets = msg

    def overhead_callback(self, msg: TrajectoryPointMsg):
        self.cur_overhead_points[msg.robot_id] = msg

    def game_state_callback(self, msg: GameState):
        self.game_state = msg

    def planning_loop(self):
        if self.cur_targets is None or self.game_state is None:
            return

        robots_to_plan = []
        for target in self.cur_targets.targets:
            robots_to_plan.append(target)

        # Run planning in parallel
        futures = []
        for target in robots_to_plan:
            futures.append(self.executor.submit(self.plan_for_robot, target))

        # Collect and publish results
        for future in futures:
            try:
                result = future.result()
                if result:
                    robot_id, trajectory = result
                    self.last_planned_trajectories[robot_id] = trajectory
                    msg = trajectory.to_msg(robot_id)
                    self.trajectory_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Planning failed: {e}")

    def plan_for_robot(self, target):
        robot_id = target.robot_id
        
        # Determine Initial State
        # Prioritize overhead (future) state from tracker to prevent jitter
        # TODO prevent use of overhead points too old, like 100ms or something
        if robot_id in self.cur_overhead_points:
            init_pos = self.cur_overhead_points[robot_id].pos
            init_vel = self.cur_overhead_points[robot_id].vel
        else:
            init_pos = target.inital_pos
            init_vel = target.inital_vel

        initial_state = State(
            Vector2D(init_pos.x, init_pos.y),
            Vector2D(init_vel.x, init_vel.y)
        )
            
        target_state = State(
            Vector2D(target.target_pos.x, target.target_pos.y),
            Vector2D(target.target_vel.x, target.target_vel.y)
        )

        # Generate Obstacles
        # Using game_state for geometry, and assuming target carries the toggles
        obstacles = self.factory.create_obstacles(
            robot_id=robot_id,
            config=target,  # Assuming MovementCommand/Target has avoid_penalty_area etc.
            geometry=self.game_state.geometry if self.game_state else None,
            balls=self.game_state.balls if self.game_state else [],
            enemy_robots={r.robot_id: r for r in self.game_state.blue_robots} if self.game_state else {}, # Simplified
            ally_robots={r.robot_id: r for r in self.game_state.yellow_robots} if self.game_state else {}, # Simplified
            ally_trajectories=self.last_planned_trajectories
        )

        # Solve
        try:
            trajectory = self.planner.find(initial_state, target_state, obstacles)
            if trajectory and trajectory.root:
                return robot_id, trajectory
        except Exception as e:
            self.get_logger().warn(f"Solver error for robot {robot_id}: {e}")
            
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    
    # Use multi-threaded executor to handle concurrent callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
