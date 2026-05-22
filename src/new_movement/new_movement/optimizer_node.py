import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from concurrent.futures import ThreadPoolExecutor

from movement_interfaces.msg import (
    Trajectory as TrajectoryMsg,
    TrajectoryPoint as TrajectoryPointMsg,
)

from time import time
from typing import Dict, Optional


from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator
from new_movement.local_planner.optimizer import TrajectoryOptimizer
from new_movement.local_planner import ObstacleFactory
from new_movement.entities.Trajectory import Trajectory
from system_interfaces.msg import GameState
from movement_interfaces.msg import TargetArray

class OptimizerNode(Node):
    def __init__(self):
        super().__init__('optimizer_node')

        self.declare_parameter('optimizer_freq', 50.0)
        self.declare_parameter('max_threads', 8)

        self.active_durations: Dict[int, float] = {}
        self.active_futures: Dict[int, any] = {}
        self.active_start_times: Dict[int, float] = {}
        self.cur_overhead_points: Dict[int, TrajectoryPointMsg] = {}
        self.pending_trajectories: Dict[int, TrajectoryMsg] = {}

        self.game_state: Optional[GameState] = None
        self.cur_targets: Optional[TargetArray] = None

        self.factory = ObstacleFactory()

        self.optimizer = TrajectoryOptimizer()
        self.generator = TrajectoryGenerator()
        self.par_executor = ThreadPoolExecutor(
            max_workers=self.get_parameter('max_threads').value
        )

        cb_group = ReentrantCallbackGroup()

        self.trajectory_sub = self.create_subscription(
            TrajectoryMsg, 'planner/trajectories', self.trajectory_callback, 10, callback_group=cb_group
        )
        self.overhead_sub = self.create_subscription(
            TrajectoryPointMsg, 'movement_tracker/overhead', self.overhead_callback, 10, callback_group=cb_group
        )
        self.game_state_sub = self.create_subscription(
            GameState, 'game_state', self.game_state_callback, 10, callback_group=cb_group
        )
        self.target_sub = self.create_subscription(
            TargetArray, 'movement_manager/targets', self.target_callback, 10, callback_group=cb_group
        )

        self.trajectory_pub = self.create_publisher(TrajectoryMsg, 'optimizer/trajectories', 10)

        freq = self.get_parameter('optimizer_freq').value
        self.timer = self.create_timer(1.0 / freq, self.optimization_loop, callback_group=cb_group)

        self.get_logger().info(f"Optimizer Node initialized at {freq}Hz")



    def trajectory_callback(self, msg: TrajectoryMsg):
        self.pending_trajectories[msg.robot_id] = msg

    def overhead_callback(self, msg: TrajectoryPointMsg):
        self.cur_overhead_points[msg.robot_id] = msg

    def optimization_loop(self):
        if not self.pending_trajectories:
            return

        for robot_id, trajectory_msg in self.pending_trajectories.items():
            if robot_id in self.active_futures and not self.active_futures[robot_id].done():
                continue

            future = self.par_executor.submit(self.optimize_for_robot, robot_id, trajectory_msg)
            self.active_futures[robot_id] = future
            future.add_done_callback(self.make_publish_callback(robot_id))

    def get_duration_from_msg(self,trajectory_msg: TrajectoryMsg) -> float:
        total = 0.0
        for segment in trajectory_msg.segments:
            for primitive in segment.motion_path.primitives:
                total += primitive.duration
        return total

    def get_remaining_time(self, robot_id: int) -> Optional[float]:
        if robot_id not in self.active_durations:
            return 0.0
        elapsed = time() - self.active_start_times[robot_id]
        remaining = self.active_durations[robot_id] - elapsed
        return max(remaining, 0.0)

    def optimize_for_robot(self, robot_id: int, trajectory_msg: TrajectoryMsg) -> TrajectoryMsg:
        new_duration = self.get_duration_from_msg(trajectory_msg)
        remaining_current = self.get_remaining_time(robot_id)

        if remaining_current <= new_duration:
            return None

        if self.cur_targets is None:
            return None

        target = next(
            (t for t in self.cur_targets.targets if t.robot_id == robot_id), None
        )

        if target is None:
            return None

        obstacles = self.factory.create_obstacles(
            robot_id=robot_id,
            config=target,
            geometry=self.game_state.geometry if self.game_state else None,
            balls=self.game_state.balls if self.game_state else [],
            enemy_robots=self.game_state.enemy_robots if self.game_state else [],
            ally_robots=self.game_state.ally_robots if self.game_state else [],
            ally_info=self.cur_overhead_points
        )

        trajectory = Trajectory.from_msg(trajectory_msg)
        optimized =  self.optimizer.optimize(trajectory, self.generator, obstacles)

        if not optimized or not optimized.root:
            return None

        return robot_id, optimized

    def make_publish_callback(self, robot_id):
        def callback(future):
            try:
                result = future.result()
                if result is None:
                    return

                _, optimized = result

                self.active_durations[robot_id] = optimized.get_total_duration()
                self.active_start_times[robot_id] = time()

                msg = optimized.to_msg(robot_id)
                self.trajectory_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Optimization failed for robot {robot_id}: {e}")

        return callback

    def game_state_callback(self, msg: GameState):
        self.game_state = msg

    def target_callback(self, msg: TargetArray):
        self.cur_targets = msg

def main(args=None):
    rclpy.init(args=args)
    node = OptimizerNode()

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