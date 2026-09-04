from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from concurrent.futures import ThreadPoolExecutor

from movement.entities.motion.motion_state import MotionState
from movement.local_planner.orchestrator import Orchestrator
from movement.local_planner.obstacle_factory import ObstacleFactory
from movement.local_planner.solver import PlanningStatus

from movement_interfaces.msg import (
    TargetArray,
    Trajectory as TrajectoryMsg,
    TrajectoryPoint as TrajectoryPointMsg,
)
from system_interfaces.msg import GameState

from utils.math_util import Vector2D

PARK_RELEASE_FACTOR = 2.0
GOAL_MOVED_EPSILON = 1.0
MAX_VISION_AGE = 0.2

class MovementPlanner(Node):
    def __init__(self):
        super().__init__('movement_planner')
        
        # Parameters
        self.declare_parameter('planner_freq', 50.0)
        self.declare_parameter('max_threads', 1)
        self.declare_parameter('overhead_max_age', 0.05)
        self.declare_parameter('accept_radius', 50.0)
        
        # MotionState
        self.cur_targets: Optional[TargetArray] = None
        self.cur_overhead_points: Dict[int, TrajectoryPointMsg] = {}
        self.game_state: Optional[GameState] = None
        self.active_futures: Dict[int, any] = {} # Track running tasks per robot
        self.last_vias: Dict[int, MotionState] = {}
        self._last_warned: Dict[str, float] = {}
        self._parked: Dict[int, Vector2D] = {}
        
        # Tools
        self.planner = Orchestrator()
        self.factory = ObstacleFactory(logger=self.get_logger())
        self.par_executor = ThreadPoolExecutor(max_workers=self.get_parameter('max_threads').value)
        
        # ROS Communication
        cb_group = ReentrantCallbackGroup()
        
        # Subscribers
        self.target_sub = self.create_subscription(
            TargetArray, 'movement_manager/targets', self.target_callback, 10, callback_group=cb_group)
        self.overhead_sub = self.create_subscription(
            TrajectoryPointMsg, 'movement_tracker/overhead', self.overhead_callback, 10, callback_group=cb_group)
        self.game_state_sub = self.create_subscription(
            GameState, 'game_state', self.game_state_callback, 10, callback_group=cb_group)
            
        # Publisher
        self.trajectory_pub = self.create_publisher(TrajectoryMsg, 'planner/trajectories', 10)
        
        # Timer
        freq = self.get_parameter('planner_freq').value
        self.timer = self.create_timer(1.0 / freq, self.planning_loop, callback_group=cb_group)
        
        self.get_logger().info(f"movement_planner initialized at {freq}Hz")

    def target_callback(self, msg: TargetArray):
        self.cur_targets = msg

    def overhead_callback(self, msg: TrajectoryPointMsg):
        self.cur_overhead_points[msg.robot_id] = msg

    def game_state_callback(self, msg: GameState):
        self.game_state = msg

    def planning_loop(self):
        if self.cur_targets is None or self.game_state is None:
            return

        for target in self.cur_targets.targets:
            rid = target.robot_id
            
            # THROTTLING: If a task is already running for this robot, skip this cycle
            if rid in self.active_futures and not self.active_futures[rid].done():
                continue

            future = self.par_executor.submit(self.plan_for_robot, target)
            self.active_futures[rid] = future
            future.add_done_callback(self.make_publish_callback(rid))

    def make_publish_callback(self, robot_id):
        def callback(future):
            try:
                # Discard if robot is no longer an active target
                if self.cur_targets:
                    current_ids = {t.robot_id for t in self.cur_targets.targets}
                    if robot_id not in current_ids:
                        return

                result = future.result()
                if result:
                    _, trajectory, handoff_stamp = result
                    msg = trajectory.to_msg(robot_id)
                    msg.handoff_stamp = handoff_stamp
                    self.trajectory_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Planning failed for robot {robot_id}: {e}")
        return callback

    def _state_from_vision(self, target):
        position = Vector2D(target.initial_pos.x, target.initial_pos.y)
        velocity = Vector2D(target.initial_vel.x, target.initial_vel.y)
        stamp = float(target.vision_stamp)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        age = now_sec - stamp
        if stamp <= 0.0 or age < 0.0 or age > MAX_VISION_AGE:
            return MotionState(position, velocity), stamp

        carried = Vector2D(
            position.x + velocity.x * age, position.y + velocity.y * age
        )
        return MotionState(carried, velocity), now_sec

    def _is_parked(self, robot_id: int, goal_pos: Vector2D, measured_pos: Vector2D) -> bool:
        radius = float(self.get_parameter('accept_radius').value)
        parked_at = self._parked.get(robot_id)

        if parked_at is not None and parked_at.distance(goal_pos) < GOAL_MOVED_EPSILON:
            if measured_pos.distance(goal_pos) <= radius * PARK_RELEASE_FACTOR:
                return True
            # Drifted out far enough to be worth correcting.
            del self._parked[robot_id]
            return False

        # A different goal, so this is a new move.
        self._parked.pop(robot_id, None)
        if measured_pos.distance(goal_pos) <= radius:
            self._parked[robot_id] = goal_pos
            return True
        return False

    def plan_for_robot(self, target):
        robot_id = target.robot_id
        
        init_pos = Vector2D(target.initial_pos.x, target.initial_pos.y)
        goal_pos = Vector2D(target.target_pos.x, target.target_pos.y)

        if self._is_parked(robot_id, goal_pos, init_pos):
            return None

        if robot_id in self.cur_overhead_points:
            overhead_point = self.cur_overhead_points[robot_id]
            age = (self.get_clock().now().nanoseconds / 1e9) - overhead_point.wall_stamp
            if age <= self.get_parameter('overhead_max_age').value:
                # Plan from predicted future state
                initial_state = MotionState(
                    Vector2D(overhead_point.pos.x, overhead_point.pos.y),
                    Vector2D(overhead_point.vel.x, overhead_point.vel.y)
                )
                handoff_stamp = overhead_point.wall_stamp
            else:
                # The cached prediction has aged out, so go back to what vision measured.
                initial_state, handoff_stamp = self._state_from_vision(target)
        else:
            initial_state, handoff_stamp = self._state_from_vision(target)

        target_state = MotionState(
            Vector2D(target.target_pos.x, target.target_pos.y),
            Vector2D(target.target_vel.x, target.target_vel.y)
            )

        try:
            obstacles = self.factory.create_obstacles(
                robot_id=robot_id,
                config=target,
                geometry=self.game_state.geometry if self.game_state else None,
                balls=self.game_state.balls if self.game_state else [],
                enemy_robots=self.game_state.enemy_robots if self.game_state else [],
                ally_robots=self.game_state.ally_robots if self.game_state else [],
                ally_info=self.cur_overhead_points
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not build the obstacle set for robot {robot_id}, skipping plan: {e}"
            )
            return None

        # Solve
        previous_via = self.last_vias.get(robot_id)
        try:
            trajectory = self.planner.find(
                initial_state, target_state, obstacles, previous_via
            )
            if trajectory and trajectory.root:
                if trajectory.status == PlanningStatus.RECOVERY:
                    self.get_logger().warn(f"Recovery for robot {robot_id}")
                if trajectory.via_state is not None:
                    self.last_vias[robot_id] = trajectory.via_state
                return robot_id, trajectory, handoff_stamp
        except Exception as e:
            self.get_logger().warn(f"Solver error for robot {robot_id}: {e}")

        return None


def main(args=None):
    rclpy.init(args=args)
    node = MovementPlanner()
    
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(node)
    
    try:
        ros_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

