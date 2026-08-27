import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from concurrent.futures import ThreadPoolExecutor

from new_movement.entities.States import State, Vector2D
from new_movement.entities.Trajectory import Trajectory
from new_movement.local_planner import Planner, ObstacleFactory, PlanningStatus

from movement_interfaces.msg import (
    TargetArray,
    Trajectory as TrajectoryMsg,
    TrajectoryPoint as TrajectoryPointMsg,
)
from system_interfaces.msg import GameState

from typing import Dict, Optional
from time import time


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # Parameters
        self.declare_parameter('planner_freq', 50.0)
        self.declare_parameter('max_threads', 8)
        # build_overhead_point stamps a point at now + (sample_time - time_offset), so a
        # fresh one is always stamped in the future: any positive age is already stale.
        # At 0.5s the planner kept trusting a cache the tracker had stopped refreshing.
        self.declare_parameter('overhead_max_age', 0.05)
        self.declare_parameter('accept_radius', 10)
        
        # State
        self.cur_targets: Optional[TargetArray] = None
        self.cur_overhead_points: Dict[int, TrajectoryPointMsg] = {}
        self.game_state: Optional[GameState] = None
        self.active_futures: Dict[int, any] = {} # Track running tasks per robot
        # Last via point per robot, warm-starting the bypass solver. Held here rather
        # than in Planner so the solver stays reentrant across the worker threads.
        self.last_vias: Dict[int, State] = {}
        self._last_warned: Dict[str, float] = {}
        
        # Tools
        self.planner = Planner()
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

    def _warn_throttled(self, key: str, message: str, period: float = 2.0) -> None:
        """One line per key per period: this repeats at the planning rate while it lasts."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_warned.get(key, float("-inf")) < period:
            return
        self._last_warned[key] = now_sec
        self.get_logger().warn(message)

    def plan_for_robot(self, target):
        robot_id = target.robot_id
        
        # from vision
        init_pos = Vector2D(target.initial_pos.x, target.initial_pos.y)
        init_vel = Vector2D(target.initial_vel.x, target.initial_vel.y)

        # handoff_stamp is the instant initial_state refers to, so the tracker starts the
        # plan at the right offset rather than at t=0 whenever it happens to arrive.
        if robot_id in self.cur_overhead_points:
            overhead_point = self.cur_overhead_points[robot_id]
            age = (self.get_clock().now().nanoseconds / 1e9) - overhead_point.wall_stamp
            if age <= self.get_parameter('overhead_max_age').value:
                # Fresh overhead point — plan from predicted future state
                initial_state = State(
                    Vector2D(overhead_point.pos.x, overhead_point.pos.y),
                    Vector2D(overhead_point.vel.x, overhead_point.vel.y)
                )
                handoff_stamp = overhead_point.wall_stamp
            else:
                # Stale overhead — check if robot is already at the goal
                goal_pos = Vector2D(target.target_pos.x, target.target_pos.y)
                if goal_pos.distance(init_pos) < float(self.get_parameter('accept_radius').value):
                    return None  # already there, nothing to plan
                # Far enough to be worth replanning from vision state
                initial_state = State(init_pos, init_vel)
                handoff_stamp = float(target.vision_stamp)
        else:
            initial_state = State(init_pos, init_vel)
            handoff_stamp = float(target.vision_stamp)

        target_state = State(
            Vector2D(target.target_pos.x, target.target_pos.y),
            Vector2D(target.target_vel.x, target.target_vel.y)
            )

        # Generate Obstacles
        # A safety-critical obstacle (field border, penalty area) that fails to build
        # aborts the plan: driving with an incomplete obstacle set is worse than not
        # issuing a new trajectory at all.
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
                    # Neither a direct path nor a bypass cleared the obstacles, so this
                    # is a stop, not a route. It arrives as a very short plan, which is
                    # what the tracker's late-handoff warnings were reporting downstream.
                    self._warn_throttled(
                        f"recovery-{robot_id}",
                        f"No path for robot {robot_id}; stopping instead",
                    )
                # Kept across direct paths rather than cleared with them. An obstacle
                # crossing the line and stepping back off it used to re-roll the route
                # from scratch each time it returned. A via that has gone stale is
                # self-correcting: the solver rebuilds through it from the current start
                # and a sampled route beats it whenever it is genuinely worse.
                if trajectory.via_state is not None:
                    self.last_vias[robot_id] = trajectory.via_state
                return robot_id, trajectory, handoff_stamp
        except Exception as e:
            self.get_logger().warn(f"Solver error for robot {robot_id}: {e}")

        return None


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    
    from rclpy.executors import MultiThreadedExecutor
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
