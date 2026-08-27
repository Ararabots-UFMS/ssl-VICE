import copy
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from movement_interfaces.msg import Trajectory as TrajectoryMsg
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg
from movement_interfaces.msg import GUITrajectories
from system_interfaces.msg import GameState

from new_movement.entities.Trajectory import Trajectory
from new_movement.entities.States import State, Vector2D


def build_overhead_point(
    robot_id: int,
    trajectory_msg: Optional[TrajectoryMsg],
    trajectory: Trajectory,
    time_offset: float,
    lookahead_time: float,
    now_sec: float,
) -> Optional[TrajectoryPointMsg]:
    if trajectory is None or trajectory_msg is None:
        return None

    total_duration = trajectory.get_total_duration()
    if total_duration <= 0:
        return None

    if lookahead_time < 0:
        lookahead_time = 0.0

    sample_time = min(time_offset + lookahead_time, total_duration)
    state = trajectory.get_state(sample_time)
    if state is None:
        return None

    point = TrajectoryPointMsg()
    point.robot_id = int(robot_id)
    point.pos = state.position.to_msg()
    point.vel = state.velocity.to_msg()
    point.timestamp = float(sample_time)
    point.wall_stamp = now_sec + (sample_time - time_offset)
    point.trajectory = trajectory_msg
    return point


def tracking_error(
    trajectory: Trajectory,
    time_offset: float,
    measured_pos: Vector2D,
    measurement_age: float,
) -> Optional[Tuple[float, float]]:
    """
    Along- and cross-track error at the instant the measurement was taken.

    Comparing vision against the reference at time_offset would report the vision delay
    as tracking error, so the reference is evaluated measurement_age seconds back.
    Along-track is signed: negative means the robot is behind its reference. Returns
    None when the measurement predates the trajectory.
    """
    t_meas = time_offset - measurement_age
    if t_meas < 0.0 or t_meas > trajectory.get_total_duration():
        return None

    reference = trajectory.get_state(t_meas)
    if reference is None:
        return None

    dx = measured_pos.x - reference.position.x
    dy = measured_pos.y - reference.position.y

    speed = math.hypot(reference.velocity.x, reference.velocity.y)
    if speed < 1e-6:
        # No heading to project onto; report it all as cross-track.
        return 0.0, math.hypot(dx, dy)

    ux, uy = reference.velocity.x / speed, reference.velocity.y / speed
    return dx * ux + dy * uy, dx * -uy + dy * ux


def build_control_reference_point(
    robot_id: int,
    trajectory_msg: Optional[TrajectoryMsg],
    state: Optional[State],
    time_offset: float,
) -> Optional[TrajectoryPointMsg]:
    if state is None or trajectory_msg is None:
        return None

    point = TrajectoryPointMsg()
    point.robot_id = int(robot_id)
    point.pos = state.position.to_msg()
    point.vel = state.velocity.to_msg()
    point.timestamp = float(time_offset)
    point.trajectory = trajectory_msg
    return point


class TrackerNode(Node):
    def __init__(self):
        super().__init__("movement_tracker")

        # Derived from the measured handoff latency (p95 ~50ms, max ~130ms) rather than
        # guessed: it has to cover the latency, and everything above that is extra
        # extrapolation the robot has to make good on.
        self.declare_parameter("lookahead_time", 0.15)
        self.declare_parameter("improvement_threshold", 0.1)
        self.declare_parameter(
            "control_reference_topic", "movement_tracker/control_reference"
        )
        self.declare_parameter('change_radius', 10)
        # Measured transients reach ~350mm in healthy tracking, so this sits above them:
        # 400mm is more than two robot diameters off the path, which is a real departure
        # rather than a bad frame. Debounced so noise cannot trip it.
        self.declare_parameter('divergence_radius', 400.0)
        self.declare_parameter('divergence_frames', 3)
        # Asymmetric on purpose: quick to distrust the prediction, slow to trust it
        # again. A single good frame used to clear the flag, which handed planning
        # straight back to the prediction that had just failed.
        self.declare_parameter('recovery_frames', 10)

        self.robot_data: Dict[int, Dict[str, object]] = {}
        self.last_time = self.get_clock().now()

        # dt_late per activated plan: the measured latency lookahead_time should follow.
        self._handoff_latencies: List[float] = []
        # (along, cross) per robot per vision frame, for sizing the divergence threshold.
        self._tracking_errors: Dict[int, List[Tuple[float, float]]] = {}
        self._divergence_streak: Dict[int, int] = {}
        self._recovery_streak: Dict[int, int] = {}
        self._diverged: Dict[int, bool] = {}

        self.traj_sub = self.create_subscription(TrajectoryMsg, "planner/trajectories", self.trajectory_callback, 10)
        self.game_state_sub = self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

        self.overhead_pub = self.create_publisher(TrajectoryPointMsg, "movement_tracker/overhead", 10)
        self.control_reference_pub = self.create_publisher(TrajectoryPointMsg, "movement_tracker/control_reference", 10)
        self.gui_trajectories_pub = self.create_publisher(GUITrajectories, "gui/trajectories", 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.gui_timer = self.create_timer(0.1, self._update_gui_trajectories)
        self.diagnostics_timer = self.create_timer(5.0, self._log_diagnostics)

        self.get_logger().info("TrackerNode ONLINE")

    def game_state_callback(self, msg: GameState):
        # Sampled here rather than in the timer so there is one measurement per vision
        # frame, not one per tracker tick over the same stale frame.
        now_sec = self.get_clock().now().nanoseconds / 1e9
        measurement_age = now_sec - msg.vision_wall_stamp

        for robot in msg.ally_robots:
            data = self.robot_data.get(robot.id)
            if data is None or data.get("trajectory") is None:
                continue

            error = tracking_error(
                data["trajectory"],
                float(data.get("time_offset", 0.0)),
                Vector2D(robot.position_x, robot.position_y),
                measurement_age,
            )
            if error is not None:
                self._tracking_errors.setdefault(robot.id, []).append(error)
                self._update_divergence(robot.id, error)

    def _update_divergence(self, robot_id: int, error: Tuple[float, float]) -> None:
        """
        Flags a robot that is no longer on the path it was given.

        The plan it is following was built from a prediction, and once the robot has
        left the path that prediction has stopped describing it — so every plan built
        on top of it is wrong too. Suppressing the overhead point (see
        _update_active_trajectory) is what breaks that: the planner's cached point ages
        out within overhead_max_age and it falls back to the measured state.
        """
        radius = float(self.get_parameter('divergence_radius').value)
        enter_after = int(self.get_parameter('divergence_frames').value)
        leave_after = int(self.get_parameter('recovery_frames').value)

        if math.hypot(*error) > radius:
            self._divergence_streak[robot_id] = self._divergence_streak.get(robot_id, 0) + 1
            self._recovery_streak[robot_id] = 0
        else:
            self._recovery_streak[robot_id] = self._recovery_streak.get(robot_id, 0) + 1
            self._divergence_streak[robot_id] = 0

        if self._diverged.get(robot_id, False):
            if self._recovery_streak.get(robot_id, 0) >= leave_after:
                self._diverged[robot_id] = False
                self.get_logger().info(f"Robot {robot_id} is back on its path")
        elif self._divergence_streak.get(robot_id, 0) >= enter_after:
            self._diverged[robot_id] = True
            self.get_logger().warn(
                f"Robot {robot_id} has left its path "
                f"(along {error[0]:.0f}mm, cross {error[1]:.0f}mm) — "
                f"replanning from the measured state"
            )

    def trajectory_callback(self, msg: TrajectoryMsg):
        if not msg.segments:
            return

        try:
            trajectory = Trajectory.from_msg(msg)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to decode trajectory for robot {msg.robot_id}: {exc}"
            )
            return

        rid = int(msg.robot_id)
        data = self.robot_data.setdefault(rid, {})

        pending = data.get("pending")
        
        if pending is None:
            data["pending"] = {
                "trajectory": trajectory,
                "trajectory_msg": copy.deepcopy(msg),
                "handoff_stamp": msg.handoff_stamp,
            }
            return
        if msg.handoff_stamp < pending["handoff_stamp"]:
            return  # strictly older handoff — discard

        dist_goal = trajectory.get_destination().position.distance(pending["trajectory"].get_destination().position)
        goal_changed = dist_goal > float(self.get_parameter('change_radius').value)  # mm

        if goal_changed:
            data["pending"] = {
                "trajectory": trajectory,
                "trajectory_msg": copy.deepcopy(msg),
                "handoff_stamp": msg.handoff_stamp,
            }
            return

        # Same goal: compare estimated arrival times
        pending_arrival = pending["trajectory"].get_total_duration() - (msg.handoff_stamp - pending["handoff_stamp"])
        new_arrival     = trajectory.get_total_duration()
        improvement     = pending_arrival - new_arrival

        threshold = float(self.get_parameter("improvement_threshold").value)

        if pending_arrival > 0 and (improvement / pending_arrival) >= threshold:
            data["pending"] = {
                "trajectory": trajectory,
                "trajectory_msg": copy.deepcopy(msg),
                "handoff_stamp": msg.handoff_stamp,
            }

    def timer_callback(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            return
        self.last_time = now

        lookahead = float(self.get_parameter("lookahead_time").value)

        for robot_id, data in self.robot_data.items():
            self._update_active_trajectory(robot_id, data, dt, now_sec, lookahead)
            self._handle_pending_handoff(robot_id, data, now_sec, lookahead)

    def _update_active_trajectory(self, robot_id, data, dt, now_sec, lookahead):
        trajectory = data.get("trajectory")
        if trajectory is None:
            return

        total_duration = trajectory.get_total_duration()
        time_offset = float(data.get("time_offset", 0.0))

        # Publish Control Reference
        current_state = trajectory.get_state(time_offset)
        control_ref = build_control_reference_point(
            robot_id,
            data.get("trajectory_msg"),
            current_state,
            time_offset,
        )
        if control_ref is not None:
            self.control_reference_pub.publish(control_ref)

        # A zero-duration plan means the robot is already at the goal. Its reference
        # still has to go out — bailing before that left the controller chasing the
        # previous, moving one — but there is nothing to predict or advance.
        if total_duration <= 0:
            return

        # Withheld while the robot is off its path, which does two things: the planner's
        # cached point ages out and it replans from the measured state, and the reference
        # stops advancing away from a robot that is not following it.
        if self._diverged.get(robot_id, False):
            return

        # Publish Overhead Point

        if time_offset < total_duration:
            overhead_point = build_overhead_point(
                robot_id,
                data.get("trajectory_msg"),
                trajectory,
                time_offset,
                lookahead,
                now_sec,
            )
            if overhead_point is not None:
                self.overhead_pub.publish(overhead_point)

        # Update Time Offset
        if time_offset < total_duration:
            new_offset = min(time_offset + dt, total_duration)
        else:
            new_offset = total_duration
        data["time_offset"] = new_offset

    def _handle_pending_handoff(self, robot_id, data, now_sec, lookahead):
        pending = data.get("pending")
        if pending is None:
            return

        handoff_stamp = pending["handoff_stamp"]
        dt_late = now_sec - handoff_stamp

        if handoff_stamp == 0.0:
            new_offset = 0.0
        elif dt_late >= 0:
            new_offset = dt_late
        else:
            # Not time yet
            return

        pending_traj = pending["trajectory"]
        duration = pending_traj.get_total_duration()

        if handoff_stamp != 0.0:
            self._handoff_latencies.append(dt_late)

        # Clamped, not discarded: a late plan still ends at the current goal, while the
        # one it replaces ends at the old one.
        if new_offset >= duration:
            new_offset = duration
            if duration > 1e-3:
                self.get_logger().warn(
                    f"Handoff for robot {robot_id} arrived {dt_late:.3f}s late, past its "
                    f"{duration:.3f}s duration — activating at the goal"
                )
        elif handoff_stamp != 0.0 and dt_late > lookahead * 0.5:
            self.get_logger().warn(
                f"Late handoff for robot {robot_id}: {dt_late:.3f}s "
                f"(lookahead was {lookahead:.3f}s)"
            )

        data["trajectory"] = pending_traj
        data["trajectory_msg"] = pending["trajectory_msg"]
        data["time_offset"] = float(new_offset)
        data["pending"] = None

        # Streaks restart against the new trajectory, but the flag does not: it clears
        # on measured recovery only. Clearing it here resumed the overhead point at the
        # moment of handoff, so planning went back to the prediction that had just
        # failed and the correction never survived a single cycle.
        self._divergence_streak[robot_id] = 0
        self._recovery_streak[robot_id] = 0

    def _log_diagnostics(self):
        self._log_handoff_latency()
        self._log_tracking_error()

    def _log_handoff_latency(self):
        if not self._handoff_latencies:
            return

        samples = sorted(self._handoff_latencies)
        self._handoff_latencies = []
        p95 = samples[min(len(samples) - 1, int(0.95 * len(samples)))]

        self.get_logger().info(
            f"Handoff latency over {len(samples)} plans: "
            f"mean {1000 * sum(samples) / len(samples):.1f}ms, "
            f"p95 {1000 * p95:.1f}ms, max {1000 * samples[-1]:.1f}ms"
        )

    def _log_tracking_error(self):
        errors, self._tracking_errors = self._tracking_errors, {}

        for robot_id, samples in sorted(errors.items()):
            if not samples:
                continue

            along = [sample[0] for sample in samples]
            cross = [abs(sample[1]) for sample in samples]

            self.get_logger().info(
                f"Robot {robot_id} tracking error over {len(samples)} frames: "
                f"along mean {sum(along) / len(along):.1f}mm worst {min(along):.1f}mm, "
                f"cross mean {sum(cross) / len(cross):.1f}mm worst {max(cross):.1f}mm"
            )

    def _update_gui_trajectories(self):
        msg = GUITrajectories()
        current_trajectories = []
        pending_trajectories = []
        time_offsets = []

        for robot_id, data in self.robot_data.items():
            traj_msg = data.get("trajectory_msg")
            if not traj_msg or not traj_msg.segments:
                continue  # robot exists in dict but no trajectory activated yet

            current_trajectories.append(traj_msg)
            pending = data.get("pending")
            pending_trajectories.append(
                pending["trajectory_msg"] if pending else TrajectoryMsg()
            )
            time_offsets.append(float(data.get("time_offset", 0.0)))

        msg.current_trajectories = current_trajectories
        msg.pending_trajectories = pending_trajectories
        msg.time_offsets = time_offsets
        self.gui_trajectories_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
