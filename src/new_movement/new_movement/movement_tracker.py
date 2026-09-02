import copy
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from movement_interfaces.msg import Trajectory as TrajectoryMsg
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg
from movement_interfaces.msg import GUITrajectories

from new_movement.entities.trajectory.trajectory import Trajectory
from new_movement.entities.motion.motion_state import MotionState


class MovementTracker(Node):
    def __init__(self):
        super().__init__("movement_tracker")

        self.declare_parameter("lookahead_time", 0.2)
        self.declare_parameter("improvement_threshold", 0.1)
        self.declare_parameter(
            "control_reference_topic", "movement_tracker/control_reference"
        )
        self.declare_parameter('change_radius', 10)

        self.robot_data: Dict[int, Dict[str, object]] = {}
        self.last_time = self.get_clock().now()

        self.traj_sub = self.create_subscription(TrajectoryMsg, "planner/trajectories", self.trajectory_callback, 10)
        
        self.overhead_pub = self.create_publisher(TrajectoryPointMsg, "movement_tracker/overhead", 10)
        self.control_reference_pub = self.create_publisher(TrajectoryPointMsg, "movement_tracker/control_reference", 10)
        self.gui_trajectories_pub = self.create_publisher(GUITrajectories, "gui/trajectories", 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.gui_timer = self.create_timer(0.1, self._update_gui_trajectories)

        self.get_logger().info("TrackerNode ONLINE")

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
        if total_duration <= 0:
            return

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
        if new_offset < pending_traj.get_total_duration():
            data["trajectory"] = pending_traj
            data["trajectory_msg"] = pending["trajectory_msg"]
            data["time_offset"] = new_offset

            if handoff_stamp != 0.0 and dt_late > lookahead * 0.5:
                self.get_logger().warn(
                    f"Late handoff for robot {robot_id}: {dt_late:.3f}s "
                    f"(lookahead was {lookahead:.3f}s)"
                )
        else:
            if pending_traj.get_total_duration() > 1e-3:
                self.get_logger().warn(
                    f"Discarding fully-elapsed pending trajectory for robot {robot_id} "
                    f"(late by {dt_late:.3f}s, duration was {pending_traj.get_total_duration():.3f}s)"
                )
        data["pending"] = None

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
            time_offsets.append(data.get("time_offset", 0.0))

        msg.current_trajectories = current_trajectories
        msg.pending_trajectories = pending_trajectories
        msg.time_offsets = time_offsets
        self.gui_trajectories_pub.publish(msg)


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


def build_control_reference_point(
    robot_id: int,
    trajectory_msg: Optional[TrajectoryMsg],
    state: Optional[MotionState],
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

def main(args=None):
    rclpy.init(args=args)
    node = MovementTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
