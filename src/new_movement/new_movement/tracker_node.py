import copy
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from movement_interfaces.msg import Trajectory as TrajectoryMsg
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg

from new_movement.entities.Trajectory import Trajectory
from new_movement.entities.States import State


def build_overhead_point(
    robot_id: int,
    trajectory_msg: Optional[TrajectoryMsg],
    trajectory: Trajectory,
    time_offset: float,
    lookahead_time: float,
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
    point.trajectory = trajectory_msg
    return point


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

        self.declare_parameter("tracker_freq", 100.0)
        self.declare_parameter("lookahead_time", 0.2)
        self.declare_parameter("trajectory_topic", "planner/trajectories")
        self.declare_parameter("overhead_topic", "movement_tracker/overhead")
        self.declare_parameter(
            "control_reference_topic", "movement_tracker/control_reference"
        )

        self.robot_data: Dict[int, Dict[str, object]] = {}
        self.last_time = self.get_clock().now()

        self.traj_sub = self.create_subscription(
            TrajectoryMsg,
            self.get_parameter("trajectory_topic").value,
            self.trajectory_callback,
            10,
        )
        self.overhead_pub = self.create_publisher(
            TrajectoryPointMsg, self.get_parameter("overhead_topic").value, 10
        )
        self.control_reference_pub = self.create_publisher(
            TrajectoryPointMsg,
            self.get_parameter("control_reference_topic").value,
            10,
        )

        freq = float(self.get_parameter("tracker_freq").value)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

        self.get_logger().info(
            "TrackerNode online: in=%s overhead=%s control_ref=%s lookahead=%.3fs freq=%.1fHz"
            % (
                self.get_parameter("trajectory_topic").value,
                self.get_parameter("overhead_topic").value,
                self.get_parameter("control_reference_topic").value,
                float(self.get_parameter("lookahead_time").value),
                freq,
            )
        )

    def trajectory_callback(self, msg: TrajectoryMsg):
        if not msg.segments:
            self.get_logger().debug(
                "Ignoring empty trajectory for robot %s" % msg.robot_id
            )
            return

        try:
            trajectory = Trajectory.from_msg(msg)
        except Exception as exc:
            self.get_logger().warn(
                "Failed to decode trajectory for robot %s: %s" % (msg.robot_id, exc)
            )
            return

        self.robot_data[int(msg.robot_id)] = {
            "trajectory": trajectory,
            "trajectory_msg": copy.deepcopy(msg),
            "time_offset": 0.0,
        }

        self.get_logger().debug(
            "Trajectory updated for robot %s (segments=%d, duration=%.3fs)"
            % (msg.robot_id, len(msg.segments), trajectory.get_total_duration())
        )

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        lookahead = float(self.get_parameter("lookahead_time").value)
        for robot_id, data in self.robot_data.items():
            trajectory = data.get("trajectory")
            if trajectory is None:
                continue

            total_duration = trajectory.get_total_duration()
            if total_duration <= 0:
                continue

            time_offset = float(data.get("time_offset", 0.0))

            current_state = trajectory.get_state(time_offset)
            control_ref = build_control_reference_point(
                robot_id,
                data.get("trajectory_msg"),
                current_state,
                time_offset,
            )
            if control_ref is not None:
                self.control_reference_pub.publish(control_ref)

            overhead_point = build_overhead_point(
                robot_id,
                data.get("trajectory_msg"),
                trajectory,
                time_offset,
                lookahead,
            )
            if overhead_point is not None:
                self.overhead_pub.publish(overhead_point)

            if time_offset < total_duration:
                time_offset = min(time_offset + dt, total_duration)
            else:
                time_offset = total_duration
            data["time_offset"] = time_offset




def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
