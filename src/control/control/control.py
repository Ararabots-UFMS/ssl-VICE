import rclpy
from rclpy.node import Node

from movement.entities.motion import MotionState

from utils.math_util import Vector2D

from movement.entities.trajectory.trajectory import Trajectory
from movement.movement_tracker import reference_state

from control.p_controller import PController
from control.pid_controller import DEFAULT_SLEW_LIMIT, RobotTrajectoryController
from control.state_predictor import RobotStatePredictor
from system_interfaces.msg import GameState, RobotCommand, TeamCommand
from system_interfaces.srv import (
    ControlParams,
    GetGameConfig,
    SetKp,
    SetOrientation,
    UpdateKick,
)
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg


# Cap on how far a measurement is carried forward. Beyond this vision has stopped
# arriving, and extrapolating a stale pose is worse than acting on it as it stands.
MAX_MEASUREMENT_AGE = 0.2

CONTROL_MODES = ("legacy", "predictive")
# predictive: the same cap for the predictor, which rolls on the commands sent rather
# than a constant velocity, so it can be trusted a little further.
MAX_PREDICTION_AGE = 0.25
# predictive: how stale a control reference may be before its age stops counting.
MAX_REFERENCE_AGE = 0.1
# predictive: a robot's predictor is dropped after this long without a new capture.
PREDICTOR_TIMEOUT = 1.0


class Controller(Node):
    """Simplified controller node.

    Consumes high-frequency GameState and control-reference messages, publishes TeamCommand.
    Fetches low-frequency configuration (team color) once via GetGameConfig service.
    """

    def __init__(self):
        super().__init__("controller")
        # information about the game state
        self.referee_command = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

        self._desired_states = ["TIMEOUT_BLUE", "TIMEOUT_YELLOW", "HALT"]

        # Caches
        self.ally_robots = {}
        self.vision_wall_stamp = 0.0
        self.control_references = {}
        self.is_halt = None
        # Low-frequency config (polled periodically)
        self.is_team_color_yellow = False
        self.is_field_side_left = False
        self._config_client = self.create_client(GetGameConfig, "get_game_config")
        self._config_call_inflight = False
        self._last_config = None
        # Kick cache por robô: mantém o último valor até nova atualização
        self.kick_cache: dict[int, float] = {}

        # Poll game config every 0.5s for robustness when values change
        self.create_timer(0.5, self._poll_game_config)

        # legacy: compare the reference with the last vision pose carried forward at the
        # EKF velocity. predictive: compare both at the instant the command takes
        # effect, the robot's state from a predictor that knows the commands sent.
        params = {
            "control_mode": "predictive",
            "actuation_delay": 0.04,          # s, command published -> robot acts on it
            "velocity_time_constant": 0.08,   # s, the robot's own velocity loop
            "robot_max_acceleration": 3.0,    # m/s², what the wheels deliver (predictor)
            "slew_limit": DEFAULT_SLEW_LIMIT, # m/s², controller output
            "vision_extra_latency": 0.0,      # s, capture latency vision_wall_stamp misses
            "coast_max_time": 0.3,            # s, as the tracker's
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.control_mode = self.get_parameter("control_mode").value
        if self.control_mode not in CONTROL_MODES:
            raise ValueError(
                f"control_mode must be one of {CONTROL_MODES}, got {self.control_mode!r}"
            )
        self.actuation_delay = float(self.get_parameter("actuation_delay").value)
        self.tau = float(self.get_parameter("velocity_time_constant").value)
        self.robot_max_acceleration = float(
            self.get_parameter("robot_max_acceleration").value
        )
        self.vision_extra_latency = float(self.get_parameter("vision_extra_latency").value)
        self.coast_max_time = float(self.get_parameter("coast_max_time").value)
        self.predictors: dict[int, RobotStatePredictor] = {}
        self._last_capture_seen: dict[int, float] = {}  # rid -> ROS time of last new capture
        self._reference_trajectories: dict[int, tuple] = {}  # rid -> (key, Trajectory)

        # Controllers
        self.robot_controller = RobotTrajectoryController()
        self.robot_controller.default_slew_limit = float(self.get_parameter("slew_limit").value)
        self.orientation_controller = PController(kp=4, max_output=6)
        self.target_orientations = {}

        # ROS Interfaces
        self.create_subscription(
            TrajectoryPointMsg,
            "movement_tracker/control_reference",
            self.receive_control_reference,
            10,
        )
        self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)
        self.create_service(ControlParams, "update_pid", self.update_parameters)
        self.create_service(
            SetOrientation, "set_orientation", self.set_orientation_callback
        )
        self.create_service(SetKp, "update_kp_angular", self.update_kp_angular_callback)
        self.create_service(UpdateKick, "update_kick", self.update_kick_callback)

        # Timing
        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f"control_mode={self.control_mode}, actuation_delay={self.actuation_delay}, "
            f"tau={self.tau}, robot_max_acceleration={self.robot_max_acceleration}, "
            f"slew_limit={self.robot_controller.default_slew_limit}"
        )

    def receive_control_reference(self, msg: TrajectoryPointMsg):
        self.control_references[msg.robot_id] = msg

    def _poll_game_config(self):
        if self._config_call_inflight:
            return
        if not self._config_client.service_is_ready():
            return
        self._config_call_inflight = True
        future = self._config_client.call_async(GetGameConfig.Request())

        def done(fut):
            self._config_call_inflight = False
            try:
                resp = fut.result()
                # Log only on changes
                changed = (
                    self._last_config is None
                    or self._last_config.is_team_color_yellow
                    != resp.is_team_color_yellow
                    or getattr(self._last_config, "is_field_side_left", None)
                    != getattr(resp, "is_field_side_left", None)
                )
                self.is_team_color_yellow = bool(resp.is_team_color_yellow)
                self.is_field_side_left = bool(
                    getattr(resp, "is_field_side_left", False)
                )
                self._last_config = resp
                if changed:
                    self.get_logger().info(
                        f"GameConfig updated: is_team_color_yellow={self.is_team_color_yellow}, is_field_side_left={self.is_field_side_left}"
                    )
            except Exception as e:
                self.get_logger().error(f"Failed to call get_game_config service: {e}")

        future.add_done_callback(done)

    def timer_callback(self):
        if self.control_mode == "predictive":
            self._predictive_timer_callback()
        else:
            self._legacy_timer_callback()

    def _legacy_timer_callback(self):
        if not self.control_references:
            return

        # config is polled periodically in _poll_game_config

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # The reference is for right now, the measurement is from whenever vision last
        # saw the robot. Comparing them directly reports the travel in between as
        # position error, which always points forward and so always asks for more speed.
        measurement_age = (now.nanoseconds / 1e9) - self.vision_wall_stamp
        measurement_age = min(max(measurement_age, 0.0), MAX_MEASUREMENT_AGE)

        team_cmd = TeamCommand()
        team_cmd.is_team_color_yellow = self.is_team_color_yellow
        team_cmd.robots = []

        for rid, ref in self.control_references.items():
            if rid not in self.ally_robots:
                continue

            cur = self.ally_robots[rid]
            cur_state = MotionState(
                Vector2D(
                    (cur.position_x + cur.velocity_x * measurement_age) / 1000.0,
                    (cur.position_y + cur.velocity_y * measurement_age) / 1000.0,
                ),
                Vector2D(cur.velocity_x / 1000.0, cur.velocity_y / 1000.0),
            )
            tgt_state = MotionState(
                Vector2D(ref.pos.x / 1000.0, ref.pos.y / 1000.0),
                Vector2D(ref.vel.x / 1000.0, ref.vel.y / 1000.0),
            )

            vel_cmd = self.robot_controller.compute_trajectory_command(
                rid, tgt_state, cur_state, dt
            )

            tgt_orientation = self.target_orientations.get(rid, cur.orientation)

            vel_ang_cmd = self.orientation_controller.compute(
                target=tgt_orientation, current=cur.orientation
            )


            if self.is_halt:
                vel_cmd = Vector2D(0, 0)
                vel_ang_cmd = 0.0

            out = RobotCommand(robot_id=rid)
            out.linear_velocity_x = float(vel_cmd.x)
            out.linear_velocity_y = float(vel_cmd.y)
            out.angular_velocity = float(vel_ang_cmd)
            out.orientation = cur.orientation
            out.kick = 0.0 if self.is_halt else float(self.kick_cache.get(rid, 0.0))

            team_cmd.robots.append(out)

        active = set(self.control_references.keys())
        self.robot_controller.cleanup_unused_robots(active)

        self.publisher.publish(team_cmd)

    def _predictive_timer_callback(self):
        if not self.control_references:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        now_sec = now.nanoseconds / 1e9
        self._drop_unseen_robots(now_sec)

        # Both sides of the comparison at the instant this command will take effect.
        t_act = now_sec + self.actuation_delay

        team_cmd = TeamCommand()
        team_cmd.is_team_color_yellow = self.is_team_color_yellow
        team_cmd.robots = []

        for rid, ref in self.control_references.items():
            if rid not in self.ally_robots:
                continue
            predictor = self.predictors.get(rid)
            if predictor is None or predictor.last_capture is None:
                continue

            cur = self.ally_robots[rid]
            x, y, vx, vy = predictor.state_at(
                min(t_act, predictor.last_capture + MAX_PREDICTION_AGE)
            )
            cur_state = MotionState(
                Vector2D(x / 1000.0, y / 1000.0), Vector2D(vx / 1000.0, vy / 1000.0)
            )
            tgt_state = self._predictive_reference(rid, ref, now_sec)

            vel_cmd = self.robot_controller.compute_trajectory_command(
                rid, tgt_state, cur_state, dt
            )

            tgt_orientation = self.target_orientations.get(rid, cur.orientation)

            vel_ang_cmd = self.orientation_controller.compute(
                target=tgt_orientation, current=cur.orientation
            )

            if self.is_halt:
                vel_cmd = Vector2D(0, 0)
                vel_ang_cmd = 0.0

            out = RobotCommand(robot_id=rid)
            out.linear_velocity_x = float(vel_cmd.x)
            out.linear_velocity_y = float(vel_cmd.y)
            out.angular_velocity = float(vel_ang_cmd)
            out.orientation = cur.orientation
            out.kick = 0.0 if self.is_halt else float(self.kick_cache.get(rid, 0.0))

            team_cmd.robots.append(out)

        active = set(self.control_references.keys())
        self.robot_controller.cleanup_unused_robots(active)

        self.publisher.publish(team_cmd)

        # Exactly what went out, HALT zeros included: the predictor's whole job is to
        # know what the robot has been told.
        for out in team_cmd.robots:
            self.predictors[out.robot_id].record_command(
                now_sec, out.linear_velocity_x * 1000.0, out.linear_velocity_y * 1000.0
            )

    def _predictive_reference(self, rid, ref, now_sec) -> MotionState:
        """
        The reference for the instant the command acts: the tracker's offset, plus how
        long the reference has been in flight, plus the actuation delay. Velocity is
        taken tau later, which inverts the robot's own velocity loop.
        """
        trajectory = self._reference_trajectory(rid, ref.trajectory)
        if trajectory is not None:
            # A tracker that does not stamp its references gives no age to add.
            age = (
                min(max(now_sec - ref.wall_stamp, 0.0), MAX_REFERENCE_AGE)
                if ref.wall_stamp > 0.0
                else 0.0
            )
            t_ref = ref.timestamp + age + self.actuation_delay
            at = reference_state(trajectory, t_ref, self.coast_max_time)
            ahead = reference_state(trajectory, t_ref + self.tau, self.coast_max_time)
            if at is not None and ahead is not None:
                return MotionState(
                    Vector2D(at.position.x / 1000.0, at.position.y / 1000.0),
                    Vector2D(ahead.velocity.x / 1000.0, ahead.velocity.y / 1000.0),
                )
        return MotionState(
            Vector2D(ref.pos.x / 1000.0, ref.pos.y / 1000.0),
            Vector2D(ref.vel.x / 1000.0, ref.vel.y / 1000.0),
        )

    def _reference_trajectory(self, rid, trajectory_msg):
        """The decoded plan behind a reference, decoded once per plan."""
        if trajectory_msg is None or not trajectory_msg.segments:
            return None
        # handoff_stamp alone can repeat: two plans may start from the same overhead
        # point. The segment starts and durations tell them apart.
        key = (
            trajectory_msg.handoff_stamp,
            tuple(
                (
                    seg.init_pos.x,
                    seg.init_pos.y,
                    seg.init_vel.x,
                    seg.init_vel.y,
                    tuple(p.duration for p in seg.motion_path.primitives),
                )
                for seg in trajectory_msg.segments
            ),
        )
        cached = self._reference_trajectories.get(rid)
        if cached is not None and cached[0] == key:
            return cached[1]
        try:
            trajectory = Trajectory.from_msg(trajectory_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode the reference plan of robot {rid}: {exc}")
            trajectory = None
        self._reference_trajectories[rid] = (key, trajectory)
        return trajectory

    def _correct_predictors(self, msg: GameState):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        t_capture = msg.vision_wall_stamp - self.vision_extra_latency
        for robot in msg.ally_robots:
            predictor = self.predictors.get(robot.id)
            if predictor is None:
                predictor = self.predictors[robot.id] = RobotStatePredictor(
                    self.actuation_delay, self.tau, self.robot_max_acceleration * 1000.0
                )
            before = predictor.last_capture
            predictor.correct(t_capture, robot.position_x, robot.position_y)
            if predictor.last_capture != before:
                self._last_capture_seen[robot.id] = now_sec
        self._drop_unseen_robots(now_sec)

    def _drop_unseen_robots(self, now_sec):
        for rid, seen in list(self._last_capture_seen.items()):
            if now_sec - seen > PREDICTOR_TIMEOUT:
                del self._last_capture_seen[rid]
                self.predictors.pop(rid, None)
                self._reference_trajectories.pop(rid, None)

    def update_parameters(self, req, resp):
        self.robot_controller.update_params(req.kp, req.ki, req.kd)
        resp.success = True
        return resp

    def set_orientation_callback(self, req, resp):
        self.target_orientations[req.robot_id] = req.orientation
        resp.success = True
        return resp

    def update_kp_angular_callback(self, req, resp):
        if req.kp >= 0:
            self.orientation_controller.kp = req.kp
            resp.success = True
        else:
            resp.success = False
        return resp

    def update_kick_callback(self, req, resp):
        robot_id = int(req.id)
        kick = float(req.kick)

        self.kick_cache[robot_id] = kick

        resp.success = True
        return resp

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        self.vision_wall_stamp = msg.vision_wall_stamp
        self.referee_command = msg.referee.command
        self.is_halt = self.referee_command in self._desired_states
        if self.control_mode == "predictive":
            self._correct_predictors(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
