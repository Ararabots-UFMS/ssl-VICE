"""
Measure a robot's command response: actuation delay, velocity time constant, max accel.

Drive mode (needs ROS; run with launch/identify_sim.py or launch/identify_real.py,
which start nothing else that publishes on commandTopic):

    ros2 run control identify_robot --ros-args -p confirm:=true -p robot_id:=0 \
        -p axis:=x -p csv:=/root/ssl-VICE/identify_robot0.csv

For each speed: +v for `hold`, 0 for `rest`, -v for `hold`, 0 for `rest`. Every command
is logged at publish time and every new vision sample with its vision_wall_stamp.

Fit mode (numpy only):

    ros2 run control identify_robot --fit /root/ssl-VICE/identify_robot0.csv

Simulates the controller's own RobotModel over a parameter grid and keeps the set with
the lowest position RMS. The logged pose is the vision EKF's estimate, which lags while
the robot accelerates (it has no input), so each simulated trajectory goes through that
same filter before it is compared; --raw skips this. The delay is measured against
vision_wall_stamp, so whatever vision latency the stamp does not remove ends up in
actuation_delay, which is what the predictor needs.
"""
import csv
import math
import sys
from typing import List, Optional, Sequence, Tuple

import numpy as np

from control.state_predictor import RobotModel

# Robot must have been at rest this long before a pulse to anchor the fit on it.
FIT_ANCHOR_WINDOW = 0.15
COMMAND_EPSILON = 1e-6

DEFAULT_DELAYS = np.arange(0.0, 0.1501, 0.005)          # s
DEFAULT_TAUS = np.arange(0.01, 0.3001, 0.01)             # s
DEFAULT_ACCELS = np.arange(0.5, 8.001, 0.25)             # m/s²

# As vision.tracker builds its filters (ObjectTracker's friction).
VISION_FRICTION = 0.01


# --------------------------------------------------------------------------- profile
def build_profile(speeds: Sequence[float], hold: float, rest: float) -> List[Tuple[float, float]]:
    """[(signed speed m/s, duration s)]: +v hold, 0 rest, -v hold, 0 rest per speed."""
    profile = []
    for v in speeds:
        profile += [(v, hold), (0.0, rest), (-v, hold), (0.0, rest)]
    return profile


def profile_command(profile, elapsed: float) -> Optional[float]:
    """The speed in effect `elapsed` seconds into the profile, None once it is over."""
    t = 0.0
    for speed, duration in profile:
        t += duration
        if elapsed < t:
            return speed
    return None


# --------------------------------------------------------------------------- fit
def load_csv(path):
    """(commands [t, vx, vy] mm/s, samples [t, x, y] mm), both sorted by time."""
    commands, samples = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t = float(row["t"])
            if row["kind"] == "cmd":
                commands.append((t, float(row["vx"]), float(row["vy"])))
            elif row["kind"] == "vis":
                samples.append((t, float(row["x"]), float(row["y"])))
    commands = np.array(sorted(commands)).reshape(-1, 3)
    samples = np.array(sorted(samples)).reshape(-1, 3)
    return commands, samples


def _pulse_windows(commands, samples):
    """
    One window per pulse, from its first non-zero command to the next pulse. Each one
    starts from rest at the position vision saw just before, so velocity-scale errors
    (slip, wheel calibration) cannot accumulate across the whole run.
    """
    moving = np.linalg.norm(commands[:, 1:], axis=1) > COMMAND_EPSILON
    onsets = [commands[i, 0] for i in range(len(commands))
              if moving[i] and (i == 0 or not moving[i - 1])]
    windows = []
    for k, t0 in enumerate(onsets):
        t_end = onsets[k + 1] if k + 1 < len(onsets) else samples[-1, 0] + 1e-9
        before = samples[(samples[:, 0] <= t0) & (samples[:, 0] > t0 - FIT_ANCHOR_WINDOW)]
        inside = samples[(samples[:, 0] > t0) & (samples[:, 0] < t_end)]
        if len(before) == 0 or len(inside) == 0:
            continue
        windows.append((t0, before[:, 1:].mean(axis=0), inside, before[-1, 0]))
    return windows


def _vision_filter_gains(sample_times, t_last_rest):
    """
    The vision EKF's (A, K) at each sample, starting from its steady state at rest.
    With u = 0 the filter is linear and its gains do not depend on what it measures,
    so one sequence serves every parameter set in the grid.
    """
    from vision.kalman_filter import ExtendedKalmanFilterClass2D

    kf = ExtendedKalmanFilterClass2D(friction=VISION_FRICTION)
    kf.initialize(0.0, 0.0)
    zero = np.matrix([[0.0], [0.0]])
    interval = float(np.median(np.diff(sample_times))) if len(sample_times) > 1 else 1 / 60
    for _ in range(200):
        kf.predict(interval)
        kf.update(zero)
    gains, t = [], t_last_rest
    for ts in sample_times:
        dt = ts - t
        t = ts
        kf.predict(dt)
        A = np.asarray(kf._jacobian_F(kf.x, dt))
        K = np.asarray(kf.P @ kf.H.T @ np.linalg.inv(kf.H @ kf.P @ kf.H.T + kf.R))
        kf.update(zero)
        gains.append((A, K))
    return gains


def _rms_grid(commands, windows, delay, taus, accels_mm, gains=None):
    """Position RMS (mm) for every (tau, accel) pair at one delay, shape (len(taus), len(accels))."""
    tau_g, acc_g = np.meshgrid(taus, accels_mm, indexing="ij")
    model = RobotModel(delay, tau_g.reshape(-1, 1), acc_g.reshape(-1, 1), history=None)
    for t, vx, vy in commands:
        model.record_command(t, vx, vy)
    sq, count = np.zeros(tau_g.size), 0
    for w, (t0, anchor, inside, _) in enumerate(windows):
        state = np.zeros((tau_g.size, 4))
        state[:, :2] = anchor
        estimate = state.copy()
        t = t0
        for k, (ts, x, y) in enumerate(inside):
            state = model.roll(t, state, ts)
            t = ts
            if gains is None:
                seen = state[:, :2]
            else:
                A, K = gains[w][k]
                estimate = estimate @ A.T
                estimate += (state[:, :2] - estimate[:, :2]) @ K.T
                seen = estimate[:, :2]
            sq += (seen[:, 0] - x) ** 2 + (seen[:, 1] - y) ** 2
            count += 1
    return np.sqrt(sq / max(count, 1)).reshape(tau_g.shape)


def _grid_search(commands, windows, delays, taus, accels_mm, gains):
    best = (math.inf, None)
    for d in delays:
        rms = _rms_grid(commands, windows, d, taus, accels_mm, gains)
        i, j = np.unravel_index(np.argmin(rms), rms.shape)
        if rms[i, j] < best[0]:
            best = (float(rms[i, j]), (float(d), float(taus[i]), float(accels_mm[j])))
    return best


def _refined(values, centre, steps=1, factor=5):
    step = values[1] - values[0] if len(values) > 1 else 0.0
    if step == 0.0:
        return np.array([centre])
    fine = np.arange(centre - steps * step, centre + steps * step + 1e-12, step / factor)
    return fine[(fine >= values[0] - 1e-12) & (fine <= values[-1] + 1e-12)]


def fit(commands, samples, delays=DEFAULT_DELAYS, taus=DEFAULT_TAUS, accels=DEFAULT_ACCELS,
        vision_filter=True):
    """
    Returns dict(actuation_delay s, tau s, max_accel m/s², rms mm, at_boundary [names]).
    A coarse grid, then a finer one around its best point. vision_filter: the samples
    are the vision EKF's estimates (what identify_robot logs), not raw detections.
    """
    windows = _pulse_windows(commands, samples)
    if not windows:
        raise ValueError("no pulse with vision samples before and after its onset")
    gains = None
    if vision_filter:
        gains = [_vision_filter_gains(inside[:, 0], t_rest) for _, _, inside, t_rest in windows]
    accels_mm = np.asarray(accels, dtype=float) * 1000.0
    delays, taus = np.asarray(delays, dtype=float), np.asarray(taus, dtype=float)

    _, (d, tau, a) = _grid_search(commands, windows, delays, taus, accels_mm, gains)
    rms, (d, tau, a) = _grid_search(commands, windows, _refined(delays, d),
                                    _refined(taus, tau), _refined(accels_mm, a), gains)
    # The refined grid is clipped to the coarse range, so an edge is the coarse edge.
    at_boundary = [name for name, value, grid in (("actuation_delay", d, delays),
                                                   ("tau", tau, taus),
                                                   ("max_accel", a, accels_mm))
                   if len(grid) > 1 and (np.isclose(value, grid[0]) or np.isclose(value, grid[-1]))]
    return dict(actuation_delay=d, tau=tau, max_accel=a / 1000.0, rms=rms,
                at_boundary=at_boundary, pulses=len(windows))


def run_fit(path, vision_filter=True):
    commands, samples = load_csv(path)
    result = fit(commands, samples, vision_filter=vision_filter)
    print(f"{result['pulses']} pulses, position RMS {result['rms']:.1f} mm")
    print(f"actuation_delay = {result['actuation_delay']:.3f} s")
    print(f"tau             = {result['tau']:.3f} s")
    print(f"max_accel       = {result['max_accel']:.2f} m/s²")
    for name in result["at_boundary"]:
        print(f"WARNING: {name} landed on the edge of the search grid; the true value may "
              f"lie outside it. Check the run, or widen the grid.", file=sys.stderr)
    print("\nFor the launch file:")
    print(f"ROBOT = {{\"actuation_delay\": {result['actuation_delay']:.3f}, "
          f"\"tau\": {result['tau']:.3f}, \"max_accel\": {result['max_accel']:.2f}}}")
    return result


# --------------------------------------------------------------------------- node
def _make_node_class():
    from rclpy.node import Node

    from system_interfaces.msg import GameState, RobotCommand, TeamCommand
    from system_interfaces.srv import GetGameConfig

    class IdentifyRobot(Node):
        RATE = 50.0
        UNSEEN_TIMEOUT = 0.3
        STOP_DURATION = 0.5
        SETTLE = 0.5  # zeros before the first pulse, so the fit's anchor is at rest

        def __init__(self):
            super().__init__("identify_robot")
            params = {
                "confirm": False,
                "robot_id": 0,
                "axis": "x",
                "speeds": [0.5, 1.0, 1.5],
                "hold": 0.8,
                "rest": 1.0,
                "max_speed": 2.0,
                # Field frame, mm. The run aborts as soon as the robot leaves it.
                "box_min_x": -4000.0,
                "box_max_x": 4000.0,
                "box_min_y": -2500.0,
                "box_max_y": 2500.0,
                "csv": "identify_robot.csv",
            }
            for name, default in params.items():
                self.declare_parameter(name, default)
            p = {name: self.get_parameter(name).value for name in params}

            self.done = False
            self.phase = "wait"
            if not p["confirm"]:
                self.get_logger().error(
                    "identify_robot drives a robot open-loop. Make sure nothing else "
                    "publishes on commandTopic and the path is clear, then rerun with "
                    "-p confirm:=true")
                self.done = True
                return
            if p["axis"] not in ("x", "y"):
                raise ValueError(f"axis must be x or y, got {p['axis']!r}")
            speeds = [float(s) for s in p["speeds"]]
            if any(s <= 0 for s in speeds):
                raise ValueError(f"speeds must be positive, got {speeds}")
            if max(speeds) > p["max_speed"]:
                raise ValueError(f"speeds {speeds} exceed max_speed {p['max_speed']}")

            self.robot_id = int(p["robot_id"])
            self.axis = p["axis"]
            self.box = (p["box_min_x"], p["box_max_x"], p["box_min_y"], p["box_max_y"])
            self.profile = build_profile(speeds, float(p["hold"]), float(p["rest"]))
            self.stroke = 1000.0 * max(speeds) * float(p["hold"])  # mm, one way

            self.is_team_color_yellow = None
            self.robot = None
            self.last_seen = None
            self.last_stamp = None
            self.phase_start = None

            self._csv_file = open(p["csv"], "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(["kind", "t", "vx", "vy", "x", "y", "theta"])

            self.publisher = self.create_publisher(TeamCommand, "commandTopic", 10)
            self.create_subscription(GameState, "game_state", self._on_game_state, 10)
            self._config_client = self.create_client(GetGameConfig, "get_game_config")
            self._config_future = None
            self.create_timer(1.0 / self.RATE, self._tick)
            self._msgs = (RobotCommand, TeamCommand, GetGameConfig)
            self.get_logger().info(f"identify_robot: robot {self.robot_id}, axis {self.axis}, "
                                   f"speeds {speeds} m/s, logging to {p['csv']}")

        def _now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def _on_game_state(self, msg):
            if self.done:
                return
            robot = next((r for r in msg.ally_robots if r.id == self.robot_id), None)
            if robot is None:
                return
            self.robot = robot
            self.last_seen = self._now()
            if self.last_stamp is None or msg.vision_wall_stamp > self.last_stamp:
                self.last_stamp = msg.vision_wall_stamp
                self._csv.writerow(["vis", f"{msg.vision_wall_stamp:.6f}", "", "",
                                    f"{robot.position_x:.2f}", f"{robot.position_y:.2f}",
                                    f"{robot.orientation:.5f}"])

        def _poll_config(self):
            if self._config_future is None:
                if self._config_client.service_is_ready():
                    self._config_future = self._config_client.call_async(self._msgs[2].Request())
            elif self._config_future.done():
                try:
                    self.is_team_color_yellow = bool(self._config_future.result().is_team_color_yellow)
                except Exception as exc:  # retry
                    self.get_logger().warn(f"get_game_config failed: {exc}")
                    self._config_future = None

        def _publish(self, speed):
            RobotCommand, TeamCommand, _ = self._msgs
            now = self._now()
            vx = speed if self.axis == "x" else 0.0
            vy = speed if self.axis == "y" else 0.0
            out = RobotCommand(robot_id=self.robot_id)
            out.linear_velocity_x = float(vx)
            out.linear_velocity_y = float(vy)
            out.angular_velocity = 0.0
            out.orientation = float(self.robot.orientation) if self.robot is not None else 0.0
            out.kick = 0.0
            team = TeamCommand()
            team.is_team_color_yellow = bool(self.is_team_color_yellow)
            team.robots = [out]
            self.publisher.publish(team)
            self._csv.writerow(["cmd", f"{now:.6f}", f"{vx * 1000.0:.3f}", f"{vy * 1000.0:.3f}",
                                "", "", ""])

        def _inside_box(self):
            x, y = self.robot.position_x, self.robot.position_y
            return self.box[0] <= x <= self.box[1] and self.box[2] <= y <= self.box[3]

        def _start_fits(self):
            x, y = self.robot.position_x, self.robot.position_y
            lo, hi = (self.box[0], self.box[1]) if self.axis == "x" else (self.box[2], self.box[3])
            pos = x if self.axis == "x" else y
            # +v first, so the forward stroke plus braking room must fit.
            return pos + self.stroke + 300.0 <= hi and pos - 300.0 >= lo

        def _stop(self, reason):
            if self.phase != "stop":
                self.get_logger().warn(f"identify_robot stopping: {reason}")
                self.phase, self.phase_start = "stop", self._now()

        def _tick(self):
            if self.done:
                return
            now = self._now()
            if self.phase == "stop":
                self._publish(0.0)
                if now - self.phase_start >= self.STOP_DURATION:
                    self.close()
                return

            if self.is_team_color_yellow is None:
                self._poll_config()
                return
            if self.robot is None:
                return

            if self.last_seen is None or now - self.last_seen > self.UNSEEN_TIMEOUT:
                if self.phase != "wait":
                    self._stop(f"robot unseen for more than {self.UNSEEN_TIMEOUT} s")
                    self._publish(0.0)
                return
            if not self._inside_box():
                self._stop(f"robot left the safety box at ({self.robot.position_x:.0f}, "
                           f"{self.robot.position_y:.0f})")
                self._publish(0.0)
                return

            if self.phase == "wait":
                if not self._start_fits():
                    self.get_logger().error(
                        f"Not enough room: the robot needs {self.stroke + 300.0:.0f} mm "
                        f"ahead along +{self.axis} inside the safety box.")
                    self.close()
                    return
                self.phase, self.phase_start = "settle", now
            if self.phase == "settle":
                self._publish(0.0)
                if now - self.phase_start >= self.SETTLE:
                    self.phase, self.phase_start = "run", now
                return

            speed = profile_command(self.profile, now - self.phase_start)
            if speed is None:
                self._stop("profile complete")
                self._publish(0.0)
                return
            self._publish(speed)

        def close(self):
            if not self.done:
                self.done = True
                if getattr(self, "_csv_file", None):
                    self._csv_file.close()
                    self.get_logger().info("identify_robot: log written; fit it with "
                                           "`ros2 run control identify_robot --fit <csv>`")

    return IdentifyRobot


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    if argv and argv[0] == "--fit":
        paths = [a for a in argv[1:] if a != "--raw"]
        if len(paths) != 1:
            print("usage: identify_robot --fit <csv> [--raw]", file=sys.stderr)
            return 2
        run_fit(paths[0], vision_filter="--raw" not in argv)
        return 0

    import rclpy

    try:
        from rclpy.signals import SignalHandlerOptions

        # rclpy's own SIGINT handler shuts the context down before the zeros below can
        # go out, and hardware_messenger keeps resending the last command at 120 Hz.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init(args=args)
    node = _make_node_class()()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C or a crash mid-pulse: still leave the robot stopped.
        if not node.done and node.phase != "wait":
            node._stop("shutdown")
            end = node._now() + node.STOP_DURATION
            while rclpy.ok() and node._now() < end:
                node._publish(0.0)
                rclpy.spin_once(node, timeout_sec=1.0 / node.RATE)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
