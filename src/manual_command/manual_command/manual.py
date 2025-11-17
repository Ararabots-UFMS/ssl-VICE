import math
import rclpy
from rclpy.node import Node
from threading import Thread
from time import time, sleep

from system_interfaces.msg import TeamCommand, RobotCommand, VisionMessage
from system_interfaces.srv import SetOrientation
from sensor_msgs.msg import Joy

import evdev
from evdev import list_devices, InputDevice, ecodes


desired_acc_x = 0.0
desired_acc_y = 0.0
desired_acc_t = 0.0

robot_theta = 0.0


def set_robot_orientation(theta: float) -> None:
    """Atualiza a orientação global do robô (em radianos).

    Chamável externamente para atualizar a orientação usada no controle field-centric.
    """
    global robot_theta
    robot_theta = float(theta)


class ManualCommand(Node):
    def __init__(self) -> None:
        super().__init__("manual_command")

        self.declare_parameter("robot_id", 1)
        self.robot_id = 1
        self.command_publisher = self.create_publisher(TeamCommand, "commandTopic", 10)
        self.create_timer(1.0 / 60.0, self.update_and_publish)

        self.create_subscription(Joy, "joy", self._joy_callback, 10)

        self.vx = 0.0
        self.vy = 0.0
        self.vt = 0.0

        self.max_velocity = 1.5
        self.max_angular_velocity = 1.5
        self.max_acceleration = 1
        self.max_angular_acceleration = 3.0 

        self._last_time = time()
        self._last_log_time = time()

        # Detect evdev event device (preferred). Fallback is /joy topic only.
        backend = None
        evdev_path = None
        if evdev is not None:
            devs = list_devices()
            for p in devs:
                d = InputDevice(p)
                caps = d.capabilities()
                if ecodes.EV_ABS in caps:
                    backend = "evdev"
                    evdev_path = p
                    break

        if backend == "evdev":
            t = Thread(target=self._evdev_thread, args=(evdev_path,), daemon=True)
            t.start()
        else:
            pass

        self.create_subscription(VisionMessage, "visionTopic", self._vision_callback, 10)

        self._got_gamepad_event = False
        self._seen_codes = set()
        self._axis_pos = {"x": 0.0, "y": 0.0, "t": 0.0, "rx": 0.0, "ry": 0.0}
        self._axis_last_time = {"x": 0.0, "y": 0.0, "t": 0.0, "rx": 0.0, "ry": 0.0}
        self._evdev_code_map = {}
        self.dead_input_timeout = 0.6
        self.dead_input_timeout_long = 5.0
        self.decel = 2.0
        self.angular_decel = 6.0
        self.declare_parameter("set_orientation_service", "set_orientation")
        svc_name = str(self.get_parameter("set_orientation_service").value)
        self._set_orientation_client = self.create_client(SetOrientation, svc_name)
        self._orientation_service_warned = False
        self._orientation_service_ready = False
        if self._set_orientation_client is not None:
            thr = Thread(target=self._orientation_service_waiter, daemon=True)
            thr.start()
        self._last_orientation_request_time = 0.0
        self._orientation_request_rate = 10.0
        self._last_requested_angle = None
        self._last_rx_log_time = 0.0
        self._orientation_target = None
        self.declare_parameter("orientation_kp", 1.0)
        self.declare_parameter("orientation_min_speed", 0.8)
        self.declare_parameter("orientation_max_accel", 20.0)
        self.declare_parameter("orientation_tolerance", 0.03)
        self._orientation_kp = float(self.get_parameter("orientation_kp").value)
        self._orientation_min_speed = float(self.get_parameter("orientation_min_speed").value)
        self._orientation_max_accel = float(self.get_parameter("orientation_max_accel").value)
        self._orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self._last_orientation_debug_time = 0.0
        self.declare_parameter("right_stick_invert_x", False)
        self.declare_parameter("right_stick_invert_y", False)
        self.declare_parameter("orientation_stick_activate", 0.25)
        self.declare_parameter("orientation_stick_deactivate", 0.10)
        self.declare_parameter("orientation_update_angle_thresh", 0.05)
        self._orientation_stick_activate = float(self.get_parameter("orientation_stick_activate").value)
        self._orientation_stick_deactivate = float(self.get_parameter("orientation_stick_deactivate").value)
        self._orientation_update_angle_thresh = float(self.get_parameter("orientation_update_angle_thresh").value)

    def update_and_publish(self) -> None:
        """Integra acelerações desejadas para atualizar velocidades e publica o comando."""
        global desired_acc_x, desired_acc_y, desired_acc_t, robot_theta

        now = time()
        dt = max(1e-6, now - self._last_time)
        self._last_time = now
        def axis_value_or_zero(axis_key):
            pos = self._axis_pos.get(axis_key, 0.0)
            last = self._axis_last_time.get(axis_key, 0.0)
            age = now - last
            if abs(pos) > 1e-4:
                if age > self.dead_input_timeout_long:
                    return 0.0
                return pos
            return 0.0

        jx = -self._axis_pos.get("ry", 0.0)
        jy = -self._axis_pos.get("rx", 0.0)
        joy_mag = math.hypot(jx, jy)
        if (now - self._last_rx_log_time) > 0.5 and (abs(jx) > 1e-3 or abs(jy) > 1e-3):
            self._last_rx_log_time = now
        if joy_mag > 1.0:
            target_angle = math.atan2(jy, jx)
            self._orientation_target = float(target_angle)
            # debug logging removed

            if self._set_orientation_client is not None:
                now_req = now
                min_dt = 1.0 / max(1.0, self._orientation_request_rate)
                angle_changed = (
                    self._last_requested_angle is None or
                    abs(((target_angle - self._last_requested_angle + math.pi) % (2 * math.pi)) - math.pi) > 0.05
                )
                if (now_req - self._last_orientation_request_time) >= min_dt and angle_changed:
                    if not self._orientation_service_ready:
                        if not self._orientation_service_warned:
                            self._orientation_service_warned = True
                        raise RuntimeError("set_orientation service missing")
                    req = SetOrientation.Request()
                    req.robot_id = int(self.robot_id)
                    req.orientation = float(target_angle)
                    self._set_orientation_client.call_async(req)
                    self._last_orientation_request_time = now_req
                    self._last_requested_angle = target_angle

        ax_pos = axis_value_or_zero("x")
        ay_pos = axis_value_or_zero("y")
        at_pos = axis_value_or_zero("t")

        if joy_mag > 0.2:
            at_pos = 0.0
            self._axis_pos["t"] = 0.0
            self._axis_last_time["t"] = now

        ax = max(-self.max_acceleration, min(self.max_acceleration, ax_pos * self.max_acceleration))
        ay = max(-self.max_acceleration, min(self.max_acceleration, ay_pos * self.max_acceleration))
        at = max(-self.max_angular_acceleration, min(self.max_angular_acceleration, at_pos * self.max_angular_acceleration))

        desired_acc_x = ax
        desired_acc_y = ay
        desired_acc_t = at

        self.vx += ax * dt
        self.vy += ay * dt
        self.vt += at * dt

        def move_towards(value, target, max_delta):
            diff = target - value
            if abs(diff) <= max_delta:
                return target
            return value + (max_delta if diff > 0 else -max_delta)

        if abs(ax_pos) < 1e-3:
            self.vx = move_towards(self.vx, 0.0, self.decel * dt)
        if abs(ay_pos) < 1e-3:
            self.vy = move_towards(self.vy, 0.0, self.decel * dt)
        if abs(at_pos) < 1e-3:
            self.vt = move_towards(self.vt, 0.0, self.angular_decel * dt)

        self.vx = max(-self.max_velocity, min(self.max_velocity, self.vx))
        self.vy = max(-self.max_velocity, min(self.max_velocity, self.vy))
        self.vt = max(-self.max_angular_velocity, min(self.max_angular_velocity, self.vt))

        def _normalize_angle(a: float) -> float:
            return ((a + math.pi) % (2 * math.pi)) - math.pi

        if self._orientation_target is not None:
            self._orientation_kp = float(self.get_parameter("orientation_kp").value)
            self._orientation_min_speed = float(self.get_parameter("orientation_min_speed").value)
            self._orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)

            err = _normalize_angle(self._orientation_target - robot_theta)
            if abs(err) <= self._orientation_tolerance:
                self.vt = 0.0
                self._orientation_target = None
            else:
                desired_vt = self._orientation_kp * err
                if abs(desired_vt) < self._orientation_min_speed:
                    desired_vt = math.copysign(self._orientation_min_speed, desired_vt)
                desired_vt = max(-self.max_angular_velocity, min(self.max_angular_velocity, desired_vt))
                self._orientation_max_accel = float(self.get_parameter("orientation_max_accel").value)
                max_delta = max(1e-6, self._orientation_max_accel * dt)
                self.vt = move_towards(self.vt, desired_vt, max_delta)
                if (now - self._last_orientation_debug_time) > 0.25:
                    self._last_orientation_debug_time = now

        theta = robot_theta
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        vx_r = cos_t * self.vx + sin_t * self.vy
        vy_r = -sin_t * self.vx + cos_t * self.vy

        robot_command = RobotCommand()
        robot_command.robot_id = 1
        robot_command.linear_velocity_x = float(vx_r)
        robot_command.linear_velocity_y = float(vy_r)
        robot_command.angular_velocity = float(self.vt)
        robot_command.kick = 0.0

        command = TeamCommand()
        command.robots.append(robot_command)

        now = time()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
        self.command_publisher.publish(command)

    def _evdev_thread(self, path: str) -> None:
        """Reader thread using python-evdev directly on the given device path."""
        global desired_acc_x, desired_acc_y, desired_acc_t
        dev = InputDevice(path)

        def norm_ev(code, value):
            ai = dev.absinfo(code)
            minv = getattr(ai, 'min', -32768)
            maxv = getattr(ai, 'max', 32767)
            if maxv == minv:
                return 0.0
            if minv < 0:
                denom = max(abs(minv), abs(maxv))
                return max(-1.0, min(1.0, float(value) / float(denom)))
            else:
                return max(-1.0, min(1.0, (2.0 * (value - minv) / (maxv - minv)) - 1.0))

        for event in dev.read_loop():
            if event.type != ecodes.EV_ABS:
                continue
            code = event.code
            val = event.value

            if not self._got_gamepad_event:
                self._got_gamepad_event = True
            if code not in self._seen_codes:
                self._seen_codes.add(code)

            dz = 0.12
            now = time()
            if code not in self._evdev_code_map:
                if code == ecodes.ABS_X:
                    self._evdev_code_map[code] = "x"
                elif code == ecodes.ABS_Y:
                    self._evdev_code_map[code] = "y"
                elif code == ecodes.ABS_RX:
                    self._evdev_code_map[code] = "rx"
                elif code == ecodes.ABS_RY:
                    self._evdev_code_map[code] = "ry"
                else:
                    mapped_vals = set(self._evdev_code_map.values())
                    if "rx" not in mapped_vals:
                        self._evdev_code_map[code] = "rx"
                    elif "ry" not in mapped_vals:
                        self._evdev_code_map[code] = "ry"
                    else:
                        self._evdev_code_map[code] = None
            axis_key = self._evdev_code_map.get(code)
            if axis_key == "x":
                v = norm_ev(code, val)
                self._axis_pos["x"] = 0.0 if abs(v) < dz else v
                self._axis_last_time["x"] = now
            elif axis_key == "y":
                v = -norm_ev(code, val)
                self._axis_pos["y"] = 0.0 if abs(v) < dz else v
                self._axis_last_time["y"] = now
            elif axis_key == "rx":
                v = norm_ev(code, val)
                self._axis_pos["rx"] = 0.0 if abs(v) < dz else v
                self._axis_last_time["rx"] = now
            elif axis_key == "ry":
                v = -norm_ev(code, val)
                self._axis_pos["ry"] = 0.0 if abs(v) < dz else v
                self._axis_last_time["ry"] = now
            else:
                if code not in self._seen_codes:
                    self._seen_codes.add(code)

    # pygame backend removed (not used in simplified configuration)

    def _joy_callback(self, msg: Joy) -> None:
        """Fallback callback to read joystick from sensor_msgs/Joy topics.

        axes layout can vary; mapping below is a reasonable default for Xbox-like controllers.
        """
        global desired_acc_x, desired_acc_y, desired_acc_t

        axes = msg.axes
        lx = axes[0] if len(axes) > 0 else 0.0
        ly = axes[1] if len(axes) > 1 else 0.0
        rx = axes[3] if len(axes) > 3 else (axes[2] if len(axes) > 2 else 0.0)
        ry = axes[4] if len(axes) > 4 else (axes[3] if len(axes) > 3 else (axes[2] if len(axes) > 2 else 0.0))

        now = time()
        self._axis_pos["x"] = float(lx)
        self._axis_last_time["x"] = now
        self._axis_pos["y"] = -float(ly)
        self._axis_last_time["y"] = now
        self._axis_pos["rx"] = float(rx)
        self._axis_last_time["rx"] = now
        self._axis_pos["ry"] = -float(ry)
        self._axis_last_time["ry"] = now

        # debug logging removed

    def _vision_callback(self, msg: VisionMessage) -> None:
        """Recebe o VisionMessage e atualiza a orientação do robô controlado (robot_id)."""
        global robot_theta
        rid = self.robot_id

        for r in msg.blue_robots:
            if int(r.id) == rid:
                robot_theta = float(r.orientation)
                return
        for r in msg.yellow_robots:
            if int(r.id) == rid:
                robot_theta = float(r.orientation)
                return

    def _orientation_service_waiter(self) -> None:
        """Background thread to wait for the set_orientation service to appear.

        This avoids blocking the main update loop and lets us know when it's safe
        to call the service.
        """
        if self._set_orientation_client is None:
            return
        while rclpy.ok() and not self._orientation_service_ready:
            ok = self._set_orientation_client.wait_for_service(timeout_sec=1.0)
            if ok:
                self._orientation_service_ready = True
                break

def main():
    rclpy.init()
    manual_command = ManualCommand()
    thread = Thread(target=rclpy.spin, args=(manual_command,), daemon=True)
    thread.start()

    while rclpy.ok():
        sleep(0.1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
