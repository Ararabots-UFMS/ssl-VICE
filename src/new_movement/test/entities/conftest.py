"""
Lightweight stand-ins for the ROS-generated `movement_interfaces.msg` package.

The entities under `new_movement.entities` are plain dataclasses/objects that
only touch `movement_interfaces.msg` for their `to_msg`/`from_msg` helpers.
Building the real rosidl-generated message package requires a full ROS2
environment (rclpy, rosidl_parser, etc.) which isn't available for a plain
`pytest` run. These fakes behave like simple attribute containers so the
entity code can be exercised (including round-tripping through to_msg/
from_msg) without any ROS dependency.
"""

import sys
import types


class Vector2D:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y


class MotionPrimitive:
    def __init__(self, acceleration=None, duration: float = 0.0):
        self.acceleration = acceleration
        self.duration = duration


class MotionPath:
    def __init__(self):
        self.primitives = []


class TrajectorySegment:
    def __init__(self):
        self.init_pos = None
        self.init_vel = None
        self.motion_path = None


class Trajectory:
    def __init__(self):
        self.robot_id = 0
        self.segments = []


_msg_module = types.ModuleType("movement_interfaces.msg")
_msg_module.Vector2D = Vector2D
_msg_module.MotionPrimitive = MotionPrimitive
_msg_module.MotionPath = MotionPath
_msg_module.TrajectorySegment = TrajectorySegment
_msg_module.Trajectory = Trajectory

_movement_interfaces_module = types.ModuleType("movement_interfaces")
_movement_interfaces_module.msg = _msg_module

sys.modules.setdefault("movement_interfaces", _movement_interfaces_module)
sys.modules.setdefault("movement_interfaces.msg", _msg_module)
