"""
Lightweight stand-in for the ROS-generated `movement_interfaces.msg` package.

`utils.math_util.Vector2D` imports `movement_interfaces.msg.Vector2D` purely
for `to_msg`/`from_msg` (de)serialization helpers. Building the real
rosidl-generated message package requires a full ROS2 environment (rclpy,
rosidl_parser, etc.) which isn't available for a plain `pytest` run. This
fake behaves like a simple attribute container so `utils.math_util.Vector2D`
can be imported and used without any ROS dependency. Mirrors the same
pattern already used in `test/entities/conftest.py`.
"""

import sys
import types


class Vector2D:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y


_msg_module = types.ModuleType("movement_interfaces.msg")
_msg_module.Vector2D = Vector2D

_movement_interfaces_module = types.ModuleType("movement_interfaces")
_movement_interfaces_module.msg = _msg_module

sys.modules.setdefault("movement_interfaces", _movement_interfaces_module)
sys.modules.setdefault("movement_interfaces.msg", _msg_module)
