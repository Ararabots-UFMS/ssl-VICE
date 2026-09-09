"""Mocks for the strategy tests.

The behaviour tree itself needs nothing mocked: after the context refactor it
imports no rclpy and no ROS message types. The one stub here is for
movement_interfaces, which utils.math_util imports and which the tactics reach
through Vector2D.
"""

import sys
from unittest.mock import MagicMock

mock_movement_interfaces = MagicMock()
sys.modules.setdefault("movement_interfaces", mock_movement_interfaces)
sys.modules.setdefault("movement_interfaces.msg", mock_movement_interfaces.msg)
