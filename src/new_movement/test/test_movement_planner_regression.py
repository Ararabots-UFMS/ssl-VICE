"""
Node-level regression tests for MovementPlanner.

Kept out of test/local_planner/ because these exercise the ROS node rather than the
planning library, and so need the real message stubs rather than that package's conftest.
"""

import sys
from unittest.mock import MagicMock

import pytest

for _name in ("movement_interfaces", "system_interfaces"):
    if _name not in sys.modules:
        _mock = MagicMock(name=_name)
        sys.modules[_name] = _mock
        sys.modules[_name + ".msg"] = _mock.msg


class TestPlanningFromVision:
    """
    A plan left on the raw vision stamp reaches the tracker 150-280ms after the instant
    it describes. For a braking plan that means naming a stop point the robot can no
    longer reach, which it runs past and is then dragged back to.
    """

    def _node(self, now_sec):
        from unittest.mock import MagicMock
        from new_movement.movement_planner import MovementPlanner

        node = MovementPlanner.__new__(MovementPlanner)
        node.get_logger = MagicMock()
        node.get_clock = MagicMock()
        node.get_clock().now().nanoseconds = int(now_sec * 1e9)
        return node

    def _target(self, position, velocity, stamp):
        from unittest.mock import MagicMock

        target = MagicMock()
        target.initial_pos.x, target.initial_pos.y = position
        target.initial_vel.x, target.initial_vel.y = velocity
        target.vision_stamp = stamp
        return target

    def test_the_pose_is_carried_forward_to_now(self):
        node = self._node(100.08)
        target = self._target((0.0, 0.0), (2000.0, 0.0), stamp=100.0)

        state, stamp = node._state_from_vision(target)

        assert state.position.x == pytest.approx(160.0)
        assert state.velocity.x == pytest.approx(2000.0)
        assert stamp == pytest.approx(100.08)

    def test_the_stamp_matches_the_state_it_describes(self):
        """Otherwise the tracker starts the plan at an offset the robot never drove."""
        node = self._node(100.08)
        target = self._target((0.0, 0.0), (2000.0, 0.0), stamp=100.0)

        _, stamp = node._state_from_vision(target)

        assert stamp == pytest.approx(node.get_clock().now().nanoseconds / 1e9)

    def test_a_stale_measurement_is_not_extrapolated(self):
        node = self._node(101.0)
        target = self._target((0.0, 0.0), (2000.0, 0.0), stamp=100.0)

        state, stamp = node._state_from_vision(target)

        assert state.position.x == pytest.approx(0.0)
        assert stamp == pytest.approx(100.0)

    def test_an_unstamped_target_is_left_alone(self):
        node = self._node(100.08)
        target = self._target((500.0, 0.0), (0.0, 0.0), stamp=0.0)

        state, stamp = node._state_from_vision(target)

        assert state.position.x == pytest.approx(500.0)
        assert stamp == 0.0

    def test_a_stationary_robot_is_unchanged(self):
        node = self._node(100.08)
        target = self._target((500.0, -200.0), (0.0, 0.0), stamp=100.0)

        state, _ = node._state_from_vision(target)

        assert state.position.x == pytest.approx(500.0)
        assert state.position.y == pytest.approx(-200.0)
