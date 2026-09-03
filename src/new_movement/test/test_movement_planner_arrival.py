"""
The planner has to stop planning once the robot has arrived.

Nothing used to stop it. Every cycle produced another plan a few centimetres long, and
a plan that short is older than its own duration by the time it reaches the tracker, so
it was activated at its end, snapped the reference, and pushed the robot off the point
it had already reached. On the field the robot reached its goal and then oscillated
across it without settling.
"""

import sys
from unittest.mock import MagicMock

import pytest

def _ensure_msg_module(module_name, *attributes):
    """
    Make sure `module_name` exists and carries `attributes`.

    test/entities/conftest.py installs a hand-written stub for these packages that only
    holds the entity message types, and it wins whenever the suite runs together — so
    the planner's own imports (TargetArray and friends) are missing from it. Filling in
    the gaps beats replacing a stub the other tests already rely on.
    """
    module = sys.modules.get(module_name)
    if module is None:
        root = module_name.split(".")[0]
        parent = sys.modules.get(root) or MagicMock(name=root)
        sys.modules[root] = parent
        module = parent.msg
        sys.modules[module_name] = module
    for attribute in attributes:
        if not hasattr(module, attribute):
            setattr(module, attribute, MagicMock(name=attribute))
    return module


_ensure_msg_module(
    "movement_interfaces.msg",
    "TargetArray",
    "Trajectory",
    "TrajectoryPoint",
    "Vector2D",
)
_ensure_msg_module("system_interfaces.msg", "GameState")

# Stubbed rather than required: the arrival check is plain geometry over the node's own
# state, and the tests build the node with __new__, so none of rclpy's machinery is
# exercised. Without this the file could only run inside a ROS environment.
if "rclpy" not in sys.modules:
    _rclpy = MagicMock(name="rclpy")

    class _Node:
        def __init__(self, *args, **kwargs):
            pass

    _rclpy.node.Node = _Node
    sys.modules["rclpy"] = _rclpy
    sys.modules["rclpy.node"] = _rclpy.node
    sys.modules["rclpy.callback_groups"] = _rclpy.callback_groups
    sys.modules["rclpy.executors"] = _rclpy.executors

from new_movement.movement_planner import (  # noqa: E402
    GOAL_MOVED_EPSILON,
    PARK_RELEASE_FACTOR,
    MovementPlanner,
)

from utils.math_util import Vector2D  # noqa: E402

ACCEPT_RADIUS = 50.0


@pytest.fixture
def planner():
    node = MovementPlanner.__new__(MovementPlanner)
    node._parked = {}
    node.get_parameter = lambda name: MagicMock(value=ACCEPT_RADIUS)
    return node


class TestArrivalLatch:
    GOAL = Vector2D(0.0, 0.0)

    def test_a_robot_far_away_is_not_parked(self, planner):
        assert not planner._is_parked(1, self.GOAL, Vector2D(2000.0, 0.0))

    def test_a_robot_inside_the_radius_is_parked(self, planner):
        assert planner._is_parked(1, self.GOAL, Vector2D(ACCEPT_RADIUS - 1, 0.0))

    def test_arrival_survives_noise_just_outside_the_radius(self, planner):
        """
        The reason for the latch: accept_radius is close to the vision noise, so a bare
        threshold would flip back to planning on the next frame and start the cycle up.
        """
        assert planner._is_parked(1, self.GOAL, Vector2D(ACCEPT_RADIUS - 1, 0.0))

        jitter = ACCEPT_RADIUS * 1.5
        assert planner._is_parked(1, self.GOAL, Vector2D(jitter, 0.0))

    def test_drifting_well_clear_resumes_planning(self, planner):
        planner._is_parked(1, self.GOAL, Vector2D(ACCEPT_RADIUS - 1, 0.0))

        drifted = ACCEPT_RADIUS * PARK_RELEASE_FACTOR + 10.0
        assert not planner._is_parked(1, self.GOAL, Vector2D(drifted, 0.0))

    def test_planning_resumes_from_scratch_after_release(self, planner):
        planner._is_parked(1, self.GOAL, Vector2D(ACCEPT_RADIUS - 1, 0.0))
        planner._is_parked(1, self.GOAL, Vector2D(ACCEPT_RADIUS * PARK_RELEASE_FACTOR + 10.0, 0.0))

        # Back inside the tight radius, it parks again.
        assert planner._is_parked(1, self.GOAL, Vector2D(1.0, 0.0))

    def test_a_new_goal_always_plans(self, planner):
        """Parking at one point must not swallow the command to go somewhere else."""
        assert planner._is_parked(1, self.GOAL, Vector2D(0.0, 0.0))

        moved = Vector2D(3000.0, 0.0)
        assert not planner._is_parked(1, moved, Vector2D(0.0, 0.0))

    def test_a_goal_that_barely_moved_is_the_same_goal(self, planner):
        assert planner._is_parked(1, self.GOAL, Vector2D(0.0, 0.0))

        nudged = Vector2D(GOAL_MOVED_EPSILON / 2.0, 0.0)
        assert planner._is_parked(1, nudged, Vector2D(0.0, 0.0))

    def test_robots_park_independently(self, planner):
        assert planner._is_parked(1, self.GOAL, Vector2D(0.0, 0.0))
        assert not planner._is_parked(2, self.GOAL, Vector2D(2000.0, 0.0))
        assert planner._is_parked(1, self.GOAL, Vector2D(0.0, 0.0))
