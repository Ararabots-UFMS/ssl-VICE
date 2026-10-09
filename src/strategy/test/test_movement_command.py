"""What the strategy node puts in the MovementCommand it hands to the planner."""

from types import SimpleNamespace

import pytest

# The node module needs ROS to import; the tactics tests next to this one do not.
pytest.importorskip("rclpy")

from strategy.skills.skills import Skill  # noqa: E402
from strategy.strategy import Strategy  # noqa: E402


def _fake_command():
    return SimpleNamespace(
        robot_id=0,
        target_pos=SimpleNamespace(x=0.0, y=0.0),
        target_vel=SimpleNamespace(x=0.0, y=0.0),
        planning_options=SimpleNamespace(
            avoid_penalty_area=False, avoid_center_area=False, avoid_ball=False
        ),
    )


def _node():
    node = Strategy.__new__(Strategy)
    node.movimento_novo = True
    node._configurou_manager = True
    node._MovementCommand = _fake_command
    node._lote_mov = []
    return node


def _skill(**flags):
    skill = Skill(robot_id=3)
    skill.target_x, skill.target_y = 100.0, -200.0
    for name, value in flags.items():
        setattr(skill, name, value)
    return skill


def test_the_ball_obstacle_reaches_the_planner():
    """Positioning behind the ball relies on the planner routing around it."""
    node = _node()

    node._send_move(_skill(ball=True))

    assert node._lote_mov[0].planning_options.avoid_ball is True


def test_a_robot_going_for_the_ball_is_not_kept_off_it():
    node = _node()

    node._send_move(_skill(ball=False))

    assert node._lote_mov[0].planning_options.avoid_ball is False


def test_the_other_obstacle_flags_travel_with_it():
    node = _node()

    node._send_move(_skill(penalty_area=True, center_area=True))

    options = node._lote_mov[0].planning_options
    assert options.avoid_penalty_area is True
    assert options.avoid_center_area is True
