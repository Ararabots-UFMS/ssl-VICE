"""What the strategy puts in the MovementCommand it hands to the planner."""

import sys
from unittest.mock import MagicMock

# The handler imports service types too; conftest only stubs the message module.
for _name in ("movement_interfaces.srv", "system_interfaces", "system_interfaces.srv"):
    sys.modules.setdefault(_name, MagicMock())

from strategy.movement_handler import MovementHandler  # noqa: E402
from strategy.skills.skills import Skill  # noqa: E402


def _command(**flags):
    skill = Skill(robot_id=3)
    skill.target_x, skill.target_y = 100.0, -200.0
    for name, value in flags.items():
        setattr(skill, name, value)
    handler = MovementHandler.__new__(MovementHandler)
    return handler._movement_command(skill)


def test_the_ball_obstacle_reaches_the_planner():
    """Positioning behind the ball relies on the planner routing around it."""
    assert _command(ball=True).planning_options.avoid_ball is True


def test_a_robot_going_for_the_ball_is_not_kept_off_it():
    assert _command(ball=False).planning_options.avoid_ball is False


def test_the_other_obstacle_flags_travel_with_it():
    options = _command(penalty_area=True, center_area=True).planning_options
    assert options.avoid_penalty_area is True
    assert options.avoid_center_area is True


def test_the_arrival_velocity_travels_with_it():
    cmd = _command(vel_x=400.0, vel_y=-300.0)
    assert (cmd.target_vel.x, cmd.target_vel.y) == (400.0, -300.0)
