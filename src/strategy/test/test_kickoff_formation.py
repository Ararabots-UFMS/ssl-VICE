"""Which kickoff robots ask the planner to keep them out of the centre circle."""

from math import hypot

import pytest

from strategy.tatics.kickoff import OurKickoff, TheirKickoff

# The planner's centre obstacle: the 500mm circle grown by a robot radius.
CENTRE_KEEP_OUT = 590.0


def _field_commands(tactic):
    return [c for c in tactic.execute() if c.robot_id != 0]


def test_our_kicker_may_stand_inside_the_circle():
    """Asked to avoid it, the planner moves the kicker's spot out and nobody kicks off."""
    kicker, *others = _field_commands(OurKickoff({0: None, 1: None, 2: None, 3: None}, False))

    assert hypot(kicker.target_x, kicker.target_y) < CENTRE_KEEP_OUT
    assert kicker.center_area is False
    assert all(c.center_area for c in others)


def test_no_robot_is_sent_to_a_spot_it_is_told_to_avoid():
    for tactic in (OurKickoff, TheirKickoff):
        for side in (True, False):
            for command in _field_commands(tactic({0: None, 1: None, 2: None, 3: None, 4: None}, side)):
                inside = hypot(command.target_x, command.target_y) < CENTRE_KEEP_OUT
                assert not (inside and command.center_area), (tactic.__name__, command.robot_id)


def test_on_their_kickoff_everyone_keeps_out():
    assert all(c.center_area for c in _field_commands(TheirKickoff({1: None, 2: None, 3: None}, True)))


def test_their_kickoff_play_builds_the_tactic_not_itself():
    """run() constructed TheirKickoffAction again, with arguments it does not take."""
    pytest.importorskip("rclpy")
    from strategy.plays.kickoff import TheirKickoffAction

    node = TheirKickoffAction.__new__(TheirKickoffAction)
    node.ally_robots = {0: None, 1: None, 2: None}
    node.on_positive_half = True

    _, commands = node.run()

    assert sorted(c.robot_id for c in commands) == [0, 1, 2]
