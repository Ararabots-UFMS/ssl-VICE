from unittest.mock import MagicMock

import pytest

from referee.referee.referee_message_wrapper import RefereeMessageWrapper


@pytest.fixture
def mock_referee():
    mock = MagicMock()
    mock.stage = 1
    mock.stage_time_left = 120
    mock.command = 2
    mock.command_counter = 5
    mock.current_action_time_remaining = 30
    mock.blue_team_on_positive_half = True

    mock.blue.name = "Team Blue"
    mock.blue.score = 2
    mock.blue.goalkeeper = 1
    mock.blue.bot_substitution_allowed = True
    mock.blue.can_place_ball = True
    mock.blue.max_allowed_bots = 11
    mock.blue.bot_substitution_intent = False
    mock.blue.bot_substitutions_left = 3
    mock.blue.bot_substitution_time_left = 0
    mock.blue.red_cards = 0
    mock.blue.yellow_cards = 1
    mock.blue.timeouts = 2
    mock.blue.foul_counter = 1
    mock.blue.ball_placement_failures = 0

    mock.yellow.name = "Team Yellow"
    mock.yellow.score = 3
    mock.yellow.goalkeeper = 2
    mock.yellow.bot_substitution_allowed = False
    mock.yellow.can_place_ball = True
    mock.yellow.max_allowed_bots = 10
    mock.yellow.bot_substitution_intent = False
    mock.yellow.bot_substitutions_left = 5
    mock.yellow.bot_substitution_time_left = 0
    mock.yellow.red_cards = 1
    mock.yellow.yellow_cards = 2
    mock.yellow.timeouts = 1
    mock.yellow.foul_counter = 2
    mock.yellow.ball_placement_failures = 1

    return mock


def test_wrap_basic_fields(mock_referee):
    wrapper = RefereeMessageWrapper(mock_referee)
    msg = wrapper.wrap()

    assert msg.stage == "NORMAL_FIRST_HALF"
    assert msg.command == "HALT"
    assert msg.stage_time_left == 120
    assert msg.command_counter == 5
    assert msg.current_action_time_remaining == 30
    assert len(msg.teams) == 2

    teams_by_color = {team.color: team for team in msg.teams}
    team_blue = teams_by_color["blue"]
    team_yellow = teams_by_color["yellow"]

    assert team_blue.name == "Team Blue"
    assert team_blue.score == 2
    assert team_blue.goalkeeper == 1
    assert team_blue.bot_substitution_allowed is True
    assert team_blue.can_place_ball is True
    assert team_blue.max_allowed_bots == 11
    assert team_blue.bot_substitution_intent is False
    assert team_blue.bot_substitutions_left == 3
    assert team_blue.red_cards == 0
    assert team_blue.yellow_cards == 1
    assert team_blue.timeouts == 2
    assert team_blue.foul_counter == 1
    assert team_blue.ball_placement_failures == 0

    assert team_yellow.name == "Team Yellow"
    assert team_yellow.score == 3
    assert team_yellow.goalkeeper == 2
    assert team_yellow.bot_substitution_allowed is False
    assert team_yellow.can_place_ball is True
    assert team_yellow.max_allowed_bots == 10
    assert team_yellow.bot_substitution_intent is False
    assert team_yellow.bot_substitutions_left == 5
    assert team_yellow.red_cards == 1
    assert team_yellow.yellow_cards == 2
    assert team_yellow.timeouts == 1
    assert team_yellow.foul_counter == 2
    assert team_yellow.ball_placement_failures == 1


def test_wrap_sets_on_positive_half_based_on_blue_side(mock_referee):
    wrapper = RefereeMessageWrapper(mock_referee)
    msg = wrapper.wrap()

    teams_by_color = {team.color: team for team in msg.teams}
    assert teams_by_color["blue"].on_positive_half is True
    assert teams_by_color["yellow"].on_positive_half is False


def test_wrap_defaults_to_false_when_blue_side_is_missing(mock_referee):
    del mock_referee.blue_team_on_positive_half

    wrapper = RefereeMessageWrapper(mock_referee)
    msg = wrapper.wrap()

    teams_by_color = {team.color: team for team in msg.teams}
    assert teams_by_color["blue"].on_positive_half is False
    assert teams_by_color["yellow"].on_positive_half is True
