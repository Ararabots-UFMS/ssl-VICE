"""The behaviour tree, ticked without a ROS graph.

Every test here builds a TickContext by hand and calls run() on it. That is the
whole point of the context refactor: no rclpy.init, no executor, no topics.
"""

import sys
from dataclasses import dataclass, field
from typing import List

import pytest

from strategy.behaviour import LeafNode, Selector, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import GameConfig, TickContext, TreeDeps
from strategy.root import RootTree


@dataclass
class FakeRobot:
    id: int = 0
    position_x: float = 0.0
    position_y: float = 0.0
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    orientation: float = 0.0
    velocity_orientation: float = 0.0


@dataclass
class FakeBall:
    id: int = 0
    position_x: float = 0.0
    position_y: float = 0.0
    velocity_x: float = 0.0
    velocity_y: float = 0.0


@dataclass
class FakeReferee:
    command: str = "HALT"


@dataclass
class FakeGameState:
    """Shaped like system_interfaces.msg.GameState, with none of the ROS weight."""

    ally_robots: List[FakeRobot] = field(default_factory=list)
    enemy_robots: List[FakeRobot] = field(default_factory=list)
    balls: List[FakeBall] = field(default_factory=list)
    referee: FakeReferee = field(default_factory=FakeReferee)
    vision_capture_stamp: float = 0.0


@pytest.fixture
def deps():
    return TreeDeps.null()


@pytest.fixture
def root(deps):
    return RootTree("RootStrategy", deps)


def make_context(command, *, allies=4, ball=True, config=True, ally_x=-1500.0):
    """A world with the allies parked well clear of the ball at the origin."""
    robots = {
        i: FakeRobot(id=i, position_x=ally_x - 400.0 * i, position_y=300.0 * i)
        for i in range(allies)
    }
    enemies = {
        i: FakeRobot(id=i, position_x=1500.0 + 400.0 * i, position_y=-300.0 * i)
        for i in range(allies)
    }
    return TickContext(
        ally_robots=robots,
        enemy_robots=enemies,
        balls=(FakeBall(),) if ball else (),
        referee_command=command,
        config=GameConfig(on_positive_half=True, robot_count=allies) if config else None,
    )


# --- the tree is plain Python ------------------------------------------------


def test_tree_builds_without_rclpy(root):
    """Constructing the whole tree must not pull in ROS."""
    assert "rclpy" not in sys.modules
    assert len(root.children) == 5


# --- the snapshot ------------------------------------------------------------


def test_snapshot_projects_the_message():
    msg = FakeGameState(
        ally_robots=[FakeRobot(id=0), FakeRobot(id=3)],
        enemy_robots=[FakeRobot(id=1)],
        balls=[FakeBall(position_x=42.0)],
        referee=FakeReferee(command="NORMAL_START"),
        vision_capture_stamp=1.5,
    )

    context = TickContext.from_game_state(msg, GameConfig(on_positive_half=True))

    assert sorted(context.ally_robots) == [0, 3]
    assert sorted(context.enemy_robots) == [1]
    assert context.ball.position_x == 42.0
    assert context.referee_command == "NORMAL_START"
    assert context.on_positive_half is True
    assert context.vision_capture_stamp == 1.5


def test_snapshot_is_unaffected_by_later_writes_to_the_message():
    """This is the tearing fix.

    A vision callback arriving mid-tick replaces the node's message reference; it
    cannot reach into a snapshot a tick is already holding.
    """
    msg = FakeGameState(ally_robots=[FakeRobot(id=0)], balls=[FakeBall(position_x=1.0)])

    context = TickContext.from_game_state(msg)

    msg.ally_robots.append(FakeRobot(id=7))
    msg.balls.clear()

    assert sorted(context.ally_robots) == [0]
    assert context.ball is not None
    assert context.ball.position_x == 1.0


def test_context_is_frozen():
    context = make_context("HALT")

    with pytest.raises(Exception):
        context.referee_command = "STOP"

    with pytest.raises(TypeError):
        TickContext.from_game_state(FakeGameState()).ally_robots[1] = FakeRobot()


def test_ball_is_none_when_vision_reports_no_ball():
    assert TickContext().ball is None
    assert TickContext().has_robots_and_ball() is False


def test_config_fields_are_none_until_the_service_answers():
    context = TickContext(config=None)

    assert context.on_positive_half is None
    assert context.is_team_color_yellow is None


# --- composites --------------------------------------------------------------


class _Fixed(LeafNode):
    def __init__(self, name, deps, status, action=None):
        super().__init__(name, deps)
        self.status = status
        self.action = action

    def run(self, context):
        return self.status, self.action


def test_empty_sequence_succeeds_without_raising(deps):
    """The previous implementation raised on an unbound name here."""
    assert Sequence("empty", deps, []).run(TickContext()) == (TaskStatus.SUCCESS, None)


def test_sequence_stops_at_the_first_non_success(deps):
    seq = Sequence(
        "seq",
        deps,
        [
            _Fixed("ok", deps, TaskStatus.SUCCESS, "first"),
            _Fixed("no", deps, TaskStatus.FAILURE, "second"),
            _Fixed("never", deps, TaskStatus.SUCCESS, "third"),
        ],
    )

    assert seq.run(TickContext()) == (TaskStatus.FAILURE, "second")


def test_selector_stops_at_the_first_non_failure(deps):
    sel = Selector(
        "sel",
        deps,
        [
            _Fixed("no", deps, TaskStatus.FAILURE, "first"),
            _Fixed("ok", deps, TaskStatus.SUCCESS, "second"),
            _Fixed("never", deps, TaskStatus.SUCCESS, "third"),
        ],
    )

    assert sel.run(TickContext()) == (TaskStatus.SUCCESS, "second")


def test_check_state_matches_the_referee_command(deps):
    node = CheckState("check", deps, ["STOP", "HALT"])

    assert node.run(make_context("HALT"))[0] == TaskStatus.SUCCESS
    assert node.run(make_context("NORMAL_START"))[0] == TaskStatus.FAILURE


# --- the whole tree ----------------------------------------------------------


@pytest.mark.parametrize(
    "command",
    [
        "HALT",
        "PREPARE_KICKOFF_BLUE",
        "PREPARE_KICKOFF_YELLOW",
        "DIRECT_FREE_BLUE",
        "DIRECT_FREE_YELLOW",
        "NORMAL_START",
        "FORCE_START",
    ],
)
def test_each_referee_command_produces_commands(root, command):
    status, action = root.run(make_context(command))

    assert status == TaskStatus.SUCCESS
    assert action, f"{command} produced no skills"
    assert all(hasattr(skill, "robot_id") for skill in action)


def test_unknown_referee_command_falls_through(root):
    assert root.run(make_context("SOMETHING_ELSE"))[0] == TaskStatus.FAILURE


def test_tree_waits_for_vision(root):
    status, action = root.run(make_context("NORMAL_START", allies=0, ball=False))

    assert status == TaskStatus.RUNNING
    assert action is None


def test_tree_waits_for_the_game_config(root):
    """Without the config the tree cannot tell which half it defends."""
    status, action = root.run(make_context("NORMAL_START", config=False))

    assert status == TaskStatus.RUNNING
    assert action is None


# --- the STOP play, which is the one with real geometry ----------------------


def test_stop_keeps_position_when_every_ally_is_clear_of_the_ball(root):
    status, action = root.run(make_context("STOP"))

    assert status == TaskStatus.SUCCESS
    assert action is None


def test_stop_moves_robots_off_a_ball_they_are_crowding(root):
    crowding = TickContext(
        ally_robots={i: FakeRobot(id=i, position_x=100.0 * i) for i in range(4)},
        balls=(FakeBall(),),
        referee_command="STOP",
        config=GameConfig(on_positive_half=True),
    )

    status, action = root.run(crowding)

    assert status == TaskStatus.SUCCESS
    assert len(action) == 4


def test_stop_waits_for_the_config_before_choosing_a_side(root):
    crowding = TickContext(
        ally_robots={i: FakeRobot(id=i, position_x=100.0 * i) for i in range(4)},
        balls=(FakeBall(),),
        referee_command="STOP",
        config=None,
    )

    assert root.run(crowding)[0] == TaskStatus.RUNNING
