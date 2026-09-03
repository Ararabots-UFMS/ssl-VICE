"""
Obstacles that overlap must not trade the robot back and forth.

Escapes used to be resolved one obstacle at a time: each was asked where the robot
should go and its answer taken without checking it against the others. The right-hand
penalty area shares its goal-line edge with the field border, so a robot inside that
area was sent out through that edge — off the field — and the border sent it straight
back inside the penalty area. The outward half of that loop is what drove a robot into
the wall on a command to (4500, 300).

Numbers here are the ones the running system reported.
"""

import random

import numpy as np
import pytest

from new_movement.entities.motion import MotionState
from new_movement.entities.obstacle import FieldBorderObstacle, PenaltyAreaObstacle
from new_movement.entities.trajectory.trajectory_sampler import TrajectorySampler
from new_movement.local_planner.orchestrator import Orchestrator
from new_movement.local_planner.solver import PlanningStatus

from utils.field_util import FieldSide
from utils.math_util import Vector2D

# Division B, which is the geometry that puts a goal at x=4500 past the touch line.
FIELD_LENGTH = 9000.0
FIELD_WIDTH = 6000.0


def _geometry():
    return type(
        "G",
        (),
        {"field_lines": [], "field_length": FIELD_LENGTH, "field_width": FIELD_WIDTH},
    )()


@pytest.fixture(autouse=True)
def deterministic_sampling():
    """
    The bypass solver draws from numpy's global RNG, so without this a result depends on
    how many samples the tests before it happened to consume.
    """
    np.random.seed(20260903)
    random.seed(20260903)


@pytest.fixture
def field():
    geometry = _geometry()
    border = FieldBorderObstacle(geometry)
    return border, [
        border,
        PenaltyAreaObstacle(geometry, FieldSide.RIGHT),
        PenaltyAreaObstacle(geometry, FieldSide.LEFT),
    ]


def _leaves_the_field(trajectory, border, samples=400):
    sampler = TrajectorySampler(trajectory.root)
    if sampler.duration <= 0:
        return 0
    points = sampler.positions(np.linspace(0.0, sampler.duration, samples))
    return sum(
        1
        for point in points
        if border.isCollidingAt(Vector2D(float(point[0]), float(point[1])))
    )


class TestTheEscapeStaysOnTheField:
    # Where the robot actually was when it jammed.
    INSIDE_PENALTY_AREA = Vector2D(4403.83, 948.03)

    def test_the_penalty_area_never_pushes_toward_the_goal_line(self):
        """
        The right-hand area's +x edge is the field boundary, so leaving through it is
        always illegal. Midfield is the only usable direction in x.
        """
        area = PenaltyAreaObstacle(_geometry(), FieldSide.RIGHT)
        min_x, max_x, _, _ = area._bounds()

        escaped = area.adaptDestination(self.INSIDE_PENALTY_AREA)

        assert escaped.x <= max_x, "escaped through the goal-line edge"
        assert min_x <= escaped.x

    def test_a_robot_in_the_area_is_given_a_legal_exit(self, field):
        border, obstacles = field
        planner = Orchestrator()

        exit_point, reachable = planner._clear_point(self.INSIDE_PENALTY_AREA, obstacles)

        assert reachable
        assert not border.isCollidingAt(exit_point)
        assert all(not planner._collides_at(o, exit_point) for o in obstacles)

    def test_the_plan_never_leaves_the_field(self, field):
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(self.INSIDE_PENALTY_AREA, Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(4500.0, 300.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, obstacles)

        assert _leaves_the_field(trajectory, border) == 0

    def test_an_ordinary_move_is_unaffected(self, field):
        """The escape changes must not cost the common case its direct route."""
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(Vector2D(2000.0, 500.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(3000.0, 300.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, obstacles)
        destination = trajectory.get_destination()

        assert trajectory.status != PlanningStatus.RECOVERY
        assert destination.position.distance(goal.position) < 1.0
        assert _leaves_the_field(trajectory, border) == 0

    def test_leaving_the_area_does_not_require_leaving_the_field(self, field):
        """
        Routing out of the penalty area and across to midfield needs to go up and over,
        which is two turns and more than the single via point BypassSolver can express,
        so it may legitimately fall back to a stop. What it must never do is take the
        robot off the field to get there.
        """
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(self.INSIDE_PENALTY_AREA, Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(3000.0, 300.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, obstacles)

        assert _leaves_the_field(trajectory, border) == 0

    def test_an_unreachable_goal_is_brought_inside_the_field(self, field):
        """(4500, 300) is past the touch line and inside the penalty area."""
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(self.INSIDE_PENALTY_AREA, Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(4500.0, 300.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, obstacles)
        destination = trajectory.get_destination()

        assert not border.isCollidingAt(destination.position)
        assert all(
            not planner._collides_at(o, destination.position) for o in obstacles
        )


class TestNoEscapeIsBetterThanAnIllegalOne:
    def test_contradicting_obstacles_report_failure_rather_than_picking_one(self):
        """
        Two obstacles whose only exits are into each other. Committing to either one's
        answer is what put the robot off the field, so the resolver has to say so.
        """
        planner = Orchestrator()

        class _Everywhere:
            """Occupies the whole plane and points every escape back to the origin."""

            def isCollidingAt(self, position, *_):
                return True

            def adaptDestination(self, position, *_):
                return Vector2D(0.0, 0.0)

        point, reachable = planner._clear_point(Vector2D(100.0, 100.0), [_Everywhere()])

        assert not reachable
        assert point == Vector2D(100.0, 100.0), "an unresolved point must not move"

    def test_a_failed_escape_emits_no_safety_segment(self, ):
        planner = Orchestrator()

        class _Everywhere:
            def isCollidingAt(self, position, *_):
                return True

            def adaptDestination(self, position, *_):
                return Vector2D(0.0, 0.0)

        start = MotionState(Vector2D(100.0, 100.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(2000.0, 0.0), Vector2D(0.0, 0.0))

        new_start, _, safety = planner._handle_start_and_goal_collisions(
            start, goal, [_Everywhere()]
        )

        assert planner.escape_failed
        assert safety.root is None
        assert new_start.position == start.position
