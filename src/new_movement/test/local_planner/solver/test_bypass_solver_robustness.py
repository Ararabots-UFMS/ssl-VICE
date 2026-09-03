"""
The bypass solver must survive a via point it cannot steer to.

TrajectoryGenerator does not always return a profile that reaches the state it was
asked for; when its steering solver gives up it hands back a zero-duration path parked
at its own start. Chaining that raised out of add_child, and the exception escaped all
the way to the planner node, which logged it and left the robot with no plan for that
cycle — about 12% of plans in a tight corridor.
"""

import pytest

from new_movement.entities.motion import MotionConstraints, MotionState
from new_movement.entities.motion.motion_path import MotionPath
from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment
from new_movement.local_planner.informed_sampler import InformedSampler
from new_movement.local_planner.solver import BypassSolver
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator

from utils.math_util import Vector2D


class _StuckGenerator:
    """Stands in for a steering solver that cannot solve the first leg."""

    def __init__(self):
        self.real = TrajectoryGenerator(
            MotionConstraints(Vector2D(2000.0, 2000.0), Vector2D(1500.0, 1500.0))
        )
        self.calls = 0

    def generate(self, current, target):
        self.calls += 1
        if self.calls == 1:
            # The degenerate result: no primitives, so it never leaves `current`.
            return TrajectorySegment(current.position, current.velocity, MotionPath([]))
        return self.real.generate(current, target)


@pytest.fixture
def solver():
    return BypassSolver(
        sampler=InformedSampler(field_length=12000, field_width=9000, max_velocity=2000),
        max_iterations=5,
        collision_time_step=0.04,
    )


class TestUnsteerableViaPointsAreRejected:
    def test_build_returns_none_instead_of_raising(self, solver):
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(2000.0, 0.0), Vector2D(0.0, 0.0))
        via = MotionState(Vector2D(1000.0, 800.0), Vector2D(500.0, 0.0))

        result = solver._build(start, goal, [], _StuckGenerator(), via)

        assert result is None

    def test_a_reachable_via_point_still_builds(self, solver):
        generator = TrajectoryGenerator(
            MotionConstraints(Vector2D(2000.0, 2000.0), Vector2D(1500.0, 1500.0))
        )
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(2000.0, 0.0), Vector2D(0.0, 0.0))
        via = MotionState(Vector2D(1000.0, 800.0), Vector2D(500.0, 0.0))

        result = solver._build(start, goal, [], generator, via)

        assert result is not None
        assert result.get_total_duration() > 0

    def test_solve_survives_a_generator_that_keeps_failing(self, solver):
        """Every candidate unusable is a planning failure, never an exception."""

        class _AlwaysStuck:
            def generate(self, current, target):
                return TrajectorySegment(
                    current.position, current.velocity, MotionPath([])
                )

        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(2000.0, 0.0), Vector2D(0.0, 0.0))

        assert solver.solve(start, goal, [], _AlwaysStuck()) is None
