import pytest

from new_movement.local_planner import CollisionEngine
from new_movement.local_planner.solver import BypassSolver
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator
from new_movement.entities.motion import MotionConstraints, MotionState
from new_movement.entities.motion.motion_path import MotionPath
from new_movement.entities.obstacle import GenericCircleObstacle
from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment

from utils.math_util import Vector2D


class TestBypassSolver:
    def test_solve_blocked_path(self, sampler, generator):
        solver = BypassSolver(max_iterations=100, sampler=sampler, collision_time_step=0.05)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))

        # Big obstacle blocking the direct line
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 500, padding=0)]

        traj = solver.solve(start, goal, obstacles, generator)
        assert traj is not None
        assert traj.root is not None
        # Verify result is collision free
        assert not CollisionEngine.is_collision(traj.root, obstacles)
        assert not CollisionEngine.is_collision(traj.root.child, obstacles)

    def test_solve_no_obstacles_still_finds_path(self, sampler, generator):
        # With no obstacles at all, every sampled via-point trajectory is
        # trivially collision free, so a trajectory should always be found.
        solver = BypassSolver(max_iterations=5, sampler=sampler, collision_time_step=0.05)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))

        traj = solver.solve(start, goal, [], generator)
        assert traj is not None
        assert traj.root is not None

    def test_solve_returns_none_when_obstacle_leaves_no_room(self, sampler, generator):
        # An enormous obstacle centered on both start and goal leaves
        # essentially no way for a via-point trajectory to avoid it within
        # a very small iteration budget, exercising the "no trajectories
        # found" path that returns None.
        solver = BypassSolver(max_iterations=3, sampler=sampler, collision_time_step=0.05)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(100, 0), Vector2D(0, 0))

        obstacles = [GenericCircleObstacle(Vector2D(0, 0), 20000, padding=0)]

        traj = solver.solve(start, goal, obstacles, generator)
        assert traj is None


class TestBypassSolverStability:
    """
    Sampling alone returns a structurally different path every cycle for unchanged
    inputs, so the controller never gets a reference it can settle on. The previous via
    point is re-solved from the current start and defended by cost_margin.
    """

    START = MotionState(Vector2D(-2000, 0), Vector2D(0, 0))
    GOAL = MotionState(Vector2D(2000, 0), Vector2D(0, 0))

    def _solver(self, sampler, margin=0.15):
        return BypassSolver(
            max_iterations=20,
            sampler=sampler,
            collision_time_step=0.04,
            cost_margin=margin,
        )

    def _blocking(self, radius=500):
        return [GenericCircleObstacle(Vector2D(0, 0), radius, padding=0)]

    def test_the_via_point_is_reported(self, sampler, generator):
        traj = self._solver(sampler).solve(
            self.START, self.GOAL, self._blocking(), generator
        )

        assert traj.via_state is not None

    def test_a_via_point_is_only_replaced_by_a_real_improvement(self, sampler, generator):
        """
        The contract is the margin, not absolute stickiness: a committed route is kept
        unless a sampled one beats it by cost_margin.
        """
        solver = self._solver(sampler, margin=0.15)
        obstacles = self._blocking()

        current = solver.solve(self.START, self.GOAL, obstacles, generator)
        replacements = 0

        for _ in range(40):
            incumbent = current.get_total_duration()
            result = solver.solve(
                self.START, self.GOAL, obstacles, generator, current.via_state
            )
            if result.via_state.position.distance(current.via_state.position) > 1e-9:
                replacements += 1
                assert result.get_total_duration() < incumbent * 0.85
            current = result

        # And it settles rather than churning.
        assert replacements < 5

    def test_a_blocked_via_point_is_abandoned(self, sampler, generator):
        solver = self._solver(sampler)
        obstacles = self._blocking()

        first = solver.solve(self.START, self.GOAL, obstacles, generator)
        blocked = obstacles + [
            GenericCircleObstacle(first.via_state.position, 500, padding=0)
        ]

        second = solver.solve(self.START, self.GOAL, blocked, generator, first.via_state)

        assert second is not None
        assert second.via_state.position.distance(first.via_state.position) > 100.0
        assert not CollisionEngine.is_collision(second.root, blocked)
        assert not CollisionEngine.is_collision(second.root.child, blocked)

    def test_a_zero_margin_takes_any_improvement(self, sampler, generator):
        """The margin is what makes it sticky, not the warm start on its own."""
        solver = self._solver(sampler, margin=0.0)
        obstacles = self._blocking()

        first = solver.solve(self.START, self.GOAL, obstacles, generator)
        durations = {first.get_total_duration()}
        via = first.via_state
        for _ in range(20):
            result = solver.solve(self.START, self.GOAL, obstacles, generator, via)
            via = result.via_state
            durations.add(result.get_total_duration())

        assert len(durations) > 1


class TestUnsteerableViaPointsAreRejected:
    """
    TrajectoryGenerator does not always return a profile that reaches the state it was
    asked for; when its steering solver gives up it hands back a zero-duration path
    parked at its own start. Chaining that raised out of add_child, and the exception
    escaped to the planner node, costing the robot its plan for that cycle — about 12%
    of plans in a tight corridor.
    """

    START = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
    GOAL = MotionState(Vector2D(2000.0, 0.0), Vector2D(0.0, 0.0))
    VIA = MotionState(Vector2D(1000.0, 800.0), Vector2D(500.0, 0.0))

    def _real_generator(self):
        return TrajectoryGenerator(
            MotionConstraints(Vector2D(2000.0, 2000.0), Vector2D(1500.0, 1500.0))
        )

    def _stuck_segment(self, current, _target):
        """The degenerate result: no primitives, so it never leaves ``current``."""
        return TrajectorySegment(current.position, current.velocity, MotionPath([]))

    @pytest.fixture
    def solver(self, sampler):
        return BypassSolver(sampler=sampler, max_iterations=5, collision_time_step=0.04)

    def test_build_returns_none_instead_of_raising(self, solver):
        real = self._real_generator()
        calls = []

        class _StuckOnTheFirstLeg:
            def generate(_self, current, target):
                calls.append(1)
                if len(calls) == 1:
                    return self._stuck_segment(current, target)
                return real.generate(current, target)

        assert solver._build(
            self.START, self.GOAL, [], _StuckOnTheFirstLeg(), self.VIA
        ) is None

    def test_a_reachable_via_point_still_builds(self, solver):
        result = solver._build(
            self.START, self.GOAL, [], self._real_generator(), self.VIA
        )

        assert result is not None
        assert result.get_total_duration() > 0

    def test_solve_survives_a_generator_that_keeps_failing(self, solver):
        """Every candidate unusable is a planning failure, never an exception."""

        class _AlwaysStuck:
            generate = self._stuck_segment

        assert solver.solve(self.START, self.GOAL, [], _AlwaysStuck()) is None
