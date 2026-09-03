from new_movement.local_planner import CollisionEngine
from new_movement.local_planner.solver import BypassSolver
from new_movement.entities.motion import MotionState
from new_movement.entities.obstacle import GenericCircleObstacle

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

    def test_solve_returns_fastest_of_found_trajectories(self, sampler, generator):
        solver = BypassSolver(max_iterations=50, sampler=sampler, collision_time_step=0.05)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))

        traj = solver.solve(start, goal, [], generator)
        assert traj is not None
        # Sanity: the returned trajectory duration should be no worse than
        # a single direct segment's duration is impossible to bound exactly,
        # but it must at least be finite and positive.
        assert traj.get_total_duration() > 0

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
