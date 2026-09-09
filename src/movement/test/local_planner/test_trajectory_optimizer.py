from movement.local_planner import TrajectoryOptimizer
from movement.entities.motion import MotionState
from movement.entities.trajectory import Trajectory

from utils.math_util import Vector2D


class TestTrajectoryOptimizer:
    def test_optimize_shortens_path(self, generator):
        optimizer = TrajectoryOptimizer(trys=10, early_stop=5)

        # Create a suboptimal path (two segments instead of one)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        mid = MotionState(Vector2D(500, 500), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))

        seg1 = generator.generate(start, mid)
        seg2 = generator.generate(mid, goal)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)

        initial_duration = traj.get_total_duration()
        optimized_traj = optimizer.optimize(traj, generator, [])

        # It should ideally be shorter or equal
        assert optimized_traj.get_total_duration() <= initial_duration + 1e-6

    def test_optimize_empty_trajectory_is_noop(self, generator):
        optimizer = TrajectoryOptimizer(trys=10, early_stop=5)
        empty_traj = Trajectory()

        result = optimizer.optimize(empty_traj, generator, [])
        assert result is empty_traj
        assert result.root is None

    def test_optimize_respects_obstacles(self, generator):
        # An obstacle sitting right where the shortcut would go should
        # prevent the optimizer from collapsing the detour through it.
        from movement.entities.obstacle import GenericCircleObstacle
        from movement.local_planner import CollisionEngine

        optimizer = TrajectoryOptimizer(trys=25, early_stop=25)

        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        mid = MotionState(Vector2D(500, 500), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))

        seg1 = generator.generate(start, mid)
        seg2 = generator.generate(mid, goal)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)

        obstacles = [GenericCircleObstacle(Vector2D(500, 0), 200, padding=0)]

        optimized_traj = optimizer.optimize(traj, generator, obstacles)
        assert not CollisionEngine.is_collision(optimized_traj.root, obstacles)
