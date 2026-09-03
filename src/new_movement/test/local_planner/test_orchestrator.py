from new_movement.local_planner import Orchestrator, CollisionEngine
from new_movement.local_planner.solver import PlanningStatus, SolverConfig
from new_movement.entities.motion import MotionState
from new_movement.entities.obstacle import GenericCircleObstacle

from utils.math_util import Vector2D


class TestOrchestrator:
    def test_find_direct_path(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))

        traj = planner.find(start, goal, [])
        assert planner.status == PlanningStatus.DIRECT_PATH
        assert traj.root is not None

    def test_find_bypass(self):
        planner = Orchestrator(SolverConfig(max_iterations=100))
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 300, padding=0)]

        traj = planner.find(start, goal, obstacles)
        assert planner.status == PlanningStatus.BYPASS_FOUND
        assert not CollisionEngine.is_collision(traj.root, obstacles)

    def test_handle_start_and_goal_collisions_start_inside(self):
        planner = Orchestrator()
        obs = GenericCircleObstacle(Vector2D(0, 0), 200, padding=0)
        start = MotionState(Vector2D(50, 50), Vector2D(0, 0))  # Inside obstacle
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))

        new_start, new_goal, safety_traj = planner._handle_start_and_goal_collisions(start, goal, [obs])

        # Safety traj should take us out of the obstacle
        assert safety_traj.root is not None
        assert not obs.isCollidingAt(new_start.position)

    def test_handle_start_and_goal_collisions_goal_inside(self):
        planner = Orchestrator()
        obs = GenericCircleObstacle(Vector2D(1000, 0), 200, padding=0)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))  # Inside obstacle

        new_start, new_goal, safety_traj = planner._handle_start_and_goal_collisions(start, goal, [obs])

        # No safety trajectory needed since start is fine, but the goal
        # should have been pushed outside the obstacle.
        assert safety_traj.root is None
        assert not obs.isCollidingAt(new_goal.position)

    def test_find_recovery_when_no_bypass_found(self):
        # With max_iterations=1 and a very large obstacle, the bypass
        # solver is extremely unlikely to find a collision-free via point,
        # forcing the orchestrator into its recovery fallback.
        planner = Orchestrator(SolverConfig(max_iterations=1))
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 5000, padding=0)]

        traj = planner.find(start, goal, obstacles)
        assert planner.status in (PlanningStatus.RECOVERY, PlanningStatus.BYPASS_FOUND)
        if planner.status == PlanningStatus.RECOVERY:
            # Recovery trajectory should bring velocity to a stop at the
            # current position.
            assert traj.root is not None
            dest = traj.get_destination()
            assert dest.velocity.x == 0
            assert dest.velocity.y == 0

    def test_validate_continuity_empty_trajectory(self):
        planner = Orchestrator()
        from new_movement.entities.trajectory import Trajectory
        assert planner.validate_continuity(Trajectory()) is True

    def test_validate_continuity_direct_path(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        traj = planner.find(start, goal, [])
        assert planner.validate_continuity(traj) is True
