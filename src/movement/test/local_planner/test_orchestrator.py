import numpy as np
import pytest

from movement.local_planner import Orchestrator, CollisionEngine
from movement.local_planner.solver import PlanningStatus, SolverConfig
from movement.entities.motion import MotionState
from movement.entities.obstacle import (
    EnemyRobotObstacle,
    FieldBorderObstacle,
    GenericCircleObstacle,
    PenaltyAreaObstacle,
)
from movement.entities.trajectory import Trajectory
from movement.entities.trajectory.trajectory_sampler import TrajectorySampler

from utils.field_util import FieldSide
from utils.math_util import Vector2D

# Division B, the geometry that puts a goal at x=4500 past the touch line.
FIELD_LENGTH = 9000.0
FIELD_WIDTH = 6000.0


def _geometry():
    return type(
        "G",
        (),
        {"field_lines": [], "field_length": FIELD_LENGTH, "field_width": FIELD_WIDTH},
    )()


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
            # Approximate rather than exact: the recovery starts from where the escape
            # segment actually ended, which is an integrated value, so the residual
            # velocity is floating-point dust rather than a literal zero.
            assert dest.velocity.x == pytest.approx(0.0, abs=1e-9)
            assert dest.velocity.y == pytest.approx(0.0, abs=1e-9)

    def test_validate_continuity_empty_trajectory(self):
        planner = Orchestrator()
        assert planner.validate_continuity(Trajectory()) is True

    def test_validate_continuity_direct_path(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        traj = planner.find(start, goal, [])
        assert planner.validate_continuity(traj) is True


class TestEscapingObstacles:
    """
    A robot pressed against another robot had every candidate path collide at t=0, so
    the solver fell through to a stop and it stayed stuck there. Escaping was gated to
    static obstacles, but adaptDestination is on the base Obstacle interface.
    """

    GOAL = MotionState(Vector2D(3000, 0), Vector2D(0, 0))

    def _pinned_against_a_robot(self):
        enemy = EnemyRobotObstacle(
            MotionState(Vector2D(0, 0), Vector2D(0, 0)), radius=190.0
        )
        # 60mm inside its radius: touching and overlapping, as after a collision.
        return enemy, MotionState(Vector2D(130.0, 0.0), Vector2D(0, 0))

    def test_the_escape_clears_the_boundary_not_just_reaches_it(self):
        """adaptDestination returns the boundary, where isCollidingAt is still true."""
        planner = Orchestrator()
        enemy, start = self._pinned_against_a_robot()

        escape, reachable = planner._clear_point(start.position, [enemy])

        assert reachable
        assert escape.distance(Vector2D(0, 0)) > 190.0
        assert not enemy.isCollidingAt(escape, 0.0)

    def test_a_clear_robot_is_left_alone(self):
        planner = Orchestrator()
        enemy = EnemyRobotObstacle(MotionState(Vector2D(0, 0), Vector2D(0, 0)), radius=190.0)
        clear = Vector2D(2000.0, 0.0)

        resolved, reachable = planner._clear_point(clear, [enemy])

        assert reachable
        assert resolved == clear

    def test_the_planner_no_longer_dead_ends_when_pinned(self):
        planner = Orchestrator()
        enemy, start = self._pinned_against_a_robot()

        _, _, safety = planner._handle_start_and_goal_collisions(
            start, self.GOAL, [enemy]
        )
        trajectory = planner.find(start, self.GOAL, [enemy])

        assert safety.root is not None
        assert trajectory.status != PlanningStatus.RECOVERY
        assert trajectory.root is not None


class TestEscapingStaysOnTheField:
    """
    Escapes used to be resolved one obstacle at a time, each answer taken without
    checking it against the others. The right-hand penalty area shares its goal-line
    edge with the field border, so a robot inside that area was sent out through that
    edge — off the field — and the border sent it straight back in. The outward half of
    that loop drove a robot into the wall on a command to (4500, 300).
    """

    # Where the robot actually was when it jammed.
    INSIDE_PENALTY_AREA = Vector2D(4403.83, 948.03)

    def test_the_penalty_area_never_pushes_toward_the_goal_line(self):
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

    @pytest.mark.parametrize("goal_x", [4500.0, 3000.0])
    def test_the_plan_never_leaves_the_field(self, field, goal_x):
        """
        Routing out to midfield needs two turns, more than the single via point
        BypassSolver can express, so it may legitimately fall back to a stop. What it
        must never do is take the robot off the field to get there.
        """
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(self.INSIDE_PENALTY_AREA, Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(goal_x, 300.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, obstacles)

        assert _leaves_the_field(trajectory, border) == 0

    def test_an_unreachable_goal_is_brought_inside_the_field(self, field):
        """(4500, 300) is past the touch line and inside the penalty area."""
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(self.INSIDE_PENALTY_AREA, Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(4500.0, 300.0), Vector2D(0.0, 0.0))

        destination = planner.find(start, goal, obstacles).get_destination()

        assert not border.isCollidingAt(destination.position)
        assert all(not planner._collides_at(o, destination.position) for o in obstacles)

    def test_an_ordinary_move_is_unaffected(self, field):
        """The escape changes must not cost the common case its direct route."""
        border, obstacles = field
        planner = Orchestrator()
        start = MotionState(Vector2D(2000.0, 500.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(3000.0, 300.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, obstacles)

        assert trajectory.status != PlanningStatus.RECOVERY
        assert trajectory.get_destination().position.distance(goal.position) < 1.0
        assert _leaves_the_field(trajectory, border) == 0

    def test_contradicting_obstacles_report_failure_rather_than_picking_one(self):
        """
        Two obstacles whose only exits are into each other. Committing to either one's
        answer is what put the robot off the field, so the resolver has to say so — and
        staying put lets the collision check fall through to a stop.
        """
        planner = Orchestrator()

        class _Everywhere:
            """Occupies the whole plane and points every escape back to the origin."""

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


class TestEscapingDoesNotBreakContinuity:
    """
    Escaping asks the steering solver to reach a point a few centimetres away while
    still carrying the robot's velocity, which is often not solvable. It returns its
    nearest attempt, and chaining the next segment onto the requested exit state rather
    than the achieved one left a gap Trajectory.append rejected — on the field, a burst
    of "Solver error: non continuous trajectory (position gap 62.4)" with the plan
    dropped for every one of those cycles.
    """

    def test_the_safety_segments_stay_continuous(self):
        planner = Orchestrator()
        obstacle = GenericCircleObstacle(Vector2D(0, 0), 300, padding=0)
        start = MotionState(Vector2D(80.0, 40.0), Vector2D(900.0, -700.0))
        goal = MotionState(Vector2D(2500.0, 0.0), Vector2D(0.0, 0.0))

        trajectory = planner.find(start, goal, [obstacle])

        node = trajectory.root
        assert node is not None
        while node is not None and node.child is not None:
            destination = node.get_local_destination()
            assert destination.position.distance(node.child.init_pos) < 1e-3
            assert destination.velocity.distance(node.child.init_vel) < 1e-3
            node = node.child

    def test_many_moving_starts_inside_an_obstacle_never_raise(self):
        import random

        planner = Orchestrator()
        obstacle = GenericCircleObstacle(Vector2D(0, 0), 400, padding=0)
        goal = MotionState(Vector2D(3000.0, 0.0), Vector2D(0.0, 0.0))
        rng = random.Random(17)

        for _ in range(150):
            start = MotionState(
                Vector2D(rng.uniform(-350, 350), rng.uniform(-350, 350)),
                Vector2D(rng.uniform(-1500, 1500), rng.uniform(-1500, 1500)),
            )
            assert planner.find(start, goal, [obstacle]) is not None


class TestRecoveryBraking:
    """
    Recovery returned the robot to the position it held when the plan was made. At speed
    that means overshooting and driving back, which showed up as positive along-track
    excursions of +262mm to +573mm right after every "no path" fallback.
    """

    SPEED = 2000.0

    def test_a_moving_robot_stops_at_its_braking_distance(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(0, 0), Vector2D(self.SPEED, 0.0))

        stop = planner._get_recovery_trajectory(start).get_destination()

        expected = self.SPEED ** 2 / (2.0 * planner.config.max_acceleration.x)
        assert stop.position.x == pytest.approx(expected, rel=0.02)
        assert stop.velocity.size() == pytest.approx(0.0, abs=1.0)

    def test_it_never_reverses(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(0, 0), Vector2D(self.SPEED, 0.0))

        recovery = planner._get_recovery_trajectory(start)
        samples = [
            recovery.get_state(float(t)).velocity.x
            for t in np.arange(0.0, recovery.get_total_duration(), 0.01)
        ]

        assert min(samples) >= -1.0

    def test_a_stationary_robot_stays_put(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(500.0, -200.0), Vector2D(0, 0))

        stop = planner._get_recovery_trajectory(start).get_destination()

        assert stop.position.distance(start.position) == pytest.approx(0.0, abs=1.0)
