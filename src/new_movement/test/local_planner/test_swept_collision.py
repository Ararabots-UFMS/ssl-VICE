"""
Regression tests for swept collision checking.

Sampling a trajectory at discrete instants and asking "is this point inside an
obstacle" misses any encounter that begins and ends between two samples. Measured
against a dense reference, that lost about 5% of grazing collisions at the step size
the planner runs at, and halving the step only quartered the error — point sampling
converges linearly and never reaches zero.

These tests pin the swept behaviour: the segment between two samples is tested, not
just its endpoints.
"""

import math
import random

import numpy as np
import pytest

from new_movement.entities.motion import MotionConstraints, MotionState
from new_movement.entities.obstacle import (
    EnemyRobotObstacle,
    FieldBorderObstacle,
    GenericCircleObstacle,
    PenaltyAreaObstacle,
)
from new_movement.entities.obstacle.ally_robot_obstacle import AllyRobotObstacle
from new_movement.local_planner.collision_engine import CollisionEngine
from new_movement.local_planner.solver import SolverConfig
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator

from utils.field_util import FieldSide
from utils.math_util import Vector2D

# Fine enough that the reference verdict is exact for these geometries.
REFERENCE_STEP = 0.002
PLANNER_STEP = SolverConfig().collision_time_step


def _geometry():
    return type(
        "G", (), {"field_lines": [], "field_length": 12000.0, "field_width": 9000.0}
    )()


@pytest.fixture
def swept_generator():
    config = SolverConfig()
    return TrajectoryGenerator(
        MotionConstraints(config.max_velocity, config.max_acceleration)
    )


def _grazing_cases(generator, radius, count=120, seed=11):
    """
    Trajectories with a circle placed so its boundary sits right on the path.

    This is the geometry the bypass solver actually produces: it samples via points
    close to obstacles, so near-tangential approaches are over-represented in play
    compared with uniformly scattered obstacles.
    """
    rng = random.Random(seed)
    cases = []
    while len(cases) < count:
        start = MotionState(
            Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
            Vector2D(rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)),
        )
        goal = MotionState(
            Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
            Vector2D(0.0, 0.0),
        )
        segment = generator.generate(start, goal)
        duration = segment.get_total_duration()
        if not 0 < duration <= 3.0:
            continue

        state = segment.get_state(rng.uniform(0.1 * duration, 0.9 * duration))
        speed = math.hypot(state.velocity.x, state.velocity.y)
        if speed < 1e-6:
            continue

        # Offset perpendicular to travel by about one radius: the disc clips the path.
        normal = (-state.velocity.y / speed, state.velocity.x / speed)
        offset = radius * rng.uniform(0.88, 1.02) * rng.choice([1, -1])
        centre = Vector2D(
            state.position.x + normal[0] * offset,
            state.position.y + normal[1] * offset,
        )
        cases.append((segment, GenericCircleObstacle(centre, radius, padding=0)))
    return cases


class TestNoGrazingCollisionIsMissed:
    @pytest.mark.parametrize("radius", [180.0, 150.0])
    def test_matches_a_dense_reference(self, swept_generator, radius):
        """
        The whole point of the sweep: at the planner's own step size, the verdict must
        agree with one taken twenty times finer.
        """
        missed = 0
        real = 0
        for segment, obstacle in _grazing_cases(swept_generator, radius):
            if CollisionEngine.is_collision(segment, [obstacle], REFERENCE_STEP):
                real += 1
                if not CollisionEngine.is_collision(segment, [obstacle], PLANNER_STEP):
                    missed += 1

        assert real > 0, "the fixture produced no collisions to check"
        assert missed == 0, f"{missed}/{real} grazing collisions missed"


class TestSweptVersusPointSampling:
    def test_a_clip_between_two_samples_is_caught(self):
        """
        A disc sitting between two consecutive samples, touching neither. The old point
        check reported clear because it only ever looked at the endpoints.
        """
        obstacle = GenericCircleObstacle(Vector2D(500.0, 0.0), 60.0, padding=0)
        starts = np.array([[0.0, 0.0]])
        ends = np.array([[1000.0, 0.0]])

        assert not obstacle._check_positions(np.vstack([starts, ends]))
        assert obstacle.batch_collides_segments(
            starts, ends, np.array([0.0]), np.array([0.5])
        )

    def test_a_segment_passing_wide_is_still_clear(self):
        obstacle = GenericCircleObstacle(Vector2D(500.0, 400.0), 60.0, padding=0)
        assert not obstacle.batch_collides_segments(
            np.array([[0.0, 0.0]]),
            np.array([[1000.0, 0.0]]),
            np.array([0.0]),
            np.array([0.5]),
        )

    def test_a_zero_length_segment_behaves_like_its_point(self):
        obstacle = GenericCircleObstacle(Vector2D(0.0, 0.0), 100.0, padding=0)
        inside = np.array([[50.0, 0.0]])
        outside = np.array([[500.0, 0.0]])

        assert obstacle.batch_collides_segments(
            inside, inside, np.array([0.0]), np.array([0.0])
        )
        assert not obstacle.batch_collides_segments(
            outside, outside, np.array([0.0]), np.array([0.0])
        )


class TestPenaltyAreaCorners:
    def test_a_corner_clip_is_caught(self):
        """A segment cutting across a corner enters the box without either end inside."""
        area = PenaltyAreaObstacle(_geometry(), FieldSide.LEFT, padding=0.0)
        min_x, _, min_y, _ = area._bounds()

        # Cuts a genuine triangle off the corner: inside for the middle third of the
        # segment, outside at both ends. A symmetric pair would only touch the corner
        # vertex, which is tangency rather than entry.
        start = np.array([[min_x - 100.0, min_y + 200.0]])
        end = np.array([[min_x + 200.0, min_y - 100.0]])

        assert not area._check_positions(np.vstack([start, end]))
        assert area._check_segments(start, end)

    def test_a_segment_clear_of_the_box_stays_clear(self):
        area = PenaltyAreaObstacle(_geometry(), FieldSide.LEFT, padding=0.0)
        _, max_x, _, max_y = area._bounds()
        start = np.array([[max_x + 500.0, max_y + 500.0]])
        end = np.array([[max_x + 900.0, max_y + 900.0]])

        assert not area._check_segments(start, end)


class TestFieldBorderNeedsNoSubdivision:
    def test_endpoints_decide_a_convex_region(self):
        """
        Leaving a convex rectangle cannot happen strictly between two interior points,
        so the endpoint check is already exact and the sweep must agree with it.
        """
        border = FieldBorderObstacle(_geometry(), padding=0.0)

        inside = np.array([[0.0, 0.0]])
        also_inside = np.array([[1000.0, 500.0]])
        outside = np.array([[100000.0, 0.0]])

        assert not border._check_segments(inside, also_inside)
        assert border._check_segments(inside, outside)


class TestMovingObstacles:
    def test_an_ally_crossing_mid_step_is_caught(self):
        """
        Both bodies move, so the pair can be far apart at both ends of the step and
        still pass through each other in between.
        """
        ally_start = Vector2D(0.0, -400.0)
        ally_velocity = Vector2D(0.0, 800.0)

        class _CrossingTrajectory:
            def get_position(self, t):
                return ally_start.add(ally_velocity.multiplyByScalar(t))

        obstacle = AllyRobotObstacle(
            MotionState(ally_start, ally_velocity),
            _CrossingTrajectory(),
            time_offset=0.0,
            radius=100.0,
        )

        # Our robot crosses the same point going the other way, over the same second.
        starts = np.array([[-400.0, 0.0]])
        ends = np.array([[400.0, 0.0]])

        assert not obstacle.batch_collides(
            np.vstack([starts, ends]), np.array([0.0, 1.0])
        )
        assert obstacle.batch_collides_segments(
            starts, ends, np.array([0.0]), np.array([1.0])
        )

    def test_an_enemy_tube_clipped_between_samples_is_caught(self):
        enemy = EnemyRobotObstacle(
            MotionState(Vector2D(500.0, -300.0), Vector2D(0.0, 600.0)), radius=90.0
        )
        assert enemy.batch_collides_segments(
            np.array([[0.0, 0.0]]),
            np.array([[1000.0, 0.0]]),
            np.array([0.0]),
            np.array([1.0]),
        )

    def test_an_enemy_well_clear_is_not_flagged(self):
        enemy = EnemyRobotObstacle(
            MotionState(Vector2D(500.0, 3000.0), Vector2D(0.0, 10.0)), radius=90.0
        )
        assert not enemy.batch_collides_segments(
            np.array([[0.0, 0.0]]),
            np.array([[1000.0, 0.0]]),
            np.array([0.0]),
            np.array([1.0]),
        )
