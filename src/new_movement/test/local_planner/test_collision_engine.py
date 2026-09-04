import math
import random

import numpy as np

from new_movement.local_planner import CollisionEngine, TrajectoryGenerator
from new_movement.local_planner.solver import SolverConfig
from new_movement.entities.motion import MotionConstraints, MotionState
from new_movement.entities.obstacle import (
    EnemyRobotObstacle,
    FieldBorderObstacle,
    GenericCircleObstacle,
    PenaltyAreaObstacle,
)
from new_movement.entities.trajectory.trajectory_sampler import TrajectorySampler

from utils.field_util import FieldSide
from utils.math_util import Vector2D


class TestCollisionEngine:
    def test_no_collision(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(500, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        obstacles = [GenericCircleObstacle(Vector2D(1000, 1000), 100, padding=0)]
        assert not CollisionEngine.is_collision(traj_seg, obstacles)

    def test_static_collision(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        # Obstacle right in the middle
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 100, padding=0)]
        assert CollisionEngine.is_collision(traj_seg, obstacles)

    def test_dynamic_collision(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        # Enemy robot moving towards the path
        enemy_state = MotionState(Vector2D(1000, 500), Vector2D(0, -1000))
        obstacles = [EnemyRobotObstacle(enemy_state, radius=100)]

        # At some point t, it should collide
        assert CollisionEngine.is_collision(traj_seg, obstacles)

    def test_no_obstacles(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        assert not CollisionEngine.is_collision(traj_seg, [])

    def test_zero_duration_trajectory_never_collides(self, generator):
        # Start == goal produces a (near) zero-duration segment; the engine
        # should short-circuit and report no collision even with an
        # obstacle sitting right on top of the point.
        state = MotionState(Vector2D(100, 100), Vector2D(0, 0))
        traj_seg = generator.generate(state, state)

        obstacles = [GenericCircleObstacle(Vector2D(100, 100), 500, padding=0)]
        assert not CollisionEngine.is_collision(traj_seg, obstacles)


REFERENCE_STEP = 0.002  # fine enough that the reference verdict is exact here
PLANNER_STEP = SolverConfig().collision_time_step


def _geometry():
    return type(
        "G", (), {"field_lines": [], "field_length": 12000.0, "field_width": 9000.0}
    )()


def _planner_generator():
    config = SolverConfig()
    return TrajectoryGenerator(
        MotionConstraints(config.max_velocity, config.max_acceleration)
    )


class TestNoGrazingCollisionIsMissed:
    """
    Sampling a trajectory at discrete instants and asking "is this point inside an
    obstacle" misses any encounter that begins and ends between two samples. Measured
    against a dense reference that lost ~5% of grazing collisions at the planner's own
    step, and halving the step only quartered the error.
    """

    RADIUS = 180.0

    def _grazing_cases(self, generator, count=120, seed=11):
        """
        Trajectories with a circle placed so its boundary sits right on the path — the
        geometry the bypass solver actually produces, since it samples via points close
        to obstacles.
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
            offset = self.RADIUS * rng.uniform(0.88, 1.02) * rng.choice([1, -1])
            centre = Vector2D(
                state.position.x + normal[0] * offset,
                state.position.y + normal[1] * offset,
            )
            cases.append((segment, GenericCircleObstacle(centre, self.RADIUS, padding=0)))
        return cases

    def test_the_planner_step_agrees_with_a_dense_reference(self):
        generator = _planner_generator()
        missed = real = 0

        for segment, obstacle in self._grazing_cases(generator):
            if CollisionEngine.is_collision(segment, [obstacle], REFERENCE_STEP):
                real += 1
                if not CollisionEngine.is_collision(segment, [obstacle], PLANNER_STEP):
                    missed += 1

        assert real > 0, "the fixture produced no collisions to check"
        assert missed == 0, f"{missed}/{real} grazing collisions missed"


class TestBroadPhase:
    """
    Comparing axis-aligned boxes first skips the swept test for obstacles that cannot
    possibly be reached. It is an optimisation, not a policy: the verdicts have to be
    identical, hit for hit and miss for miss.
    """

    def _without_broad_phase(self, segment, obstacles, time_step):
        sampler = TrajectorySampler(segment)
        if sampler.duration <= 0:
            return False

        steps = max(1, int(np.ceil(sampler.duration / time_step)))
        times = np.linspace(0.0, sampler.duration, steps + 1)
        positions = sampler.positions(times)
        return any(
            obs.batch_collides_segments(
                positions[:-1], positions[1:], times[:-1], times[1:]
            )
            for obs in obstacles
        )

    def test_it_agrees_with_testing_every_obstacle(self):
        generator = _planner_generator()
        rng = random.Random(23)
        step = PLANNER_STEP

        for _ in range(400):
            obstacles = [
                FieldBorderObstacle(_geometry()),
                PenaltyAreaObstacle(_geometry(), FieldSide.RIGHT),
                PenaltyAreaObstacle(_geometry(), FieldSide.LEFT),
                GenericCircleObstacle(
                    Vector2D(rng.uniform(-2000, 2000), rng.uniform(-1500, 1500)), 60
                ),
            ]
            for _ in range(rng.randint(0, 21)):
                obstacles.append(
                    EnemyRobotObstacle(
                        MotionState(
                            Vector2D(rng.uniform(-4500, 4500), rng.uniform(-3000, 3000)),
                            Vector2D(rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)),
                        ),
                        radius=200,
                    )
                )

            segment = generator.generate(
                MotionState(
                    Vector2D(rng.uniform(-4300, 4300), rng.uniform(-2800, 2800)),
                    Vector2D(rng.uniform(-1500, 1500), rng.uniform(-1500, 1500)),
                ),
                MotionState(
                    Vector2D(rng.uniform(-4300, 4300), rng.uniform(-2800, 2800)),
                    Vector2D(0.0, 0.0),
                ),
            )

            assert CollisionEngine.is_collision(
                segment, obstacles, step
            ) == self._without_broad_phase(segment, obstacles, step)

    def test_a_distant_obstacle_is_never_asked(self, generator):
        """The point of the exercise: far obstacles cost a box comparison, not a sweep."""

        class Counting(GenericCircleObstacle):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.sweeps = 0

            def batch_collides_segments(self, *args, **kwargs):
                self.sweeps += 1
                return super().batch_collides_segments(*args, **kwargs)

        far = Counting(Vector2D(5000.0, 4000.0), 100.0)
        near = Counting(Vector2D(500.0, 0.0), 100.0)
        segment = generator.generate(
            MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0)),
            MotionState(Vector2D(1000.0, 0.0), Vector2D(0.0, 0.0)),
        )

        assert CollisionEngine.is_collision(segment, [far, near])
        assert far.sweeps == 0
        assert near.sweeps == 1
