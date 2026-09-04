import pytest

from new_movement.local_planner.solver import SolverConfig

from utils.math_util import Vector2D


class TestSolverConfig:
    def test_default_values(self):
        config = SolverConfig()

        assert config.max_iterations == 20
        assert config.field_length == 12000.0
        assert config.field_width == 9000.0
        # Bounds are magnitudes and must be positive on both axes: MoveConstraints
        # derives min = -max, so a negative y inverted the bounds and broke braking.
        assert config.max_velocity == Vector2D(2000.0, 2000.0)
        assert config.max_acceleration == Vector2D(1500.0, 1500.0)
        assert config.continuity_threshold == 1e-3
        # 0.2s advanced 400mm between samples at top speed, wide enough to tunnel
        # straight through an obstacle.
        assert config.collision_time_step == 0.04

    def test_can_override_individual_fields(self):
        config = SolverConfig(max_iterations=100, collision_time_step=0.05)

        assert config.max_iterations == 100
        assert config.collision_time_step == 0.05
        # Untouched fields keep their defaults
        assert config.field_length == 12000.0

    def test_instances_are_independent(self):
        a = SolverConfig(max_iterations=1)
        b = SolverConfig(max_iterations=2)

        assert a.max_iterations == 1
        assert b.max_iterations == 2
        # default_factory, so the Vector2D defaults are not shared either.
        assert a.max_velocity is not b.max_velocity

    @pytest.mark.parametrize(
        "bounds",
        [
            {"max_velocity": Vector2D(2000.0, -2000.0)},
            {"max_acceleration": Vector2D(1500.0, -1500.0)},
        ],
    )
    def test_a_negative_bound_is_rejected(self, bounds):
        """
        A negative bound inverts min = -max, and the trapezoidal steer then brakes the
        wrong way and stops enforcing the velocity cap — which left ~65% of generated
        trajectories short of their goal without ever raising.
        """
        with pytest.raises(ValueError):
            SolverConfig(**bounds)
