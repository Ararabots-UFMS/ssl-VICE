from new_movement.local_planner.solver import SolverConfig

from utils.math_util import Vector2D


class TestSolverConfig:
    def test_default_values(self):
        config = SolverConfig()

        assert config.max_iterations == 20
        assert config.field_length == 12000.0
        assert config.field_width == 9000.0
        assert config.max_velocity == Vector2D(2000.0, -2000.0)
        assert config.max_acceleration == Vector2D(1500.0, -1500.0)
        assert config.continuity_threshold == 1e-3
        assert config.collision_time_step == 0.2

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
