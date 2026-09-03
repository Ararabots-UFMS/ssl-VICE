import numpy as np
import pytest

from new_movement.entities.obstacle.static_obstacle import StaticObstacle
from utils.math_util import Vector2D


class _ConcreteStatic(StaticObstacle):
    """Minimal concrete StaticObstacle used to test the shared behavior."""

    def __init__(self, center: Vector2D, radius: float):
        self.center = center
        self.radius = radius
        self.check_positions_calls = []

    def distanceTo(self, curPosition: Vector2D) -> float:
        return self.center.distance(curPosition) - self.radius

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        return self.distanceTo(curPosition) < 0

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        return tarPosition

    def _check_positions(self, positions: np.ndarray) -> bool:
        self.check_positions_calls.append(positions)
        center = np.array([self.center.x, self.center.y])
        diffs = positions - center
        dists_sq = np.einsum("ij,ij->i", diffs, diffs)
        return bool(np.any(dists_sq < self.radius ** 2))


def test_velocity_is_always_zero():
    obs = _ConcreteStatic(Vector2D(0, 0), 100)
    assert obs.velocity() == 0.0


def test_batch_collides_delegates_to_check_positions_and_ignores_times():
    obs = _ConcreteStatic(Vector2D(0, 0), 100)
    positions = np.array([[500.0, 500.0]])
    # times should have zero influence on a static obstacle
    times_a = np.array([0.0])
    times_b = np.array([999.0])

    result_a = obs.batch_collides(positions, times_a)
    result_b = obs.batch_collides(positions, times_b)

    assert result_a == result_b is False
    assert len(obs.check_positions_calls) == 2


def test_batch_collides_detects_collision_via_check_positions():
    obs = _ConcreteStatic(Vector2D(0, 0), 100)
    positions = np.array([[10.0, 0.0], [5000.0, 5000.0]])
    times = np.array([0.0, 1.0])

    assert obs.batch_collides(positions, times) is True


def test_check_positions_is_abstract_on_static_obstacle_directly():
    # StaticObstacle itself is abstract (still has an unimplemented
    # abstractmethod), so it cannot be instantiated directly.
    with pytest.raises(TypeError):
        StaticObstacle()
