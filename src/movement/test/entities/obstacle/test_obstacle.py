import numpy as np
import pytest

from movement.entities.obstacle.obstacle import Obstacle
from utils.math_util import Vector2D


def test_obstacle_is_abstract():
    with pytest.raises(TypeError):
        Obstacle()


class _ConcreteObstacle(Obstacle):
    """Minimal concrete implementation used to exercise the default
    `batch_collides` fallback loop defined on the base class."""

    def __init__(self, center: Vector2D, radius: float):
        self.center = center
        self.radius = radius
        self.collision_calls = []

    def distanceTo(self, curPosition: Vector2D, t: float = 0.0) -> float:
        return self.center.distance(curPosition) - self.radius

    def isCollidingAt(self, curPosition: Vector2D, t: float = 0.0) -> bool:
        self.collision_calls.append((curPosition.x, curPosition.y, t))
        return self.distanceTo(curPosition, t) < 0

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        return tarPosition

    @property
    def velocity(self) -> float:
        return 0.0


class TestConcreteObstacleContract:
    def test_cannot_instantiate_missing_methods(self):
        class Incomplete(Obstacle):
            def distanceTo(self, curPosition):
                return 0.0

        with pytest.raises(TypeError):
            Incomplete()

    def test_concrete_subclass_can_be_instantiated(self):
        obs = _ConcreteObstacle(Vector2D(0, 0), 100)
        assert obs.velocity == 0.0

    def test_default_batch_collides_detects_collision(self):
        obs = _ConcreteObstacle(Vector2D(0, 0), 100)
        positions = np.array([[500.0, 500.0], [10.0, 0.0]])
        times = np.array([0.0, 0.0])

        assert obs.batch_collides(positions, times) is True

    def test_default_batch_collides_no_collision(self):
        obs = _ConcreteObstacle(Vector2D(0, 0), 100)
        positions = np.array([[500.0, 500.0], [1000.0, 1000.0]])
        times = np.array([0.0, 0.0])

        assert obs.batch_collides(positions, times) is False

    def test_default_batch_collides_short_circuits(self):
        obs = _ConcreteObstacle(Vector2D(0, 0), 100)
        # First position already collides, second one should never be checked.
        positions = np.array([[0.0, 0.0], [9999.0, 9999.0]])
        times = np.array([0.0, 0.0])

        assert obs.batch_collides(positions, times) is True
        assert len(obs.collision_calls) == 1

    def test_default_batch_collides_empty(self):
        obs = _ConcreteObstacle(Vector2D(0, 0), 100)
        positions = np.zeros((0, 2))
        times = np.zeros((0,))

        assert obs.batch_collides(positions, times) is False
