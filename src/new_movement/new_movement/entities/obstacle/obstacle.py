from abc import ABC, abstractmethod
import numpy as np

from utils.math_util import Vector2D


class Obstacle(ABC):
    @abstractmethod
    def distanceTo(self, curPosition: Vector2D) -> float:
        """Calculate shortest euclidian distance between a position to the obstacle."""
        pass

    @abstractmethod
    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        """Check if a position collides with the obstacles and returns the if collision"""
        pass

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        """
        Default fallback: Python loop over the base isCollidingAt.
        Override in subclasses for vectorized performance.
        """
        for i, t in enumerate(times):
            if self.isCollidingAt(Vector2D(float(positions[i, 0]), float(positions[i, 1])), t):
                return True
        return False

    # How many sub-samples the generic fallback splits each segment into. Only reached
    # by obstacle types that have not implemented an exact swept test.
    _FALLBACK_SUBDIVISIONS = 8

    def batch_collides_segments(
        self,
        starts: np.ndarray,
        ends: np.ndarray,
        t_starts: np.ndarray,
        t_ends: np.ndarray,
    ) -> bool:
        """
        Swept test: does the straight move starts[i] -> ends[i], travelled over
        [t_starts[i], t_ends[i]], touch this obstacle at any point along the way?

        Sampling positions alone misses an encounter whenever the path clips the
        obstacle between two samples. Measured against a dense reference, that loses
        ~5% of grazing collisions at the step size the planner uses, and shrinking the
        step only trades cost for a slowly decreasing error. Testing the connecting
        segment removes the whole class of miss, and the remaining error is the gap
        between the chord and the real parabola (acceleration * dt^2 / 8, a couple of
        millimetres), which no longer depends on how fast the robot is moving.

        The default subdivides and reuses the point test, which narrows the gap without
        closing it. Subclasses should override with an exact sweep.
        """
        divisions = self._FALLBACK_SUBDIVISIONS
        delta_p = ends - starts
        delta_t = t_ends - t_starts
        for i in range(divisions + 1):
            fraction = i / divisions
            if self.batch_collides(starts + fraction * delta_p, t_starts + fraction * delta_t):
                return True
        return False

    def bounds(self) -> tuple | None:
        """
        Conservative axis-aligned box holding everything this obstacle can occupy over
        its whole horizon, as (min_x, min_y, max_x, max_y), or None when the occupied
        region cannot be bounded (the field border occupies the outside of a rectangle,
        which no finite box contains).

        Used only as a broad-phase reject: a path whose own box misses this one cannot
        touch the obstacle, so the swept test is skipped. Returning None costs nothing
        but the check. Being too generous is safe; being too tight is not.
        """
        return None

    @abstractmethod
    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        """
        If a destination position is in the obstacles,
        then return a alternative destination outside obstacle,
        """
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        """Returns the velocity of the obstacle"""
        pass
