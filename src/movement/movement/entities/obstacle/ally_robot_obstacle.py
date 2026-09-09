import numpy as np

from movement.entities.obstacle.obstacle import Obstacle
from movement.entities.motion.motion_state import MotionState
from movement.entities.trajectory.trajectory import Trajectory
from movement.entities.trajectory.trajectory_sampler import TrajectorySampler

from utils.math_util import Vector2D

class AllyRobotObstacle(Obstacle):
    """
    Allies Robots are modeled by their trajectory, in a tube shaped manner.
    """

    def __init__(
        self,
        robotState: MotionState,
        trajectory: Trajectory,
        time_offset: float = 0.0,
        radius: float = 90,
    ):
        self.robotState = robotState
        self.trajectory = trajectory
        self.time_offset = time_offset
        self.radius = radius

    def distanceTo(self, curPosition: Vector2D, t: float) -> float:
        return (
            self.trajectory.get_position(self.time_offset + t).distance(curPosition)
            - self.radius
        )

    def isCollidingAt(self, curPosition: Vector2D, t: float) -> bool:
        if self.distanceTo(curPosition, t) < 0:
            return True

        return False

    def _ally_positions(self, times: np.ndarray) -> np.ndarray:
        """
        Where the ally is at each of ``times``, in one pass where possible.

        A real Trajectory is flattened once and evaluated as arrays; anything else that
        merely offers get_position still works one instant at a time.
        """
        offsets = self.time_offset + np.asarray(times, dtype=float)
        if offsets.size == 0:
            return np.empty((0, 2))

        root = getattr(self.trajectory, "root", None)
        if root is not None:
            return TrajectorySampler(root).positions(offsets)

        points = [self.trajectory.get_position(float(t)) for t in offsets]
        if any(p is None for p in points):
            return np.empty((0, 2))
        return np.array([[p.x, p.y] for p in points], dtype=float)

    def bounds(self) -> tuple | None:
        """The box the ally's own trajectory stays inside, grown by the radius."""
        root = getattr(self.trajectory, "root", None)
        if root is None:
            return None

        min_x, min_y, max_x, max_y = TrajectorySampler(root).position_bounds()
        r = self.radius
        return (min_x - r, min_y - r, max_x + r, max_y + r)

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        ally = self._ally_positions(times)
        if ally.size == 0:
            return False

        offset = positions - ally
        return bool(np.any(np.einsum("ij,ij->i", offset, offset) < self.radius ** 2))

    def batch_collides_segments(
        self,
        starts: np.ndarray,
        ends: np.ndarray,
        t_starts: np.ndarray,
        t_ends: np.ndarray,
    ) -> bool:
        """
        Swept test against a moving ally.

        Both bodies move, so they can be apart at both endpoints and still cross in
        between. Taking each as travelling in a straight line makes the separation
        linear in time, and the closest approach the minimum of a quadratic.
        """
        ally_start = self._ally_positions(t_starts)
        ally_end = self._ally_positions(t_ends)
        if ally_start.size == 0:
            return False

        initial = starts - ally_start                      # separation at t_start
        relative = (ends - starts) - (ally_end - ally_start)  # closing motion over the step

        speed_sq = np.einsum("ij,ij->i", relative, relative)
        approach = -np.einsum("ij,ij->i", initial, relative)
        # speed_sq == 0 means they hold station relative to each other; the separation is
        # constant, so the start of the interval already answers it.
        param = np.divide(
            approach, speed_sq, out=np.zeros_like(approach), where=speed_sq > 0
        )
        np.clip(param, 0.0, 1.0, out=param)

        closest = initial + param[:, np.newaxis] * relative
        return bool(np.any(np.einsum("ij,ij->i", closest, closest) < self.radius ** 2))

    def adaptDestination(self, tarPosition: Vector2D, t: float) -> Vector2D:
        if not self.isCollidingAt(tarPosition, t):
            return tarPosition

        center_to_target = tarPosition.subtract(self.trajectory.get_position(t))

        dist = center_to_target.size()
        if dist == 0:
            center_to_target = Vector2D(1, 0)  # arbitrary direction

        center_to_target = center_to_target.norm()

        return self.trajectory.get_position(t).add(
            center_to_target.multiplyByScalar(self.radius)
        )

    def updateState(self, robotState: MotionState) -> None:
        self.robotState = robotState

    def updateTrajectory(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory

    def velocity(self) -> Vector2D:
        return self.robotState.velocity
