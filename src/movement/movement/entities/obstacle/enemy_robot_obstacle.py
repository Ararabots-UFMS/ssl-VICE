import numpy as np

from movement.entities.obstacle.obstacle import Obstacle
from movement.entities.motion.motion_state import MotionState

from utils.math_util import Vector2D


# TODO maybe give preference in adapt destination to give a new destination
# close to the initial position instead of the closest of the target position...
class EnemyRobotObstacle(Obstacle):
    """
    Enemy Robots are modeled as a tube with thickness equal to the diameter of the robots,
    in the same orientation of the robot velocity
    and length calculated based on the current velocity and lookahead time.
    """

    def __init__(
        self, robotState: MotionState, radius: float = 90, max_lookahead: float = 0.5
    ):
        self.robotState = robotState
        self.radius = radius
        self.max_lookahead = max_lookahead

    def distanceTo(self, curPosition: Vector2D, t: float) -> float:
        if t > self.max_lookahead:
            t = self.max_lookahead

        start_point = self._getPos()
        end_point = self._getPredictedPos(t)

        return (
            curPosition.distance(
                self._closestPointInLine(curPosition, start_point, end_point)
            )
            - self.radius
        )

    def isCollidingAt(self, curPosition: Vector2D, t: float) -> bool:
        if (
            self.distanceTo(curPosition, t) <= 0
        ):  # Negative distance, by convention, is inside obstacle.
            return True

        return False

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        start = np.array([self.robotState.position.x, self.robotState.position.y])
        vel   = np.array([self.robotState.velocity.x, self.robotState.velocity.y])

        ts   = np.minimum(times, self.max_lookahead)
        ends = start + vel * ts[:, np.newaxis]          # (N, 2)

        AB = ends - start                               # (N, 2)
        AP = positions - start                          # (N, 2)

        ab2       = np.einsum("ij,ij->i", AB, AB)
        ap_dot_ab = np.einsum("ij,ij->i", AP, AB)

        t_param = np.where(ab2 > 0, ap_dot_ab / ab2, 0.0)
        t_param = np.clip(t_param, 0.0, 1.0)

        closest  = start + t_param[:, np.newaxis] * AB
        dists_sq = np.einsum("ij,ij->i", positions - closest, positions - closest)
        
        return bool(np.any(dists_sq < self.radius ** 2))

    def batch_collides_segments(
        self,
        starts: np.ndarray,
        ends: np.ndarray,
        t_starts: np.ndarray,
        t_ends: np.ndarray,
    ) -> bool:
        """
        Exact sweep of each path segment against the enemy tube.

        The tube only grows with time, so the one at the end of an interval contains
        every instant within it. What is left is the shortest distance between two
        segments — ours and the tube's spine — against the radius.
        """
        start = np.array([self.robotState.position.x, self.robotState.position.y])
        velocity = np.array([self.robotState.velocity.x, self.robotState.velocity.y])

        horizon = np.minimum(t_ends, self.max_lookahead)
        spine_ends = start + velocity * horizon[:, np.newaxis]

        distances_sq = self._segment_distance_sq(
            starts, ends, np.broadcast_to(start, spine_ends.shape), spine_ends
        )
        return bool(np.any(distances_sq < self.radius ** 2))

    def bounds(self) -> tuple:
        """The swept tube over the full lookahead, grown by the radius."""
        x0 = self.robotState.position.x
        y0 = self.robotState.position.y
        x1 = x0 + self.robotState.velocity.x * self.max_lookahead
        y1 = y0 + self.robotState.velocity.y * self.max_lookahead
        r = self.radius
        return (min(x0, x1) - r, min(y0, y1) - r, max(x0, x1) + r, max(y0, y1) + r)

    @staticmethod
    def _segment_distance_sq(p0, p1, q0, q1) -> np.ndarray:
        """Squared shortest distance between segments p0->p1 and q0->q1, elementwise."""
        d1 = p1 - p0
        d2 = q1 - q0
        r = p0 - q0

        a = np.einsum("ij,ij->i", d1, d1)
        e = np.einsum("ij,ij->i", d2, d2)
        f = np.einsum("ij,ij->i", d2, r)
        b = np.einsum("ij,ij->i", d1, d2)
        c = np.einsum("ij,ij->i", d1, r)

        denominator = a * e - b * b
        # Parallel or degenerate segments leave s free; anchoring it at 0 and letting the
        # clamp below place t is the standard resolution and stays exact.
        s = np.divide(
            b * f - c * e, denominator, out=np.zeros_like(a), where=denominator > 1e-12
        )
        np.clip(s, 0.0, 1.0, out=s)

        t = np.divide(b * s + f, e, out=np.zeros_like(a), where=e > 1e-12)
        np.clip(t, 0.0, 1.0, out=t)

        # Re-solve s for the clamped t so a clamped endpoint still gets its true closest
        # point rather than the unconstrained one.
        s = np.divide(b * t - c, a, out=np.zeros_like(a), where=a > 1e-12)
        np.clip(s, 0.0, 1.0, out=s)

        closest = (p0 + s[:, np.newaxis] * d1) - (q0 + t[:, np.newaxis] * d2)
        return np.einsum("ij,ij->i", closest, closest)

    def adaptDestination(self, tarPosition: Vector2D, t: float) -> Vector2D:
        start_point = self._getPos()
        end_point = self._getPredictedPos(t)

        closest = self._closestPointInLine(tarPosition, start_point, end_point)
        dir_vector = tarPosition.subtract(closest)

        # tarPosition is inside the line
        if dir_vector.size() == 0:
            dir_vector = end_point.subtract(start_point)
            dir_vector = dir_vector.perpendicular().norm()

            dist = self.radius / 2  # arbitrary nonzero number inside tube
        else:
            dist = dir_vector.size()
            dir_vector = dir_vector.norm()

            # already outside obstacle
            if dist > self.radius:
                return tarPosition

        return closest.add(dir_vector.multiplyByScalar(self.radius))

    def updateState(self, robotState: MotionState) -> None:
        self.robotState = robotState

    def velocity(self) -> Vector2D:
        return self.robotState.velocity

    def _getPos(self) -> Vector2D:
        return self.robotState.position

    def _getPredictedPos(self, t: float) -> Vector2D:
        # curPos + curVel * t
        return self._getPos().add(self.velocity().multiplyByScalar(t))

    def _closestPointInLine(
        self, curPos: Vector2D, start_point: Vector2D, end_point: Vector2D
    ) -> Vector2D:
        """Returns the closest point in a line segment to a position"""
        # curPos = P; start_point = A; end_point = B;
        AB = end_point.subtract(start_point)
        AP = curPos.subtract(start_point)
        ab2 = AB.dot(AB)

        # if A == B
        if ab2 == 0:
            return start_point

        t = AP.dot(AB) / ab2
        t = max(0, min(1, t))
        closest = start_point.add(AB.multiplyByScalar(t))

        return closest
