from new_movement.entities.obstacles import Obstacle
from new_movement.entities.States import Vector2D, State
from new_movement.entities.Trajectory import Trajectory
import numpy as np

# TODO maybe give preference in adapt destination to give a new destination
# close to the initial position instead of the closest of the target position...
class EnemyRobotObstacle(Obstacle):
    """
    Enemy Robots are modeled as a tube with thickness equal to the diameter of the robots,
    in the same orientation of the robot velocity
    and length calculated based on the current velocity and lookahead time.
    """

    def __init__(
        self, robotState: State, radius: float = 90, max_lookahead: float = 0.5
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

    def updateState(self, robotState: State) -> None:
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


class AllyRobotObstacle(Obstacle):
    """
    Allies Robots are modeled by their trajectory, in a tube shaped manner.
    """

    def __init__(
        self,
        robotState: State,
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

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        r2 = self.radius ** 2
        for i, t in enumerate(times):
            p = self.trajectory.get_position(self.time_offset + float(t))
            dx = positions[i, 0] - p.x
            dy = positions[i, 1] - p.y
            if dx * dx + dy * dy < r2:
                return True
        
        return False

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

    def updateState(self, robotState: State) -> None:
        self.robotState = robotState

    def updateTrajectory(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory

    def velocity(self) -> Vector2D:
        return self.robotState.velocity
