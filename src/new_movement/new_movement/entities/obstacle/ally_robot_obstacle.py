import numpy as np

from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.trajectory.trajectory import Trajectory

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

    def updateState(self, robotState: MotionState) -> None:
        self.robotState = robotState

    def updateTrajectory(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory

    def velocity(self) -> Vector2D:
        return self.robotState.velocity
