from new_movement.entities.obstacles import Obstacle, ObstaclePriority
from new_movement.entities.States import Vector2D, State, MoveConstraints
from new_movement.entities.Trajectory import Trajectory
from new_movement.utils.BB_steer import integrate_control_2d
from typing import Tuple


class EnemyRobotObstacle(Obstacle):
    ''' 
    Enemy Robots are modeled as a tube with thickness equal to the diameter of the robots, 
    in the same orientation of the robot velocity
    and length calculated based on the current velocity and lookahead time.
    '''
    def __init__(self, robotState: State, radius: float = 90):
        self.robotState = robotState
        self.radius = radius

    def distanceTo(self, curPosition: Vector2D, t: float) -> float:
        start_point = self._getPos()
        end_point = self._getPredictedPos(t)

        return curPosition.distance(self._closestPointInLine(curPosition, start_point, end_point)) - self.radius

    def isCollidingAt(self, curPosition: Vector2D, t: float) -> bool:
        if self.distanceTo(curPosition, t) <= 0: # Negative distance, by convention, is inside obstacle.
            return True
        
        return False

    def adaptDestination(self, tarPosition: Vector2D, t: float) -> Vector2D:
        start_point = self._getPos()
        end_point = self._getPredictedPos(t)

        closest = self._closestPointInLine(tarPosition, start_point, end_point)
        dir_vector = tarPosition.subtract(closest)

        # tarPosition is inside the line
        if dir_vector.size() == 0:
            dir_vector = end_point.subtract(start_point)
            dir_vector = dir_vector.perpendicular().norm()

            dist = self.radius / 2 # arbitrary nonzero number inside tube
        else:
            dist = dir_vector.size()
            dir_vector = dir_vector.norm()

            # already outside obstacle
            if dist > self.radius:
                return tarPosition

        return closest.add(dir_vector.multiplyByScalar(self.radius))

    def updateState(self, robotState: State) -> None:
        self.robotState = robotState

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGH
    
    def _getPos(self) -> Vector2D:
        return self.robotState.position

    def _getPredictedPos(self, t: float) -> Vector2D:
        # curPos + curVel * t
        return self.robotState.position.add(self.robotState.velocity.multiplyByScalar(t))
    
    def _closestPointInLine(self, curPos: Vector2D, start_point: Vector2D, end_point: Vector2D) -> Vector2D:
        ''' Returns the closest point in a line segment to a position '''
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
    def __init__(self, robotState: State, trajectory: Trajectory, radius: float = 90):
        self.robotState = robotState
        self.trajectory = trajectory
        self.radius = radius

    def distanceTo(self, curPosition: Vector2D, t: float) -> float:
        return self.trajectory.get_position(t).distance(curPosition) - self.radius
    
    def isCollidingAt(self, curPosition: Vector2D, t: float) -> bool:
        if self.distanceTo(curPosition, t) <= 0:
            return True
        
        return False
    
    def adaptDestination(self, tarPosition: Vector2D, t: float) -> Vector2D:
        center_to_target = tarPosition.subtract(self.trajectory.get_position(t))

        dist = center_to_target.size()
        if dist == 0:
            center_to_target = Vector2D(1, 0) # arbitrary direction

        center_to_target = center_to_target.norm()

        return self.trajectory.get_position(t).add(center_to_target.multiplyByScalar(self.radius))
    
    def updateState(self, robotState: State) -> None:
        self.robotState = robotState

    def updateTrajectory(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.MEDIUM

class MovingBallObstacle(Obstacle):
    def __init__(self):
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOW

