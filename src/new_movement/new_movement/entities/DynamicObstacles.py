from new_movement.entities.obstacles import Obstacle, ObstaclePriority
from new_movement.entities.States import Vector2D, State, MoveConstraints
from new_movement.entities.Motion import MotionPrimitive
from new_movement.utils.BB_steer import integrate_control_2d
from copy import deepcopy
from typing import Tuple


#TODO PRECISA MUITO TESTAR ESSA DESGRA;A
class EnemyRobotObstacle(Obstacle):
    ''' 
    Enemy Robots are modeled as a tube with thickness equal to the diameter of the robots, 
    in the same orientation of the robot velocity
    and length calculated based on the current velocity and time.
    '''
    def __init__(self, robotState: State, constraints: MoveConstraints, radius: float = 90):
        self.robotState = robotState
        self.radius = radius
        self.constraints = constraints

    def distanceTo(self, curPosition: Vector2D) -> float:
        return self.robotState.position.distance(curPosition)

    def isCollidingAt(self, curPosition: Vector2D, t: float) -> bool:
        back_point, front_point = self._constructTube(t)

        tube_vector = front_point.subtract(back_point)
        point_vector = back_point.subtract(curPosition)

        # Calculate projection parameter along the line segment
        t0 = point_vector.dot(tube_vector) / tube_vector.dot(tube_vector)
    
        # Clamp t0 to [0,1] to ensure we're on the segment
        t0 = max(0, min(1, t0))
        
        # Calculate closest point on the segment
        closest_point = back_point.add(tube_vector.multiplyByScalar(t0))
        
        return curPosition.distance(closest_point) < self.radius

    def adaptDestination(self, tarPosition: Vector2D, t: float) -> Vector2D:
        pass

    def updatePosition(self, robotState: State):
        self.robotState = robotState

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGH

    def _constructTube(self, t: float) -> Tuple[Vector2D, Vector2D]:
        '''
        Returns the back point and front point of the tube 
        The back point is the position in which the robot moves againts its own velocity
        The front point is the position in which the robot accelerates to the max in the velocity direction
        '''
        #TODO limit the t overhead to something like 0.5 seconds
        # This implementation does not consider the robots maximum velocity, but as
        # the time lookhead is short it should'nt metter too much.
        velocity_direction_norm: Vector2D = self.robotState.velocity.norm() # Velocity unit vector
        _velocity_direction_norm: Vector2D = deepcopy(velocity_direction_norm)

        if velocity_direction_norm.size() < 1e-2: # MAGIC NUMBER
            return self.robotState.position, self.robotState.position

        full_break = velocity_direction_norm.multiplyByScalar(self.constraints.min_acceleration.size())
        full_acceleration = _velocity_direction_norm.multiplyByScalar(self.constraints.max_acceleration.size())

        full_break_motion = MotionPrimitive(full_break, t)
        full_acceleration_motion = MotionPrimitive(full_acceleration, t)
        
        # AOS OLHOS DE ALGUNS ISSO PODE SER FEIO
        back_bbstate = integrate_control_2d(self.robotState, [full_break_motion])
        front_bbstate = integrate_control_2d(self.robotState, [full_acceleration_motion])
        # OU TALVEZ SEJA SO FEIO MESMO

        back_point = Vector2D(back_bbstate[0], back_bbstate[1]) # bbstate = [posx, posy, velx, vely]
        front_point = Vector2D(front_bbstate[0], front_bbstate[1])
        
        return back_point, front_point

class AllyRobotObstacle(Obstacle):
    def __init__(self):
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.MEDIUM

class MovingBallObstacle(Obstacle):
    def __init__(self):
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOW

