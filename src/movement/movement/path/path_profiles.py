from ruckig import InputParameter, Result, Ruckig, Trajectory, ControlInterface, Synchronization
from math import sin, cos
from random import uniform

from abc import ABC, abstractmethod

from typing import List, Optional, Tuple

class PathProfile(ABC):
    ''' Profiles are the used in the Path generator, it defines how a path will be generated '''
    @abstractmethod
    def generate(self):
        pass

class StraightProfile(PathProfile):
    ''' Return a straight line between two states ignoring the position plane and orientation'''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory, theta: float):
        inp.control_interface = ControlInterface.Velocity

        bottleneck_vel = inp.max_velocity[0] if inp.max_velocity[0] < inp.max_velocity[1] else inp.max_velocity[1]

        inp.target_velocity = [bottleneck_vel[0] * cos(theta), bottleneck_vel * sen(theta), 0]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path

class NormalProfile(PathProfile):
    ''' Return a bang-bang trajectory from inital state to final state given acceleration constrainsts '''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory):
        inp.control_interface = ControlInterface.Position
        
        inp.target_position = goal_state[0]
        inp.target_velocity = goal_state[1]
        
        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path

class GetInAngleProfile(PathProfile):
    ''' Return a bang-bang trajectory from inital state to final state arriving with a velocity in a angle'''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory, theta: float):
        inp.control_interface = ControlInterface.Position

        # Using 5% of the total velocity.
        bottleneck_vel = inp.max_velocity[0] if inp.max_velocity[0] < inp.max_velocity[1] else inp.max_velocity[1]
        arriving_velocity = bottleneck_vel * 0.05
        
        inp.target_position = goal_state[0]
        inp.target_velocity = [arriving_velocity * cos(theta) , arriving_velocity * sin(theta) , 0]
        
        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path

class BreakProfile(PathProfile):
    ''' Return a maximum breaking path, in this case, a acceleration in the oposite direction of the movement '''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory):
        # Goal state will not be used, sincronization will be turned off and each Dof will reach a stop independently.
        inp.control_interface = ControlInterface.Velocity
        inp.synchronization = Synchronization.No

        inp.target_velocity = [0.0, 0.0, 0.0]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path

class AimProfile(PathProfile):
    ''' Return a rotation movement profile to angle while being stationary'''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory, angle: float):
        inp.control_interface = ControlInterface.Position
        
        inp.target_position = [inp.current_position[0], inp.current_position[1], angle]
        inp.target_velocity = [0, 0, 0]
        
        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path

class SpinProfile(PathProfile):
    ''' Return a spin movement profile while being stationary'''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory, clockwise: bool = True):
        inp.control_interface = ControlInterface.Velocity
        inp.synchronization = Synchronization.No

        direction = 1 if clockwise else -1

        inp.target_velocity = [0, 0, inp.max_velocity[2] * direction]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path

class BypassProfile(PathProfile):
    ''' Should not be used in strategy, in case of collision, it's a alternative deviation path '''
    def generate(self, config: Ruckig, inp: InputParameter, goal_state: Optional[Tuple[List[float]]], path: Trajectory, t: float = 0.2):
        inp.control_interface = ControlInterface.Velocity

        x_random = uniform(-1, 1)
        y_random = uniform(-1, 1)

        inp.target_velocity = [inp.max_velocity[0] * x_random, inp.max_velocity[1] * y_random, 0]
        inp.target_acceleration = [0, 0, 0]

        inp.minimum_duration = t

        result = config.calculate(inp, path)

        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        
        return path


