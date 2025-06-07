from dataclasses import dataclass
from States import Vector2D

@dataclass
class MotionPrimitive:
    acceleration: Vector2D
    duration: float

@dataclass
class MotionPath:
    '''' Piecewise constant acceleration motion path '''
    motion_path: list[MotionPrimitive]