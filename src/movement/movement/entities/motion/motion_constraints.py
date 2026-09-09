from dataclasses import dataclass
from typing import Optional

from utils.math_util import Vector2D


@dataclass
class MotionConstraints:
    max_velocity: Vector2D
    max_acceleration: Vector2D
    min_velocity: Optional[Vector2D] = None
    min_acceleration: Optional[Vector2D] = None

    def __post_init__(self):
        if self.min_velocity is None:
            self.min_velocity = -self.max_velocity
        if self.min_acceleration is None:
            self.min_acceleration = -self.max_acceleration
